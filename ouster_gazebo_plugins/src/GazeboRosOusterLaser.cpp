/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <assert.h>
#include <ouster_gazebo_plugins/GazeboRosOusterLaser.h>

#include <algorithm>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/Param.hh>
#include <sdf/sdf.hh>
#if GAZEBO_GPU_RAY
#include <gazebo/sensors/GpuRaySensor.hh>
#else
#include <gazebo/sensors/RaySensor.hh>
#endif
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>

#if GAZEBO_GPU_RAY
#define RaySensor GpuRaySensor
#define STR_Gpu "Gpu"
#define STR_GPU_ "GPU "
#else
#define STR_Gpu ""
#define STR_GPU_ ""
#endif

namespace gazebo {
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosOusterLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosOusterLaser::GazeboRosOusterLaser()
: Node("gazebo_ros_ouster_laser"), min_range_(0), max_range_(0), gaussian_noise_(0) {}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosOusterLaser::~GazeboRosOusterLaser() {
  ////////////////////////////////////////////////////////////////////////////////
  // Finalize the controller / Custom Callback Queue
  rclcpp::shutdown();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosOusterLaser::Load(sensors::SensorPtr _parent,
                                sdf::ElementPtr _sdf) {
  // Load plugin
  RayPlugin::Load(_parent, _sdf);

  // Initialize Gazebo node
  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init();

  // Get the parent ray sensor
#if GAZEBO_MAJOR_VERSION >= 7
  parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
  parent_ray_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif
  if (!parent_ray_sensor_) {
    gzthrow("GazeboRosOuster" << STR_Gpu << "Laser controller requires a "
                              << STR_Gpu << "Ray Sensor as its parent.");
  }

  RCLCPP_INFO(get_logger(),"Ouster laser plugin : Getting the %sRay sensor parameters.",
           STR_Gpu);

  robot_namespace_ = "/";
  if (_sdf->HasElement("robotNamespace")) {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    RCLCPP_INFO(get_logger(),"Ouster laser plugin : Robot namespace set to %s",
             robot_namespace_.c_str());
  }

  if (!_sdf->HasElement("frameName")) {
    RCLCPP_INFO(get_logger(),"Ouster laser plugin missing <frameName>, defaults to /world");
    frame_name_ = "/world";
  } else {
    frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
    RCLCPP_INFO(get_logger(),"Ouster laser plugin : Frame name set to %s", frame_name_.c_str());
  }

  if (!_sdf->HasElement("min_range")) {
    RCLCPP_INFO(get_logger(),"Ouster laser plugin missing <min_range>, defaults to 0");
    min_range_ = 0;
  } else {
    min_range_ = _sdf->GetElement("min_range")->Get<double>();
    RCLCPP_INFO(get_logger(),"Ouster laser plugin : Min range set to %f", min_range_);
  }

  if (!_sdf->HasElement("max_range")) {
    RCLCPP_INFO(get_logger(),"Ouster laser plugin missing <max_range>, defaults to infinity");
    max_range_ = INFINITY;
  } else {
    max_range_ = _sdf->GetElement("max_range")->Get<double>();
    RCLCPP_INFO(get_logger(),"Ouster laser plugin : Max range set to %f", max_range_);
  }

  if (!_sdf->HasElement("topicName")) {
    RCLCPP_INFO(get_logger(),"Ouster laser plugin missing <topicName>, defaults to /points");
    topic_name_ = "/points";
  } else {
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
    RCLCPP_INFO(get_logger(),"Ouster laser plugin : Topic name set to %s", topic_name_.c_str());
  }

  if (!_sdf->HasElement("gaussianNoise")) {
    RCLCPP_INFO(get_logger(),
        "Ouster laser plugin missing <gaussianNoise>, defaults to 0.0, using "
        "noise generation based on datasheet");
    gaussian_noise_ = 0;
  } else {
    gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();
    if (gaussian_noise_ == 0.0)  // shouldn't it be compared to epsilon ?
      RCLCPP_INFO(get_logger(),
          "Ouster laser plugin : gaussian Noise set to 0, using noise "
          "generation based on datasheet");
    else if (gaussian_noise_ == -1.0)
      RCLCPP_INFO(get_logger(),"Ouster laser plugin : gaussian Noise unset");
    else
      RCLCPP_INFO(get_logger(),"Ouster laser plugin : gaussian Noise set to %f",
               gaussian_noise_);
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok()) {
    RCLCPP_FATAL_STREAM(get_logger(),
        "A ROS node for Gazebo has not been initialized, unable to load "
        "plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the "
           "gazebo_ros package)");
    return;
  }

  // Create node handle


  // Advertise publisher with a custom callback queue
  if (topic_name_ != "") {
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, 1);
  }

  // Sensor generation off by default
  parent_ray_sensor_->SetActive(false);


#if GAZEBO_MAJOR_VERSION >= 7
  RCLCPP_INFO(get_logger(),"Ouster %slaser plugin ready, %i lasers", STR_GPU_,
           parent_ray_sensor_->VerticalRangeCount());
#else
  RCLCPP_INFO(get_logger(),"Ouster %slaser plugin ready, %i lasers", STR_GPU_,
           parent_ray_sensor_->GetVerticalRangeCount());
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Subscribe on-demand
void GazeboRosOusterLaser::ConnectCb() {
  boost::lock_guard<boost::mutex> lock(lock_);
  if (pub_->get_subscription_count()) {
    if (!sub_) {
#if GAZEBO_MAJOR_VERSION >= 7
      sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                     &GazeboRosOusterLaser::OnScan, this);
#else
      sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(),
                                     &GazeboRosOusterLaser::OnScan, this);
#endif
    }
    parent_ray_sensor_->SetActive(true);
  } else {
#if GAZEBO_MAJOR_VERSION >= 7
    if (sub_) {
      sub_->Unsubscribe();
      sub_.reset();
    }
#endif
    parent_ray_sensor_->SetActive(false);
  }
}

void GazeboRosOusterLaser::OnScan(ConstLaserScanStampedPtr& _msg) {
#if GAZEBO_MAJOR_VERSION >= 7
  const ignition::math::Angle maxAngle = parent_ray_sensor_->AngleMax();
  const ignition::math::Angle minAngle = parent_ray_sensor_->AngleMin();

  const double maxRange = parent_ray_sensor_->RangeMax();
  const double minRange = parent_ray_sensor_->RangeMin();

  // const int rayCount = parent_ray_sensor_->RayCount();
  const int rangeCount = parent_ray_sensor_->RangeCount();

  const int verticalRayCount = parent_ray_sensor_->VerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();

  const ignition::math::Angle verticalMaxAngle =
      parent_ray_sensor_->VerticalAngleMax();
  const ignition::math::Angle verticalMinAngle =
      parent_ray_sensor_->VerticalAngleMin();
#else
  math::Angle maxAngle = parent_ray_sensor_->GetAngleMax();
  math::Angle minAngle = parent_ray_sensor_->GetAngleMin();

  const double maxRange = parent_ray_sensor_->GetRangeMax();
  const double minRange = parent_ray_sensor_->GetRangeMin();

  // const int rayCount = parent_ray_sensor_->GetRayCount();
  const int rangeCount = parent_ray_sensor_->GetRangeCount();

  const int verticalRayCount = parent_ray_sensor_->GetVerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();

  const math::Angle verticalMaxAngle =
      parent_ray_sensor_->GetVerticalAngleMax();
  const math::Angle verticalMinAngle =
      parent_ray_sensor_->GetVerticalAngleMin();
#endif

  const double yDiff = maxAngle.Radian() - minAngle.Radian();
  const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

  const double MIN_RANGE = std::max(min_range_, minRange);
  const double MAX_RANGE = std::min(max_range_, maxRange);

  // Populate message fields
  const uint32_t POINT_STEP = 32;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp = rclcpp::Time(_msg->time().sec(), _msg->time().nsec());
  msg.fields.resize(5);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 20;
  msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

  int i, j;
  uint8_t* ptr = msg.data.data();
  for (i = 0; i < rangeCount; i++) {
    for (j = 0; j < verticalRangeCount; j++) {
      // Range
      double r = _msg->scan().ranges(i + j * rangeCount);

      // Noise
      if (gaussian_noise_ > 0.0) {  // shouldn't it be compared to epsilon ?
        r += gaussianKernel(0, gaussian_noise_);
      } else if (gaussian_noise_ != -1.0) {
        // Noise set by the datasheet
        double g_noise = gaussianKernel(0, 1.);
        if (r <= 2.) {
          r += 0.03 * g_noise;
        } else if (r <= 20.) {
          r += 0.015 * g_noise;
        } else if (r <= 60.) {
          r += 0.03 * g_noise;
        } else {
          r += 0.1 * g_noise;
        }
      }

      // Intensity
      double intensity = _msg->scan().intensities(i + j * rangeCount);

      // Get angles of ray to get xyz for point
      double yAngle;
      double pAngle;

      if (rangeCount > 1) {
        yAngle = i * yDiff / (rangeCount - 1) + minAngle.Radian();
      } else {
        yAngle = minAngle.Radian();
      }

      if (verticalRayCount > 1) {
        pAngle =
            j * pDiff / (verticalRangeCount - 1) + verticalMinAngle.Radian();
      } else {
        pAngle = verticalMinAngle.Radian();
      }

      // pAngle is rotated by yAngle:
      if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
        *((float*)(ptr + 0)) = (float)(r * cos(pAngle) * cos(yAngle));  // x
        *((float*)(ptr + 4)) = (float)(r * cos(pAngle) * sin(yAngle));  // y
#if GAZEBO_MAJOR_VERSION > 2
        *((float*)(ptr + 8)) = (float)(r * sin(pAngle));  // z
#else
        *((float*)(ptr + 8)) = (float)(-r * sin(pAngle));  // z
#endif
        *((float*)(ptr + 16)) = (float)intensity;  // I
#if GAZEBO_MAJOR_VERSION > 2
        *((uint16_t*)(ptr + 20)) = (uint16_t)j;  // ring
#else
        *((uint16_t*)(ptr + 20)) =
            (uint16_t)verticalRangeCount - 1 - j;  // ring
#endif
        ptr += POINT_STEP;
      } else {  // if out of range, create a "NULL" point to keep the
                // organization
        *((float*)(ptr + 0)) = 0.;  // x
        *((float*)(ptr + 4)) = 0.;  // y
#if GAZEBO_MAJOR_VERSION > 2
        *((float*)(ptr + 8)) = 0.;  // z
#else
        *((float*)(ptr + 8)) = 0.;                 // z
#endif
        *((float*)(ptr + 16)) = 0.;  // I
#if GAZEBO_MAJOR_VERSION > 2
        *((uint16_t*)(ptr + 20)) = (uint16_t)j;  // ring
#else
        *((uint16_t*)(ptr + 20)) =
            (uint16_t)(verticalRangeCount - 1 - j);  // ring
#endif
        ptr += POINT_STEP;
      }
    }
  }

  // Populate message with number of valid points
  msg.point_step = POINT_STEP;
  msg.row_step = (uint32_t)(ptr - msg.data.data());
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.resize(msg.row_step);  // Shrink to actual size

  // Publish output
  pub_->publish(msg);
}


}  // namespace gazebo
