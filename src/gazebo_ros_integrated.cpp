/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo_ros_integrated.h"

#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>

using namespace cv;
using namespace std;

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(IntegratedPlugin)

/////////////////////////////////////////////////
IntegratedPlugin::IntegratedPlugin()
    : SensorPlugin(), width(0), height(0), depth(0)
{
}

/////////////////////////////////////////////////
IntegratedPlugin::~IntegratedPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();

  this->rosnode_->shutdown();

  delete this->rosnode_;
}

/////////////////////////////////////////////////
void IntegratedPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "IntegratedPlugin requires a CameraSensor.\n";
    if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
      gzmsg << "It is a depth camera sensor\n";
  }

  if (!this->parentSensor)
  {
    gzerr << "IntegratedPlugin not attached to a camera sensor\n";
    return;
  }

  this->world = physics::get_world(this->parentSensor->WorldName());

#if GAZEBO_MAJOR_VERSION >= 7
  this->camera = this->parentSensor->Camera();
  this->width = this->camera->ImageWidth();
  this->height = this->camera->ImageHeight();
  this->depth = this->camera->ImageDepth();
  this->format = this->camera->ImageFormat();
  hfov_ = float(this->camera->HFOV().Radian());
  first_frame_time_ = this->camera->LastRenderWallTime().Double();
  const string scopedName = _sensor->ParentName();
#else
  this->camera = this->parentSensor->GetCamera();
  this->width = this->camera->GetImageWidth();
  this->height = this->camera->GetImageHeight();
  this->depth = this->camera->GetImageDepth();
  this->format = this->camera->GetImageFormat();
  hfov_ = float(this->camera->GetHFOV().Radian());
  first_frame_time_ = this->camera->GetLastRenderWallTime().Double();
  const string scopedName = _sensor->GetParentName();
#endif

  focal_length_ = (this->width / 2) / tan(hfov_ / 2);

  if (this->width != 64 || this->height != 64)
  {
    gzerr << "[gazebo_optical_flow_plugin] Incorrect image size, must by 64 x 64.\n";
  }

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_optical_flow_plugin] Please specify a robotNamespace.\n";

  if (_sdf->HasElement("outputRate"))
  {
    output_rate_ = _sdf->GetElement("outputRate")->Get<int>();
  }
  else
  {
    output_rate_ = DEFAULT_RATE;
    gzwarn << "[gazebo_optical_flow_plugin] Using default output rate " << output_rate_ << ".";
  }

  if (_sdf->HasElement("hasGyro"))
    has_gyro_ = _sdf->GetElement("hasGyro")->Get<bool>();
  else
    has_gyro_ = HAS_GYRO;

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (has_gyro_)
  {
    if (_sdf->HasElement("hasGyro"))
      gyro_sub_topic_ = _sdf->GetElement("gyroTopic")->Get<std::string>();
    else
      gyro_sub_topic_ = kDefaultGyroTopic;

    string topicName = "~/" + _sensor->ParentName() + gyro_sub_topic_;
    boost::replace_all(topicName, "::", "/");
    imuSub_ = node_handle_->Subscribe(topicName, &IntegratedPlugin::ImuCallback, this);
  }

  if (_sdf->HasElement("altTopic"))
    alt_sub_topic_ = _sdf->GetElement("altTopic")->Get<std::string>();
  else
    alt_sub_topic_ = "/gazebo_alt";
  altSub_ = node_handle_->Subscribe(alt_sub_topic_, &IntegratedPlugin::AltCallback, this);

  magSub_ = node_handle_->Subscribe("/gazebo_mag", &IntegratedPlugin::MagCallback, this);

  string topicName = "~/" + scopedName + "/opticalFlow";
  boost::replace_all(topicName, "::", "/");

  opticalFlow_pub_ = node_handle_->Advertise<sensor_msgs::msgs::OpticalFlow>(topicName, 10);

  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      boost::bind(&IntegratedPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));

  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&IntegratedPlugin::OnUpdate, this, _1));

  if (!(_sdf->HasElement("topicName")))
  {
    ROS_WARN_NAMED("gazebo_ros_integrated", "topic_name_ : /gazebo_ros_integarted");
    topic_name_ = "/gazebo_ros_integrated";
  }
  else
    topic_name_ = _sdf->Get<std::string>("topicName");

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("imu", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  else
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
  }

  this->rosnode_ = new ros::NodeHandle(namespace_);

  this->pub_ = this->rosnode_->advertise<kari_estimator::kari_integrated>(
      this->topic_name_, 1);

  this->parentSensor->SetActive(true);

  // init flow
  optical_flow_ = new OpticalFlowOpenCV(focal_length_, focal_length_, output_rate_);
  // _optical_flow = new OpticalFlowPX4(focal_length_, focal_length_, output_rate_, this->width);
}

void IntegratedPlugin::OnNewFrame(const unsigned char *_image,
                                  unsigned int _width,
                                  unsigned int _height,
                                  unsigned int _depth,
                                  const std::string &_format)
{

// get data depending on gazebo version
#if GAZEBO_MAJOR_VERSION >= 7
  _image = this->camera->ImageData(0);
  double frame_time = this->camera->LastRenderWallTime().Double();
#else
  _image = this->camera->GetImageData(0);
  double frame_time = this->camera->GetLastRenderWallTime().Double();
#endif

  frame_time_us_ = (frame_time - first_frame_time_) * 1e6; // since start

  float flow_x_ang = 0.0f;
  float flow_y_ang = 0.0f;
  // calculate angular flow
  int quality = optical_flow_->calcFlow((uchar *)_image, frame_time_us_, dt_us_, flow_x_ang, flow_y_ang);

  if (quality >= 0)
  { // calcFlow(...) returns -1 if data should not be published yet -> output_rate
// prepare optical flow message
//  Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world->SimTime();
#else
    common::Time now = world->GetSimTime();
#endif

    opticalFlow_message.set_time_usec(now.Double() * 1e6);
    opticalFlow_message.set_sensor_id(2.0);
    opticalFlow_message.set_integration_time_us(quality ? dt_us_ : 0);
    opticalFlow_message.set_integrated_x(quality ? flow_x_ang : 0.0f);
    opticalFlow_message.set_integrated_y(quality ? flow_y_ang : 0.0f);

    int_msg_temp.delt_sec = dt_us_ / 1000000.0;
    // int_msg_.t_sec = now.Double();
    // prev_time = int_msg_.t_sec;
    int_msg_temp.delx_rps = flow_x_ang;
    int_msg_temp.dely_rps = flow_y_ang;
    int_msg_temp.qual = quality;

    // int_msg_.opt.time_usec = now.Double() * 1e6;
    // int_msg_.opt.sensor_id = 2.0;
    // int_msg_.opt.integration_time_us = quality ? dt_us_ : 0;
    // int_msg_.opt.integrated_x = quality ? flow_x_ang : 0.0f;
    // int_msg_.opt.integrated_y = quality ? flow_y_ang : 0.0f;

    if (has_gyro_)
    {
      opticalFlow_message.set_integrated_xgyro(opticalFlow_rate.X());
      opticalFlow_message.set_integrated_ygyro(opticalFlow_rate.Y());
      opticalFlow_message.set_integrated_zgyro(opticalFlow_rate.Z());

      // int_msg_.opt.integrated_xgyro = opticalFlow_rate.X();
      // int_msg_.opt.integrated_ygyro = opticalFlow_rate.Y();
      // int_msg_.opt.integrated_zgyro = opticalFlow_rate.Z();

      // reset gyro integral
      opticalFlow_rate.Set();
    }
    else
    {
      // no gyro
      opticalFlow_message.set_integrated_xgyro(NAN);
      opticalFlow_message.set_integrated_ygyro(NAN);
      opticalFlow_message.set_integrated_zgyro(NAN);

      // int_msg_.opt.integrated_xgyro = NAN;
      // int_msg_.opt.integrated_ygyro = NAN;
      // int_msg_.opt.integrated_zgyro = NAN;
    }
    opticalFlow_message.set_temperature(20.0f);
    opticalFlow_message.set_quality(quality);
    opticalFlow_message.set_time_delta_distance_us(0);
    opticalFlow_message.set_distance(0.0f); // get real values in gazebo_mavlink_interface.cpp
    // send message
    opticalFlow_pub_->Publish(opticalFlow_message);

    // int_msg_.opt.quality = quality;
    // int_msg_.opt.time_delta_distance_us = 0;
    // int_msg_.opt.distance = 0.0f;
    // this->pub_.publish(this->int_msg_);
    isUpdated = true;
  }
}

void IntegratedPlugin::ImuCallback(ConstIMUPtr &_imu)
{
// accumulate gyro measurements that are needed for the optical flow message
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world->SimTime();
#else
  common::Time now = world->GetSimTime();
#endif

  uint32_t now_us = now.Double() * 1e6;
  ignition::math::Vector3d px4flow_gyro = ignition::math::Vector3d(_imu->angular_velocity().x(),
                                                                   _imu->angular_velocity().y(),
                                                                   _imu->angular_velocity().z());

  static uint32_t last_dt_us = now_us;
  uint32_t dt_us = now_us - last_dt_us;

  if (dt_us > 1000)
  {
    opticalFlow_rate += px4flow_gyro * (dt_us / 1000000.0f);
    last_dt_us = now_us;
  }

  // int_msg_.imu.orientation.x = _imu->orientation().x();
  // int_msg_.imu.orientation.y = _imu->orientation().y();
  // int_msg_.imu.orientation.z = _imu->orientation().z();
  // int_msg_.imu.orientation.w = _imu->orientation().w();
  // int_msg_.imu.angular_velocity.x = _imu->angular_velocity().x();
  // int_msg_.imu.angular_velocity.y = _imu->angular_velocity().y();
  // int_msg_.imu.angular_velocity.z = _imu->angular_velocity().z();
  // int_msg_.imu.linear_acceleration.x = _imu->linear_acceleration().x();
  // int_msg_.imu.linear_acceleration.y = _imu->linear_acceleration().y();
  // int_msg_.imu.linear_acceleration.z = _imu->linear_acceleration().z();

  int_msg_.gx_rps = _imu->angular_velocity().x();
  int_msg_.gy_rps = _imu->angular_velocity().y();
  int_msg_.gz_rps = _imu->angular_velocity().z();
  int_msg_.ax_mps2 = _imu->linear_acceleration().x();
  int_msg_.ay_mps2 = _imu->linear_acceleration().y();
  int_msg_.az_mps2 = _imu->linear_acceleration().z();
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */

void IntegratedPlugin::AltCallback(const boost::shared_ptr<const sensor_msgs::msgs::Range> &_alt)
{
  // alt_msg.set_current_distance(_alt->current_distance());
  // int_msg_.alt.range = _alt->current_distance();
  int_msg_.h_mtr = _alt->current_distance();
}

void IntegratedPlugin::MagCallback(const boost::shared_ptr<const sensor_msgs::msgs::MagneticField> &_mag)
{
  int_msg_.mx_gauss = _mag->magnetic_field().x();
  int_msg_.my_gauss = _mag->magnetic_field().y();
  int_msg_.mz_gauss = _mag->magnetic_field().z();
}

void IntegratedPlugin::OnUpdate(const common::UpdateInfo &_info)
{

#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = this->world->SimTime();
#else
  common::Time current_time = this->world->GetSimTime();
#endif
  //   double dt = (current_time - last_time_).Double();
  //   last_time_ = current_time;
  //   double t = current_time.Double();
  if ((current_time - last_time).Double() < (1.0 / 100))
  {
    return;
  }

  if (isUpdated == true)
  {
    int_msg_.delt_sec = int_msg_temp.delt_sec;
    int_msg_.delx_rps = int_msg_temp.delx_rps;
    int_msg_.dely_rps = int_msg_temp.dely_rps;
    int_msg_.qual = int_msg_temp.qual;
    isUpdated = false;
  }
  else
  {
    int_msg_.delt_sec = 0.0;
    int_msg_.delx_rps = 0.0;
    int_msg_.dely_rps = 0.0;
    int_msg_.qual = 0.0;
  }

  int_msg_.t_sec = current_time.Double();

  this->pub_.publish(this->int_msg_);

  last_time = current_time;
  return;
}
