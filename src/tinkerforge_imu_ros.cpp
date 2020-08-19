// Copyright (c) 2020 Steve Macenski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string.h>
#include "../include/ip_connection.h"
#include "../include/brick_imu_v2.h"

#define HOST "localhost"
#define PORT 4223

ros::Publisher imuPub_;
std::string frameId_;
boost::array<const double, 9> zeros_ = { 0, 0, 0,
                                         0, 0, 0,
                                         0, 0, 0 };
double varianceOrientation;
double varianceAngularVelocity;
double varianceLinearAcceleration;


void cb_all_data(int16_t acceleration[3], int16_t magnetic_field[3], int16_t angular_velocity[3], int16_t euler_angle[3], int16_t quaternion[4], int16_t linear_acceleration[3], int16_t gravity_vector[3], int8_t temperature, uint8_t calibration_status, void *user_data) {
  (void)user_data;

  sensor_msgs::Imu imuMsg;
  imuMsg.header.frame_id = frameId_;
  imuMsg.header.stamp = ros::Time::now();

  imuMsg.orientation.x = quaternion[1]/16383.0;
  imuMsg.orientation.y = quaternion[2]/16383.0;
  imuMsg.orientation.z = quaternion[3]/16383.0;
  imuMsg.orientation.w = quaternion[0]/16383.0;
  imuMsg.orientation_covariance = {
    varianceOrientation, 0                  , 0,
    0                  , varianceOrientation, 0,
    0                  , 0                  , varianceOrientation
  };

  imuMsg.angular_velocity.x = angular_velocity[0]/16.0*M_PI/180;
  imuMsg.angular_velocity.y = angular_velocity[1]/16.0*M_PI/180;
  imuMsg.angular_velocity.z = angular_velocity[2]/16.0*M_PI/180;
  imuMsg.angular_velocity_covariance = {
    varianceAngularVelocity, 0                      , 0,
    0                      , varianceAngularVelocity, 0,
    0                      , 0                      , varianceAngularVelocity
  };

  imuMsg.linear_acceleration.x = acceleration[0]/100.0;
  imuMsg.linear_acceleration.y = acceleration[1]/100.0;
  imuMsg.linear_acceleration.z = acceleration[2]/100.0;
  imuMsg.linear_acceleration_covariance = {
    varianceLinearAcceleration, 0                         , 0,
    0                         , varianceLinearAcceleration, 0,
    0                         , 0                         , varianceLinearAcceleration
  };

  //publish the message
  imuPub_.publish(imuMsg);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "brick_imu_v2");

  int period, port;
  bool disableLeds;
  std::string host, uid;
  ros::NodeHandle nh("~");
  nh.param("frame_id", frameId_, std::string("imu"));
  nh.param("period_ms", period, 10);
  nh.param("host", host, std::string("localhost"));
  nh.param("port", port, 4223);
  nh.param("uid", uid, std::string("6Det55"));
  nh.param("disable_leds", disableLeds, false);
  nh.param("variance_orientation", varianceOrientation, 1e-8);
  // Observed angular velocity variance: 0.006223 (10k samples), => round up to 0.02
  nh.param("variance_angular_velocity", varianceAngularVelocity, 0.02);
  // Observed linear acceleration variance: 0.001532 (10k samples)
  // Calculation for variance taken from razor imu:
  // nonliniarity spec: 1% of full scale (+-2G) => 0.2m/s^2
  // Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
  nh.param("variance_linear_acceleration", varianceLinearAcceleration, 0.04);

  imuPub_ = nh.advertise<sensor_msgs::Imu>("imu", 10);

  IMUV2 imu;
  IPConnection ipcon;
  ipcon_create(&ipcon);
  imu_v2_create(&imu, uid.c_str(), &ipcon);
  if(ipcon_connect(&ipcon, HOST, PORT) < 0)
  {
    ROS_WARN("Bricks IMU V2: Could not connect!");
    return 1;
  }

  if (disableLeds) {
    imu_v2_leds_off(&imu);
  }
  imu_v2_register_callback(&imu, IMU_V2_CALLBACK_ALL_DATA, (void *)cb_all_data, NULL);
  imu_v2_set_all_data_period(&imu, period);

  ros::spin();

  imu_v2_destroy(&imu);
  ipcon_destroy(&ipcon);
  return 0;
}
