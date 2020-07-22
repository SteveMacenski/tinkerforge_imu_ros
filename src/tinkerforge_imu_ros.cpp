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

void cb_quaternion(int16_t w, int16_t x, int16_t y, int16_t z, void * user_data)
{
  (void)user_data;

  sensor_msgs::Imu imuMsg;
  imuMsg.header.frame_id = frameId_;
  imuMsg.header.stamp = ros::Time::now();

  imuMsg.orientation.w = w/16383.0;
  imuMsg.orientation.x = x/16383.0;
  imuMsg.orientation.y = y/16383.0;
  imuMsg.orientation.z = z/16383.0;

  imuMsg.orientation_covariance = zeros_;
  imuMsg.orientation_covariance[0] = 1e-8;
  imuMsg.orientation_covariance[4] = 1e-8;
  imuMsg.orientation_covariance[8] = 1e-8;

  //publish the message
  imuPub_.publish(imuMsg);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "brick_imu_v2");

  int period;
  std::string uid;
  ros::NodeHandle nh("~");
  nh.param("frame_id", frameId_, std::string("imu"));
  nh.param("period_ms", period, 10);
  nh.param("uid", uid, std::string("6Det55"));

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

  // TODO grab all sensors
  imu_v2_register_callback(&imu,
                           IMU_V2_CALLBACK_QUATERNION,
                           (void *)cb_quaternion,
                           NULL);
  imu_v2_set_quaternion_period(&imu, period);

  ros::spin();
  ipcon_destroy(&ipcon);
  return 0;
}
