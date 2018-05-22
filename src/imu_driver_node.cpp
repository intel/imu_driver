/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

// ROS Node for IMU Sensor Devices
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include <ros/ros.h>

#include "ImuDeviceBmi160.h"

class ImuDriverNode {
public:
  ros::NodeHandle mNH;
  ros::Publisher mPubImuMsg;
  sensor_msgs::Imu mImuMsg;
  ImuDevice *mImuDev;
  ImuDriverNode(ros::NodeHandle h);
  ~ImuDriverNode();
  int start();
  int stop();
  bool spin();
  int pubData();
  int readData(sensor_msgs::Imu &data);
};

ImuDriverNode::ImuDriverNode(ros::NodeHandle h) : mNH(h) {
  ROS_INFO_STREAM("ROS Node imu_driver_node");

  // Create Node Publishers
  mPubImuMsg = mNH.advertise<sensor_msgs::Imu>("imu/data_raw", 5);

  // Create Node advertiseService

  mImuDev = new ImuDeviceBmi160();

  // Set Node Params

  // Fill IMU Message with constants
  mImuMsg.header.frame_id = "imu";

  double covOrient, covAngVel, covLinAccel;
  if (mImuDev->getCovariance(covOrient, covAngVel, covLinAccel)) {
    covOrient = 0;
    covAngVel = 0;
    covLinAccel = 0;
  }

  mImuMsg.linear_acceleration_covariance[0] = covLinAccel;
  mImuMsg.linear_acceleration_covariance[4] = covLinAccel;
  mImuMsg.linear_acceleration_covariance[8] = covLinAccel;

  mImuMsg.angular_velocity_covariance[0] = covAngVel;
  mImuMsg.angular_velocity_covariance[4] = covAngVel;
  mImuMsg.angular_velocity_covariance[8] = covAngVel;

  mImuMsg.orientation_covariance[0] = covOrient;
  mImuMsg.orientation_covariance[4] = covOrient;
  mImuMsg.orientation_covariance[8] = covOrient;
}

ImuDriverNode::~ImuDriverNode() {
  ROS_INFO_STREAM("~ImuDriverNode");

  stop();
}

int ImuDriverNode::start() {
  ROS_INFO_STREAM("start");

  mImuDev->init();
  mImuDev->start();
  return 0;
}

int ImuDriverNode::stop() {
  ROS_INFO_STREAM("stop");

  mImuDev->stop();
  mImuDev->uninit();
  return 0;
}

int ImuDriverNode::pubData() {
  ROS_INFO_STREAM("pubData");

  // read
  readData(mImuMsg);
  mPubImuMsg.publish(mImuMsg);
}

int ImuDriverNode::readData(sensor_msgs::Imu &data) {
  uint64_t time;
  double accel[3];
  double angrate[3];
  double orientation[9];
  ImuData value;

  // read
  mImuDev->read(value);

  data.linear_acceleration.x = value.accel[0];
  data.linear_acceleration.y = value.accel[1];
  data.linear_acceleration.z = value.accel[2];

  data.angular_velocity.x = value.angrate[0];
  data.angular_velocity.y = value.angrate[1];
  data.angular_velocity.z = value.angrate[2];

  data.orientation_covariance[0] = -1;

  /*
    tf::Quaternion quat;
    (tf::Matrix3x3(-1, 0, 0, 0, 1, 0, 0, 0, -1) *
     tf::Matrix3x3(
         value.orientation[0], value.orientation[3], value.orientation[6],
         value.orientation[1], value.orientation[4], value.orientation[7],
         value.orientation[2], value.orientation[5], value.orientation[8]))
        .getRotation(quat);

    tf::quaternionTFToMsg(quat, data.orientation);
  */

  data.header.stamp = ros::Time::now();
}

bool ImuDriverNode::spin() {
  while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid
                                 // restarting the node during a shutdown.
  {
    if (start() == 0) {
      while (mNH.ok()) {
        if (pubData() < 0)
          break;
        ros::spinOnce();
      }
    } else {
      // No need for diagnostic here since a broadcast occurs in start
      // when there is an error.
      usleep(1000000);
      ros::spinOnce();
    }
  }

  stop();

  return true;
}

int main(int argc, char **argv) {
  // Initialize the ROS system
  ros::init(argc, argv, "imu_driver");

  ros::NodeHandle nh;

  ImuDriverNode idn(nh);
  idn.spin();

  return (0);
}
