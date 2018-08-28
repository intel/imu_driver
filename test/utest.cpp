/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#include "../src/ImuDeviceBmi160.h"
// Bring in gtest
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(TestSuite, AccessImu) {
  ImuDevice *dev = new ImuDeviceBmi160();
  ASSERT_TRUE(dev != nullptr);

  ImuDevice::Status ret = ImuDevice::Status::SUCCESS;
  ret = dev->init();
  ASSERT_EQ(ret, ImuDevice::Status::SUCCESS);

  ret = dev->uninit();
  ASSERT_EQ(ret, ImuDevice::Status::SUCCESS);

  delete dev;
}

TEST(TestSuite, ReadImu) {
  ImuDevice *dev = new ImuDeviceBmi160();
  ASSERT_TRUE(dev != nullptr);

  ImuDevice::Status ret = ImuDevice::Status::SUCCESS;
  ret = dev->init();
  ASSERT_EQ(ret, ImuDevice::Status::SUCCESS);

  ret = dev->start();
  ASSERT_EQ(ret, ImuDevice::Status::SUCCESS);

  ImuData data;
  ret = dev->read(data);
  EXPECT_EQ(ret, ImuDevice::Status::SUCCESS);

  ret = dev->stop();
  ASSERT_EQ(ret, ImuDevice::Status::SUCCESS);

  ret = dev->uninit();
  ASSERT_EQ(ret, ImuDevice::Status::SUCCESS);

  delete dev;
}

TEST(TestSuite, GetImuState) {
  ImuDevice *dev = new ImuDeviceBmi160();
  ASSERT_TRUE(dev != nullptr);

  ImuDevice::Status ret = ImuDevice::Status::SUCCESS;
  ret = dev->init();
  ASSERT_EQ(ret, ImuDevice::Status::SUCCESS);

  ImuDevice::State state = dev->getState();
  EXPECT_EQ(state, ImuDevice::State::INIT);

  ret = dev->start();
  ASSERT_EQ(ret, ImuDevice::Status::SUCCESS);

  state = dev->getState();
  EXPECT_EQ(state, ImuDevice::State::RUN);

  ImuData data;
  ret = dev->read(data);
  EXPECT_EQ(ret, ImuDevice::Status::SUCCESS);

  state = dev->getState();
  EXPECT_EQ(state, ImuDevice::State::RUN);

  ret = dev->stop();
  ASSERT_EQ(ret, ImuDevice::Status::SUCCESS);

  state = dev->getState();
  EXPECT_EQ(state, ImuDevice::State::INIT);

  ret = dev->uninit();
  ASSERT_EQ(ret, ImuDevice::Status::SUCCESS);

  state = dev->getState();
  EXPECT_EQ(state, ImuDevice::State::IDLE);

  delete dev;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "utest");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
