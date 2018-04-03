/*
 * This file is part of the Aero ROS Driver project
 *
 * Copyright (C) 2018  Intel Corporation. All rights reserved.
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
 */
#pragma once

#include <atomic>

#include <bmi160.h>
#include <bmi160_defs.h>

#include "ImuDevice.h"
#include "spi.h"

class ImuDeviceBmi160 final : public ImuDevice {
public:
  ImuDeviceBmi160();
  ~ImuDeviceBmi160();
  int init();
  int uninit();
  int start();
  int stop();
  int read(ImuData &value);
  int getCovariance(double &orient, double &angVel, double &linAccel);
  int getState();

private:
  int setState(int state);
  static void intHandler(int dummy);
  static void delayMs(uint32_t period);
  static int8_t writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t *data,
                              uint16_t len);
  static int8_t readRegister(uint8_t dev_addr, uint8_t reg,
                             uint8_t *recv_buffer, uint16_t recv_len);
  std::atomic<int> mState;
  struct bmi160_dev mSensor;
  static SPI *mSpi;
};
