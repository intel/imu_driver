/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

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
  static void delayMs(uint32_t period);
  static int8_t writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t *data,
                              uint16_t len);
  static int8_t readRegister(uint8_t dev_addr, uint8_t reg,
                             uint8_t *recv_buffer, uint16_t recv_len);
  std::atomic<int> mState;
  struct bmi160_dev mSensor;
  static SPI *mSpi;
};
