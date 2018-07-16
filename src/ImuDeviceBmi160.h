/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#pragma once

#include <atomic>
#include <string>

#include <bmi160.h>
#include <bmi160_defs.h>

#include "ImuDevice.h"
#include "spi.h"

class ImuDeviceBmi160 final : public ImuDevice {
public:
  ImuDeviceBmi160();
  ~ImuDeviceBmi160();
  Status init();
  Status uninit();
  Status start();
  Status stop();
  Status read(ImuData &value);
  Status getCovariance(double &orient, double &angVel, double &linAccel);
  State getState() const;

private:
  static const std::string SPI_DEVICE_NAME;
  Status setState(State state);
  static void delayMs(uint32_t period);
  static int8_t writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t *data,
                              uint16_t len);
  static int8_t readRegister(uint8_t dev_addr, uint8_t reg,
                             uint8_t *recv_buffer, uint16_t recv_len);
  std::atomic<State> mState;
  struct bmi160_dev mSensor;
  static SPI sSpi;
};
