/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#pragma once

#include <errno.h>
#include <stdint.h>

class ImuData {
public:
  ImuData() {}
  ~ImuData() {}

  uint32_t sec;
  uint32_t nsec;
  double accel[3];   // in m/s^2
  double angrate[3]; // in rad/sec
  double orientation[9];
};

class ImuDevice {
public:
  ImuDevice() {}
  virtual ~ImuDevice() {}

  enum State {
    STATE_ERROR = -1,
    STATE_IDLE = 0,
    STATE_INIT = 1,
    STATE_RUN = 2,
  };

  virtual int init() = 0;
  virtual int uninit() = 0;
  virtual int start() { return -ENOTSUP; }
  virtual int stop() { return -ENOTSUP; }
  virtual int read(ImuData &value) { return -ENOTSUP; }
  virtual int getState() { return -ENOTSUP; }
  virtual int getCovariance(double &orient, double &angVel, double &linAccel) {
    return -ENOTSUP;
  }
};
