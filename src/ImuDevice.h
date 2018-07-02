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

  uint32_t sec = 0;
  uint32_t nsec = 0;
  double accel[3];   // in m/s^2
  double angrate[3]; // in rad/sec
  double orientation[9];
};

class ImuDevice {
public:
  ImuDevice() {}
  virtual ~ImuDevice() {}

  enum class Status {
    SUCCESS,
    ERROR_UNKNOWN,
    INVALID_ARGUMENT,
    INVALID_STATE,
    NO_MEMORY,
    PERM_DENIED,
    TIMED_OUT,
    NOT_SUPPORTED
  };

  enum State {
    STATE_ERROR = -1,
    STATE_IDLE = 0,
    STATE_INIT = 1,
    STATE_RUN = 2,
  };

  virtual Status init() = 0;
  virtual Status uninit() = 0;
  virtual Status start() { return Status::NOT_SUPPORTED; }
  virtual Status stop() { return Status::NOT_SUPPORTED; }
  virtual Status read(ImuData &value) { return Status::NOT_SUPPORTED; }
  virtual State getState() { return State::STATE_ERROR; }
  virtual Status getCovariance(double &orient, double &angVel,
                               double &linAccel) {
    return Status::NOT_SUPPORTED;
  }
};
