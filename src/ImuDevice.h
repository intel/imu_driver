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

#include <cstdio>
#include <errno.h>
#include <stdint.h>

#define log_error(fmt, ...)                                                    \
  printf("[Error] ");                                                          \
  printf(fmt, ##__VA_ARGS__);                                                  \
  printf("\n")
#define log_warn(fmt, ...)                                                     \
  printf("[Warn] ");                                                           \
  printf(fmt, ##__VA_ARGS__);                                                  \
  printf("\n")
#define log_debug(fmt, ...)                                                    \
  printf("[Debug] ");                                                          \
  printf(fmt, ##__VA_ARGS__);                                                  \
  printf("\n")

class ImuData {
public:
  ImuData() {}
  ~ImuData() {}

  uint64_t time;
  double accel[3];
  double angrate[3];
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
