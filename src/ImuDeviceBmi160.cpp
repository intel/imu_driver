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

#include "ImuDeviceBmi160.h"

ImuDeviceBmi160::ImuDeviceBmi160() {}

ImuDeviceBmi160::~ImuDeviceBmi160() {}

int ImuDeviceBmi160::init() {
  log_debug("%s", __func__);
  setState(STATE_INIT);
  return 0;
}

int ImuDeviceBmi160::uninit() {
  log_debug("%s", __func__);
  setState(STATE_IDLE);
  return 0;
}

int ImuDeviceBmi160::start() {
  log_debug("%s", __func__);
  setState(STATE_RUN);
  return 0;
}

int ImuDeviceBmi160::stop() {
  log_debug("%s", __func__);
  setState(STATE_INIT);
  return 0;
}

int ImuDeviceBmi160::read(ImuData &value) {
  log_debug("%s", __func__);
  if (STATE_RUN != getState()) {
    log_error("read() called in wrong state");
    return -1;
  }

  return 0;
}

/*
    IDLE <--> INIT <--> RUN
*/
int ImuDeviceBmi160::setState(int state) {
  int ret = 0;
  log_debug("%s", __func__);

  if (mState == state)
    return 0;

  if (state == STATE_ERROR) {
    mState = state;
    return 0;
  }

  switch (mState) {
  case STATE_IDLE:
    if (state == STATE_INIT)
      mState = state;
    break;
  case STATE_INIT:
    if (state == STATE_IDLE || state == STATE_RUN)
      mState = state;
    break;
  case STATE_RUN:
    if (state == STATE_INIT)
      mState = state;
    break;
  case STATE_ERROR:
    log_error("In Error State");
    // Free up resources, restart?
    break;
  default:
    break;
  }

  if (mState != state) {
    ret = -1;
    log_error("InValid State Transition");
  }

  return ret;
}

int ImuDeviceBmi160::getState() {
  log_debug("%s", __func__);
  return mState;
}

int ImuDeviceBmi160::getCovariance(double &orient, double &angVel,
                                   double &linAccel) {
  log_debug("%s", __func__);

  return 0;
}
