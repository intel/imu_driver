/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2018 Intel Corporation
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
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
