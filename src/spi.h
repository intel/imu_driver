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

#include <stdint.h>

class SPI {
public:
  SPI(const char *device);
  virtual ~SPI();
  int init(uint8_t mode, uint32_t freq);
  int transfer(const uint8_t *send_buffer, uint16_t send_len,
               uint8_t *recv_buffer, uint16_t recv_len);

private:
  char *_device;
  int _fd = -1;
  uint32_t _freq;
};
