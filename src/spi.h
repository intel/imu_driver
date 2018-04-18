/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

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
