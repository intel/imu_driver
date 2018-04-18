/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#include "spi.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "log.h"

#define BITS_PER_WORD 8

SPI::SPI(const char *device) { _device = strdup(device); }

SPI::~SPI() {
  free(_device);
  if (_fd != -1) {
    close(_fd);
  }
}

int SPI::init(uint8_t mode, uint32_t freq) {
  int ret = -1;
  uint8_t bits_per_word = BITS_PER_WORD;
  uint8_t lsb_first = 0;

  _fd = open(_device, O_RDWR);
  if (_fd == -1) {
    log_error("Unable to open device file descriptor: %s", _device);
    return -1;
  }

  ret = ioctl(_fd, SPI_IOC_WR_MODE, &mode);
  if (ret == -1) {
    log_error("Unable to set SPI mode");
    goto error;
  }

  ret = ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
  if (ret == -1) {
    log_error("Unable to set SPI bits per word");
    goto error;
  }

  ret = ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq);
  if (ret == -1) {
    log_error("Unable to set SPI max frequency");
    goto error;
  }

  ret = ioctl(_fd, SPI_IOC_WR_LSB_FIRST, &lsb_first);
  if (ret == -1) {
    log_error("Unable to set SPI bit order");
    goto error;
  }

  _freq = freq;

  return 0;

error:
  close(_fd);
  _fd = -1;
  return ret;
}

int SPI::transfer(const uint8_t *send_buffer, uint16_t send_len,
                  uint8_t *recv_buffer, uint16_t recv_len) {
  struct spi_ioc_transfer t[2] = {};
  uint8_t t_len = 0;

  if (send_buffer && send_len) {
    t[t_len].tx_buf = (uint64_t)send_buffer;
    t[t_len].len = send_len;
    t[t_len].rx_buf = 0;
    t[t_len].speed_hz = _freq;
    t[t_len].delay_usecs = 0;
    t[t_len].bits_per_word = BITS_PER_WORD;
    t[t_len].cs_change = 0;
    t[t_len].tx_nbits = 0;
    t[t_len].rx_nbits = 0;
    t_len++;
  }

  if (recv_buffer && recv_len) {
    t[t_len].rx_buf = (uint64_t)recv_buffer;
    t[t_len].len = recv_len;
    t[t_len].tx_buf = 0;
    t[t_len].speed_hz = _freq;
    t[t_len].delay_usecs = 0;
    t[t_len].bits_per_word = BITS_PER_WORD;
    t[t_len].cs_change = 0;
    t[t_len].tx_nbits = 0;
    t[t_len].rx_nbits = 0;
    t_len++;
  }

  if (!t_len) {
    return 0;
  }

  return ioctl(_fd, SPI_IOC_MESSAGE(t_len), &t);
}
