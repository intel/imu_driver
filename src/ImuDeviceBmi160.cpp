/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#include <cassert>
#include <signal.h>
#include <unistd.h>

#include "ImuDeviceBmi160.h"
#include "log.h"

#define BMI160_READ_FLAG 0x80

/* For range 2G,
   = 2^16 / (2 * 2) - Multiply by 2 to account for the +2G and -2G
   = 65536 / 4
   = 16384
*/
#define G_TO_LSB (16384.0f)

/* For range 250dps,
   = 2^16 / (2 * 250) - Multiply by 2 to account for the +250dps and -250dps
   = 65536 / 500
   = 131.072
*/
#define DPS_TO_LSB (131.072f)

SPI *ImuDeviceBmi160::mSpi = nullptr;

ImuDeviceBmi160::ImuDeviceBmi160() : mState(STATE_IDLE) {}

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
  int ret = 0;
  log_debug("%s", __func__);
  signal(SIGINT, &ImuDeviceBmi160::intHandler);
  mSpi = new SPI("/dev/spidev3.0");
  mSpi->init(0, 1 * 1000 * 1000);

  /* You may assign a chip select identifier to be handled later */
  mSensor.id = 0;
  mSensor.interface = BMI160_SPI_INTF;
  mSensor.read = &ImuDeviceBmi160::readRegister;
  mSensor.write = &ImuDeviceBmi160::writeRegister;
  mSensor.delay_ms = &ImuDeviceBmi160::delayMs;

  int8_t retBmi = BMI160_OK;
  retBmi = bmi160_init(&mSensor);
  if (retBmi != BMI160_OK) {
    printf("sensor initialization failed\n");
    return -1;
  }

  /* Select the Output data rate, range of accelerometer sensor */
  mSensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
  mSensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
  mSensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

  /* Select the power mode of accelerometer sensor */
  mSensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

  /* Select the Output data rate, range of Gyroscope sensor */
  mSensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
  mSensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
  mSensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

  /* Select the power mode of Gyroscope sensor */
  mSensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

  /* Set the sensor configuration */
  retBmi = bmi160_set_sens_conf(&mSensor);
  if (retBmi != BMI160_OK) {
    printf("sensor configuration failed\n");
    return -1;
  }

  setState(STATE_RUN);
  return 0;
}

int ImuDeviceBmi160::stop() {
  log_debug("%s", __func__);
  delete mSpi;
  mSpi = nullptr;
  setState(STATE_INIT);
  return 0;
}

int ImuDeviceBmi160::read(ImuData &value) {
  int ret = 0;
  if (STATE_RUN != getState()) {
    log_error("read() called in wrong state");
    assert(0);
    return -1;
  }
  struct bmi160_sensor_data gyro;
  struct bmi160_sensor_data accel;

  int8_t retBmi = bmi160_get_sensor_data(
      (BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro,
      &mSensor);

  value.sec = accel.sensortime;

  value.accel[0] = (((double)accel.x) / G_TO_LSB) * 9.80655;
  value.accel[1] = (((double)accel.y) / G_TO_LSB) * 9.80655;
  value.accel[2] = (((double)accel.z) / G_TO_LSB) * 9.80655;

  value.angrate[0] = (((double)gyro.x) / DPS_TO_LSB) * 0.0174532925;
  value.angrate[1] = (((double)gyro.y) / DPS_TO_LSB) * 0.0174532925;
  value.angrate[2] = (((double)gyro.z) / DPS_TO_LSB) * 0.0174532925;

  /*
      value.accel[0] = static_cast<double>(accel.x);
      value.accel[1] = static_cast<double>(accel.y);
      value.accel[2] = static_cast<double>(accel.z);

      value.angrate[0] = static_cast<double>(gyro.x);
      value.angrate[1] = static_cast<double>(gyro.y);
      value.angrate[2] = static_cast<double>(gyro.z);
  */
  // value.orientation[] = ?

  printf("[%d] Accel data: %f %f %f\n", accel.sensortime, value.accel[0],
         value.accel[1], value.accel[2]);
  printf("[%d] Gyro data: %f %f %f\n", gyro.sensortime, value.angrate[0],
         value.angrate[1], value.angrate[2]);

  return ret;
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
    assert(0);
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
    assert(0);
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
  orient = 0;
  angVel = 0;
  linAccel = 0;

  return 0;
}

void ImuDeviceBmi160::intHandler(int dummy) {
  // setState(STATE_ERROR);
}

void ImuDeviceBmi160::delayMs(uint32_t period) { usleep(period * 1000); }

int8_t ImuDeviceBmi160::writeRegister(uint8_t dev_addr, uint8_t reg,
                                      uint8_t *data, uint16_t len) {
  int ret = 0;
  uint8_t buffer[2];

  // printf("%x: %d\n", reg, *data);

  buffer[0] = reg;
  buffer[1] = *data;
  if (mSpi->transfer(buffer, 2, NULL, 0) != 2)
    ret = -1;

  return ret;
}

int8_t ImuDeviceBmi160::readRegister(uint8_t dev_addr, uint8_t reg,
                                     uint8_t *recv_buffer, uint16_t recv_len) {
  int ret = 0;

  // printf("%x\n", reg & 0x7f);
  reg |= BMI160_READ_FLAG;
  if (mSpi->transfer(&reg, 1, recv_buffer, recv_len) != (recv_len + 1))
    ret = -1;
  return ret;
}
