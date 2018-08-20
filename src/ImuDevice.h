/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#pragma once

#include <errno.h>
#include <stdint.h>

/**
 *  The ImuData structure is used to hold the IMU data and meta-data
 */
struct ImuData {
  uint32_t sec = 0;  /**< system time in sec. */
  uint32_t nsec = 0; /**< system time in nano sec. */
  double accel[3];   /**< Linear acceleration in m/s^2. */
  double angrate[3]; /**< Angular velocity in rad/sec. */
  double orientation[9];
};

/**
 *  The ImuDevice class is an abstraction for different IMU Devices
 */
class ImuDevice {
public:
  /**
   *  Constructor. Creates the IMU Device.
   */
  ImuDevice() {}

  /**
   *  Destructor.
   */
  virtual ~ImuDevice() {}

  /**
   *  Possible return values from class methods.
   */
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

  /**
   *  Possible states of the object.
   */
  enum class State { ERROR, IDLE, INIT, RUN };

  /**
   *  The CoVariance structure
   */
  struct CoVariance {
    double orientation = 0;     /**< Orientation */
    double angularVelocity = 0; /**< Angular velocity */
    double acceleration = 0;    /**< Linear acceleration */
  };

  /**
   *  Initialize the IMU Device.
   *
   *  @return Status of request.
   */
  virtual Status init() = 0;

  /**
   *  Un-Initialize the IMU Device.
   *
   *  @return Status of request.
   */
  virtual Status uninit() = 0;

  /**
   *  Start the IMU Device.
   *
   *  @return Status of request.
   */
  virtual Status start() { return Status::NOT_SUPPORTED; }

  /**
   *  Stop the IMU Device.
   *
   *  @return Status of request.
   */
  virtual Status stop() { return Status::NOT_SUPPORTED; }

  /**
   *  Read IMU Data from the IMU device.
   *
   *  @param[out] value The ImuData object to hold the readings
   *
   *  @return Status of request.
   */
  virtual Status read(ImuData &value) { return Status::NOT_SUPPORTED; }

  /**
   *  Get status of the IMU device object.
   *
   *  @return Status of request.
   */
  virtual State getState() const { return State::ERROR; }

  /**
   *  Get covariances of the measurement from IMU device.
   *
   *  @param[out] orient Orientation covariance
   *  @param[out] angVel Angular velocity covariance
   *  @param[out] linAccel Linear acceleration covariance
   *
   *  @return Status of request.
   */
  virtual Status getCovariance(CoVariance &coVariance) const {
    return Status::NOT_SUPPORTED;
  }
};
