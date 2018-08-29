/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#pragma once

#include <cstdio>

#define log_error(fmt, ...)                                                    \
  printf("[Error] ");                                                          \
  printf(fmt, ##__VA_ARGS__);                                                  \
  printf("\n")
#define log_warn(fmt, ...)                                                     \
  printf("[Warn] ");                                                           \
  printf(fmt, ##__VA_ARGS__);                                                  \
  printf("\n")
#ifdef DEBUG
#define log_debug(fmt, ...)                                                    \
  printf("[Debug] ");                                                          \
  printf(fmt, ##__VA_ARGS__);                                                  \
  printf("\n")
#else
#define log_debug(fmt, ...)
#endif
