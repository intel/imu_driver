/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#pragma once

#include <cstdio>

#define log_error(fmt, ...)                                                    \
  fprintf(stderr, "[Error] ");                                                 \
  fprintf(stderr, fmt, ##__VA_ARGS__);                                         \
  fprintf(stderr, "\n")
#define log_warn(fmt, ...)                                                     \
  fprintf(stderr, "[Warn] ");                                                  \
  fprintf(stderr, fmt, ##__VA_ARGS__);                                         \
  fprintf(stderr, "\n")
#ifdef DEBUG
#define log_debug(fmt, ...)                                                    \
  fprintf(stdout, "[Debug] ");                                                 \
  fprintf(stdout, fmt, ##__VA_ARGS__);                                         \
  fprintf(stdout, "\n")
#else
#define log_debug(fmt, ...)
#endif
