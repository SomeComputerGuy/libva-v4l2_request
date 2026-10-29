/*
 * Process-wide pool of open decoder device file descriptors, see devpool.c.
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef V4L2R_DEVPOOL_H
#define V4L2R_DEVPOOL_H

#include <stdbool.h>

/* Descriptors opened ahead per decoder video node (= concurrent VA contexts). */
#define V4L2R_DEVPOOL_PER_DECODER	2

/* An idle pooled descriptor for path opened with flags, or a fresh open(). */
int v4l2r_devpool_open(const char *path, int flags);
/* Return a descriptor from v4l2r_devpool_open(): pooled ones are reset and kept. */
void v4l2r_devpool_close(int fd);
/* Make sure count descriptors for path/flags are pooled (no-op with V4L2R_FD_POOL=0). */
void v4l2r_devpool_reserve(const char *path, int flags, bool video,
			   unsigned int count);

#endif /* V4L2R_DEVPOOL_H */
