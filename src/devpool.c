/*
 * Process-wide pool of open decoder device file descriptors.
 *
 * Sandboxed VA-API clients may initialize the driver before their sandbox
 * is engaged and then lose the right to open device nodes, while ioctl()
 * on already open descriptors stays allowed. Chromium's GPU process does
 * this: VaapiWrapper::PreSandboxInitialization() initializes VA-API, tears
 * the display down again, and the decoders are created after the seccomp
 * broker only lets it open DRM nodes. So the driver keeps a few descriptors
 * per decoder video node (each is an independent mem2mem context) and per
 * media node open for the whole process lifetime and hands them out instead
 * of calling open() for every VA context. A descriptor that comes back is
 * reset (both queues freed) before it is reused.
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include "devpool.h"

#define POOL_SIZE	32

struct pool_entry {
	char path[256];
	int fd;
	int flags;
	bool video;	/* a video node: reset its queues when it comes back */
	bool busy;
};

static struct pool_entry pool[POOL_SIZE];
static unsigned int pool_len;
static pthread_mutex_t pool_mutex = PTHREAD_MUTEX_INITIALIZER;

static bool pool_disabled(void)
{
	const char *env = getenv("V4L2R_FD_POOL");

	return env && !strcmp(env, "0");
}

int v4l2r_devpool_open(const char *path, int flags)
{
	int fd = -1;

	pthread_mutex_lock(&pool_mutex);
	for (unsigned int i = 0; i < pool_len; i++) {
		if (!pool[i].busy && pool[i].flags == flags &&
		    !strcmp(pool[i].path, path)) {
			pool[i].busy = true;
			fd = pool[i].fd;
			break;
		}
	}
	pthread_mutex_unlock(&pool_mutex);

	return fd >= 0 ? fd : open(path, flags);
}

static void reset_video_fd(int fd)
{
	static const enum v4l2_buf_type types[] = {
		V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		V4L2_BUF_TYPE_VIDEO_OUTPUT, V4L2_BUF_TYPE_VIDEO_CAPTURE,
	};

	for (unsigned int i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
		struct v4l2_requestbuffers reqbufs = {
			.type = types[i],
			.memory = V4L2_MEMORY_MMAP,
		};
		enum v4l2_buf_type type = types[i];

		/* Buffers still exported as dma-bufs are orphaned, not freed. */
		ioctl(fd, VIDIOC_STREAMOFF, &type);
		ioctl(fd, VIDIOC_REQBUFS, &reqbufs);
	}
}

void v4l2r_devpool_close(int fd)
{
	bool pooled = false;

	if (fd < 0)
		return;

	pthread_mutex_lock(&pool_mutex);
	for (unsigned int i = 0; i < pool_len; i++) {
		if (pool[i].fd == fd) {
			if (pool[i].video)
				reset_video_fd(fd);
			pool[i].busy = false;
			pooled = true;
			break;
		}
	}
	pthread_mutex_unlock(&pool_mutex);

	if (!pooled)
		close(fd);
}

void v4l2r_devpool_reserve(const char *path, int flags, bool video,
			   unsigned int count)
{
	unsigned int have = 0;

	if (pool_disabled())
		return;

	pthread_mutex_lock(&pool_mutex);
	for (unsigned int i = 0; i < pool_len; i++)
		if (pool[i].flags == flags && !strcmp(pool[i].path, path))
			have++;

	while (have < count && pool_len < POOL_SIZE &&
	       strlen(path) < sizeof(pool[0].path)) {
		int fd = open(path, flags | O_CLOEXEC);

		if (fd < 0)
			break;
		strcpy(pool[pool_len].path, path);
		pool[pool_len].fd = fd;
		pool[pool_len].flags = flags;
		pool[pool_len].video = video;
		pool[pool_len].busy = false;
		pool_len++;
		have++;
	}
	pthread_mutex_unlock(&pool_mutex);
}
