/*
 * Host test: a pooled decoder descriptor is reset with the queue types the
 * device really has, and a descriptor whose reset fails is never handed out
 * again.
 *
 * A STREAMOFF with a buffer type the device does not have leaves a mem2mem
 * context aborted for good (v4l2_m2m_streamoff() cancels the job before the
 * type is checked), so the next VA context that got the descriptor queued
 * requests the kernel never ran. ioctl() is wrapped (-Wl,--wrap=ioctl) and
 * plays a mem2mem device; the descriptors are real ones of /dev/null.
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include "devpool.h"

#define CHECK(cond) do { if (!(cond)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); exit(1); } } while (0)

static uint32_t fake_caps;		/* device_caps QUERYCAP reports */
static int fail_streamoff;		/* make STREAMOFF fail with this errno */
static unsigned int bad_type_calls;	/* STREAMOFF/REQBUFS with a type the device lacks */
static unsigned int streamoffs, reqbufs;

static int device_has_type(uint32_t type)
{
	if (fake_caps & V4L2_CAP_VIDEO_M2M_MPLANE)
		return type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ||
		       type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	return type == V4L2_BUF_TYPE_VIDEO_OUTPUT ||
	       type == V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

int __real_ioctl(int fd, unsigned long request, ...);

int __wrap_ioctl(int fd, unsigned long request, ...)
{
	va_list ap;
	void *arg;

	(void)fd;
	va_start(ap, request);
	arg = va_arg(ap, void *);
	va_end(ap);

	switch (request) {
	case VIDIOC_QUERYCAP: {
		struct v4l2_capability *cap = arg;

		memset(cap, 0, sizeof(*cap));
		cap->capabilities = fake_caps | V4L2_CAP_DEVICE_CAPS;
		cap->device_caps = fake_caps;
		return 0;
	}
	case VIDIOC_STREAMOFF:
		streamoffs++;
		if (!device_has_type(*(uint32_t *)arg)) {
			bad_type_calls++;
			errno = EINVAL;
			return -1;
		}
		if (fail_streamoff) {
			errno = fail_streamoff;
			return -1;
		}
		return 0;
	case VIDIOC_REQBUFS:
		reqbufs++;
		if (!device_has_type(((struct v4l2_requestbuffers *)arg)->type)) {
			bad_type_calls++;
			errno = EINVAL;
			return -1;
		}
		return 0;
	default:
		errno = ENOTTY;
		return -1;
	}
}

static void reset_counters(void)
{
	bad_type_calls = streamoffs = reqbufs = 0;
	fail_streamoff = 0;
}

static void check_device(const char *path, uint32_t caps)
{
	int fd, again;

	fake_caps = caps;
	reset_counters();
	v4l2r_devpool_reserve(path, O_RDWR | O_NONBLOCK, true, 1);

	fd = v4l2r_devpool_open(path, O_RDWR | O_NONBLOCK);
	CHECK(fd >= 0);

	/* Handing it back resets exactly the device's two queues... */
	v4l2r_devpool_close(fd);
	CHECK(bad_type_calls == 0);
	CHECK(streamoffs == 2);
	CHECK(reqbufs == 2);

	/* ...and the next context gets the same (clean) descriptor. */
	again = v4l2r_devpool_open(path, O_RDWR | O_NONBLOCK);
	CHECK(again == fd);

	/* A reset that fails retires the descriptor. */
	fail_streamoff = EBUSY;
	v4l2r_devpool_close(again);
	CHECK(fcntl(fd, F_GETFD) < 0 && errno == EBADF);

	/* The next open is a plain open() of the node. */
	fail_streamoff = 0;
	again = v4l2r_devpool_open(path, O_RDWR | O_NONBLOCK);
	CHECK(again >= 0);
	v4l2r_devpool_close(again);	/* not pooled: just closed */
}

int main(void)
{
	/* /dev/null and /dev/zero stand in for two decoder video nodes. */
	check_device("/dev/null", V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING);
	check_device("/dev/zero", V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING);

	printf("devpool: OK\n");
	return 0;
}
