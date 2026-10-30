/*
 * Host test: drive the profile/config/surface-attribute queries against a
 * fake decoder, with a watchdog, to catch lock recursion (the converter
 * probe used to take drv->mutex while vaQueryConfigProfiles held it, and
 * the board hung in a futex wait).
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "v4l2_request.h"

#define CHECK(cond) do { if (!(cond)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

int main(void)
{
	static struct VADriverContext va_ctx;
	struct v4l2r_driver *drv = calloc(1, sizeof(*drv));
	struct v4l2r_decoder *dec = &drv->decoders[0];
	VAProfile profiles[V4L2R_MAX_PROFILES];
	VAEntrypoint eps[V4L2R_MAX_ENTRYPOINTS];
	VASurfaceAttrib attribs[16];
	unsigned int nattr = 16;
	VAConfigID config;
	int n = 0, m = 0;
	bool main10 = false, main = false;

	alarm(10);			/* a deadlock fails the test instead of hanging it */
	setenv("V4L2R_NO_CONVERT", "1", 1);	/* don't touch the host's /dev/video* */

	CHECK(v4l2r_handles_init(&drv->configs, V4L2R_ID_OFFSET_CONFIG) == 0);
	CHECK(v4l2r_handles_init(&drv->contexts, V4L2R_ID_OFFSET_CONTEXT) == 0);
	CHECK(v4l2r_handles_init(&drv->surfaces, V4L2R_ID_OFFSET_SURFACE) == 0);
	CHECK(v4l2r_handles_init(&drv->buffers, V4L2R_ID_OFFSET_BUFFER) == 0);
	CHECK(v4l2r_handles_init(&drv->images, V4L2R_ID_OFFSET_IMAGE) == 0);
	pthread_mutex_init(&drv->mutex, NULL);
	pthread_mutex_init(&drv->converter_mutex, NULL);
	va_ctx.pDriverData = drv;

	/* rkvdec on RK3588: H.264/HEVC/VP9 up to 4K, 10-bit only as NV15 */
	strcpy(dec->video_path, "/nonexistent/video0");
	strcpy(dec->media_path, "/nonexistent/media0");
	dec->pixelformats[0] = V4L2_PIX_FMT_H264_SLICE;
	dec->pixelformats[1] = V4L2_PIX_FMT_HEVC_SLICE;
	dec->pixelformats[2] = V4L2_PIX_FMT_VP9_FRAME;
	for (int i = 0; i < 3; i++) {
		dec->max_width[i] = 4096;
		dec->max_height[i] = 2304;
		dec->max_area[i] = 4096ull * 2304;
	}
	dec->nb_pixelformats = 3;
	dec->hevc_10bit = true;
	dec->hevc_10bit_va_output = false;
	drv->nb_decoders = 1;

	CHECK(v4l2r_QueryConfigProfiles(&va_ctx, profiles, &n) == VA_STATUS_SUCCESS);
	for (int i = 0; i < n; i++) {
		main |= profiles[i] == VAProfileHEVCMain;
		main10 |= profiles[i] == VAProfileHEVCMain10;
		CHECK(profiles[i] != VAProfileVP9Profile2);
	}
	CHECK(main && !main10);		/* no converter, no P010: no 10-bit profiles */

	CHECK(v4l2r_QueryConfigEntrypoints(&va_ctx, VAProfileHEVCMain, eps, &m) == VA_STATUS_SUCCESS);
	CHECK(m >= 1 && eps[0] == VAEntrypointVLD);
	CHECK(v4l2r_QueryConfigEntrypoints(&va_ctx, VAProfileNone, eps, &m) == VA_STATUS_SUCCESS);
	CHECK(v4l2r_CreateConfig(&va_ctx, VAProfileHEVCMain, VAEntrypointVLD, NULL, 0, &config) == VA_STATUS_SUCCESS);
	CHECK(v4l2r_QuerySurfaceAttributes(&va_ctx, config, attribs, &nattr) == VA_STATUS_SUCCESS);
	for (unsigned int i = 0; i < nattr; i++) {
		if (attribs[i].type == VASurfaceAttribMaxWidth)
			CHECK(attribs[i].value.value.i == 4096);
		if (attribs[i].type == VASurfaceAttribMaxHeight)
			CHECK(attribs[i].value.value.i == 2304);
	}
	/* again, now that the converter probe has run */
	CHECK(v4l2r_QueryConfigProfiles(&va_ctx, profiles, &n) == VA_STATUS_SUCCESS);
	CHECK(v4l2r_CreateConfig(&va_ctx, VAProfileHEVCMain10, VAEntrypointVLD, NULL, 0, &config) != VA_STATUS_SUCCESS);

	printf("locking_test: %d profiles, no deadlock\n", n);
	return 0;
}
