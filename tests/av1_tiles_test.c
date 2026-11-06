/*
 * Host test: AV1 tile entries come out the same whether a client sends a
 * picture's slice parameters before its slice data (FFmpeg) or the data
 * first and one parameter buffer per tile after it (Chromium), and the
 * first submission takes the CAPTURE bit depth from the sequence; and uniform
 * tile spacing gives the same tile geometry whether the client fills VA's
 * width/height_in_sbs_minus_1 (FFmpeg) or leaves them zero as VA allows
 * (Chromium).
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/codec_av1.c"

#if HAVE_V4L2_CTRL_AV1

#define CHECK(cond) do { if (!(cond)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); exit(1); } } while (0)

/* --- stubs for what codec_av1.c uses from the rest of the driver --- */

int v4l2r_array_reserve(void **array, unsigned int *alloc_elements,
			unsigned int needed, size_t element_size)
{
	void *p;

	if (needed <= *alloc_elements)
		return 0;
	p = realloc(*array, needed * element_size);
	if (!p)
		return -1;
	*array = p;
	*alloc_elements = needed;
	return 0;
}

VAStatus v4l2r_append_output(struct v4l2r_context *ctx, const void *data,
			     size_t size)
{
	struct v4l2r_output_buffer *output = ctx->pic.output;

	CHECK(output->bytesused + size <= output->size);
	memcpy(output->addr + output->bytesused, data, size);
	output->bytesused += size;
	return VA_STATUS_SUCCESS;
}

static struct v4l2_ctrl_av1_tile_group_entry submitted[16];
static unsigned int nb_submitted;
static unsigned int depth_at_submit;

VAStatus v4l2r_decode(struct v4l2r_context *ctx,
		      struct v4l2_ext_control *controls, unsigned int count,
		      bool first_slice, bool last_slice)
{
	(void)first_slice;
	(void)last_slice;
	for (unsigned int i = 0; i < count; i++) {
		if (controls[i].id != V4L2_CID_STATELESS_AV1_TILE_GROUP_ENTRY)
			continue;
		nb_submitted = controls[i].size / sizeof(submitted[0]);
		CHECK(nb_submitted <= 16);
		memcpy(submitted, controls[i].ptr, controls[i].size);
	}
	depth_at_submit = ctx->bit_depth;
	ctx->streaming = true;
	return VA_STATUS_SUCCESS;
}

uint64_t v4l2r_surface_timestamp(struct v4l2r_driver *drv, VASurfaceID id)
{
	(void)drv;
	(void)id;
	return 0;
}

int v4l2r_query_control(struct v4l2r_context *ctx,
			struct v4l2_query_ext_ctrl *control)
{
	(void)ctx;
	(void)control;
	return -1;
}

void v4l2r_flush_surface(struct v4l2r_surface *surface)
{
	(void)surface;
}

void *v4l2r_handles_lookup(struct v4l2r_handles *h, uint32_t id)
{
	(void)h;
	(void)id;
	return NULL;
}

/* As in driver.c: AV1 Main leaves the depth to the stream. */
unsigned int v4l2r_profile_bit_depth(VAProfile profile)
{
	return profile == VAProfileAV1Profile0 ? 0 : 8;
}

/* --- test --- */

#define NTILES 4

static VAStatus render(struct v4l2r_context *ctx, VABufferType type,
		       void *data, unsigned int size, unsigned int n)
{
	struct v4l2r_buffer buf = {
		.type = type, .element_size = size, .nb_elements = n, .data = data,
	};

	return v4l2r_codec_av1.render_buffer(ctx, &buf);
}

/* Decode one frame with a 3-byte header before 4 tiles of 10 bytes; order 0
 * = FFmpeg (params, data), 1 = Chromium (data, then one param per tile). */
static void run(int order, uint8_t bit_depth_idx)
{
	static uint8_t storage[4096];
	struct v4l2r_output_buffer output = { .addr = storage, .size = sizeof(storage) };
	struct v4l2r_surface surface = { .id = 1, .capture_index = 0 };
	struct v4l2r_context ctx = { 0 };
	VADecPictureParameterBufferAV1 pic = { 0 };
	VASliceParameterBufferAV1 slices[NTILES];
	uint8_t data[3 + NTILES * 10];

	ctx.drv = calloc(1, sizeof(*ctx.drv));	/* empty handle tables */
	ctx.codec = &v4l2r_codec_av1;
	ctx.codec_priv = calloc(1, v4l2r_codec_av1.priv_size);
	ctx.bit_depth = v4l2r_profile_bit_depth(VAProfileAV1Profile0);
	ctx.pic.output = &output;
	ctx.pic.target = &surface;
	nb_submitted = 0;

	memset(data, 0xa5, sizeof(data));
	pic.bit_depth_idx = bit_depth_idx;
	pic.current_frame = 1;
	pic.pic_info_fields.bits.frame_type = AV1_KEY_FRAME;
	for (unsigned int i = 0; i < NTILES; i++)
		slices[i] = (VASliceParameterBufferAV1) {
			.slice_data_size = 10,
			.slice_data_offset = 3 + 10 * i,
			.tile_row = (uint16_t)(i / 2),
			.tile_column = (uint16_t)(i % 2),
		};

	/* Something already staged in the OUTPUT buffer, to check offsets are
	 * relative to the data buffer's own position. */
	output.bytesused = 7;

	CHECK(v4l2r_codec_av1.init(&ctx) == VA_STATUS_SUCCESS);
	CHECK(v4l2r_codec_av1.begin_picture(&ctx) == VA_STATUS_SUCCESS);
	CHECK(render(&ctx, VAPictureParameterBufferType, &pic, sizeof(pic), 1) == VA_STATUS_SUCCESS);
	if (order == 0) {
		CHECK(render(&ctx, VASliceParameterBufferType, slices, sizeof(slices[0]), NTILES) == VA_STATUS_SUCCESS);
		CHECK(render(&ctx, VASliceDataBufferType, data, 1, sizeof(data)) == VA_STATUS_SUCCESS);
	} else {
		CHECK(render(&ctx, VASliceDataBufferType, data, 1, sizeof(data)) == VA_STATUS_SUCCESS);
		for (unsigned int i = 0; i < NTILES; i++)
			CHECK(render(&ctx, VASliceParameterBufferType, &slices[i], sizeof(slices[0]), 1) == VA_STATUS_SUCCESS);
	}
	CHECK(v4l2r_codec_av1.end_picture(&ctx) == VA_STATUS_SUCCESS);
	/* The frame is held until its surface is read. */
	CHECK(v4l2r_codec_av1.flush(&ctx, &surface) == VA_STATUS_SUCCESS);

	CHECK(nb_submitted == NTILES);
	for (unsigned int i = 0; i < NTILES; i++) {
		CHECK(submitted[i].tile_offset == 7 + 3 + 10 * i);
		CHECK(submitted[i].tile_size == 10);
		CHECK(submitted[i].tile_row == i / 2);
		CHECK(submitted[i].tile_col == i % 2);
	}
	CHECK(depth_at_submit == (bit_depth_idx ? 10u : 8u));

	/* A tile reaching past its data buffer is refused. */
	CHECK(v4l2r_codec_av1.begin_picture(&ctx) == VA_STATUS_SUCCESS);
	CHECK(render(&ctx, VAPictureParameterBufferType, &pic, sizeof(pic), 1) == VA_STATUS_SUCCESS);
	slices[0].slice_data_size = sizeof(data);
	CHECK(render(&ctx, VASliceDataBufferType, data, 1, sizeof(data)) == VA_STATUS_SUCCESS);
	CHECK(render(&ctx, VASliceParameterBufferType, &slices[0], sizeof(slices[0]), 1) == VA_STATUS_ERROR_INVALID_BUFFER);

	v4l2r_codec_av1.uninit(&ctx);
	free(ctx.codec_priv);
	free(ctx.drv);
}

/* Tile geometry of one frame of w x h with cols x rows uniform tiles. With
 * fill, width/height_in_sbs_minus_1 hold the spec's sizes (FFmpeg) and are
 * checked against the expected ones; without, they stay zero (Chromium). */
static void geometry(uint32_t w, uint32_t h, unsigned int cols, unsigned int rows,
		     bool uniform, const uint32_t *col_sbs, const uint32_t *row_sbs,
		     bool fill, struct v4l2_av1_tile_info *out)
{
	struct v4l2r_context ctx = { 0 };
	struct av1_context *codec = calloc(1, sizeof(*codec));
	VADecPictureParameterBufferAV1 *pic = &codec->va_pic;

	CHECK(codec);
	ctx.drv = calloc(1, sizeof(*ctx.drv));	/* empty handle tables */
	CHECK(ctx.drv);
	ctx.codec = &v4l2r_codec_av1;
	ctx.codec_priv = codec;
	pic->frame_width_minus1 = (uint16_t)(w - 1);
	pic->frame_height_minus1 = (uint16_t)(h - 1);
	pic->tile_cols = (uint8_t)cols;
	pic->tile_rows = (uint8_t)rows;
	pic->pic_info_fields.bits.uniform_tile_spacing_flag = uniform;
	if (fill) {
		for (unsigned int i = 0; i < cols; i++)
			pic->width_in_sbs_minus_1[i] = (uint16_t)(col_sbs[i] - 1);
		for (unsigned int i = 0; i < rows; i++)
			pic->height_in_sbs_minus_1[i] = (uint16_t)(row_sbs[i] - 1);
	}
	av1_fill_frame(&ctx);
	*out = codec->frame.tile_info;
	free(codec);
	free(ctx.drv);

	uint32_t mi = 0;
	for (unsigned int i = 0; i < cols; i++) {
		CHECK(out->width_in_sbs_minus_1[i] == col_sbs[i] - 1);
		CHECK(out->mi_col_starts[i] == mi);
		mi += col_sbs[i] << 4;
	}
	CHECK(out->mi_col_starts[cols] == 2 * ((w + 7) >> 3));
	mi = 0;
	for (unsigned int i = 0; i < rows; i++) {
		CHECK(out->height_in_sbs_minus_1[i] == row_sbs[i] - 1);
		CHECK(out->mi_row_starts[i] == mi);
		mi += row_sbs[i] << 4;
	}
	CHECK(out->mi_row_starts[rows] == 2 * ((h + 7) >> 3));
}

static void tile_geometry(void)
{
	static const struct {
		uint32_t w, h;
		unsigned int cols, rows;
		uint32_t col_sbs[4], row_sbs[2];
	} cases[] = {
		{ 1920, 1080, 1, 1, { 30 }, { 17 } },
		{ 3840, 2160, 1, 1, { 60 }, { 34 } },
		{ 3840, 2160, 4, 2, { 15, 15, 15, 15 }, { 17, 17 } },
		{ 1920, 1080, 4, 2, { 8, 8, 8, 6 }, { 9, 8 } },
		{ 1280, 720, 2, 1, { 10, 10 }, { 12 } },
	};
	struct v4l2_av1_tile_info chromium, ffmpeg;

	for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		geometry(cases[i].w, cases[i].h, cases[i].cols, cases[i].rows, true,
			 cases[i].col_sbs, cases[i].row_sbs, false, &chromium);
		geometry(cases[i].w, cases[i].h, cases[i].cols, cases[i].rows, true,
			 cases[i].col_sbs, cases[i].row_sbs, true, &ffmpeg);
		CHECK(!memcmp(&chromium, &ffmpeg, sizeof(chromium)));
	}

	/* Explicit spacing is taken as given. */
	{
		const uint32_t cols[2] = { 20, 10 }, rows[1] = { 17 };

		geometry(1920, 1080, 2, 1, false, cols, rows, true, &ffmpeg);
	}
}

int main(void)
{
	tile_geometry();
	run(0, 0);
	run(1, 0);
	run(1, 1);
	printf("av1_tiles: OK\n");
	return 0;
}

#else

int main(void)
{
	printf("av1_tiles: skipped (no AV1 controls in the kernel headers)\n");
	return 77;
}

#endif
