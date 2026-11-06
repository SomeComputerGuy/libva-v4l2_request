/*
 * CPU conversion of the packed 10-bit NV15 layout (Rockchip decoders) to
 * P010 or NV12, used when no hardware converter can do it. See pixconv.c.
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef V4L2R_PIXCONV_H
#define V4L2R_PIXCONV_H

#include <stddef.h>
#include <stdint.h>

/* Bytes one row of n NV15 samples occupies (4 samples in 5 bytes). */
static inline size_t v4l2r_nv15_row_bytes(unsigned int samples)
{
	return ((size_t)samples * 10 + 7) / 8;
}

/*
 * One row of n samples: NV15 -> P010 (10 bits MSB-aligned in 16, low six
 * bits zero) and NV15 -> NV12 (the 8 most significant bits, truncated as
 * the Rockchip RGA does). Luma rows and interleaved CbCr rows pack the same
 * way, so both planes go through these. The source needs only the bytes
 * v4l2r_nv15_row_bytes(n) says; nothing past them is read.
 */
void v4l2r_nv15_row_to_p010(const uint8_t *src, uint16_t *dst, unsigned int n);
void v4l2r_nv15_row_to_nv12(const uint8_t *src, uint8_t *dst, unsigned int n);

/* Plain C versions of the above, the reference for the vector code. */
void v4l2r_nv15_row_to_p010_c(const uint8_t *src, uint16_t *dst,
			      unsigned int n);
void v4l2r_nv15_row_to_nv12_c(const uint8_t *src, uint8_t *dst,
			      unsigned int n);

/* Whether the vector (NEON) row converters are compiled in. */
int v4l2r_pixconv_have_simd(void);

/*
 * Whole frame, both planes. src: NV15 with luma at offset 0 and chroma at
 * src_chroma_offset, src_pitch bytes per row; dst: P010 or NV12 (dst_p010)
 * with its chroma at dst_chroma_offset. width and height are in luma
 * samples; height is rounded up to even for the chroma plane. Rows are
 * split over up to max_threads threads (1 = run on the caller's thread).
 * Returns 0, or -1 when a layout does not fit the given sizes.
 */
struct v4l2r_pixconv_frame {
	const uint8_t *src;
	size_t src_size;
	uint32_t src_pitch;
	size_t src_chroma_offset;
	uint8_t *dst;
	size_t dst_size;
	uint32_t dst_pitch;
	size_t dst_chroma_offset;
	unsigned int width;
	unsigned int height;
	int dst_p010;
};

int v4l2r_nv15_convert_frame(const struct v4l2r_pixconv_frame *frame,
			     unsigned int max_threads);

#endif
