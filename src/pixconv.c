/*
 * CPU fallback for the format converter chain: NV15 (4 samples of 10 bits
 * in 5 little-endian bytes, the layout Rockchip decoders write) to P010
 * or NV12. Used when no mem2mem converter reads NV15 (e.g. the mainline
 * RGA2), so 10-bit streams still decode in hardware and keep their depth.
 *
 * Sample k (0..3) of a 5-byte group starting at byte b is the 16-bit
 * little-endian word at b + k, shifted right by 2k, masked to 10 bits.
 * The NEON path gathers those words with one table lookup per 8 samples
 * and shifts each lane by its own amount; P010 wants the sample << 6,
 * which is the word shifted left by 6 - 2k and masked with 0xffc0.
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <pthread.h>
#include <string.h>

#include "pixconv.h"

#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#define HAVE_NEON 1
#else
#define HAVE_NEON 0
#endif

static inline unsigned int nv15_sample(const uint8_t *group, unsigned int k)
{
	unsigned int word = group[k] | (unsigned int)group[k + 1] << 8;

	return (word >> (2 * k)) & 0x3ff;
}

void v4l2r_nv15_row_to_p010_c(const uint8_t *src, uint16_t *dst,
			      unsigned int n)
{
	unsigned int i = 0;

	for (; i + 4 <= n; i += 4, src += 5) {
		dst[i + 0] = (uint16_t)(nv15_sample(src, 0) << 6);
		dst[i + 1] = (uint16_t)(nv15_sample(src, 1) << 6);
		dst[i + 2] = (uint16_t)(nv15_sample(src, 2) << 6);
		dst[i + 3] = (uint16_t)(nv15_sample(src, 3) << 6);
	}

	/* A partial last group occupies only ceil(10 * left / 8) bytes;
	 * never touch the byte after it. */
	for (unsigned int k = 0; i < n; i++, k++) {
		unsigned int bit = 10 * k;
		/* bit % 8 is at most 6, so the sample always ends in the
		 * next byte, which is still part of the row. */
		unsigned int word = src[bit / 8] |
				    (unsigned int)src[bit / 8 + 1] << 8;
		dst[i] = (uint16_t)(((word >> (bit % 8)) & 0x3ff) << 6);
	}
}

void v4l2r_nv15_row_to_nv12_c(const uint8_t *src, uint8_t *dst,
			      unsigned int n)
{
	unsigned int i = 0;

	for (; i + 4 <= n; i += 4, src += 5) {
		dst[i + 0] = (uint8_t)(nv15_sample(src, 0) >> 2);
		dst[i + 1] = (uint8_t)(nv15_sample(src, 1) >> 2);
		dst[i + 2] = (uint8_t)(nv15_sample(src, 2) >> 2);
		dst[i + 3] = (uint8_t)(nv15_sample(src, 3) >> 2);
	}

	for (unsigned int k = 0; i < n; i++, k++) {
		unsigned int bit = 10 * k;
		/* bit % 8 is at most 6, so the sample always ends in the
		 * next byte, which is still part of the row. */
		unsigned int word = src[bit / 8] |
				    (unsigned int)src[bit / 8 + 1] << 8;
		dst[i] = (uint8_t)(((word >> (bit % 8)) & 0x3ff) >> 2);
	}
}

#if HAVE_NEON
/* Byte pairs (little-endian words) holding samples 0..3 of the groups at
 * bytes 0 and 5 of a 16-byte load. */
static const uint8_t nv15_gather[16] = {
	0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9,
};
static const int16_t p010_shift[8] = { 6, 4, 2, 0, 6, 4, 2, 0 };
static const int16_t nv12_shift[8] = { -2, -4, -6, -8, -2, -4, -6, -8 };

/* 8 samples from the 10 bytes at src (16 bytes are loaded). */
static inline uint16x8_t nv15_words(const uint8_t *src, uint8x16_t gather)
{
	return vreinterpretq_u16_u8(vqtbl1q_u8(vld1q_u8(src), gather));
}

void v4l2r_nv15_row_to_p010(const uint8_t *src, uint16_t *dst, unsigned int n)
{
	const uint8x16_t gather = vld1q_u8(nv15_gather);
	const int16x8_t shift = vld1q_s16(p010_shift);
	const uint16x8_t mask = vdupq_n_u16(0xffc0);
	size_t avail = v4l2r_nv15_row_bytes(n);
	unsigned int i = 0;

	/* 32 samples (40 bytes) per step; the last load reads 16 bytes at
	 * offset 30, so 46 bytes must be there. */
	for (; i + 32 <= n && avail >= 46; i += 32, src += 40, avail -= 40) {
		uint16x8_t a = nv15_words(src, gather);
		uint16x8_t b = nv15_words(src + 10, gather);
		uint16x8_t c = nv15_words(src + 20, gather);
		uint16x8_t d = nv15_words(src + 30, gather);

		vst1q_u16(dst + i, vandq_u16(vshlq_u16(a, shift), mask));
		vst1q_u16(dst + i + 8, vandq_u16(vshlq_u16(b, shift), mask));
		vst1q_u16(dst + i + 16, vandq_u16(vshlq_u16(c, shift), mask));
		vst1q_u16(dst + i + 24, vandq_u16(vshlq_u16(d, shift), mask));
	}

	for (; i + 8 <= n && avail >= 16; i += 8, src += 10, avail -= 10)
		vst1q_u16(dst + i, vandq_u16(vshlq_u16(nv15_words(src, gather),
						       shift), mask));

	v4l2r_nv15_row_to_p010_c(src, dst + i, n - i);
}

void v4l2r_nv15_row_to_nv12(const uint8_t *src, uint8_t *dst, unsigned int n)
{
	const uint8x16_t gather = vld1q_u8(nv15_gather);
	const int16x8_t shift = vld1q_s16(nv12_shift);
	size_t avail = v4l2r_nv15_row_bytes(n);
	unsigned int i = 0;

	for (; i + 32 <= n && avail >= 46; i += 32, src += 40, avail -= 40) {
		uint16x8_t a = vshlq_u16(nv15_words(src, gather), shift);
		uint16x8_t b = vshlq_u16(nv15_words(src + 10, gather), shift);
		uint16x8_t c = vshlq_u16(nv15_words(src + 20, gather), shift);
		uint16x8_t d = vshlq_u16(nv15_words(src + 30, gather), shift);

		vst1q_u8(dst + i, vcombine_u8(vmovn_u16(a), vmovn_u16(b)));
		vst1q_u8(dst + i + 16, vcombine_u8(vmovn_u16(c), vmovn_u16(d)));
	}

	for (; i + 8 <= n && avail >= 16; i += 8, src += 10, avail -= 10)
		vst1_u8(dst + i, vmovn_u16(vshlq_u16(nv15_words(src, gather),
						     shift)));

	v4l2r_nv15_row_to_nv12_c(src, dst + i, n - i);
}

int v4l2r_pixconv_have_simd(void)
{
	return 1;
}
#else
void v4l2r_nv15_row_to_p010(const uint8_t *src, uint16_t *dst, unsigned int n)
{
	v4l2r_nv15_row_to_p010_c(src, dst, n);
}

void v4l2r_nv15_row_to_nv12(const uint8_t *src, uint8_t *dst, unsigned int n)
{
	v4l2r_nv15_row_to_nv12_c(src, dst, n);
}

int v4l2r_pixconv_have_simd(void)
{
	return 0;
}
#endif

/* Rows [first, last) of the combined row space: luma rows, then chroma. */
struct pixconv_job {
	const struct v4l2r_pixconv_frame *frame;
	unsigned int first;
	unsigned int last;
};

static void convert_rows(const struct pixconv_job *job)
{
	const struct v4l2r_pixconv_frame *f = job->frame;

	for (unsigned int row = job->first; row < job->last; row++) {
		const uint8_t *src;
		uint8_t *dst;

		if (row < f->height) {
			src = f->src + (size_t)row * f->src_pitch;
			dst = f->dst + (size_t)row * f->dst_pitch;
		} else {
			unsigned int c = row - f->height;

			src = f->src + f->src_chroma_offset +
			      (size_t)c * f->src_pitch;
			dst = f->dst + f->dst_chroma_offset +
			      (size_t)c * f->dst_pitch;
		}

		if (f->dst_p010)
			v4l2r_nv15_row_to_p010(src, (uint16_t *)(void *)dst,
					       f->width);
		else
			v4l2r_nv15_row_to_nv12(src, dst, f->width);
	}
}

static void *convert_thread(void *arg)
{
	convert_rows(arg);
	return NULL;
}

static int frame_layout_ok(const struct v4l2r_pixconv_frame *f)
{
	size_t chroma_rows = (f->height + 1) / 2;
	size_t src_row = v4l2r_nv15_row_bytes(f->width);
	size_t dst_row = (size_t)f->width * (f->dst_p010 ? 2 : 1);

	if (!f->width || !f->height || f->src_pitch < src_row ||
	    f->dst_pitch < dst_row)
		return 0;
	if (f->dst_p010 && ((uintptr_t)f->dst % 2 || f->dst_pitch % 2 ||
			    f->dst_chroma_offset % 2))
		return 0;

	/* Last row of each plane must end inside the buffer. */
	if (f->src_chroma_offset < (size_t)f->src_pitch * (f->height - 1) + src_row ||
	    f->src_size < f->src_chroma_offset +
			  (size_t)f->src_pitch * (chroma_rows - 1) + src_row)
		return 0;
	if (f->dst_chroma_offset < (size_t)f->dst_pitch * (f->height - 1) + dst_row ||
	    f->dst_size < f->dst_chroma_offset +
			  (size_t)f->dst_pitch * (chroma_rows - 1) + dst_row)
		return 0;

	return 1;
}

#define PIXCONV_MAX_THREADS 8

int v4l2r_nv15_convert_frame(const struct v4l2r_pixconv_frame *frame,
			     unsigned int max_threads)
{
	struct pixconv_job jobs[PIXCONV_MAX_THREADS];
	pthread_t threads[PIXCONV_MAX_THREADS];
	unsigned int rows = frame->height + (frame->height + 1) / 2;
	unsigned int nb_jobs, started = 0;

	if (!frame_layout_ok(frame))
		return -1;

	nb_jobs = max_threads ? max_threads : 1;
	if (nb_jobs > PIXCONV_MAX_THREADS)
		nb_jobs = PIXCONV_MAX_THREADS;
	/* Not worth a thread for less than 64 rows. */
	if (nb_jobs > rows / 64)
		nb_jobs = rows / 64 ? rows / 64 : 1;

	for (unsigned int i = 0; i < nb_jobs; i++) {
		jobs[i].frame = frame;
		jobs[i].first = (unsigned int)((uint64_t)rows * i / nb_jobs);
		jobs[i].last = (unsigned int)((uint64_t)rows * (i + 1) / nb_jobs);
	}

	/* Job 0 runs here; a thread that cannot be started runs here too. */
	for (unsigned int i = 1; i < nb_jobs; i++) {
		if (pthread_create(&threads[i], NULL, convert_thread, &jobs[i]))
			break;
		started = i;
	}

	convert_rows(&jobs[0]);
	for (unsigned int i = started + 1; i < nb_jobs; i++)
		convert_rows(&jobs[i]);

	for (unsigned int i = 1; i <= started; i++)
		pthread_join(threads[i], NULL);

	return 0;
}
