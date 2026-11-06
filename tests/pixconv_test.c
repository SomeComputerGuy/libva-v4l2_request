/*
 * Host test: the CPU NV15 -> P010 / NV12 conversion is bit-exact.
 *
 * The reference packs random 10-bit samples into NV15 straight from the
 * kernel's description (pixfmt-yuv-planar.rst: 4 samples in 5 bytes,
 * little endian, sample i at bit 10 * i of the row) and expects P010 =
 * sample << 6 and NV12 = sample >> 2. Rows are placed right before a
 * PROT_NONE page so a read past the row's last byte faults; every width
 * from 1 to 200 covers all vector/scalar tail splits. Whole frames are
 * checked with padded pitches, separate chroma offsets and 1..8 threads.
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "pixconv.h"

static int failures;

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		failures++; \
		fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
		fprintf(stderr, __VA_ARGS__); \
		fputc('\n', stderr); \
	} \
} while (0)

static uint32_t rng = 0x12345678;

static uint16_t rand10(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 17;
	rng ^= rng << 5;
	return rng & 0x3ff;
}

/* Reference packer, independent of the converter: bit 10 * i. */
static void pack_nv15(const uint16_t *samples, unsigned int n, uint8_t *dst)
{
	memset(dst, 0, v4l2r_nv15_row_bytes(n));
	for (unsigned int i = 0; i < n; i++) {
		for (unsigned int b = 0; b < 10; b++) {
			unsigned int bit = 10 * i + b;

			if (samples[i] & (1u << b))
				dst[bit / 8] |= (uint8_t)(1u << (bit % 8));
		}
	}
}

/* The table of the kernel documentation, first row: Y00..Y03. */
static void test_doc_layout(void)
{
	/* Y00 = 0x3ff, Y01 = 0x000, Y02 = 0x155, Y03 = 0x2aa:
	 * byte0 = Y00[7:0], byte1 = Y01[5:0] Y00[9:8], byte2 = Y02[3:0]
	 * Y01[9:6], byte3 = Y03[1:0] Y02[9:4], byte4 = Y03[9:2]. */
	const uint8_t row[5] = {
		0xff, 0x03, (0x155 & 0xf) << 4, (0x2aa & 0x3) << 6 | 0x155 >> 4,
		0x2aa >> 2,
	};
	const uint16_t want[4] = { 0x3ff, 0x000, 0x155, 0x2aa };
	uint16_t p010[4];
	uint8_t nv12[4];

	v4l2r_nv15_row_to_p010(row, p010, 4);
	v4l2r_nv15_row_to_nv12(row, nv12, 4);
	for (int i = 0; i < 4; i++) {
		CHECK(p010[i] == want[i] << 6, "doc sample %d p010 %#x want %#x",
		      i, p010[i], want[i] << 6);
		CHECK(nv12[i] == want[i] >> 2, "doc sample %d nv12 %#x want %#x",
		      i, nv12[i], want[i] >> 2);
	}
}

/* A buffer whose end touches an inaccessible page. */
struct guarded {
	uint8_t *base;
	size_t size;
};

static uint8_t *guarded_tail(struct guarded *g, size_t bytes)
{
	size_t page = (size_t)sysconf(_SC_PAGESIZE);
	size_t span = (bytes + page - 1) / page * page;

	g->size = span + page;
	g->base = mmap(NULL, g->size, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (g->base == MAP_FAILED)
		abort();
	if (mprotect(g->base + span, page, PROT_NONE))
		abort();
	return g->base + span - bytes;
}

static void guarded_free(struct guarded *g)
{
	munmap(g->base, g->size);
}

static void test_rows(void)
{
	for (unsigned int n = 1; n <= 200; n++) {
		uint16_t samples[200], p010[200 + 8], p010_c[200];
		uint8_t nv12[200 + 8], nv12_c[200];
		size_t bytes = v4l2r_nv15_row_bytes(n);
		struct guarded g;
		uint8_t *row = guarded_tail(&g, bytes);

		for (unsigned int i = 0; i < n; i++)
			samples[i] = rand10();
		pack_nv15(samples, n, row);

		/* Canaries after the n outputs must survive. */
		for (unsigned int i = 0; i < 8; i++) {
			p010[n + i] = 0xbeef;
			nv12[n + i] = 0xa5;
		}

		v4l2r_nv15_row_to_p010(row, p010, n);
		v4l2r_nv15_row_to_nv12(row, nv12, n);
		v4l2r_nv15_row_to_p010_c(row, p010_c, n);
		v4l2r_nv15_row_to_nv12_c(row, nv12_c, n);

		for (unsigned int i = 0; i < n; i++) {
			CHECK(p010[i] == (uint16_t)(samples[i] << 6),
			      "n=%u sample %u p010 %#x want %#x", n, i,
			      p010[i], samples[i] << 6);
			CHECK(p010_c[i] == (uint16_t)(samples[i] << 6),
			      "n=%u sample %u p010_c %#x", n, i, p010_c[i]);
			CHECK(nv12[i] == samples[i] >> 2,
			      "n=%u sample %u nv12 %#x want %#x", n, i,
			      nv12[i], samples[i] >> 2);
			CHECK(nv12_c[i] == samples[i] >> 2,
			      "n=%u sample %u nv12_c %#x", n, i, nv12_c[i]);
		}
		for (unsigned int i = 0; i < 8; i++) {
			CHECK(p010[n + i] == 0xbeef, "n=%u p010 wrote past the row", n);
			CHECK(nv12[n + i] == 0xa5, "n=%u nv12 wrote past the row", n);
		}

		guarded_free(&g);
	}
}

static void test_frame(unsigned int width, unsigned int height,
		       unsigned int src_pad, unsigned int dst_pad,
		       unsigned int src_gap_rows, int p010,
		       unsigned int threads)
{
	unsigned int crows = (height + 1) / 2;
	uint32_t src_pitch = (uint32_t)v4l2r_nv15_row_bytes(width) + src_pad;
	uint32_t dst_pitch = width * (p010 ? 2 : 1) + dst_pad;
	size_t src_coff = (size_t)src_pitch * (height + src_gap_rows);
	size_t src_size = src_coff + (size_t)src_pitch * crows;
	size_t dst_coff = (size_t)dst_pitch * height;
	size_t dst_size = dst_coff + (size_t)dst_pitch * crows;
	uint16_t *samples = malloc(sizeof(uint16_t) * width * (height + crows));
	uint8_t *dst = malloc(dst_size);
	struct guarded g;
	uint8_t *src = guarded_tail(&g, src_size);
	struct v4l2r_pixconv_frame frame;
	unsigned int bad = 0;

	memset(src, 0xcc, src_size);
	memset(dst, 0x5a, dst_size);
	for (unsigned int r = 0; r < height + crows; r++) {
		uint16_t *s = samples + (size_t)r * width;
		uint8_t *row = r < height ? src + (size_t)r * src_pitch :
			       src + src_coff + (size_t)(r - height) * src_pitch;

		for (unsigned int i = 0; i < width; i++)
			s[i] = rand10();
		pack_nv15(s, width, row);
	}

	frame = (struct v4l2r_pixconv_frame) {
		.src = src, .src_size = src_size, .src_pitch = src_pitch,
		.src_chroma_offset = src_coff,
		.dst = dst, .dst_size = dst_size, .dst_pitch = dst_pitch,
		.dst_chroma_offset = dst_coff,
		.width = width, .height = height, .dst_p010 = p010,
	};
	CHECK(v4l2r_nv15_convert_frame(&frame, threads) == 0,
	      "frame %ux%u rejected", width, height);

	for (unsigned int r = 0; r < height + crows; r++) {
		const uint16_t *s = samples + (size_t)r * width;
		const uint8_t *row = r < height ? dst + (size_t)r * dst_pitch :
				     dst + dst_coff + (size_t)(r - height) * dst_pitch;

		for (unsigned int i = 0; i < width; i++) {
			unsigned int got, want;

			if (p010) {
				got = row[2 * i] | row[2 * i + 1] << 8;
				want = (unsigned int)s[i] << 6;
			} else {
				got = row[i];
				want = s[i] >> 2;
			}
			bad += got != want;
		}
		/* Pitch padding is never written. */
		for (unsigned int i = width * (p010 ? 2 : 1); i < dst_pitch; i++)
			bad += row[i] != 0x5a;
	}
	CHECK(!bad, "frame %ux%u pad %u/%u gap %u %s threads %u: %u mismatches",
	      width, height, src_pad, dst_pad, src_gap_rows,
	      p010 ? "P010" : "NV12", threads, bad);

	guarded_free(&g);
	free(dst);
	free(samples);
}

static void test_frame_rejects(void)
{
	static uint8_t src[4096], dst[8192];
	struct v4l2r_pixconv_frame ok = {
		.src = src, .src_size = 40 * 24, .src_pitch = 40,
		.src_chroma_offset = 40 * 16,
		.dst = dst, .dst_size = 64 * 24, .dst_pitch = 64,
		.dst_chroma_offset = 64 * 16,
		.width = 32, .height = 16, .dst_p010 = 1,
	};
	struct v4l2r_pixconv_frame f;

	CHECK(v4l2r_nv15_convert_frame(&ok, 1) == 0, "valid layout rejected");

	f = ok; f.src_pitch = 39;
	CHECK(v4l2r_nv15_convert_frame(&f, 1) < 0, "short source pitch taken");
	f = ok; f.dst_pitch = 62;
	CHECK(v4l2r_nv15_convert_frame(&f, 1) < 0, "short P010 pitch taken");
	f = ok; f.src_size = 40 * 24 - 1;
	CHECK(v4l2r_nv15_convert_frame(&f, 1) < 0, "source overrun taken");
	f = ok; f.dst_size = 64 * 24 - 1;
	CHECK(v4l2r_nv15_convert_frame(&f, 1) < 0, "destination overrun taken");
	f = ok; f.src_chroma_offset = 40 * 15;
	CHECK(v4l2r_nv15_convert_frame(&f, 1) < 0, "overlapping chroma taken");
	f = ok; f.width = 0;
	CHECK(v4l2r_nv15_convert_frame(&f, 1) < 0, "empty frame taken");
}

int main(void)
{
	test_doc_layout();
	test_rows();

	/* Decoder-like layouts: rkvdec 1080p/4K (64-aligned, 2176 rows of
	 * which 2160 are used would be the gap case), odd sizes, padding. */
	test_frame(1920, 1080, 0, 0, 0, 1, 4);
	test_frame(1920, 1088, 0, 64, 0, 0, 4);
	test_frame(3840, 2160, 0, 0, 16, 1, 4);
	test_frame(416, 240, 12, 32, 0, 1, 1);
	test_frame(130, 66, 3, 6, 2, 1, 3);
	test_frame(130, 67, 3, 1, 0, 0, 8);
	test_frame(36, 2, 0, 0, 0, 1, 8);
	test_frame_rejects();

	printf("pixconv: %s (simd %d)\n", failures ? "FAIL" : "PASS",
	       v4l2r_pixconv_have_simd());
	return failures ? 1 : 0;
}
