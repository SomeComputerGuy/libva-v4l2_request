/*
 * Host test: format negotiation for 10-bit streams (fmtsel.c) on the
 * hardware combinations the driver meets:
 *  - stock RK3588 kernel: rkvdec writes NV15, the RGA2 cannot read it, so
 *    the CPU route turns it into P010;
 *  - RGA3 with NV15 support: NV15 -> P010 (and NV12) in hardware;
 *  - a converter that writes NV12 only (RGA2 with the NV15 input patch):
 *    keep NV12 as the 10-bit default so FFmpeg does not pick P010 over
 *    the free hardware path;
 *  - CPU route disabled and no converter: no 10-bit profiles;
 *  - a decoder writing P010 itself (VPU981 AV1).
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdio.h>

#include <va/va.h>

#include "fmtsel.h"

static int failures;

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		failures++; \
		fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
		fprintf(stderr, __VA_ARGS__); \
		fputc('\n', stderr); \
	} \
} while (0)

#define RT8  VA_RT_FORMAT_YUV420
#define RT10 VA_RT_FORMAT_YUV420_10
#define NV12 VA_FOURCC_NV12
#define P010 VA_FOURCC_P010

static const char *fcc(uint32_t f)
{
	return f == NV12 ? "NV12" : f == P010 ? "P010" : f ? "?" : "-";
}

static const char *rname(enum v4l2r_conv_route r)
{
	return r == V4L2R_ROUTE_HW ? "HW" : r == V4L2R_ROUTE_CPU ? "CPU" : "NONE";
}

struct route_case {
	uint32_t want;
	int strict;
	enum v4l2r_conv_route route;
	uint32_t dst;
};

struct system_case {
	const char *name;
	struct v4l2r_10bit_caps caps;
	int usable;
	int offer_p010;
	/* surface fourcc for (requested, rt_format) */
	uint32_t surf_default10, surf_default8, surf_nv12_10, surf_p010_8;
	struct route_case routes[4];
};

static const struct system_case systems[] = {
	{
		.name = "stock 15 (rkvdec NV15, RGA2 without NV15)",
		.caps = { .cpu = true },
		.usable = 1, .offer_p010 = 1,
		.surf_default10 = P010, .surf_default8 = NV12,
		.surf_nv12_10 = NV12, .surf_p010_8 = P010,
		.routes = {
			{ P010, 1, V4L2R_ROUTE_CPU, P010 },
			{ P010, 0, V4L2R_ROUTE_CPU, P010 },
			{ NV12, 1, V4L2R_ROUTE_CPU, NV12 },
			{ NV12, 0, V4L2R_ROUTE_CPU, NV12 },
		},
	},
	{
		.name = "RGA3 NV15 -> P010/NV12",
		.caps = { .hw_p010 = true, .hw_nv12 = true, .cpu = true },
		.usable = 1, .offer_p010 = 1,
		.surf_default10 = P010, .surf_default8 = NV12,
		.surf_nv12_10 = NV12, .surf_p010_8 = P010,
		.routes = {
			{ P010, 1, V4L2R_ROUTE_HW, P010 },
			{ P010, 0, V4L2R_ROUTE_HW, P010 },
			{ NV12, 1, V4L2R_ROUTE_HW, NV12 },
			{ NV12, 0, V4L2R_ROUTE_HW, NV12 },
		},
	},
	{
		.name = "RGA3, the converter refused the layout (P010 off)",
		.caps = { .hw_nv12 = true, .cpu = true },
		.usable = 1, .offer_p010 = 0,
		.surf_default10 = NV12, .surf_default8 = NV12,
		.surf_nv12_10 = NV12, .surf_p010_8 = P010,
		.routes = {
			{ P010, 1, V4L2R_ROUTE_CPU, P010 },
			{ P010, 0, V4L2R_ROUTE_CPU, P010 },
			{ NV12, 1, V4L2R_ROUTE_HW, NV12 },
			{ NV12, 0, V4L2R_ROUTE_HW, NV12 },
		},
	},
	{
		.name = "NV12-only converter, CPU route disabled",
		.caps = { .hw_nv12 = true },
		.usable = 1, .offer_p010 = 0,
		.surf_default10 = NV12, .surf_default8 = NV12,
		.surf_nv12_10 = NV12, .surf_p010_8 = P010,
		.routes = {
			{ P010, 1, V4L2R_ROUTE_NONE, 0 },
			{ P010, 0, V4L2R_ROUTE_HW, NV12 },
			{ NV12, 1, V4L2R_ROUTE_HW, NV12 },
			{ NV12, 0, V4L2R_ROUTE_HW, NV12 },
		},
	},
	{
		.name = "P010-only converter, CPU route disabled",
		.caps = { .hw_p010 = true },
		.usable = 1, .offer_p010 = 1,
		.surf_default10 = P010, .surf_default8 = NV12,
		.surf_nv12_10 = NV12, .surf_p010_8 = P010,
		.routes = {
			{ P010, 1, V4L2R_ROUTE_HW, P010 },
			{ P010, 0, V4L2R_ROUTE_HW, P010 },
			{ NV12, 1, V4L2R_ROUTE_NONE, 0 },
			{ NV12, 0, V4L2R_ROUTE_HW, P010 },
		},
	},
	{
		.name = "no converter, CPU route disabled",
		.caps = { 0 },
		.usable = 0, .offer_p010 = 0,
		.surf_default10 = NV12, .surf_default8 = NV12,
		.surf_nv12_10 = NV12, .surf_p010_8 = P010,
		.routes = {
			{ P010, 1, V4L2R_ROUTE_NONE, 0 },
			{ P010, 0, V4L2R_ROUTE_NONE, 0 },
			{ NV12, 1, V4L2R_ROUTE_NONE, 0 },
			{ NV12, 0, V4L2R_ROUTE_NONE, 0 },
		},
	},
	{
		.name = "decoder writes P010 itself (VPU981 AV1)",
		.caps = { .native_p010 = true },
		.usable = 1, .offer_p010 = 1,
		.surf_default10 = P010, .surf_default8 = NV12,
		.surf_nv12_10 = NV12, .surf_p010_8 = P010,
		.routes = {
			{ P010, 1, V4L2R_ROUTE_NONE, 0 },
			{ P010, 0, V4L2R_ROUTE_NONE, 0 },
			{ NV12, 1, V4L2R_ROUTE_NONE, 0 },
			{ NV12, 0, V4L2R_ROUTE_NONE, 0 },
		},
	},
};

int main(void)
{
	for (unsigned int i = 0; i < sizeof(systems) / sizeof(systems[0]); i++) {
		const struct system_case *c = &systems[i];
		uint32_t f;

		CHECK(v4l2r_fmtsel_10bit_usable(&c->caps) == c->usable,
		      "%s: usable %d", c->name, !c->usable);
		CHECK(v4l2r_fmtsel_offer_p010(&c->caps) == c->offer_p010,
		      "%s: offer P010 %d", c->name, !c->offer_p010);

		f = v4l2r_fmtsel_surface_fourcc(0, RT10, &c->caps);
		CHECK(f == c->surf_default10, "%s: 10-bit default %s", c->name, fcc(f));
		/* AV1 render targets carry both rt formats */
		f = v4l2r_fmtsel_surface_fourcc(0, RT8 | RT10, &c->caps);
		CHECK(f == c->surf_default10, "%s: 8+10-bit default %s", c->name, fcc(f));
		f = v4l2r_fmtsel_surface_fourcc(0, RT8, &c->caps);
		CHECK(f == c->surf_default8, "%s: 8-bit default %s", c->name, fcc(f));
		f = v4l2r_fmtsel_surface_fourcc(NV12, RT10, &c->caps);
		CHECK(f == c->surf_nv12_10, "%s: NV12 requested -> %s", c->name, fcc(f));
		f = v4l2r_fmtsel_surface_fourcc(P010, RT8, &c->caps);
		CHECK(f == c->surf_p010_8, "%s: P010 requested -> %s", c->name, fcc(f));

		for (unsigned int j = 0; j < 4; j++) {
			const struct route_case *r = &c->routes[j];
			uint32_t dst = 0xdeadbeef;
			enum v4l2r_conv_route got =
				v4l2r_fmtsel_route(r->want, r->strict, &c->caps, &dst);

			CHECK(got == r->route && dst == r->dst,
			      "%s: route to %s%s = %s %s, want %s %s", c->name,
			      fcc(r->want), r->strict ? " (strict)" : "",
			      rname(got), fcc(dst), rname(r->route), fcc(r->dst));
		}
	}

	printf("fmtsel: %s\n", failures ? "FAIL" : "PASS");
	return failures ? 1 : 0;
}
