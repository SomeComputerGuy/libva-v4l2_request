/*
 * Format negotiation for 10-bit streams: which surface formats to offer
 * and how a decoder-only CAPTURE format (NV15) reaches the surface format.
 * Pure decisions over what the hardware can do, see fmtsel.c.
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef V4L2R_FMTSEL_H
#define V4L2R_FMTSEL_H

#include <stdbool.h>
#include <stdint.h>

/* What the system can do with 10-bit decodes. */
struct v4l2r_10bit_caps {
	bool native_p010;	/* some decoder writes P010 itself */
	bool hw_p010;		/* a mem2mem converter turns NV15 into P010 */
	bool hw_nv12;		/* a mem2mem converter turns NV15 into NV12 */
	bool cpu;		/* the CPU fallback may convert NV15 */
};

enum v4l2r_conv_route {
	V4L2R_ROUTE_NONE = 0,
	V4L2R_ROUTE_HW,		/* mem2mem converter, zero-copy */
	V4L2R_ROUTE_CPU,	/* pixconv.c */
};

/* Whether 10-bit profiles can hand out frames at all. */
bool v4l2r_fmtsel_10bit_usable(const struct v4l2r_10bit_caps *caps);

/*
 * Whether 10-bit configs offer P010 surfaces. They do unless the only fast
 * route is a hardware converter that writes NV12 alone: FFmpeg picks P010
 * as the exact match for 10-bit streams whenever it is offered, and P010
 * would then go through the CPU instead of the converter.
 */
bool v4l2r_fmtsel_offer_p010(const struct v4l2r_10bit_caps *caps);

/*
 * The VA fourcc a surface presents: what the client asked for, otherwise
 * P010 for 10-bit render targets (when offered) and NV12 for the rest.
 */
uint32_t v4l2r_fmtsel_surface_fourcc(uint32_t requested_fourcc,
				     unsigned int rt_format,
				     const struct v4l2r_10bit_caps *caps);

/*
 * Route from NV15 to the wanted surface fourcc (NV12 or P010): hardware
 * first, then the CPU. *dst_fourcc gets the fourcc the route produces.
 * When the client did not insist on a format (strict false) and the wanted
 * one has no route, the other one is used.
 */
enum v4l2r_conv_route v4l2r_fmtsel_route(uint32_t wanted_fourcc, bool strict,
					 const struct v4l2r_10bit_caps *caps,
					 uint32_t *dst_fourcc);

#endif
