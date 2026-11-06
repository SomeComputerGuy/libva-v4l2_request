/*
 * Format negotiation for 10-bit streams.
 *
 * Rockchip decoders write 10-bit frames as packed NV15, which neither VA
 * nor any display or GPU importer takes. The surfaces clients see are P010
 * (10 bits kept, what HDR output needs) or, for clients that ask for it,
 * NV12. The CAPTURE buffer reaches the surface format through a mem2mem
 * converter (RGA3: NV15 -> P010/NV12) or, without one, the CPU.
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <va/va.h>

#include "fmtsel.h"

bool v4l2r_fmtsel_10bit_usable(const struct v4l2r_10bit_caps *caps)
{
	return caps->native_p010 || caps->hw_p010 || caps->hw_nv12 ||
	       caps->cpu;
}

bool v4l2r_fmtsel_offer_p010(const struct v4l2r_10bit_caps *caps)
{
	if (caps->native_p010 || caps->hw_p010)
		return true;

	return !caps->hw_nv12 && caps->cpu;
}

uint32_t v4l2r_fmtsel_surface_fourcc(uint32_t requested_fourcc,
				     unsigned int rt_format,
				     const struct v4l2r_10bit_caps *caps)
{
	if (requested_fourcc == VA_FOURCC_NV12 ||
	    requested_fourcc == VA_FOURCC_P010)
		return requested_fourcc;

	if ((rt_format & VA_RT_FORMAT_YUV420_10) &&
	    v4l2r_fmtsel_offer_p010(caps))
		return VA_FOURCC_P010;

	return VA_FOURCC_NV12;
}

static enum v4l2r_conv_route route_to(uint32_t fourcc,
				      const struct v4l2r_10bit_caps *caps)
{
	if (fourcc == VA_FOURCC_P010) {
		if (caps->hw_p010)
			return V4L2R_ROUTE_HW;
		if (caps->cpu)
			return V4L2R_ROUTE_CPU;
	} else if (fourcc == VA_FOURCC_NV12) {
		if (caps->hw_nv12)
			return V4L2R_ROUTE_HW;
		if (caps->cpu)
			return V4L2R_ROUTE_CPU;
	}

	return V4L2R_ROUTE_NONE;
}

enum v4l2r_conv_route v4l2r_fmtsel_route(uint32_t wanted_fourcc, bool strict,
					 const struct v4l2r_10bit_caps *caps,
					 uint32_t *dst_fourcc)
{
	enum v4l2r_conv_route route = route_to(wanted_fourcc, caps);
	uint32_t other;

	if (route != V4L2R_ROUTE_NONE || strict) {
		*dst_fourcc = route != V4L2R_ROUTE_NONE ? wanted_fourcc : 0;
		return route;
	}

	other = wanted_fourcc == VA_FOURCC_P010 ? VA_FOURCC_NV12 :
						  VA_FOURCC_P010;
	route = route_to(other, caps);
	*dst_fourcc = route != V4L2R_ROUTE_NONE ? other : 0;
	return route;
}
