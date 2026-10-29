/*
 * HEVC slice segment header RPS normalisation.
 *
 * Some stateless HEVC decoders (Rockchip VDPU381/VDPU383, as found in
 * RK3588 and RK3576) parse the slice segment header in hardware and look up
 * short_term_ref_pic_set_idx / lt_idx_sps in tables built from the SPS
 * st_ref_pic_set() / lt_ref_pic_poc_lsb_sps syntax
 * (V4L2_CID_STATELESS_HEVC_EXT_SPS_ST_RPS / _LT_RPS). VA-API never passes
 * that SPS syntax, only the reference picture set of the current picture.
 *
 * This module rewrites a slice segment NAL unit so that it carries its
 * reference picture set explicitly (short_term_ref_pic_set_sps_flag = 0,
 * st_ref_pic_set() without inter RPS prediction, and every long-term entry
 * coded with poc_lsb_lt), as if the SPS had num_short_term_ref_pic_sets = 0
 * and num_long_term_ref_pics_sps = 0. The decoder is then given an SPS with
 * those counts set to 0 and needs no SPS RPS tables. The rewrite is
 * semantics-preserving: RefPicSetStCurrBefore/After/Foll and
 * RefPicSetLtCurr/Foll, including their order, are unchanged, and the rest
 * of the slice segment header and the slice data are copied bit-exactly.
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef V4L2R_HEVC_RPS_H
#define V4L2R_HEVC_RPS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HEVC_RPS_MAX_ST		16
#define HEVC_RPS_MAX_LT		32
/* Largest slice segment header handled (RBSP bytes, including entry points). */
#define HEVC_RPS_MAX_HEADER	4096
/* How much an explicit RPS can add to a header. */
#define HEVC_RPS_HEADER_GROWTH	512
/* Output buffer size that is always enough for hevc_rps_rewrite_header(). */
#define HEVC_RPS_OUT_SIZE	((HEVC_RPS_MAX_HEADER + HEVC_RPS_HEADER_GROWTH) * 3 / 2)

/* What the slice header parser needs to know about the active SPS/PPS. */
struct hevc_rps_ps {
	uint32_t pic_size_in_ctbs_y;
	uint8_t log2_max_poc_lsb;
	uint8_t num_extra_slice_header_bits;
	uint8_t chroma_array_type;
	uint8_t num_short_term_ref_pic_sets;
	uint8_t num_long_term_ref_pics_sps;
	uint8_t num_ref_idx_l0_default_active_minus1;
	uint8_t num_ref_idx_l1_default_active_minus1;
	/* Size in bits of the st_ref_pic_set() coded in the slice header (VA st_rps_bits). */
	uint32_t st_rps_bits;
	bool st_rps_bits_valid;
	bool dependent_slice_segments_enabled;
	bool output_flag_present;
	bool separate_colour_plane;
	bool long_term_ref_pics_present;
	bool sps_temporal_mvp_enabled;
	bool sample_adaptive_offset_enabled;
	bool lists_modification_present;
	bool cabac_init_present;
	bool weighted_pred;
	bool weighted_bipred;
	bool pps_slice_chroma_qp_offsets_present;
	bool chroma_qp_offset_list_enabled;
	bool deblocking_filter_override_enabled;
	bool pps_deblocking_filter_disabled;
	bool pps_loop_filter_across_slices_enabled;
	bool tiles_enabled;
	bool entropy_coding_sync_enabled;
	bool slice_segment_header_extension_present;
};

struct hevc_rps_st_entry {
	int32_t delta_poc;	/* POC of the reference minus POC of the current picture */
	bool used;		/* in RefPicSetStCurrBefore/After rather than StFoll */
};

struct hevc_rps_lt_entry {
	int32_t poc;		/* full PicOrderCntVal of the long-term reference */
	bool used;		/* in RefPicSetLtCurr rather than LtFoll */
};

/*
 * The reference picture set of the current picture, as VA-API describes it
 * (VAPictureParameterBufferHEVC.ReferenceFrames and their RPS flags).
 * Order does not matter; the rewriter sorts the short-term entries and takes
 * the long-term order from the original slice header.
 */
struct hevc_rps_set {
	int32_t poc;			/* PicOrderCntVal of the current picture */
	unsigned int num_st;
	struct hevc_rps_st_entry st[HEVC_RPS_MAX_ST];
	unsigned int num_lt;
	struct hevc_rps_lt_entry lt[HEVC_RPS_MAX_LT];
	/*
	 * NumPicTotalCurr (7-55), needed to parse ref_pic_lists_modification().
	 * The rewrite does not change it.
	 */
	unsigned int num_pic_total_curr;
};

struct hevc_rps_result {
	bool rewritten;			/* false: the header was kept (IDR, dependent slice segment) */
	uint32_t nal_unit_type;
	uint32_t temporal_id_plus1;
	bool no_output_of_prior_pics;
	/* Sizes of the new header's RPS syntax, in bits. */
	uint32_t short_term_ref_pic_set_size;
	uint32_t long_term_ref_pic_set_size;
	/* Size of the new slice segment header including byte_alignment(), RBSP bytes. */
	uint32_t header_rbsp_bytes;
	/* Where slice_segment_data() starts in the input NAL unit (escaped bytes). */
	uint32_t src_data_offset;
};

enum hevc_rps_error {
	HEVC_RPS_OK = 0,
	HEVC_RPS_ERR_PARSE = -1,	/* slice header could not be parsed */
	HEVC_RPS_ERR_ALIGN = -2,	/* no byte_alignment() where the header should end */
	HEVC_RPS_ERR_LT_MATCH = -3,	/* long-term entry could not be matched to a reference */
	HEVC_RPS_ERR_LT_ORDER = -4,	/* long-term MSB cycles not representable without lt_idx_sps */
	HEVC_RPS_ERR_SPACE = -5,	/* output buffer too small */
	HEVC_RPS_ERR_RANGE = -6,	/* RPS too large for the syntax */
};

/*
 * Rewrite the slice segment header of one slice segment NAL unit (starting
 * at the 2-byte NAL unit header, no start code, emulation prevention bytes
 * included). The new header, NAL unit header included and escaped, is
 * written to out (out_cap >= HEVC_RPS_OUT_SIZE is always enough); the
 * rewritten NAL unit is out[0 .. *out_size) followed by
 * nal[res->src_data_offset .. nal_size), so the slice data is never copied
 * here. ps describes the original SPS/PPS; the result is valid with the
 * same PPS and an SPS whose num_short_term_ref_pic_sets and
 * num_long_term_ref_pics_sps are 0.
 * Returns HEVC_RPS_OK or a negative enum hevc_rps_error.
 */
int hevc_rps_rewrite_header(const struct hevc_rps_ps *ps,
			    const struct hevc_rps_set *rps,
			    const uint8_t *nal, size_t nal_size,
			    uint8_t *out, size_t out_cap, size_t *out_size,
			    struct hevc_rps_result *res);

const char *hevc_rps_strerror(int err);

#endif /* V4L2R_HEVC_RPS_H */
