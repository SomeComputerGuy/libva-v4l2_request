/*
 * HEVC slice segment header RPS normalisation, see hevc_rps.h.
 * Section and equation numbers refer to ITU-T H.265 (V10, 07/2024).
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdlib.h>
#include <string.h>

#include "hevc_rps.h"

#define NAL_BLA_W_LP		16
#define NAL_IDR_W_RADL		19
#define NAL_IDR_N_LP		20
#define NAL_RSV_IRAP_VCL23	23

#define SLICE_B			0
#define SLICE_P			1
#define SLICE_I			2

/* --- plain RBSP bit reader / writer (no emulation prevention handling) --- */

struct rd {
	const uint8_t *p;
	size_t bits;	/* total bits */
	size_t pos;
	bool err;
};

static uint32_t rd_bit(struct rd *r)
{
	uint32_t v;

	if (r->pos >= r->bits) {
		r->err = true;
		return 0;
	}
	v = (r->p[r->pos >> 3] >> (7 - (r->pos & 7))) & 1;
	r->pos++;
	return v;
}

static uint32_t rd_u(struct rd *r, unsigned int n)
{
	uint32_t v = 0;

	while (n--)
		v = (v << 1) | rd_bit(r);
	return v;
}

static uint32_t rd_ue(struct rd *r)
{
	unsigned int zeros = 0;

	while (!rd_bit(r)) {
		if (r->err || ++zeros > 31) {
			r->err = true;
			return 0;
		}
	}
	return (uint32_t)((1ull << zeros) - 1 + rd_u(r, zeros));
}

static int32_t rd_se(struct rd *r)
{
	uint32_t v = rd_ue(r);

	return (v & 1) ? (int32_t)((v + 1) / 2) : -(int32_t)(v / 2);
}

struct wr {
	uint8_t *p;
	size_t cap_bits;
	size_t pos;
	bool err;
};

static void wr_bit(struct wr *w, uint32_t v)
{
	if (w->pos >= w->cap_bits) {
		w->err = true;
		return;
	}
	if (v)
		w->p[w->pos >> 3] |= 0x80 >> (w->pos & 7);
	else
		w->p[w->pos >> 3] &= ~(0x80 >> (w->pos & 7));
	w->pos++;
}

static void wr_u(struct wr *w, uint32_t v, unsigned int n)
{
	while (n--)
		wr_bit(w, (v >> n) & 1);
}

static void wr_ue(struct wr *w, uint32_t v)
{
	uint64_t x = (uint64_t)v + 1;
	unsigned int len = 0;

	while ((x >> len) > 1)
		len++;
	wr_u(w, 0, len);
	wr_u(w, (uint32_t)x, len + 1);
}

/* Copy bits [from, to) of the reader's buffer. */
static void wr_copy(struct wr *w, const struct rd *r, size_t from, size_t to)
{
	for (size_t i = from; i < to; i++)
		wr_bit(w, (r->p[i >> 3] >> (7 - (i & 7))) & 1);
}

static unsigned int ceil_log2(uint32_t v)
{
	unsigned int n = 0;

	while ((1ull << n) < v)
		n++;
	return n;
}

/* --- emulation prevention (7.4.2) --- */

static size_t unescape(const uint8_t *src, size_t n, uint8_t *dst)
{
	size_t o = 0;
	unsigned int zeros = 0;

	for (size_t i = 0; i < n; i++) {
		if (zeros >= 2 && src[i] == 0x03) {
			zeros = 0;
			continue;
		}
		dst[o++] = src[i];
		zeros = src[i] ? 0 : zeros + 1;
	}
	return o;
}

/* Returns the escaped size, or 0 when out of space. *mark is translated in place. */
static size_t escape(const uint8_t *src, size_t n, uint8_t *dst, size_t cap,
		     uint32_t *mark)
{
	size_t o = 0;
	unsigned int zeros = 0;
	uint32_t m = *mark;

	for (size_t i = 0; i < n; i++) {
		if (zeros >= 2 && src[i] <= 0x03) {
			if (o >= cap)
				return 0;
			dst[o++] = 0x03;
			zeros = 0;
		}
		if (i == m)
			*mark = (uint32_t)o;
		if (o >= cap)
			return 0;
		dst[o++] = src[i];
		zeros = src[i] ? 0 : zeros + 1;
	}
	/* 7.4.2: a final 0x00 (cabac_zero_word) is followed by 0x03. */
	if (n && src[n - 1] == 0x00) {
		if (o >= cap)
			return 0;
		dst[o++] = 0x03;
	}
	if (m >= n)
		*mark = (uint32_t)o;
	return o;
}

/* --- slice segment header parsing (7.3.6.1) --- */

struct lt_orig {
	bool from_sps;
	uint32_t lt_idx_sps;
	uint32_t poc_lsb;
	bool used;
	bool msb_present;
	uint32_t msb_cycle;	/* delta_poc_msb_cycle_lt as coded */
};

struct hdr {
	uint32_t nal_unit_type, temporal_id_plus1;
	bool idr, dependent;
	bool no_output_of_prior_pics;
	uint32_t slice_type;
	uint32_t poc_lsb;
	size_t rps_start, rps_end;	/* [short_term_ref_pic_set_sps_flag, end of LT syntax) */
	uint32_t num_lt_sps, num_lt_pics;
	struct lt_orig lt[HEVC_RPS_MAX_LT];
	size_t hdr_end;			/* position of alignment_bit_equal_to_one */
	size_t data_byte;		/* first RBSP byte of slice_segment_data() */
};

/* 7.3.7, size only: parse an st_ref_pic_set(num_short_term_ref_pic_sets) in a slice header. */
static void skip_st_ref_pic_set(struct rd *r, const struct hevc_rps_ps *ps)
{
	size_t start = r->pos;

	if (ps->st_rps_bits_valid) {
		/*
		 * With inter RPS prediction the syntax length depends on
		 * NumDeltaPocs[RefRpsIdx] from the SPS, which we do not have;
		 * VA-API gives the coded size instead.
		 */
		if (start + ps->st_rps_bits > r->bits)
			r->err = true;
		else
			r->pos = start + ps->st_rps_bits;
		return;
	}

	if (ps->num_short_term_ref_pic_sets && rd_bit(r)) {
		/* inter_ref_pic_set_prediction_flag without a known size */
		r->err = true;
		return;
	}
	uint32_t neg = rd_ue(r), pos = rd_ue(r);

	if (neg > HEVC_RPS_MAX_ST || pos > HEVC_RPS_MAX_ST) {
		r->err = true;
		return;
	}
	for (uint32_t i = 0; i < neg + pos && !r->err; i++) {
		rd_ue(r);	/* delta_poc_sX_minus1 */
		rd_bit(r);	/* used_by_curr_pic_sX_flag */
	}
}

static void skip_pred_weight_table(struct rd *r, const struct hevc_rps_ps *ps,
				   uint32_t slice_type, uint32_t n_l0, uint32_t n_l1)
{
	rd_ue(r);					/* luma_log2_weight_denom */
	if (ps->chroma_array_type)
		rd_se(r);				/* delta_chroma_log2_weight_denom */

	for (int list = 0; list < (slice_type == SLICE_B ? 2 : 1); list++) {
		uint32_t n = list ? n_l1 : n_l0;
		bool luma[16] = { 0 }, chroma[16] = { 0 };

		if (n > 16) {
			r->err = true;
			return;
		}
		/*
		 * The flags are skipped for references with the same POC as the
		 * current picture only with pps_curr_pic_ref_enabled_flag (SCC),
		 * which this module does not support.
		 */
		for (uint32_t i = 0; i < n; i++)
			luma[i] = rd_bit(r);
		if (ps->chroma_array_type)
			for (uint32_t i = 0; i < n; i++)
				chroma[i] = rd_bit(r);
		for (uint32_t i = 0; i < n; i++) {
			if (luma[i]) {
				rd_se(r);		/* delta_luma_weight_lX */
				rd_se(r);		/* luma_offset_lX */
			}
			if (chroma[i]) {
				for (int j = 0; j < 2; j++) {
					rd_se(r);	/* delta_chroma_weight_lX */
					rd_se(r);	/* delta_chroma_offset_lX */
				}
			}
		}
	}
}

static int parse_header(struct rd *r, const struct hevc_rps_ps *ps,
			const struct hevc_rps_set *rps, struct hdr *h)
{
	bool first_slice, irap;
	bool slice_temporal_mvp = false, sao_luma = false, sao_chroma = false;
	bool deblocking_disabled = ps->pps_deblocking_filter_disabled;

	memset(h, 0, sizeof(*h));

	rd_bit(r);				/* forbidden_zero_bit */
	h->nal_unit_type = rd_u(r, 6);
	rd_u(r, 6);				/* nuh_layer_id */
	h->temporal_id_plus1 = rd_u(r, 3);
	if (h->nal_unit_type > NAL_RSV_IRAP_VCL23)
		return HEVC_RPS_ERR_PARSE;	/* not a VCL NAL unit we know */

	h->idr = h->nal_unit_type == NAL_IDR_W_RADL || h->nal_unit_type == NAL_IDR_N_LP;
	irap = h->nal_unit_type >= NAL_BLA_W_LP;

	first_slice = rd_bit(r);
	if (irap)
		h->no_output_of_prior_pics = rd_bit(r);
	rd_ue(r);				/* slice_pic_parameter_set_id */
	if (!first_slice) {
		if (ps->dependent_slice_segments_enabled)
			h->dependent = rd_bit(r);
		rd_u(r, ceil_log2(ps->pic_size_in_ctbs_y));	/* slice_segment_address */
	}

	if (!h->dependent) {
		uint32_t n_l0 = ps->num_ref_idx_l0_default_active_minus1 + 1;
		uint32_t n_l1 = ps->num_ref_idx_l1_default_active_minus1 + 1;

		rd_u(r, ps->num_extra_slice_header_bits);	/* slice_reserved_flag[] */
		h->slice_type = rd_ue(r);
		if (h->slice_type > SLICE_I)
			return HEVC_RPS_ERR_PARSE;
		if (ps->output_flag_present)
			rd_bit(r);				/* pic_output_flag */
		if (ps->separate_colour_plane)
			rd_u(r, 2);				/* colour_plane_id */

		if (!h->idr) {
			h->poc_lsb = rd_u(r, ps->log2_max_poc_lsb);
			h->rps_start = r->pos;
			if (!rd_bit(r))				/* short_term_ref_pic_set_sps_flag */
				skip_st_ref_pic_set(r, ps);
			else if (ps->num_short_term_ref_pic_sets > 1)
				rd_u(r, ceil_log2(ps->num_short_term_ref_pic_sets));

			if (ps->long_term_ref_pics_present) {
				if (ps->num_long_term_ref_pics_sps)
					h->num_lt_sps = rd_ue(r);
				h->num_lt_pics = rd_ue(r);
				if (h->num_lt_sps + h->num_lt_pics > HEVC_RPS_MAX_LT)
					return HEVC_RPS_ERR_RANGE;
				for (uint32_t i = 0; i < h->num_lt_sps + h->num_lt_pics; i++) {
					struct lt_orig *lt = &h->lt[i];

					if (i < h->num_lt_sps) {
						lt->from_sps = true;
						if (ps->num_long_term_ref_pics_sps > 1)
							lt->lt_idx_sps = rd_u(r,
								ceil_log2(ps->num_long_term_ref_pics_sps));
					} else {
						lt->poc_lsb = rd_u(r, ps->log2_max_poc_lsb);
						lt->used = rd_bit(r);
					}
					lt->msb_present = rd_bit(r);
					if (lt->msb_present)
						lt->msb_cycle = rd_ue(r);
				}
			}
			h->rps_end = r->pos;
			if (ps->sps_temporal_mvp_enabled)
				slice_temporal_mvp = rd_bit(r);
		}

		if (ps->sample_adaptive_offset_enabled) {
			sao_luma = rd_bit(r);
			if (ps->chroma_array_type)
				sao_chroma = rd_bit(r);
		}

		if (h->slice_type != SLICE_I) {
			bool collocated_from_l0 = true;

			if (rd_bit(r)) {			/* num_ref_idx_active_override_flag */
				n_l0 = rd_ue(r) + 1;
				if (h->slice_type == SLICE_B)
					n_l1 = rd_ue(r) + 1;
			}
			if (n_l0 > 16 || n_l1 > 16)
				return HEVC_RPS_ERR_PARSE;
			if (ps->lists_modification_present && rps->num_pic_total_curr > 1) {
				unsigned int bits = ceil_log2(rps->num_pic_total_curr);

				if (rd_bit(r))			/* ref_pic_list_modification_flag_l0 */
					for (uint32_t i = 0; i < n_l0; i++)
						rd_u(r, bits);
				if (h->slice_type == SLICE_B && rd_bit(r))
					for (uint32_t i = 0; i < n_l1; i++)
						rd_u(r, bits);
			}
			if (h->slice_type == SLICE_B)
				rd_bit(r);			/* mvd_l1_zero_flag */
			if (ps->cabac_init_present)
				rd_bit(r);			/* cabac_init_flag */
			if (slice_temporal_mvp) {
				if (h->slice_type == SLICE_B)
					collocated_from_l0 = rd_bit(r);
				if ((collocated_from_l0 && n_l0 > 1) ||
				    (!collocated_from_l0 && n_l1 > 1))
					rd_ue(r);		/* collocated_ref_idx */
			}
			if ((ps->weighted_pred && h->slice_type == SLICE_P) ||
			    (ps->weighted_bipred && h->slice_type == SLICE_B))
				skip_pred_weight_table(r, ps, h->slice_type, n_l0, n_l1);
			rd_ue(r);				/* five_minus_max_num_merge_cand */
		}

		rd_se(r);					/* slice_qp_delta */
		if (ps->pps_slice_chroma_qp_offsets_present) {
			rd_se(r);				/* slice_cb_qp_offset */
			rd_se(r);				/* slice_cr_qp_offset */
		}
		if (ps->chroma_qp_offset_list_enabled)
			rd_bit(r);				/* cu_chroma_qp_offset_enabled_flag */
		if (ps->deblocking_filter_override_enabled && rd_bit(r)) {
			deblocking_disabled = rd_bit(r);
			if (!deblocking_disabled) {
				rd_se(r);			/* slice_beta_offset_div2 */
				rd_se(r);			/* slice_tc_offset_div2 */
			}
		}
		if (ps->pps_loop_filter_across_slices_enabled &&
		    (sao_luma || sao_chroma || !deblocking_disabled))
			rd_bit(r);		/* slice_loop_filter_across_slices_enabled_flag */
	}

	if (ps->tiles_enabled || ps->entropy_coding_sync_enabled) {
		uint32_t n = rd_ue(r);

		if (n) {
			uint32_t len = rd_ue(r) + 1;

			if (len > 32)
				return HEVC_RPS_ERR_PARSE;
			for (uint32_t i = 0; i < n && !r->err; i++)
				rd_u(r, len);		/* entry_point_offset_minus1 */
		}
	}
	if (ps->slice_segment_header_extension_present) {
		uint32_t len = rd_ue(r);

		r->pos += (size_t)len * 8;		/* slice_segment_header_extension_data_byte */
		if (r->pos > r->bits)
			r->err = true;
	}
	if (r->err)
		return HEVC_RPS_ERR_PARSE;

	/* byte_alignment() */
	h->hdr_end = r->pos;
	if (!rd_bit(r))
		return HEVC_RPS_ERR_ALIGN;
	while (r->pos & 7)
		if (rd_bit(r))
			return HEVC_RPS_ERR_ALIGN;
	if (r->err)
		return HEVC_RPS_ERR_ALIGN;
	h->data_byte = r->pos >> 3;
	return HEVC_RPS_OK;
}

/* --- long-term entry reconstruction --- */

/*
 * Map the long-term entries of the original header onto the long-term
 * references VA-API gave us, keeping their order (it is the order of
 * RefPicSetLtCurr). An LtFoll entry may name a picture that is not in the
 * DPB (8.3.2 allows "no reference picture" there); such entries do not
 * affect the decoding of this picture and are dropped. Returns the number
 * of entries to write, or a negative error.
 */
static int rebuild_lt(const struct hevc_rps_ps *ps, const struct hevc_rps_set *rps,
		      const struct hdr *h, uint32_t lsb[], bool used[],
		      bool msb_present[], uint32_t new_cycle[])
{
	uint32_t max_lsb = 1u << ps->log2_max_poc_lsb;
	unsigned int n = h->num_lt_sps + h->num_lt_pics;
	int match[HEVC_RPS_MAX_LT];
	bool taken[HEVC_RPS_MAX_LT] = { 0 };
	int64_t cycle[HEVC_RPS_MAX_LT];
	int64_t prev = 0;
	int64_t poc_msb_base = (int64_t)rps->poc - h->poc_lsb;
	unsigned int out = 0, matched = 0;

	/* DeltaPocMsbCycleLt (7-52) of the original header. */
	for (unsigned int i = 0; i < n; i++) {
		int64_t v = h->lt[i].msb_present ? h->lt[i].msb_cycle : 0;

		cycle[i] = (i == 0 || i == h->num_lt_sps) ? v : v + cycle[i - 1];
		match[i] = -1;
	}

	/* Entries whose LSBs are coded in the slice header first, then those from the SPS. */
	for (int pass = 0; pass < 2; pass++) {
		for (unsigned int i = 0; i < n; i++) {
			const struct lt_orig *lt = &h->lt[i];
			int found = -1;

			if (lt->from_sps != (pass == 1))
				continue;
			for (unsigned int k = 0; k < rps->num_lt; k++) {
				int64_t poc = rps->lt[k].poc;

				if (taken[k])
					continue;
				if (lt->msb_present) {
					/* 8-5: pocLt = PicOrderCntVal - DeltaPocMsbCycleLt * MaxLsb - (lsb - PocLsbLt) */
					int64_t msb = poc_msb_base - cycle[i] * max_lsb;

					if (poc - (poc & (max_lsb - 1)) != msb)
						continue;
				}
				if (!lt->from_sps &&
				    (uint32_t)(poc & (max_lsb - 1)) != lt->poc_lsb)
					continue;
				if (!lt->from_sps && rps->lt[k].used != lt->used)
					continue;
				if (found >= 0)
					return HEVC_RPS_ERR_LT_MATCH;	/* ambiguous */
				found = (int)k;
			}
			if (found < 0) {
				/* only a picture not used by the current one can be absent */
				if (!lt->from_sps && lt->used)
					return HEVC_RPS_ERR_LT_MATCH;
				continue;
			}
			taken[found] = true;
			match[i] = found;
			matched++;
		}
	}
	if (matched != rps->num_lt)
		return HEVC_RPS_ERR_LT_MATCH;

	for (unsigned int i = 0; i < n; i++) {
		const struct hevc_rps_lt_entry *ref;

		if (match[i] < 0)
			continue;
		ref = &rps->lt[match[i]];
		lsb[out] = (uint32_t)ref->poc & (max_lsb - 1);
		used[out] = ref->used;
		msb_present[out] = h->lt[i].msb_present;
		/* New header: the differential coding of 7-52 only restarts at i == 0. */
		if (h->lt[i].msb_present) {
			if (cycle[i] < prev)
				return HEVC_RPS_ERR_LT_ORDER;
			new_cycle[out] = (uint32_t)(cycle[i] - prev);
			prev = cycle[i];
		}
		out++;
	}
	return (int)out;
}

static int cmp_st(const void *a, const void *b)
{
	const struct hevc_rps_st_entry *x = a, *y = b;

	return (x->delta_poc > y->delta_poc) - (x->delta_poc < y->delta_poc);
}

int hevc_rps_rewrite_slice(const struct hevc_rps_ps *ps,
			   const struct hevc_rps_set *rps,
			   const uint8_t *nal, size_t nal_size,
			   uint8_t *out, size_t out_cap, size_t *out_size,
			   struct hevc_rps_result *res)
{
	uint8_t *rbsp = NULL, *nrbsp = NULL;
	struct hdr h;
	struct rd r;
	struct wr w;
	struct hevc_rps_st_entry st[HEVC_RPS_MAX_ST];
	unsigned int num_neg = 0, num_pos = 0;
	uint32_t lt_lsb[HEVC_RPS_MAX_LT], lt_cycle[HEVC_RPS_MAX_LT] = { 0 };
	bool lt_used[HEVC_RPS_MAX_LT], lt_msb[HEVC_RPS_MAX_LT];
	int num_lt = 0;
	size_t rbsp_size, nrbsp_cap, nsize, st_start, lt_start;
	uint32_t mark;
	int ret;

	memset(res, 0, sizeof(*res));

	if (nal_size < 3 || rps->num_st > HEVC_RPS_MAX_ST || rps->num_lt > HEVC_RPS_MAX_LT)
		return HEVC_RPS_ERR_RANGE;

	rbsp = malloc(nal_size);
	if (!rbsp)
		return HEVC_RPS_ERR_SPACE;
	rbsp_size = unescape(nal, nal_size, rbsp);

	r = (struct rd){ .p = rbsp, .bits = rbsp_size * 8 };
	ret = parse_header(&r, ps, rps, &h);
	if (ret)
		goto out;

	res->nal_unit_type = h.nal_unit_type;
	res->temporal_id_plus1 = h.temporal_id_plus1;
	res->no_output_of_prior_pics = h.no_output_of_prior_pics;

	if (h.idr || h.dependent) {
		/* No RPS syntax in this slice segment header: pass it through. */
		if (nal_size > out_cap) {
			ret = HEVC_RPS_ERR_SPACE;
			goto out;
		}
		memcpy(out, nal, nal_size);
		*out_size = nal_size;
		res->header_rbsp_bytes = (uint32_t)h.data_byte;
		mark = (uint32_t)h.data_byte;
		/* Translate the RBSP data offset into the escaped NAL. */
		{
			unsigned int zeros = 0;
			size_t i, o = 0;

			for (i = 0; i < nal_size && o < h.data_byte; i++) {
				if (zeros >= 2 && nal[i] == 0x03) {
					zeros = 0;
					continue;
				}
				zeros = nal[i] ? 0 : zeros + 1;
				o++;
			}
			mark = (uint32_t)i;
		}
		res->data_byte_offset = mark;
		ret = HEVC_RPS_OK;
		goto out;
	}

	/* Short-term set, 7.4.8: S0 by decreasing POC, S1 by increasing POC. */
	memcpy(st, rps->st, rps->num_st * sizeof(st[0]));
	qsort(st, rps->num_st, sizeof(st[0]), cmp_st);
	for (unsigned int i = 0; i < rps->num_st; i++) {
		if (st[i].delta_poc < 0)
			num_neg++;
		else if (st[i].delta_poc > 0)
			num_pos++;
		else {
			ret = HEVC_RPS_ERR_RANGE;
			goto out;
		}
	}

	if (ps->long_term_ref_pics_present) {
		num_lt = rebuild_lt(ps, rps, &h, lt_lsb, lt_used, lt_msb, lt_cycle);
		if (num_lt < 0) {
			ret = num_lt;
			goto out;
		}
	} else if (rps->num_lt) {
		ret = HEVC_RPS_ERR_LT_MATCH;
		goto out;
	}

	nrbsp_cap = rbsp_size + 64 + (HEVC_RPS_MAX_ST + HEVC_RPS_MAX_LT) * 8;
	nrbsp = calloc(1, nrbsp_cap);
	if (!nrbsp) {
		ret = HEVC_RPS_ERR_SPACE;
		goto out;
	}
	w = (struct wr){ .p = nrbsp, .cap_bits = nrbsp_cap * 8 };

	wr_copy(&w, &r, 0, h.rps_start);
	wr_bit(&w, 0);					/* short_term_ref_pic_set_sps_flag */
	st_start = w.pos;
	/* st_ref_pic_set(0): no inter_ref_pic_set_prediction_flag for idx 0 */
	wr_ue(&w, num_neg);
	wr_ue(&w, num_pos);
	for (unsigned int i = 0; i < num_neg; i++) {
		/* S0 is st[num_neg - 1 .. 0] (closest first) */
		int32_t d = st[num_neg - 1 - i].delta_poc;
		int32_t prev = i ? st[num_neg - i].delta_poc : 0;

		wr_ue(&w, (uint32_t)(prev - d - 1));	/* delta_poc_s0_minus1 */
		wr_bit(&w, st[num_neg - 1 - i].used);	/* used_by_curr_pic_s0_flag */
	}
	for (unsigned int i = 0; i < num_pos; i++) {
		int32_t d = st[num_neg + i].delta_poc;
		int32_t prev = i ? st[num_neg + i - 1].delta_poc : 0;

		wr_ue(&w, (uint32_t)(d - prev - 1));	/* delta_poc_s1_minus1 */
		wr_bit(&w, st[num_neg + i].used);	/* used_by_curr_pic_s1_flag */
	}
	res->short_term_ref_pic_set_size = (uint32_t)(w.pos - st_start);

	lt_start = w.pos;
	if (ps->long_term_ref_pics_present) {
		/* num_long_term_sps is absent with num_long_term_ref_pics_sps == 0 */
		wr_ue(&w, (uint32_t)num_lt);		/* num_long_term_pics */
		for (int i = 0; i < num_lt; i++) {
			wr_u(&w, lt_lsb[i], ps->log2_max_poc_lsb);	/* poc_lsb_lt */
			wr_bit(&w, lt_used[i]);		/* used_by_curr_pic_lt_flag */
			wr_bit(&w, lt_msb[i]);		/* delta_poc_msb_present_flag */
			if (lt_msb[i])
				wr_ue(&w, lt_cycle[i]);	/* delta_poc_msb_cycle_lt */
		}
	}
	res->long_term_ref_pic_set_size = (uint32_t)(w.pos - lt_start);

	wr_copy(&w, &r, h.rps_end, h.hdr_end);
	wr_bit(&w, 1);					/* alignment_bit_equal_to_one */
	while (w.pos & 7)
		wr_bit(&w, 0);				/* alignment_bit_equal_to_zero */
	if (w.err) {
		ret = HEVC_RPS_ERR_SPACE;
		goto out;
	}
	res->header_rbsp_bytes = (uint32_t)(w.pos >> 3);

	nsize = w.pos >> 3;
	if (nsize + (rbsp_size - h.data_byte) > nrbsp_cap) {
		ret = HEVC_RPS_ERR_SPACE;
		goto out;
	}
	memcpy(nrbsp + nsize, rbsp + h.data_byte, rbsp_size - h.data_byte);
	nsize += rbsp_size - h.data_byte;

	mark = res->header_rbsp_bytes;
	*out_size = escape(nrbsp, nsize, out, out_cap, &mark);
	if (!*out_size) {
		ret = HEVC_RPS_ERR_SPACE;
		goto out;
	}
	res->data_byte_offset = mark;
	res->rewritten = true;
	ret = HEVC_RPS_OK;
out:
	free(rbsp);
	free(nrbsp);
	return ret;
}

const char *hevc_rps_strerror(int err)
{
	switch (err) {
	case HEVC_RPS_OK:		return "ok";
	case HEVC_RPS_ERR_PARSE:	return "slice segment header parse error";
	case HEVC_RPS_ERR_ALIGN:	return "slice segment header does not end in byte_alignment()";
	case HEVC_RPS_ERR_LT_MATCH:	return "long-term reference does not match the VA reference frames";
	case HEVC_RPS_ERR_LT_ORDER:	return "long-term MSB cycles cannot be re-coded";
	case HEVC_RPS_ERR_SPACE:	return "out of buffer space";
	case HEVC_RPS_ERR_RANGE:	return "reference picture set out of range";
	}
	return "unknown error";
}
