/*
 * Host test for the HEVC RPS normaliser (src/hevc_rps.c).
 *
 *   hevc_rps_test IN.hevc OUT.hevc
 *
 * Reads an Annex-B HEVC elementary stream and writes the stream a VDPU381
 * class decoder would be fed by the VA driver: every SPS gets
 * num_short_term_ref_pic_sets = 0 and num_long_term_ref_pics_sps = 0, and every
 * slice segment header is rewritten by hevc_rps_rewrite_slice() from the
 * picture's reference picture set, described the way VA-API does (POCs plus
 * Curr/Foll flags). This harness derives that set with its own SPS/PPS/slice
 * header parser and the POC/RPS decoding process of H.265 8.3.1/8.3.2.
 *
 * The output must decode to exactly the same pictures as the input with any
 * conforming software decoder (tests/hevc_rps_check.py does that with FFmpeg).
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hevc_rps.h"

/* ---------- bit reader / writer over RBSP ---------- */

struct br {
	const uint8_t *p;
	size_t bits, pos;
	bool err;
};

static uint32_t u1(struct br *b)
{
	if (b->pos >= b->bits) {
		b->err = true;
		return 0;
	}
	uint32_t v = (b->p[b->pos >> 3] >> (7 - (b->pos & 7))) & 1;
	b->pos++;
	return v;
}

static uint32_t un(struct br *b, unsigned n)
{
	uint32_t v = 0;
	while (n--)
		v = (v << 1) | u1(b);
	return v;
}

static uint32_t ue(struct br *b)
{
	unsigned z = 0;
	while (!u1(b)) {
		if (b->err || ++z > 31) {
			b->err = true;
			return 0;
		}
	}
	return (uint32_t)((1ull << z) - 1 + un(b, z));
}

static int32_t se(struct br *b)
{
	uint32_t v = ue(b);
	return (v & 1) ? (int32_t)((v + 1) / 2) : -(int32_t)(v / 2);
}

struct bw {
	uint8_t *p;
	size_t pos;
};

static void w1(struct bw *w, uint32_t v)
{
	if (v)
		w->p[w->pos >> 3] |= 0x80 >> (w->pos & 7);
	else
		w->p[w->pos >> 3] &= ~(0x80 >> (w->pos & 7));
	w->pos++;
}

static void wn(struct bw *w, uint32_t v, unsigned n)
{
	while (n--)
		w1(w, (v >> n) & 1);
}

static void wue(struct bw *w, uint32_t v)
{
	uint64_t x = (uint64_t)v + 1;
	unsigned len = 0;
	while ((x >> len) > 1)
		len++;
	wn(w, 0, len);
	wn(w, (uint32_t)x, len + 1);
}

static void wcopy(struct bw *w, const struct br *b, size_t from, size_t to)
{
	for (size_t i = from; i < to; i++)
		w1(w, (b->p[i >> 3] >> (7 - (i & 7))) & 1);
}

static size_t unescape(const uint8_t *s, size_t n, uint8_t *d)
{
	size_t o = 0;
	unsigned z = 0;
	for (size_t i = 0; i < n; i++) {
		if (z >= 2 && s[i] == 3) {
			z = 0;
			continue;
		}
		d[o++] = s[i];
		z = s[i] ? 0 : z + 1;
	}
	return o;
}

static size_t escape(const uint8_t *s, size_t n, uint8_t *d)
{
	size_t o = 0;
	unsigned z = 0;
	for (size_t i = 0; i < n; i++) {
		if (z >= 2 && s[i] <= 3) {
			d[o++] = 3;
			z = 0;
		}
		d[o++] = s[i];
		z = s[i] ? 0 : z + 1;
	}
	if (n && s[n - 1] == 0)
		d[o++] = 3;
	return o;
}

static unsigned clog2(uint32_t v)
{
	unsigned n = 0;
	while ((1ull << n) < v)
		n++;
	return n;
}

/* ---------- parameter sets ---------- */

struct strps {
	unsigned num_neg, num_pos;
	int32_t d0[16], d1[16];
	bool u0[16], u1[16];
};

struct sps {
	bool valid;
	unsigned chroma_format_idc;
	bool separate_colour_plane;
	uint32_t width, height;
	unsigned log2_max_poc_lsb;
	unsigned log2_ctb;
	unsigned num_st;
	struct strps st[65];
	bool lt_present;
	unsigned num_lt_sps;
	uint32_t lt_lsb[33];
	bool lt_used[33];
	bool temporal_mvp;
	bool sao;
};

struct pps {
	bool valid;
	unsigned sps_id;
	bool dependent_slice_segments_enabled, output_flag_present;
	unsigned num_extra_slice_header_bits;
	bool cabac_init_present;
	unsigned l0_default, l1_default;
	bool slice_chroma_qp_offsets_present, weighted_pred, weighted_bipred;
	bool tiles, entropy_sync, loop_filter_across_slices;
	bool deblocking_override_enabled, deblocking_disabled;
	bool lists_modification_present, slice_header_ext;
	bool chroma_qp_offset_list;
};

static struct sps g_sps[16];
static struct pps g_pps[64];

static void profile_tier_level(struct br *b, unsigned max_sub_layers_minus1)
{
	bool sub_profile[8], sub_level[8];

	un(b, 8);		/* profile space, tier, idc */
	un(b, 32);		/* compatibility flags */
	un(b, 4);		/* progressive, interlaced, non_packed, frame_only */
	un(b, 32);
	un(b, 12);		/* 43 constraint/reserved bits + general_inbld/reserved */
	un(b, 8);		/* general_level_idc */
	for (unsigned i = 0; i < max_sub_layers_minus1; i++) {
		sub_profile[i] = u1(b);
		sub_level[i] = u1(b);
	}
	if (max_sub_layers_minus1 > 0)
		for (unsigned i = max_sub_layers_minus1; i < 8; i++)
			un(b, 2);
	for (unsigned i = 0; i < max_sub_layers_minus1; i++) {
		if (sub_profile[i]) {
			un(b, 32);
			un(b, 32);
			un(b, 24);
		}
		if (sub_level[i])
			un(b, 8);
	}
}

static void scaling_list_data(struct br *b)
{
	for (int size = 0; size < 4; size++) {
		for (int m = 0; m < 6; m += (size == 3) ? 3 : 1) {
			if (!u1(b)) {
				ue(b);	/* pred_matrix_id_delta */
			} else {
				int n = 64 < (1 << (4 + (size << 1))) ? 64 : (1 << (4 + (size << 1)));
				if (size > 1)
					se(b);
				for (int i = 0; i < n; i++)
					se(b);
			}
		}
	}
}

/* 7.3.7 + 7.4.8; idx == num for the slice header case (with VA's st_rps_bits). */
static void st_ref_pic_set(struct br *b, unsigned idx, unsigned num,
			   struct strps *sets, struct strps *out)
{
	bool inter = idx ? u1(b) : false;

	memset(out, 0, sizeof(*out));
	if (inter) {
		unsigned delta_idx_minus1 = (idx == num) ? ue(b) : 0;
		unsigned sign = u1(b);
		unsigned abs_minus1 = ue(b);
		int ref = (int)idx - (int)(delta_idx_minus1 + 1);
		int delta_rps = (1 - 2 * (int)sign) * (int)(abs_minus1 + 1);
		struct strps *r;
		bool used[33], use_delta[33];
		unsigned nd, i;

		if (ref < 0) {
			b->err = true;
			return;
		}
		r = &sets[ref];
		nd = r->num_neg + r->num_pos;
		for (unsigned j = 0; j <= nd; j++) {
			used[j] = u1(b);
			use_delta[j] = used[j] ? true : u1(b);
		}
		/* 7-61 */
		i = 0;
		for (int j = (int)r->num_pos - 1; j >= 0; j--) {
			int d = r->d1[j] + delta_rps;
			if (d < 0 && use_delta[r->num_neg + j]) {
				out->d0[i] = d;
				out->u0[i++] = used[r->num_neg + j];
			}
		}
		if (delta_rps < 0 && use_delta[nd]) {
			out->d0[i] = delta_rps;
			out->u0[i++] = used[nd];
		}
		for (unsigned j = 0; j < r->num_neg; j++) {
			int d = r->d0[j] + delta_rps;
			if (d < 0 && use_delta[j]) {
				out->d0[i] = d;
				out->u0[i++] = used[j];
			}
		}
		out->num_neg = i;
		/* 7-62 */
		i = 0;
		for (int j = (int)r->num_neg - 1; j >= 0; j--) {
			int d = r->d0[j] + delta_rps;
			if (d > 0 && use_delta[j]) {
				out->d1[i] = d;
				out->u1[i++] = used[j];
			}
		}
		if (delta_rps > 0 && use_delta[nd]) {
			out->d1[i] = delta_rps;
			out->u1[i++] = used[nd];
		}
		for (unsigned j = 0; j < r->num_pos; j++) {
			int d = r->d1[j] + delta_rps;
			if (d > 0 && use_delta[r->num_neg + j]) {
				out->d1[i] = d;
				out->u1[i++] = used[r->num_neg + j];
			}
		}
		out->num_pos = i;
	} else {
		out->num_neg = ue(b);
		out->num_pos = ue(b);
		if (out->num_neg > 16 || out->num_pos > 16) {
			b->err = true;
			return;
		}
		int prev = 0;
		for (unsigned i = 0; i < out->num_neg; i++) {
			prev -= (int)ue(b) + 1;
			out->d0[i] = prev;
			out->u0[i] = u1(b);
		}
		prev = 0;
		for (unsigned i = 0; i < out->num_pos; i++) {
			prev += (int)ue(b) + 1;
			out->d1[i] = prev;
			out->u1[i] = u1(b);
		}
	}
}

/*
 * Parse an SPS and write a copy with no SPS RPS candidates into out (RBSP).
 * Returns the new RBSP size, 0 on error.
 */
static size_t parse_sps(const uint8_t *rbsp, size_t n, uint8_t *out)
{
	struct br b = { rbsp, n * 8, 16, false };
	struct bw w = { out, 0 };
	unsigned max_sub, id;
	struct sps s;
	size_t st_pos, st_end, lt_pos, lt_end, stop;

	memset(&s, 0, sizeof(s));
	un(&b, 4);			/* vps id */
	max_sub = un(&b, 3);
	u1(&b);
	profile_tier_level(&b, max_sub);
	id = ue(&b);
	if (id > 15)
		return 0;
	s.chroma_format_idc = ue(&b);
	if (s.chroma_format_idc == 3)
		s.separate_colour_plane = u1(&b);
	s.width = ue(&b);
	s.height = ue(&b);
	if (u1(&b)) {
		ue(&b); ue(&b); ue(&b); ue(&b);
	}
	ue(&b);
	ue(&b);				/* bit depths */
	s.log2_max_poc_lsb = ue(&b) + 4;
	{
		bool all = u1(&b);
		for (unsigned i = all ? 0 : max_sub; i <= max_sub; i++) {
			ue(&b); ue(&b); ue(&b);
		}
	}
	unsigned log2_min_cb = ue(&b) + 3;
	s.log2_ctb = log2_min_cb + ue(&b);
	ue(&b); ue(&b); ue(&b); ue(&b);
	if (u1(&b) && u1(&b))		/* scaling_list_enabled, sps_scaling_list_data_present */
		scaling_list_data(&b);
	u1(&b);				/* amp */
	s.sao = u1(&b);
	if (u1(&b)) {			/* pcm */
		un(&b, 8);
		ue(&b);
		ue(&b);
		u1(&b);
	}
	st_pos = b.pos;
	s.num_st = ue(&b);
	if (s.num_st > 64)
		return 0;
	for (unsigned i = 0; i < s.num_st; i++)
		st_ref_pic_set(&b, i, s.num_st, s.st, &s.st[i]);
	st_end = b.pos;
	s.lt_present = u1(&b);
	lt_pos = b.pos;
	if (s.lt_present) {
		s.num_lt_sps = ue(&b);
		if (s.num_lt_sps > 32)
			return 0;
		for (unsigned i = 0; i < s.num_lt_sps; i++) {
			s.lt_lsb[i] = un(&b, s.log2_max_poc_lsb);
			s.lt_used[i] = u1(&b);
		}
	}
	lt_end = b.pos;
	s.temporal_mvp = u1(&b);
	if (b.err)
		return 0;
	s.valid = true;
	g_sps[id] = s;

	/* rbsp_trailing_bits: the last 1 bit is the stop bit */
	stop = n * 8;
	while (stop > lt_end && !((rbsp[(stop - 1) >> 3] >> (7 - ((stop - 1) & 7))) & 1))
		stop--;
	if (stop <= lt_end)
		return 0;
	stop--;

	memset(out, 0, n + 16);
	wcopy(&w, &b, 0, st_pos);
	wue(&w, 0);			/* num_short_term_ref_pic_sets */
	wcopy(&w, &b, st_end, lt_pos);	/* long_term_ref_pics_present_flag */
	if (s.lt_present)
		wue(&w, 0);		/* num_long_term_ref_pics_sps */
	wcopy(&w, &b, lt_end, stop);
	w1(&w, 1);
	while (w.pos & 7)
		w1(&w, 0);
	return w.pos >> 3;
}

static bool parse_pps(const uint8_t *rbsp, size_t n)
{
	struct br b = { rbsp, n * 8, 16, false };
	struct pps p;
	unsigned id;
	bool transform_skip;

	memset(&p, 0, sizeof(p));
	id = ue(&b);
	if (id > 63)
		return false;
	p.sps_id = ue(&b);
	p.dependent_slice_segments_enabled = u1(&b);
	p.output_flag_present = u1(&b);
	p.num_extra_slice_header_bits = un(&b, 3);
	u1(&b);				/* sign data hiding */
	p.cabac_init_present = u1(&b);
	p.l0_default = ue(&b);
	p.l1_default = ue(&b);
	se(&b);
	u1(&b);
	transform_skip = u1(&b);
	if (u1(&b))
		ue(&b);
	se(&b);
	se(&b);
	p.slice_chroma_qp_offsets_present = u1(&b);
	p.weighted_pred = u1(&b);
	p.weighted_bipred = u1(&b);
	u1(&b);				/* transquant bypass */
	p.tiles = u1(&b);
	p.entropy_sync = u1(&b);
	if (p.tiles) {
		unsigned cols = ue(&b), rows = ue(&b);
		if (!u1(&b)) {
			for (unsigned i = 0; i < cols; i++)
				ue(&b);
			for (unsigned i = 0; i < rows; i++)
				ue(&b);
		}
		u1(&b);
	}
	p.loop_filter_across_slices = u1(&b);
	if (u1(&b)) {			/* deblocking_filter_control_present */
		p.deblocking_override_enabled = u1(&b);
		p.deblocking_disabled = u1(&b);
		if (!p.deblocking_disabled) {
			se(&b);
			se(&b);
		}
	}
	if (u1(&b))
		scaling_list_data(&b);
	p.lists_modification_present = u1(&b);
	ue(&b);
	p.slice_header_ext = u1(&b);
	if (u1(&b)) {			/* pps_extension_present */
		bool range = u1(&b);
		un(&b, 7);
		if (range) {
			if (transform_skip)
				ue(&b);
			u1(&b);		/* cross_component_prediction */
			p.chroma_qp_offset_list = u1(&b);
		}
	}
	if (b.err)
		return false;
	p.valid = true;
	g_pps[id] = p;
	return true;
}

/* ---------- POC and DPB tracking (8.3.1, 8.3.2) ---------- */

struct dpbpic {
	int32_t poc;
	bool ref;	/* "used for reference" */
	bool lt;
};

static struct dpbpic g_dpb[64];
static unsigned g_ndpb;
static int32_t g_prev_tid0_poc;
static bool g_first = true, g_after_eos;
static bool g_skip_rasl;

static void fill_ps(struct hevc_rps_ps *ps, const struct sps *s, const struct pps *p,
		    uint32_t st_rps_bits)
{
	uint32_t ctb = 1u << s->log2_ctb;

	memset(ps, 0, sizeof(*ps));
	ps->pic_size_in_ctbs_y = ((s->width + ctb - 1) >> s->log2_ctb) *
				 ((s->height + ctb - 1) >> s->log2_ctb);
	ps->log2_max_poc_lsb = s->log2_max_poc_lsb;
	ps->num_extra_slice_header_bits = p->num_extra_slice_header_bits;
	ps->chroma_array_type = s->separate_colour_plane ? 0 : s->chroma_format_idc;
	ps->num_short_term_ref_pic_sets = s->num_st;
	ps->num_long_term_ref_pics_sps = s->num_lt_sps;
	ps->num_ref_idx_l0_default_active_minus1 = p->l0_default;
	ps->num_ref_idx_l1_default_active_minus1 = p->l1_default;
	ps->st_rps_bits = st_rps_bits;
	ps->st_rps_bits_valid = true;
	ps->dependent_slice_segments_enabled = p->dependent_slice_segments_enabled;
	ps->output_flag_present = p->output_flag_present;
	ps->separate_colour_plane = s->separate_colour_plane;
	ps->long_term_ref_pics_present = s->lt_present;
	ps->sps_temporal_mvp_enabled = s->temporal_mvp;
	ps->sample_adaptive_offset_enabled = s->sao;
	ps->lists_modification_present = p->lists_modification_present;
	ps->cabac_init_present = p->cabac_init_present;
	ps->weighted_pred = p->weighted_pred;
	ps->weighted_bipred = p->weighted_bipred;
	ps->pps_slice_chroma_qp_offsets_present = p->slice_chroma_qp_offsets_present;
	ps->chroma_qp_offset_list_enabled = p->chroma_qp_offset_list;
	ps->deblocking_filter_override_enabled = p->deblocking_override_enabled;
	ps->pps_deblocking_filter_disabled = p->deblocking_disabled;
	ps->pps_loop_filter_across_slices_enabled = p->loop_filter_across_slices;
	ps->tiles_enabled = p->tiles;
	ps->entropy_coding_sync_enabled = p->entropy_sync;
	ps->slice_segment_header_extension_present = p->slice_header_ext;
}

/* Current picture state, set by the first slice segment of a picture. */
static struct hevc_rps_set g_rps;
static struct hevc_rps_ps g_ps_cur;
static bool g_cur_skip;

static unsigned g_stats_rewritten, g_stats_copied, g_stats_skipped, g_stats_failed;

/*
 * Parse the first slice segment of a picture: derive POC and the RPS, update
 * the DPB marking, and fill g_rps the way a VA client would describe it.
 * Returns false if the picture is to be skipped (RASL after a CRA start).
 */
static bool start_picture(const uint8_t *rbsp, size_t n, unsigned nut, unsigned tid)
{
	struct br b = { rbsp, n * 8, 16, false };
	bool irap = nut >= 16 && nut <= 23;
	bool idr = nut == 19 || nut == 20;
	bool no_rasl_output = idr || (nut >= 16 && nut <= 18) || g_first || g_after_eos;
	const struct pps *p;
	const struct sps *s;
	unsigned slice_type;
	uint32_t poc_lsb = 0, st_rps_bits = 0;
	int32_t poc;
	struct strps cur = { 0 };
	unsigned num_lt = 0;
	int32_t lt_poc[33];
	bool lt_used[33], lt_full[33];

	u1(&b);					/* first_slice_segment_in_pic_flag */
	if (irap)
		u1(&b);
	p = &g_pps[ue(&b) & 63];
	if (!p->valid || !g_sps[p->sps_id & 15].valid)
		return false;
	s = &g_sps[p->sps_id & 15];

	if (nut == 8 || nut == 9) {		/* RASL */
		if (g_skip_rasl)
			return false;
	}
	if (irap)
		g_skip_rasl = no_rasl_output;

	un(&b, p->num_extra_slice_header_bits);
	slice_type = ue(&b);
	(void)slice_type;
	if (p->output_flag_present)
		u1(&b);
	if (s->separate_colour_plane)
		un(&b, 2);

	if (!idr) {
		poc_lsb = un(&b, s->log2_max_poc_lsb);
		if (!u1(&b)) {
			size_t start = b.pos;
			st_ref_pic_set(&b, s->num_st, s->num_st, (struct strps *)s->st, &cur);
			st_rps_bits = (uint32_t)(b.pos - start);
		} else {
			unsigned idx = s->num_st > 1 ? un(&b, clog2(s->num_st)) : 0;
			cur = s->st[idx];
		}
		if (s->lt_present) {
			unsigned nsps = s->num_lt_sps ? ue(&b) : 0;
			unsigned npics = ue(&b);
			int32_t cyc = 0;

			num_lt = nsps + npics;
			if (num_lt > 32)
				return false;
			for (unsigned i = 0; i < num_lt; i++) {
				uint32_t lsb;
				bool used, msb;

				if (i < nsps) {
					unsigned k = s->num_lt_sps > 1 ? un(&b, clog2(s->num_lt_sps)) : 0;
					lsb = s->lt_lsb[k];
					used = s->lt_used[k];
				} else {
					lsb = un(&b, s->log2_max_poc_lsb);
					used = u1(&b);
				}
				msb = u1(&b);
				{
					uint32_t v = msb ? ue(&b) : 0;
					cyc = (i == 0 || i == nsps) ? (int32_t)v : (int32_t)v + cyc;
				}
				lt_used[i] = used;
				lt_full[i] = msb;
				lt_poc[i] = (int32_t)lsb;
				if (msb)
					lt_poc[i] = (int32_t)lsb - cyc * (1 << s->log2_max_poc_lsb);
			}
		}
	}
	if (b.err)
		return false;

	/* 8.3.1 */
	{
		int32_t max = 1 << s->log2_max_poc_lsb;
		int32_t prev_lsb = g_prev_tid0_poc & (max - 1);
		int32_t prev_msb = g_prev_tid0_poc - prev_lsb;
		int32_t msb;

		if (irap && no_rasl_output)
			msb = 0;
		else if ((int32_t)poc_lsb < prev_lsb && prev_lsb - (int32_t)poc_lsb >= max / 2)
			msb = prev_msb + max;
		else if ((int32_t)poc_lsb > prev_lsb && (int32_t)poc_lsb - prev_lsb > max / 2)
			msb = prev_msb - max;
		else
			msb = prev_msb;
		poc = msb + (int32_t)poc_lsb;
		/* full LT POCs: PicOrderCntVal - DeltaPocMsbCycleLt * MaxLsb - (slice lsb - lsb) */
		for (unsigned i = 0; i < num_lt; i++)
			if (lt_full[i])
				lt_poc[i] = lt_poc[i] + poc - (int32_t)poc_lsb;
		/* prevTid0Pic: TemporalId 0 and not RASL, RADL or a sub-layer non-reference picture */
		if (tid == 0 && !(nut <= 14 && (nut % 2) == 0) && !(nut >= 6 && nut <= 9))
			g_prev_tid0_poc = poc;
	}
	g_first = false;
	g_after_eos = false;

	if (irap && no_rasl_output) {
		for (unsigned i = 0; i < g_ndpb; i++)
			g_dpb[i].ref = false;
	}

	/* 8.3.2: build the set, mark the DPB */
	memset(&g_rps, 0, sizeof(g_rps));
	g_rps.poc = poc;
	{
		bool keep[64] = { 0 };
		int32_t max = 1 << s->log2_max_poc_lsb;

		for (unsigned i = 0; i < num_lt; i++) {
			int found = -1;
			for (unsigned k = 0; k < g_ndpb; k++) {
				if (!g_dpb[k].ref)
					continue;
				if (lt_full[i] ? g_dpb[k].poc == lt_poc[i] :
				    (g_dpb[k].poc & (max - 1)) == lt_poc[i]) {
					found = (int)k;
					break;
				}
			}
			if (found < 0 && !lt_used[i])
				continue;	/* LtFoll may be "no reference picture" */
			if (found < 0) {
				fprintf(stderr, "poc %d: missing LT ref %d (full %d) lsb %u dpb:", poc, lt_poc[i], lt_full[i], poc_lsb);
				for (unsigned k = 0; k < g_ndpb; k++) fprintf(stderr, " %d%s%s", g_dpb[k].poc, g_dpb[k].lt ? "L" : "", g_dpb[k].ref ? "" : "x");
				fprintf(stderr, "\n");
				return false;
			}
			keep[found] = true;
			g_rps.lt[g_rps.num_lt].poc = g_dpb[found].poc;
			g_rps.lt[g_rps.num_lt++].used = lt_used[i];
			if (lt_used[i])
				g_rps.num_pic_total_curr++;
		}
		for (unsigned i = 0; i < cur.num_neg + cur.num_pos; i++) {
			int32_t d = i < cur.num_neg ? cur.d0[i] : cur.d1[i - cur.num_neg];
			bool used = i < cur.num_neg ? cur.u0[i] : cur.u1[i - cur.num_neg];
			int found = -1;

			for (unsigned k = 0; k < g_ndpb; k++)
				if (g_dpb[k].ref && !g_dpb[k].lt && !keep[k] &&
				    g_dpb[k].poc == poc + d)
					found = (int)k;
			if (found < 0 && !used)
				continue;	/* StFoll may be "no reference picture" */
			if (found < 0) {
				fprintf(stderr, "poc %d: missing ST ref %d\n", poc, poc + d);
				return false;
			}
			keep[found] = true;
			g_rps.st[g_rps.num_st].delta_poc = d;
			g_rps.st[g_rps.num_st++].used = used;
			if (used)
				g_rps.num_pic_total_curr++;
		}
		/* LT marking, then drop everything not in the RPS */
		for (unsigned k = 0; k < g_ndpb; k++) {
			for (unsigned i = 0; i < num_lt; i++)
				if (keep[k] && g_dpb[k].poc == g_rps.lt[i].poc)
					g_dpb[k].lt = true;
			if (!keep[k])
				g_dpb[k].ref = false;
		}
		/* compact */
		unsigned o = 0;
		for (unsigned k = 0; k < g_ndpb; k++)
			if (g_dpb[k].ref)
				g_dpb[o++] = g_dpb[k];
		g_ndpb = o;
	}

	fill_ps(&g_ps_cur, s, p, st_rps_bits);
	/* the current picture becomes a short-term reference after decoding */
	if (g_ndpb < 64)
		g_dpb[g_ndpb++] = (struct dpbpic){ .poc = poc, .ref = true, .lt = false };
	return true;
}

/* ---------- Annex-B I/O ---------- */

static void put_nal(FILE *f, const uint8_t *nal, size_t n)
{
	static const uint8_t sc[4] = { 0, 0, 0, 1 };
	fwrite(sc, 1, 4, f);
	fwrite(nal, 1, n, f);
}

int main(int argc, char **argv)
{
	FILE *in, *out;
	uint8_t *buf, *rbsp, *tmp, *tmp2;
	size_t size, cap;

	if (argc != 3) {
		fprintf(stderr, "usage: %s IN.hevc OUT.hevc\n", argv[0]);
		return 2;
	}
	in = fopen(argv[1], "rb");
	out = fopen(argv[2], "wb");
	if (!in || !out) {
		perror("open");
		return 2;
	}
	fseek(in, 0, SEEK_END);
	size = (size_t)ftell(in);
	fseek(in, 0, SEEK_SET);
	buf = malloc(size + 4);
	if (!buf || fread(buf, 1, size, in) != size)
		return 2;
	cap = size + 1024;
	rbsp = malloc(cap);
	tmp = malloc(cap * 2);
	tmp2 = malloc(cap * 2);

	size_t i = 0;
	while (i + 3 <= size) {
		size_t s, e;

		/* find start code */
		while (i + 3 <= size && !(buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1))
			i++;
		if (i + 3 > size)
			break;
		s = i + 3;
		e = s;
		while (e + 3 <= size && !(buf[e] == 0 && buf[e + 1] == 0 && buf[e + 2] == 1))
			e++;
		if (e + 3 > size)
			e = size;
		/* trailing_zero_8bits / the zero_byte of a 4-byte start code */
		while (e > s && buf[e - 1] == 0)
			e--;
		i = e;

		const uint8_t *nal = buf + s;
		size_t n = e - s;
		if (n < 2)
			continue;
		unsigned nut = (nal[0] >> 1) & 0x3f;
		unsigned tid = (nal[1] & 7) - 1;
		size_t rn = unescape(nal, n, rbsp);

		if (nut == 33) {	/* SPS */
			size_t nn = parse_sps(rbsp, rn, tmp);
			if (!nn) {
				fprintf(stderr, "SPS parse failed\n");
				return 1;
			}
			memcpy(tmp, rbsp, 2);	/* NAL header is unchanged */
			{
				/* parse_sps wrote the whole RBSP starting at bit 0 (header copied) */
				size_t en = escape(tmp, nn, tmp2);
				put_nal(out, tmp2, en);
			}
			continue;
		}
		if (nut == 34) {
			if (!parse_pps(rbsp, rn)) {
				fprintf(stderr, "PPS parse failed\n");
				return 1;
			}
			put_nal(out, nal, n);
			continue;
		}
		if (nut == 36 || nut == 37)	/* EOS, EOB */
			g_after_eos = true;
		if (nut > 31) {
			put_nal(out, nal, n);
			continue;
		}
		if (nut > 21 && nut <= 31) {	/* reserved VCL */
			put_nal(out, nal, n);
			continue;
		}

		bool first = (rbsp[2] & 0x80) != 0;
		if (first)
			g_cur_skip = !start_picture(rbsp, rn, nut, tid);
		if (g_cur_skip) {
			g_stats_skipped++;
			put_nal(out, nal, n);
			continue;
		}

		struct hevc_rps_result res;
		size_t on = 0;
		int ret = hevc_rps_rewrite_slice(&g_ps_cur, &g_rps, nal, n, tmp, cap * 2, &on, &res);
		if (ret) {
			fprintf(stderr, "poc %d nut %u: rewrite failed: %s\n", g_rps.poc, nut,
				hevc_rps_strerror(ret));
			g_stats_failed++;
			put_nal(out, nal, n);
			continue;
		}
		if (res.rewritten)
			g_stats_rewritten++;
		else
			g_stats_copied++;
		put_nal(out, tmp, on);
	}
	fclose(out);
	printf("rewritten %u copied %u skipped %u failed %u\n", g_stats_rewritten,
	       g_stats_copied, g_stats_skipped, g_stats_failed);
	return g_stats_failed ? 1 : 0;
}
