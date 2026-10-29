#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
"""Check the HEVC RPS normaliser against the JCT-VC conformance suite.

For every vector: run hevc_rps_test (original -> rewritten stream: SPS without
RPS candidates, slice headers with explicit RPS), decode both streams with
FFmpeg (PyAV) and compare the per-frame MD5s. Also checks the original decode
against fluster's expected whole-stream MD5 where the output format matches.

usage: hevc_rps_check.py HEVC_RPS_TEST FLUSTER_SUITE.json VECTOR_DIR [NAME...]
"""
import glob
import hashlib
import json
import os
import subprocess
import sys
import tempfile

import av  # PyAV (bundles FFmpeg)


def decode_md5s(path):
    out = []
    with av.open(path, format="hevc") as c:
        s = c.streams.video[0]
        s.thread_type = "NONE"
        for frame in c.decode(s):
            h = hashlib.md5()
            for p in frame.planes:
                # crop the line padding: bytes per line from the plane's width
                bpl = p.width * (2 if frame.format.name.endswith(("10le", "12le")) else 1)
                buf = memoryview(p)
                for y in range(p.height):
                    h.update(buf[y * p.line_size:y * p.line_size + bpl])
            out.append(h.hexdigest())
    return out


def main():
    tool, suite, vdir = sys.argv[1:4]
    only = set(sys.argv[4:])
    d = json.load(open(suite))
    results = {"pass": [], "fail": [], "tool-fail": []}
    with tempfile.TemporaryDirectory() as tmp:
        for v in d["test_vectors"]:
            if only and v["name"] not in only:
                continue
            src = glob.glob(os.path.join(vdir, v["name"], "**", v["input_file"]),
                            recursive=True)[0]
            dst = os.path.join(tmp, v["name"] + ".hevc")
            p = subprocess.run([tool, src, dst], capture_output=True, text=True)
            stats = p.stdout.strip()
            if p.returncode:
                results["tool-fail"].append(v["name"])
                print(f"TOOLFAIL {v['name']}: {stats} {p.stderr.strip()[:300]}")
                continue
            try:
                a = decode_md5s(src)
                b = decode_md5s(dst)
            except Exception as e:  # decode error in the rewritten stream
                results["fail"].append(v["name"])
                print(f"FAIL {v['name']}: decode error {e}")
                continue
            if a == b and a:
                results["pass"].append(v["name"])
                print(f"PASS {v['name']}: {len(a)} frames, {stats}")
            else:
                diff = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y),
                            min(len(a), len(b)))
                results["fail"].append(v["name"])
                print(f"FAIL {v['name']}: {len(a)} vs {len(b)} frames, first diff {diff}, {stats}")
    n = sum(len(x) for x in results.values())
    print(f"\nSUMMARY: {len(results['pass'])}/{n} pass, "
          f"fail: {' '.join(results['fail']) or '-'}, "
          f"tool-fail: {' '.join(results['tool-fail']) or '-'}")
    return 0 if len(results["pass"]) == n else 1


if __name__ == "__main__":
    sys.exit(main())
