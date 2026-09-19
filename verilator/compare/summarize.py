#!/usr/bin/env python3
"""Summarize validate_cart.sh results: summarize.py OUTROOT

Writes OUTROOT/summary.json and OUTROOT/summary.csv and prints a table.
"""
import csv
import json
import os
import sys

EARLY = [120, 400, 740]   # before any input: should match ColEm closely
LATE = [1000, 1300]       # after keypad 1 / fire / stick input


def parse(path):
    r = {"shots": {}, "done": False}
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("cart="):
                r["cart"] = line[len("cart="):]
                continue
            if line == "done":
                r["done"] = True
                continue
            kv = dict(t.split("=", 1) for t in line.split() if "=" in t)
            if "shot" in kv:
                r["shots"][int(kv["shot"])] = kv
            else:
                r.update(kv)
    wide = os.path.join(os.path.dirname(path), "rescore.txt")
    if os.path.isfile(wide):
        r["rescore"] = {}
        with open(wide) as f:
            for line in f:
                kv = dict(t.split("=", 1) for t in line.split() if "=" in t)
                if "shot" in kv:
                    r["rescore"][int(kv["shot"])] = kv
    return r


def classify(r):
    reasons = []
    shots = r["shots"]
    missing = [s for s in EARLY + LATE if s not in shots or "match" not in shots[s]]
    if not r["done"] or r.get("core_exit") != "0" or missing:
        return "FAIL", ["core run failed or shots missing %s" % missing]
    num = lambda s, k: float(shots[s][k])

    if all(int(shots[s]["ref_colours"]) == 1 for s in EARLY + LATE):
        return "FAIL", ["screen is a single colour in every shot"]
    if any(num(s, "nocart_match") >= 0.995 for s in (400, 740)):
        return "FAIL", ["shows the BIOS no-cartridge screen"]

    for s in EARLY:
        if num(s, "match") < 0.98 or num(s, "fgmatch") < 0.90:
            reasons.append("frame %d differs from ColEm (match %.3f, fg %.3f)" % (s, num(s, "match"), num(s, "fgmatch")))
    for s in LATE:
        if num(s, "fgmatch") < 0.50:
            reasons.append("frame %d gameplay differs from ColEm (match %.3f, fg %.3f)" % (s, num(s, "match"), num(s, "fgmatch")))
    if float(r.get("still_match", "0")) >= 0.9999 and int(shots[1300]["ref_colours"]) <= 1:
        reasons.append("blank and unchanged at the end")
    if not reasons:
        return "PASS", []

    # rescore.sh searched +/-60 ColEm frames: the same screens a few frames apart is timing drift
    wide = r.get("rescore")
    if wide and all(s in wide for s in EARLY + LATE):
        early_ok = all(float(wide[s]["match"]) >= 0.99 and float(wide[s]["fgmatch"]) >= 0.90 for s in EARLY)
        late_ok = all(float(wide[s]["fgmatch"]) >= 0.60 for s in LATE)
        if early_ok and late_ok:
            offsets = sorted({int(wide[s]["offset"]) for s in EARLY + LATE})
            return "DRIFT", ["same screens as ColEm within %+d..%+d frames" % (offsets[0], offsets[-1])]
        for s in EARLY + LATE:
            reasons.append("best within ±60 frames at %d: match %.3f, fg %.3f (offset %+d)"
                           % (s, float(wide[s]["match"]), float(wide[s]["fgmatch"]), int(wide[s]["offset"])))
    return "REVIEW", reasons


def main():
    root = sys.argv[1]
    rows = []
    for d in sorted(os.listdir(root)):
        p = os.path.join(root, d, "result.txt")
        if not os.path.isfile(p):
            continue
        r = parse(p)
        status, reasons = classify(r)
        row = {
            "dir": d,
            "cart": os.path.basename(r.get("cart", d)),
            "size": int(r.get("size", 0)),
            "header": r.get("header", ""),
            "status": status,
            "reasons": reasons,
            "core_seconds": int(r.get("core_seconds", 0)),
            "fps": r.get("fps", ""),
            "still_match": float(r.get("still_match", "nan")),
            "shots": {
                s: {k: r["shots"][s].get(k) for k in ("match", "fgmatch", "ref_colours", "nocart_match")}
                for s in EARLY + LATE if s in r["shots"]
            },
            "strip": os.path.join(d, "strip.png") if os.path.isfile(os.path.join(root, d, "strip.png")) else None,
        }
        rows.append(row)

    with open(os.path.join(root, "summary.json"), "w") as f:
        json.dump(rows, f, indent=1)
    with open(os.path.join(root, "summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["cart", "status", "header", "size"] + ["match_%d" % s for s in EARLY + LATE] +
                   ["fg_%d" % s for s in EARLY + LATE] + ["reasons"])
        for row in rows:
            sh = row["shots"]
            w.writerow([row["cart"], row["status"], row["header"], row["size"]] +
                       [sh.get(s, {}).get("match") for s in EARLY + LATE] +
                       [sh.get(s, {}).get("fgmatch") for s in EARLY + LATE] + ["; ".join(row["reasons"])])

    counts = {}
    for row in rows:
        counts[row["status"]] = counts.get(row["status"], 0) + 1
    print("carts=%d %s" % (len(rows), " ".join("%s=%d" % kv for kv in sorted(counts.items()))))
    for row in rows:
        if row["status"] != "PASS":
            print("%-6s %s: %s" % (row["status"], row["cart"], "; ".join(row["reasons"])))


if __name__ == "__main__":
    main()
