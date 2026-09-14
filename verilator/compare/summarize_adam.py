#!/usr/bin/env python3
"""Summarize validate_adam.sh results: summarize_adam.py OUTROOT

Classifies each scenario and writes OUTROOT/summary.json.
  MATCH    core and ColEm end on the same screen (final foreground match >= 0.98)
  CLOSE    final foreground match >= 0.85
  DIFFERS  lower than that
  FAIL     a run crashed, or no frames could be aligned
"""
import json
import os
import sys


def parse_result(path):
    r = {"notes": [], "done": False}
    for line in open(path):
        line = line.rstrip("\n")
        if line == "done":
            r["done"] = True
        elif line.startswith("args="):
            r["args"] = line[len("args="):].strip()
        elif line.startswith("note="):
            r["notes"].append(line[len("note="):])
        else:
            r.update(dict(t.split("=", 1) for t in line.split() if "=" in t))
    return r


def parse_align(path):
    rows = []
    if os.path.isfile(path):
        for line in open(path):
            kv = dict(t.split("=", 1) for t in line.split())
            rows.append({"core": int(kv["core"]), "best": int(kv["best"]), "match": float(kv["match"]),
                         "fg": float(kv["fgmatch"])})
    return rows


def main():
    root = sys.argv[1]
    out = []
    for d in sorted(os.listdir(root)):
        rp = os.path.join(root, d, "result.txt")
        if not os.path.isfile(rp):
            continue
        r = parse_result(rp)
        rows = parse_align(os.path.join(root, d, "align.txt"))
        unmatched = [x["core"] for x in rows if x["fg"] < 0.90]
        if not r["done"] or r.get("core_exit") != "0" or r.get("colem_exit") != "0" or not rows:
            status = "FAIL"
        else:
            fg = float(r["final_fg"])
            status = "MATCH" if fg >= 0.98 else "CLOSE" if fg >= 0.85 else "DIFFERS"
        out.append({
            "name": r.get("name", d), "dir": d, "status": status, "frames": int(r.get("frames", 0)),
            "core_fps": r.get("fps", ""), "final_match": r.get("final_match"), "final_fg": r.get("final_fg"),
            "final_offset": r.get("final_offset"), "worst_fg": r.get("worst_fg"), "worst_at": r.get("worst_at"),
            "unmatched_frames": unmatched, "aligned_frames": len(rows), "notes": r["notes"], "args": r.get("args", ""),
            "strip": os.path.join(d, "strip.png") if os.path.isfile(os.path.join(root, d, "strip.png")) else None,
        })

    json.dump(out, open(os.path.join(root, "summary.json"), "w"), indent=1)
    counts = {}
    for s in out:
        counts[s["status"]] = counts.get(s["status"], 0) + 1
    print("scenarios=%d %s" % (len(out), " ".join("%s=%d" % kv for kv in sorted(counts.items()))))
    print("%-8s %-44s %7s %7s %7s  %s" % ("status", "scenario", "fin_fg", "offset", "worst", "core frames with no ColEm match"))
    for s in out:
        um = s["unmatched_frames"]
        print("%-8s %-44s %7s %7s %7s  %s" % (s["status"], s["name"], s["final_fg"], s["final_offset"], s["worst_fg"],
                                            "%d of %d%s" % (len(um), s["aligned_frames"], (": " + ",".join(map(str, um[:8])) + ("..." if len(um) > 8 else "")) if um else "")))
        for n in s["notes"]:
            print("         note: " + n)


if __name__ == "__main__":
    main()
