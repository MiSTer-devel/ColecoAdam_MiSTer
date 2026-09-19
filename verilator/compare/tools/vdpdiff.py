#!/usr/bin/env python3
"""vdpdiff.py CORE.vdp COLEM.vdp — compare TMS9918 state dumps (16 KB VRAM + registers 0-7)."""
import sys


def load(path):
    data = open(path, "rb").read()
    return data[:0x4000], list(data[0x4000:0x4008])


def tables(r):
    """VRAM regions for the display mode selected by the registers (TMS9918A datasheet)."""
    m1, m2, m3 = (r[1] >> 4) & 1, (r[0] >> 1) & 1, (r[1] >> 3) & 1
    if m2:  # Graphics II: colour and pattern tables are 3 x 256 x 8 bytes, masked
        colour_base = ((r[3] & 0x80) << 6)
        pattern_base = ((r[4] & 0x04) << 11)
        colour_len = ((((r[3] & 0x7F) << 3) | 0x07) + 1) * 8
        pattern_len = ((((r[4] & 0x03) << 8) | 0xFF) + 1) * 8
        mode = "Graphics II"
    else:
        colour_base, colour_len = r[3] << 6, 32
        pattern_base, pattern_len = (r[4] & 7) << 11, 2048
        mode = "Text" if m1 else "Multicolour" if m3 else "Graphics I"
    return mode, [
        ("name table", (r[2] & 0x0F) << 10, 960 if m1 else 768),
        ("colour table", colour_base, 0 if (m1 or m3) else colour_len),
        ("pattern table", pattern_base, pattern_len),
        ("sprite attributes", (r[5] & 0x7F) << 7, 128),
        ("sprite patterns", (r[6] & 7) << 11, 2048),
    ]


def main():
    core_vram, core_r = load(sys.argv[1])
    colem_vram, colem_r = load(sys.argv[2])
    print("register   core  colem")
    for i in range(8):
        flag = "" if core_r[i] == colem_r[i] else "   <-- differs"
        print("R%d         %02X    %02X%s" % (i, core_r[i], colem_r[i], flag))

    mode, regions = tables(colem_r)
    print("mode (from ColEm registers): %s" % mode)
    for name, base, length in regions:
        if not length:
            continue
        a = core_vram[base:base + length]
        b = colem_vram[base:base + length]
        diffs = [i for i in range(min(len(a), len(b))) if a[i] != b[i]]
        line = "%-18s @%04X len %4d: %4d bytes differ" % (name, base, length, len(diffs))
        if diffs:
            first = diffs[:6]
            line += "  first at +%s" % ", +".join("%X (core %02X colem %02X)" % (i, a[i], b[i]) for i in first)
        print(line)
    total = sum(1 for i in range(0x4000) if core_vram[i] != colem_vram[i])
    print("whole VRAM: %d of 16384 bytes differ" % total)


if __name__ == "__main__":
    main()
