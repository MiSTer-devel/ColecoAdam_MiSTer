#!/usr/bin/env python3
"""Find tile cells that change between consecutive frames.

Point it at a directory of PPMs captured on consecutive frames with the game
standing still. Anything in the background that changes is either an animating
tile the game meant to animate, or the artefact we are hunting.

    flicker.py DIR [--dx 13] [--dy 26] [--top 20]

The core's 320x240 frame carries the 256x192 display at (dx, dy), the same
offsets verilator/compare/validate_cart.sh uses, so cell coordinates come out as
VDP tile columns and rows - which is what you need to index the name table.

Reports, per 8x8 tile cell, how many frame-to-frame changes it saw, and how many
of those were a *revert* to the previous appearance. A reverting cell is the
signature of flicker rather than of something being drawn once.
"""
import collections
import os
import sys


def read_ppm(path):
    data = open(path, 'rb').read()
    if not data.startswith(b'P6'):
        raise ValueError(f'{path} is not a P6 PPM')
    fields, pos = [], 2
    while len(fields) < 3:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        fields.append(int(data[start:pos]))
    pos += 1
    w, h, _ = fields
    return w, h, data[pos:pos + w * h * 3]


def cell_hashes(w, h, rgb, dx, dy, cols=32, rows=24):
    """One hash per 8x8 tile cell of the 256x192 display."""
    out = {}
    for r in range(rows):
        for c in range(cols):
            acc = 0
            for y in range(dy + r * 8, dy + r * 8 + 8):
                if y >= h:
                    continue
                base = (y * w + dx + c * 8) * 3
                acc = hash((acc, rgb[base:base + 24]))
            out[(c, r)] = acc
    return out


def main():
    args = sys.argv[1:]
    if not args:
        sys.exit(__doc__)
    d = args[0]
    dx, dy, top = 13, 26, 20
    i = 1
    while i < len(args):
        if args[i] == '--dx':
            dx = int(args[i + 1]); i += 2
        elif args[i] == '--dy':
            dy = int(args[i + 1]); i += 2
        elif args[i] == '--top':
            top = int(args[i + 1]); i += 2
        else:
            sys.exit(f'unknown option {args[i]}')

    files = sorted(f for f in os.listdir(d) if f.endswith('.ppm'))
    if len(files) < 2:
        sys.exit(f'{d}: need at least two frames, found {len(files)}')

    changes = collections.Counter()
    reverts = collections.Counter()
    prev = None
    prev2 = None
    for name in files:
        w, h, rgb = read_ppm(os.path.join(d, name))
        cur = cell_hashes(w, h, rgb, dx, dy)
        if prev is not None:
            for key, val in cur.items():
                if val != prev[key]:
                    changes[key] += 1
                    if prev2 is not None and val == prev2[key]:
                        reverts[key] += 1
        prev2, prev = prev, cur

    print(f'{len(files)} frames, display at ({dx},{dy})')
    changed = sorted(changes.items(), key=lambda kv: -kv[1])
    if not changed:
        print('no tile cell changed at all: the screen is completely static')
        return
    print(f'{len(changed)} of 768 cells changed at least once')
    print(f'{"col,row":>10}  {"changes":>7}  {"reverts":>7}')
    for (c, r), n in changed[:top]:
        print(f'{c:>4},{r:<5}  {n:>7}  {reverts[(c, r)]:>7}')

    # A map is easier to read than a list: '.' static, digits = how busy.
    print('\ncolumn 0 at left, row 0 at top; . = static, 1-9 = changes, # = 10+')
    for r in range(24):
        line = ''
        for c in range(32):
            n = changes[(c, r)]
            line += '.' if n == 0 else ('#' if n >= 10 else str(n))
        print(f'{r:2} {line}')


if __name__ == '__main__':
    main()
