#!/usr/bin/env python3
"""Render what the VDP tables say the background should look like, and diff it against a frame.

This is the ground truth the Gauntlet investigation had been missing. ColEm will not run that
cartridge, so the usual reference is unavailable, and "is that solid block a bug or is it the
game's art?" cannot be answered by looking. Rendering the name, pattern and colour tables in
software per the TMS9918A datasheet answers it directly: if the core's pixels differ from what
its own VRAM says they should be, the core is drawing wrongly, and the differing cells are
named. If they agree, whatever looks odd is the game's own doing and the VDP is exonerated.

Sprites are not drawn - only the background - so cells the diff reports are cells to look at,
not automatically faults. Run it on a frame and eyeball the reported cells against the picture.

    vdpref.py --vram DUMP --regs R0,R1,...,R7 --frame FRAME.ppm [--dx 13] [--dy 26] [--out REF.ppm]

DUMP is the output of the simulator's `--peek v:0000:14336@FRAME`, either the whole log line or
just the hex bytes. Registers are the values last written before that frame, which the
SIM_VDP_TRACE output gives.
"""
import sys

# Same table cmpframes.cpp uses for the core, so a diff is like for like.
CORE_PAL = [(0, 0, 0), (0, 0, 0), (33, 200, 66), (94, 220, 120), (84, 85, 237), (125, 118, 252),
            (212, 82, 77), (66, 235, 245), (252, 85, 84), (255, 121, 120), (212, 193, 84),
            (230, 206, 128), (33, 176, 59), (201, 91, 186), (204, 204, 204), (255, 255, 255)]


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


def write_ppm(path, w, h, rgb):
    with open(path, 'wb') as f:
        f.write(b'P6\n%d %d\n255\n' % (w, h))
        f.write(bytes(rgb))


def load_vram(path):
    text = open(path).read()
    # Accept a whole "peek frame N vram 0000: AA BB ..." line or bare hex.
    if ':' in text:
        text = text.split(':', 1)[1]
    return bytes(int(t, 16) for t in text.split())


def render(vram, regs):
    """Background only, per the TMS9918A datasheet. Returns 256x192 colour indices."""
    r0, r1, r2, r3, r4, r5, r6, r7 = regs
    m3 = (r0 >> 1) & 1
    m1 = (r1 >> 4) & 1
    m2 = (r1 >> 3) & 1
    name_base = (r2 & 0x0F) << 10
    backdrop = r7 & 0x0F
    px = [[backdrop] * 256 for _ in range(192)]

    if m1 or m2:
        sys.exit('only Graphics I and II are implemented; this frame is text or multicolor')

    if m3:
        # Graphics II: the screen is three bands, each with its own third of the pattern and
        # colour tables, and R4/R3's low bits mask the high bits of the tile index.
        ptrn_base = (r4 & 0x04) << 11
        colr_base = (r3 & 0x80) << 6
        ptrn_mask = ((r4 & 0x03) << 8) | 0xFF
        colr_mask = ((r3 & 0x7F) << 3) | 0x07
    else:
        ptrn_base = (r4 & 0x07) << 11
        colr_base = r3 << 6

    for row in range(24):
        for col in range(32):
            name = vram[name_base + row * 32 + col]
            if m3:
                idx = (row // 8) * 256 + name
                p = ptrn_base + ((idx & ptrn_mask) << 3)
                c = colr_base + ((idx & colr_mask) << 3)
            else:
                p = ptrn_base + (name << 3)
                c = colr_base + (name >> 3)
            for y in range(8):
                pattern = vram[(p + y) & 0x3FFF]
                colour = vram[(c + (y if m3 else 0)) & 0x3FFF]
                fg, bg = colour >> 4, colour & 0x0F
                if fg == 0:
                    fg = backdrop
                if bg == 0:
                    bg = backdrop
                for x in range(8):
                    px[row * 8 + y][col * 8 + x] = fg if (pattern >> (7 - x)) & 1 else bg
    return px


def main():
    args = sys.argv[1:]
    vram_path = regs = frame_path = out_path = None
    vram_blob = None
    dx, dy = 13, 26
    i = 0
    while i < len(args):
        a = args[i]
        if a == '--vram':
            vram_path = args[i + 1]; i += 2
        elif a == '--vdp':
            # The simulator's .vdp file: eight register bytes then 16384 of VRAM. This is what
            # --shots and the F5/F6/F7 capture hotkeys write beside every frame, so normally
            # neither --vram nor --regs is needed.
            blob = open(args[i + 1], 'rb').read()
            if len(blob) != 8 + 16384:
                sys.exit(f'{args[i + 1]}: expected {8 + 16384} bytes, got {len(blob)}')
            regs = list(blob[:8])
            vram_path = args[i + 1]
            vram_blob = blob[8:]
            i += 2
        elif a == '--regs':
            regs = [int(v, 16) for v in args[i + 1].split(',')]; i += 2
        elif a == '--frame':
            frame_path = args[i + 1]; i += 2
        elif a == '--out':
            out_path = args[i + 1]; i += 2
        elif a == '--dx':
            dx = int(args[i + 1]); i += 2
        elif a == '--dy':
            dy = int(args[i + 1]); i += 2
        else:
            sys.exit(f'unknown option {a}')
    if not (vram_path and regs and frame_path):
        sys.exit(__doc__)
    if len(regs) != 8:
        sys.exit('--regs needs all eight values, R0 first')

    vram = vram_blob if vram_blob is not None else load_vram(vram_path)
    ref = render(vram, regs)
    if out_path:
        flat = bytearray()
        for row in ref:
            for v in row:
                flat.extend(CORE_PAL[v])
        write_ppm(out_path, 256, 192, flat)
        print(f'wrote {out_path}')

    w, h, rgb = read_ppm(frame_path)
    # Compare per 8x8 cell. A cell counts as differing only if a pixel disagrees on colour, and
    # sprites are reported separately because this renderer does not draw them.
    bad = []
    for row in range(24):
        for col in range(32):
            for y in range(8):
                for x in range(8):
                    sy, sx = dy + row * 8 + y, dx + col * 8 + x
                    if sy >= h or sx >= w:
                        continue
                    o = (sy * w + sx) * 3
                    got = (rgb[o], rgb[o + 1], rgb[o + 2])
                    want = CORE_PAL[ref[row * 8 + y][col * 8 + x]]
                    if got != want:
                        bad.append((col, row))
                        break
                else:
                    continue
                break
    print(f'{len(bad)} of 768 cells differ from what the VDP tables say')
    if bad:
        print('(sprites are not rendered here, so cells under a sprite will show up)')
        print('col,row:', ' '.join(f'{c},{r}' for c, r in bad[:40]))
    grid = [['.'] * 32 for _ in range(24)]
    for c, r in bad:
        grid[r][c] = 'X'
    for r in range(24):
        print('%2d %s' % (r, ''.join(grid[r])))


if __name__ == '__main__':
    main()
