#!/usr/bin/env python3
"""Convert a binary P6 PPM to PNG using only the standard library."""
import struct, sys, zlib


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
    w, h, _maxval = fields
    return w, h, data[pos:pos + w * h * 3]


def write_png(path, w, h, rgb):
    raw = b''.join(b'\x00' + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))

    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload
                + struct.pack('>I', zlib.crc32(tag + payload) & 0xffffffff))

    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 9))
           + chunk(b'IEND', b''))
    open(path, 'wb').write(png)


def crop(w, h, rgb, x, y, cw, ch):
    cw, ch = min(cw, w - x), min(ch, h - y)
    rows = [rgb[((y + r) * w + x) * 3:((y + r) * w + x + cw) * 3] for r in range(ch)]
    return cw, ch, b''.join(rows)


def scale(w, h, rgb, n):
    """Nearest neighbour, so a 240 line frame stays readable when a detail is 30 px wide."""
    out = []
    for y in range(h):
        row = rgb[y * w * 3:(y + 1) * w * 3]
        big = b''.join(row[x * 3:(x + 1) * 3] * n for x in range(w))
        out.extend([big] * n)
    return w * n, h * n, b''.join(out)


if __name__ == '__main__':
    # ppm2png.py [--crop X,Y,W,H] [--scale N] file.ppm...
    args, box, mag = sys.argv[1:], None, 1
    while args and args[0].startswith('--'):
        opt = args.pop(0)
        if opt == '--crop':
            box = [int(v) for v in args.pop(0).split(',')]
        elif opt == '--scale':
            mag = int(args.pop(0))
        else:
            sys.exit(f'unknown option {opt}')
    for src in args:
        w, h, rgb = read_ppm(src)
        if box:
            w, h, rgb = crop(w, h, rgb, *box)
        if mag > 1:
            w, h, rgb = scale(w, h, rgb, mag)
        dst = src.rsplit('.', 1)[0] + '.png'
        write_png(dst, w, h, rgb)
        print(f'{dst} {w}x{h}')
