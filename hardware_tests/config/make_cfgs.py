#!/usr/bin/env python3
"""Write the .CFG files the test MGLs load.

A MiSTer .CFG for a core is just the 16 byte status word, little endian, which is
the same `status` the OSD sets and `ColecoAdam.sv` reads. Hand editing those bytes
is how a test ends up silently running with the wrong settings, so they are
generated from the table below instead, and the bit layout is written down once.

Run from anywhere:  python3 make_cfgs.py [output dir]

Keep BITS in step with CONF_STR in ColecoAdam.sv. The Expansion RAM field moved
from bits 4-5 to bits 17-19 when the larger cards were added - it needed three
bits and there was no room to grow in place - so any .CFG written before that
selects 64K now whatever it used to say.
"""
import os
import sys

# name -> (first bit, width). From CONF_STR in ColecoAdam.sv.
BITS = {
    'reset':        (0, 1),
    'aspect':       (1, 2),
    'joyswap':      (3, 1),
    'border':       (6, 1),
    'scandoubler':  (7, 3),
    'scale':        (10, 2),
    'mode':         (12, 1),   # 0 = Computer, 1 = Console
    'numpad':       (13, 1),
    'stickkeypad':  (14, 1),
    'spinner':      (15, 2),   # 0 off, 1 spinner, 2 stick X, 3 stick XY
    'expram':       (17, 3),   # 0 64K, 1 256K, 2 512K, 3 1M, 4 2M, 5 none
}

EXP = {'64K': 0, '256K': 1, '512K': 2, '1M': 3, '2M': 4, 'none': 5}

# The settings each test needs. Anything not named is left at 0.
CONFIGS = {
    'AdamT_64K':     {'expram': EXP['64K']},
    'AdamT_256K':    {'expram': EXP['256K']},
    'AdamT_512K':    {'expram': EXP['512K']},
    'AdamT_1M':      {'expram': EXP['1M']},
    'AdamT_2M':      {'expram': EXP['2M']},
    'AdamT_None':    {'expram': EXP['none']},
    'AdamT_Console': {'mode': 1},
    'AdamT_BB':      {'numpad': 1},
    # Stick X, not the spinner device: that is the mode the roller and Super Action
    # Controller testers were actually checked with on hardware.
    'AdamT_Spin':    {'mode': 1, 'spinner': 2},
    # The Super Game Module cartridges are ColecoVision software: console mode,
    # and no ADAM expander in the way.
    'CVSgm':         {'mode': 1, 'expram': EXP['none']},
}


def status_word(settings):
    word = 0
    for name, value in settings.items():
        first, width = BITS[name]
        if value >= (1 << width):
            raise ValueError(f'{name}={value} does not fit in {width} bits')
        word |= value << first
    return word


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    for name, settings in sorted(CONFIGS.items()):
        word = status_word(settings)
        data = word.to_bytes(8, 'little') + bytes(8)
        path = os.path.join(out, name + '.CFG')
        with open(path, 'wb') as f:
            f.write(data)
        bits = ' '.join(f'{k}={v}' for k, v in sorted(settings.items())) or '(defaults)'
        print(f'{name}.CFG  status={word:08X}  {bits}')


if __name__ == '__main__':
    main()
