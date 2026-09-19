#!/usr/bin/env python3
"""adam_exp_ramtest.py [--banks] [--trace] OUT.rom

Builds an 8K ADAM cartridge that checks the memory expander and shows the result as the
backdrop colour of a blanked screen:

    medium green  both windows pass
    medium red    the upper window (port 7Fh upper bits 10, 8000h-FFFFh) fails
    magenta       the lower window (port 7Fh lower bits 10, 0000h-7FFFh) fails
    black         both fail

Default: each 32K window is filled with (H xor L xor 55h) and read back, which checks a 64K
expander. With --banks, each window instead gets a different byte in each of banks 0-3 through
port 42h (bank xor A5h at both ends of the window), read back afterwards. That passes only
when there are four distinct banks (256K or more). A plain 64K expander aliases every bank and
shows black.

Load it in ADAM mode (--adam --cart), which starts it through the cartridge reset: OS-7, 24K
of RAM at 2000h-7FFFh and the cartridge at 8000h. The upper window is tested from lower RAM
(6000h), the lower window from intrinsic upper RAM (C000h), with no stack use while lower
memory is switched away. The VDP interrupt is switched off first, so no NMI arrives mid-test.

--trace adds progress markers written to port 3Fh, which the simulator logs ("SETTING MEMORY
MODE 3F? ... data NN"). Their values keep bits 0-1 clear (net reset, EOS enable):
    04 start  08 upper window done  14 lower tester running  18 lower window done  24 finish

There is no Z80 assembler on the build machine, so this is a tiny two-pass assembler over
hand-encoded bytes; check the output with: z80dasm -a -g 0x8000 OUT.rom
"""
import sys

VDP_CTRL = 0xBF


def full_test(p, top, set_bit):
    """Fill a 32K window starting at top*100h with (H xor L xor 55h) and verify it."""
    if top == 0x80:
        at_end = (0x7C, 0xB5)                       # LD A,H / OR L: HL wrapped to 0000h
    else:
        at_end = (0x7C, 0xFE, 0x80)                 # LD A,H / CP 80h
    return [
        ('b', 0x21, 0x00, top),                     # LD HL,window
        ('label', p + 'w'),
        ('b', 0x7C, 0xAD, 0xEE, 0x55, 0x77,         # LD A,H / XOR L / XOR 55h / LD (HL),A
               0x23, *at_end, 0x20, ('r', p + 'w')),  # INC HL / end test / JR NZ
        ('b', 0x21, 0x00, top),                     # LD HL,window
        ('label', p + 'v'),
        ('b', 0x7C, 0xAD, 0xEE, 0x55, 0xBE,         # LD A,H / XOR L / XOR 55h / CP (HL)
               0x20, ('r', p + 'fail'),             # JR NZ,fail
               0x23, *at_end, 0x20, ('r', p + 'v')),  # INC HL / end test / JR NZ
        ('b', 0x18, ('r', p + 'ok')),               # JR ok
        ('label', p + 'fail'),
        ('b', 0xCB, set_bit),                       # SET n,D
        ('label', p + 'ok'),
    ]


def bank_test(p, first, last, set_bit):
    """Write (bank xor A5h) at two addresses in banks 3..0, then check banks 0..3."""
    def addr(a):
        return (a & 0xFF, a >> 8)
    return [
        ('b', 0x06, 0x03),                          # LD B,3
        ('label', p + 'w'),
        ('b', 0x78, 0xD3, 0x42,                     # LD A,B / OUT (42h),A
               0xEE, 0xA5,                          # XOR 0A5h
               0x32, *addr(first),                  # LD (first),A
               0x32, *addr(last),                   # LD (last),A
               0x05,                                # DEC B
               0xF2, ('w', p + 'w')),               # JP P,write loop (B = 3, 2, 1, 0)
        ('b', 0x06, 0x00),                          # LD B,0
        ('label', p + 'v'),
        ('b', 0x78, 0xD3, 0x42, 0xEE, 0xA5, 0x4F,   # LD A,B / OUT (42h),A / XOR 0A5h / LD C,A
               0x3A, *addr(first), 0xB9,            # LD A,(first) / CP C
               0x20, ('r', p + 'fail'),             # JR NZ,fail
               0x3A, *addr(last), 0xB9,             # LD A,(last) / CP C
               0x20, ('r', p + 'fail'),             # JR NZ,fail
               0x04, 0x78, 0xFE, 0x04,              # INC B / LD A,B / CP 4
               0x20, ('r', p + 'v'),                # JR NZ,verify loop
               0x18, ('r', p + 'ok')),              # JR ok
        ('label', p + 'fail'),
        ('b', 0xCB, set_bit),                       # SET n,D
        ('label', p + 'ok'),
    ]


def program(trace, banks):
    def mark(value):
        return [('b', 0x3E, value, 0xD3, 0x3F)] if trace else []   # LD A,n / OUT (3Fh),A

    if banks:
        upper = bank_test('u', 0x8000, 0xFFFF, 0xC2)                # SET 0,D on failure
        lower = bank_test('l', 0x0000, 0x7FFF, 0xCA)                # SET 1,D on failure
    else:
        upper = full_test('u', 0x80, 0xC2)
        lower = full_test('l', 0x00, 0xCA)

    return [
        ('org', 0x8000),
        # Cartridge header: 55h AAh (test cartridge) skips the ~12 s title screen that AAh 55h
        # shows; 800Ah is the start address
        ('b', 0x55, 0xAA, 0, 0, 0, 0, 0, 0, 0, 0, ('w', 'start')),
        ('pad', 0x8021, 0xC9),                      # RST 08h-30h vectors: RET
        ('b', 0xED, 0x45),                          # 8021h NMI vector: RETN
        ('pad', 0x8024, 0x00),
        ('label', 'start'),
        ('b', 0xF3),                                # DI
        ('b', 0x31, 0x00, 0x70),                    # LD SP,7000h
        ('b', 0x3E, 0x80, 0xD3, VDP_CTRL,           # VDP register 1 = 80h:
               0x3E, 0x81, 0xD3, VDP_CTRL),         #   16K, display blanked, no interrupt
        ('b', 0xDB, VDP_CTRL),                      # IN A,(BFh): clear any pending frame flag
        *mark(0x04),
        ('b', 0x21, ('w', 'blob'),                  # LD HL,blob
               0x11, 0x00, 0x60,                    # LD DE,6000h
               0x01, 0x00, 0x01,                    # LD BC,0100h
               0xED, 0xB0,                          # LDIR
               0xC3, 0x00, 0x60),                   # JP 6000h
        ('label', 'blob'),

        ('org', 0x6000),
        ('b', 0x16, 0x00),                          # LD D,0: bit 0 upper failed, bit 1 lower
        ('b', 0x3E, 0x0B, 0xD3, 0x7F),              # port 7Fh = 0Bh: OS-7 + 24K, expansion RAM
        *upper,
        *mark(0x08),
        # Lower window: run the tester from intrinsic upper RAM, since 0000h-7FFFh goes away
        ('b', 0xD5),                                # PUSH DE: keep the upper result; LDIR uses DE
        ('b', 0x3E, 0x03, 0xD3, 0x7F),              # port 7Fh = 03h: upper = intrinsic RAM
        ('b', 0x21, ('w', 'ltest'),                 # LD HL,ltest
               0x11, 0x00, 0xC0,                    # LD DE,C000h
               0x01, 0x80, 0x00,                    # LD BC,0080h
               0xED, 0xB0,                          # LDIR
               0xC3, 0x00, 0xC0),                   # JP C000h
        ('label', 'finish'),
        *mark(0x24),
        ('b', 0x3E, 0x00, 0xD3, 0x42),              # port 42h = 0: back to bank 0
        ('b', 0x3E, 0x0F, 0xD3, 0x7F),              # port 7Fh = 0Fh: OS-7 + 24K, cartridge
        ('b', 0xC1,                                 # POP BC: B = upper result
               0x78, 0xB2, 0x4F, 0x06, 0x00,        # LD A,B / OR D / LD C,A / LD B,0
               0x21, ('w', 'colours'),              # LD HL,colours
               0x09, 0x7E),                         # ADD HL,BC / LD A,(HL)
        ('b', 0xD3, VDP_CTRL, 0x3E, 0x87, 0xD3, VDP_CTRL),  # VDP register 7 = backdrop colour
        ('b', 0x18, 0xFE),                          # JR $
        ('label', 'colours'),
        ('b', 0x02, 0x08, 0x0D, 0x01),              # pass, upper failed, lower failed, both
        ('label', 'ltest'),

        ('org', 0xC000),
        ('b', 0x16, 0x00),                          # LD D,0: lower result (SP stays in lower
                                                    # RAM, untouched until finish pops it)
        *mark(0x14),
        ('b', 0x3E, 0x02, 0xD3, 0x7F),              # port 7Fh = 02h: expansion RAM, intrinsic RAM
        *lower,
        *mark(0x18),
        ('b', 0x3E, 0x03, 0xD3, 0x7F),              # port 7Fh = 03h: OS-7 + 24K again
        ('b', 0xC3, ('w', 'finish')),               # JP finish
        ('label', 'end'),
    ]


def assemble(items, labels):
    """One pass. Labels are run addresses; 'org' changes the run address, not the load order."""
    out = bytearray()
    run = 0
    new_labels = {}
    for item in items:
        kind = item[0]
        if kind == 'org':
            run = item[1]
        elif kind == 'label':
            new_labels[item[1]] = run
        elif kind == 'pad':
            while run < item[1]:
                out.append(item[2])
                run += 1
        elif kind == 'b':
            for field in item[1:]:
                if isinstance(field, int):
                    out.append(field)
                    run += 1
                elif field[0] == 'w':
                    value = labels.get(field[1], 0)
                    out += bytes((value & 0xFF, value >> 8))
                    run += 2
                elif field[0] == 'r':
                    offset = labels.get(field[1], run + 1) - (run + 1)
                    if labels and not -128 <= offset <= 127:
                        raise SystemExit(f"relative jump to {field[1]} out of range")
                    out.append(offset & 0xFF)
                    run += 1
    return out, new_labels


def main():
    args = sys.argv[1:]
    trace = '--trace' in args
    banks = '--banks' in args
    args = [a for a in args if a not in ('--trace', '--banks')]
    if len(args) != 1:
        raise SystemExit(__doc__)
    items = program(trace, banks)
    _, labels = assemble(items, {})
    code, labels2 = assemble(items, labels)
    if labels2 != labels:
        raise SystemExit("labels moved between passes")
    blob_len = labels['ltest'] - 0x6000
    ltest_len = labels['end'] - 0xC000
    if blob_len + ltest_len > 0x100 or ltest_len > 0x80:
        raise SystemExit("copied routines outgrew their LDIR lengths")
    rom = code + bytes([0xFF] * (0x2000 - len(code)))
    open(args[0], 'wb').write(rom)
    print(f"wrote {args[0]}: {len(code)} bytes of code; 6000h routine {blob_len} bytes, "
          f"C000h routine {ltest_len} bytes{' (bank test)' if banks else ''}"
          f"{' (trace markers on)' if trace else ''}")


if __name__ == '__main__':
    main()
