#!/usr/bin/env python3
"""Build vramtest.col, which writes VRAM faster than the CPU access window.

The TMS9918A datasheet (2.1.6) says that in Graphics mode with sprites, during
the active display, "CPU windows occur once every 16 memory cycles giving a
maximum delay of 6 microseconds (a memory cycle takes about 372 nanoseconds)".
A program that writes the data port faster than that gets ahead of the VDP.

What the real chip does then is lose the byte but keep counting: "A 1-byte
transfer is then required to read the data from the addressed VRAM byte. The
address register is then autoincremented" - the increment belongs to the
transfer, not to the VRAM cycle. So a too-fast burst leaves one stale byte and
everything after it is still in the right place.

This cartridge writes 4096 bytes to VRAM 2800h with the value equal to the low
byte of the offset, from an unrolled loop of

    out (0BEh),a        12 T with the ColecoVision M1 wait
    inc a                5 T

which is 17 T, about 4.75 us per byte at 3.58 MHz - inside the 6 us window, so
the VDP cannot keep up. The burst runs for about 20 ms, longer than a frame, so
part of it always lands in the active display however it is timed.

Read the result with the simulator:

    --peek v:2800:32@FRAME    and    --peek v:37E0:32@FRAME

VRAM[2800h + i] should be i & 0FFh. If the address counter is losing increments
the values drift: at the end of the buffer the byte is (i + drops) & 0FFh, so
the drift is visible as the expected value minus what is there.
"""
import sys

code = bytearray()
labels, fixups = {}, []


def emit(*bs):
    code.extend(bs)


def label(name):
    labels[name] = 0x8000 + len(code)


def djnz(name):
    emit(0x10, 0)
    fixups.append((len(code) - 1, name))


# --- cartridge header ---------------------------------------------------------
emit(0x55, 0xAA)
emit(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
emit(0x24, 0x80)                      # 800A: entry
for _ in range(7):
    emit(0xC3, 0x24, 0x80)            # 800C-8020: RST vectors
emit(0xED, 0x45, 0x00)                # 8021: NMI, RETN
assert len(code) == 0x24, hex(len(code))

label('start')
emit(0xF3)                            # di


def setreg(reg, val):
    emit(0x3E, val)                   # ld a,val
    emit(0xD3, 0xBF)                  # out (BFh),a
    emit(0x3E, 0x80 | reg)            # ld a,80h|reg
    emit(0xD3, 0xBF)                  # out (BFh),a


def vram_addr(addr, write=True):
    emit(0x3E, addr & 0xFF)
    emit(0xD3, 0xBF)
    emit(0x3E, ((0x40 if write else 0x00) | (addr >> 8)) & 0xFF)
    emit(0xD3, 0xBF)


# Graphics I, display ON with the interrupt off, sprites 16x16 and magnified so
# that each one covers 32 scan lines. The display has to be on: with it blanked
# the VDP needs no access windows at all and there is nothing to contend with.
setreg(0, 0x00)
setreg(1, 0xC3)                       # 16K, display on, IE off, size 1, magnified
setreg(2, 0x06)                       # name table    1800h
setreg(3, 0x80)                       # colour table  2000h
setreg(4, 0x00)                       # pattern table 0000h
setreg(5, 0x36)                       # sprite attributes 1B00h
setreg(6, 0x07)                       # sprite patterns   3800h
setreg(7, 0x01)                       # backdrop black

# Six magnified 16x16 sprites, 32 lines each, at 32 line spacing: every one of the
# 192 active lines has exactly one sprite on it. That is the datasheet's worst
# case - Graphics mode with sprites in use - on every line, so no write in the
# burst can dodge the contention by landing on a sprite-free line.
vram_addr(0x1B00)
for y in (0x00, 0x20, 0x40, 0x60, 0x80, 0xA0):
    emit(0x3E, y);    emit(0xD3, 0xBE)    # Y
    emit(0x3E, 0x10); emit(0xD3, 0xBE)    # X
    emit(0x3E, 0x00); emit(0xD3, 0xBE)    # name
    emit(0x3E, 0x0F); emit(0xD3, 0xBE)    # colour white
emit(0x3E, 0xD0); emit(0xD3, 0xBE)        # Y=D0h ends the list

# Give the sprites a solid pattern so they are really drawn.
vram_addr(0x3800)
emit(0x06, 0x20)                      # ld b,32
label('sp')
emit(0x3E, 0xFF)                      # ld a,FFh
emit(0xD3, 0xBE)                      # out (BEh),a
djnz('sp')

# --- the burst ---------------------------------------------------------------
vram_addr(0x2800)
emit(0x06, 0x00)                      # ld b,0   -> 256 passes
emit(0xAF)                            # xor a
label('burst')
for _ in range(16):                   # 16 bytes per pass, 4096 in total
    emit(0xD3, 0xBE)                  # out (BEh),a
    emit(0x3C)                        # inc a
djnz('burst')

label('spin')
emit(0x18, 0xFE)                      # jr spin

for pos, name in fixups:
    delta = labels[name] - (0x8000 + pos + 1)
    assert -128 <= delta <= 127, f'{name} out of range: {delta}'
    code[pos] = delta & 0xFF

while len(code) < 8192:
    code.append(0xFF)

out = sys.argv[1] if len(sys.argv) > 1 else 'vramtest.col'
open(out, 'wb').write(bytes(code))
print(f'{out} {len(code)} bytes, entry 8024h')
