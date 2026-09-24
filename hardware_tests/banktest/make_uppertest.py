#!/usr/bin/env python3
"""Build uppertest.col, the bank walk done through the UPPER 32K window.

banktest.col only ever exercised the lower window: it sets port 7Fh to 0Eh, which is
lower = RAM expansion and upper = cartridge, and writes at 0000h and 4000h. But a bank
is 64K - one 32K half in the lower window and one in the upper - so half of every bank
has never been tested. A card that answers correctly at 0000h and not at 8000h would
pass banktest and still be broken.

Testing the upper half means the expander covers 8000-FFFF, which is where a cartridge
lives, so the code cannot stay there. This cartridge copies its worker into the 24K of
intrinsic RAM at 7000h and runs it from there with port 7Fh set to 0Bh: lower = OS7 plus
the 24K RAM, upper = RAM expansion.

Results come out the same way banktest's do, so the two are directly comparable:
VRAM 3800h holds the byte read back from each bank, 3820h the count of good banks from
bank 0, and the backdrop is that count halved and clamped to 15 - black for none, green
4, blue 8, red 16, white 32.
"""
import sys

WORKER_ORG = 0x7000


def assemble_worker():
    """The part that runs from RAM, with the expander in the upper window."""
    code = bytearray()
    labels, fixups = {}, []

    def emit(*bs):
        code.extend(bs)

    def label(n):
        labels[n] = WORKER_ORG + len(code)

    def jr(op, n):
        emit(op, 0)
        fixups.append((len(code) - 1, n))

    # lower = 11 (OS7 + 24K RAM, so this code stays put), upper = 10 (RAM expansion)
    emit(0x3E, 0x0B); emit(0xD3, 0x7F)

    # write phase: bank^5A at 8000h, bank^A5 at C000h - the two halves of the upper window
    emit(0x0E, 0x00)                       # ld c,0
    label('w')
    emit(0x79); emit(0xD3, 0x42)           # ld a,c ; out (42h),a
    emit(0x79); emit(0xEE, 0x5A)           # ld a,c ; xor 5Ah
    emit(0x32, 0x00, 0x80)                 # ld (8000h),a
    emit(0x79); emit(0xEE, 0xA5)           # ld a,c ; xor A5h
    emit(0x32, 0x00, 0xC0)                 # ld (C000h),a
    emit(0x0C); emit(0x79); emit(0xFE, 0x20)
    jr(0x20, 'w')                          # jr nz,w

    # read phase
    emit(0x0E, 0x00)                       # ld c,0
    emit(0x1E, 0x00)                       # ld e,0   good banks from 0
    emit(0x16, 0x01)                       # ld d,1   run still unbroken
    label('r')
    emit(0x79); emit(0xD3, 0x42)           # ld a,c ; out (42h),a
    emit(0x3A, 0x00, 0x80)                 # ld a,(8000h)
    emit(0x47)                             # ld b,a
    emit(0x79); emit(0xEE, 0x5A); emit(0xB8)
    jr(0x20, 'bad')
    emit(0x3A, 0x00, 0xC0)                 # ld a,(C000h)
    emit(0x6F)                             # ld l,a
    emit(0x79); emit(0xEE, 0xA5); emit(0xBD)
    jr(0x20, 'bad')
    emit(0x7A); emit(0xB7)                 # ld a,d ; or a
    jr(0x28, 'store')                      # jr z,store
    emit(0x1C)                             # inc e
    jr(0x18, 'store')
    label('bad')
    emit(0x16, 0x00)                       # ld d,0
    label('store')
    # VRAM 3800h + c = the byte read back. The VDP ports work whatever the memory map is.
    emit(0x79); emit(0xD3, 0xBF)
    emit(0x3E, 0x78); emit(0xD3, 0xBF)
    emit(0x78); emit(0xD3, 0xBE)
    emit(0x0C); emit(0x79); emit(0xFE, 0x20)
    jr(0x20, 'r')

    # put the cartridge back before reporting, so the machine is in a sane state
    emit(0x3E, 0x0F); emit(0xD3, 0x7F)

    # VRAM 3820h = the raw count
    emit(0x3E, 0x20); emit(0xD3, 0xBF)
    emit(0x3E, 0x78); emit(0xD3, 0xBF)
    emit(0x7B); emit(0xD3, 0xBE)

    # backdrop = count / 2, clamped to 15
    emit(0x7B); emit(0xCB, 0x3F); emit(0xFE, 0x10)
    jr(0x38, 'col')                        # jr c,col
    emit(0x3E, 0x0F)
    label('col')
    emit(0x4F)                             # ld c,a

    # Paint the whole screen that colour, as banktest does. Leaving the display blanked
    # shows only the backdrop, but the MiSTer's screenshot then comes back as a strip of
    # stale scaler memory rather than the frame, so the display goes back on over an
    # emptied screen: every name entry tile 0, whose pattern is zero and colour 00, so the
    # backdrop shows through. VRAM is whatever the last core left, so write all of it.
    for reg, val in ((0, 0x00),            # Graphics I
                     (2, 0x06),            # name table        1800h
                     (3, 0x80),            # colour table      2000h
                     (4, 0x01),            # pattern table     0800h
                     (5, 0x20),            # sprite attributes 1000h
                     (6, 0x00)):           # sprite patterns   0000h
        emit(0x3E, val); emit(0xD3, 0xBF)
        emit(0x3E, 0x80 | reg); emit(0xD3, 0xBF)

    def vram_addr(addr):
        emit(0x3E, addr & 0xFF); emit(0xD3, 0xBF)
        emit(0x3E, 0x40 | (addr >> 8)); emit(0xD3, 0xBF)

    vram_addr(0x0800)                      # tile 0's pattern: eight zeros
    emit(0x06, 0x08)                       # ld b,8
    label('patclr')
    emit(0xAF); emit(0xD3, 0xBE)           # xor a ; out (BEh),a
    jr(0x10, 'patclr')                     # djnz patclr
    vram_addr(0x2000)                      # tile 0's colour: 00
    emit(0xAF); emit(0xD3, 0xBE)
    vram_addr(0x1000)                      # D0h ends the sprite list: no sprites
    emit(0x3E, 0xD0); emit(0xD3, 0xBE)
    vram_addr(0x1800)                      # name table: 768 entries of tile 0
    emit(0x16, 0x03)                       # ld d,3
    label('nameout')
    emit(0x06, 0x00)                       # ld b,0   256 per pass
    label('namein')
    emit(0xAF); emit(0xD3, 0xBE)
    jr(0x10, 'namein')                     # djnz namein
    emit(0x15)                             # dec d
    jr(0x20, 'nameout')                    # jr nz,nameout

    # backdrop, then the display on: 16K, display on, interrupt off
    emit(0x79); emit(0xD3, 0xBF)
    emit(0x3E, 0x87); emit(0xD3, 0xBF)
    emit(0x3E, 0xC0); emit(0xD3, 0xBF)
    emit(0x3E, 0x81); emit(0xD3, 0xBF)
    label('spin')
    jr(0x18, 'spin')

    for pos, n in fixups:
        d = labels[n] - (WORKER_ORG + pos + 1)
        assert -128 <= d <= 127, f'{n} out of range: {d}'
        code[pos] = d & 0xFF
    return bytes(code)


worker = assemble_worker()

code = bytearray()
code += bytes([0x55, 0xAA]) + bytes(8) + bytes([0x24, 0x80])
for _ in range(7):
    code += bytes([0xC3, 0x24, 0x80])
code += bytes([0xED, 0x45, 0x00])
assert len(code) == 0x24

# Entry: blank the display and turn the interrupt off - there is no stack discipline here
# and an NMI would push onto RAM that is about to change shape.
code += bytes([0xF3])
code += bytes([0x3E, 0x80, 0xD3, 0xBF, 0x3E, 0x81, 0xD3, 0xBF])

# Copy the worker into RAM at 7000h and jump to it. The cartridge reset already left the
# lower window as OS7 plus the 24K RAM, so 7000h is writable right now.
# The copy stub below is 14 bytes: ld hl / ld de / ld bc / ldir / jp.
src = 0x8000 + len(code) + 14           # where the worker bytes sit, after this stub
code += bytes([0x21, src & 0xFF, src >> 8])          # ld hl,src
code += bytes([0x11, WORKER_ORG & 0xFF, WORKER_ORG >> 8])  # ld de,7000h
code += bytes([0x01, len(worker) & 0xFF, len(worker) >> 8])  # ld bc,len
code += bytes([0xED, 0xB0])                          # ldir
code += bytes([0xC3, WORKER_ORG & 0xFF, WORKER_ORG >> 8])   # jp 7000h
assert len(code) == src - 0x8000, (len(code), src - 0x8000)
code += worker

while len(code) < 8192:
    code.append(0xFF)

out = sys.argv[1] if len(sys.argv) > 1 else 'uppertest.col'
open(out, 'wb').write(bytes(code))
print(f'{out} {len(code)} bytes, worker {len(worker)} bytes at {WORKER_ORG:04X}')
