#!/usr/bin/env python3
"""Build banktest.col, a cartridge that walks every memory expander bank.

No period software counts past 512K - PowerPAINT's sizer saturates at four banks
beyond the base and reports 512 for anything larger, and Orphanware's RAMTEST is
an address range tester, not a sizer. So to check the 1MB and 2MB cards there has
to be something that counts all 32 banks, and it has to run on the FPGA as well as
in the simulator, because that is where the SDRAM path can actually go wrong.

What it does, with the lower 32K window switched to the expander (port 7Fh) and the
bank chosen by port 42h:

  write phase  every bank gets two markers, bank^5A at offset 0000 and bank^A5 at
               offset 4000. Two different values in the two halves of the 64K bank
               catch a card that follows the bank in one window but not the other,
               and XOR keeps the marker from matching the bank number a stuck bus
               would leave behind.
  read phase   every bank is selected again and both markers checked. A bank that
               aliases to another bank fails, because the later write clobbered it.

Results come out two ways, because the two places this runs can see different things:

  VRAM 3800h  one byte per bank: the marker read back from offset 0000. 3820h holds
              the number of consecutive good banks from bank 0. The simulator reads
              these with --peek v:3800:33@FRAME.
  backdrop    VDP register 7, set to banks/2 clamped to 15, which is all a screen can
              show without a character set. The TMS9918 colours are far enough apart
              to read across a room:

                  0  black   no banking:  no card, or the 64K card (which has no
                             bank register, so every bank is the same 64K)
                  2  green   4 banks   256K
                  4  blue    8 banks   512K
                  8  red    16 banks   1MB
                 15  white  32 banks   2MB

Usage: make_banktest.py [output.col]
"""
import sys

# --- a tiny Z80 assembler: just enough for one straight-line test -------------
code = bytearray()
labels, fixups = {}, []


def emit(*bs):
    code.extend(bs)


def label(name):
    labels[name] = 0x8000 + len(code)


def jr_nz(name):
    emit(0x20, 0)
    fixups.append((len(code) - 1, name))


def jr_c(name):
    emit(0x38, 0)
    fixups.append((len(code) - 1, name))


def jr_z(name):
    emit(0x28, 0)
    fixups.append((len(code) - 1, name))


def jr(name):
    emit(0x18, 0)
    fixups.append((len(code) - 1, name))


# --- cartridge header ---------------------------------------------------------
# 55 AA tells OS-7 to skip its title screen and jump straight to the address at
# 800Ah. The four pointers are unused here: this test never calls back into the
# BIOS, which it could not do anyway once the expander is mapped over 0000-7FFF.
emit(0x55, 0xAA)
emit(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
emit(0x24, 0x80)                      # 800A: entry point, 8024h
for _ in range(7):                    # 800C-8020: RST 08 through RST 38
    emit(0xC3, 0x24, 0x80)
emit(0xED, 0x45, 0x00)                # 8021: NMI. RETN - the display interrupt is
                                      # turned off below, so this should never run,
                                      # and it must not, because there is no stack
                                      # once the expander replaces the RAM.
assert len(code) == 0x24, hex(len(code))

# --- the test -----------------------------------------------------------------
label('start')
emit(0xF3)                            # di

# R1 = 80h: 16K VRAM, display blanked, and the vertical interrupt OFF. The ADAM's
# display interrupt reaches the Z80 as NMI, which DI cannot mask, and an NMI here
# would push a return address onto a stack that is about to stop being RAM.
emit(0x3E, 0x80)                      # ld a,80h
emit(0xD3, 0xBF)                      # out (BFh),a
emit(0x3E, 0x81)                      # ld a,81h   (register 1)
emit(0xD3, 0xBF)                      # out (BFh),a

# Port 7Fh: lower window (bits 1:0) = 10, RAM expansion; upper (bits 3:2) = 11,
# cartridge, so this code keeps running while 0000-7FFF becomes the card.
emit(0x3E, 0x0E)                      # ld a,0Eh
emit(0xD3, 0x7F)                      # out (7Fh),a

# write phase
emit(0x0E, 0x00)                      # ld c,0
label('wr')
emit(0x79)                            # ld a,c
emit(0xD3, 0x42)                      # out (42h),a
emit(0x79)                            # ld a,c
emit(0xEE, 0x5A)                      # xor 5Ah
emit(0x32, 0x00, 0x00)                # ld (0000h),a
emit(0x79)                            # ld a,c
emit(0xEE, 0xA5)                      # xor A5h
emit(0x32, 0x00, 0x40)                # ld (4000h),a
emit(0x0C)                            # inc c
emit(0x79)                            # ld a,c
emit(0xFE, 0x20)                      # cp 32
jr_nz('wr')

# read phase. E counts banks that are good from bank 0 onwards; D stays 1 only
# while that run is unbroken, so one aliased bank stops the count where it should.
emit(0x0E, 0x00)                      # ld c,0
emit(0x1E, 0x00)                      # ld e,0
emit(0x16, 0x01)                      # ld d,1
label('rd')
emit(0x79)                            # ld a,c
emit(0xD3, 0x42)                      # out (42h),a
emit(0x3A, 0x00, 0x00)                # ld a,(0000h)
emit(0x47)                            # ld b,a        keep it for the VRAM table
emit(0x79)                            # ld a,c
emit(0xEE, 0x5A)                      # xor 5Ah
emit(0xB8)                            # cp b
jr_nz('bad')
emit(0x3A, 0x00, 0x40)                # ld a,(4000h)
emit(0x6F)                            # ld l,a
emit(0x79)                            # ld a,c
emit(0xEE, 0xA5)                      # xor A5h
emit(0xBD)                            # cp l
jr_nz('bad')
emit(0x7A)                            # ld a,d        still an unbroken run?
emit(0xB7)                            # or a
jr_z('store')
emit(0x1C)                            # inc e
jr('store')
label('bad')
emit(0x16, 0x00)                      # ld d,0
label('store')
# VRAM 3800h + c = the byte read back from this bank.
emit(0x79)                            # ld a,c        low address byte
emit(0xD3, 0xBF)                      # out (BFh),a
emit(0x3E, 0x78)                      # ld a,78h      40h | 38h, write to 3800h
emit(0xD3, 0xBF)                      # out (BFh),a
emit(0x78)                            # ld a,b
emit(0xD3, 0xBE)                      # out (BEh),a
emit(0x0C)                            # inc c
emit(0x79)                            # ld a,c
emit(0xFE, 0x20)                      # cp 32
jr_nz('rd')

# VRAM 3820h = the raw count, which the simulator reads exactly.
emit(0x3E, 0x20)                      # ld a,20h
emit(0xD3, 0xBF)                      # out (BFh),a
emit(0x3E, 0x78)                      # ld a,78h
emit(0xD3, 0xBF)                      # out (BFh),a
emit(0x7B)                            # ld a,e
emit(0xD3, 0xBE)                      # out (BEh),a

# Backdrop colour = banks/2, clamped to 15, so a screen can show the answer.
emit(0x7B)                            # ld a,e
emit(0xCB, 0x3F)                      # srl a
emit(0xFE, 0x10)                      # cp 16
jr_c('setcol')
emit(0x3E, 0x0F)                      # ld a,15
label('setcol')
emit(0x4F)                            # ld c,a        keep the colour for register 7

# Paint the whole screen that colour.
#
# Leaving the display blanked would show the backdrop and nothing else, which is
# tempting, but on the MiSTer it changes the core's video output enough that the
# screenshot comes back 93 lines tall instead of 192 - so the display is turned
# back on over a screen deliberately emptied instead. Every name table entry is
# tile 0, whose pattern is all zeros and whose colour entry is 00: foreground and
# background both transparent, so the backdrop shows through the whole frame.
# VRAM is whatever the last core left in it, which is why all three tables have to
# be written rather than trusted.
for reg, val in ((0, 0x00),           # Graphics I
                 (2, 0x06),           # name table    1800h
                 (3, 0x80),           # colour table  2000h
                 (4, 0x01),           # pattern table 0800h
                 (5, 0x20),           # sprite attributes 1000h
                 (6, 0x00)):          # sprite patterns   0000h
    emit(0x3E, val)                   # ld a,val
    emit(0xD3, 0xBF)                  # out (BFh),a
    emit(0x3E, 0x80 | reg)            # ld a,80h|reg
    emit(0xD3, 0xBF)                  # out (BFh),a


def vram_addr(addr):
    """Point the VDP at addr for writing."""
    emit(0x3E, addr & 0xFF)           # ld a,low
    emit(0xD3, 0xBF)                  # out (BFh),a
    emit(0x3E, 0x40 | (addr >> 8))    # ld a,40h|high
    emit(0xD3, 0xBF)                  # out (BFh),a


# Tile 0's pattern: eight zero bytes.
vram_addr(0x0800)
emit(0x06, 0x08)                      # ld b,8
label('patclr')
emit(0xAF)                            # xor a
emit(0xD3, 0xBE)                      # out (BEh),a
emit(0x10, 0xFB)                      # djnz patclr (-5)

# Tile 0's colour: 00, both nibbles transparent.
vram_addr(0x2000)
emit(0xAF)                            # xor a
emit(0xD3, 0xBE)                      # out (BEh),a

# The name table: 768 entries, all tile 0.
vram_addr(0x1800)
emit(0x16, 0x03)                      # ld d,3
label('nameout')
emit(0x06, 0x00)                      # ld b,0     256 per pass
label('namein')
emit(0xAF)                            # xor a
emit(0xD3, 0xBE)                      # out (BEh),a
emit(0x10, 0xFB)                      # djnz namein (-5)
emit(0x15)                            # dec d
jr_nz('nameout')

# Backdrop, then the display on.
emit(0x79)                            # ld a,c
emit(0xD3, 0xBF)                      # out (BFh),a
emit(0x3E, 0x87)                      # ld a,87h      (register 7)
emit(0xD3, 0xBF)                      # out (BFh),a
emit(0x3E, 0xC0)                      # ld a,C0h      16K, display on, interrupt off
emit(0xD3, 0xBF)                      # out (BFh),a
emit(0x3E, 0x81)                      # ld a,81h      (register 1)
emit(0xD3, 0xBF)                      # out (BFh),a
label('spin')
jr('spin')

for pos, name in fixups:
    delta = labels[name] - (0x8000 + pos + 1)
    assert -128 <= delta <= 127, f'{name} out of range: {delta}'
    code[pos] = delta & 0xFF

# ColecoVision cartridges are read as whole 8K pages.
while len(code) < 8192:
    code.append(0xFF)

out = sys.argv[1] if len(sys.argv) > 1 else 'banktest.col'
open(out, 'wb').write(bytes(code))
print(f'{out} {len(code)} bytes, entry 8024h, {len(labels)} labels')
