#!/bin/bash
# setup.sh -- one-time setup for the core vs ColEm comparison (safe to re-run).
#  - converts the core's BIOS, SmartWriter and EOS ROMs to binaries so both emulators run the same code
#  - builds cmpframes
#  - builds the ColEm 5.6 reference from a private, patched copy of ColEm56-Source
#  - records the core's no-cartridge screen (used to spot cartridges that never start)
#
# Environment: WORK (default ./work), COLEM_SRC (default <repo>/ColEm56-Source), VEMU_DIR (default ..)
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
WORK="${WORK:-$HERE/work}"
COLEM_SRC="${COLEM_SRC:-$HERE/../../ColEm56-Source}"
VEMU_DIR="${VEMU_DIR:-$HERE/..}"

if [ ! -x "$VEMU_DIR/obj_dir/Vemu" ]; then
    echo "Build the simulator first: make in $VEMU_DIR" >&2
    exit 1
fi
if [ ! -f "$COLEM_SRC/ColEm/Coleco.c" ]; then
    echo "ColEm 5.6 source not found at $COLEM_SRC (set COLEM_SRC)" >&2
    exit 1
fi
mkdir -p "$WORK/roms" "$WORK/colem"

echo "== ROMs"
xxd -r -p "$VEMU_DIR/rtl/bios.hex" > "$WORK/roms/coleco_bios.bin"
xxd -r -p "$VEMU_DIR/rtl/writer.hex" > "$WORK/roms/writer.bin"
# eos.hex holds 16 KB but the core only addresses the first 8 KB (cv_console.sv eos_rom_a_o is 13 bits)
xxd -r -p "$VEMU_DIR/rtl/eos.hex" | head -c 8192 > "$WORK/roms/eos.bin"

echo "== cmpframes"
clang++ -O2 -std=c++17 -o "$WORK/cmpframes" "$HERE/tools/cmpframes.cpp"

echo "== ColEm reference"
rm -rf "$WORK/colem/src"
mkdir -p "$WORK/colem/src"
for d in ColEm EMULib Z80; do cp -R "$COLEM_SRC/$d" "$WORK/colem/src/$d"; done
# Coleco.c opens COLECO.ROM, WRITER.ROM and EOS.ROM by fixed name; let the harness pass paths
perl -0pi -e '
    s/(const char \*HomeDir = 0;[^\n]*\n)/$1const char *BiosName = 0,*WriterName = 0,*EosName = 0;\r\n/ or die "HomeDir anchor\n";
    s/fopen\("COLECO\.ROM","rb"\)/fopen(BiosName? BiosName:"COLECO.ROM","rb")/ or die "COLECO.ROM anchor\n";
    s/fopen\("WRITER\.ROM","rb"\)/fopen(WriterName? WriterName:"WRITER.ROM","rb")/ or die "WRITER.ROM anchor\n";
    s/fopen\("EOS\.ROM","rb"\)/fopen(EosName? EosName:"EOS.ROM","rb")/ or die "EOS.ROM anchor\n";
' "$WORK/colem/src/ColEm/Coleco.c"
# ColEm centres Text mode 8 px in; the TMS9918A data manual (Table 3-3) gives a 19-pixel
# left border against 13 for the graphics modes, so the text area starts 6 px in
perl -0pi -e 's/(VDP->Width\/2-128)\+8;/$1+6;/ or die "RefreshLine0 anchor\n"' "$WORK/colem/src/EMULib/DRV9918.c"
# The game board holds WAIT for one clock of every M1 cycle (74LS74 U8 cleared by /M1, driving
# /WAIT through U7), which ColEm leaves out. Charge that clock on each opcode fetch (INCR counts
# them for R), on the CB of DD CB / FD CB, and on interrupt acknowledge. COLEM_M1_WAIT=0 skips it.
if [ "${COLEM_M1_WAIT:-1}" != 0 ]; then
    perl -0pi -e '
        s/(#define INCR\(N\)\s+)/$1R->ICount-=(N);/ or die "INCR anchor\n";
        s/R->ICount-=CyclesXXCB\[I\];/R->ICount-=CyclesXXCB[I]+1;/g == 2 or die "CyclesXXCB anchor\n";
        s/(if\(\(R->IFF&IFF_1\)\|\|\(Vector==INT_NMI\)\)\s*\{)/$1 R->ICount--;/ or die "IntZ80 anchor\n";
    ' "$WORK/colem/src/Z80/Z80.c"
fi
"$HERE/colem/build.sh" "$WORK/colem/src" "$WORK/colem"

echo "== core no-cartridge screen"
( cd "$VEMU_DIR" && ./obj_dir/Vemu --headless --console --frames 300 --shots 300 --outdir "$WORK" > "$WORK/nocart.log" 2>&1 )
mv "$WORK/frame_00300.ppm" "$WORK/nocart.ppm"

echo "setup done: $WORK"
