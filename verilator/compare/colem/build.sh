#!/bin/sh
# build.sh SRC OUT -- build OUT/colem_ref: headless ColEm 5.6 from SRC (the patched copy setup.sh makes)
# plus headless.c, which replaces ColEm's Unix/X11 front end. No window, no audio, no speed throttle.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$1"
OUT="$2"

CC=${CC:-clang}
DEFS="-DCOLEM -DLSB_FIRST -DZLIB -DBPP32"
INCS="-I$SRC/ColEm -I$SRC/EMULib -I$SRC/Z80"
# Sound.c calls InitAudio()/TrashAudio(), declared only in platform headers; headless.c provides them.
VENDOR_CFLAGS="-O2 -w -Wno-error=implicit-function-declaration $DEFS $INCS"
CFLAGS="-O2 -Wall $DEFS $INCS"

mkdir -p "$OUT/obj"
OBJS=""
for f in ColEm/Coleco.c ColEm/AdamNet.c Z80/Z80.c EMULib/TMS9918.c EMULib/DRV9918.c EMULib/SN76489.c \
         EMULib/AY8910.c EMULib/C24XX.c EMULib/CRC32.c EMULib/FDIDisk.c EMULib/Sound.c; do
    o="$OUT/obj/$(basename "$f" .c).o"
    $CC $VENDOR_CFLAGS -c -o "$o" "$SRC/$f"
    OBJS="$OBJS $o"
done
$CC $CFLAGS -c -o "$OUT/obj/headless.o" "$HERE/headless.c"
$CC -o "$OUT/colem_ref" $OBJS "$OUT/obj/headless.o" -lz
echo "built $OUT/colem_ref"
