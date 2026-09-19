#!/bin/bash
# screens.sh OUTROOT NAME FRAMES [simulator options...]
# Runs one scenario on the core alone, for programs ColEm can't be the reference for (ADAM
# cartridges, test programs, odd image sizes). Saves every EVERY-th frame and a contact sheet, and
# reports how many bytes the run changed in each mounted image. The core writes to copies under
# OUTROOT/NAME/media, never to the originals.
#
#   EVERY     frame interval (default 60)
#   VEMU_DIR  directory Vemu runs from (default ..)
#
# Example: ./screens.sh work/screens buck 7200 --tape 1 "../SoftwareFromMiSTer/E.O.S/Games/x.ddp" --press fire1@4000:10
set -u
OUTROOT="$1"
NAME="$2"
FRAMES="$3"
shift 3
HERE="$(cd "$(dirname "$0")" && pwd)"
: "${VEMU_DIR:=$HERE/..}" "${EVERY:=60}"
mkdir -p "$OUTROOT"
D="$(cd "$OUTROOT" && pwd)/$NAME"
rm -rf "$D"
mkdir -p "$D/frames" "$D/media"

# The core runs from VEMU_DIR, so make paths absolute, and give it copies of the media
args=()
media=()
while (( $# )); do
    case "$1" in
        --disk|--tape)
            src="$(cd "$(dirname "$3")" && pwd)/$(basename "$3")"
            dst="$D/media/${1#--}$2-$(basename "$3")"
            cp "$src" "$dst"
            media+=("$src|$dst")
            args+=("$1" "$2" "$dst")
            shift 3
            ;;
        --cart)
            args+=(--cart "$(cd "$(dirname "$2")" && pwd)/$(basename "$2")")
            shift 2
            ;;
        *)
            args+=("$1")
            shift
            ;;
    esac
done

R="$D/result.txt"
{
    echo "name=$NAME frames=$FRAMES every=$EVERY"
    printf 'args='; printf ' %q' "${args[@]}"; echo
} > "$R"
start=$(date +%s)
( cd "$VEMU_DIR" && ./obj_dir/Vemu --headless --frames "$FRAMES" --every "$EVERY" --outdir "$D/frames" "${args[@]}" ) > "$D/core.log" 2>&1
echo "core_exit=$? core_seconds=$(( $(date +%s) - start )) $(grep '^frames=' "$D/core.log")" >> "$R"
for m in ${media[@]+"${media[@]}"}; do
    src=${m%%|*}
    dst=${m#*|}
    echo "media=$(basename "$dst") changed_bytes=$(cmp -l "$src" "$dst" 2>/dev/null | wc -l | tr -d ' ')" >> "$R"
done

shots=$(( FRAMES / EVERY ))
ffmpeg -loglevel error -y -pattern_type glob -i "$D/frames/frame_*.ppm" \
    -vf "scale=160:120,tile=10x$(( (shots + 9) / 10 ))" -frames:v 1 "$D/contact.png"
echo "done" >> "$R"
cat "$R"
