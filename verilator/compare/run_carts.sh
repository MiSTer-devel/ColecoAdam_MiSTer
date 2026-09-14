#!/bin/bash
# run_carts.sh OUTROOT [JOBS]
# Validate every .col/.rom cartridge in ROMS_DIR in parallel, then summarize.
# Follow with finalize.sh OUTROOT for the wide-window rescore and the report page.
set -u
OUT="$1"
JOBS="${2:-12}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ROMS_DIR="${ROMS_DIR:-$HERE/../roms colecovision}"

mkdir -p "$OUT"
start=$(date +%s)
find "$ROMS_DIR" -maxdepth 1 -type f \( -iname '*.col' -o -iname '*.rom' \) -print0 | sort -z |
    xargs -0 -P "$JOBS" -I{} "$HERE/validate_cart.sh" {} "$OUT"
echo "elapsed_seconds=$(( $(date +%s) - start ))" > "$OUT/elapsed.txt"
python3 "$HERE/summarize.py" "$OUT"
