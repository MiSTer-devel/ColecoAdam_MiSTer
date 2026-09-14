#!/bin/bash
# validate_cart.sh CART OUTROOT
# Runs one cartridge on the Verilator core and on the ColEm reference with the same
# scripted input, compares the frames, and leaves OUTROOT/<name>/result.txt + strip.png.
#
# Settings (environment):
#   WORK      setup.sh output (default ./work)
#   VEMU_DIR  directory Vemu runs from (it loads rtl/bios.hex relative to it)
#   OFFSET    ColEm frame = core frame + OFFSET
#   DX DY     position of the 256x192 display in the core's 320x240 frame
#   WIN       search +/- WIN ColEm frames around each shot
set -u
# The core runs from VEMU_DIR, so make paths absolute first
CART="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
mkdir -p "$2"
OUT="$(cd "$2" && pwd)"

HERE="$(cd "$(dirname "$0")" && pwd)"
WORK="${WORK:-$HERE/work}"
: "${VEMU_DIR:=$HERE/..}"
: "${OFFSET:=-1}" "${DX:=13}" "${DY:=26}" "${WIN:=8}"
COLEM=$WORK/colem/colem_ref
BIOS=$WORK/roms/coleco_bios.bin
CMP=$WORK/cmpframes
NOCART=$WORK/nocart.ppm

SHOTS=(120 400 740 1000 1300)
FRAMES=1300
# core frame numbers; shifted by OFFSET+1 for ColEm (it applies a press one frame earlier)
PRESSES=("1@760:10" "fire1@900:10" "right@1050:40" "fire1@1150:10")

name=$(basename "$CART")
name="${name%.*}"
safe=$(printf '%s' "$name" | tr -c 'A-Za-z0-9._-' '_')
D="$OUT/$safe"
rm -rf "$D"
mkdir -p "$D/core" "$D/colem"
R="$D/result.txt"
{
    echo "cart=$CART"
    echo "size=$(stat -f %z "$CART") header=$(xxd -p -l2 "$CART")"
} > "$R"

# Core
core_args=(--headless --console --cart "$CART" --frames $FRAMES --outdir "$D/core")
core_args+=(--shots "$(IFS=,; echo "${SHOTS[*]}")")
for p in "${PRESSES[@]}"; do core_args+=(--press "$p"); done
start=$(date +%s)
( cd "$VEMU_DIR" && ./obj_dir/Vemu "${core_args[@]}" ) > "$D/core.log" 2>&1
echo "core_exit=$? core_seconds=$(( $(date +%s) - start )) $(grep '^frames=' "$D/core.log")" >> "$R"

# ColEm, dumping a window of frames around each shot
colem_shots=()
for s in "${SHOTS[@]}"; do
    for ((f = s + OFFSET - WIN; f <= s + OFFSET + WIN; f++)); do (( f > 0 )) && colem_shots+=($f); done
done
colem_args=(--cart "$CART" --bios "$BIOS" --frames $((FRAMES + OFFSET + WIN)) --outdir "$D/colem")
colem_args+=(--shots "$(IFS=,; echo "${colem_shots[*]}")")
for p in "${PRESSES[@]}"; do
    key=${p%@*}; rest=${p#*@}; frame=${rest%:*}; len=${rest#*:}
    colem_args+=(--press "$key@$((frame + OFFSET + 1)):$len")
done
"$COLEM" "${colem_args[@]}" > "$D/colem.log" 2>&1
echo "colem_exit=$? $(tail -1 "$D/colem.log")" >> "$R"

# Compare each shot with its ColEm window
tiles_core=()
tiles_colem=()
for s in "${SHOTS[@]}"; do
    c=$(printf '%s/core/frame_%05d.ppm' "$D" $s)
    # nearest to the expected frame first, so identical static screens pick the closest one
    cands=()
    for ((k = 0; k <= WIN; k++)); do
        frames_k=$((s + OFFSET - k))
        (( k > 0 )) && frames_k="$frames_k $((s + OFFSET + k))"
        for f in $frames_k; do
            cf=$(printf '%s/colem/frame_%05d.ppm' "$D" $f)
            [[ -f $cf ]] && cands+=("$cf")
        done
    done
    if [[ -f $c && ${#cands[@]} -gt 0 ]]; then
        line=$("$CMP" score $DX $DY "$c" "${cands[@]}")
        best=$(sed -E 's/^best=([^ ]*) .*/\1/' <<< "$line")
        nocart=$("$CMP" score $DX $DY "$c" "$NOCART" | sed -E 's/.* match=([0-9.]*) .*/\1/')
        echo "shot=$s $line nocart_match=$nocart" >> "$R"
        tiles_core+=("$c")
        tiles_colem+=("$best")
    else
        echo "shot=$s missing" >> "$R"
    fi
done

# Did the picture change between the last two shots?
last=$(printf '%s/core/frame_%05d.ppm' "$D" ${SHOTS[4]})
prev=$(printf '%s/core/frame_%05d.ppm' "$D" ${SHOTS[3]})
if [[ -f $last && -f $prev ]]; then
    echo "still_match=$("$CMP" score $DX $DY "$last" "$prev" | sed -E 's/.* match=([0-9.]*) .*/\1/')" >> "$R"
fi

# Strip: core shots on top, best ColEm matches underneath
n=${#tiles_core[@]}
if (( n == ${#SHOTS[@]} )); then
    inputs=()
    filter=""
    for ((i = 0; i < n; i++)); do
        inputs+=(-i "${tiles_core[$i]}")
        filter+="[$i:v]crop=256:192:$DX:$DY[c$i];"
    done
    for ((i = 0; i < n; i++)); do inputs+=(-i "${tiles_colem[$i]}"); done
    top=""; bot=""
    for ((i = 0; i < n; i++)); do top+="[c$i]"; bot+="[$((i + n)):v]"; done
    filter+="${top}hstack=$n[top];${bot}hstack=$n[bot];[top][bot]vstack=2"
    ffmpeg -loglevel error -y "${inputs[@]}" -filter_complex "$filter" "$D/strip.png"
fi

rm -rf "$D/core" "$D/colem"
echo "done" >> "$R"
