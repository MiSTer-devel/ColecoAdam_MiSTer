#!/bin/bash
# rescore.sh RESULTDIR [WIDE]
# Re-score a finished cartridge against +/-WIDE ColEm frames around each shot, using the
# core frames already saved in the top row of strip.png, so the core does not run again.
# Writes RESULTDIR/rescore.txt: shot, best ColEm frame, offset from the expected frame, scores.
set -u
D="$1"
WIDE="${2:-60}"

HERE="$(cd "$(dirname "$0")" && pwd)"
WORK="${WORK:-$HERE/work}"
COLEM=$WORK/colem/colem_ref
BIOS=$WORK/roms/coleco_bios.bin
CMP=$WORK/cmpframes
OFFSET=-1
SHOTS=(120 400 740 1000 1300)
PRESSES=("1@760:10" "fire1@900:10" "right@1050:40" "fire1@1150:10")

cart=$(sed -n 's/^cart=//p' "$D/result.txt")
W="$D/rescore"
rm -rf "$W"
mkdir -p "$W/core" "$W/colem"

for ((i = 0; i < ${#SHOTS[@]}; i++)); do
    ffmpeg -loglevel error -y -i "$D/strip.png" -vf "crop=256:192:$((i * 256)):0" -pix_fmt rgb24 -c:v ppm \
        "$W/core/shot_${SHOTS[$i]}.ppm"
done

colem_shots=()
for s in "${SHOTS[@]}"; do
    for ((f = s + OFFSET - WIDE; f <= s + OFFSET + WIDE; f++)); do (( f > 0 )) && colem_shots+=($f); done
done
colem_args=(--cart "$cart" --bios "$BIOS" --frames $((1300 + OFFSET + WIDE)) --outdir "$W/colem")
colem_args+=(--shots "$(IFS=,; echo "${colem_shots[*]}")")
for p in "${PRESSES[@]}"; do
    key=${p%@*}; rest=${p#*@}; frame=${rest%:*}; len=${rest#*:}
    colem_args+=(--press "$key@$((frame + OFFSET + 1)):$len")
done
"$COLEM" "${colem_args[@]}" > /dev/null 2>&1

: > "$D/rescore.txt"
for s in "${SHOTS[@]}"; do
    cands=()
    for ((k = 0; k <= WIDE; k++)); do
        frames_k=$((s + OFFSET - k))
        (( k > 0 )) && frames_k="$frames_k $((s + OFFSET + k))"
        for f in $frames_k; do
            cf=$(printf '%s/colem/frame_%05d.ppm' "$W" $f)
            [[ -f $cf ]] && cands+=("$cf")
        done
    done
    line=$("$CMP" score 0 0 "$W/core/shot_$s.ppm" "${cands[@]}")
    best=$(sed -E 's/^best=[^ ]*frame_0*([0-9]+)\.ppm .*/\1/' <<< "$line")
    scores=$(sed -E 's/^best=[^ ]* (match=[0-9.]+ fgmatch=[0-9.]+).*/\1/' <<< "$line")
    echo "shot=$s best_frame=$best offset=$((best - s - OFFSET)) $scores" >> "$D/rescore.txt"
done
rm -rf "$W"
