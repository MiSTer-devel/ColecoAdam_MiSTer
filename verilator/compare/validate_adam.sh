#!/bin/bash
# validate_adam.sh OUTROOT NAME FRAMES [scenario options...]
# Runs one Adam-mode scenario on the Verilator core and on the ColEm reference, then
# matches every core frame against ColEm's whole run (Adam disk and tape I/O can run at
# different speeds in the two, so frame numbers do not always line up).
#
# Scenario options are passed to both emulators:
#   --disk N FILE  --tape N FILE  --cart FILE  --type TEXT@FRAME  --key NAME@FRAME  --press KEY@FRAME[:N]
# The core writes media back to the file, so it gets its own copies; ColEm never writes them.
#
# Leaves OUTROOT/NAME/result.txt, align.txt and strip.png (core row above ColEm's best matches).
set -u
# The core runs from VEMU_DIR, so make paths absolute first
mkdir -p "$1"
OUT="$(cd "$1" && pwd)"; NAME="$2"; FRAMES="$3"; shift 3
abspath() { echo "$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"; }

HERE="$(cd "$(dirname "$0")" && pwd)"
WORK="${WORK:-$HERE/work}"
: "${VEMU_DIR:=$HERE/..}"
: "${CORE_EVERY:=60}" "${COLEM_EVERY:=10}" "${COLEM_EXTRA:=600}" "${DX:=13}" "${DY:=26}"
COLEM=$WORK/colem/colem_ref
CMP=$WORK/cmpframes
ROMS=$WORK/roms

D="$OUT/$NAME"
rm -rf "$D"
mkdir -p "$D/core" "$D/colem" "$D/media"

core_args=()
colem_args=()
while (( $# )); do
    case "$1" in
        --disk|--tape)
            copy="$D/media/$1-$2-$(basename "$3")"
            cp "$3" "$copy"
            core_args+=("$1" "$2" "$copy")
            colem_file="$3"
            # ColEm only recognises a raw Adam disk of exactly 163840 bytes. Some images carry
            # one extra 512-byte block of E5 filler; ColEm rejects those and formats a blank disk.
            if [[ $1 == --disk ]] && (( $(stat -f %z "$3") == 164352 )) &&
               (( $(tail -c 512 "$3" | LC_ALL=C tr -d '\345' | wc -c) == 0 )); then
                colem_file="$D/media/colem-$2-$(basename "$3")"
                head -c 163840 "$3" > "$colem_file"
                echo "note=ColEm got a 163840-byte copy of $(basename "$3") (its last block is E5 filler)" >> "$D/notes.txt"
            fi
            colem_args+=("$1" "$2" "$colem_file")
            shift 3 ;;
        --cart)
            core_args+=("$1" "$(abspath "$2")")
            colem_args+=("$1" "$2")
            shift 2 ;;
        *)
            core_args+=("$1" "$2")
            colem_args+=("$1" "$2")
            shift 2 ;;
    esac
done

R="$D/result.txt"
{
    echo "name=$NAME frames=$FRAMES"
    printf 'args='; printf ' %q' "${colem_args[@]}"; echo
    [[ -f $D/notes.txt ]] && cat "$D/notes.txt"
} > "$R"

start=$(date +%s)
( cd "$VEMU_DIR" && ./obj_dir/Vemu --headless --adam --frames "$FRAMES" --every "$CORE_EVERY" --outdir "$D/core" "${core_args[@]}" ) > "$D/core.log" 2>&1
echo "core_exit=$? core_seconds=$(( $(date +%s) - start )) $(grep '^frames=' "$D/core.log")" >> "$R"

"$COLEM" --adam --writer "$ROMS/writer.bin" --eos "$ROMS/eos.bin" --bios "$ROMS/coleco_bios.bin" \
    --frames $((FRAMES + COLEM_EXTRA)) --every "$COLEM_EVERY" --outdir "$D/colem" "${colem_args[@]}" > "$D/colem.log" 2>&1
echo "colem_exit=$? $(grep '^frames=' "$D/colem.log")" >> "$R"

"$CMP" align "$DX" "$DY" "$D/core" "$D/colem" > "$D/align.txt"

# Summary: the last three core frames tell whether both ended on the same screen
python3 - "$D/align.txt" >> "$R" <<'EOF'
import sys
rows = []
for line in open(sys.argv[1]):
    kv = dict(t.split("=", 1) for t in line.split())
    rows.append((int(kv["core"]), int(kv["best"]), float(kv["match"]), float(kv["fgmatch"])))
if not rows:
    print("aligned=0")
    sys.exit()
tail = rows[-3:]
worst = min(rows, key=lambda r: r[3])
print("aligned=%d final_match=%.4f final_fg=%.4f final_offset=%+d worst_fg=%.4f worst_at=%d"
      % (len(rows), sum(r[2] for r in tail) / len(tail), sum(r[3] for r in tail) / len(tail),
         rows[-1][1] - rows[-1][0], worst[3], worst[0]))
EOF

# Strip: six core frames spread over the run, with ColEm's best match for each underneath
PICK=()
while IFS= read -r line; do PICK+=("$line"); done < <(python3 - "$D/align.txt" <<'EOF'
import sys
rows = [dict(t.split("=", 1) for t in l.split()) for l in open(sys.argv[1])]
n = len(rows)
for k in range(6):
    if n:
        r = rows[round(k * (n - 1) / 5)]
        print("%s %s" % (r["core"], r["best"]))
EOF
)
if (( ${#PICK[@]} == 6 )); then
    inputs=(); filter=""; top=""; bot=""
    for ((i = 0; i < 6; i++)); do
        read -r c e <<< "${PICK[$i]}"
        inputs+=(-i "$(printf '%s/core/frame_%05d.ppm' "$D" "$c")")
        filter+="[$i:v]crop=256:192:$DX:$DY[c$i];"
        top+="[c$i]"
    done
    for ((i = 0; i < 6; i++)); do
        read -r c e <<< "${PICK[$i]}"
        inputs+=(-i "$(printf '%s/colem/frame_%05d.ppm' "$D" "$e")")
        bot+="[$((i + 6)):v]"
    done
    filter+="${top}hstack=6[top];${bot}hstack=6[bot];[top][bot]vstack=2"
    ffmpeg -loglevel error -y "${inputs[@]}" -filter_complex "$filter" "$D/strip.png"
    echo "strip_frames=$(printf '%s,' "${PICK[@]}" | tr ' ' ':')" >> "$R"
fi

rm -rf "$D/core" "$D/colem" "$D/media"
echo "done" >> "$R"
