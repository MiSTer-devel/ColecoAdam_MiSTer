#!/bin/bash
# library_scenarios.sh OUTROOT DIR [JOBS] [FRAMES]
# Boots every disk and tape image under DIR (for example "../SoftwareFromMiSTer/E.O.S/Games")
# through validate_adam.sh; then summarize_adam.py OUTROOT.
# Alternate and bad dumps ([a1], [b1], [o1], ...) are skipped, as are images ColEm cannot read:
# it takes 160K disks (164,352-byte ones are trimmed for it) and 256K tapes.
set -u
OUT="$1"
DIR="$(cd "$2" && pwd)"
JOBS="${3:-11}"
FRAMES="${4:-3000}"
HERE="$(cd "$(dirname "$0")" && pwd)"
LIST="$OUT/scenarios.list"
mkdir -p "$OUT"
: > "$LIST"

# Arguments are NUL-separated so file names with spaces and apostrophes survive
scenario() { printf '%s\0' "$@" >> "$LIST"; printf '\n\0' >> "$LIST"; }

alternate='\[[abo][0-9]*\]'
count=0
skipped=0
while IFS= read -r -d '' f; do
    base=$(basename "$f")
    [[ $base =~ $alternate ]] && continue
    size=$(stat -f %z "$f")
    case "$base" in
        *.[dD][sS][kK]) kind=disk; frames=$FRAMES;            ok=$(( size == 163840 || size == 164352 )) ;;
        *)              kind=tape; frames=$((FRAMES + 1000)); ok=$(( size == 262144 )) ;;
    esac
    if (( ! ok )); then
        skipped=$((skipped + 1))
        continue
    fi
    count=$((count + 1))
    # numbered so names cut to length stay unique
    n=$(printf '%s' "${f#$DIR/}" | sed -E 's/\.[^.]*$//' | tr -c 'A-Za-z0-9' '_' | cut -c1-80)
    scenario "$(printf '%s_%04d_%s' $kind $count "$n")" "$frames" --$kind 1 "$f"
done < <(find "$DIR" -type f \( -iname '*.dsk' -o -iname '*.ddp' \) -print0 | sort -z)
echo "scenarios=$count skipped_sizes=$skipped"

export VALIDATE="$HERE/validate_adam.sh" OUTROOT="$OUT"
python3 - "$LIST" <<'EOF' | xargs -0 -P "$JOBS" -n 1 bash -c 'eval "set -- $1"; "$VALIDATE" "$OUTROOT" "$@"' _
import shlex, sys
data = open(sys.argv[1], "rb").read().decode()
for block in data.split("\n\0"):
    parts = [p for p in block.split("\0") if p != ""]
    if parts:
        sys.stdout.write(" ".join(shlex.quote(p) for p in parts) + "\0")
EOF
