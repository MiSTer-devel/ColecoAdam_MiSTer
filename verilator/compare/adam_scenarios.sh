#!/bin/bash
# adam_scenarios.sh OUTROOT [JOBS]
# The Adam-mode scenarios, run in parallel through validate_adam.sh; then summarize_adam.py OUTROOT.
# Add a scenario with: scenario NAME FRAMES [options for both emulators]
set -u
OUT="$1"
JOBS="${2:-11}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ROMS_DIR="${ROMS_DIR:-$HERE/../roms colecovision}"
LIST="$OUT/scenarios.list"
mkdir -p "$OUT"
: > "$LIST"

# Arguments are NUL-separated so file names with spaces and apostrophes survive
scenario() { printf '%s\0' "$@" >> "$LIST"; printf '\n\0' >> "$LIST"; }

# SmartWriter prints each line on Return; ColEm drops keys typed while it does, so wait before typing on
scenario smartwriter_typing 1500 --type "HELLO ADAM@600" --key "enter@720" --type "The Quick Brown Fox 12345!@900"
scenario smartwriter_keys   1800 --type "ABC@600" --key "enter@640" --key "esc@700" --type "WORD PROCESSOR@900"
for f in "$ROMS_DIR"/*.dsk; do
    [ -e "$f" ] || continue
    n=$(basename "$f" .dsk | tr -c 'A-Za-z0-9' '_')
    scenario "disk_$n" 3000 --disk 1 "$f"
done
for f in "$ROMS_DIR"/*.ddp; do
    [ -e "$f" ] || continue
    n=$(basename "$f" .ddp | tr -c 'A-Za-z0-9' '_')
    scenario "tape_$n" 4000 --tape 1 "$f"
done

export VALIDATE="$HERE/validate_adam.sh" OUTROOT="$OUT"
python3 - "$LIST" <<'EOF' | xargs -0 -P "$JOBS" -n 1 bash -c 'eval "set -- $1"; "$VALIDATE" "$OUTROOT" "$@"' _
import shlex, sys
data = open(sys.argv[1], "rb").read().decode()
for block in data.split("\n\0"):
    parts = [p for p in block.split("\0") if p != ""]
    if parts:
        sys.stdout.write(" ".join(shlex.quote(p) for p in parts) + "\0")
EOF
