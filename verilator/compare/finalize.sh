#!/bin/bash
# finalize.sh OUTROOT [REPORTDIR]
# Re-score every REVIEW cartridge against a +/-60 frame ColEm window, summarize again
# (timing drift becomes DRIFT), and build the report page (default OUTROOT/report).
# Optional OUTROOT/notes.json and findings.json add per-cartridge verdicts and findings.
set -u
OUT="$1"
REPORT="${2:-$OUT/report}"
HERE="$(cd "$(dirname "$0")" && pwd)"

python3 "$HERE/summarize.py" "$OUT" > /dev/null
python3 - "$OUT" <<'EOF' | xargs -0 -P 8 -I{} "$HERE/rescore.sh" {} 60
import json, os, sys
root = sys.argv[1]
for r in json.load(open(os.path.join(root, "summary.json"))):
    if r["status"] == "REVIEW":
        sys.stdout.write(os.path.join(root, r["dir"]) + "\0")
EOF
python3 "$HERE/summarize.py" "$OUT"
rm -rf "$REPORT"
python3 "$HERE/make_report.py" "$OUT" "$REPORT"
