#!/bin/sh
# usage: run_test.sh "MGL name" label t1 [t2 ...]   screenshot at t seconds after load
mgl="/media/fat/_AdamTests/$1.mgl"; label=$2; shift 2
out=/media/fat/screenshots/AdamTests; mkdir -p $out
[ -f "$mgl" ] || { echo "$label: missing $mgl"; exit 1; }
echo "load_core $mgl" > /dev/MiSTer_cmd
start=$(date +%s)
for t in "$@"; do
  now=$(( $(date +%s) - start )); [ $t -gt $now ] && sleep $((t-now))
  touch /tmp/shotmark; sleep 1
  echo screenshot > /dev/MiSTer_cmd; sleep 2
  f=$(find /media/fat/screenshots -newer /tmp/shotmark -name "*.png" ! -path "*/AdamTests/*" | tail -1)
  if [ -n "$f" ]; then mv "$f" "$out/${label}_${t}s.png"; echo "$label ${t}s ok"; else echo "$label ${t}s NO SCREENSHOT"; fi
done
