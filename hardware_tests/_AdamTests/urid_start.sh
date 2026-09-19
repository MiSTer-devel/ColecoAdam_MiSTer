#!/bin/sh
out=/media/fat/screenshots/AdamUrid; mkdir -p $out; rm -f $out/*.png
shot() {
  touch /tmp/shotmark; sleep 1
  echo screenshot > /dev/MiSTer_cmd; sleep 2
  f=$(find /media/fat/screenshots -newer /tmp/shotmark -name "*.png" ! -path "*/Adam[A-Z]*" | tail -1)
  if [ -n "$f" ]; then mv "$f" "$out/$1.png"; echo "$1 ok"; else echo "$1 NO SHOT"; fi
}
echo "load_core /media/fat/_AdamTests/C5 Uridium.mgl" > /dev/MiSTer_cmd
sleep 30
shot 1_menu
python3 /media/fat/vpad.py a; sleep 6
shot 2_after_start
sleep 8
shot 3_later
