#!/bin/sh
out=/media/fat/screenshots/AdamHome; mkdir -p $out; rm -f $out/*.png
shot() {
  touch /tmp/shotmark; sleep 1
  echo screenshot > /dev/MiSTer_cmd; sleep 2
  f=$(find /media/fat/screenshots -newer /tmp/shotmark -name "*.png" ! -path "*/AdamHome/*" ! -path "*/AdamTests*" | tail -1)
  if [ -n "$f" ]; then mv "$f" "$out/$1.png"; echo "$1 ok"; else echo "$1 NO SHOT"; fi
}
echo "load_core /media/fat/_AdamTests/A1 SmartWRITER.mgl" > /dev/MiSTer_cmd
sleep 28
shot 1_booted
python3 /media/fat/vkbd.py esc; sleep 4
shot 2_wordproc
python3 /media/fat/vkbd.py --gap 0.15 h e l l o space w o r l d; sleep 4
shot 3_typed
python3 /media/fat/vkbd.py 102; sleep 4
shot 4_after_home
