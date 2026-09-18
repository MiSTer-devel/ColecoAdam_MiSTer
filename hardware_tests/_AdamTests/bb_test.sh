#!/bin/sh
out=/media/fat/screenshots/AdamBB; mkdir -p $out; rm -f $out/*.png
shot() {
  touch /tmp/shotmark; sleep 1
  echo screenshot > /dev/MiSTer_cmd; sleep 2
  f=$(find /media/fat/screenshots -newer /tmp/shotmark -name "*.png" ! -path "*/AdamBB/*" ! -path "*/AdamHome/*" ! -path "*/AdamTests*" | tail -1)
  if [ -n "$f" ]; then mv "$f" "$out/$1.png"; echo "$1 ok"; else echo "$1 NO SHOT"; fi
}
echo "load_core /media/fat/_AdamTests/B1 Broderbund disk.mgl" > /dev/MiSTer_cmd
sleep 50
shot 1_menu
python3 /media/fat/vkbd.py 79; sleep 30
shot 2_after_select
python3 /media/fat/vkbd.py 79; sleep 25
shot 3_gameplay
