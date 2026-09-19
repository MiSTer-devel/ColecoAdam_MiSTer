#!/bin/sh
T=/media/fat/_AdamTests; R=$T/run_test.sh; K="python3 /media/fat/vkbd.py --settle 3"; P="python3 /media/fat/vpad.py --settle 3"
out=/media/fat/screenshots/AdamTests
mkdir -p /media/fat/screenshots/AdamTests_b2 && mv $out/*.png /media/fat/screenshots/AdamTests_b2/ 2>/dev/null
shot() { touch /tmp/shotmark; sleep 1; echo screenshot > /dev/MiSTer_cmd; sleep 2
  f=$(find /media/fat/screenshots -newer /tmp/shotmark -name "*.png" ! -path "*/AdamTests*" | tail -1)
  if [ -n "$f" ]; then mv "$f" "$out/$1.png"; echo "$1 ok"; else echo "$1 NO SCREENSHOT"; fi; }

# A4 read-back of the file saved earlier (tape still holds TEST)
$R "A4 Tape save blank" A4R_01_boot 40
$K esc; sleep 3; shot A4R_02_wordproc
$K 109; sleep 3; shot A4R_03_storeget
$K f6; sleep 3; shot A4R_04_get
$K f3; sleep 25; shot A4R_05_listing

# A7 Tape-Disk Verification (index 0), lowercase c
$R "A7 Tape-Disk Verify cart" A7_01_menu 20
$K c; sleep 4; shot A7_02_after_c
sleep 8; shot A7_03_later

# A6 Diagnostic: try pad A, then pad X
$R "A6 Diagnostic cart" A6a_01_title 20
$P a; sleep 4; shot A6a_02_after_a
sleep 40; shot A6a_03_later
$R "A6 Diagnostic cart" A6x_01_title 20
$P x; sleep 4; shot A6x_02_after_x
sleep 40; shot A6x_03_later

# RAM tests (index 0)
$R "R1 ramtest 64K"  R1 15 30
$R "R2 banks 64K"    R2 15 30
$R "R3 ramtest 256K" R3 15 30
$R "R4 banks 256K"   R4 15 30
$R "R5 ramtest None" R5 15 30
$R "R6 banks None"   R6 15 30

# Boots with reset after mount
$R "A2 DK Jr disk boot"       A2 30 60 90
$R "A3 Trolls Tale tape boot" A3 40 80 130
$R "T1 T-DOS 256K"            T1 40 80
$R "A5 Buck Rogers tape"      A5 60 120
echo BATCH3 DONE
