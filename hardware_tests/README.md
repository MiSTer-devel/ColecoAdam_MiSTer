# MiSTer hardware test kit

Scripts and MGL files used to test branch `adam-accuracy-fixes` on a DE10-Nano on 2026-09-13.
The results are in `HANDOFF.md`, section 3.

## Layout on the MiSTer

| Repository path | Copy to | What |
|---|---|---|
| `_AdamTests/*.mgl` | `/media/fat/_AdamTests/` | One MGL per checklist item; appears as a menu folder |
| `_AdamTests/run_test.sh`, `run_suite.sh` | `/media/fat/_AdamTests/` | Load an MGL and take screenshots; run the suite |
| `config/AdamT_*.CFG` | `/media/fat/config/` | OSD settings for each test set name |
| `vkbd.py`, `vpad.py` | `/media/fat/` | Virtual keyboard and Xbox 360 pad through `/dev/uinput` |

The MGLs load `_Computer/ColecoAdam_20260913_accuracy.rbf` and media from
`/media/fat/games/Adam/_accuracy_tests/`. Copy the files they name there under the short names
they use, for example `dkjr.dsk`, `trollstale.ddp`, `blank_tape.ddp`, `ramtest.rom`. Build the
RAM test cartridges with `verilator/compare/tools/adam_exp_ramtest.py`.

## How the MGLs work

- Each MGL sets `<setname same_dir="1">`, so its OSD settings come from its own
  `config/<setname>.CFG` and the normal `Adam.CFG` is untouched.
  - A CFG is 16 raw status bytes.
  - Byte 0 bit 4 selects 256K and bit 5 selects None (`status[5:4]`).
  - Byte 1 bit 4 selects Console mode (`status[12]`).
- The cartridge entry in `CONF_STR` is a plain `F`, so its MGL `index` is 0. Main_MiSTer matches
  `index` against the digit after `F` or `S`.
- Disk and tape boots add `<reset delay="2" hold="1"/>` after the mount. The ADAM only looks for
  boot media at reset, and an MGL mounts after the core has booted.

## Running from a PC

    ssh root@MISTER 'echo "load_core /media/fat/_AdamTests/A2 DK Jr disk boot.mgl" > /dev/MiSTer_cmd'
    ssh root@MISTER 'echo screenshot > /dev/MiSTer_cmd'
    ssh root@MISTER 'sh /media/fat/_AdamTests/run_suite.sh'

`run_suite.sh` saves named screenshots in `/media/fat/screenshots/AdamTests/`. Its tape read-back
step expects `blank_tape.ddp` to hold the file saved by an earlier tape save run.

## Things to know

- The ADAM smart keys I-VI are F1-F6, and STORE/GET is Page Down (evdev code 109).
- On the Xbox 360 pad mapping, `vpad.py a` is the ColecoVision right fire button.
- Screenshots of the RAM test cartridges came back stale: byte-identical 960x90 captures of
  SmartWRITER from 3 s to 20 s after loading. Judge those tests on the monitor instead. The other
  tests' screenshots were live.
- The MiSTer has no `pkill`.
