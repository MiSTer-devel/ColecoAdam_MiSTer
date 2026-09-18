# ColecoAdam MiSTer core

MiSTer FPGA core for the Coleco ADAM computer and the ColecoVision console: a SystemVerilog
conversion of Arnim Laeuger's ColecoVision core with AdamNet (keyboard, disks, tapes) adapted
from ColEm. `STATUS.md` records where the testing and bug fixing currently stand, and `TODO.md`
is the work list. `HANDOFF.md` is the hardware test plan for branch `adam-accuracy-fixes`: what
each fix changes, the checks to run on a MiSTer, and how to back a single fix out.
`docs/`, `ColEm56-Source/`, the software library and ROM collection stay local and are not in
git.

## Layout

| Path | What it is |
|---|---|
| `ColecoAdam.sv` | MiSTer top level: hps_io, OSD (`CONF_STR`), ROMs/RAM, video, audio |
| `sys/` | MiSTer framework, synced from upstream; don't edit |
| `ColecoAdam.qpf/.qsf`, `files.qip` | Quartus project; `releases/` holds built `.rbf` files |
| `rtl/cv_console.sv` | Console: tv80 Z80, M1 WAIT flip-flop, bus mux, VDP, SN76489, AY, controllers, AdamNet |
| `rtl/cv_addr_dec.sv` | Memory map (port 7Fh memory select, 3Fh EOS enable, 53h SGM RAM enable, MegaCart paging) and I/O decode |
| `rtl/cv_ctrl.sv` | Controller ports: keypad/joystick select, the port read mux, and the spinner strobe and interrupt. `cv_ctrl.vhd` is the older VHDL, no longer built |
| `rtl/cv_spinner.sv` | Turns a spinner device or an analog axis into the pin 7/9 signalling of Coleco's roller controllers |
| `rtl/cv_adamnet.sv` | AdamNet devices and the PS/2 to ADAM key tables |
| `rtl/track_loader_adam.sv` | Moves disk/tape blocks between AdamNet and the SD image |
| `rtl/vdp18v/` | TMS9918A (SystemVerilog). `rtl/vdp18/` is the older VHDL |
| `rtl/tv80/` | Z80 used by both hardware and simulation. `rtl/T80/` is unused |
| `rtl/{bios,writer,eos}.hex` | OS-7 BIOS, SmartWriter, EOS. `verilator/rtl/` has identical copies for the simulator |
| `verilator/` | Verilator simulator (see below) |
| `verilator/compare/` | Core vs ColEm comparison framework; read its `README.md` |
| `verilator/roms colecovision/` | Test cartridges plus a few `.dsk`/`.ddp` images |
| `verilator/SoftwareFromMiSTer/` | The MiSTer ADAM library: ~2,000 disks, ~170 tapes, ADAM test/diagnostic ROMs |
| `docs/` | Datasheets, schematics and manuals (see Hardware references) |
| `ColEm56-Source/` | ColEm 5.6 source, reference only. Its licence forbids redistribution: never commit it |

## Mode polarity (easy to get backwards)

`cv_console`/`cv_addr_dec` input `mode`: **1 = ColecoVision console, 0 = ADAM**. At reset,
`mode=1` selects OS-7 + RAM + cartridge (port 7Fh value 1111), `mode=0` selects SmartWriter.
`sim.v` sets `mode = ~adam`. `ColecoAdam.sv` passes `.mode(~mode)` where `mode = ~status[12]`
and the OSD reads "Mode,Computer,Console".

- Console mode is a ColecoVision: 1K of RAM mirrored through 6000-7FFFh, with 2000-5FFFh
  empty until port 53h bit 0 enables the Super Game Module's 24K.
- ADAM mode has 24K there. Loading a cartridge in ADAM mode sets `game_mode_i`, which acts as
  the ADAM's cartridge reset switch: it resets to OS-7 + 24K + cartridge. The OSD reset clears
  it and returns to SmartWriter (ADAM Technical Manual 2.3, 2.6). ADAM-only cartridges such as
  the diagnostics need this path.

## Simulator

Build and run from `verilator/` (the ROMs load by the relative path `rtl/*.hex`):

    make                                   # Verilator 5.x and SDL2
    ./obj_dir/Vemu                         # GUI, ADAM mode
    ./obj_dir/Vemu --headless --console --cart "roms colecovision/Frogger I.col" --frames 600 --shots 300,600 --outdir /tmp/f
    ./obj_dir/Vemu --headless --adam --disk 1 some.dsk --frames 3000 --every 60 --outdir /tmp/d
    ./obj_dir/Vemu --help

Options: `--cart`, `--console`/`--adam`, `--headless`, `--frames N`, `--shots F1,F2`,
`--every K`, `--outdir`, `--press KEY@FRAME[:N]` (controller 1), `--disk N FILE`,
`--tape N FILE`, `--type TEXT@FRAME`, `--key NAME@FRAME` (ADAM keyboard),
`--exp-ram 64|256|0` (memory expander, matching the OSD's Expansion RAM option),
`--spin STEPS@FRAME[:N]` and `--spin2` (roller/spinner, matching the OSD's Spinner option),
`--peek ADDR[:N]@FRAME` (print RAM bytes; the index is the Z80 address, except that console
mode mirrors its 1K so 7038h is index 6038h). Frames are saved as 320×240 PPMs.

Things to know:
- Headless speed is about 9 frames/s (1300 frames ≈ 150 s). Runs are single-threaded
  (`--threads` is slower), so run several in parallel.
- `--no-timing` in the Makefile is required: tv80 uses `<= #1`, which `--timing` makes very slow.
- `sim.v` ties `ce_10m7` to 1 on purpose. The sim clock *is* the 10.7 MHz rate, while hardware
  runs clk_sys at twice that with a clock enable. It was verified frame-identical; don't "fix" it.
- Per-access `$display` logging is behind `SIM_DEBUG` (commented line in the Makefile).
  Waveform tracing is off; add `--trace` back to the verilator command to get it.
- Reset is held while a cartridge download is in progress.
- Media: disks go on block devices 0-3, tapes on 4-7. Writes go back to the image file, so
  give the core a copy.
- `verilate.sh` is left over from another core and is not used.
- `lldb` can't attach to `Vemu` on the dev Mac ("attach failed"). Debug simulator crashes
  (exit 139) by reading the harness code or adding output.
- To see where Z80 code is looping, run headless with `SIM_ADDR_PROFILE=37`. It samples the
  address bus every 37 steps and prints the busiest addresses at exit. `SIM_SPR5_PROFILE=1`
  prints the VDP's fifth-sprite activity (games use that number as a scanline counter) and how
  often the VDP asserted its interrupt, which is what a program sitting in `HALT` is waiting for.
  `--peek` reads memory at a frame, `v:` for VRAM, which is how to look at the VDP's tables.
  To build a debug simulator without disturbing a sweep that is using `obj_dir`:
  `sed 's|obj_dir|obj_probe|g' Makefile > Makefile.probe && make -f Makefile.probe`. Both are
  gitignored. For AdamNet
  debugging, build with `ADAMNET_TRACE` (commented line in the Makefile). Build into a separate
  directory, e.g. a Makefile copy with `obj_dir` renamed, so runs using `obj_dir` aren't
  disturbed.
- The simulator's per-drive pointers into the Verilated model (`sd_lba`, `sd_buff_din`) are
  set up in `sim_main.cpp`. Disks are drives 0-3 and tapes 4-7, so every drive needs its own.

## Comparing against ColEm

`verilator/compare/` runs the core and a headless ColEm 5.6 on the same ROMs and input and
compares screens by TMS9918 colour index:

    cd verilator/compare && ./setup.sh     # again after every simulator rebuild
    ./run_carts.sh work/carts 12 && ./finalize.sh work/carts    # ~40 min
    ./adam_scenarios.sh work/adam 11 && python3 summarize_adam.py work/adam

`VEMU_DIR` points the scripts at another simulator build. It must be a directory with
`obj_dir/Vemu` and an `rtl` link to `verilator/rtl`, which makes before/after comparisons easy.

ColEm is a reference, not ground truth. It holds only one pending key, drops keys while
SmartWriter prints, only accepts 163,840-byte disks, and gets some cartridges wrong. Where the
two disagree, settle it with the datasheet, schematics or ADAM Technical Manual before changing
RTL. The same applies to ColEm patches. `setup.sh` makes two, each backed by documentation:
- Text mode drawn at +6 px instead of +8, per the datasheet.
- One extra T-state per M1 cycle, the game board's WAIT flip-flop (`COLEM_M1_WAIT=0` turns it
  off). With it the core and ColEm stay frame-exact.

## Hardware references

- `docs/TMS9918A-TMS9928A-TMS9929A_Video_Display_Processors.pdf`: Table 3-3 horizontal
  timing (Text mode borders 19/25 px, graphics 13/15), Table 2-2 VRAM access windows.
- ADAM Technical Manual (memory map, ports, AdamNet):
  https://web.archive.org/web/20130521155109/http://drushel.cwru.edu/atm/atm.html
  (WebFetch is blocked for web.archive.org; use curl).
- `docs/adam_computer_schematics/ADAM Schematics/Original/`: clean redrawn schematics.
  `gameboard_working.pdf` is the ColecoVision-compatible game board (Z80, decoders, U8 M1 WAIT
  flip-flop); `adamboard_working.pdf` is the memory/EOS board. `Updated/` holds colour scans,
  the same set as `docs/adamschematics/`.
- `docs/colecovision_repair_manual/`: theory of operation and a parts list (1K×4 RAMs U3/U4,
  74LS138 U5/U6, 74LS74 U8), but no logic schematic.
- Manuals: https://adamarchive.org/archive.php?dir=%2FManuals%2FADAM+Hardware
- To read a schematic, crop it at high resolution and view the PNG:
  `pdftoppm -r 300 -x X -y Y -W 1000 -H 800 -png -singlefile in.pdf out`

## Working conventions

- Nothing is committed unless the user asks. Keep `ColEm56-Source/`, `verilator/compare/work/`,
  the software library, the ROM collection and test output out of commits.
- RTL comments state the hardware reason and cite the datasheet table or schematic part.
- The shell is zsh: unquoted `$var` does not word-split, and `echo =====` fails unquoted. Use
  absolute paths in commands that run in parallel, since they share one working directory.
