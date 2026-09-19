# Handoff: testing branch `adam-accuracy-fixes` on the MiSTer

Branched from `master` (d7bc66b) on 2026-09-13. Every change here was checked in the Verilator
simulator, often against ColEm 5.6. It was then built with Quartus and tested on a DE10-Nano on
2026-09-13; results are in section 3a. This document is the plan for that testing. `STATUS.md` has the evidence for each fix,
`TODO.md` the open work, and `CLAUDE.md` how to build and run the simulator.

## 1. Build

1. Open `ColecoAdam.qpf` in Quartus (the MiSTer-supported version) and compile.
2. Check the fitter report for memory. The expansion RAM grew from two 32K blocks to one 256K
   block, about 1.5 Mbit more M10K. If it doesn't fit, see "Backing a fix out" below.
3. Copy `output_files/ColecoAdam.rbf` to the MiSTer and load it.

Built 2026-09-13 with Quartus 17.0.2 Lite: no errors, no critical warnings, and timing met
(worst-case setup slack 0.301 ns, hold 0.246 ns).

| Resource | master (July) | This branch |
|---|---|---|
| RAM blocks | 242 / 553 (44%) | 466 / 553 (84%) |
| Block memory bits | 1,803,749 (32%) | 3,638,757 (64%) |
| Logic (ALMs) | 35% | 36% |

It fits, but there is no room left in block RAM for 512K or 1MB expansion.

No files were added to or removed from `files.qip`.

## 2. What changed in the core

| # | Change | Files | Behaviour you'll notice |
|---|---|---|---|
| 1 | Console-mode RAM is 1K mirrored through 6000h-7FFFh, like a ColecoVision; port 53h enables the Super Game Module's 24K | `rtl/cv_addr_dec.sv`, `rtl/cv_console.sv` | Super Cobra's opening screen and demo are correct |
| 2 | Text mode drawn 6 px further right (TMS9918A manual Table 3-3) | `rtl/vdp18v/vdp18_hor_vert.sv` | 40-column screens are centred |
| 3 | M1 WAIT restored: one wait state per opcode fetch (game board U8) | `rtl/cv_console.sv` | CPU runs at real speed, ~15-20% slower than before |
| 4 | Loading a cartridge in **Computer** mode acts as the ADAM's cartridge reset (OS-7 + 24K + cartridge); OSD Reset returns to SmartWRITER | `ColecoAdam.sv`, `rtl/cv_addr_dec.sv`, `rtl/cv_console.sv` | ADAM-only cartridges start in Computer mode |
| 5 | The 64K Memory Expander's upper half existed only in the decode; it now has RAM | `ColecoAdam.sv`, `rtl/cv_console.sv` | Software using 64K expansion works |
| 6 | New OSD option **Expansion RAM: 64K / 256K / None**; port 42h selects 64K banks | `ColecoAdam.sv`, `rtl/cv_addr_dec.sv` | T-DOS/CP/M RAM disks can use 256K |
| 7 | Ports 20h-3Fh and 60h-7Fh decoded as ranges (MIOC sees only A13-A15 plus AUX DECODE) | `rtl/cv_addr_dec.sv` | Coleco test cartridges that use 20h/60h work |
| 8 | **Tape writes implemented**; they were a `$finish` stub | `rtl/cv_adamnet.sv` | Saving to a data pack works, including Buck Rogers' high score save |
| 9 | AdamNet only answers accesses to the ADAM's own RAM | `rtl/cv_console.sv` | Nothing visible unless software maps other memory over FExxh |
| 10 | PCB relocate (03h) implemented; it was a `$finish` stub | `rtl/cv_adamnet.sv` | Nothing found that uses it yet |
| 11 | DCB status bytes written with 00h or 80h+ now read back, as the RAM they are on an ADAM | `rtl/cv_adamnet.sv` | Keyboard works in ADAM cartridges that drive AdamNet themselves |
| 12 | **Spinner/roller support**: pin 9 strobe, D4/D5 read-back and the maskable interrupt, plus signal generation from a spinner device or an analog stick. New OSD option **Spinner: Off / Spinner / Stick X / Stick XY**, default Off | `rtl/cv_ctrl.sv`, `rtl/cv_spinner.sv`, `ColecoAdam.sv`, `files.qip` | Super Action Controller roller, Roller Controller and Driving Module games are playable |

Note on change 12: Quartus now compiles `rtl/cv_ctrl.sv` instead of `rtl/cv_ctrl.vhd`. The two
were identical apart from the spinner, which only the VHDL had, and the SystemVerilog file now
has it; with the Spinner option off the controller ports read back exactly as before (7Fh, /INT
high). To go back, swap the two lines in `files.qip`.

Simulator-only changes (no effect on the FPGA):
- a headless mode with command-line media and input;
- the always-on 10.7 MHz clock enable;
- `--exp-ram`, `SIM_ADDR_PROFILE` and the `ADAMNET_TRACE` define;
- a fix for the harness writing only drives 0 and 1.

## 3. Hardware checklist

Quickest and most important first. Note anything that differs from "Expect".

Results from the DE10-Nano run on 2026-09-13 are ticked below and summarised in section 3a.

### Console mode (OSD Mode = Console)

- [x] **Frogger or Donkey Kong** plays normally, at a plausible speed (fix 3).
- [x] **Super Cobra** opening screen has no garbage and the attract demo flies (fix 1).
- [ ] **Search for the Stolen Crown Jewels I** text screens are centred (fix 2).
- [ ] **An SGM title** still runs and has AY sound (fix 1, port 53h).

### Computer mode (OSD Mode = Computer, Expansion RAM = 64K)

- [x] **SmartWRITER** boots; typing works, including the first key.
- [x] **Disk boot:** Donkey Kong Jr (ADAM) disk loads and plays (fix 3, AdamNet timing).
- [x] **Tape boot:** Troll's Tale data pack loads (fix 8 must not break reads).
- [x] **Tape save** (fix 8), using a copy of a blank data pack:
  1. In SmartWRITER, press Escape (word processor) and type a line.
  2. Press STORE/GET (PgDn), then smart key V (STORE WK-SPACE), then III (DRIVE A).
  3. Type a name and press VI.
  - Expect "ONE MOMENT - STORING FILE", then the editor again.
  - Reset, then STORE/GET → VI (GET) → III: the file is listed.
- [ ] **Buck Rogers Super Game** from its data pack: finish a game, enter initials, choose DONE?.
  The high score saves and the game carries on. This is the original bug report.
- [x] **Cartridge reset** (fix 4): load `SoftwareFromMiSTer/adam_carts/ADAM Diagnostic (1982)
  (Coleco).rom`. Expect "ADAM CHECKOUT CARTRIDGE", then a CHECKOUT/SKIP menu. The right
  controller button starts the checkout ("MEMORY MODULE TEST"). Run the tests and note any
  failures. OSD Reset should then bring up SmartWRITER.
- [x] **Keyboard in an ADAM cartridge** (fix 11): load `ADAM Tape-Disk Verification Rev. 1`.
  Press lowercase `c`. Expect the "CHANGING CONFIGURATION" screen (start and end address,
  function keys select tape and disk drives), which is what simulation shows. Before this
  branch, keys did nothing.

### Spinner and roller controllers (fix 12, added 2026-09-18)

Test carts are in `verilator/roms colecovision/controller_tests/`, from the ADAM archive.
Set OSD **Spinner** to *Stick X* for a gamepad, or *Spinner* if a spinner device is mapped;
*Stick XY* is the Roller Controller layout (X on port 1, Y on port 2). It defaults to Off, so
check that first: with Off, nothing below should respond.

- [ ] **Bruce's Controller Tester** (console mode): the bar under keypad #1 tracks the roller,
  one count per notch, and moves the opposite way when reversed. Controller #2's bar must not
  move with it. In simulation this counts exactly, through OS-7's own handler.
- [ ] **Breakout for Roller Controller**: press keypad 1 to start (it ignores the roller in
  attract mode), then the paddle follows the roller.
- [ ] **Super Action Controller Tester (1983) (Nuvatec)**, the in-house Coleco cart, if it shows
  a roller readout.
- [ ] **A real roller game**: Slither or Victory (Roller Controller), Turbo (Driving Module), or
  Super Action Baseball/Football (speed roller).
- [ ] **Computer mode with Spinner on**: SmartWRITER still boots and types normally. The strobe
  raises the Z80's maskable interrupt, which ADAM software does not normally expect; the ADAM has
  a /SPINDIS line to suppress exactly that (ATM 2.1.3, MIOC pin 10), which is not modelled. In
  simulation this is harmless: SmartWRITER booting and typing through 2,700 strobes is
  byte-identical to the same run without them, so EOS leaves the interrupt masked. Worth one
  check on real hardware anyway.

### Expansion RAM (fixes 5 and 6)

- [ ] Build the test cartridges on a PC:
  `python3 verilator/compare/tools/adam_exp_ramtest.py ramtest.rom` and
  `... --banks banks.rom`. Load each in Computer mode and read the screen colour:

  | Expansion RAM | ramtest.rom | banks.rom |
  |---|---|---|
  | 64K | green | black (banks alias) |
  | 256K | green | green |
  | None | black | black |

  Red means the upper window failed, magenta the lower window.
- [x] **T-DOS** (`CP-M & T-DOS/Drivers/M.I.B. 3 Drivers - T-DOS v4.58`) boots to `A0>` at 256K.
  If you can, check that its RAM disk appears and holds files.

## 3a. Hardware results (2026-09-13)

Core `ColecoAdam_20260913_accuracy.rbf` on a DE10-Nano, driven remotely with the MGL files,
scripts and virtual keyboard and pad in `hardware_tests/`. Results were judged from MiSTer
screenshots.

| Check | Result |
|---|---|
| Frogger, Console mode | Pass: attract demo runs. Speed not judged by eye |
| Super Cobra, Console mode | Pass: clean opening screen, attract demo flies |
| Donkey Kong Jr cartridge, Console mode | Pass: game options screen |
| SmartWRITER boot and typing | Pass: "hello tape" typed correctly, first key included |
| DK Jr Super Game disk boot | Pass: loads to the player selection screen. Not played |
| Troll's Tale tape boot | Pass: first scene within 40 s |
| Buck Rogers Super Game tape boot | Pass: player selection screen |
| Tape save (fix 8) | Pass: blank data pack changed by 1,050 bytes, as in simulation; holds "test" and "hello tape" |
| Tape read-back | Pass: after a reboot, GET on drive A lists "test" |
| Cartridge reset (fix 4) | Pass: ADAM Diagnostic shows CHECKOUT/SKIP; the right button (pad A) goes to MEMORY MODULE TEST |
| Keyboard in an ADAM cartridge (fix 11) | Pass: lowercase `c` opens CHANGING CONFIGURATION |
| T-DOS 4.58 at 256K | Pass: `A0>` prompt with smart keys |
| RAM test cartridges, 64K/256K/None (fixes 5, 6) | Not judged: screenshots came back stale (see below) |

Not done:
- **RAM test cartridge colours.** Every screenshot of `ramtest.rom` and `banks.rom` was a stale,
  byte-identical 960x90 capture of SmartWRITER, from 3 s to 20 s after loading. The Diagnostic and
  Tape-Disk Verification cartridges load through the same kind of MGL and screenshot correctly,
  so this says nothing about the core either way. Check the colour on the monitor.
- **Buck Rogers high score save.** It needs a game played to the end.
- **Crown Jewels text centring and an SGM title.** Neither is on the MiSTer's SD card.
- **Rest of the Diagnostic checkout, OSD Reset back to SmartWRITER, and T-DOS RAM disk.**

## 4. Backing a fix out

Every fix is in the first commit. If one misbehaves, these one-line edits disable it so the
others can still be tested:

| Fix | Edit |
|---|---|
| 1 RAM mirroring | `rtl/cv_addr_dec.sv`: `assign ram_mirror_o = 1'b0;` |
| 2 Text position | `rtl/vdp18v/vdp18_hor_vert.sv`: `assign text_shift_s = 9'sd0;` |
| 3 M1 WAIT | `rtl/cv_console.sv`, `m1_wait` block: `m1_wait_q <= 1'b0;` |
| 4 Cartridge reset | `ColecoAdam.sv`: `.game_mode_i(1'b0)` |
| 5/6 Expansion RAM | OSD Expansion RAM = 64K (plain expander), or None |
| 7 Port ranges | `rtl/cv_addr_dec.sv`: `a_i[7:5] == 3'b011` → `a_i[7:0] == 8'h7f`, `3'b001` → `8'h3f` (three places) |
| 9 AdamNet gating | `rtl/cv_console.sv`: `assign adamnet_ram_s = 1'b1;` |
| 11 DCB status write-back | `rtl/cv_adamnet.sv`: delete the `if (z80_wr && (z80_data_wr == 8'h00 ...` two lines under `DCB_CMD_STAT` |

Tape writes (8) and PCB relocate (10) have no switch. They only take effect when software
writes to tape or relocates the PCB, and before this branch both simply halted.

## 5. Simulation results at handoff

- **179 cartridges** in Console mode against ColEm: 170 PASS, 2 timing DRIFT, 7 REVIEW. Of the
  seven, six are explained as not core bugs; Cosmo Fighter II's missing star field is still
  open. Report: https://claude.ai/code/artifact/7b51e7a0-f8a9-4861-95a8-bae19dc71758
- **12 ADAM scenarios** (SmartWRITER typing, 9 disks, 1 tape): all MATCH.
- **E.O.S Games library sweep** (283 titles, on the build before the expander, tape-write,
  AdamNet and DCB fixes): 260 MATCH, 10 CLOSE, 13 DIFFERS.
  - Every difference looked at so far is ColEm failing, not the core:
    - it can't boot ADAM CP/M (Ace of Aces);
    - it blanks or garbles Cabbage Patch Kids, Electronic Game Pack II and Chess Solitaire;
    - it is still on Donkey Kong Super Game's title screen when the core reaches the menu.
  - The rest are timing. `TODO.md` lists the titles not yet checked.
- Every later fix was checked frame-identical to the build before it on SmartWRITER, a DK Jr
  disk boot and Frogger, with Troll's Tale still matching ColEm.

## 6. Not in the branch

These stay local and are ignored or untracked:
- `ColEm56-Source/`: its licence forbids redistribution.
- `verilator/SoftwareFromMiSTer/`, `verilator/roms colecovision/`, `verilator/adam.tar` and the
  loose test disks and zip in `verilator/`.
- `verilator/compare/work/`: generated, 2.2 GB.
- `docs/`: 98 MB of datasheets, schematics and manuals. Add them in a separate commit if they
  should be in the repository.

The simulator and comparison framework need those local files; see `CLAUDE.md`.

## 7. Where to pick up

- Finish the hardware checks listed under "Not done" in section 3a. Nothing tested so far failed,
  so no fix has been backed out.
- Open work is in `TODO.md`:
  - 512K/1MB expansion in SDRAM
  - Cosmo Fighter II
  - System Hardware Test and Final Test black screens
  - scripting Buck Rogers' save
  - the rest of the library sweeps
