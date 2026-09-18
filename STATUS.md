# Status

Where testing and bug fixing stood on 2026-09-13. The work is committed on branch
`adam-accuracy-fixes`.
`CLAUDE.md` explains how to build, run and compare.

## In short

- The Verilator simulator builds again. It runs about 2.4× faster, headless at roughly
  9 frames/s, and command-line options load cartridges, disks and tapes, script controller and
  keyboard input, and save frames.
- `verilator/compare/` checks the core frame by frame against a headless ColEm 5.6, for every
  cartridge in `roms colecovision` and for ADAM scenarios (SmartWriter typing, disk and tape boots).
- Eleven changes are in the RTL, each backed by the TMS9918A data manual, the ADAM
  Technical Manual, the ADAM schematics or ColEm/MESS:
  - RAM mirroring
  - Text mode position
  - M1 WAIT
  - cartridge reset
  - the expander's upper half
  - 256K expansion RAM
  - port range decoding
  - tape writes
  - AdamNet memory gating
  - PCB relocate
  - DCB status write-back

  All are confirmed in simulation, except that nothing yet exercises PCB relocate. The
  hardware test plan is in `HANDOFF.md`.
- A simulator harness bug that crashed on any write to drives 2-7 is fixed as well.
- One cartridge difference is still unexplained (Cosmo Fighter II's missing star field). Two ADAM
  test cartridges show a black screen, before and after the fixes.
- Built with Quartus 17.0.2 and tried on a DE10-Nano on 2026-09-13. Every check that was
  scripted passed; the RAM test cartridges still need checking by eye. Results are in
  `HANDOFF.md` section 3a.

## Test results

The fixed build against ColEm (with its Text-mode and M1 patches), compared with the run before
the fixes:

| Suite | Before | After |
|---|---|---|
| 179 cartridges, Console mode, 1300 frames with scripted input | 164 PASS, 3 DRIFT, 12 REVIEW | 170 PASS, 2 DRIFT, 7 REVIEW |
| 12 ADAM scenarios (SmartWRITER typing and keys, 9 disks, 1 tape) | 10 MATCH, 1 CLOSE, 1 DIFFERS | 12 MATCH |

- **Now passing:**
  - Cartridges: Super Cobra, Crown Jewels I-III, Power Lords, Aquattack, Kevtris, Slither.
  - ADAM scenarios: Adam Links Golf and SmartWRITER typing now match.
- **Still REVIEW:**
  - Cosmo Fighter II (open, below).
  - Artillery Duel, Jungle Hunt, Wizmath, Bejeweled and Blockade Runner (not core bugs, below).
  - Steamroller: the core shows the right title screen at every compared frame, but ColEm
    shows a plain green screen. This is ColEm's fault: with the M1 patch it fails whichever
    core build it is compared with, and without the patch it matches the build without the M1
    fix on all five frames.
- **DRIFT:** Evolution and Smurf Paint 'n Play Workshop show the same screens as ColEm, 27 and
  48 frames apart.
- **ADAM test cartridges** (loaded in Computer mode, so through the new cartridge reset; 900 frames):
  - Start: ADAM Demonstration, Coleco Graphics Processor, Resident Debugger 2.0, 64K Expansion
    RAM Test, Video RAM Test.
  - ADAM Diagnostic draws its test frame.
  - Black screen: System Hardware Test, Final Test 3.3.
- **No-cartridge check:** SmartWRITER, and Frogger in Console mode, are frame-identical with and
  without the cartridge-reset change. The suites ran on the build before that change.

The cartridge report with every strip is in `verilator/compare/work/carts/report/index.html`,
published privately at https://claude.ai/code/artifact/7b51e7a0-f8a9-4861-95a8-bae19dc71758.

## Bugs fixed

### 1. Console mode had 24K of unmirrored RAM instead of a ColecoVision's 1K

- **Symptom:** Super Cobra drew garbage on its opening screen and its attract demo went wrong.
- **Hardware:** a ColecoVision has 1K of RAM (two 1K×4 RAMs, U3/U4 in the repair manual's parts
  list), decoded by a 74LS138 at 6000-7FFFh and repeated through that range. Games write through
  any mirror. The ADAM has 24K at 2000-7FFFh (ADAM Technical Manual 2.1), and so does the Super
  Game Module once port 53h enables it.
- **Fix:** `rtl/cv_addr_dec.sv` and `rtl/cv_console.sv`. In Console mode with the OS-7 map and no
  SGM RAM enabled, RAM answers only at 6000-7FFFh and is addressed modulo 1K; 2000-5FFFh reads
  FFh. A new register at port 53h (bit 0, cleared on reset) enables the SGM's 24K. Computer mode
  is unchanged.
- **Check:** ColEm given unmirrored RAM had reproduced the core's broken Super Cobra pixel for
  pixel. After the fix Super Cobra matches ColEm on every compared frame.

### 2. Text mode was drawn 6 pixels too far left

- **Symptom:** 40-column screens (Search for the Stolen Crown Jewels I-III, Adam Links Golf)
  never matched.
- **Hardware:** TMS9918A data manual Table 3-3 gives Text mode a 19-pixel left border and
  25-pixel right border, against 13 and 15 in the graphics modes. The core used the graphics sync
  and blanking positions in every mode.
- **Fix:** `rtl/vdp18v/vdp18_hor_vert.sv` moves horizontal sync and blanking 6 pixels earlier in
  Text mode.
- **Reference:** ColEm puts Text mode 8 px in, which disagrees with the data manual, so
  `setup.sh` patches it to 6.
- **Check:** Crown Jewels I and Adam Links Golf now match. Golf went from 0.00 to 0.9993 of
  foreground pixels.

### 3. The M1 WAIT state was never inserted

- **Hardware:** the game board holds the Z80's /WAIT for one clock during every opcode fetch.
  74LS74 U8 has /Q fed back to D, is cleared while /M1 is high (through U22, 74LS04), and drives
  /WAIT through U7 (74LS05). See `docs/adam_computer_schematics/ADAM Schematics/Original/gameboard_working.pdf`.
  The ADAM Technical Manual describes /WAIT as inserting "extra clock cycles into Z80 timing
  during opcode fetch cycles".
- **Bug:** the SystemVerilog conversion inverted the flip-flop's clear, holding it cleared
  whenever the CPU ran, and replaced the toggle with a constant. The original VHDL is still in
  the comments.
- **Fix:** `rtl/cv_console.sv` restores both. tv80 samples WAIT in T2, so every M1 cycle gains
  exactly one T-state.
- **Reference:** ColEm leaves this out, so the fixed core ran about 7 frames behind it by
  frame 120. `setup.sh` now charges ColEm one T-state per M1 cycle (`COLEM_M1_WAIT=0` turns that
  off). With both changes, Super Cobra, Crown Jewels, Frogger and Artillery Duel's opening are
  frame-exact again, which shows the two agree on the added time.

### 4. ADAM-only cartridges had no proper way to start

- **Problem:** fix 1 exposed this. ADAM diagnostic and utility cartridges need an ADAM, and
  they had been running in Console mode only because it had 24K of RAM. After fix 1, Video RAM
  Test showed a black screen in Console mode.
- **Hardware:** the ADAM's cartridge reset switch starts in game mode, with OS-7 + 24K of RAM
  below the cartridge (ADAM Technical Manual 2.6). The computer reset switch starts SmartWRITER.
- **Fix:** `ColecoAdam.sv`, `verilator/sim.v`, `cv_console.sv` and `cv_addr_dec.sv`. Loading a
  cartridge in Computer mode now acts as the cartridge reset switch. The OSD reset (or a mode
  change) clears it and returns to SmartWRITER, with the cartridge still readable.
- **Check:**
  - ADAM Demonstration, Coleco Graphics Processor, Resident Debugger 2.0, 64K Expansion RAM
    Test and Video RAM Test all start.
  - SmartWRITER with no cartridge, and Frogger in Console mode, are frame-identical to the
    build without this change.
- **Behaviour change:** a cartridge loaded in Computer mode used to be ignored until software
  switched to it; now it starts.

### 5. The upper half of the 64K Memory Expander was missing (found 2026-09-13)

- **Bug:** port 7Fh upper bits `10` select 32K Expansion RAM (ADAM Technical Manual 2.2).
  `cv_addr_dec.sv` decoded it, but no RAM was connected and the bus mux ignored it, so the
  window read FFh. Only the lower half existed.
- **Fix:** a second 32K RAM in `ColecoAdam.sv` and `verilator/sim.v`, wired through
  `cv_console.sv`. ColEm models both halves.
- **Check:** `verilator/compare/tools/adam_exp_ramtest.py` builds a cartridge that
  write-verifies both halves. It shows red on the build without the fix and green with it.
  SmartWRITER, Frogger and a DK Jr disk boot are frame-identical.

### 6. Memory map and EOS/net reset ports were decoded as single addresses (found 2026-09-13)

- **Bug:** the core answered only 7Fh and 3Fh. The ADAM memory board's MIOC (U7) sees only
  BA13-BA15 and the game board's AUX DECODE lines, so the real machine answers 60h-7Fh and
  20h-3Fh. ColEm and MESS decode the same ranges, and Coleco's own 64K Expansion RAM Test
  writes ports 60h and 20h.
- **Fix:** range decoding in `cv_addr_dec.sv`.
- **Check:** no regressions in the same three titles.

### 7. Tape writes were never implemented (found 2026-09-13; verified in simulation)

- **Bug:** `cv_adamnet.sv` handled tapes with its own copy of the disk read states, whose
  write state was just `$display("Write not supported"); $finish;`. On hardware `$finish` does
  nothing, so a tape save left AdamNet stuck. That is the likely cause of Buck Rogers Super
  Game hanging on its high score table.
- **Fix:** tapes now use the disk states, whose writes the July fix verified on hardware,
  without the disk sector interleave.
- **Check:** tape reads are unchanged (Troll's Tale still matches ColEm, the others are
  frame-identical). SmartWRITER STORE to a blank tape now completes: the image changes by 1,050
  bytes and holds the file name and the document text. Booting again with that tape, GET shows
  the file in the directory.
  - The old build logs "Tape G: Writing 1024 bytes" and stops at "Write not supported".
  - The first fixed run exposed a simulator harness bug: `sim_main.cpp` connected the block
    device's write buffer only for drives 0 and 1, so a tape write crashed the simulator.
    That is fixed too.

### 8. AdamNet responded to memory the ADAM hadn't mapped in (found 2026-09-13)

- **Bug:** `cv_adamnet.sv` treated every CPU access to the PCB/DCB addresses (FEC0h and up)
  as AdamNet traffic, whatever memory was mapped there. On an ADAM the 6801 master reads the
  PCB from the Z80's memory as mapped, so expansion RAM or a cartridge at FExxh isn't AdamNet.
  Software that writes those addresses with other memory mapped in, such as a RAM disk,
  would send the core bogus commands.
- **Fix:** `cv_console.sv` passes AdamNet's strobes only when intrinsic RAM is selected.
- **Check:** SmartWRITER typing, a DK Jr disk boot, Frogger and the expander test cartridge are
  frame-identical to the build without it, and Troll's Tale still matches ColEm.

### 9. Expansion RAM beyond 64K (added 2026-09-13)

- **Hardware:** MicroFox and Lundy Electronics sell 256K-1MB expanders. Software selects a 64K
  bank by writing its number to port 42h (MESS adam.c), then uses the ordinary expansion windows
  of port 7Fh. Both RAMTEST v2.0 and the T-DOS system tracks do exactly this.
- **Change:** an OSD option "Expansion RAM" with 64K (default, bank 0 only), 256K and None;
  simulator flag `--exp-ram`. It adds a port 42h register and one 256K RAM in place of the two
  32K halves.
- **Check:**
  - `verilator/compare/tools/adam_exp_ramtest.py` passes at 64K and 256K.
  - Its `--banks` mode passes only at 256K.
  - "None" fails.
  - Other titles are frame-identical.
  - T-DOS boots with 256K.
- **Not done:** 512K/1MB, which need SDRAM. The extra 1.5 Mbit of BRAM also needs a Quartus
  build to confirm it fits.

### 10. AdamNet PCB relocate stopped the simulation (found 2026-09-13)

- **Bug:** PCB command 03h (relocate) called `$finish`, which ends a simulation and would hang
  AdamNet on hardware. ColEm's `WritePCB` moves the PCB to the address just written and resets
  its DCBs.
- **Fix:** `cv_adamnet.sv` now runs its `MOVE_PCB` state for 03h, keeping the DCB count and
  acknowledging with 83h. PCB commands 00h and 80h+, which software writes routinely and ColEm
  ignores, no longer log "Unimplemented PCB Operation".
- **Check:** SmartWRITER, DK Jr, Frogger and the Tape-Disk Verification cartridge are
  frame-identical, and Troll's Tale matches ColEm. None of them relocates, so the new path is
  still untested.

### 11. ADAM cartridges never received keys (found 2026-09-13)

- **Bug:** cartridges that drive AdamNet themselves (no EOS) clear a finished command by writing
  00h to the DCB status byte, and start the next command only when they read 00h back. The core
  serves DCB reads from its own table, but updated the status byte only for command writes
  (01h-7Fh). The Tape-Disk Verification cartridge therefore kept reading a stale 80h, believed
  a key was waiting, and never issued a keyboard read. ColEm keeps DCBs in RAM, so its 00h reads
  back.
- **How it was found:**
  - An `ADAMNET_TRACE` build.
  - A `SIM_ADDR_PROFILE` address profile, which showed the program blinking its menu.
  - Disassembly of the key reader at 2F00h.
- **Fix:** `cv_adamnet.sv` stores DCB status writes of 00h and 80h+ so they read back.
- **Check:**
  - The trace shows the cartridge now issuing keyboard reads and both typed keys delivered to
    its buffer.
  - With a lowercase `c`, the cartridge moves from its menu to "CHANGING CONFIGURATION".
  - SmartWRITER, DK Jr and Frogger are frame-identical, and Troll's Tale still matches ColEm.
- ADAM Diagnostic, a cartridge that uses controller buttons, now reaches its checkout menu and
  its "MEMORY MODULE TEST" screen in Computer mode.

### Spinner and roller controllers (2026-09-18)

- **Symptom:** nothing drove the speed roller of the Super Action Controller, the Roller
  Controller trackball or the Expansion Module #2 steering wheel, so those games could not be
  played. `cv_ctrl.sv` said so: "NOTE: The quadrature decoders are not implemented!"
- **What was there:** `cv_ctrl.vhd` did have the logic, and `files.qip` built the VHDL, so the
  FPGA had half the feature and the simulator none - and both tops tied controller pins 7 and 9
  high, so it could never fire either way.
- **The hardware:** all three controllers are one circuit, an optical encoder on pins 7 and 9
  (Expansion Module #2 Technical Guide, Theory of Operation III-1 and schematic V-1, in
  `docs/controllers/`). ADAM Technical Manual, "Controller Connector Pin Out": pin 7 reads back
  as D5, pin 9 is an "Indirect /INT input ... Strobe signal: typical 350 usec pulse width".
  Confirmed against OS-7 itself, whose spinner handler at BIOS 116Ah tests D4 for "was it this
  controller" and D5 for the direction, and against MAME, which does the same with a long
  readable strobe and a short interrupt.
- **Fix:** `rtl/cv_ctrl.sv` implements the strobe, the D4/D5 read-back and the interrupt, and
  `files.qip` now builds it instead of the VHDL so hardware and simulation share one file. New
  `rtl/cv_spinner.sv` produces the pin 7/9 signalling from a MiSTer spinner device or an analog
  stick axis, one per port, with an OSD "Spinner" option (Off, Spinner, Stick X, Stick XY).
- **Check:**
  - A unit test of the two modules: 350 µs strobe, 11 µs interrupt, correct D4/D5, and with no
    movement the port still reads 7Fh with /INT high, which is bit-identical to the old stub.
  - Bruce's Controller Tester, which counts through OS-7's own routine: 60 steps up counted as
    exactly +60, 60 down as exactly -60, and 100 steps on controller 2 left controller 1 alone.
  - Breakout for Roller Controller moves its paddle; the SAS roller test responds; Frogger is
    unchanged.

### The fifth sprite was lost whenever it was not the next one in the table (2026-09-18)

- **Symptom:** Uridium (2019) (Team Pixelboy) (SGM) drew its title and then sat on a green screen
  for ever. GitHub issue #12, reported in 2022, where rampa069 noted that a build with an
  F18A-style VDP fixed both this and Gauntlet.
- **Cause:** `vdp18_sprite.sv` stopped the sprite scan as soon as four visible sprites had been
  found (`if (sprite_idx_q == 4) stop_sprite_o = '1;`). A TMS9918A keeps reading the attribute
  table past entries that are not on the current line until it finds a fifth that is, or reaches
  the Y=208 terminator or sprite 31. So whenever the fifth sprite on a line was not the very next
  entry, the 5S flag and its sprite number were lost - and datasheet 2.3.3 puts that number in
  the low 5 bits of the status register, which games read in a tight loop as a scanline counter.
- **How it was found:** an address profile of the frozen phase alone (`SIM_ADDR_PROFILE_FROM`)
  put the CPU at 00ECh, `IN A,(BF) / AND 5F / CP 5C / JP C` - a wait for fifth-sprite number 28,
  followed by another for 29 at 00F7h. `SIM_SPR5_PROFILE` showed the engine reporting 4, 8, 12,
  16, 20 and 28: an evenly spaced ladder with 24 missing and 28 counted twice.
- **Fix:** stop only when that fifth sprite is actually visible.
- **Check:** the engine now reports 29 as well, and Uridium warps in and plays. Cosmo Fighter II
  is byte-identical before and after, so its star field is something else.

### Nothing waited for the SDRAM when reading the cartridge (2026-09-18)

- **Symptom:** Uridium and Gauntlet misbehave on hardware while playing correctly in simulation.
  Uridium corrupts its screen when a game starts; Gauntlet's maze breaks up into displaced bands
  when scrolling. The same Gauntlet breakup happens on the stock MiSTer ColecoVision core.
- **Cause:** on hardware the cartridge is in SDRAM. `ColecoAdam.sv` left the controller's
  `.ready()` output unconnected, while `cart_rd` and `cart_a_o` are combinational from the
  address decode and `cart_d_i` is muxed straight onto the CPU data bus. `sdram.sv` answers a
  new address only after its CAS latency, and later when an auto refresh is in the way, so a late
  read handed the Z80 whatever the previous access had left behind.
- **Why it hits these two:** a Z80 T-state is about twelve `clk_sys` cycles, so a read normally
  lands inside the M-cycle. A MegaCart streaming tile data across bank switches reads far harder
  than a 32K cartridge, which is where the margin runs out. The stock ColecoVision core keeps
  cartridges in SDRAM too, which fits the same breakup appearing there.
- **Fix:** `cv_console` takes a `cart_ready_i` and holds the CPU with the wait chain it already
  has for the PSG, the M1 flip-flop and AdamNet. `sdram.sv` keeps `ready` high when the byte is
  already in the latched 16 bit word, so consecutive bytes cost nothing.
- **Check:** none possible in simulation - the simulator keeps the cartridge in block RAM and
  ties `cart_ready_i` high, and Frogger's frames are identical either side of the change. This
  one can only be judged on hardware.

## Still open

- **Cosmo Fighter II's star field is missing.** ColEm draws about 100 dots a frame, the core
  0-3, because the game never writes them to VRAM. Ruled out:
  - CPU speed, including the M1 fix
  - VDP read-ahead and write-back timing (no stale reads or lost writes in 511,000 accesses)
  - VDP rendering
  - the RAM map
  - cartridge padding and mirroring

  Next idea: compare the CPU trace against ColEm up to the first star write.
- **System Hardware Test and ADAM Final Test 3.3 are not black after all** (checked 2026-09-18 in
  Computer mode, where an ADAM diagnostic belongs). Both run:
  - System Hardware Test draws its title and reports "FAIL CONTROLLER PORT #1", "FAIL AUX.
    VIDEO" and "FAIL AUX. AUDIO". **All three are expected**: this is a factory test that needs
    Coleco's manufacturing fixture. Its controller test (83D0h) writes a pattern to **port 09h**,
    which no ADAM decodes, waits, then reads controller 1 and compares against a table at 8724h -
    `7F 7E 7B 77 7D 3F 5F`, the idle value and then each controller line pulled low in turn by
    the fixture. With no fixture the port stays at 7Fh, so it times out after 32 tries and
    prints the failure. Port 2 is not "passing" either: the handler at 85B3h prints the message
    and jumps past the port 2 test. The AUX video and audio lines are the same kind of thing.
  - ADAM Final Test 3.3 draws "ADAM SYSTEM FINAL TEST REV 3.3" and waits at a "STATION ID -"
    prompt for keyboard input, which is why it looked dead.
- **ColEm can't be the reference for ADAM-only cartridges.** It switches to ColecoVision mode
  whenever a cartridge is loaded, so those are checked by eye.
- **Hardware checks still to do** (the rest passed on 2026-09-13, `HANDOFF.md` section 3a):
  - the RAM test cartridge colours at 64K, 256K and None, on the monitor, since screenshots
    of them came back stale
  - Buck Rogers' high score save, played to the end
  - a Crown Jewels game and an SGM title, which aren't on the test MiSTer's SD card
  - the rest of the ADAM Diagnostic checkout, OSD Reset back to SmartWRITER, and a T-DOS RAM disk
- **README's known bugs** (not re-checked): a bad character on the first keypress, reset not
  quite like an ADAM, no printer, key repeat, tape/disk writes. Disk writes were fixed in commit
  1029386.

## Differences from ColEm that are not core bugs

- **Artillery Duel, Jungle Hunt, Wizard of Id's Wizmath:** a colour chosen at random from CPU
  timing. ColEm's own choice changes when its CPU budget moves by one cycle per line. After the
  M1 fix Wizmath's border is cyan or red where ColEm's is grey, on otherwise identical screens.
- **Bejeweled:** ColEm misses a 10-frame fire press.
- **Power Lords:** ColEm's picture breaks up; the core's is clean.
- **Blockade Runner:** its colour-cycling border is at a different phase at one compared frame.
- **Aquattack and Kevtris** (these now pass): colour-cycling animations caught at a different
  phase.

## Changes in the working tree

| Area | Files | What |
|---|---|---|
| Hardware fixes | `rtl/cv_addr_dec.sv`, `rtl/cv_console.sv`, `rtl/vdp18v/vdp18_hor_vert.sv`, `ColecoAdam.sv` | The four fixes above |
| Spinner | `rtl/cv_ctrl.sv`, `rtl/cv_spinner.sv`, `files.qip`, `ColecoAdam.sv`, `verilator/sim.v` | Roller/spinner strobe, interrupt and signal generation; Quartus now builds `cv_ctrl.sv` rather than `cv_ctrl.vhd` |
| Simulator | `verilator/Makefile`, `verilator/sim.v`, `verilator/sim_main.cpp`, `verilator/sim/sim_video.*`, `verilator/sim/sim_adam_keys.h` | Build fixes for Verilator 5.044, `--no-timing`, no waveform dump, always-on 10.7 MHz enable, headless mode and command-line options, cartridge reset |
| Debug output | `rtl/bram.sv`, `rtl/dpramv.sv`, `rtl/cv_adamnet.sv`, `rtl/track_loader_adam.sv` | Per-access `$display` behind `SIM_DEBUG` |
| Comparison | `verilator/compare/` | Framework, ColEm harness and patches, cartridge/ADAM/library scripts, report builder |
| Docs | `CLAUDE.md`, `STATUS.md`, `docs/` | Project guide, this file, collected manuals and schematics |
| Hardware tests | `hardware_tests/` | MGL files, OSD configs and scripts for testing on a MiSTer |

Keep `ColEm56-Source/`, `verilator/compare/work/`, `verilator/SoftwareFromMiSTer/`,
`verilator/roms colecovision/`, `verilator/adam.tar` and the loose `.dsk`/`.zip` files in
`verilator/` out of commits.

## Software on hand

- `verilator/roms colecovision/`: 179 ColecoVision cartridges plus 9 ADAM disks and a tape;
  the cartridge and ADAM suites run all of them.
- `verilator/SoftwareFromMiSTer/` (428 MB), the MiSTer ADAM library:
  - `E.O.S/`: 1,964 files; `Games/` alone has 296 titles once alternate dumps are dropped.
  - `CP-M & T-DOS/`: 250 files.
  - Test and diagnostic cartridges in `adam_carts/` and `colrom/`.
  - Firmware dumps and SmartWRITER revisions in `Chip Sets/`.
  - `ADE SD Drive/`, `Boot PROM/`, blank media.
  - Most disks are 160K. Other sizes (320K, 720K, 1.44M, 8M) need drive support and ColEm
    can't read them.
- `verilator/compare/library_scenarios.sh` boots every title in a folder against ColEm, at about
  140 titles an hour with 12 jobs.

## Documentation collected

- `docs/TMS9918A-TMS9928A-TMS9929A_Video_Display_Processors.pdf`: TI data manual
- `docs/adam_computer_schematics/`: ADAM schematics, redrawn (`Original/`) and colour scans
  (`Updated/`, same as `docs/adamschematics/`)
- `docs/colecovision_repair_manual/`: ColecoVision Repair Technical Manual (theory of operation,
  assembly drawings, parts list)
- ADAM Technical Manual (online only, see `CLAUDE.md`) and the adamarchive.org hardware manuals

## Suggested next steps

The full work list, including tape writes, expansion RAM, the test programs and SuperADAM
features, is in `TODO.md`.

1. Finish the hardware checks listed under "Still open". The kit in `hardware_tests/` repeats
   the rest.
2. Run `library_scenarios.sh` over `E.O.S/Games` (about 2 hours), then the rest of `E.O.S/`
   and `CP-M & T-DOS/`, and triage whatever doesn't boot or match.
3. Chase Cosmo Fighter II with a CPU trace comparison.
4. Look at why System Hardware Test and Final Test stay black.
5. Commit in reviewable pieces: simulator, comparison framework, then each hardware fix on its
   own.
