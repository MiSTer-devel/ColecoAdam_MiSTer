# TODO

Work list as of 2026-09-18. `STATUS.md` has the background and evidence for the fixes;
`CLAUDE.md` has the build, simulator and comparison commands.

## 0. Next up, in simulation

Everything here can be done on the dev machine. Ordered by value, longest-running first so it
can sweep in the background.

- [x] **A. Regression sweep of the cartridge library.** Done 2026-09-18 into `work/carts_spin`:
  179 cartridges, and every one of the 12 score fields per cartridge is **identical** to the
  `work/carts` baseline - 2,148 numbers, no differences. So the `cv_ctrl` rewrite, building the
  `.sv` instead of `cv_ctrl.vhd`, and the two `cv_spinner` instances change nothing with the
  spinner off. (Only the `status` labels of Evolution and Smurf Paint 'n Play Workshop read
  differently, because the baseline had `finalize.sh`'s rescore applied and this run has not;
  their match numbers are the same.)
  - [ ] Run `./finalize.sh work/carts_spin` if the labels are wanted for the record.
- [x] **B. Uridium's freeze** - fixed, and confirmed on hardware. Two faults, the lost fifth
  sprite (`b6f6e63`) and the unwaited SDRAM read (`a88e4ff`).
- [x] **C. Gauntlet's maze** - settled 2026-09-19: **not a core bug, the game tears itself.**
  The footage that was wanted exists - "Gauntlet - Colecovision" by ed1475, uploaded 2019-02-01,
  described as "Recorded using the real hardware" with the Opcode SGM - and it shows the same
  breakup. See section 8 for the evidence trail.
- [x] **D. The "black-screen" test cartridges** - they are not black. In Computer mode, where an
  ADAM diagnostic belongs, System Hardware Test reports "FAIL CONTROLLER PORT #1", "FAIL AUX.
  VIDEO" and "FAIL AUX. AUDIO", and ADAM Final Test 3.3 waits at a "STATION ID -" prompt for
  keyboard input. See section 4 for what is left of it.
- [ ] **E. Triage the sweep titles never looked at** (section 5): Pitfall II, Zenji and
  Squish 'Em Sam differ; Adventure Pack 1-3, Evolution, Journey to Maraud Mountain and
  Tournament Tennis are close.
- [x] **F. Cosmo Fighter II's star field** (section 6). Fixed 2026-09-21, and it was never in the
  VDP: `TV80_REFRESH` was undefined in both builds, so the Z80 had no R register and `LD A,R`
  returned a constant 0. The game seeds its stars from R. No `.rbf` carries the fix yet.

Needs hardware, not this machine: the rest of section 1, the spinner checklist in 6a, and the
multicart reset check in section 8. None of them needs a new build any more - `cv_spinner.sv` has
shipped since `releases/ColecoAdam_20260918.rbf`, and `docs/builds/ColecoAdam_20260922_refresh.rbf`
carries the expander and the R register on top of it. `HANDOFF.md` is the checklist.

## 1. Try the September fixes on a MiSTer

- [x] Build with Quartus: done 2026-09-13 with 17.0.2 on the Linux machine. It fits, at 84% of
  RAM blocks, and meets timing.
- Hardware results from 2026-09-13 are in `HANDOFF.md` section 3a; the kit is in `hardware_tests/`.
Also confirmed on the DE10-Nano on 2026-09-18, with the same build: the HOME key and the Best of
Broderbund disk (section 8). The MiSTer is reachable at `mister.local`, and `_AdamTests/` now
also holds `home_test.sh`, `bb_test.sh` and `B1 Broderbund disk.mgl`, with `config/AdamT_BB.CFG`
for a test set with "Keypad on numpad" on, which is how to work the hand-controller keypad from
a keyboard.

- Console mode:
  - [x] Super Cobra: opening screen and attract demo, for the RAM mirroring.
  - [ ] A Crown Jewels game: text screens centred. Not on the test MiSTer's SD card.
  - [ ] An SGM title: 24K RAM through port 53h, and AY sound. Two are now on this machine, in
    `verilator/roms colecovision/issue_tests/`, and can be copied to the SD card.
- Computer mode:
  - [x] SmartWRITER typing, including the first key.
  - [x] A disk boot (Donkey Kong Jr) and a tape boot (Troll's Tale), plus Buck Rogers from tape.
  - [x] Tape save and read-back.
  - [x] Keyboard in the Tape-Disk Verification cartridge.
  - [x] T-DOS at 256K boots to `A0>`.
- [x] Load the ADAM Diagnostic cartridge in Computer mode: it starts, and the right button
  starts the checkout.
  - [ ] Run the checkout through, and check that OSD Reset returns to SmartWRITER.
- [ ] RAM test cartridges at 64K, 256K and None: check the colours on the monitor. MiSTer
  screenshots of them were stale.
- [x] Buck Rogers high score save on hardware, reported by a tester 2026-09-21: **the DSK
  version works**, and writing from SmartWRITER works with a 1.44MB DSK. That covers the disk
  path end to end on real hardware, including a disk image far larger than the 160K standard.
  The **DDP (tape)** version is still unconfirmed - see section 2, which is the tape path.
- [ ] Commit in pieces once hardware looks right:
  - [ ] the simulator
  - [ ] the comparison framework
  - [ ] one commit per RTL fix

## 2. Tape (DDP) writes

Last note: Buck Rogers Super Game on DDP sticks on the high score table, which suggests it
tries to save and fails. Floppy writes were fixed and checked on a DE10-Nano in commit 1029386
(July 2026). Tapes go through the same track loader on drives 4-7 but have their own AdamNet
code (`rtl/cv_adamnet.sv`, tape `CMD_WRITE` around line 566). It isn't known whether the note
came before that fix.

- **Cause found (2026-09-13):** tape writes were never implemented. `cv_adamnet.sv` handled
  tapes with its own copy of the disk read states, and its write state `TAPE_WRITE0` was just
  `$display("Write not supported"); $finish;`. In simulation that ends the run. On hardware
  `$finish` is ignored, so the AdamNet state machine stays in that state for good: the save
  never completes, and no disk or tape is served again until reset. Tapes now use the disk
  states, which the July fix verified on hardware; tapes skip the disk sector interleave. Being
  tested.
  - Reproduced in simulation with SmartWRITER. ESC, type, STORE, V (store workspace), III
    (drive A), type a name, VI. The old build logs "Tape G: Writing 1024 bytes, sector 0x2,
    memory 0xd400", then "Write not supported", and stops.
  - The first run of the fixed build crashed (exit 139). That was the simulator harness, not
    the core: `sim_main.cpp` connected `sd_buff_din` only for drives 0 and 1, so the first
    write to drives 2-7 (all tapes) dereferenced NULL. All eight are now connected.
  - [x] **Verified in simulation.** SmartWRITER shows "ONE MOMENT - STORING FILE" and
    returns to the editor. The blank tape image changes by 1,050 bytes and now holds the file
    name (TEST) and the document text (HELLO TAPE).
  - [x] **Read back.** A fresh SmartWRITER boot with that tape reads blocks 0 and 2, and GET
    on drive A shows the FILE DIRECTORY listing TEST. Still to check: opening the file,
    Buck Rogers' high score save, and hardware.
  - Buck Rogers' save is not reached by the script yet. With `--press 1@300 --press 1@700` (one
    player, skill 1) and no steering, the game ends around frame 4,200. The name-entry grid
    (PLAYER ONE … DONE?) follows at ~4,400; fire adds a letter, and DONE? sits at the
    top right, above the grid. It needs a scripted path to DONE?, which is easier on hardware
    with a real joystick. The tape write itself is verified above.
- [ ] Reproduce in the simulator with `SoftwareFromMiSTer/E.O.S/Games/Buck Rogers - Planet of
  Zoom Super Game (1983) (Coleco).ddp` (on a copy). Script `--press` input to reach game over
  and the initials entry, then check that the image changed and the game carries on.
- [x] Do the same with the `.dsk` version, to separate tape from disk. Done on hardware by a
  tester 2026-09-21: Super Buck on DSK works, and SmartWRITER writes to a 1.44MB DSK. So the
  fault, if any remains, is in the tape path rather than shared write code.
- [ ] Run the ADAM Tape-Disk Verification Rev. 1 cartridge (`adam_carts/`) against a blank DDP
  from `Blank Media/`.
- [ ] Re-run the July write tests: `_wrtest/cpmtest.dsk` against `_wrtest/golden.dsk`, and
  `wordtest.dsk`.
- [ ] If the simulator writes correctly but hardware doesn't, look for the same kind of SD
  handshake problem that broke floppies (sd_ack vs a shared, stale sd_buff_addr).

## 3. Expansion RAM

- [x] **Bug: the upper half of the 64K Memory Expander is missing.** Fixed; see the next item.
  Re-verified 2026-09-21 at every size with `hardware_tests/banktest/uppertest.col`, which walks
  all 32 banks through the upper window (7Fh = 0Bh, worker relocated to RAM at 7000h because the
  expander covers where the cartridge lives). Counts match the lower window exactly: none and 64K
  give 0, then 4, 8, 16, 32. Port 7Fh upper bits `10` select 32K Expansion RAM (ADAM Technical
  Manual 2.2).
  - `cv_addr_dec.sv` decodes it (`expansion_ram_ce_n_o`), but nothing answers:
    `cv_console.sv`'s bus mux ignores it, and `ColecoAdam.sv`/`sim.v` have no RAM behind it, so
    it reads FFh. The lower half (`lowerexpansion_ram`) exists.
  - Add a matching 32K RAM. ColEm has both halves (`RAM_EXP_LO`/`RAM_EXP_HI`), so it can be
    compared.
- [x] Upper expansion RAM added: a 32K RAM in `ColecoAdam.sv`/`sim.v`, and `cv_console.sv`'s
  bus mux answers for it. SmartWRITER, Frogger and a DK Jr disk boot are frame-identical to the
  build without it.
- [x] Ports 20h-3Fh and 60h-7Fh are now decoded as ranges. The ADAM memory board's MIOC (U7)
  sees only BA13-BA15 and the game board's AUX DECODE lines, ColEm and MESS decode the same
  ranges, and Coleco's 64K Expansion RAM Test writes 20h and 60h. No regressions in the same
  three checks.
- [x] T-DOS boots with the expander and AdamNet gating. The M.I.B. 3 Drivers T-DOS 4.58 disk
  reaches its `A0>` prompt and smart keys. At boot it maps port 7Fh = 02h (expansion RAM in the
  lower window), which is its bank probe, and carries on normally.
- [ ] Test the 64K expander:
  - [ ] 64K Expansion RAM Test cartridge (`colrom/`)
  - [x] RAMTEST v2.0 (Eric Pearson): run on a DE10-Nano 2026-09-21, reports "32 Banks Detected"
    at the 2M setting and walks them. RAMTest Rel 2.0 (Orphanware) turns out to be an address
    range tester rather than a sizer, so it says nothing about the expander.
  - [ ] CP/M or T-DOS reporting the extra memory
- [x] **AdamNet ignores the memory map.** Fixed 2026-09-13: `cv_console.sv` now passes the
  AdamNet strobes only when intrinsic RAM is selected (`ram_ce_n_s` or `upper_ram_ce_n_s`).
  SmartWRITER typing, a DK Jr disk boot, Frogger and the expander test cartridge are
  frame-identical to the build without it, and Troll's Tale still matches ColEm. Original note: `cv_adamnet.sv` treats every CPU memory access to the
  PCB/DCB range (FEC0h on reset, relocatable) as AdamNet traffic. It uses the raw address and
  strobes (`z80_addr = a_s`, `z80_wr = wr_z80_c`) and never checks which memory is mapped. On
  an ADAM the 6801 master reads the PCB from whatever memory the Z80 has mapped. Software that
  maps expansion RAM (or a cartridge) over FExxh and writes there, as a RAM disk does, would
  send the core bogus commands or relocations. Gate the snooping on intrinsic upper RAM being
  selected (`upper_ram_ce_n`), or model DMA properly, before trusting T-DOS RAM disks.
- [x] Expander verified with `tools/adam_exp_ramtest.py --trace`, whose port 3Fh markers show
  the path taken. On the build without the fix, the upper half is written and then fails, and
  the lower half passes. On the fixed build both halves verify and the program finishes. (An
  earlier run looked hung only because a cartridge with the AAh 55h header sits on the
  ColecoVision title screen for ~12 s; the tool now uses 55h AAh.) The finished cartridge
  shows medium red on the build without the fix (upper half missing) and medium green on the
  fixed build. It needs ~200 frames: `--adam --cart ramtest.rom --frames 400`. The VDP also
  checked out: a blanked display shows the backdrop colour, as the data manual says.
- [x] **256K expansion RAM added (2026-09-13).**
  - What: an OSD option "Expansion RAM" with 64K / 256K / None (simulator flag
    `--exp-ram 64|256|0`), a port 42h bank register in `cv_addr_dec.sv`, and one 256K RAM
    addressed {bank, window, address} in `ColecoAdam.sv`/`sim.v`. 64K ignores port 42h, like
    an expander without an addressor.
  - Verified in simulation:
    - `adam_exp_ramtest.py` passes at 64K and 256K.
    - Its `--banks` mode passes only at 256K; at 64K the banks alias.
    - "None" fails both.
    - SmartWRITER, DK Jr and Frogger are frame-identical to the previous build.
    - T-DOS boots with 256K.
  - Hardware: the RAM adds about 1.5 Mbit of BRAM, still to check on the MiSTer.
  - 512K/1MB remain open (SDRAM, below).
- [x] The original item, done 2026-09-18: the OSD offers 64K / 256K / 512K / 1M / 2M / None.
- [ ] **Beyond 64K** (MicroFox 256K/512K/1MB cards, Lundy 1MB board; both need an addressor
  signal). MESS's adam.c documents port 42h: "42-42 (W) = Expansion RAM page selection, only
  useful if expansion greater than 64k". Still unknown:
  - [ ] Page size and bit layout. The old note's "D3:D0 picks one of 16 × 64K banks" is
    unconfirmed.
  - [ ] Reset value, and whether 42h can be read.
  - [ ] Whether both 32K windows follow the same page.
  - [ ] Whether page 0 is the same RAM as the 64K expander.
  - Where to settle it: the EXPAnDDR schematic PDF (github.com/epearsoe/EXPAnDDR), T-DOS and
    CP/M RAM-disk drivers in `CP-M & T-DOS/`, or John Lundy.
  - **Found in RAMTEST v2.0 (Eric Pearson, 2018):** a bank is 64K, seen through the normal
    expansion windows.
    - It counts banks with `LD A,02h / OUT (7Fh),A` (lower window = expansion RAM), then
      `LD A,C / OUT (42h),A` with C = 0, 1, 2… until a probe fails.
    - It tests each bank's lower half with 7Fh = 02h at 0000-7FFFh, and its upper half with
      7Fh = 0Bh at 8000-FFFFh.
    - So port 42h takes the bank number as a byte: 1MB is 16 banks, 256K is 4. Bank 0 is
      presumably what a card shows without an addressor, the plain 64K expander.
    - Its probe (8142h) stops counting at the first bank that either already holds its 2680h
      marker at 0086h (an alias of a bank it has seen) or fails an XOR 55h write/verify at
      0084h + n×400h. So bank numbers past the card's size can wrap (bank & mask) or read as
      missing RAM; both count correctly. A core that ignores 42h reports 1 bank.
    - The `OUT (42h),A` hits on CP/M and T-DOS disks (ASCOM, QTerm, the MIB and Orphanware
      drivers, SuperCalc, dBASE II, WordStar) are T-DOS itself, not serial code.
      - Disassembled on the MIB3 T-DOS 4.58 and ASCOM disks: the T-DOS system tracks contain
        the same bank count as RAMTEST (`OUT (7Fh),02h`, `OUT (42h),C` in a loop, 2680h marker
        at 0086h).
      - They also contain a RAM-disk routine that walks banks with `OUT (42h),A`.
      - So booting any T-DOS disk exercises memory beyond 64K, and 42h is the RAM bank port
        there.
- [x] **Put large expansion RAM in SDRAM.** Done 2026-09-18, commit `337ebcd`; see section 3a
  for the controller and 3b for the card behaviour. The fit went from 466 of 553 M10K blocks to
  210. 1MB did not fit in BRAM: the Cyclone V has about
  5.5 Mbit (~700 KB) of M10K in total, shared with the MiSTer framework, and the core's RAMs and
  ROMs already take about 1.4 Mbit.
  - The core already requires SDRAM and uses it for cartridge ROM (the first 1 MB).
  - Map the expansion at an offset such as 100000h and share `sdram.sv`'s single request port;
    the Z80 only makes one access at a time.
  - Give `sim.v` the same memory.
- [x] **Power Paint**'s memory readout, checked 2026-09-18 in simulation and on hardware: it
  reads 64, 256 and 512 at the matching settings, and 512 at 1M and 2M as well. That ceiling is
  its own - its sizer counts to four banks past the base and then stores a saturated code - not
  the core's.
- [ ] Test beyond-64K software:
  - [ ] T-DOS and CP/M 2.2 RAM disks
  - [ ] EOS RAM disks (ADAM's Desktop, SmartDSK, MegaDisk)
  - [ ] RAM Test v1.3 & Utilities (AJM, CP/M)
  - [x] RAMTEST v2.0 - 32 banks on hardware, above
  - [x] RAM-disk contents surviving a reset, as they do on hardware. They do, and it bites:
    RAMTEST leaves its 2680h marker at offset 0086h of every bank, a core reset does not clear
    SDRAM, so a second run reads its own marker back, reads it as an alias and reports 0 banks.
    Faithful - a warm reset does not clear DRAM on a real ADAM either - but worth knowing before
    reading a 0 as a fault.
- [ ] Accept the introduction to John Lundy (Lundy Electronics). Questions for him:
  - [ ] Port 42h layout and reset state.
  - [ ] Addressor behaviour.
  - [ ] What survives a reset.
  - [ ] The SuperADAM memory map.
  - [ ] Whether he has seen DDP write-timing trouble.

## 4. Run the test programs

Disks: `./obj_dir/Vemu --headless --adam --disk 1 COPY.dsk --frames 3000 --every 60 --outdir DIR`.
Cartridges: the same with `--cart FILE`, which now starts them through the cartridge reset.
Most of these wait for keys, so each needs its key sequence worked out (`--key`, `--type`).

- [ ] ADAM Diagnostic (1982):
  - [ ] the disk (`verilator/`, `E.O.S/Utilities/`)
  - [ ] the cartridge (`adam_carts/`). Loaded in ADAM mode (build with the DCB status fix) it
    shows "ADAM CHECKOUT CARTRIDGE", then a menu asking for a game-controller button: SKIP on
    the left, CHECKOUT on the right. So it takes controller input (`--press fire1`/`fire2`),
    not the keyboard. A checkout run with the right button is under way.
- [ ] Coleco In-House Test Utilities, versions 1, 2 and 3 (`E.O.S/Utilities/`).
- [ ] RAMTEST v2.0 (2018, Eric Pearson) and RAMTest Rel 2.0 (1986, Orphanware): base RAM now,
  expansion RAM after section 3.
  - Orphanware: after "REMOVE RAMTEST NOW / PRESS ANY KEY" it asks for start and end addresses
    in decimal, tests, and says "RAMTEST COMPLETED". Empty entries test only address 0; script
    real ranges with `--type "0\n65535\n@..."`.
  - Pearson: it does boot normally. Block 0 sets port 7Fh = 03h, reads blocks into 2000h and
    8000h and jumps to 8000h, which prints its title through EOS's VRAM write. But the screen
    stays black for 9,000 frames on the core and on ColEm too, so this isn't a core bug. Keys
    (Return, space, Escape) change nothing.
    - The core logs its port 7Fh writes as 04h, 00h, 03h (its loader), 00h, 01h, but never
      the 02h its bank count writes first. So it never reaches its test code; the loader's
      block reads or the jump to 8000h go wrong. Low priority while ColEm agrees.
- [ ] R.I.D. Test v1.0 for Disk Drives (Dymek): disk reads and writes.
- [ ] RAM Test v1.3 & Utilities (AJM, `CP-M & T-DOS/Utilities/`).
- [ ] Cartridges:
  - [x] System Hardware Test and ADAM Final Test Rev. 3.3 run in Computer mode (2026-09-18).
    System Hardware Test reports "FAIL CONTROLLER PORT #1" plus AUX video and AUX audio, which
    the core does not emulate. Final Test 3.3 waits at "STATION ID -" for keyboard input.
    - [ ] Type a station ID into Final Test 3.3 and run its tests through.
    - [x] "FAIL CONTROLLER PORT #1" is expected, and so are the two AUX lines: the test needs
      Coleco's manufacturing fixture. It writes a pattern to port 09h, which nothing on an ADAM
      decodes, and expects the controller port to read back 7Fh then each line pulled low in
      turn (table at 8724h). Without the fixture it times out. Port 2 is not passing either -
      the failure handler skips that test. See `STATUS.md`.
      - [ ] Optional and quite appealing: emulate that fixture in the simulator only, so port 09h
        drives the controller lines. The cartridge would then check our controller decode line by
        line, which is a better test of it than any game.
    - [ ] Check the Menu Version too.
  - [ ] Old note, now known to be wrong: black screen in
    both old and new builds. Find out whether they wait for input or need test hardware.
  - [ ] Video RAM Test and 64K Expansion RAM Test: they start. Check results, and re-check the
    RAM test after the expander fix.
  - [ ] ADAM Tape-Disk Verification (section 2). Its menu comes up, and it lists TAPE 1 and
    DISC 1 as active devices, so it talks to AdamNet. But no key reaches it: V, v and C (change
    configuration) are all ignored, and the 64K Expansion RAM Test cartridge also sits at its
    prompt. Keyboard input to cartridges started through the cartridge reset (no EOS) looks
    broken.
    - The cartridge copies itself to RAM and maps port 7Fh = 01h, so its PCB at FEC0h is
      intrinsic RAM. The core logs "MovePCB Address: fec0" and the Z80/6801 syncs (81h/82h).
    - It then probes devices through one DCB. It sets the PCB's DCB count (IX+3) to 1, then
      for A = 1…0Fh writes A to DCB 0 offset 10h (`LD (IY+10h),A` with IY = FEC4h), issues
      STATUS, and waits for 80h (ROM 9F19h).
    - The core classifies DCBs by `{dcb_dev_num[3:0], dcb_add_code[3:0]}` (keyboard = 1). It logs
      "Writing Unknown device #8" ×102, "#2" ×17 and one "Unimplemented PCB Operation", so
      the keyboard is never recognised.
    - In the core's DCB layout offset 10h is `DCB_ADD_CODE` (offset 9 is `DCB_DEV_NUM`), and the
      cartridge first zeroes FEC0h-FFFFh (ROM 9EDFh). So the first probe makes DCB 0 read as
      `{0, 1}`, the keyboard, and classification should match. The failure is later: the
      "Unknown device #2/#8" DCBs it uses after probing, the DCB count it changes with
      `DEC (IX+3)`, or the STATUS reply the probe waits for.
    - Writes to `DCB_DEV_NUM` (offset 9) and `DCB_ADD_CODE` (10h) are copied into `dcb_table`
      (`cv_adamnet.sv` ~624-635), so the core does follow a DCB that is renumbered. The fault is
      later: the keyboard `CMD_READ` → `kbd_req` → `KBD_KEY` → `watch_key` delivery, or the
      status reply. `watch_key` also needs the disk state machine idle and a key queued.
    - Traced with the `ADAMNET_TRACE` build (`Makefile` has the commented define):
      - SmartWRITER polls keyboard DCB 0 with CMD_READ (buffer FD75h, length 1), and keys
        are delivered.
      - The cartridge sends keyboard DCB 0 only one STATUS, and never a CMD_READ. The keys
        it is sent are queued (`KBD queued 56`, `43`) but nothing collects them.
      - It does send 102 commands to "unknown device #8" and 17 to #2, so it probably polls the
        keyboard through DCB 8 with a device number/add code the core doesn't classify as
        keyboard.
      - The "Unimplemented PCB Operation" messages are PCB command 00h (idle) and FFh, and are
        harmless.
    - Compared with ColEm's `AdamNet.c` (`WritePCB`, `MovePCB`, `UpdateDCB`):
      - Sync commands 01h/02h only acknowledge in both; neither rebuilds the DCB table.
      - [ ] **Relocate (03h):** ColEm calls `MovePCB` (DCB J gets DEV_NUM 0, ADD_CODE J) and
        acknowledges. The core called `$finish`, which ends a simulation and would hang AdamNet
        on hardware. Changed 2026-09-13 to run `MOVE_PCB`, keeping the DCB count and
        acknowledging with 83h.
        - No regressions: SmartWRITER typing, a DK Jr disk boot, Frogger and the Tape-Disk
          Verification cartridge are frame-identical, and Troll's Tale still matches ColEm.
        - None of them relocates, so the new path itself is still unexercised. Find software
          that writes 03h to the PCB.
        - A byte-pattern scan doesn't settle it. `LD (IY+0),03h` appears 4 times in EOS, and
          `LD (IX+0),03h` in 34 library titles next to FEC0h references, but 03h is also the
          DCB CMD_WRITE. Look for the PCB command write specifically, or log "Relocate PCB"
          across a library sweep.
      - DCB count: ColEm's `WritePCB` checks `Dev <= MaxDCB`, but `IsPCB` has already
        excluded DCB index MaxDCB, so like the core it serves DCBs 0 to MaxDCB-1. No change
        needed.
      - [x] ColEm silently ignores PCB commands 00h and 80h+. The core now logs "Unimplemented
        PCB Operation" only for 01h-7Fh, and still stores the byte, which reads back like RAM.
      - ColEm answers an unknown device with `RSP_ACK + 0Bh`; the core answers `RSP_BUSY`.
      - Device ID is `(DEV_NUM << 4) + (ADD_CODE & 0Fh)` in both.
    - Second trace, with the richer "Unknown device" message: the "unknown" DCBs are the
      cartridge probing IDs that don't exist in ColEm's table either.
      - DCB 2 is probed with add_code 03h, and DCB 8 with 0Ah-0Fh.
      - Each gets commands 01h, 15h and 7Eh.
      - The keyboard (ID 1) answers its STATUS on DCB 0, and the typed V and C are queued, but
        the cartridge never sends a keyboard CMD_READ.
      - The reply to unknown devices matches ColEm: the core's `RSP_BUSY` is 9Bh, which is
        ColEm's `RSP_ACK + 0Bh`.
      - The scan itself (ROM 9F19h) works in the core. It probes IDs 1-0Fh through the current
        DCB, treats a STATUS reply of 80h as found, then moves to the next DCB and raises the
        PCB's DCB count. A helper at 9F58h later finds a device's DCB by add code.
    - Its keyboard read exists (ROM 8F37h). It sets the DCB buffer to 7003h, the length from
      the DCB's max-length field, add code 1, and clears the other fields. It then issues
      CMD_READ; 8F66h reads the key from 7003h, and 8F30h checks for status 8Ch (no key).
    - Its block-command helpers busy-wait with no timeout: `LD (IX+0),04h / BIT 7,(IX+0) / JR Z`
      at 8BB5h and 8C06h. If the core never completes some command, the main program stalls
      there while the NMI keeps blinking "PRESS V", which is what the screen shows. The trace
      has no keyboard read at all, so this is the likely state.
    - **Cause found (address profile plus disassembly).**
      - `SIM_ADDR_PROFILE` shows the cartridge copies itself to 2000h. It spends ~94% of its
        time in a 0.12 s delay (3ED6h) and ~5% writing VRAM, blinking its menu.
      - The menu loop (2755h) blinks a label, calls the key reader at 2F00h, and loops while
        that returns FFh.
      - 2F00h finds the keyboard DCB (add code 1) among the first five DCBs and looks at its
        status: 00h starts a CMD_READ, bit 7 clear means busy, 80h means a key is waiting
        (it reads it from 7003h and writes 00h to clear), 8Ch means no key.
      - The core serves DCB status reads from `dcb_table`, but updated it only for command
        writes (01h-7Fh). The cartridge's 00h clear was dropped, so it kept reading the stale 80h
        left by its STATUS probe, never saw 00h, and never started a read. ColEm keeps DCBs in
        RAM, so a written 00h reads back.
    - [x] **Fixed and verified in simulation (2026-09-13):** `DCB_CMD_STAT` writes of 00h or
      80h+ now go into `dcb_table` so they read back. With a lowercase `c` the cartridge goes
      from its menu to "CHANGING CONFIGURATION".
      - No regressions: SmartWRITER typing, a DK Jr disk boot and Frogger are frame-identical,
        and Troll's Tale still matches ColEm.
      - The trace build with the fix shows it working. The cartridge now issues keyboard
        CMD_READs (buffer 7003h, length 1) every poll: 72 reads, 70 empty, and both typed keys
        delivered (`KBD deliver 43 to 7003`, `56`).
      - The screen didn't react because the test typed uppercase, which `--type` sends as
        Shift+letter (43h/56h). The menu (2765h) subtracts 20h before comparing, so it wants
        lowercase `c`/`v` (63h/76h), as an unshifted ADAM keyboard sends. A rerun with lowercase
        keys is under way to confirm.
  - [ ] Printer burn-in test (needs a printer).
  - [ ] Resident Debugger.
  - [ ] Final Test Cartridge (1982), the 8K `.bin` inside `verilator/Final_Test_Cartridge_(1982)(Coleco).zip`.
- [x] Add a screens-only runner to `verilator/compare/` for programs ColEm can't mirror:
  `screens.sh` saves a contact sheet and reports bytes changed in each mounted image.
- [ ] Record
  each program's expected screens once confirmed on hardware, so later runs catch regressions.
- [ ] Patch the ColEm harness to start ADAM cartridges in game mode (keep `CV_ADAM`, memory map
  0Fh) so they can be compared too.

## 5. Library sweeps

- [x] `./library_scenarios.sh work/games "../SoftwareFromMiSTer/E.O.S/Games" 12`. Ran
  2026-09-13 on the build before the expander, tape-write, AdamNet and DCB fixes: 283 titles
  (13 skipped for image size), 260 MATCH, 10 CLOSE, 13 DIFFERS, in 3.8 hours with 10 jobs.
  - Checked, not core bugs; in each case ColEm is the one failing:
    - Cabbage Patch Kids (128K prototype): the core reaches the players menu, ColEm stays black.
    - Electronic Game Pack II (disk and tape): the core shows its menu, ColEm a blank screen.
    - Chess Solitaire: the core's menu is readable, ColEm's text is garbled.
    - Donkey Kong Super Game (disk and tape): the core reaches the players menu while ColEm
      is still on the ADAM/Nintendo title.
    - Ace of Aces (disk and tape): ColEm can't boot ADAM CP/M.
    - Beamrider (tape): animation phase only.
  - [ ] Not checked yet:
    - DIFFERS: Pitfall II (tape), Zenji (two tapes), Squish 'Em Sam (tape)
    - CLOSE: Adventure Pack 1-3, Evolution, Journey to Maraud Mountain, Squish 'Em Sam, Tournament Tennis
  - [ ] Re-run on the current build; Cabbage Patch Kids, which needs 128K, may behave differently
    with the expander fixed.
- [ ] The rest of `E.O.S/`, then `CP-M & T-DOS/` (250 files). ColEm can't be the reference
  for CP/M. On "CP-M 2.2 & Assembler (1984) (Coleco)" the core boots to the ADAM CP/M banner
  and `A>` prompt with its smart keys, while ColEm shows only the backdrop. Ace of Aces (2021)
  boots CP/M and shows the same split. Check CP/M and T-DOS titles by their screens with
  `screens.sh`, not `library_scenarios.sh`.
- [ ] Images that aren't 160K disks or 256K tapes: 19 × 320K, 16 × 1.44MB, 4 × 1MB, 4 × 8MB,
  2 × 720K and others. Check whether the core boots them; ColEm can't, so check by screens.
- [ ] SGM titles: port 53h RAM and AY sound.

## 5a. Frogger's green block, and what it is not

Seen on hardware on 2026-09-18: a solid green block on the left of the trucks in the top road
row, flashing on and off. Investigated 2026-09-18; **not reproduced in simulation** in ~17,000
frames across five scenarios, nor by ColEm on the same scenario.

Measured from the hardware screenshots and from VRAM:

- It is exactly one character cell - char row 15, columns 12 and 30, solid colour index 12 - and
  **the road vehicles are background characters, not sprites**. The sprite attribute table at
  1E00h holds ten sprites at a gameplay frame and none are anywhere near the road.
- The cell holds **name 00h, Frogger's own blank tile**, where the truck's lower-left cell should
  hold E6h. So two name-table bytes, 19ECh and 19FEh, read 00h instead. Both trucks in a row use
  the same tile names, which is why both show it at once.
- Ruled out: sprites; corruption of the truck's tile pattern or colour bytes, which are static and
  would render black rather than green; and dropped CPU writes to VRAM - 609,938 writes scheduled
  in 1400 frames with zero collisions.

**Fixed by the SDRAM wait (`a88e4ff`)**, confirmed on hardware on 2026-09-18: Frogger plays
cleanly on `ColecoAdam_20260918_sdram.rbf`.

That also settles what it was. The first guess - the beam catching Frogger mid-redraw, since it
rewrites the whole road row of names on every scroll step - was wrong. The value written was
wrong, not merely caught early: the cell held Frogger's blank tile 00h where E6h belonged, which
is what happens when the Z80 reads a **stale byte from the cartridge** and stores it. Before the
fix nothing waited for the SDRAM, so a late read returned whatever the previous access left
behind, and Frogger reads tile data from the cartridge constantly while scrolling.

Worth remembering as a diagnostic pattern: a *wrong but valid* value in VRAM points at the source
the program read from, while a torn or half-updated screen points at timing.

## 6. Known bugs

- [x] The simulator drove both controllers from one keyboard (`joystick_1 = joystick_0`), so a
  two player game was unplayable. Fixed 2026-09-19: player 2 is on I/J/K/L plus F and G, and the
  recording format grew a third column for its bits. Recordings written before the fix have two
  columns and `--replay` drives both ports from them, so they still reproduce what they
  captured - checked by replaying one and comparing frames byte for byte.
- [x] Cosmo Fighter II's star field is missing. Fixed 2026-09-21: `TV80_REFRESH` was never
  defined, so `LD A,R` read back a constant 0. See `STATUS.md`.
- [x] Re-check README's known bugs and update it. Done 2026-09-21: the typing bug and the
  "tape/disk write is not supported" line are gone (disk writes confirmed on hardware by a
  tester, tape implemented but not yet confirmed), and the README gained an **Expansion RAM**
  section. That section exists to stop the same two questions coming back as issues: PowerPAINT
  reporting 512 for any card 512K or larger, which is its own sizer saturating, and RAMTEST
  reporting "0 Banks Detected" on a second run, which is it reading back the 2680h marker it
  stamped at offset 0086h of every bank - a reset does not clear the expander, exactly as a warm
  reset does not clear DRAM on a real ADAM. Reload the core to run it again.
  - [x] a bad character the first time something is typed: fixed in commit 1029386 (July
    2026) and checked on a DE10-Nano. EOS used to read the key after the PS/2 value had moved
    on to the release event (^@), and a blocked keyboard read swallowed keystrokes.
  - [ ] reset not quite like an ADAM
  - [ ] no printer
  - [ ] key repeat
  - [x] Cosmo Fighter II's star field was listed there as a known bug. Moved to the "fixed since
    this list was written" paragraph on 2026-09-21, naming the cause. Note the README now runs
    ahead of the releases: the fix is in the tree and no published `.rbf` has it yet, so the next
    build needs to go out reasonably promptly.

## 6a. Spinner and roller controllers

Implemented on 2026-09-18 (`rtl/cv_spinner.sv`, `rtl/cv_ctrl.sv`, OSD "Spinner"); see `STATUS.md`
and the references in `docs/controllers/`. What is left:

- [x] **Works on hardware.** Bruce's Controller Tester (`S1`) with the OSD Spinner option on
  *Stick X* tracks the analog stick on a DE10-Nano, 2026-09-18, on
  `ColecoAdam_20260918_sdram.rbf` - the first build containing `cv_spinner.sv`. That is the
  spinner path proven end to end: OSD option, `cv_spinner`, the pin 7/9 signalling, `cv_ctrl`'s
  strobe and interrupt, and OS-7's own counting routine.
- [ ] The rest of `HANDOFF.md` section 3, "Spinner and roller controllers": the *Spinner* setting
  with a real spinner device, *Stick XY* for a Roller Controller title, and a real game - Slither,
  Victory, Turbo or Super Action Baseball.
- [ ] Play the real titles and judge the feel: Slither and Victory (Roller Controller), Turbo and
  Destructor (Driving Module), Super Action Baseball and Football (speed roller). The step rate
  for an analog stick is MAME's sensitivity, |rate| * 2 steps per second, and may want tuning per
  game or an OSD sensitivity setting.
- [ ] Decide whether the Driving Module's pedal should be mapped: it is just a switch to ground
  on the fire line (Tech Guide V-1), so it may need no work beyond documenting which button it is.
- [ ] /SPINDIS (ATM 2.1.3, MIOC pin 10) is not modelled. Only needed if ADAM software turns out to
  be disturbed by spinner interrupts; simulation says SmartWRITER is not.
- [ ] The Roller Controller passes the controller signals through to its own pass-through ports
  and has a "keypad only" arrangement in some games; check a two-player Roller title.

## 7. SuperADAM features on the MiSTer

From colecovisionadam.com/Coleco/adam/SuperADAM.php and lundyelectronics.com/product/superadam-build/.

| SuperADAM feature | On the MiSTer |
|---|---|
| 1MB RAM Expander | Yes: section 3 |
| ADE Pro / FujiNet drive emulator, SD DDP, external data drive housing | No: the MiSTer already mounts DSK/DDP images from its SD card (4 disks, 4 tapes) |
| PICO9918 with HDMI, VGA bracket | No: the MiSTer outputs HDMI and analogue video |
| DRAM-to-SRAM converters, power supply, IEC inlet, switch and LED, feet, badge | No: physical and reliability items |
| 8-BIOS selector | Maybe: an OSD choice of OS-7 BIOS files |
| Boot PROM / expansion ROM (instant SmartBASIC, ADAM's Desk Top), SmartWRITER R89 | Maybe: ROM file options. The expansion ROM decode exists and reads FFh today; `Chip Sets/` has SmartWRITER revisions and `Boot PROM/` has ADAM's Desk Top |
| MIB238 / MIB2 / MIB3 serial, parallel, WiFi modem, 80-column terminal | Later: serial could map to the MiSTer UART; needs register-level documentation |
| ADAMnet printer (with reset silencer) | Later: no printer support yet; could print to a file |
| C88 sound balance mod | Only if a speech synthesizer or new audio mixing is added |
| MicroFox IDE hard disk (slot 1) | Later: emulate from a MiSTer image, with the boot driver in `Boot PROM/` |

## 8. GitHub issues

The three open issues on MiSTer-devel/ColecoAdam_MiSTer, all filed in 2022, checked against this
branch on 2026-09-18. Test ROMs that were not already in the collection are in
`verilator/roms colecovision/issue_tests/`, from the ADAM archive.

### #14 "Home" key doesn't work - fixed in 2022, just never closed

The reporter filed it on 2022-06-23 and fixed it himself in PR #16, merged three days later:
`cv_adamnet.sv` had `9'h16c : key_code = 'h87`, and 87h is the filler this table uses for every
unmapped key, so HOME really did nothing. It has been 80h since.

Verified in simulation on this branch: `--key esc` into the word processor, type a line, then
`--key home`, and the cursor jumps from the end of the line back to the first character.

The trap, worth saying when closing it: **SmartWRITER boots in typewriter mode, where HOME and
the arrow keys do nothing at all.** Escape gets you the word processor, where they work. Both
keys look equally dead until you do that, which is most likely what the reporter saw in 2022 on
a build that also lacked the mapping.

- [x] Confirmed on a DE10-Nano (2026-09-18, `ColecoAdam_20260913_accuracy.rbf`): Escape, type
  "hello world", press HOME, and the cursor moves from the end of the line back onto the "h".
  Issue closed with the typewriter/word-processor explanation.
- [ ] Optional: the same check inside SmartLOGO's editor, which is what the reporter wanted it
  for.

### #9 and the Broderbund part of #12 - no longer reproduces

"The Best of Broderbund collection doesn't get past game title screens", reported for both tape
and disk. On this branch, both images:

- boot to "PRESS KEYPAD NUMBER ON HAND CONTROLLERS TO SELECT GAME: 1. A.E. 2. CHOPLIFTER";
- take keypad 1 or 2 and load past the game title to the skill/players menu;
- start the game: A.E. reaches "GET READY", Choplifter reaches gameplay.

The September library sweep also scored both images `final_match=1.0000` against ColEm. The
AdamNet and DCB fixes on this branch are the likely reason it works now.

- [x] Confirmed on a DE10-Nano (2026-09-18, `ColecoAdam_20260913_accuracy.rbf`, disk image):
  the menu appears, keypad 1 selects A.E., a skill key starts it, and the game plays. Driven from
  the keyboard with "Keypad on numpad" on, via `config/AdamT_BB.CFG` and
  `_AdamTests/B1 Broderbund disk.mgl`. #9 closed.
- [ ] Tick Broderbund off #12. Uridium is fixed and Gauntlet is not a core bug, so the
  issue can be closed once that is written up.

### #12 Uridium - still reproduces

Uridium (2019) (Team Pixelboy) (SGM) draws its title screen correctly, then goes to a flat green
screen and freezes once fire is pressed. Both reporters said in 2022 that it fails the same way
on the stock MiSTer ColecoVision core, so this is probably shared with upstream rather than
anything ADAM-specific.

ColEm is no use as a reference here: its own frames for this cartridge are a single colour, so
it renders the cartridge worse than the core does.

Traced on 2026-09-18 with `SIM_ADDR_PROFILE`, the new `SIM_SPR5_PROFILE` and `--peek v:`. What it
is **not**:

- Not a dead CPU. The two hot addresses are a `HALT`/`DJNZ` vblank wait at 285Eh and a 300-frame
  attract wait at A6DDh that also exits on a new button press, so the program is running.
- Not a lost interrupt. The VDP asserts its interrupt 1,508 times in 1,650 frames, right up to
  the last one.
- Not the fifth-sprite trick, though the game does use it: at 00ECh it spins on `IN A,(BF)`,
  `AND 5F`, `CP 5C`, waiting for the fifth-sprite number as a scanline counter. Our VDP does
  report fifth sprites (numbers 4, 8, 12, 16, 20 and 28), and that loop is an earlier phase.
- Not the memory map. The game uses the SGM's 32K mode - port 53h for the RAM at 2000-5FFF, then
  port 7Fh with lower=01 so 0000-7FFF is all RAM - and runs code from there, which is decoded.

What it is: **the menu works and choosing Start does not.** Left alone, the game reaches its own
menu - HEWSON & Trilobyte credits with Start / Settings / Instructions - and waits there quite
correctly. Press fire to choose Start and it draws the Uridium title, then switches the display
off (flat border colour, and the sprite table holds three sprites and a D0 terminator) and never
switches it back on. MegaCart paging runs until exactly that moment: 63 bank switches, the last
at frame 1599, using pages 0, 1, 2, 3 and 5 of the eight. So it stops loading when it should be
loading a level.

Traced further with the new `SIM_VDP_TRACE`, `SIM_MEGA_TRACE` and `SIM_ADDR_PROFILE_FROM`:

- At frame 1598 the game writes R1 = 32h: **text mode with the display off**, and never turns it
  back on. R5 = 37h, so the sprite table is at 1B80h, and during the freeze it holds two sprites
  and a D0 terminator.
- Profiled from frame 1700 - the frozen phase alone - the CPU spins at 00ECh in a routine in SGM
  RAM: `IN A,(BF) / AND 5F / CP 5C / JP C`, waiting for the fifth-sprite flag with a number of 28
  or more. The sprite probe reports **zero** fifth-sprite detections in that phase, which is
  correct: a 9918 runs no sprites in text mode, and there are only two sprites anyway.
- That routine is not corrupt. The bytes at 00E0h are identical at frames 1000, 1400 and 1700,
  so it is resident code the game put there deliberately.

So the game calls a raster-sync routine written for graphics mode while it is in text mode with
the screen off, which cannot ever return. The wrong turn is earlier, in whatever chose that path.

- [x] **Fixed, and confirmed on hardware on 2026-09-18.** Two separate faults:
  - the sprite scan stopped at four visible sprites, losing the 5S flag and its number, so the
    game's wait for fifth-sprite number 29 never ended (`b6f6e63`);
  - nothing waited for the SDRAM, so the Z80 read stale cartridge bytes, which corrupted the
    screen on hardware while simulation ran fine (`a88e4ff`).
  With both, Uridium warps in and plays "Zinc" on a DE10-Nano, matching simulation.
  (The AY was never the problem; ports 50h/51h/52h are decoded and `ym2149_audio` is wired up.)
- [ ] Both fixes are candidates to raise upstream with the MiSTer ColecoVision core, which shares
  the vdp18 lineage and also keeps cartridges in SDRAM. Gauntlet's breakup is already confirmed
  there; the Uridium faults are very likely present too.

### #12 Gauntlet - boots; graphics complaint not yet judged

Gauntlet (2019) (Team Pixelboy) (SGM), a 256K MegaCart, reaches its credits screen, takes fire,
and draws its character-select screen (Thor, Questor, Chyra, Merlin) correctly. Everything up to
the maze renders properly, so whatever the 2022 report is about starts later. The 2022 complaint was "flashing blocks on the walls ... even when no other sprites are
moving", which does not happen on real hardware or the ColecoVision core, and rampa069 later
found that a build using an F18A-style VDP fixed both this and Uridium - which points at the VDP
rather than the memory map.

- [x] **Does not reproduce.** Played in the maze on a DE10-Nano on 2026-09-18 with
  `ColecoAdam_20260913_accuracy.rbf`: no flashing blocks on the walls. The 2022 report is fixed,
  whichever change did it. The cartridge and a `C4 Gauntlet` launcher are on the MiSTer, in
  `games/Adam/_accuracy_tests/` and `_AdamTests/`.
- **The maze breaks up when moving left and right**, noticed on hardware on 2026-09-18 and shown
  in two screenshots: wall runs step between tile rows, as if part of the screen holds the map
  from one frame and part from the next. The game otherwise plays well. Ruled out so far:
  - the fifth-sprite fix below - it changed nothing here;
  - the SDRAM wait fix, which cured Uridium completely and left this untouched;
  - **the stock MiSTer ColecoVision core in SGM mode does the same**, which is no surprise since
    this core started from the ColecoVision parts. A shared bug in the vdp18 lineage rather than
    anything this branch did, so a fix belongs upstream too.
  - [x] **Settled 2026-09-19: not a core bug. The game tears itself, and real hardware does it
    too.** The footage question above is answered - "Gauntlet - Colecovision" by ed1475, uploaded
    2019-02-01, whose description says "Recorded using the real hardware" with the Opcode SGM,
    shows the same breakup: https://www.youtube.com/watch?v=olxiDo_rVLo

    Reproduced in simulation and taken apart with the capture hotkeys and the VRAM write log
    (`captures/writes.txt`, `frame scanline addr data`). Gauntlet splits its screen update
    across two frames and puts half of it inside the visible picture:

        frame N    rows 0-7  written at scanlines 192-213   (blanking, safe)
        frame N+1  rows 8-15 written at scanlines  85-117   (mid-picture, tears)
        frame N+2, N+3  nothing

    Across 51 captured frames the correlation is exact: every frame whose picture was drawn while
    those 256 writes were happening shows the wall displaced by one tile, and every frame without
    them is clean. It is not a case of running out of vblank and spilling over the edge either -
    the whole 4.3 ms blanking was free and the game wrote at scanline 85 anyway, a frame later.

    `verilator/compare/tools/vdpref.py` re-renders the background from the captured tables and
    confirms the core draws its VRAM exactly; the differing cells are only the ones a write
    crossed. So the VDP is faithful and the tear is the program's.

    **Why an F18A appeared to fix it.** The F18A renders a whole scanline ahead into a line
    buffer (`f18a_tiles.vhd`, `prescan_start` at `raster_x = 1`), so a write landing mid-line
    changes the *next* line rather than the one being drawn, and the tear is hidden. That is
    masking, not fixing, and it is less faithful than the real chip - the same direction as its
    dual-ported VRAM, which drops the access windows the real part has. rampa069's 2022 note on
    this issue was reading a nicer picture as a more correct one.

    Two claims in the older notes above are wrong and left only for the record: the 2022 report's
    "does not happen on real hardware" (it does), and the idea that the game follows the beam
    using the fifth-sprite number (it writes at a fixed point in the frame instead).
- Tearing was also noticed on the monitor, separately from that. It is not the core: six screenshots
  taken while it was happening are all clean, and a MiSTer screenshot comes from the core's own
  framebuffer, so a tear the core produced would be in them. `/media/fat/MiSTer.ini` has
  `vsync_adjust=0` with `video_mode=0`, so HDMI runs at a fixed 60 Hz against the core's 59.92 Hz
  and a tear line rolls through. `vsync_adjust=1` is the fix, and it applies to every core.

### #12 multicarts and reset

rampa069 noted in 2022: "if a cart don't start, try to load another (eg: dragons lair) or reset
another time ... once it starts loading seems to be stable."

- [ ] Check on hardware whether loading cartridges back to back is reliable, since `game_reset`
  and the reset path changed on this branch (fix 4).

## Old notes, checked

- **Right:**
  - Tape writes need checking.
  - 256K, 512K and 1MB expanders exist (MicroFox; Lundy 1MB), and T-DOS, CP/M and EOS RAM
    disks use them.
  - Memory beyond 64K needs an addressor.
  - Expansion RAM paging is on port 42h (MESS).
  - MicroFox IDE hard disks exist.
- **Wrong:**
  - The memory map port is 7Fh (decoded across 60h-7Fh), not 30h; 20h-3Fh is AdamNet.
  - Lower bits `10` replace the lower 32K with expansion RAM. SmartWRITER/EOS is lower `00`,
    and upper `10` replaces intrinsic upper RAM, not SmartWRITER.
  - There is no `memory.v`. The map is in `rtl/cv_addr_dec.sv`, the bus mux in
    `rtl/cv_console.sv`, and the RAMs in `ColecoAdam.sv` and `verilator/sim.v`.
  - The core does use SDRAM (for cartridges); RAM, VRAM and ROMs are BRAM.
  - `MISTER_SMALL_VBUF` is commented out in `ColecoAdam.qsf`.
  - 1MB does not fit in the FPGA's BRAM; SDRAM has room.
- **Unconfirmed:**
  - "16 banks of 64K selected by D3:D0".
  - "Page 0 is separate from internal RAM".
  - Slot placement: Lundy puts the 1MB board in slot 3 (right-most) with the addressor in the
    centre slot 2, the reverse of the old note.

## 3a. A better SDRAM controller, if the expander grows

Measured on the 2026-09-18 build, so these are the real numbers rather than estimates:

| | |
|---|---|
| BRAM on the Cyclone V | 691 KB in 553 M10K blocks |
| used by the core today | 444 KB, 466 blocks (84%) |
| free | 87 blocks, about 109 KB at best packing |
| a 512 KB expander needs | 410 blocks - does not fit |
| 1 MB needs | 819 blocks - nowhere near |

The 256K expander already takes 204 blocks, 44% of all the block RAM the core
uses. One more 64K bank would fit, but 320K is not a configuration any ADAM had,
and PowerPAINT shows why that is a bad idea: it sizes its buffers from the bank
count (FD5Fh, read in 46 places), so a program told it has more memory than
exists allocates into banks that are not there. Anything past 256K has to go in
SDRAM.

### Our controller is the weak one

`rtl/sdram.sv` is the single-port controller from the MiSTer template: one request
at a time, and **it refreshes on its own counter**. That self-timed refresh is
what made cartridge reads occasionally late, which was the Uridium and Frogger
bug fixed in `a88e4ff`. Adding expansion RAM to it would mean hanging a second
master off a controller that has no arbitration at all.

### What other cores use

- **`NES_MiSTer/rtl/sdram.sv` - the best fit.** Three **byte-wide** channels, each
  with `addr`, `rd`, `wr`, `din`, `dout` and `busy`, arbitrated by priority in its
  idle state. Two properties matter for us:
  - each channel remembers the last 16 bit word it fetched and skips the access
    when the next byte is in it, which is what a Z80 walking through memory does
    almost every cycle;
  - **refresh is an input**, so the core schedules it - during blanking, say -
    instead of it landing in the middle of a CPU read.
  A Z80 machine wants byte channels and a busy line, and `busy` drops straight
  into the wait chain `cart_wait_n` already uses. ch0 for the cartridge, ch1 for
  the expander, ch2 spare.
- **`Genesis_MiSTer/rtl/sdram.sv`** and **`MegaCD_MiSTer/rtl/sdram.sv`**: word-wide
  ports with `req`/`ack`. Fine designs, but 16 bit oriented, so every Z80 byte
  access would need the halves picking apart.
- **`SNES_MiSTer/rtl/sdram.sv`**: two 16 bit ports plus a host port, and the most
  complex of them. More machinery than this core needs.

Timing is not a problem: the NES controller is written for up to 128 MHz with its
delays sized for 85 MHz, and our `clk_sys` is 42.666 MHz, so every constraint is
met with room to spare.

- [x] Port `NES_MiSTer/rtl/sdram.sv`, cartridge on ch0 and expander on ch1.
- [x] Do it only once the card behaviour below is known. How many bits of port 42h
  a real 512K or 1MB card latches, and whether both 32K windows follow the same
  bank, decides the address map - and guessing at exactly those semantics is what
  produced the PowerPAINT bug in the first place.

### What was actually built, 2026-09-18

Two channels rather than three - there is no third master - and `ready` per channel
rather than `busy`, so the wait chain keeps the shape `cart_wait_n` already had.
Two things were changed from the NES design on purpose:

- **Refresh stayed internal.** Making it an input only helps if the core has a
  quiet window to spend it in, and driving it from video blanking cannot work:
  the part needs a refresh every 7.8 us and a scanline is 63.7 us, so blanking is
  eight times too slow. Instead an overdue refresh outranks the channels and a
  merely due one waits for a slot nobody wants. Now that `a88e4ff` makes the CPU
  wait properly, a refresh landing on a read costs a wait state instead of
  corrupting the read, which is what it used to do.
- **A cached read never enters the state machine.** The NES version runs the full
  seven clock slot even when the byte is already in the channel's latched word,
  which would have put a wait state where the single port controller had none. The
  fast path answers from the register with `ready` never dropping, so the cartridge
  path costs exactly what it did before.

Requests are also captured the moment a strobe rises rather than when the
controller is free, because `ioctl_wr` is a single cycle pulse, and reads and
writes queue separately so a Z80 writing a byte and reading it straight back
cannot have the read answered from the write's address.

SDRAM map: cartridge at 000000-0FFFFF, expander at 200000-3FFFFF. The two must not
overlap - a write on one channel does not invalidate the other's cached word.

### Verified by `verilator/sdram_tb/`

The core's own simulator cannot check this: it clocks `clk_sys` at the 10.7 MHz
rate with `ce_10m7` tied high, so a seven clock slot there would be two and a half
Z80 clocks instead of the half clock it is on hardware, and every access would look
like a stall. So the controller has its own testbench against a behavioural chip
model. `cd verilator/sdram_tb && make`. It checks round trips on both channels
across bank, row and column bits, that sequential bytes hit the cached word (32 of
64 reads free), that the channels do not evict each other (32 of 64 free when
interleaved), a read queued behind a write, and the refresh rate.

Writing it caught the one thing worth catching: the exact cycle the chip's data is
on the bus. The board drives `SDRAM_CLK` from `~clk_sys`, so the part clocks half a
cycle ahead of the controller, and the read has to be sampled half a cycle *after*
the CAS latency edge. The anchor for that is the single port controller this core
shipped with, which samples three controller clocks after issuing READ and works on
the hardware; the model is built to match it, and is commented to say so, because
a model that released the bus half a period earlier would fail a correct controller.

### Confirmed on the MiSTer, 2026-09-18

Built with Quartus 17.0.2 and run on the DE10-Nano as
`_Computer/ColecoAdam_20260918_expander.rbf`, md5 e89c476ef186dce68a64b70ac1e774c3.

The bank walk cartridge (`hardware_tests/banktest/`, MGLs R7-R12) paints the screen
with the number of banks it found. Every size is right, and every colour is the
same value the simulator produced:

| setting | hardware | |
|---|---|---|
| None | black | nothing answers |
| 64K | black | no bank register, every bank aliased |
| 256K | rgb(33,200,66) green | 4 banks |
| 512K | rgb(84,85,237) blue | 8 banks |
| 1M | rgb(252,85,84) red | 16 banks |
| 2M | white | **32 banks, all distinct** |

PowerPAINT (MGLs P1-P4) reads 64, 256 and 512 at the matching settings, so the
sizer that started this whole thread now agrees with the hardware.

Nothing regressed: Uridium - the cartridge whose corruption `a88e4ff` fixed - draws
its title screen cleanly, and SmartWRITER, Frogger, Super Cobra and Donkey Kong Jr
all boot. The 179 cartridge sweep is unchanged at 170 PASS, 2 DRIFT, 7 REVIEW with
the same titles in each bucket.

- [x] Sustained expander traffic, checked 2026-09-21: **RAMTEST v2.0 (2018) by Eric Pearson**,
  who designed the EXPAnDDR expander, reports "32 Banks Detected" with the 2M setting on a
  DE10-Nano and walks them. Independent confirmation of the full 2MB from software written
  for real expander hardware, and it works the channel far harder than the bank walk does.
- [ ] RAMTEST v2.0 does not run in the simulator. The disk boots, the program takes the VDP
  (R0=00 then R1=E0 at frame 16) and writes no further register for 7,000 frames, leaving a
  black screen with VRAM all zeros. Same at every expander size including None, so it is not
  the expander. A disk boot that does work, PowerPAINT, sets R0 at frame 16 then R1, R2, R3,
  R4 and R7 at frame 18. The same disk runs on hardware, so this is a simulator problem -
  likely AdamNet or disk timing - and worth finding, because it costs us a test the hardware
  can run and the simulator cannot.

### What it cost, from the Quartus 17.0.2 fit

| | before (20260918) | after |
|---|---|---|
| M10K blocks | 466 / 553 (84%) | **210 / 553 (38%)** |
| block memory bits | 3,638,757 | 1,541,605 |
| ALMs | 15,551 (37%) | 15,293 (36%) |
| clk_sys setup slack | - | +3.586 ns of 23.432 |

The 256 blocks the 256K expander used are back, and the core now has room for
whatever wants block RAM next. The two channel controller costs slightly *fewer*
ALMs than the single port one it replaced, which is the word cache and the boot
sequence being simpler than the old controller's eight idle states. Timing passes
with no failing paths at all - TNS is 0.000 on every clock.

## 3b. How the expanders bank, settled 2026-09-18

The bank number is written to port 42h as **data**, and the memory card latches it. See
`docs/memory_expanders/README.md` for the evidence, which comes from three open hardware designs
and two manufacturers' manuals. In short: the addressor in the centre slot decodes the write and
pulses one wire; the memory card latches D0-D7 on that edge and drives its upper address lines,
so the register is as wide as the card - 2 bits for 256K, 3 for 512K, 4 for 1MB, 5 for 2MB.
Bank 0 at power-up. A 64K card has no bank logic at all and needs no addressor.

So an expander larger than 256K needs, on top of the SDRAM work in section 3a:

- [x] widen `exp_ram_bank` from 2 bits to 3 or 4, and widen the "bank past the last one fitted"
  test that `2f68257` added. Done as `rtl/cv_expander.sv`, which now holds the whole decode -
  windows, bank register, absent test - and is shared by `ColecoAdam.sv` and `verilator/sim.v`
  so the two cannot drift. Five bits of bank, so 2MB.
- [ ] extend the OSD list past 256K (the simulator's `--exp-ram` already takes
  `64|256|512|1024|2048|none`; the OSD still has to move to a three bit field);
- [x] nothing else about the interface - port 42h and the data bus are already right.

### Proved in simulation, 2026-09-18

`hardware_tests/banktest/` builds a cartridge that writes `bank^5A` at offset 0000 and
`bank^A5` at offset 4000 of all 32 banks, reads them all back, and reports the run of good
banks from bank 0. Results come out in VRAM at 3800h for the simulator and as the backdrop
colour for a screen, so the same cartridge is the hardware check once the expander is in SDRAM.

| setting | banks counted | read back | PowerPAINT shows |
|---|---|---|---|
| None | 0 | all FF | 64 |
| 64K | 0 | every bank the last write, `45` | 64 |
| 256K | 4 | 5A 5B 58 59 then FF | 256 |
| 512K | 8 | through 5D then FF | 512 |
| 1M | 16 | through 55 then FF | 512 |
| 2M | 32 | all 32 distinct, to 45 | 512 |

Both columns are right. The 64K card aliasing every bank to one 64K is what a card with no
bank register does, and PowerPAINT stopping at 512 is its own sizer saturating: it counts to
four banks past the base and then stores code 7, so it cannot tell 512K from 2MB. No period
software we have counts higher, which is why the test cartridge exists.

- [ ] Ask Eric Pearson (EXPAnDDR, MIB238, RAMTEST v2.0) or Michael Carter (Coleco-Cheap-Memory,
  Coleco-2MB-Memory) to confirm the original Orphanware and Micro Innovations cards latch the same
  way. Both are active on GitHub. Their designs are built to work with the original addressors and
  software, so the interface has to match, but the original boards' own decoding is still inferred.
