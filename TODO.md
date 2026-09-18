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
- [ ] **B. Uridium's freeze** (section 8). The one open GitHub issue that still reproduces.
- [ ] **C. Gauntlet's maze** (section 8), to settle the rest of that issue.
- [x] **D. The "black-screen" test cartridges** - they are not black. In Computer mode, where an
  ADAM diagnostic belongs, System Hardware Test reports "FAIL CONTROLLER PORT #1", "FAIL AUX.
  VIDEO" and "FAIL AUX. AUDIO", and ADAM Final Test 3.3 waits at a "STATION ID -" prompt for
  keyboard input. See section 4 for what is left of it.
- [ ] **E. Triage the sweep titles never looked at** (section 5): Pitfall II, Zenji and
  Squish 'Em Sam differ; Adventure Pack 1-3, Evolution, Journey to Maraud Mountain and
  Tournament Tennis are close.
- [ ] **F. Cosmo Fighter II's star field** (section 6). The oldest open bug and the most work:
  a CPU trace against ColEm up to the first star write.

Needs hardware, not this machine: the rest of section 1, the spinner checklist in 6a (which also
needs a new Quartus build, since no `.rbf` on the MiSTer contains `cv_spinner.sv`), and the
multicart reset check in section 8.

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
- [ ] Buck Rogers high score save, played to the end on hardware.
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
- [ ] Do the same with the `.dsk` version, to separate tape from disk.
- [ ] Run the ADAM Tape-Disk Verification Rev. 1 cartridge (`adam_carts/`) against a blank DDP
  from `Blank Media/`.
- [ ] Re-run the July write tests: `_wrtest/cpmtest.dsk` against `_wrtest/golden.dsk`, and
  `wordtest.dsk`.
- [ ] If the simulator writes correctly but hardware doesn't, look for the same kind of SD
  handshake problem that broke floppies (sd_ack vs a shared, stale sd_buff_addr).

## 3. Expansion RAM

- [ ] **Bug: the upper half of the 64K Memory Expander is missing.** Port 7Fh upper bits `10`
  select 32K Expansion RAM (ADAM Technical Manual 2.2).
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
  - [ ] RAMTEST v2.0 (Eric Pearson) and RAMTest Rel 2.0 (Orphanware)
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
- [ ] The original item, for the remaining sizes: add an OSD option, Expansion RAM: None / 64K / 256K / 512K / 1MB. Keep "None" for
  software that behaves differently with an expander fitted.
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
- [ ] **Put large expansion RAM in SDRAM.** 1MB does not fit in BRAM: the Cyclone V has about
  5.5 Mbit (~700 KB) of M10K in total, shared with the MiSTer framework, and the core's RAMs and
  ROMs already take about 1.4 Mbit.
  - The core already requires SDRAM and uses it for cartridge ROM (the first 1 MB).
  - Map the expansion at an offset such as 100000h and share `sdram.sv`'s single request port;
    the Z80 only makes one access at a time.
  - Give `sim.v` the same memory.
- [ ] Test beyond-64K software:
  - [ ] T-DOS and CP/M 2.2 RAM disks
  - [ ] EOS RAM disks (ADAM's Desktop, SmartDSK, MegaDisk)
  - [ ] RAM Test v1.3 & Utilities (AJM, CP/M)
  - [ ] RAMTEST v2.0
  - [ ] RAM-disk contents surviving a reset, as they do on hardware
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
    - [ ] Find what "FAIL CONTROLLER PORT #1" is testing: the string is in the cartridge, so
      disassemble backwards from it. It is not the spinner lines - the result is identical with
      a spinner turning - and it is not this branch's `cv_ctrl` change either, since with the
      spinner idle that port reads exactly as it did before (7Fh).
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

## 6. Known bugs

- [ ] Cosmo Fighter II's star field is missing. Compare a CPU trace against ColEm up to the
  first star write.
- [ ] Re-check README's known bugs, then update README (its "tape/disk write is not supported"
  line is out of date for floppies):
  - [x] a bad character the first time something is typed: fixed in commit 1029386 (July
    2026) and checked on a DE10-Nano. EOS used to read the key after the PS/2 value had moved
    on to the release event (^@), and a blocked keyboard read swallowed keystrokes.
  - [ ] reset not quite like an ADAM
  - [ ] no printer
  - [ ] key repeat

## 6a. Spinner and roller controllers

Implemented on 2026-09-18 (`rtl/cv_spinner.sv`, `rtl/cv_ctrl.sv`, OSD "Spinner"); see `STATUS.md`
and the references in `docs/controllers/`. What is left:

- [ ] Run the hardware checklist for it, `HANDOFF.md` section 3, "Spinner and roller controllers".
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
- [ ] Tick Broderbund off #12, which stays open for Uridium and Gauntlet.

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

- [ ] Log VDP register 1 writes to find where the display is turned off and what the program is
  waiting for before it would turn it back on. Also log `megacart_page`: both this and Gauntlet
  are MegaCarts, and a level load reading the wrong bank would look exactly like this.
- [ ] Check whether the AY is the thing being waited on. Ports 50h/51h/52h are decoded and
  `ym2149_audio` is wired up, so reads return something, but an SGM detection routine that expects
  particular values would not know that.
- [ ] Compare against the MiSTer ColecoVision core to confirm it fails there too, and if so raise
  it upstream rather than here.

### #12 Gauntlet - boots; graphics complaint not yet judged

Gauntlet (2019) (Team Pixelboy) (SGM), a 256K MegaCart, reaches its credits screen, takes fire,
and draws its character-select screen (Thor, Questor, Chyra, Merlin) correctly. Everything up to
the maze renders properly, so whatever the 2022 report is about starts later. The 2022 complaint was "flashing blocks on the walls ... even when no other sprites are
moving", which does not happen on real hardware or the ColecoVision core, and rampa069 later
found that a build using an F18A-style VDP fixed both this and Uridium - which points at the VDP
rather than the memory map.

- [ ] Get into the maze and compare consecutive frames for blocks that flash on a still screen.
- [ ] If it reproduces, this is a VDP bug and belongs with the Cosmo Fighter II star field in
  section 6: both are `rtl/vdp18v` behaviour that ColEm and the F18A get right.

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
