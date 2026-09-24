# Handoff: testing branch `sdram-expansion` on the MiSTer

Branch `sdram-expansion`, ahead of `master` (`b6af71b`). The bitstream below was built from
`932e977`; commits after that one are documentation unless section 2 says otherwise. Everything
here was checked in the Verilator simulator against ColEm 5.6; **none of it has been on hardware
except the memory expander**, which was checked on a DE10-Nano on 2026-09-19. This document is the
plan for the rest of that testing. `STATUS.md` has the evidence for each fix, `TODO.md` the open
work, and `CLAUDE.md` how to build and run the simulator.

The previous handoff covered branch `adam-accuracy-fixes`. That is merged and released as
`releases/ColecoAdam_20260918.rbf`, so its twelve fixes — console RAM mirroring, text mode
position, the M1 WAIT state, cartridge reset, the 64K expander's upper half, port range decoding,
tape writes, AdamNet gating, PCB relocate, DCB status write-back and spinner/roller support — are
the **baseline** this branch builds on, not the thing under test. Its hardware results are kept in
section 6 because several of its checks were never finished.

## 1. The build to test

`docs/builds/ColecoAdam_20260922_refresh.rbf`, md5 `d06c748cfec60b315e1198c40f69eef2`.

Not a release. Copy it into `/media/fat/_Computer/` on the SD card; the `ColecoAdam_*` name means
MiSTer picks it up automatically and it sits beside the existing core, so going back is just
picking the old one. `docs/builds/TESTING.md` is the short version of this document to hand to
someone else, with `banktest.col` and `uppertest.col` beside it.

Built with Quartus 17.0.2 Lite, 0 errors, 156 warnings, in 5m21s. Build it again with:

    git worktree add /tmp/build 932e977
    cd /tmp/build && quartus_sh --flow compile ColecoAdam

A worktree at the commit rather than the working tree, so there is nothing to verify by hand — the
bitstream is exactly that commit. No `.qsf` or `files.qip` change is needed for anything in this
branch.

| Resource | `20260918` release | This build |
|---|---|---|
| RAM blocks | 466 / 553 (84%) | **210 / 553 (38%)** |
| Block memory bits | 3,638,757 (64%) | 1,541,605 (27%) |
| Logic (ALMs) | 36% | 37% |
| `clk_sys` setup slack | +0.301 ns | **+5.406 ns** |

Block RAM more than halved because the expander left it for SDRAM. The worst-case setup slack in
the log is 0.488 ns on the HDMI PLL, which is MiSTer framework rather than core; every clock is
positive with TNS 0.000.

## 2. What changed since master

| # | Change | Files | Behaviour you'll notice |
|---|---|---|---|
| 13 | **Memory expander moved to SDRAM**, and grew: OSD **Expansion RAM: 64K / 256K / 512K / 1M / 2M / None**. A bank is 64K, selected by port 42h and seen through the two 32K windows of port 7Fh | `rtl/cv_expander.sv` (new), `rtl/sdram.sv`, `rtl/cv_console.sv`, `ColecoAdam.sv`, `files.qip` | Cards past 256K work; a bank the card does not have no longer aliases back to bank 0 |
| 14 | **The Z80 has its R register again.** `TV80_REFRESH` was defined in neither build system, so `LD A,R` returned a constant 0, `LD R,A` was discarded and `RFSH_N` was tied high | `rtl/tv80/tv80_core.v`, `rtl/tv80/tv80e.v` | Cosmo Fighter II has a star field. Any game seeding an RNG from `R` behaves differently — for the better |

Two things about 14 are worth knowing before testing:

- The define now lives **in the RTL**, guarded by `` `ifndef ``, rather than in `ColecoAdam.qsf`
  and `verilator/Makefile`. The two build systems disagreeing is exactly how it came to be off
  everywhere, and an in-file define cannot be lost the same way. It is repeated in both files
  because neither Quartus nor Verilator promises an order for them.
- It also makes `MREQ_N` pulse during M1 T4 with `{I, R}` on the address bus — **a bus cycle on
  every instruction fetch that did not exist before.** `cv_addr_dec.sv` and `cv_console.sv` are the
  only files in the tree that touch `mreq_n`, and all twelve sites already qualify on `RFSH_N`, so
  those cycles are ignored. The decode was written for refresh from the start; the gating had been
  sitting as dead code. This is the one part of the change that wants hardware confirmation rather
  than simulation, because it touches every memory access in the machine.

Simulator, tooling and documentation changes with no effect on the FPGA: `--record`/`--replay` and
the `[`/`]`/`\` capture keys, per-frame `.vdp` state dumps and the `vdpref.py` software reference
renderer, the two-player keyboard fix, the SDRAM testbench in `verilator/sdram_tb/`, and the
`hardware_tests/banktest` and `hardware_tests/vramtest` cartridges.

## 3. Hardware checklist

Quickest and most important first. Note anything that differs from "Expect".

### The R register (change 14) — all new, nothing checked on hardware

- [x] **Cosmo Fighter II has a star field.** Confirmed on hardware 2026-09-24: stars on the title
      and credits screens. The whole point of the change. In simulation it goes
      from 0-3 dots a frame to roughly ColEm's hundred.
- [x] **A general pass, because this changes CPU behaviour for every game.** Passed 2026-09-24:
      Frogger and Super Cobra in attract play, DK Jr to its menu, Uridium in play on Zinc, SmartWRITER
      typed "hello world" with nothing lost, DK Jr disk and Troll's Tale tape boot, T-DOS 4.5 to
      `A0>` at 256K, PowerPAINT at 512K reads 512, ADAM Diagnostic to its checkout screen. Frogger, Donkey Kong
      Jr, Super Cobra and Uridium boot and play; SmartWRITER boots and types; a disk boots. In
      simulation 176 of 179 cartridges are bit-identical and the other three moved by under 0.4%,
      but that is Console mode against ColEm, not hardware.
- [x] **The expander still works with refresh on** (2026-09-24: `banktest` 2M white, `uppertest`
      all six settings correct) — the bank test below. This is the pairing the
      simulation covers least well: the new `MREQ` pulses and the newest memory path together.

### Memory expander (change 13) — partly checked on hardware 2026-09-19

- [x] All six Expansion RAM settings via `banktest.col`, colours matching the simulator exactly.
- [x] PowerPAINT SYSTEM STATUS reads 64, 256 and 512. It reads 512 for 1M and 2M as well; that is
      its own sizer saturating after four banks, not the core.
- [x] RAMTEST v2.0 (Eric Pearson) prints "32 Banks Detected" at the 2M setting and tests them.
- [x] Uridium, SmartWRITER, Frogger, Super Cobra and Donkey Kong Jr still boot.
- [x] **`uppertest.col`** Passed 2026-09-24, all six settings, each screen one flat colour matching
      the simulator. The first run came back as garbage screenshots because the cartridge left the
      display blanked; it now paints the screen as `banktest.col` does. — the same 32-bank walk through the **upper** 32K window, which
      `banktest.col` never exercised. Simulation says the two agree. Load it in Computer mode and
      read the backdrop: black none/64K, green 256K, blue 512K, red 1M, white 2M.
- [ ] **Anything that hammers the expander**: T-DOS, a RAM disk, PowerPAINT doing real work at
      512K or larger. The bank walk proves addressing, not sustained traffic with the cartridge
      busy at the same time.
- [ ] **RAM disk contents surviving a reset**, as they do on hardware.

### Carried over from the 20260918 release, never finished

These are baseline checks, not this branch's, but they are still open and the SD card is already
set up for them.

- [ ] **Search for the Stolen Crown Jewels I** text screens are centred. Not on the MiSTer's card.
- [ ] **An SGM title** runs with AY sound. Not on the card. `config/CVSgm.CFG` is ready for it.
- [ ] **Buck Rogers Super Game high score save from its data pack**: play to the end, enter
      initials, DONE?. The **disk** version is confirmed working by a tester; the DDP path is the
      one still unconfirmed, and it is the original bug report.
- [ ] **The spinner and roller checklist** in `TODO.md` section 6a. The 20260918 release contains
      `cv_spinner.sv`, so this needs no new build — Bruce's Controller Tester, Breakout for Roller
      Controller, and a real roller game. Set OSD **Spinner** to *Stick X* for a gamepad. Check
      Off first: nothing should respond.
- [ ] **The rest of the ADAM Diagnostic checkout**, and OSD Reset returning to SmartWRITER.

The RAM test cartridge colours from the old checklist are superseded: `banktest.col` covers the
same ground and has passed on hardware, and it reports a bank count rather than one pass/fail
colour. `verilator/compare/tools/adam_exp_ramtest.py` still builds the older pair if wanted.

## 4. Backing a change out

| Change | Edit |
|---|---|
| 13 Expander size | OSD **Expansion RAM = 64K**, the plain expander, or **None**. No rebuild |
| 13 Expander entirely | It is the only user of `cv_expander.sv`; going back means reverting `337ebcd`, which also reverts the `sdram.sv` rewrite the cartridge path now shares |
| 14 R register | Delete the `` `ifndef TV80_REFRESH `` block from **both** `rtl/tv80/tv80_core.v` and `rtl/tv80/tv80e.v`. Removing it from only one leaves the halves disagreeing about whether `R` exists |

The fixes from `master` keep the switches listed in the 20260918 handoff; they are in that
release's history if one of them needs disabling.

## 5. Simulation results at handoff

- **179 cartridges** in Console mode against ColEm: **171 PASS, 2 timing DRIFT, 6 REVIEW.** That
  is the committed baseline's 170/2/7 with Cosmo Fighter II moved from REVIEW to PASS, which is the
  only verdict this branch changes. Derived rather than read off a single sweep: `finalize.sh` was
  not re-run over the R-register sweep, so its own summary still shows the two DRIFT titles as
  REVIEW. Of the six REVIEW, all six are explained as not core bugs.
  - Cosmo Fighter II improves on every metric: shot fgmatch 0.9442/0.9530/0.9558/0.5324/0.4437 to
    0.9747/0.9704/0.9628/0.6073/0.5382, still_match 0.9791 to 0.9842.
  - 176 cartridges are bit-identical. Squares and Squish 'Em Sam shift by under 0.4% on one shot
    each **in both directions** — animation phase from a game RNG that now has a live `R`.
- **12 ADAM scenarios**: 11 identical, and Diablo's `worst_fg` improves 0.9611 to 0.9647. Both
  SmartWRITER scenarios match in every metric, which is the AdamNet keyboard path the new refresh
  cycles would have disturbed.
- **Gauntlet's maze breakup is not a core bug** and is closed. The game rewrites rows 8-15 mid
  picture, at scanlines 85-117; real hardware tears identically (YouTube `olxiDo_rVLo`, recorded
  from real hardware with an Opcode SGM). An F18A hides it by rendering a scanline ahead into a
  line buffer, which is masking and less faithful, not a fix.
- A caution when reading sweep output: a row with `core_exit` other than 0 is a **run** failure,
  not a result. Long sweeps run as background jobs, and when a job's process group is torn down it
  takes the in-flight `Vemu` children with it — that shows up as eight or so alphabetically
  adjacent FAILs, each having scored `match=1.0000` on every shot it reached before dying. Discard
  those rows and re-run them; do not read them as regressions.

## 6. Hardware results from the 20260918 baseline (2026-09-13 and 2026-09-19)

Kept because section 3 carries several of its unfinished items forward. Driven remotely with the
MGL files, scripts and virtual keyboard and pad in `hardware_tests/`, judged from MiSTer
screenshots.

| Check | Result |
|---|---|
| Frogger / Super Cobra / Donkey Kong Jr, Console mode | Pass |
| SmartWRITER boot and typing, first key included | Pass |
| DK Jr Super Game disk boot | Pass to player selection |
| Troll's Tale tape boot | Pass within 40 s |
| Buck Rogers Super Game tape **boot** | Pass to player selection |
| Tape **save** | Pass: blank data pack changed by 1,050 bytes, as in simulation |
| Tape read-back after reboot | Pass: GET on drive A lists the file |
| Cartridge reset | Pass: ADAM Diagnostic reaches MEMORY MODULE TEST |
| Keyboard in an ADAM cartridge | Pass: lowercase `c` opens CHANGING CONFIGURATION |
| T-DOS 4.58 at 256K | Pass: `A0>` prompt |
| Disk writes, incl. a 1.44MB DSK (2026-09-21, tester) | Pass: SmartWRITER writes, and Buck Rogers DSK saves its high score |
| All six expander settings (2026-09-19) | Pass: `banktest.col` colours match the simulator |
| RAM test cartridge colours | Not judged: every screenshot came back a stale capture of SmartWRITER. Superseded by `banktest.col` |

## 7. Not in the branch

These stay local and are ignored or untracked:
- `ColEm56-Source/`: its licence forbids redistribution.
- `verilator/SoftwareFromMiSTer/`, `verilator/roms colecovision/`, `verilator/adam.tar` and the
  loose test disks and zip in `verilator/`.
- `verilator/compare/work/`, `verilator/captures*/`, `verilator/recordings/`: generated.
- `docs/`, including `docs/builds/` and the `.rbf` above: 98 MB of datasheets, schematics and
  manuals plus test builds.

The simulator and comparison framework need those local files; see `CLAUDE.md`.

## 8. Where to pick up

- **Test this build**, section 3. Cosmo Fighter II's stars and the general pass are the new work;
  the rest is carried over.
- **Then merge to `master` and release.** The release note **must** say that saved settings read
  as 64K: Expansion RAM needed a third bit and moved from status bits 4-5 to 17-19, so any `.CFG`
  written by an older core selects 64K whatever it used to say. Set it again in the OSD and re-save.
- Open work is in `TODO.md`. Nothing below blocks the release:
  - Tape (DDP) writes still unconfirmed on hardware — the DDP Buck Rogers save is the test.
  - The ColEm Text-mode patch is half applied, a tooling bug: `setup.sh` moves the anchor to +6 but
    `RefreshBorder()` still centres at +8 and runs after the text, so it clobbers columns 6-7 and
    never paints 246-247. It depresses every Text-mode score with a non-black backdrop and has no
    effect on the core.
  - RAMTEST v2.0's program blocks never load **in simulation**; only the boot block arrives. It
    runs correctly on hardware, so this is the harness.
  - System Hardware Test and ADAM Final Test 3.3 both run; what is left is why parts report FAIL.
  - The rest of the library sweeps, and the triage titles in section 0 of `TODO.md`.
  - A Pitfall II hang was reported from a triage run. **Treat it as unconfirmed**: the standard
    sweep has `Pitfall! II - Lost Caverns` at PASS with `match=1.0000` on all five shots, both
    before and after the R register change, so it needs re-deriving before it is worth chasing.
