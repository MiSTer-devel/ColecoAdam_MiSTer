# Core vs ColEm comparison

Runs the Verilator simulator and a headless build of ColEm 5.6 with the same ROMs, media and
input, and compares their screens. It covers ColecoVision cartridges and Adam mode (SmartWriter,
disks, tapes, typed input). Nothing here is needed to build or run the simulator itself.

## Requirements

- The simulator built: `make` in `verilator/`
- `clang`, `python3`, `ffmpeg`, `xxd`
- ColEm 5.6 source in `ColEm56-Source/` at the repo root, or set `COLEM_SRC`. ColEm's licence
  does not allow redistributing it, so keep it out of commits. `setup.sh` builds from a private
  copy under `work/`. It adds the option to pass ROM file paths and makes the two hardware
  corrections described under "Details worth knowing".

## Setup

Run once, and again after rebuilding the simulator or changing the ROMs in `verilator/rtl`:

    ./setup.sh

This converts the core's BIOS, SmartWriter and EOS ROMs to `work/roms/` so both emulators run the
same code, builds `work/cmpframes` and `work/colem/colem_ref`, and records the core's
no-cartridge screen. Everything generated goes in `work/` (ignored by git).

## Cartridges

    ./run_carts.sh work/carts 12      # every .col and .rom in "roms colecovision"
    ./finalize.sh work/carts          # wide-window rescore, summary, work/carts/report/index.html

One cartridge: `./validate_cart.sh "../roms colecovision/Frogger I.col" work/carts`

Each cartridge runs 1300 frames in console mode with keypad 1, fire and stick input, and is
compared with ColEm at frames 120, 400, 740 (before any input), 1000 and 1300. Results are
PASS, DRIFT (the same screens a few frames apart), REVIEW or FAIL. Put `notes.json` and
`findings.json` in the output folder to record verdicts on the report page.

## Adam mode

    ./adam_scenarios.sh work/adam 11
    python3 summarize_adam.py work/adam

The scenarios are SmartWriter typing, SmartWriter's word-processor key, and a boot of every
`.dsk` and `.ddp` in `roms colecovision`. Add more by editing `adam_scenarios.sh`, or run one:

    ./validate_adam.sh work/adam hello 1500 --type "HELLO ADAM@600" --key enter@700
    ./validate_adam.sh work/adam dkjr 3000 --disk 1 "../roms colecovision/Donkey Kong Jr (ADAM).dsk" --press 1@2500

Options, given identically to both emulators:

| Option | Meaning |
|---|---|
| `--disk N FILE`, `--tape N FILE` | Mount media in drive N (1-4) |
| `--cart FILE` | Insert a cartridge as well |
| `--type TEXT@FRAME` | Type TEXT from FRAME, 8 frames per key; `\n` is Return |
| `--key NAME@FRAME` | One key: `enter esc bs tab space up down left right home f1`-`f6` (smart keys I-VI) `undo wildcard move store insert print clear delete` |
| `--press KEY@FRAME[:N]` | Controller 1: `0`-`9 star pound up down left right fire1 fire2` |

The core saves a frame every 60 frames and ColEm every 10. Because disk and tape access can run
at different speeds, each core frame is matched against ColEm's whole run rather than the same
frame number. `result.txt` records how well the final screens match and the frame offset,
`align.txt` has every core frame's best match, and `strip.png` shows six core frames above
ColEm's matches. Results are MATCH (final screens agree), CLOSE, DIFFERS or FAIL.

Details worth knowing:

- Keys are spaced 8 frames apart because ColEm holds only one pending key.
- `--type` sends uppercase letters as Shift+letter. Software that reads the keyboard itself
  (ADAM test cartridges, for instance) may expect lowercase and ignore a shifted key, so type
  what a user would press, e.g. `--type "v@900"`.
- SmartWriter prints each line when you press Return, and ColEm drops keys typed meanwhile.
  Leave about 150 frames after a Return before typing more.
- Matching compares pixels in place, so a picture drawn a few pixels to one side matches
  nothing. ColEm puts 40-column Text mode 8 px in from the graphics modes; `setup.sh` changes
  that to the 6 px the TMS9918A data manual gives (Table 3-3, 19-pixel left border against 13),
  which is where the core draws it.
- The ColecoVision and ADAM game boards hold the Z80's WAIT for one clock in every M1 cycle
  (74LS74 U8 on the game board schematic). ColEm leaves that out, so `setup.sh` charges the
  extra T-state on each opcode fetch and interrupt acknowledge. Without it the core runs about
  7 frames behind ColEm by frame 120. `COLEM_M1_WAIT=0 ./setup.sh` builds the unpatched timing.
  The patched ColEm never gets past a green screen in Steamroller, though the core runs it;
  use the unpatched build to check that one.
- `--cart` in Adam mode doesn't compare. The core starts the cartridge the way the ADAM's
  cartridge reset switch does (OS-7, 24K RAM, cartridge), but ColEm switches to ColecoVision
  mode, so ADAM-only cartridges such as the diagnostics differ. Check those by their screens.
- The core writes disk and tape images back, so it always gets a copy.
- ColEm only accepts 163,840-byte disk images. Images of 164,352 bytes whose last block is `E5`
  filler are trimmed for ColEm (the core gets the original), and `result.txt` notes it.

## Software library

`verilator/SoftwareFromMiSTer/` holds the MiSTer ADAM library (about 2,000 disks and 170 tapes).
Boot every title in one folder, skipping alternate dumps and image sizes ColEm cannot read:

    ./library_scenarios.sh work/games "../SoftwareFromMiSTer/E.O.S/Games" 12
    python3 summarize_adam.py work/games

Each boot takes about 5 minutes of simulator time, so a 12-job run covers roughly 140 titles an
hour. ColEm doesn't boot ADAM CP/M or T-DOS disks (it shows only the backdrop while the core
reaches the `A>` prompt), so check `CP-M & T-DOS/` and CP/M-based games with `screens.sh`. The ADAM test cartridges in `adam_carts/` and `colrom/` run with
`./validate_adam.sh work/adam NAME 1800 --cart FILE`.

## Core only

For programs ColEm can't run the same way, such as ADAM cartridges, test programs and odd
image sizes, `screens.sh` runs the core alone:

    ./screens.sh work/screens ram64k 1800 --adam --cart "../SoftwareFromMiSTer/colrom/64K Expansion RAM Test (198x) (Coleco).rom"

It leaves `contact.png`, with one tile every `EVERY` frames (default 60), plus the frames.
`result.txt` also lists how many bytes the run changed in each mounted image, which shows
whether software saved anything. The core writes to copies under `media/`.

## Other tools

- `tools/vdpdiff.py CORE.vdp COLEM.vdp` compares VDP registers and VRAM tables from state dumps.
- `tools/adam_exp_ramtest.py OUT.rom` builds an 8K ADAM cartridge that write-verifies both 32K
  halves of the 64K Memory Expander and shows the result as the screen colour. Green passes,
  red means the upper half failed, magenta the lower half, and black both. Run it with
  `--adam --cart OUT.rom`. With `--banks`, it instead writes a different byte into each of
  banks 0-3 through port 42h and reads them back. That is green only with four distinct banks
  (`--exp-ram 256`); a plain 64K expander aliases them and shows black. `--trace` logs progress
  through port 3Fh.
- `work/cmpframes offset|score|align` compares frames by TMS9918 colour index, so the two
  emulators' slightly different palettes do not count as differences.
