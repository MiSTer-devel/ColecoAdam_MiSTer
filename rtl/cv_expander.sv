//-----------------------------------------------------------------------------
//
// ColecoAdam - ADAM memory expander
//
// Decodes the two 32K expansion windows and the port 42h bank register into one
// flat byte address inside the expander card, and says when nothing is fitted at
// the selected bank.
//
// How the real cards work (docs/memory_expanders/README.md, settled from the
// Orphanware MX64/MX128-MX512 and Micro Innovations 64-256Kb/1-2Mb manuals and
// three open hardware designs - Eric Pearson's EXPAnDDR and MIB238, Michael
// Carter's Coleco-Cheap-Memory and Coleco-2MB-Memory):
//
//   - The expander is two cards. The addressor in the ADAM's centre slot decodes
//     a write to port 42h and pulses one wire; the memory card latches D0-D7 on
//     that edge and drives its own upper address lines from the latch. So the
//     bank number travels on the data bus, not on the address bus, and port 42h
//     is a write-only register that the card owns.
//   - A bank is 64K: one 32K half appears in the lower window (0000-7FFF) and the
//     other in the upper window (8000-FFFF), chosen by port 7Fh (ADAM Technical
//     Manual 2.2). Both halves are always in the same bank.
//   - The latch is eight bits wide but the card only decodes as many as it has
//     memory for: Carter's 2MB card latches all of D0-D7 into an ATF22V10 that
//     drives A16-A18 plus four chip selects. A bank number past the last bank
//     fitted therefore selects no chip at all and the Z80 reads open bus, rather
//     than aliasing back to bank 0.
//   - A 64K card has no bank logic and needs no addressor, so port 42h does
//     nothing there and every bank is the same 64K.
//
// That last point is the one that matters for software. A sizer counts banks by
// writing one, writing a byte and reading it back, until a bank stops answering.
// If bank 4 on a 256K card aliased to bank 0 it would answer, and the sizer would
// keep going: PowerPAINT's (disk offset 1300h) reported 512K on our 256K setting
// for exactly that reason before this was decoded properly.
//
//-----------------------------------------------------------------------------

module cv_expander
  (
   // Card fitted, as the OSD's Expansion RAM option orders them.
   input  logic  [2:0] size_i,
   // Port 42h, latched by cv_addr_dec. Eight bits, as the card latches them.
   input  logic  [7:0] bank_i,

   // Lower window: 0000-7FFF when port 7Fh's lower field selects RAM expansion.
   input  logic [14:0] lower_a_i,
   input  logic        lower_ce_n_i,
   input  logic        lower_rd_n_i,
   input  logic        lower_we_n_i,
   input  logic  [7:0] lower_d_i,

   // Upper window: 8000-FFFF when port 7Fh's upper field selects RAM expansion.
   input  logic [14:0] upper_a_i,
   input  logic        upper_ce_n_i,
   input  logic        upper_rd_n_i,
   input  logic        upper_we_n_i,
   input  logic  [7:0] upper_d_i,

   // Flat byte address in the card: {bank, window, offset}. 21 bits reaches 2MB.
   output logic [20:0] a_o,
   output logic        rd_o,
   output logic        we_o,
   output logic  [7:0] d_o,
   // Nothing answers this access: no card fitted, or a bank past the last one.
   output logic        absent_o
   );

  // The OSD list, in order. SIZE_64K is first so that a zeroed status word - a
  // fresh config, or one saved before the larger cards existed - still means the
  // 64K expander the core has always defaulted to.
  localparam logic [2:0] SIZE_64K  = 3'd0;
  localparam logic [2:0] SIZE_256K = 3'd1;
  localparam logic [2:0] SIZE_512K = 3'd2;
  localparam logic [2:0] SIZE_1M   = 3'd3;
  localparam logic [2:0] SIZE_2M   = 3'd4;
  // 3'd5 and up: no card. The OSD only offers 3'd5, but an out of range value
  // must not read as a fitted card.

  // Banks of 64K on the card.
  logic [5:0] banks_v;
  always_comb begin : bank_count
    case (size_i)
      SIZE_64K:  banks_v = 6'd1;
      SIZE_256K: banks_v = 6'd4;
      SIZE_512K: banks_v = 6'd8;
      SIZE_1M:   banks_v = 6'd16;
      SIZE_2M:   banks_v = 6'd32;
      default:   banks_v = 6'd0;   // nothing fitted
    endcase
  end

  // A 2MB card decodes five bits of the latch; smaller ones decode fewer, but
  // the bits above their own count select no chip rather than wrapping, which
  // absent_o below covers. Masking here would be the aliasing we must not do.
  logic [4:0] bank_v;
  assign bank_v = (size_i == SIZE_64K) ? 5'd0 : bank_i[4:0];

  // No card at all, or a bank number the card has no memory for. The 64K card is
  // never absent for a bank reason: it has no bank register to write.
  assign absent_o = (banks_v == 6'd0) |
                    ((size_i != SIZE_64K) & (bank_i >= {2'b00, banks_v}));

  // The upper window wins when both are selected. They cannot both be selected in
  // one Z80 cycle: cv_addr_dec picks one from a_i[15].
  logic upper_sel_v;
  logic access_v;
  assign upper_sel_v = ~upper_ce_n_i;
  assign access_v    = ~upper_ce_n_i | ~lower_ce_n_i;

  assign a_o = {bank_v,
                upper_sel_v,
                upper_sel_v ? upper_a_i : lower_a_i};

  assign d_o = upper_sel_v ? upper_d_i : lower_d_i;

  // Gated on absent_o so that a write to a bank the card does not have is thrown
  // away instead of landing on some other bank. A sizer that wrote its marker to
  // bank 4 of a 256K card and then found it in bank 0 would size the card wrong
  // just as surely as an aliased read.
  assign we_o = access_v & ~absent_o &
                ~(upper_sel_v ? upper_we_n_i : lower_we_n_i);
  assign rd_o = access_v & ~absent_o &
                ~(upper_sel_v ? upper_rd_n_i : lower_rd_n_i);

endmodule
