//-----------------------------------------------------------------------------
//
// FPGA Colecovision
//
// $Id: cv_ctrl.vhd,v 1.3 2006/01/08 23:58:04 arnim Exp $
//
// Controller Interface Module
//
//-----------------------------------------------------------------------------
//
// Copyright (c) 2006, Arnim Laeuger (arnim.laeuger@gmx.net)
//
// All rights reserved
//
// Redistribution and use in source and synthezised forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// Redistributions in synthesized form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// Neither the name of the author nor the names of other contributors may
// be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please report bugs to the author, but before you do so, please
// make sure that this is not a derivative work and that
// you have the latest version of this file.
//
// SystemVerilog conversion (c) 2022 Frank Bruno (fbruno@asicsolutions.com)
//
//-----------------------------------------------------------------------------

module cv_ctrl
  (
   input              clk_i,
   input              clk_en_3m58_i,
   input              reset_n_i,
   input              ctrl_en_key_n_i,
   input              ctrl_en_joy_n_i,
   input              a1_i,
   input [2:1]        ctrl_p1_i,
   input [2:1]        ctrl_p2_i,
   input [2:1]        ctrl_p3_i,
   input [2:1]        ctrl_p4_i,
   output [2:1]       ctrl_p5_o,
   input [2:1]        ctrl_p6_i,
   input [2:1]        ctrl_p7_i,
   output [2:1]       ctrl_p8_o,
   input [2:1]        ctrl_p9_i,
   output logic [7:0] d_o,
   output logic     int_n_o

   );

  logic               sel_q;

  //---------------------------------------------------------------------------
  // Process seq
  //
  // Purpose:
  //   Implements the R/S flip-flop which selects the controller function.
  //

  always @(posedge clk_i, negedge reset_n_i) begin: seq
    if (~reset_n_i) sel_q <= '0;
    else begin
      if (clk_en_3m58_i) begin
        case ({ctrl_en_key_n_i, ctrl_en_joy_n_i})
          2'b01:   sel_q <= '0;
          2'b10:   sel_q <= '1;
          default: sel_q <= sel_q; // make verilator happy
        endcase
      end
    end
  end
  //
  //---------------------------------------------------------------------------

  //---------------------------------------------------------------------------
  // Controller select
  //---------------------------------------------------------------------------
  assign ctrl_p5_o[1] = sel_q;
  assign ctrl_p5_o[2] = sel_q;
  assign ctrl_p8_o[1] = (~sel_q);
  assign ctrl_p8_o[2] = (~sel_q);


  //---------------------------------------------------------------------------
  // Spinner / roller strobe and interrupt
  //
  // The Super Action Controller's speed roller, the Roller Controller trackball
  // and the Expansion Module #2 steering wheel are all the same circuit: an
  // optical encoder whose two phases leave the controller on pins 7 and 9
  // (Expansion Module #2 Technical Guide, Theory of Operation III-1 and
  // schematic V-1: two LEDs, a slotted disc, two photo transistors and Schmitt
  // trigger buffers, 50-50 duty cycle and the phases offset from each other).
  //
  // ADAM Technical Manual, "Controller Connector Pin Out": pin 7 is read back as
  // D5, and pin 9 is an "Indirect /INT input ... Strobe signal: typical 350 usec
  // pulse width". Section 2.2.7 adds that the spinner switches are "connected to
  // the CPU maskable interrupt, and the cartridge software determines which
  // switch caused the interrupt" - it does that by reading D4, which is low for
  // the length of that controller's strobe.
  //
  // OS-7's spinner handler (BIOS 116Ah) is exactly that: read the port, skip the
  // controller when D4 is high, then count up when D5 is set and down when it is
  // clear.
  //
  // This replaces the NAND/rctimer one-shot in cv_ctrl.vhd and is observably the
  // same: D4 low and D7 high for one strobe window, /INT pulsed at the start of
  // it. The windows are counted in 3.58 MHz clock enables rather than raw clk_i
  // cycles, so they last the same time in simulation (clk_i = 10.7 MHz) as on
  // hardware (clk_i = 42.666 MHz) instead of being 4x apart, and the lengths are
  // the documented ones rather than whatever the raw counter happened to give.
  //
  // /INT is pulsed, not held for the whole strobe: the handler re-enables
  // interrupts with EI before it returns, so a level that outlasted the handler
  // would be taken a second time and count one step of the roller twice. 11 us
  // is what MAME settled on (coleco.cpp, paddle_pulse_callback) and is longer
  // than the slowest Z80 instruction, so it cannot be missed either.
  //---------------------------------------------------------------------------

  localparam logic [10:0] STROBE_TICKS = 11'd1253; // 350 us of clk_en_3m58_i
  localparam logic  [5:0] INT_TICKS    =  6'd40;   // ~11 us

  logic [10:0] strobe_cnt [2:1];
  logic  [5:0] int_cnt    [2:1];
  logic  [2:1] p9_q;
  logic  [2:1] moving_s;

  always_ff @(posedge clk_i, negedge reset_n_i) begin : spinner
    if (~reset_n_i) begin
      for (int i = 1; i <= 2; i++) begin
        p9_q[i]       <= 1'b1;
        strobe_cnt[i] <= '0;
        int_cnt[i]    <= '0;
      end
    end else if (clk_en_3m58_i) begin
      for (int i = 1; i <= 2; i++) begin
        p9_q[i] <= ctrl_p9_i[i];

        if (p9_q[i] & ~ctrl_p9_i[i]) begin
          // falling edge on pin 9: one step of the roller
          strobe_cnt[i] <= STROBE_TICKS;
          int_cnt[i]    <= INT_TICKS;
        end else begin
          if (strobe_cnt[i] != '0) strobe_cnt[i] <= strobe_cnt[i] - 1'b1;
          if (int_cnt[i]    != '0) int_cnt[i]    <= int_cnt[i]    - 1'b1;
        end
      end
    end
  end

  assign moving_s[1] = (strobe_cnt[1] != '0);
  assign moving_s[2] = (strobe_cnt[2] != '0);

  // Both controllers' strobes reach the same maskable interrupt.
  assign int_n_o = ~((int_cnt[1] != '0) | (int_cnt[2] != '0));

  //---------------------------------------------------------------------------
  // Process ctrl_read
  //
  // Purpose:
  //   Read multiplexer for the controller lines.
  //
  always_comb begin : ctrl_read
    logic [1:0]        idx_v;
    if (~a1_i) idx_v = 1; // read controller #1
    else       idx_v = 2; // read controller #2

    d_o = {moving_s[idx_v],      // D7, complement of D4, as in cv_ctrl.vhd
           ctrl_p6_i[idx_v],
           ctrl_p7_i[idx_v],     // D5, direction of travel
           ~moving_s[idx_v],     // D4, low while this controller is strobing
           ctrl_p3_i[idx_v],
           ctrl_p2_i[idx_v],
           ctrl_p4_i[idx_v],
           ctrl_p1_i[idx_v]};
  end

endmodule
