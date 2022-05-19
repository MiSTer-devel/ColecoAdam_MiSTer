//////////////////////////////////////////////////////////////////////////////-
//
// Synthesizable model of TI's SN76489AN.
//
// $Id: sn76489_tone.vhd,v 1.5 2006/02/27 20:30:10 arnim Exp $
//
// Tone Generator
//
//////////////////////////////////////////////////////////////////////////////-
//
// Copyright (c) 2005, 2006, Arnim Laeuger (arnim.laeuger@gmx.net)
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
//////////////////////////////////////////////////////////////////////////////-

module sn76489_tone
  (
   input        clock_i,
   input        clk_en_i,
   input        res_n_i,
   input        we_i,
   input [0:7]  d_i,
   input        r2_i,
   output       ff_o,
   output [0:7] tone_o
   );

  logic [0:9]   f_q;
  logic [0:3]   a_q;
  logic [0:9]   freq_cnt_q;
  logic         freq_ff_q;

  ////////////////////////////////////////////////////////////////////////////-
  // Process cpu_regs
  //
  // Purpose:
  //   Implements the registers writable by the CPU.
  //
  always_ff @(posedge clock_i, negedge res_n_i) begin : cpu_regs
    if (~res_n_i) begin
      f_q <= '0;
      a_q <= '1;
    end else begin
      if (clk_en_i && we_i) begin
        if (~r2_i) begin
          // access to frequency register
          if (~d_i[0]) f_q[0:5] <= d_i[2:7];
          else         f_q[6:9] <= d_i[4:7];
        end else begin
          // access to attenuator register
          // both access types can write to the attenuator register!
          a_q <= d_i[4:7];
        end
      end
    end
  end : cpu_regs
  //
  ////////////////////////////////////////////////////////////////////////////-


  ////////////////////////////////////////////////////////////////////////////-
  // Process freq_gen
  //
  // Purpose:
  //   Implements the frequency generation components.
  //
  always_ff @(posedge clock_i, negedge res_n_i) begin : freq_gen
    if (~res_n_i) begin
      freq_cnt_q <= '0;
      freq_ff_q  <= '0;
    end else begin
      if (clk_en_i) begin
        if (freq_cnt_q == 0) begin
          // update counter from frequency register
          freq_cnt_q <= f_q;

          // and toggle the frequency flip-flop if enabled
          if (|f_q)
            freq_ff_q <= ~freq_ff_q;
          else
            // if frequency setting is 0, then keep flip-flop at +1
            freq_ff_q <= '1;
        end else begin
          // decrement frequency counter
          freq_cnt_q <= freq_cnt_q - 1'b1;
        end
      end
    end
  end : freq_gen

  ////////////////////////////////////////////////////////////////////////////-
  // The attenuator itself
  ////////////////////////////////////////////////////////////////////////////-
  sn76489_attenuator attenuator_b
    (
     .attenuation_i (a_q),
     .factor_i      (freq_ff_q),
     .product_o     (tone_o)
     );


  ////////////////////////////////////////////////////////////////////////////-
  // Output mapping
  ////////////////////////////////////////////////////////////////////////////-
  assign ff_o = freq_ff_q;

endmodule // sn76489_tone
