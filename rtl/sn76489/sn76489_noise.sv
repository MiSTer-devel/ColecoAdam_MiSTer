//////////////////////////////////////////////////////////////////////////////-
//
// Synthesizable model of TI's SN76489AN.
//
// $Id: sn76489_noise.vhd,v 1.6 2006/02/27 20:30:10 arnim Exp $
//
// Noise Generator
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

module sn76489_noise
  (
   input        clock_i,
   input        clk_en_i,
   input        res_n_i,
   input        we_i,
   input [0:7]  d_i,
   input        r2_i,
   input        tone3_ff_i,
   output [0:7] noise_o
   );

  logic [0:1]   nf_q;
  logic         fb_q;
  logic [0:3]   a_q;
  logic [0:6]   freq_cnt_q;
  logic         freq_ff_q;

  logic         shift_source_s, shift_source_q;
  logic         shift_rise_edge_s;

  logic [0:15]  lfsr_q;

  ////////////////////////////////////////////////////////////////////////////-
  // Process cpu_regs
  //
  // Purpose:
  //   Implements the registers writable by the CPU.
  //
  always_ff @(posedge clock_i, negedge res_n_i) begin : cpu_regs
    if (~res_n_i) begin
      nf_q <= '0;
      fb_q <= '0;
      a_q  <= '1;
    end else begin
      if (clk_en_i && we_i) begin
        if (~r2_i) begin
          // access to control register
          // both access types can write to the control register!
          nf_q <= d_i[6:7];
          fb_q <= d_i[5];
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
          // reload frequency counter according to NF setting
          case (nf_q)
            2'b00: freq_cnt_q <= 16 * 2 - 1;
            2'b01: freq_cnt_q <= 16 * 4 - 1;
            2'b10: freq_cnt_q <= 16 * 8 - 1;
            default: freq_cnt_q <= freq_cnt_q;
          endcase
          freq_ff_q <= ~freq_ff_q;
        end else begin
          // decrement frequency counter
          freq_cnt_q <= freq_cnt_q - 1'b1;
        end
      end
    end
  end : freq_gen
  //
  ////////////////////////////////////////////////////////////////////////////-


  ////////////////////////////////////////////////////////////////////////////-
  // Multiplex the source of the LFSR's shift enable
  ////////////////////////////////////////////////////////////////////////////-
  assign shift_source_s = &nf_q ? tone3_ff_i : freq_ff_q;

  ////////////////////////////////////////////////////////////////////////////-
  // Process rise_edge
  //
  // Purpose:
  //   Detect the rising edge of the selected LFSR shift source.
  //
  always_ff @(posedge clock_i, negedge res_n_i) begin : rise_edge
    if (~res_n_i) begin
      shift_source_q <= '0;
    end else begin
      if (clk_en_i) begin
        shift_source_q <= shift_source_s;
      end
    end
  end : rise_edge
  //
  ////////////////////////////////////////////////////////////////////////////-

  // detect rising edge on shift source
  assign shift_rise_edge_s = ~shift_source_q & shift_source_s;

  ////////////////////////////////////////////////////////////////////////////-
  // Process lfsr
  //
  // Purpose:
  //   Implements the LFSR that generates noise.
  //   Note: This implementation shifts the register right, i.e. from index
  //         15 towards 0 => bit 15 is the input, bit 0 is the output
  //
  //   Tapped bits according to MAME's sn76496.c, implemented in function
  //   lfsr_tapped_f.
  //
  always_ff @(posedge clock_i, negedge res_n_i) begin : lfsr
    if (~res_n_i) begin
      // reset LFSR to "0000000000000001"
      lfsr_q               <= 16'b1;
    end else begin
      if (clk_en_i) begin
        if (we_i && ~r2_i) begin
          // write to noise register
          // -> reset LFSR
          lfsr_q               <= 16'b1;

        end else if (shift_rise_edge_s) begin

          // shift LFSR left towards MSB
          lfsr_q <= lfsr_q << 1;

          // determine input bit
          if (~fb_q) begin
            // "Periodic" Noise
            // -> input to LFSR is output
            lfsr_q[15] <= lfsr_q[0];
          end else begin
            // "White" Noise
            // -> input to LFSR is parity of tapped bits
            lfsr_q[15] <= lfsr_tapped_f(lfsr_q);
          end
        end
      end
    end // else: !if(~res_n_i)
  end : lfsr

  ////////////////////////////////////////////////////////////////////////////-
  // The attenuator itself
  ////////////////////////////////////////////////////////////////////////////-
  sn76489_attenuator attenuator_b
    (
     .attenuation_i (a_q),
     .factor_i      (lfsr_q[0]),
     .product_o     (noise_o)
     );

  function logic lfsr_tapped_f;
    input logic [0:15] lfsr;
    logic [0:15]       tapped_bits_c;
    logic              parity_v;
    begin
      tapped_bits_c = 16'b1010000000000001; // tapped bits are 0, 2, 15
      parity_v = '0;

      for (int idx = 0; idx < 16; idx++) begin
        parity_v = parity_v ^ (lfsr[idx] & tapped_bits_c[idx]);
      end

      return parity_v;
    end
  endfunction // lfsr

endmodule // sn76489_noise
