//-----------------------------------------------------------------------------
//
// Synthesizable model of TI's SN76489AN.
//
// $Id: sn76489_clock_div.vhd,v 1.4 2005/10/10 21:51:27 arnim Exp $
//
// Clock Divider Circuit
//
//-----------------------------------------------------------------------------
//
// Copyright (c) 2005, Arnim Laeuger (arnim.laeuger@gmx.net)
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

module sn76489_clock_div
  #
  (
   parameter clock_div_16_g = 1
   )
  (
   input        clock_i,
   input        clock_en_i,
   input        res_n_i,
   output logic clk_en_o
   );

  logic [3:0] cnt_s, cnt_q;

  //---------------------------------------------------------------------------
  // Process seq
  //
  // Purpose:
  //   Implements the sequential counter element.
  //
  always_ff @(posedge clock_i, negedge res_n_i) begin : seq
    if (~res_n_i) cnt_q <= '0;
    else          cnt_q <= cnt_s;
  end : seq
  //
  //---------------------------------------------------------------------------


  //---------------------------------------------------------------------------
  // Process comb
  //
  // Purpose:
  //   Implements the combinational counter logic.
  //
  always_comb begin : comb
    // default assignments
    cnt_s    = cnt_q;
    clk_en_o = '0;

    if (clock_en_i) begin
      if (cnt_q == 0) begin
        clk_en_o = '1;

        if (clock_div_16_g)       cnt_s = 15;
        else if (~clock_div_16_g) cnt_s = 1;
        else begin
          // Need to fix for SV
          // pragma translate_off
          //assert false
          //  report "Generic clock_div_16_g must be either 0 or 1."
          //  severity failure;
          // pragma translate_on
        end
      end else
        cnt_s    = cnt_q - 1'b1;
    end // if (clock_en_i)
  end : comb
endmodule
