//-----------------------------------------------------------------------------
//
// FPGA Colecovision
//
// $Id: cv_clock.vhd,v 1.3 2006/01/05 22:22:29 arnim Exp $
//
// Clock Generator
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

module cv_clock
  (
   input  clk_i,
   input  clk_en_10m7_i,
   input  reset_n_i,
   output clk_en_3m58_p_o,
   output clk_en_3m58_n_o
   );

  logic [1:0] clk_cnt_q;

  //---------------------------------------------------------------------------
  // Process clk_cnt
  //
  // Purpose:
  //   Implements the counter which is used to generate the clock enable
  //   for the 3.58 MHz clock.
  //

  always @(posedge clk_i, negedge reset_n_i) begin : clk_cnt
    if (~reset_n_i) begin
      clk_cnt_q <= '0;
    end else begin
      if (clk_en_10m7_i) begin
        if (clk_cnt_q == 0) clk_cnt_q <= 2'b10;
        else                clk_cnt_q <= clk_cnt_q - 1'b1;
      end
    end
  end : clk_cnt

  //
  //---------------------------------------------------------------------------
  assign clk_en_3m58_p_o = (clk_cnt_q == 0    ) ? clk_en_10m7_i : '0;
  assign clk_en_3m58_n_o = (clk_cnt_q == 2'b10) ? clk_en_10m7_i : '0;

endmodule
