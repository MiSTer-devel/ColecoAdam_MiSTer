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


   assign int_n_o = 1;
  //---------------------------------------------------------------------------
  // Process ctrl_read
  //
  // Purpose:
  //   Read multiplexer for the controller lines.
  //   NOTE: The quadrature decoders are not implemented!
  //
  always_comb begin : ctrl_read
    logic [1:0]        idx_v;
    if (~a1_i) idx_v = 1; // read controller #1
    else       idx_v = 2; // read controller #2

    // quadrature information
    d_o = {1'b0, ctrl_p6_i[idx_v], ctrl_p7_i[idx_v], 1'b1, ctrl_p3_i[idx_v],
           ctrl_p2_i[idx_v], ctrl_p4_i[idx_v], ctrl_p1_i[idx_v]};
  end

endmodule
