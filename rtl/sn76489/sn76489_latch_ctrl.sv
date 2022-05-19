//////////////////////////////////////////////////////////////////////////////-
//
// Synthesizable model of TI's SN76489AN.
//
// $Id: sn76489_latch_ctrl.vhd,v 1.6 2006/02/27 20:30:10 arnim Exp $
//
// Latch Control Unit
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

module sn76489_latch_ctrl
  (
   input        clock_i,
   input        clk_en_i,
   input        res_n_i,
   input        ce_n_i,
   input        we_n_i,
   input [0:7]  d_i,
   output logic ready_o,
   output logic tone1_we_o,
   output logic tone2_we_o,
   output logic tone3_we_o,
   output logic noise_we_o,
   output logic r2_o
   );

  logic [0:2]   reg_q;
  logic         we_q;
  logic         ready_q;

  ////////////////////////////////////////////////////////////////////////////-
  // Process seq
  //
  // Purpose:
  //   Implements the sequential elements.
  //
  always_ff @(posedge clock_i, negedge res_n_i) begin : seq
    if (~res_n_i) begin
      reg_q     <= '0;
      we_q      <= '0;
      ready_q   <= '0;
    end else begin
      // READY Flag Output ////////////////////////////////////////////////////
      if (~ready_q && we_q) begin
        if (clk_en_i) begin
          // assert READY when write access happened
          ready_q <= '1;
        end
      end else if (ce_n_i) begin
        // deassert READY when access has finished
        ready_q <= '0;
      end

      // Register Selection //////////////////////////////////////////////////-
      if (~ce_n_i && ~we_n_i) begin
        if (clk_en_i) begin
          if (d_i[0]) begin
            reg_q <= d_i[1:3];
          end
          we_q <= '1;
        end
      end else begin
        we_q  <= '0;
      end
    end // else: !if(~res_n_i)
  end : seq
  //
  ////////////////////////////////////////////////////////////////////////////-


  ////////////////////////////////////////////////////////////////////////////-
  // Output mapping
  ////////////////////////////////////////////////////////////////////////////-
  assign tone1_we_o = (reg_q[0:1] == 2'b00) & we_q;
  assign tone2_we_o = (reg_q[0:1] == 2'b01) & we_q;
  assign tone3_we_o = (reg_q[0:1] == 2'b10) & we_q;
  assign noise_we_o = (reg_q[0:1] == 2'b11) & we_q;

  assign r2_o       = reg_q[2];

  assign ready_o    = ~ce_n_i ? ready_q : '1;
endmodule // sn76489_latch_ctrl
