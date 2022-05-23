//-----------------------------------------------------------------------------
//
// FPGA Colecovision
//
// $Id: cv_addr_dec.vhd,v 1.3 2006/01/05 22:22:29 arnim Exp $
//
// Address Decoder
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

module cv_addr_dec
  (
   input              clk_i,
   input              reset_n_i,
   input              adam,
	input              mode,
   input [15:0]       a_i,
   input [7:0]        d_i,
   input [5:0]        cart_pages_i,
   output logic [5:0] cart_page_o,
   input              iorq_n_i,
   input              rd_n_i,
   input              wr_n_i,
   input              mreq_n_i,
   input              rfsh_n_i,
   output logic       bios_rom_ce_n_o,
   output logic       eos_rom_ce_n_o,
   output logic       writer_rom_ce_n_o,
   output logic       ram_ce_n_o,
   output logic       lowerexpansion_ram_ce_n_o,
   output logic       upper_ram_ce_n_o,
   output logic       expansion_ram_ce_n_o,
   output logic       expansion_rom_ce_n_o,
	output logic       cartridge_rom_ce_n_o,
	output logic       vdp_r_n_o,
   output logic       vdp_w_n_o,
   output logic       psg_we_n_o,

   output logic       adam_reset_pcb_n_o,
   output logic       ctrl_r_n_o,
   output logic       ctrl_en_key_n_o,
   output logic       ctrl_en_joy_n_o
   );


  logic               bios_en;
  logic               eos_en;
  logic               last_35_reset_bit;
  logic [1:0]         lower_mem_adam;
  logic [1:0]         upper_mem_adam;
  logic [1:0]         lower_mem_nadam;
  logic [1:0]         upper_mem_nadam;
  logic [1:0]         lower_mem;
  logic [1:0]         upper_mem;
  //---------------------------------------------------------------------------
  // Process dec
  //
  // Purpose:
  //   Implements the address decoding logic.
  //
  always_comb begin : dec
    // default assignments
    bios_rom_ce_n_o    = '1;
    eos_rom_ce_n_o     = '1;
    writer_rom_ce_n_o  = '1;
    eos_rom_ce_n_o     = '1;
    ram_ce_n_o         = '1;
    lowerexpansion_ram_ce_n_o         = '1;
    upper_ram_ce_n_o   = '1;
    expansion_ram_ce_n_o='1;
    expansion_rom_ce_n_o='1;
    cartridge_rom_ce_n_o='1;
    vdp_r_n_o          = '1;
    vdp_w_n_o          = '1;
    psg_we_n_o         = '1;
    adam_reset_pcb_n_o = '1;
    ctrl_r_n_o         = '1;
    ctrl_en_key_n_o    = '1;
    ctrl_en_joy_n_o    = '1;



 
    // Memory access ----------------------------------------------------------
    if (~mreq_n_i && rfsh_n_i) begin

//      end else begin
        if (~a_i[15])
        begin
        if (lower_mem == 2'b11) begin  // OS7 / 24k RAM
          case (a_i[15:13])
          3'b000: bios_rom_ce_n_o = '0;
          3'b001,
			 3'b010,
			 3'b011: ram_ce_n_o     = '0;	// 2000 - 7fff = 24k
            default: begin
            end
          endcase
			 
        end
        else if (lower_mem == 2'b10) begin // RAM expansion
          lowerexpansion_ram_ce_n_o     = '0;
        end
        else if (lower_mem == 2'b01) begin // 32K of RAM
          ram_ce_n_o     = '0;	// 2000 - 7fff = 24k
        end
        else if (lower_mem == 2'b00) begin // WRITER ROM (when do we use EOS?)
          if (eos_en)
             eos_rom_ce_n_o ='0;
          else
             writer_rom_ce_n_o ='0;
        end
        end
		  
        if (a_i[15])
        begin

        if (upper_mem == 2'b10) begin // RAM expansion
            expansion_ram_ce_n_o='0;
        end
        else if (upper_mem == 2'b01) begin // ROM expansion
            expansion_rom_ce_n_o='0;
        end
        else if (upper_mem == 2'b00) begin // 32k RAM
              upper_ram_ce_n_o ='0;
        end
		  else if (upper_mem == 2'b11) begin // ROM Cartridge (expansion rom at the moment)
            cartridge_rom_ce_n_o='0;
        end
        end
    end

    // I/O access -------------------------------------------------------------
    if (~iorq_n_i) begin
      if (a_i[7]) begin
        case ({a_i[6], a_i[5], wr_n_i})
          3'b000: ctrl_en_key_n_o = '0;
          3'b010: vdp_w_n_o = '0;
          3'b011: if (~rd_n_i) vdp_r_n_o = '0;
          3'b100: ctrl_en_joy_n_o = '0;
          3'b110: psg_we_n_o = '0;
          3'b111: if (~rd_n_i) ctrl_r_n_o = '0;
            default: begin
            end
        endcase
      end

      else if (a_i[7:0] == 8'h3f && ~wr_n_i && d_i==8'h0F) adam_reset_pcb_n_o = '0;
    end
  end

  //
  //---------------------------------------------------------------------------
  always @(negedge reset_n_i, posedge clk_i) begin : m_adam
    if (~reset_n_i) begin
      lower_mem_adam     <= mode ? 2'b11 : 2'b00;  // computer mode
      upper_mem_adam     <= mode ? 2'b11 : 2'b00;
    end else begin
      //if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:0] == 8'h7f))
		if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:5] == 3'b011))

      begin
                $display("D CHANGING MEM 7F lower %x upper %x",d_i[1:0],d_i[3:2]);
              lower_mem_adam <= d_i[1:0];
              upper_mem_adam <= d_i[3:2];
      end
    end
  end

//  always @(negedge reset_n_i, posedge clk_i) begin : m_nadam
//    if (~reset_n_i) begin
//      lower_mem_nadam     <= 2'b11;  // console mode
//      upper_mem_nadam     <= 2'b11;
//    end else begin
//      if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:0] == 8'h7f))
//      begin
//                $display("C CHANGING MEM 7F lower %x upper %x",d_i[1:0],d_i[3:2]);
//              lower_mem_nadam <= d_i[1:0];
//              upper_mem_nadam <= d_i[3:2];
//      end
//    end
//  end

  assign lower_mem =  lower_mem_adam ;
  assign upper_mem =   upper_mem_adam;

  always @(negedge reset_n_i, posedge clk_i) begin : megacart
    if (~reset_n_i) begin
      bios_en       <= '1;
      last_35_reset_bit <= '0;
      eos_en       <= '0;
    end else begin
     if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:0] == 8'h7f))
      begin
        bios_en <= d_i[1];
              $display("A SETTING MEMORY MODE 7F? %x",a_i,d_i);
      end

        // just 3F or all addresses?
      if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:0] == 8'h3f))
      begin
              $display("B SETTING MEMORY MODE 3F? addr %x data %x",a_i,d_i);
        last_35_reset_bit <= d_i[0];
        eos_en <= d_i[1];
      end
    end
  end

endmodule
