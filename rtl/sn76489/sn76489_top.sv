//////////////////////////////////////////////////////////////////////////////-
//
// Synthesizable model of TI's SN76489AN.
//
// $Id: sn76489_top.vhd,v 1.9 2006/02/27 20:30:10 arnim Exp $
//
// Chip Toplevel
//
// References:
//
//   * TI Data sheet SN76489.pdf
//     ftp://ftp.whtech.com/datasheets%20&%20manuals/SN76489.pdf
//
//   * John Kortink's article on the SN76489:
//     http://web.inter.nl.net/users/J.Kortink/home/articles/sn76489/
//
//   * Maxim's "SN76489 notes" in
//     http://www.smspower.org/maxim/docs/SN76489.txt
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

module sn76489_top
  #
  (
   parameter clock_div_16_g = 1
   )
  (
   input        clock_i,
   input        clock_en_i,
   input        res_n_i,
   input        ce_n_i,
   input        we_n_i,
   output       ready_o,
   input [0:7]  d_i,
   output [0:9] aout_o
   );

  logic         clk_en_s;

  logic         tone1_we_s,
                tone2_we_s,
                tone3_we_s,
                noise_we_s;
  logic         r2_s;

  logic [0:7]   tone1_s,
                tone2_s,
                tone3_s,
                noise_s;

  logic         tone3_ff_s;

  ////////////////////////////////////////////////////////////////////////////-
  // Clock Divider
  ////////////////////////////////////////////////////////////////////////////-
  sn76489_clock_div
    #
    (
     .clock_div_16_g (clock_div_16_g)
     )
  clock_div_b
    (
     .clock_i        (clock_i),
     .clock_en_i     (clock_en_i),
     .res_n_i        (res_n_i),
     .clk_en_o       (clk_en_s)
     );

  ////////////////////////////////////////////////////////////////////////////-
  // Latch Control = CPU Interface
  ////////////////////////////////////////////////////////////////////////////-
  sn76489_latch_ctrl latch_ctrl_b
    (
     .clock_i    (clock_i),
     .clk_en_i   (clk_en_s),
     .res_n_i    (res_n_i),
     .ce_n_i     (ce_n_i),
     .we_n_i     (we_n_i),
     .d_i        (d_i),
     .ready_o    (ready_o),
     .tone1_we_o (tone1_we_s),
     .tone2_we_o (tone2_we_s),
     .tone3_we_o (tone3_we_s),
     .noise_we_o (noise_we_s),
     .r2_o       (r2_s)
     );


  ////////////////////////////////////////////////////////////////////////////-
  // Tone Channel 1
  ////////////////////////////////////////////////////////////////////////////-
  sn76489_tone tone1_b
    (
     .clock_i  (clock_i),
     .clk_en_i (clk_en_s),
     .res_n_i  (res_n_i),
     .we_i     (tone1_we_s),
     .d_i      (d_i),
     .r2_i     (r2_s),
     .ff_o     (),
     .tone_o   (tone1_s)
     );

  ////////////////////////////////////////////////////////////////////////////-
  // Tone Channel 2
  ////////////////////////////////////////////////////////////////////////////-
  sn76489_tone tone2_b
    (
     .clock_i  (clock_i),
     .clk_en_i (clk_en_s),
     .res_n_i  (res_n_i),
     .we_i     (tone2_we_s),
     .d_i      (d_i),
     .r2_i     (r2_s),
     .ff_o     (),
     .tone_o   (tone2_s)
     );

  ////////////////////////////////////////////////////////////////////////////-
  // Tone Channel 3
  ////////////////////////////////////////////////////////////////////////////-
  sn76489_tone tone3_b
    (
     .clock_i  (clock_i),
     .clk_en_i (clk_en_s),
     .res_n_i  (res_n_i),
     .we_i     (tone3_we_s),
     .d_i      (d_i),
     .r2_i     (r2_s),
     .ff_o     (tone3_ff_s),
     .tone_o   (tone3_s)
     );

  ////////////////////////////////////////////////////////////////////////////-
  // Noise Channel
  ////////////////////////////////////////////////////////////////////////////-
  sn76489_noise noise_b
    (
     .clock_i    (clock_i),
     .clk_en_i   (clk_en_s),
     .res_n_i    (res_n_i),
     .we_i       (noise_we_s),
     .d_i        (d_i),
     .r2_i       (r2_s),
     .tone3_ff_i (tone3_ff_s),
     .noise_o    (noise_s)
     );


  assign aout_o = tone1_s + tone2_s + tone3_s + noise_s;

endmodule // sn76489_top
