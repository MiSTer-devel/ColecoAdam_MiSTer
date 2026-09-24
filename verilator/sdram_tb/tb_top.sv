// Controller plus memory model, wired the way ColecoAdam.sv wires them.
//
// The one detail that has to match the board is the clock: the top level drives
// SDRAM_CLK from ~clk_sys, so the part sees a rising edge half a cycle before the
// controller does. Getting that wrong here would let a broken controller pass.

module tb_top
(
   input         clk,
   input         init,

   input  [24:0] ch0_addr,
   input         ch0_rd,
   input         ch0_wr,
   input   [7:0] ch0_din,
   output  [7:0] ch0_dout,
   output        ch0_ready,

   input  [24:0] ch1_addr,
   input         ch1_rd,
   input         ch1_wr,
   input   [7:0] ch1_din,
   output  [7:0] ch1_dout,
   output        ch1_ready,

   output [31:0] refresh_count
);

wire [15:0] dq;
wire [12:0] a;
wire  [1:0] ba;
wire        nCS, nWE, nRAS, nCAS, CKE, dqml, dqmh;

sdram dut
(
   .init(init),
   .clk(clk),

   .SDRAM_DQ(dq),
   .SDRAM_A(a),
   .SDRAM_DQML(dqml),
   .SDRAM_DQMH(dqmh),
   .SDRAM_BA(ba),
   .SDRAM_nCS(nCS),
   .SDRAM_nWE(nWE),
   .SDRAM_nRAS(nRAS),
   .SDRAM_nCAS(nCAS),
   .SDRAM_CKE(CKE),

   .ch0_addr(ch0_addr), .ch0_rd(ch0_rd), .ch0_wr(ch0_wr),
   .ch0_din(ch0_din), .ch0_dout(ch0_dout), .ch0_ready(ch0_ready),

   .ch1_addr(ch1_addr), .ch1_rd(ch1_rd), .ch1_wr(ch1_wr),
   .ch1_din(ch1_din), .ch1_dout(ch1_dout), .ch1_ready(ch1_ready)
);

sdram_model chip
(
   .clk(~clk),          // SDRAM_CLK = ~clk_sys, exactly as the top level drives it
   .nCS(nCS),
   .nRAS(nRAS),
   .nCAS(nCAS),
   .nWE(nWE),
   .BA(ba),
   .A(a),
   .DQ(dq),
   .refresh_count(refresh_count)
);

endmodule
