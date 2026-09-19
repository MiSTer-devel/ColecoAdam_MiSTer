// Behavioural MT48LC16M16, enough of one to test a controller against.
//
// Simulation only - it is not in files.qip and never reaches Quartus. It models
// what the controller has to get right and nothing else: the command truth table,
// one open row per bank, CAS latency on reads, and the two byte masks. It does
// not model any of the analog timing, so it proves a controller is logically
// correct and says nothing about whether it meets setup and hold at 42 MHz.
//
// It complains on stdout about anything a correct controller would never do, so a
// silent run is part of the result.

module sdram_model
(
   input         clk,          // the chip's own clock, which is ~clk_sys
   input         nCS,
   input         nRAS,
   input         nCAS,
   input         nWE,
   input   [1:0] BA,
   input  [12:0] A,
   inout  [15:0] DQ,

   output [31:0] refresh_count // for the testbench to check refresh really happens
);

localparam CMD_NOP          = 3'b111;
localparam CMD_ACTIVE       = 3'b011;
localparam CMD_READ         = 3'b101;
localparam CMD_WRITE        = 3'b100;
localparam CMD_PRECHARGE    = 3'b010;
localparam CMD_AUTO_REFRESH = 3'b001;
localparam CMD_LOAD_MODE    = 3'b000;

// 4 banks x 8192 rows x 512 columns of 16 bits.
reg [15:0] mem [0:16777215];

reg [12:0] row     [0:3];
reg        row_ok  [0:3];
reg        ready = 1'b0;      // the mode register has been loaded
reg [31:0] refreshes = 0;
reg [2:0]  cas_latency = 3'd2;

assign refresh_count = refreshes;

// Read data with CAS latency 2: the word occupies the clock period that begins at
// the second chip edge after the READ, so it is driven from edge N+2 to edge N+3.
//
// That extra half period matters and is not guesswork. The core's board drives
// SDRAM_CLK from ~clk_sys, so the controller samples half a clock after each chip
// edge - and the single port controller this core shipped with, which works on the
// hardware, reads the bus three controller clocks after issuing READ, which lands
// inside this window and outside a window that ended at edge N+2. Release the bus
// a half period earlier than this and a correct controller reads nothing.
reg [15:0] q1, q2, q3;
reg        oe1, oe2, oe3;
assign DQ = oe3 ? q3 : 16'bZZZZZZZZZZZZZZZZ;

wire [2:0] cmd = {nRAS, nCAS, nWE};

integer i;
initial begin
   for (i = 0; i < 4; i = i + 1) begin
      row[i]    = 13'd0;
      row_ok[i] = 1'b0;
   end
end

always @(posedge clk) begin
   reg [23:0] addr;

   q3  <= q2;
   oe3 <= oe2;
   q2  <= q1;
   oe2 <= oe1;
   q1  <= 16'd0;
   oe1 <= 1'b0;

   if (!nCS) begin
      case (cmd)
         CMD_ACTIVE: begin
            row[BA]    <= A;
            row_ok[BA] <= 1'b1;
         end

         CMD_READ: begin
            if (!ready)      $display("SDRAM MODEL: read before the mode register was loaded");
            if (!row_ok[BA]) $display("SDRAM MODEL: read from bank %0d with no row open", BA);
            addr = {BA, row[BA], A[8:0]};
            q1  <= mem[addr];
            oe1 <= 1'b1;
`ifdef TB_TRACE
            $display("MODEL READ  addr=%06x -> %04x", addr, mem[addr]);
`endif
            // A10 high is read with auto precharge, which closes the row.
            if (A[10]) row_ok[BA] <= 1'b0;
         end

         CMD_WRITE: begin
            if (!ready)      $display("SDRAM MODEL: write before the mode register was loaded");
            if (!row_ok[BA]) $display("SDRAM MODEL: write to bank %0d with no row open", BA);
            addr = {BA, row[BA], A[8:0]};
            // A[11] is DQML and A[12] is DQMH on this board, both active low.
            if (!A[11]) mem[addr][7:0]  <= DQ[7:0];
            if (!A[12]) mem[addr][15:8] <= DQ[15:8];
`ifdef TB_TRACE
            $display("MODEL WRITE addr=%06x dqm=%b%b DQ=%04x", addr, A[12], A[11], DQ);
`endif
            if (A[10]) row_ok[BA] <= 1'b0;
         end

         CMD_PRECHARGE: begin
            if (A[10]) begin
               row_ok[0] <= 1'b0; row_ok[1] <= 1'b0;
               row_ok[2] <= 1'b0; row_ok[3] <= 1'b0;
            end else begin
               row_ok[BA] <= 1'b0;
            end
         end

         CMD_AUTO_REFRESH: begin
            refreshes <= refreshes + 1'd1;
         end

         CMD_LOAD_MODE: begin
            cas_latency <= A[6:4];
            ready       <= 1'b1;
            if (A[6:4] != 3'd2)
               $display("SDRAM MODEL: this model only implements CAS latency 2, got %0d", A[6:4]);
         end

         default: ;
      endcase
   end
end

endmodule
