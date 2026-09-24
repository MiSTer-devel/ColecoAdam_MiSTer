//
// sdram.sv
//
// Two channel, byte wide SDRAM controller for the MT48LC16M16 on a MiSTer.
//
// Copyright (c) 2015-2019 Sorgelig
// Arbitration and the per channel word cache follow NES_MiSTer's three channel
// version of this controller; the bus timing is unchanged from both.
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//-----------------------------------------------------------------------------
//
// Why two channels
//
// The single port version of this controller served the cartridge alone. The
// ADAM memory expander cannot go in block RAM - a 512K card needs 410 M10K
// blocks and the core has 87 free - so it has to share the SDRAM, and a second
// master needs arbitration the single port version has none of.
//
// The two channels are:
//
//   ch0  the cartridge: Z80 reads, and the ioctl writes that load it
//   ch1  the ADAM memory expander
//
// ch0 has priority. The Z80 only ever accesses one of them at a time, so the
// two rarely collide; what the separate channels really buy is a cache each,
// which is the next paragraph.
//
// The word cache, and why the CPU usually does not wait
//
// The chip is 16 bits wide and the Z80 reads bytes, so half of all sequential
// reads want the byte next to one already fetched. Each channel keeps the last
// word it read, and a read that hits it is answered from the register with no
// bus cycle at all and without `ready` ever dropping - so the CPU is not
// stalled, exactly as in the single port version. That fast path is why moving
// the expander in here does not slow the cartridge down: a run through cartridge
// code still costs a bus cycle only every other byte, and the expander has its
// own cache so the two do not evict each other.
//
// A miss costs one slot of seven clocks. At clk_sys = 42.666 MHz that is 164 ns,
// inside one 3.58 MHz Z80 clock of 279 ns, so even a miss usually resolves
// before the CPU samples WAIT. Refresh takes a slot of its own and is what
// actually stalls the CPU now and then.
//
// The two channels must not be given overlapping address ranges: a write on one
// does not invalidate the other's cached word.
//
// Timing at 42.666 MHz, one clock being 23.4 ns, for a -75 part:
//   tRCD  20 ns  ACTIVE to READ/WRITE is 1 clock, 23.4 ns
//   tRC   66 ns  one slot is 7 clocks, 164 ns
//   tRAS  44 ns  covered by the same slot
//   tRFC  66 ns  refresh gets a whole slot
//   CAS latency 2, which the part allows below 100 MHz
//
//-----------------------------------------------------------------------------

module sdram
(
   input             init,        // hold in reset until the PLL has locked
   input             clk,         // clk_sys, 42.666 MHz

   inout      [15:0] SDRAM_DQ,    // 16 bit bidirectional data bus
   output reg [12:0] SDRAM_A,     // 13 bit multiplexed address bus
   output            SDRAM_DQML,  // two byte masks, driven from the address bus
   output            SDRAM_DQMH,  //
   output reg  [1:0] SDRAM_BA,    // two banks
   output            SDRAM_nCS,   // a single chip select
   output            SDRAM_nWE,   // write enable
   output            SDRAM_nRAS,  // row address select
   output            SDRAM_nCAS,  // column address select
   output            SDRAM_CKE,   // clock enable

   // ch0: the cartridge, and the ioctl download that fills it
   input      [24:0] ch0_addr,
   input             ch0_rd,
   input             ch0_wr,
   input       [7:0] ch0_din,
   output      [7:0] ch0_dout,
   output            ch0_ready,

   // ch1: the ADAM memory expander
   input      [24:0] ch1_addr,
   input             ch1_rd,
   input             ch1_wr,
   input       [7:0] ch1_din,
   output      [7:0] ch1_dout,
   output            ch1_ready
);

assign SDRAM_nCS = 1'b0;
assign SDRAM_CKE = 1'b1;
assign {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} = cmd;
assign {SDRAM_DQMH, SDRAM_DQML} = SDRAM_A[12:11];

// Driven only for the one cycle a WRITE command is on the bus. Written as an
// output enable rather than a registered 'z because a non blocking assignment of
// high impedance is something Verilator cannot elaborate, and this way the
// controller can still be linted.
reg [15:0] dq_out;
reg        dq_oe;
assign SDRAM_DQ = dq_oe ? dq_out : 16'bZZZZZZZZZZZZZZZZ;

localparam BURST_LENGTH   = 3'd0; // 0=1, 1=2, 2=4, 3=8, 7=full page
localparam ACCESS_TYPE    = 1'd0; // 0=sequential, 1=interleaved
localparam CAS_LATENCY    = 3'd2; // 2 below 100MHz, 3 above
localparam OP_MODE        = 2'd0; // only 0 (standard operation) allowed
localparam NO_WRITE_BURST = 1'd1; // 0=write burst enabled, 1=only single access write
localparam MODE = {3'b000, NO_WRITE_BURST, OP_MODE, CAS_LATENCY, ACCESS_TYPE, BURST_LENGTH};

localparam RASCAS_DELAY = 3'd1;
localparam STATE_IDLE   = 3'd0;
localparam STATE_START  = STATE_IDLE + 3'd1;              // 1: ACTIVE
localparam STATE_CONT   = STATE_START + RASCAS_DELAY;     // 2: READ or WRITE
localparam STATE_READY  = STATE_CONT + CAS_LATENCY + 3'd2;// 6: data has arrived
localparam STATE_LAST   = STATE_READY;

localparam CMD_NOP          = 3'b111;
localparam CMD_ACTIVE       = 3'b011;
localparam CMD_READ         = 3'b101;
localparam CMD_WRITE        = 3'b100;
localparam CMD_PRECHARGE    = 3'b010;
localparam CMD_AUTO_REFRESH = 3'b001;
localparam CMD_LOAD_MODE    = 3'b000;

// 8192 rows every 64 ms is one refresh every 333 clocks at 42.666 MHz.
localparam REFRESH_PERIOD = 11'd333;

reg  [2:0] cmd = CMD_NOP;
reg  [2:0] state = STATE_IDLE;

// The access being serviced.
reg [22:0] a;
reg  [1:0] bank;
reg  [7:0] wdata;
reg        we;
reg        ram_req;     // 1 = a real bus cycle, 0 = the slot is a refresh
reg        serving;     // which channel the slot belongs to
reg  [1:0] busy;        // that channel has a slot in flight

// Captured requests. A strobe is latched the moment it rises rather than when
// the controller happens to be free, because hps_io's ioctl_wr is a single cycle
// pulse and would otherwise be lost whenever the other channel or a refresh has
// the bus.
//
// Reads and writes are held apart so that a channel can have one of each
// outstanding. A Z80 writing a byte and reading it straight back does exactly
// that if a refresh delays the write, and sharing one address register between
// the two would answer the read from the write's address.
reg [24:0] rq_a [0:1];
reg        rq   [0:1];
reg [24:0] wq_a [0:1];
reg  [7:0] wq_d [0:1];
reg        wq   [0:1];

// The last word each channel read, and which byte of it is being answered.
reg [23:0] last_a  [0:1];
reg [15:0] last_d  [0:1];
reg        last_ok [0:1];
reg        sel_hi  [0:1];

assign ch0_dout = sel_hi[0] ? last_d[0][15:8] : last_d[0][7:0];
assign ch1_dout = sel_hi[1] ? last_d[1][15:8] : last_d[1][7:0];

// A channel is ready when it has nothing queued and nothing in flight. Deriving
// it rather than pulsing it means a read queued behind a write cannot be
// answered early, whatever order the two arrive in. A read that hits the cached
// word queues nothing, so ready never drops and the CPU is not stalled at all.
assign ch0_ready = ~(rq[0] | wq[0] | busy[0] | booting);
assign ch1_ready = ~(rq[1] | wq[1] | busy[1] | booting);

// Power up: the part wants 100 us of NOPs, then all banks precharged, two
// refreshes and the mode register. One slot is 164 ns, so counting slots gives
// each command far more than the tRP, tRFC and tMRD it needs, and 2047 slots of
// waiting is 336 us. init comes from the PLL lock, which on its own is already
// long after power on, so this is belt and braces.
reg [10:0] boot_slot = 11'h7FF;
wire       booting   = (boot_slot != 0);
reg  [2:0] boot_cmd;

always @(*) begin
   case (boot_slot)
      11'd20:  boot_cmd = CMD_PRECHARGE;
      11'd15:  boot_cmd = CMD_AUTO_REFRESH;
      11'd10:  boot_cmd = CMD_AUTO_REFRESH;
      11'd5:   boot_cmd = CMD_LOAD_MODE;
      default: boot_cmd = CMD_NOP;
   endcase
end

reg [11:0] refresh_cnt;
wire refresh_due  = (refresh_cnt >= {REFRESH_PERIOD, 1'b0}); // overdue: takes the bus
wire refresh_want = (refresh_cnt >= {1'b0, REFRESH_PERIOD}); // due: takes a free slot

wire [1:0] dqm = {we & ~a[0], we & a[0]};

// The bus, sampled every clock. The top level drives SDRAM_CLK from ~clk_sys, so
// the part clocks half a cycle ahead of us: a READ issued in STATE_CONT is latched
// in the middle of the next slot state and its data is on the bus by the clock
// edge that begins STATE_READY, which is the edge this register catches it on.
// Inverting SDRAM_CLK is therefore not cosmetic - without it every read would be
// sampled a cycle early.
reg [15:0] sdram_q;

//-----------------------------------------------------------------------------
// Access manager: captures requests, arbitrates, and answers the channels.
//-----------------------------------------------------------------------------
always @(posedge clk) begin
   reg old_rd0, old_wr0, old_rd1, old_wr1;

   refresh_cnt <= refresh_cnt + 1'd1;

   // ---- capture ------------------------------------------------------------
   old_rd0 <= ch0_rd;
   if (ch0_rd & ~old_rd0) begin
      if (last_ok[0] && (last_a[0] == ch0_addr[24:1])) begin
         sel_hi[0] <= ch0_addr[0];        // already have the word: no bus cycle
      end else begin
         rq[0]     <= 1'b1;
         rq_a[0]   <= ch0_addr;
      end
   end
   old_wr0 <= ch0_wr;
   if (ch0_wr & ~old_wr0) begin
      wq[0]      <= 1'b1;
      wq_a[0]    <= ch0_addr;
      wq_d[0]    <= ch0_din;
      last_ok[0] <= 1'b0;                 // the cached word may be what we wrote over
   end

   old_rd1 <= ch1_rd;
   if (ch1_rd & ~old_rd1) begin
      if (last_ok[1] && (last_a[1] == ch1_addr[24:1])) begin
         sel_hi[1] <= ch1_addr[0];
      end else begin
         rq[1]     <= 1'b1;
         rq_a[1]   <= ch1_addr;
      end
   end
   old_wr1 <= ch1_wr;
   if (ch1_wr & ~old_wr1) begin
      wq[1]      <= 1'b1;
      wq_a[1]    <= ch1_addr;
      wq_d[1]    <= ch1_din;
      last_ok[1] <= 1'b0;
   end

   // ---- arbitrate ----------------------------------------------------------
   if (state == STATE_IDLE) begin
      if (booting) begin
         state <= STATE_START;
      end
      // An overdue refresh outranks the channels; a merely due one waits for a
      // slot nobody wants, so the CPU is stalled as little as possible.
      else if (refresh_due) begin
         ram_req     <= 1'b0;
         refresh_cnt <= refresh_cnt - {1'b0, REFRESH_PERIOD} + 1'd1;
         state       <= STATE_START;
      end
      // ch0 is the cartridge, so it outranks the expander: it carries the Z80's
      // instruction stream. Within a channel the write goes first, because a
      // write queued alongside a read is always the older of the two.
      else if (wq[0]) begin
         wq[0]     <= 1'b0;
         we        <= 1'b1;
         {bank, a} <= wq_a[0];
         wdata     <= wq_d[0];
         serving   <= 1'b0;
         busy[0]   <= 1'b1;
         ram_req   <= 1'b1;
         state     <= STATE_START;
      end
      else if (rq[0]) begin
         rq[0]     <= 1'b0;
         we        <= 1'b0;
         {bank, a} <= rq_a[0];
         serving   <= 1'b0;
         busy[0]   <= 1'b1;
         ram_req   <= 1'b1;
         state     <= STATE_START;
      end
      else if (wq[1]) begin
         wq[1]     <= 1'b0;
         we        <= 1'b1;
         {bank, a} <= wq_a[1];
         wdata     <= wq_d[1];
         serving   <= 1'b1;
         busy[1]   <= 1'b1;
         ram_req   <= 1'b1;
         state     <= STATE_START;
      end
      else if (rq[1]) begin
         rq[1]     <= 1'b0;
         we        <= 1'b0;
         {bank, a} <= rq_a[1];
         serving   <= 1'b1;
         busy[1]   <= 1'b1;
         ram_req   <= 1'b1;
         state     <= STATE_START;
      end
      else if (refresh_want) begin
         ram_req     <= 1'b0;
         refresh_cnt <= refresh_cnt - {1'b0, REFRESH_PERIOD} + 1'd1;
         state       <= STATE_START;
      end
   end
   else begin
      state <= (state == STATE_LAST) ? STATE_IDLE : state + 3'd1;
   end

   // ---- answer -------------------------------------------------------------
   if (state == STATE_READY) begin
      if (booting) begin
         boot_slot <= boot_slot - 1'd1;
      end
      else if (ram_req) begin
         busy <= 2'b00;
         if (!we) begin
            last_d[serving]  <= sdram_q;
            last_a[serving]  <= {bank, a[22:1]};
            last_ok[serving] <= 1'b1;
            sel_hi[serving]  <= a[0];
         end
      end
   end

   if (init) begin
      boot_slot   <= 11'h7FF;
      state       <= STATE_IDLE;
      refresh_cnt <= 0;
      ram_req     <= 1'b0;
      busy        <= 2'b00;
      rq[0]       <= 1'b0;
      wq[0]       <= 1'b0;
      rq[1]       <= 1'b0;
      wq[1]       <= 1'b0;
      last_ok[0]  <= 1'b0;
      last_ok[1]  <= 1'b0;
   end
end

//-----------------------------------------------------------------------------
// Bus: commands and addresses, keyed on the slot state.
//-----------------------------------------------------------------------------
always @(posedge clk) begin
   dq_oe   <= 1'b0;
   sdram_q <= SDRAM_DQ;

   cmd     <= CMD_NOP;
   SDRAM_A <= 13'd0;

   if (state == STATE_START) SDRAM_BA <= booting ? 2'b00 : bank;

   if (booting) begin
      if (state == STATE_START) begin
         cmd <= boot_cmd;
         if (boot_cmd == CMD_LOAD_MODE)      SDRAM_A <= MODE;
         else if (boot_cmd == CMD_PRECHARGE) SDRAM_A <= 13'b0010000000000; // A10: all banks
      end
   end
   else if (ram_req) begin
      if (state == STATE_START) begin
         cmd     <= CMD_ACTIVE;
         SDRAM_A <= a[13:1];
      end
      else if (state == STATE_CONT) begin
         cmd     <= we ? CMD_WRITE : CMD_READ;
         // A10 high is auto precharge, so the row closes itself and the next
         // slot can open any row it likes.
         SDRAM_A <= {dqm, 2'b10, a[22:14]};
         // The byte goes on both halves and DQM picks the one that lands.
         if (we) begin
            dq_out <= {wdata, wdata};
            dq_oe  <= 1'b1;
         end
      end
   end
   else if (state == STATE_START) begin
      cmd <= CMD_AUTO_REFRESH;
   end
end

endmodule
