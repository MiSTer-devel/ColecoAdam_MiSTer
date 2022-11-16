`timescale 1ns / 1ps
/*============================================================================
===========================================================================*/

module emu
  #
  (
   parameter NUM_DISKS = 4,
   parameter NUM_TAPES = 4,
   parameter USE_REQ = 1,
   localparam TOT_DISKS = NUM_DISKS + NUM_TAPES
   )
  (

   input                 clk_sys,
   input                 reset,
   input                 soft_reset,
   input                 menu,
   input                 adam,

   input [31:0]          joystick_0,
   input [31:0]          joystick_1,
   input [31:0]          joystick_2,
   input [31:0]          joystick_3,
   input [31:0]          joystick_4,
   input [31:0]          joystick_5,

   input [15:0]          joystick_l_analog_0,
   input [15:0]          joystick_l_analog_1,
   input [15:0]          joystick_l_analog_2,
   input [15:0]          joystick_l_analog_3,
   input [15:0]          joystick_l_analog_4,
   input [15:0]          joystick_l_analog_5,

   input [15:0]          joystick_r_analog_0,
   input [15:0]          joystick_r_analog_1,
   input [15:0]          joystick_r_analog_2,
   input [15:0]          joystick_r_analog_3,
   input [15:0]          joystick_r_analog_4,
   input [15:0]          joystick_r_analog_5,

   input [7:0]           paddle_0,
   input [7:0]           paddle_1,
   input [7:0]           paddle_2,
   input [7:0]           paddle_3,
   input [7:0]           paddle_4,
   input [7:0]           paddle_5,

   input [8:0]           spinner_0,
   input [8:0]           spinner_1,
   input [8:0]           spinner_2,
   input [8:0]           spinner_3,
   input [8:0]           spinner_4,
   input [8:0]           spinner_5,

        // ps2 alternative interface.
        // [8] - extended, [9] - pressed, [10] - toggles with every press/release
   input [10:0]          ps2_key,

        // [24] - toggles with every event
   input [24:0]          ps2_mouse,
   input [15:0]          ps2_mouse_ext, // 15:8 - reserved(additional buttons), 7:0 - wheel movements

        // [31:0] - seconds since 1970-01-01 00:00:00, [32] - toggle with every change
   input [32:0]          timestamp,

   output [7:0]          VGA_R,
   output [7:0]          VGA_G,
   output [7:0]          VGA_B,

   output                VGA_HS,
   output                VGA_VS,
   output                VGA_HB,
   output                VGA_VB,

   output                CE_PIXEL,

   output [15:0]         AUDIO_L,
   output [15:0]         AUDIO_R,

   input                 ioctl_download,
   input                 ioctl_wr,
   input [24:0]          ioctl_addr,
   input [7:0]           ioctl_dout,
   input [7:0]           ioctl_index,
   output reg            ioctl_wait=1'b0,

   output [31:0]         sd_lba[TOT_DISKS],
   output [9:0]          sd_rd,
   output [9:0]          sd_wr,
   input [9:0]           sd_ack,
   input [8:0]           sd_buff_addr,
   input [TOT_DISKS-1:0] sd_buff_dout,
   output [7:0]          sd_buff_din[TOT_DISKS],
   input                 sd_buff_wr,
   input [9:0]           img_mounted,
   input                 img_readonly,

   input [63:0]          img_size



);

//  initial begin
 //   $dumpfile("test.fst");
//    $dumpvars;
//  end

 wire [15:0] joystick_a0 =  joystick_l_analog_0;

wire UART_CTS;
wire UART_RTS;
wire UART_RXD;
wire UART_TXD;
wire UART_DTR;
wire UART_DSR;

// CHEAT THE CLOCK TO SPEED IT UP
 reg ce_10m7 = 0;
 reg ce_5m3 = 0;
 always @(posedge clk_sys) begin
       reg [1:0] div;

       div <= div+1'd1;
       ce_10m7 <= !div[0];
       ce_5m3  <= !div[1:0];
 end

/////////////////  Memory  ////////////////////////


wire [12:0] bios_a;
wire  [7:0] bios_d;

`ifdef NO
spram #(13,8,"rtl/bios.mif") rom
(
        .clock(clk_sys),
        .address(bios_a),
        .q(bios_d)
);
`else
rom #(13,8,"rtl/bios.hex") bios
(
        .clock(clk_sys),
        .address(bios_a),
        .enable(1'b1),
        .q(bios_d)
);
`endif

wire [14:0] writer_a;
wire  [7:0] writer_d;
rom #(15,8,"rtl/writer.hex") writer
(
        .clock(clk_sys),
        .address(writer_a),
        .enable(1'b1),
        .q(writer_d)
);

wire [13:0] eos_a;
wire  [7:0] eos_d;

rom #(14,8,"rtl/eos.hex") eos
(
        .clock(clk_sys),
        .address(eos_a),
        .enable(1'b1),
        .q(eos_d)
);

wire [14:0] cpu_ram_a;
wire        ram_we_n, ram_ce_n;
wire  [7:0] ram_di;
wire  [7:0] ram_do;
/*
wire [14:0] ram_a = (extram)            ? cpu_ram_a       :
                    (status[5:4] == 1)  ? cpu_ram_a[12:0] : // 8k
                    (status[5:4] == 0)  ? cpu_ram_a[9:0]  : // 1k
                    (sg1000)            ? cpu_ram_a[12:0] : // SGM means 8k on SG1000
                                          cpu_ram_a;        // SGM/32k
*/
wire [14:0] ram_a = cpu_ram_a;
/*
wire [14:0] ram_a = (extram)     ? cpu_ram_a       :
                    (1'b1 == 1)  ? cpu_ram_a[12:0] : // 8k
                    (1'b1 == 0)  ? cpu_ram_a[9:0]  : // 1k
                    (sg1000)     ? cpu_ram_a[12:0] : // SGM means 8k on SG1000
                                          cpu_ram_a;        // SGM/32k
                                  */

  logic [15:0] ramb_addr;
  logic [15:0] ramb_addr_del;
  logic        ramb_wr;
  logic        ramb_rd;
  logic [7:0]  ramb_dout;
  logic [7:0]  int_ramb_din[2];
  logic [7:0]  ramb_din;
  logic        ramb_wr_ack;
  logic        ramb_rd_ack;

dpramv #(8, 15) ram
(
        .clock_a(clk_sys),
        .address_a(ram_a),
        .wren_a(ce_10m7 & ~(ram_we_n | ram_ce_n)),
        .data_a(ram_do),
        .q_a(ram_di),
        .clock_b(clk_sys),
        .address_b(ramb_addr[14:0]),
        .wren_b(ramb_wr & ~ramb_addr[15]),
        .data_b(ramb_dout),
        .q_b(int_ramb_din[0]),

        .enable_b(1'b1),
        .ce_a(1'b1)
);

  always @(posedge clk_sys) begin
    ramb_wr_ack <= ramb_wr;
    ramb_rd_ack <= ramb_rd;
    ramb_addr_del <= ramb_addr;
  end

  always @* if (ramb_rd) $display("Readingb %0x: %0x", ramb_addr_del[14:0], ramb_din);
  assign ramb_din = ~ramb_addr[15] ? int_ramb_din[0] : int_ramb_din[1];

wire [13:0] vram_a;
wire        vram_we;
wire  [7:0] vram_di;
wire  [7:0] vram_do;

spramv #(14) vram
(
        .clock(clk_sys),
        .address(vram_a),
        .wren(vram_we),
        .data(vram_do),
        .enable(1'b1),
        .cs(1'b1),
        .q(vram_di)
);
/*
  always @(posedge clk_sys) begin
    if (vram_we)
      $display("%t Write VRAM %h: %h", $stime, vram_a, vram_do);
  end
*/
   wire [14:0]         lowerexpansion_ram_a;
   wire lowerexpansion_ram_ce_n;
   wire lowerexpansion_ram_rd_n;
   wire lowerexpansion_ram_we_n;
   wire [7:0] lowerexpansion_ram_di;
   wire [7:0] lowerexpansion_ram_do;
  spramv #(15) lowerexpansion_ram
    (
     .clock(clk_sys),
     .address(lowerexpansion_ram_a),
     .wren(ce_10m7 & ~(lowerexpansion_ram_we_n | lowerexpansion_ram_ce_n)),
     .data(lowerexpansion_ram_do),
     .q(lowerexpansion_ram_di),
     .cs(1'b1),
     );


wire [14:0] upper_ram_a;
wire        upper_ram_we_n, upper_ram_ce_n;
wire  [7:0] upper_ram_di;
wire  [7:0] upper_ram_do;
  dpramv #(8, 15) upper_ram
    (
     .clock_a(clk_sys),
     .address_a(upper_ram_a),
     .wren_a(ce_10m7 & ~(upper_ram_we_n | upper_ram_ce_n)),
     .data_a(upper_ram_do),
     .q_a(upper_ram_di),
     .clock_b(clk_sys),
     .address_b(ramb_addr[14:0]),
     .wren_b(ramb_wr & ramb_addr[15]),
     .data_b(ramb_dout),
     .q_b(int_ramb_din[1]),

     .enable_b(1'b1),
     .ce_a(1'b1)
     );
/*
  always @(posedge clk_sys) begin
    if (ce_10m7 && ~upper_ram_ce_n &&
        ((upper_ram_a > 15'h400 && upper_ram_a < 15'hfff ||
          (upper_ram_a > 15'hc800 && upper_ram_a < 15'hcbff))))
        $display("Read Disk %h: %h", upper_ram_a, upper_ram_do);
  end
*/
wire [19:0] cart_a;
wire  [7:0] cart_d;
wire        cart_rd;

reg [5:0] cart_pages;
always @(posedge clk_sys) if(ioctl_wr) cart_pages <= ioctl_addr[19:14];

/*
assign SDRAM_CLK = ~clk_sys;
sdram sdram
(
        .*,
        .init(~pll_locked),
        .clk(clk_sys),

   .wtbt(0),
   .addr(ioctl_download ? ioctl_addr : cart_a),
   .rd(cart_rd),
   .dout(cart_d),
   .din(ioctl_dout),
   .we(ioctl_wr),
   .ready()
);
*/
spramv #(20) cart
(
        .clock(clk_sys),
        .address(ioctl_download ? ioctl_addr : cart_a),
        .wren(ioctl_wr),
        .data(ioctl_dout),
        .enable(1'b1),
        .cs(1'b1),
        .q(cart_d)
);

reg sg1000 = 0;
reg extram = 0;
always @(posedge clk_sys) begin
        if(ioctl_wr) begin
                if(!ioctl_addr) begin
                        extram <= 0;
                        sg1000 <= (ioctl_index[4:0] == 2);
                end
                if(ioctl_addr[24:13] == 1 && sg1000) extram <= (!ioctl_addr[12:0] | extram) & &ioctl_dout; // 2000-3FFF on SG-1000
        end
end


////////////////  Console  ////////////////////////

wire [10:0] audio;
assign AUDIO_L = {audio,5'd0};
assign AUDIO_R = {audio,5'd0};
assign AUDIO_S = 0;
assign AUDIO_MIX = 0;


wire [1:0] ctrl_p1;
wire [1:0] ctrl_p2;
wire [1:0] ctrl_p3;
wire [1:0] ctrl_p4;
wire [1:0] ctrl_p5;
wire [1:0] ctrl_p6;
wire [1:0] ctrl_p7 = 2'b11;
wire [1:0] ctrl_p8;
wire [1:0] ctrl_p9 = 2'b11;

wire [7:0] R,G,B;
wire hblank, vblank;
wire hsync, vsync;

wire [31:0] joya = joystick_0;
wire [31:0] joyb = joystick_1;

wire [19:0] ext_rom_a;
wire  [7:0] ext_rom_d=8'hff;


  logic [TOT_DISKS-1:0] disk_present;
  logic [31:0]          disk_sector; // sector
  logic [TOT_DISKS-1:0] disk_load; // load the 512 byte sector
  logic [TOT_DISKS-1:0] disk_sector_loaded; // set high when sector ready
  logic [8:0]           disk_addr; // Byte to read or write from sector
  logic [TOT_DISKS-1:0] disk_wr; // Write data into sector (read when low)
  logic [TOT_DISKS-1:0] disk_flush; // sector access done, so flush (hint)
  logic [TOT_DISKS-1:0] disk_error; // out of bounds (?)
  logic [7:0]           disk_data[TOT_DISKS];
  logic [7:0]           disk_din;


  logic mode = ~adam;

  cv_console
    #
    (
     .NUM_DISKS (NUM_DISKS),
     .NUM_TAPES (NUM_TAPES),
     .USE_REQ   (USE_REQ)
     )
  console
    (
     .clk_i(clk_sys),
     .clk_en_10m7_i(ce_10m7),
     .reset_n_i(~reset),
     .por_n_o(),
     //.sg1000(sg1000),
     //.dahjeeA_i(extram),
     //.adam(adam),
     .mode(mode),

     .ctrl_p1_i(ctrl_p1),
     .ctrl_p2_i(ctrl_p2),
     .ctrl_p3_i(ctrl_p3),
     .ctrl_p4_i(ctrl_p4),
     .ctrl_p5_o(ctrl_p5),
     .ctrl_p6_i(ctrl_p6),
     .ctrl_p7_i(ctrl_p7),
     .ctrl_p8_o(ctrl_p8),
     .ctrl_p9_i(ctrl_p9),
     .joy0_i(~{|joya[19:6], 1'b0, joya[5:0]}),
     .joy1_i(~{|joyb[19:6], 1'b0, joyb[5:0]}),

     .bios_rom_a_o(bios_a),
     .bios_rom_d_i(bios_d),

     .eos_rom_a_o(eos_a),
     .eos_rom_d_i(eos_d),

     .writer_rom_a_o(writer_a),
     .writer_rom_d_i(writer_d),

     .cpu_ram_a_o(cpu_ram_a),
     .cpu_ram_we_n_o(ram_we_n),
     .cpu_ram_ce_n_o(ram_ce_n),
     .cpu_ram_d_i(ram_di),
     .cpu_ram_d_o(ram_do),

     .cpu_lowerexpansion_ram_a_o(lowerexpansion_ram_a),
     .cpu_lowerexpansion_ram_we_n_o(lowerexpansion_ram_we_n),
     .cpu_lowerexpansion_ram_ce_n_o(lowerexpansion_ram_ce_n),
     .cpu_lowerexpansion_ram_d_i(lowerexpansion_ram_di),
     .cpu_lowerexpansion_ram_d_o(lowerexpansion_ram_do),

     .cpu_upper_ram_a_o(upper_ram_a),
     .cpu_upper_ram_we_n_o(upper_ram_we_n),
     .cpu_upper_ram_ce_n_o(upper_ram_ce_n),
     .cpu_upper_ram_d_i(upper_ram_di),
     .cpu_upper_ram_d_o(upper_ram_do),

     .ramb_addr(ramb_addr),
     .ramb_wr(ramb_wr),
     .ramb_rd(ramb_rd),
     .ramb_dout(ramb_dout),
     .ramb_din(ramb_din),
     .ramb_wr_ack(ramb_wr_ack),
     .ramb_rd_ack(ramb_rd_ack),

     .vram_a_o(vram_a),
     .vram_we_o(vram_we),
     .vram_d_o(vram_do),
     .vram_d_i(vram_di),

     .cart_pages_i(cart_pages),
     .cart_a_o(cart_a),
     .cart_d_i(cart_d),
     .cart_rd(cart_rd),

     .ext_rom_a_o(ext_rom_a),
     .ext_rom_d_i(ext_rom_d),


     .border_i(1'b1),
     .rgb_r_o(R),
     .rgb_g_o(G),
     .rgb_b_o(B),
     .hsync_n_o(hsync),
     .vsync_n_o(vsync),
     .hblank_o(hblank),
     .vblank_o(vblank),

     .audio_o(audio),

     .disk_present(disk_present),
     //.disk_present('1),
     .disk_sector(disk_sector),
     .disk_load(disk_load),
     .disk_sector_loaded(disk_sector_loaded),
     .disk_addr(disk_addr),
     .disk_wr(disk_wr),
     .disk_flush(disk_flush),
     .disk_error(disk_error),
     .disk_data(disk_data),
     .disk_din(disk_din),

     .ps2_key (ps2_key)
     );

  generate
    for (genvar i = 0; i < TOT_DISKS; i++) begin : g_TL
      track_loader_adam
        #
        (
        .drive_num      (i)
        )
      track_loader_a
        (
        .clk            (clk_sys),
        .reset          (reset),
        .img_mounted    (img_mounted[i]),
        .img_size       (img_size),
        .lba_fdd        (sd_lba[i]),
        .sd_ack         (sd_ack[i]),
        .sd_rd          (sd_rd[i]),
        .sd_wr          (sd_wr[i]),
        .sd_buff_addr   (sd_buff_addr),
        .sd_buff_wr     (sd_buff_wr),
        .sd_buff_dout   (sd_buff_dout),
        .sd_buff_din    (sd_buff_din[i]),

        // Disk interface
        .disk_present   (disk_present[i]),
        .disk_sector    (disk_sector),
        .disk_load      (disk_load[i]),
        .disk_sector_loaded (disk_sector_loaded[i]),
        .disk_addr          (disk_addr),
        .disk_wr            (disk_wr[i]),
        .disk_flush         (disk_flush[i]),
        .disk_error         (disk_error[i]),
        .disk_din           (disk_din),
        .disk_data          (disk_data[i])
        );
    end // block: g_TL
  endgenerate

     /*
  track_loader_adam
    #
    (
     .drive_num      (1)
     )
  track_loader_b
    (
     .clk            (clk_sys),
     .reset          (reset),
     .img_mounted    (img_mounted[1]),
     .img_size       (img_size),
     .lba_fdd        (sd_lba[1]),
     .sd_ack         (sd_ack[1]),
     .sd_rd          (sd_rd[1]),
     .sd_wr          (sd_wr[1]),
     .sd_buff_addr   (sd_buff_addr),
     .sd_buff_wr     (sd_buff_wr),
     .sd_buff_dout   (sd_buff_dout),
     .sd_buff_din    (sd_buff_din[1]),

     // Disk interface
     .disk_present   (disk_present),
     .disk_sector    (disk_sector),
     .disk_load      (disk_load),
     .disk_sector_loaded (disk_sector_loaded),
     .disk_addr          (disk_addr),
     .disk_wr            (disk_wr),
     .disk_flush         (disk_flush),
     .disk_error         (disk_error),
     .disk_din           (disk_din),
     .disk_data          (disk_data)
     );
*/

assign VGA_R=R;
assign VGA_G=G;
assign VGA_B=B;
assign VGA_HS=hsync;
assign VGA_VS=vsync;
assign VGA_HB=hblank;
assign VGA_VB=vblank;

assign CE_PIXEL=ce_5m3;




////////////////  Control  ////////////////////////

wire [0:19] keypad0 = {joya[8],joya[9],joya[10],joya[11],joya[12],joya[13],joya[14],joya[15],joya[16],joya[17],joya[6],joya[7],joya[18],joya[19],joya[3],joya[2],joya[1],joya[0],joya[4],joya[5]};
wire [0:19] keypad1 = {joyb[8],joyb[9],joyb[10],joyb[11],joyb[12],joyb[13],joyb[14],joyb[15],joyb[16],joyb[17],joyb[6],joyb[7],joyb[18],joyb[19],joyb[3],joyb[2],joyb[1],joyb[0],joyb[4],joyb[5]};
wire [0:19] keypad[2];
assign keypad[0] = keypad0;
assign keypad[1] = keypad1;

reg [3:0] ctrl1[2] = '{'0,'0};
assign {ctrl_p1[0],ctrl_p2[0],ctrl_p3[0],ctrl_p4[0]} = ctrl1[0];
assign {ctrl_p1[1],ctrl_p2[1],ctrl_p3[1],ctrl_p4[1]} = ctrl1[1];

localparam cv_key_0_c        = 4'b0011;
localparam cv_key_1_c        = 4'b1110;
localparam cv_key_2_c        = 4'b1101;
localparam cv_key_3_c        = 4'b0110;
localparam cv_key_4_c        = 4'b0001;
localparam cv_key_5_c        = 4'b1001;
localparam cv_key_6_c        = 4'b0111;
localparam cv_key_7_c        = 4'b1100;
localparam cv_key_8_c        = 4'b1000;
localparam cv_key_9_c        = 4'b1011;
localparam cv_key_asterisk_c = 4'b1010;
localparam cv_key_number_c   = 4'b0101;
localparam cv_key_pt_c       = 4'b0100;
localparam cv_key_bt_c       = 4'b0010;
localparam cv_key_none_c     = 4'b1111;

generate
        genvar i;
        for (i = 0; i <= 1; i++) begin : ctl
                always_comb begin
                        reg [3:0] ctl1, ctl2;
                        reg p61,p62;

                        ctl1 = 4'b1111;
                        ctl2 = 4'b1111;
                        p61 = 1;
                        p62 = 1;

                        if (~ctrl_p5[i]) begin
                                casex(keypad[i][0:13])
                                        'b1xxxxxxxxxxxxx: ctl1 = cv_key_0_c;
                                        'b01xxxxxxxxxxxx: ctl1 = cv_key_1_c;
                                        'b001xxxxxxxxxxx: ctl1 = cv_key_2_c;
                                        'b0001xxxxxxxxxx: ctl1 = cv_key_3_c;
                                        'b00001xxxxxxxxx: ctl1 = cv_key_4_c;
                                        'b000001xxxxxxxx: ctl1 = cv_key_5_c;
                                        'b0000001xxxxxxx: ctl1 = cv_key_6_c;
                                        'b00000001xxxxxx: ctl1 = cv_key_7_c;
                                        'b000000001xxxxx: ctl1 = cv_key_8_c;
                                        'b0000000001xxxx: ctl1 = cv_key_9_c;
                                        'b00000000001xxx: ctl1 = cv_key_asterisk_c;
                                        'b000000000001xx: ctl1 = cv_key_number_c;
                                        'b0000000000001x: ctl1 = cv_key_pt_c;
                                        'b00000000000001: ctl1 = cv_key_bt_c;
                                        'b00000000000000: ctl1 = cv_key_none_c;
                                endcase
                                p61 = ~keypad[i][19]; // button 2
                        end

                        if (~ctrl_p8[i]) begin
                                ctl2 = ~keypad[i][14:17];
                                p62 = ~keypad[i][18];  // button 1
                        end

                        ctrl1[i] = ctl1 & ctl2;
                        ctrl_p6[i] = p61 & p62;
                end
        end
endgenerate


endmodule
