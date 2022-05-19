//============================================================================
//  ColecoVision
//
//  Port to MiSTer
//  Copyright (C) 2017-2019 Sorgelig
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module emu
  #
  (
   parameter  NUM_DISKS = 4,
   parameter  NUM_TAPES = 4,
   parameter  USE_REQ   = 0,
   parameter  TOT_DISKS = NUM_DISKS + NUM_TAPES
   )
  (
        //Master input clock
        input         CLK_50M,

        //Async reset from top-level module.
        //Can be used as initial reset.
        input         RESET,

        //Must be passed to hps_io module
        inout  [48:0] HPS_BUS,

        //Base video clock. Usually equals to CLK_SYS.
        output        CLK_VIDEO,

        //Multiple resolutions are supported using different CE_PIXEL rates.
        //Must be based on CLK_VIDEO
        output        CE_PIXEL,

        //Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
        //if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
        output [12:0] VIDEO_ARX,
        output [12:0] VIDEO_ARY,

        output  [7:0] VGA_R,
        output  [7:0] VGA_G,
        output  [7:0] VGA_B,
        output        VGA_HS,
        output        VGA_VS,
        output        VGA_DE,    // = ~(VBlank | HBlank)
        output        VGA_F1,
        output [1:0]  VGA_SL,
        output        VGA_SCALER, // Force VGA scaler

        input  [11:0] HDMI_WIDTH,
        input  [11:0] HDMI_HEIGHT,
        output        HDMI_FREEZE,

`ifdef MISTER_FB
        // Use framebuffer in DDRAM (USE_FB=1 in qsf)
        // FB_FORMAT:
        //    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
        //    [3]   : 0=16bits 565 1=16bits 1555
        //    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
        //
        // FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
        output        FB_EN,
        output  [4:0] FB_FORMAT,
        output [11:0] FB_WIDTH,
        output [11:0] FB_HEIGHT,
        output [31:0] FB_BASE,
        output [13:0] FB_STRIDE,
        input         FB_VBL,
        input         FB_LL,
        output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
        // Palette control for 8bit modes.
        // Ignored for other video modes.
        output        FB_PAL_CLK,
        output  [7:0] FB_PAL_ADDR,
        output [23:0] FB_PAL_DOUT,
        input  [23:0] FB_PAL_DIN,
        output        FB_PAL_WR,
`endif
`endif

        output        LED_USER,  // 1 - ON, 0 - OFF.

        // b[1]: 0 - LED status is system status OR'd with b[0]
        //       1 - LED status is controled solely by b[0]
        // hint: supply 2'b00 to let the system control the LED.
        output  [1:0] LED_POWER,
        output  [1:0] LED_DISK,

        // I/O board button press simulation (active high)
        // b[1]: user button
        // b[0]: osd button
        output  [1:0] BUTTONS,

        input         CLK_AUDIO, // 24.576 MHz
        output [15:0] AUDIO_L,
        output [15:0] AUDIO_R,
        output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
        output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

        //ADC
        inout   [3:0] ADC_BUS,

        //SD-SPI
        output        SD_SCK,
        output        SD_MOSI,
        input         SD_MISO,
        output        SD_CS,
        input         SD_CD,

        //High latency DDR3 RAM interface
        //Use for non-critical time purposes
        output        DDRAM_CLK,
        input         DDRAM_BUSY,
        output  [7:0] DDRAM_BURSTCNT,
        output [28:0] DDRAM_ADDR,
        input  [63:0] DDRAM_DOUT,
        input         DDRAM_DOUT_READY,
        output        DDRAM_RD,
        output [63:0] DDRAM_DIN,
        output  [7:0] DDRAM_BE,
        output        DDRAM_WE,

        //SDRAM interface with lower latency
        output        SDRAM_CLK,
        output        SDRAM_CKE,
        output [12:0] SDRAM_A,
        output  [1:0] SDRAM_BA,
        inout  [15:0] SDRAM_DQ,
        output        SDRAM_DQML,
        output        SDRAM_DQMH,
        output        SDRAM_nCS,
        output        SDRAM_nCAS,
        output        SDRAM_nRAS,
        output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
        //Secondary SDRAM
        //Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
        input         SDRAM2_EN,
        output        SDRAM2_CLK,
        output [12:0] SDRAM2_A,
        output  [1:0] SDRAM2_BA,
        inout  [15:0] SDRAM2_DQ,
        output        SDRAM2_nCS,
        output        SDRAM2_nCAS,
        output        SDRAM2_nRAS,
        output        SDRAM2_nWE,
`endif

        input         UART_CTS,
        output        UART_RTS,
        input         UART_RXD,
        output        UART_TXD,
        output        UART_DTR,
        input         UART_DSR,

        // Open-drain User port.
        // 0 - D+/RX
        // 1 - D-/TX
        // 2..6 - USR2..USR6
        // Set USER_OUT to 1 to read from USER_IN.
        input   [6:0] USER_IN,
        output  [6:0] USER_OUT,

        input         OSD_STATUS
);

assign ADC_BUS = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;

assign LED_USER   = ioctl_download;
assign LED_DISK   = 0;
assign LED_POWER  = 0;
assign BUTTONS    = 0;
assign VGA_SCALER = 0;
assign HDMI_FREEZE= 0;

wire [1:0] ar = status[2:1];
wire vga_de;
reg  en216p;
always @(posedge CLK_VIDEO) en216p <= ((HDMI_WIDTH == 1920) && (HDMI_HEIGHT == 1080) && !forced_scandoubler && !scale);

video_freak video_freak
(
        .*,
        .VGA_DE_IN(vga_de),
        .ARX((!ar) ? 12'd4 : (ar - 1'd1)),
        .ARY((!ar) ? 12'd3 : 12'd0),
        .CROP_SIZE(en216p ? 10'd216 : 10'd0),
        .CROP_OFF(0),
        .SCALE(status[11:10])
);

`include "build_id.v"
parameter CONF_STR = {
        "Adam;;",
        "-;",
        "F,COLBINROM;",
        "F,SG,Load SG-1000;",
        "S0,DSK,Load Floppy 1;",
        "S1,DSK,Load Floppy 2;",
        "S2,DSK,Load Floppy 3;",
        "S3,DSK,Load Floppy 4;",
        "S4,DDP,Load Tape 1;",
        "S5,DDP,Load Tape 2;",
        "S6,DDP,Load Tape 3;",
        "S7,DDP,Load Tape 4;",
        "-;",
        "O12,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
        "O79,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
        "-;",
        "O6,Border,No,Yes;",
        "OAB,Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
        "-;",
        "O3,Joysticks swap,No,Yes;",
        "-;",
        "O45,RAM Size,1KB,8KB,SGM;",
                  "OC,Adam Mode,On,Off;",
        "R0,Reset;",
        "J,Fire 1,Fire 2,*,#,0,1,2,3,4,5,6,7,8,9,Purple Tr,Blue Tr;",
        "V,v",`BUILD_DATE
};

/////////////////  CLOCKS  ////////////////////////

wire clk_sys;
wire pll_locked;

pll pll
(
        .refclk(CLK_50M),
        .rst(0),
        .outclk_0(clk_sys),
        .locked(pll_locked)
);

reg ce_10m7 = 0;
reg ce_5m3 = 0;
always @(posedge clk_sys) begin
        reg [2:0] div;

        div <= div+1'd1;
        ce_10m7 <= !div[1:0];
        ce_5m3  <= !div[2:0];
end

/////////////////  HPS  ///////////////////////////

wire [31:0] status;
wire  [1:0] buttons;

wire [31:0] joy0, joy1;

wire        ioctl_download;
wire  [7:0] ioctl_index;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        forced_scandoubler;
wire [21:0] gamma_bus;

wire [31:0] sd_lba[TOT_DISKS];
reg   [TOT_DISKS-1:0] sd_rd;
reg   [TOT_DISKS-1:0] sd_wr;
wire  [TOT_DISKS-1:0] sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din[TOT_DISKS];
wire        sd_buff_wr;

wire  [TOT_DISKS-1:0] img_mounted;
wire        img_readonly;

wire [63:0] img_size;

wire [10:0] ps2_key;

hps_io #(.CONF_STR(CONF_STR), .VDNUM(TOT_DISKS)) hps_io
(
   .clk_sys(clk_sys),
   .HPS_BUS(HPS_BUS),

   .buttons(buttons),
   .status(status),
   .forced_scandoubler(forced_scandoubler),
   .gamma_bus(gamma_bus),
   .sd_lba(sd_lba),
   .sd_rd(sd_rd),
   .sd_wr(sd_wr),
   .sd_ack(sd_ack),
   .sd_buff_addr(sd_buff_addr),
   .sd_buff_dout(sd_buff_dout),
   .sd_buff_din(sd_buff_din),
   .sd_buff_wr(sd_buff_wr),
   .img_mounted(img_mounted),
   .img_readonly(img_readonly),
   .img_size(img_size),

   .ioctl_download(ioctl_download),
   .ioctl_index(ioctl_index),
   .ioctl_wr(ioctl_wr),
   .ioctl_addr(ioctl_addr),
   .ioctl_dout(ioctl_dout),
        .ps2_key(ps2_key),

   .joystick_0(joy0),
   .joystick_1(joy1)
);

/////////////////  RESET  /////////////////////////

wire reset = RESET | status[0] | buttons[1] | ioctl_download;

/////////////////  Memory  ////////////////////////

wire [12:0] bios_a;
wire  [7:0] bios_d;

`ifdef NO
spram #(13,8,"rtl/bios.mif") rom0
(
        .clock(clk_sys),
        .address(bios_a),
        .q(bios_d)
);
`endif
rom #(.AW(13),.DW(8),.FN("rtl/bios.hex")) rom1
(
        .clock(clk_sys),
        .address(bios_a),
        .enable(1'b1),
        .q(bios_d)
);

wire [14:0] writer_a;
wire  [7:0] writer_d;
rom #(15,8,"rtl/writer.hex") rom2
(
        .clock(clk_sys),
        .address(writer_a),
        .enable(1'b1),
        .q(writer_d)
);

wire [13:0] eos_a;
wire  [7:0] eos_d;

rom #(14,8,"rtl/eos.hex") rom3
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

//wire [14:0] ram_a = cpu_ram_a;

wire [14:0] ram_a = (extram|adam)            ? cpu_ram_a       :
                    (status[5:4] == 1)  ? cpu_ram_a[12:0] : // 8k
                    (status[5:4] == 0)  ? cpu_ram_a[9:0]  : // 1k
                    (sg1000)            ? cpu_ram_a[12:0] : // SGM means 8k on SG1000
                                          cpu_ram_a;        // SGM/32k




														
  logic [15:0] ramb_addr;
  logic        ramb_wr;
  logic        ramb_rd;
  logic [7:0]  ramb_dout;
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
        .q_b(),

        .enable_b(1'b1),
        .ce_a(1'b1)
);

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
     .cs(1'b1)
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
     .q_b(),

     .enable_b(1'b1),
     .ce_a(1'b1)
     );

  always @(posedge clk_sys) begin
    ramb_wr_ack <= ramb_wr;
    ramb_rd_ack <= ramb_rd;
  end


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
        .q(vram_di)
);

wire [19:0] cart_a;
wire  [7:0] cart_d;
wire        cart_rd;

reg [5:0] cart_pages;
always @(posedge clk_sys) if(ioctl_wr) cart_pages <= ioctl_addr[19:14];

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

assign CLK_VIDEO = clk_sys;

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

wire [31:0] joya = status[3] ? joy1 : joy0;
wire [31:0] joyb = status[3] ? joy0 : joy1;

wire adam=~status[12];

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
        .sg1000(sg1000),
        .dahjeeA_i(extram),
        .adam(adam),

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

        .border_i(status[6]),
        .rgb_r_o(R),
        .rgb_g_o(G),
        .rgb_b_o(B),
        .hsync_n_o(hsync),
        .vsync_n_o(vsync),
        .hblank_o(hblank),
        .vblank_o(vblank),

        .audio_o(audio),
        .disk_present(disk_present),
        .disk_sector(disk_sector),
        .disk_load(disk_load),
        .disk_sector_loaded(disk_sector_loaded),
        .disk_addr(disk_addr),
        .disk_wr(disk_wr),
        .disk_flush(disk_flush),
        .disk_error(disk_error),
        .disk_data(disk_data),
        .disk_din(disk_din),
        .ps2_key     (ps2_key)
);
  genvar                tla_i;
  generate
    for (tla_i = 0; tla_i < TOT_DISKS; tla_i++) begin : g_TL
      track_loader_adam
        #
        (
        .drive_num      (tla_i)
        )
      track_loader_a
        (
        .clk            (clk_sys),
        .reset          (reset),
        .img_mounted    (img_mounted[tla_i]),
        .img_size       (img_size),
        .lba_fdd        (sd_lba[tla_i]),
        .sd_ack         (sd_ack[tla_i]),
        .sd_rd          (sd_rd[tla_i]),
        .sd_wr          (sd_wr[tla_i]),
        .sd_buff_addr   (sd_buff_addr),
        .sd_buff_wr     (sd_buff_wr),
        .sd_buff_dout   (sd_buff_dout),
        .sd_buff_din    (sd_buff_din[tla_i]),

        // Disk interface
        .disk_present   (disk_present[tla_i]),
        .disk_sector    (disk_sector),
        .disk_load      (disk_load[tla_i]),
        .disk_sector_loaded (disk_sector_loaded[tla_i]),
        .disk_addr          (disk_addr),
        .disk_wr            (disk_wr[tla_i]),
        .disk_flush         (disk_flush[tla_i]),
        .disk_error         (disk_error[tla_i]),
        .disk_din           (disk_din),
        .disk_data          (disk_data[tla_i])
        );
    end // block: g_TL
  endgenerate

assign VGA_F1 = 0;
assign VGA_SL = sl[1:0];

wire [2:0] scale = status[9:7];
wire [2:0] sl = scale ? scale - 1'd1 : 3'd0;

reg hs_o, vs_o;
always @(posedge CLK_VIDEO) begin
        hs_o <= ~hsync;
        if(~hs_o & ~hsync) vs_o <= ~vsync;
end

video_mixer #(.LINE_LENGTH(290), .GAMMA(1)) video_mixer
(
        .*,

        .ce_pix(ce_5m3),
        .freeze_sync(),

        .scandoubler(scale || forced_scandoubler),
        .hq2x(scale==1),

        .VGA_DE(vga_de),
        .R(R),
        .G(G),
        .B(B),

        // Positive pulses.
        .HSync(hs_o),
        .VSync(vs_o),
        .HBlank(hblank),
        .VBlank(vblank)
);



////////////////  Control  ////////////////////////

wire [0:19] keypad0 = {joya[8],joya[9],joya[10],joya[11],joya[12],joya[13],joya[14],joya[15],joya[16],joya[17],joya[6],joya[7],joya[18],joya[19],joya[3],joya[2],joya[1],joya[0],joya[4],joya[5]};
wire [0:19] keypad1 = {joyb[8],joyb[9],joyb[10],joyb[11],joyb[12],joyb[13],joyb[14],joyb[15],joyb[16],joyb[17],joyb[6],joyb[7],joyb[18],joyb[19],joyb[3],joyb[2],joyb[1],joyb[0],joyb[4],joyb[5]};
wire [0:19] keypad[2] = '{keypad0,keypad1};

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
