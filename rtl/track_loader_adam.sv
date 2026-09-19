module track_loader_adam
  #
  (
   parameter drive_num= 0
   )
  (
   input               clk,
   input               reset,
   input               img_mounted, // High when image is mounted
   input [63:0]        img_size, // Size of image
   output logic [31:0] lba_fdd,
   input               sd_ack,
   output logic        sd_rd,
   output logic        sd_wr,
   input [8:0]         sd_buff_addr,
   input               sd_buff_wr,
   input [7:0]         sd_buff_dout,
   output logic [7:0]  sd_buff_din,

   // Disk interface
   output logic        disk_present, // We have a disk loaded
   input [31:0]        disk_sector, // sector
   input               disk_load, // load the 512 byte sector
   output logic        disk_sector_loaded, // set high when sector ready
   input [8:0]         disk_addr, // Byte to read or write from sector
   input               disk_wr, // Write data into sector (read when low)
   input               disk_flush, // sector access done, so flush (hint)
   output              disk_flushed, // Flush completed
   output logic        disk_error, // out of bounds (?)
   input [7:0]         disk_din,
   output logic [7:0]  disk_data
   );

  enum bit [2:0] {
                  IDLE,
                  READ,
                  WRITE,
                  W4IDLE_READ,
                  W4IDLE_WRITE
                  } floppy_state;

  // when we write to the disk, we need to mark it dirty
  logic                         floppy_track_dirty;

  logic [63:0]                  disk_size; // Size of disk loaded in bytes
  logic [31:0]                  curr_sector;
  // Which sector the 512 byte buffer actually holds. The buffer is the only
  // copy of that sector's data, so a flush has to go back to *this* LBA, not to
  // whatever sector the AdamNet side happens to point at when the flush lands.
  // The two differ on every 1024 byte (two sector) EOS block write, because
  // cv_adamnet advances disk_sec between the halves without reloading.
  logic [31:0]                  buf_sector;
  logic                         buf_valid;   // the buffer holds a real sector
  logic                         old_ack;

  // "The sector YOU asked for is in the buffer" -- not merely "some sector is".
  // This used to be a latch that was only ever cleared by a flush, so on the
  // second sector of a 1024 byte EOS block cv_adamnet saw a stale 1 left from
  // the previous sector and copied the WRONG sector out of the buffer into Adam
  // RAM. CP/M then wrote that back and clobbered the directory. Deriving the
  // flag from the buffer's identity makes the stale case impossible.
  assign disk_sector_loaded = buf_valid && (buf_sector == disk_sector);

  always_ff @(posedge clk) begin
    disk_flushed <= '0;

    if (disk_wr && disk_present) begin
      // First write after the buffer goes clean claims the buffer for whatever
      // sector AdamNet is filling now.
      if (!floppy_track_dirty) buf_sector <= disk_sector;
      floppy_track_dirty <= '1;
    end

    // If the disk is loaded, we capture the image size
    if (img_mounted) begin
      disk_size    <= img_size;
      disk_present <= |img_size;
      buf_valid    <= '0;   // whatever we were holding belongs to the old image
    end

    case (floppy_state)
      IDLE: begin
        if (disk_load) begin
          // We need to load a sector
          if (floppy_track_dirty) begin
            // The buffer is dirty, so flush it back to the sector it holds
            // before we overwrite it with a different one.
            $display("%x THIS SECTOR HAS CHANGES buf_sector %x sector %x",drive_num,buf_sector,disk_sector);
            floppy_track_dirty <= '0;
            lba_fdd            <= buf_sector;
            floppy_state       <= WRITE;
            sd_wr              <= 1;
          end else begin
            // Load the sector. buf_valid drops until the data actually lands,
            // so AdamNet cannot consume the previous sector by mistake.
            $display("%x READ NEW SECTOR sector %x",drive_num,disk_sector);
            curr_sector  <= disk_sector;
            buf_sector   <= disk_sector;
            buf_valid    <= '0;
            lba_fdd      <= disk_sector; // base of 512 byte address
            floppy_state <= READ;
            sd_rd        <= 1;
          end
        end else if (disk_flush) begin
          if (floppy_track_dirty) begin
            // Write the buffer back to the sector it actually holds
            $display("%x FLUSH CURR SECTOR buf_sector %x sector %x",drive_num,buf_sector,disk_sector);
            floppy_track_dirty <= '0;
            lba_fdd            <= buf_sector;
            floppy_state       <= WRITE;
            sd_wr              <= 1;
          end // if (floppy_track_dirty)
        end
      end // case: IDLE
      // Completion is the FALLING EDGE OF sd_ack, not &sd_buff_addr.
      //
      // sd_buff_addr is one bus shared by every drive, hps_io leaves it stuck at
      // 511 after a transfer, and only rezeroes it on its next command. So on
      // entering READ it is routinely ALREADY 511 from somebody else's transfer:
      // the read "completes" instantly, before the HPS has moved a single byte,
      // and AdamNet then copies the PREVIOUS sector out of the buffer. That is
      // the directory corruption -- the SD read does happen, just too late.
      // sd_ack is per-drive and is the interface's real completion signal.
      //
      // The simulator never showed this because sim_blkdevice zeroes sd_buff_addr the
      // moment it sees sd_rd/sd_wr, so the core can never observe the stale 511.
      READ: begin
        if (~old_ack & sd_ack) begin
          sd_rd <= 0;                  // request accepted, drop the strobe
        end else if (old_ack & ~sd_ack) begin
          floppy_state <= W4IDLE_READ;
          buf_valid    <= '1;          // buffer now really holds buf_sector
        end
      end
      WRITE: begin
        if (~old_ack & sd_ack) begin
          sd_wr <= 0;                  // request accepted, drop the strobe
        end else if (old_ack & ~sd_ack) begin
          floppy_state <= W4IDLE_WRITE;
        end
      end
      W4IDLE_READ: begin
        if (~disk_load && ~disk_flush) begin
          disk_flushed <= '1;
          floppy_state <= IDLE;
        end
      end
      W4IDLE_WRITE: begin
        if (~disk_flush) begin
          disk_flushed <= '1;
          floppy_state <= IDLE;
        end
      end
    endcase // case (floppy_state)

    old_ack <= sd_ack;

    if (reset) begin
      floppy_track_dirty <= '0;
      floppy_state       <= IDLE;
      sd_rd              <= '0;
      sd_wr              <= '0;
      buf_valid          <= '0;
    end
  end

//`ifdef VERILATOR
bram #(8,9) floppy_dpram_onetrack
(
        .clock_a(clk),
        .address_a(sd_buff_addr),
        .wren_a(sd_buff_wr & sd_ack),
        .data_a(sd_buff_dout),
        .q_a(sd_buff_din),

        .clock_b(clk),
        .address_b(disk_addr),
        .wren_b(disk_wr),
        .data_b(disk_din),
        .q_b(disk_data)
);
/*
`else

dpram #(9,8) floppy_dpram
(
        .clock(clk),
        .address_a(sd_buff_addr),
        .wren_a(sd_buff_wr & sd_ack),
        .data_a(sd_buff_dout),
        .q_a(sd_buff_din),

        //.clock_b(clk),
        .address_b(disk_addr),
        .wren_b(disk_wr),
        .data_b(disk_din),
        .q_b(disk_data)

);
`endif
*/
endmodule
