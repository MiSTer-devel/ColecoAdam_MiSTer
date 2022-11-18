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
   output logic        disk_error, // out of bounds (?)
   input [7:0]         disk_din,
   output logic [7:0]  disk_data
   );

  enum bit [1:0] {
                  IDLE,
                  READ,
                  WRITE,
                  W4IDLE
                  } floppy_state;

  // when we write to the disk, we need to mark it dirty
  logic                         floppy_track_dirty;

  logic [63:0]                  disk_size; // Size of disk loaded in bytes
  logic [31:0]                  curr_sector;
  logic                         old_ack;

  always_ff @(posedge clk) begin

    if (disk_wr && disk_present) floppy_track_dirty <= '1;

    // If the disk is loaded, we capture the image size
    if (img_mounted) begin
      disk_size    <= img_size;
      disk_present <= |img_size;
    end

    case (floppy_state)
      IDLE: begin
        if (disk_load) begin
          // We need to load a sector
          if (floppy_track_dirty) begin
            // The current sector is dirty, so we need to flush before reading
            $display("%x THIS SECTOR HAS CHANGES curr_sector %x sector %x",drive_num,curr_sector,disk_sector);
            disk_sector_loaded <= '0;
            floppy_track_dirty <= '0;
            //lba_fdd            <= {curr_sector, 9'b0}; // base of 512 byte address
            lba_fdd            <= curr_sector; // base of 512 byte address
            floppy_state       <= WRITE;
            sd_wr              <= 1;
          end else begin
            // Load the sector
            $display("%x READ NEW SECTOR sector %x",drive_num,disk_sector);
            curr_sector  <= disk_sector;
            //lba_fdd      <= {disk_sector, 9'b0}; // base of 512 byte address
            lba_fdd      <= disk_sector; // base of 512 byte address
            floppy_state <= READ;
            sd_rd        <= 1;
          end
        end else if (disk_flush) begin
          disk_sector_loaded <= '0;
          if (floppy_track_dirty) begin
            // Write the current sector
            $display("%x FLUSH CURR SECTOR sector %x",drive_num,disk_sector);
            disk_sector_loaded <= '0;
            floppy_track_dirty <= '0;
            //lba_fdd            <= {disk_sector, 9'b0}; // base of 512 byte address
            lba_fdd            <= disk_sector; // base of 512 byte address
            floppy_state       <= WRITE;
            sd_wr              <= 1;
          end // if (floppy_track_dirty)
        end
      end // case: IDLE
      READ: begin
        if (&sd_buff_addr) begin
          sd_rd              <= 0;
          lba_fdd            <= lba_fdd + 1'd1;
          floppy_state       <= W4IDLE;
          disk_sector_loaded <= '1;
        end
        /*
        if (~old_ack & sd_ack) begin
          if (&sd_buff_addr) sd_rd <= 0;
          lba_fdd <= lba_fdd + 1'd1;
        end else if (old_ack & ~sd_ack) begin
          if(~sd_rd) begin
            floppy_state <= IDLE;
            disk_sector_loaded <= '1;
          end
        end
         */
      end
      WRITE: begin
        $display("Disk Writing %x", sd_buff_addr);
        if (&sd_buff_addr) begin
          floppy_track_dirty <= '0;
          sd_wr              <= 0;
          lba_fdd            <= lba_fdd + 1'd1;
          floppy_state       <= W4IDLE;
        end
        /*
        if (~old_ack & sd_ack) begin
          if (&sd_buff_addr) sd_wr <= 0;
          lba_fdd <= lba_fdd + 1'd1;
        end else if (old_ack & ~sd_ack) begin
          if (~sd_wr) floppy_state <= IDLE;
        end
         */
      end
      W4IDLE: begin
        if (~disk_load && ~disk_flush) begin
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
    end
  end

`ifdef VERILATOR
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

endmodule
