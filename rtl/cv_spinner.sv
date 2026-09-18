//-----------------------------------------------------------------------------
//
// Spinner / roller controller model
//
// Drives one controller port's pins 7 and 9 the way Coleco's optical controllers
// do, so that cv_ctrl and the cartridge software see a real roller:
//
//   pin 7 -> D5   the direction of travel, held for the whole strobe
//   pin 9 -> D4   one low strobe per step, which also raises the Z80's
//                 maskable interrupt in cv_ctrl
//
// The hardware being modelled is the photo coupler of Expansion Module #2
// (Technical Guide, Theory of Operation III-1 and schematic V-1): two LEDs, a
// slotted disc and two photo transistors feeding Schmitt trigger buffers, which
// come out of the module on connector pins 7 and 9. The Roller Controller uses
// the same arrangement - the Lundy service bulletin names its optocouplers,
// originally a proprietary Linear LTM72564 - and so does the Super Action
// Controller's speed roller. The ADAM Technical Manual gives the timing for the
// pin 9 side: "Strobe signal: typical 350 usec pulse width".
//
// A Roller Controller is wired to both ports at once, X on controller 1 and Y on
// controller 2, so one of these is instantiated per port.
//
// Steps come from either of two sources, which may be used together:
//   step_en_i  a discrete step count, e.g. from a MiSTer spinner device
//   rate_i     a velocity, e.g. an analog stick axis; |rate_i| sets the rate at
//              which steps are produced, matching MAME's sensitivity of
//              |rate| * 2 steps per second (coleco.cpp, paddle_update_callback)
//
// Steps are accumulated as a net signed count, so reversing direction cancels
// queued steps instead of queueing them behind the old direction. That gives the
// software's pulse counter the same final value either way, since it adds 1 per
// strobe in one direction and subtracts 1 in the other.
//
//-----------------------------------------------------------------------------

module cv_spinner
  #(
    // all times are in clk_en_i ticks, which the tops drive at 10.7 MHz
    parameter logic [12:0] STROBE_TICKS = 13'd3758,    // 350 us, per the ATM
    parameter logic [12:0] GAP_TICKS    = 13'd1074,    // 100 us idle between
    parameter logic [22:0] RATE_UNIT    = 23'd5369318  // 0.5 s: |rate| * 2 Hz
    )
  (
   input               clk_i,
   input               clk_en_i,      // 10.7 MHz clock enable
   input               reset_n_i,

   input               step_en_i,     // one clk_en_i tick wide
   input               step_dir_i,    // 1 = count up (D5 = 1)
   input [4:0]         step_cnt_i,    // steps to queue with step_en_i

   input signed [7:0]  rate_i,        // velocity, 0 = idle

   output logic        p7_o,          // D5, direction of travel
   output logic        p9_o           // D4 and /INT strobe, active low
   );

  logic signed [6:0] pending;         // net steps still to emit, +-31
  logic       [22:0] acc;             // rate accumulator
  logic       [12:0] timer;
  logic              strobing;
  logic              emit_up;         // direction of the strobe in flight
  logic              nco_step;

  // magnitude of the velocity
  wire         [7:0] rate_abs  = rate_i[7] ? (~rate_i + 8'd1) : rate_i;

  wire        [23:0] acc_sum   = {1'b0, acc} + {16'd0, rate_abs};
  wire               acc_wrap  = (acc_sum >= {1'b0, RATE_UNIT});
  wire        [23:0] acc_next  = acc_wrap ? (acc_sum - {1'b0, RATE_UNIT}) : acc_sum;

  // steps arriving this tick, from either source
  wire signed  [7:0] step_delta = !step_en_i  ? 8'sd0 :
                                  step_dir_i  ?  $signed({3'b000, step_cnt_i}) :
                                                -$signed({3'b000, step_cnt_i});
  wire signed  [7:0] nco_delta  = !nco_step   ? 8'sd0 :
                                  rate_i[7]   ? -8'sd1 : 8'sd1;
  wire signed  [7:0] delta      = step_delta + nco_delta;

  // the step consumed when a strobe completes
  wire signed  [7:0] consumed   = emit_up ? -8'sd1 : 8'sd1;

  function automatic logic signed [6:0] sat_add
    (
     input logic signed [6:0] a,
     input logic signed [7:0] b
     );
    logic signed [8:0] s;
    begin
      s = $signed({{2{a[6]}}, a}) + $signed({b[7], b});
      if      (s >  9'sd31) sat_add =  7'sd31;
      else if (s < -9'sd31) sat_add = -7'sd31;
      else                  sat_add =  s[6:0];
    end
  endfunction

  always_ff @(posedge clk_i, negedge reset_n_i) begin : gen
    if (~reset_n_i) begin
      pending  <= 7'sd0;
      acc      <= '0;
      timer    <= '0;
      strobing <= 1'b0;
      emit_up  <= 1'b1;
      nco_step <= 1'b0;
      p7_o     <= 1'b1;
      p9_o     <= 1'b1;
    end else if (clk_en_i) begin
      // velocity to steps
      nco_step <= 1'b0;
      if (rate_i == 8'sd0) begin
        acc <= '0;
      end else begin
        acc <= acc_next[22:0];
        if (acc_wrap) nco_step <= 1'b1;
      end

      if (timer != '0) begin
        // inside a strobe or the gap after it
        timer   <= timer - 1'b1;
        pending <= sat_add(pending, delta);
      end else if (strobing) begin
        // strobe done: release pin 9, take the gap, and consume the step
        p9_o     <= 1'b1;
        strobing <= 1'b0;
        timer    <= GAP_TICKS;
        pending  <= sat_add(pending, delta + consumed);
      end else if (pending != 7'sd0) begin
        // start the next strobe in whichever direction is outstanding
        emit_up  <= (pending > 7'sd0);
        p7_o     <= (pending > 7'sd0);
        p9_o     <= 1'b0;
        strobing <= 1'b1;
        timer    <= STROBE_TICKS;
        pending  <= sat_add(pending, delta);
      end else begin
        pending  <= sat_add(pending, delta);
      end
    end
  end

endmodule
