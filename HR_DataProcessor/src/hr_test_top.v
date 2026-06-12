//=============================================================================
// Module     : hr_test_top
// Description: Test-only top for the FPGA-side HR pipeline. Synthesize this
//              in place of max30102_top.v to validate IR readout, the ppg_filter
//              output, and the sDFT+median HR computation on real hardware.
//
// Signal chain:
//   start -> MAX30102 --I2C--> max30102_driver -> ppg_filter -> HR
//                                                  |             |
//                                                  v             v
//                                              UART TX (13B/frame @ 100 Hz)
//
// Test frame layout (16 bytes, sent at ~25 Hz on maxim_dec_valid):
//   [0]    0xAA                          sync
//   [1]    0x55                          sync
//   [2]    0x01                          frame type (PPG)
//   [3-5]  hamm[17:0]                    Maxim filter output (LSB first)
//   [6-8]  thresh[17:0]                  EMA threshold
//   [9]    HR_tens                       ASCII '0'..'9', or 0xFF before first
//   [10]   HR_ones                       ASCII '0'..'9', or 0xFF
//   [11]   status                        bit 0 = hr_locked, bit 1 = hr_valid
//   [12-14] ir_raw[17:0]                 Raw IR from MAX30102 (diagnostic)
//   [15]   reserved
//
// The IR_DC / IR_AC fields keep the same byte positions as the production
// 9-byte frame (max30102_top.v) so the existing parse_hr.py can read them.
// The 4 trailing bytes are diagnostic only.
//
// Expected sequence after power-on:
//   - First ~2.5 s: IR frames, HR_tens=HR_ones=0xFF (sDFT still filling)
//   - Then: HR_tens=HR_ones=0xFF for ~7.7 s more (sDFT running, median filling)
//   - After ~10 s: HR digits appear, hr_valid pulses once per ~7.7 s when locked
//   - If sensor falls off or no signal: HR=0xFF 0xFF, hr_locked=0
//
// External pins:
//   uart_tx  -> USB-UART RX
//   led_init -> on once MAX30102 init completes
//   led_tx   -> on while a frame is mid-UART
//   led_hr   -> on when hr_locked=1 (heart beat "valid" indicator)
//=============================================================================

`timescale 1ns / 1ps

module hr_test_top #
(
    parameter CLK_FREQ = 100_000_000,
    parameter BAUD_RATE = 19_200
)
(
    input  wire       clk,
    input  wire       rst_n_raw,
    input  wire       start,

    // I2C
    output wire       scl,
    inout  wire       sda,

    // Status LEDs
    output wire       led_init,
    output wire       led_tx,
    output wire       led_hr,

    // UART output
    output wire       uart_tx
);

    wire rst_n = ~rst_n_raw;

    // ------------------------------------------------------------------------
    // MAX30102 driver
    // ------------------------------------------------------------------------
    wire [17:0] ir_raw, red_raw;
    wire        data_valid;
    wire        init_done;

    max30102_driver u_driver (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .scl        (scl),
        .sda        (sda),
        .ir_data    (ir_raw),
        .red_data   (red_raw),
        .data_valid (data_valid),
        .init_done  (init_done),
        .sda_link   (),
        .sda_r      (),
        .state      ()
    );

    // ------------------------------------------------------------------------
    // HR pipeline: hr_maxim (Maxim-style, raw IR direct)
    // ------------------------------------------------------------------------
    wire [7:0]  hr_bpm_bin;
    wire        hr_locked;
    wire        hr_valid;
    wire signed [17:0] maxim_hamm, maxim_thresh;
    wire maxim_dec_valid;

    hr_maxim u_hr_maxim (
        .clk            (clk),
        .rst_n          (rst_n),
        .in_valid       (data_valid),
        .ir_ac          (ir_raw),
        .hr_bpm         (hr_bpm_bin),
        .hr_valid       (hr_valid),
        .hr_locked      (hr_locked),
        .hamm_out       (maxim_hamm),
        .threshold_out  (maxim_thresh),
        .dec_valid      (maxim_dec_valid)
    );

    // BCD-to-ASCII for HR display
    reg [3:0] bcd_tens, bcd_ones;
    always @(*) begin
        case (1'b1)
            (hr_bpm_bin >= 8'd100): begin bcd_tens = 4'd1; bcd_ones = hr_bpm_bin - 8'd100; end
            (hr_bpm_bin >= 8'd90 ): begin bcd_tens = 4'd9; bcd_ones = hr_bpm_bin - 8'd90;  end
            (hr_bpm_bin >= 8'd80 ): begin bcd_tens = 4'd8; bcd_ones = hr_bpm_bin - 8'd80;  end
            (hr_bpm_bin >= 8'd70 ): begin bcd_tens = 4'd7; bcd_ones = hr_bpm_bin - 8'd70;  end
            (hr_bpm_bin >= 8'd60 ): begin bcd_tens = 4'd6; bcd_ones = hr_bpm_bin - 8'd60;  end
            (hr_bpm_bin >= 8'd50 ): begin bcd_tens = 4'd5; bcd_ones = hr_bpm_bin - 8'd50;  end
            (hr_bpm_bin >= 8'd40 ): begin bcd_tens = 4'd4; bcd_ones = hr_bpm_bin - 8'd40;  end
            (hr_bpm_bin >= 8'd30 ): begin bcd_tens = 4'd3; bcd_ones = hr_bpm_bin - 8'd30;  end
            (hr_bpm_bin >= 8'd20 ): begin bcd_tens = 4'd2; bcd_ones = hr_bpm_bin - 8'd20;  end
            (hr_bpm_bin >= 8'd10 ): begin bcd_tens = 4'd1; bcd_ones = hr_bpm_bin - 8'd10;  end
            default:                begin bcd_tens = 4'd0; bcd_ones = hr_bpm_bin;           end
        endcase
    end

    wire [7:0] hr_tens_ascii = {4'h3, bcd_tens};
    wire [7:0] hr_ones_ascii = {4'h3, bcd_ones};

    // ------------------------------------------------------------------------
    // UART TX
    // ------------------------------------------------------------------------
    wire       tx_busy;
    reg        tx_start;
    reg  [7:0] tx_data;

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_uart (
        .clk       (clk),
        .rst_n     (rst_n),
        .tx_start  (tx_start),
        .tx_data   (tx_data),
        .tx        (uart_tx),
        .tx_busy   (tx_busy)
    );

    // ------------------------------------------------------------------------
    // Frame builder: 13 bytes per filt_valid tick.
    //
    //   FB_IDLE      : wait for filt_valid -> latch, go FB_PULSE
    //   FB_PULSE     : drive tx_data, pulse tx_start, go FB_WAIT_HI
    //   FB_WAIT_HI   : wait for tx_busy to RISE
    //   FB_WAIT_LO   : wait for tx_busy to FALL, advance byte_idx
    //
    // For HR digits we use 0xFF as a sentinel for "no reading yet" (cleaner
    // than 0x20 = space; the parse script can pattern-match it).
    // ------------------------------------------------------------------------
    parameter FB_IDLE    = 2'd0;
    parameter FB_PULSE   = 2'd1;
    parameter FB_WAIT_HI = 2'd2;
    parameter FB_WAIT_LO = 2'd3;

    reg  [1:0]  fb_state;
    reg  [4:0]  byte_idx;          // 0..18 (19-byte frame)
    reg         sending;
    reg signed [17:0] hamm_l, thresh_l;
    reg [17:0]  ir_raw_l;          // raw IR for diagnostics
    reg [17:0]  ir_prev_l;         // previous raw IR
    reg signed [18:0] ir_diff_l;   // raw IR difference (current - previous)
    reg  [7:0]  hr_tens_l, hr_ones_l;
    reg  [1:0]  hr_status_l;       // bit 0 = locked, bit 1 = valid (pulse)
    reg         hr_ever_read;      // 1 once the first HR reading has arrived

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fb_state      <= FB_IDLE;
            byte_idx      <= 4'd0;
            sending       <= 1'b0;
            tx_start      <= 1'b0;
            tx_data       <= 8'd0;
            hamm_l        <= 18'sd0;
            thresh_l      <= 18'sd0;
            ir_raw_l      <= 18'd0;
            ir_prev_l     <= 18'd0;
            ir_diff_l     <= 19'd0;
            hr_tens_l     <= 8'hFF;
            hr_ones_l     <= 8'hFF;
            hr_status_l   <= 2'b00;
            hr_ever_read  <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            // Latch HR digits on hr_valid pulse (merged from standalone
            // always block to avoid multi-driver on hr_tens_l/hr_ones_l).
            if (hr_valid) begin
                hr_tens_l    <= hr_tens_ascii;
                hr_ones_l    <= hr_ones_ascii;
                hr_ever_read <= 1'b1;
            end

            case (fb_state)
                FB_IDLE: begin
                    if (maxim_dec_valid) begin
                        hamm_l   <= maxim_hamm;
                        thresh_l <= maxim_thresh;
                        ir_raw_l <= ir_raw;       // latch raw IR for diagnostics
                        ir_diff_l <= $signed({1'b0, ir_raw}) - $signed({1'b0, ir_prev_l});
                        ir_prev_l <= ir_raw;
                        // HR status: capture lock + valid for byte 11
                        hr_status_l <= {hr_valid, hr_locked};
                        byte_idx <= 4'd0;
                        sending  <= 1'b1;
                        fb_state <= FB_PULSE;
                    end
                end

                FB_PULSE: begin
                    case (byte_idx)
                        5'd0:  tx_data <= 8'hAA;
                        5'd1:  tx_data <= 8'h55;
                        5'd2:  tx_data <= 8'h01;
                        5'd3:  tx_data <= hamm_l[7:0];
                        5'd4:  tx_data <= hamm_l[15:8];
                        5'd5:  tx_data <= {hamm_l[17:16], 6'd0};
                        5'd6:  tx_data <= thresh_l[7:0];
                        5'd7:  tx_data <= thresh_l[15:8];
                        5'd8:  tx_data <= {thresh_l[17:16], 6'd0};
                        5'd9:  tx_data <= hr_ever_read ? hr_tens_l : 8'hFF;
                        5'd10: tx_data <= hr_ever_read ? hr_ones_l : 8'hFF;
                        5'd11: tx_data <= {6'd0, hr_status_l};
                        5'd12: tx_data <= ir_raw_l[7:0];
                        5'd13: tx_data <= ir_raw_l[15:8];
                        5'd14: tx_data <= {ir_raw_l[17:16], 6'd0};
                        5'd15: tx_data <= ir_diff_l[7:0];
                        5'd16: tx_data <= ir_diff_l[15:8];
                        5'd17: tx_data <= {ir_diff_l[18:16], 5'd0};
                        5'd18: tx_data <= 8'd0;   // reserved
                    endcase
                    tx_start <= 1'b1;
                    fb_state <= FB_WAIT_HI;
                end

                FB_WAIT_HI: begin
                    if (tx_busy) fb_state <= FB_WAIT_LO;
                end

                FB_WAIT_LO: begin
                    if (!tx_busy) begin
                        if (byte_idx == 5'd18) begin
                            sending  <= 1'b0;
                            byte_idx <= 4'd0;
                            fb_state <= FB_IDLE;
                        end else begin
                            byte_idx <= byte_idx + 4'd1;
                            fb_state <= FB_PULSE;
                        end
                    end
                end
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Status LEDs
    // ------------------------------------------------------------------------
    assign led_init = init_done;
    assign led_tx   = sending;
    assign led_hr   = hr_locked;

endmodule
