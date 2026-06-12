//=============================================================================
// Module     : uart_tx
// Description: Minimal 8N1 UART transmitter, no FIFO.
//
//   IDLE : tx held high, waiting for tx_start
//   START: drive low for one bit period
//   DATA : shift out 8 bits, LSB first, one bit period each
//   STOP : drive high for one bit period
//   back to IDLE
//
// Baud rate derived from CLK_FREQ.  At 100 MHz / 19200 baud, each bit is
// 5208 clocks (actual baud = 19201.2, error 0.006% — well within UART tolerance).
//
// Usage:
//   // Single byte:
//   pulse tx_start <= 1; tx_data <= 8'h55;  // one cycle
//   wait while (tx_busy);
//
//   // Back-to-back:
//   pulse tx_start whenever !tx_busy.
//=============================================================================

`timescale 1ns / 1ps

module uart_tx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 19200
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       tx_start,
    input  wire [7:0] tx_data,

    output reg        tx,
    output wire       tx_busy
);

    localparam integer CYCLES_PER_BIT = CLK_FREQ / BAUD_RATE;

    reg [15:0] cycle_cnt;     // counts up to CYCLES_PER_BIT-1
    reg [3:0]  bit_cnt;       // 0=start, 1..8=data, 9=stop
    reg [7:0]  shift_reg;
    reg        busy;

    assign tx_busy = busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx         <= 1'b1;
            cycle_cnt  <= 16'd0;
            bit_cnt    <= 4'd0;
            shift_reg  <= 8'd0;
            busy       <= 1'b0;
        end else begin
            if (tx_start && !busy) begin
                // Latch data, start the frame
                shift_reg <= tx_data;
                bit_cnt   <= 4'd0;
                cycle_cnt <= 16'd0;
                busy      <= 1'b1;
                tx        <= 1'b0;     // start bit
            end else if (busy) begin
                cycle_cnt <= cycle_cnt + 16'd1;
                if (cycle_cnt == CYCLES_PER_BIT - 1) begin
                    cycle_cnt <= 16'd0;
                    if (bit_cnt == 4'd8) begin
                        tx <= 1'b1;    // stop bit
                        bit_cnt <= 4'd9;
                    end else if (bit_cnt == 4'd9) begin
                        // Frame done, return to idle
                        busy <= 1'b0;
                        bit_cnt <= 4'd0;
                    end else begin
                        // Shift out next LSB
                        tx <= shift_reg[0];
                        shift_reg <= {1'b0, shift_reg[7:1]};
                        bit_cnt <= bit_cnt + 4'd1;
                    end
                end
            end
        end
    end

endmodule
