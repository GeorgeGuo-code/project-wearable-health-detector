`timescale 1ns / 1ps
//=============================================================================
// Module : uart_tx
// Purpose: Simple 8N1 UART transmitter. One-cycle `start` pulse with a byte
//          on `data` begins transmission at `BAUD` ticks per bit. `busy` is
//          high while the byte is shifting out.
//=============================================================================
module uart_tx #(
    parameter CLK_HZ  = 100_000_000,
    parameter BAUD    = 115_200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,      // one-cycle pulse to begin
    input  wire [7:0] data,
    output reg        tx,         // serial line (idle high)
    output reg        busy
);

    localparam integer DIV = CLK_HZ / BAUD;  // 868 @ 100 MHz, 115200 baud
    localparam integer CW  = $clog2(DIV);

    reg [CW-1:0] divcnt;
    reg [3:0]    bit_idx;     // 0..9 (start + 8 data + stop)
    reg [7:0]    shreg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx     <= 1'b1;
            busy   <= 1'b0;
            divcnt <= {CW{1'b0}};
            bit_idx<= 4'd0;
            shreg  <= 8'd0;
        end else begin
            if (start) begin
                busy   <= 1'b1;
                tx     <= 1'b0;          // start bit
                shreg  <= data;
                divcnt <= {CW{1'b0}};
                bit_idx<= 4'd0;
            end else if (busy) begin
                if (divcnt == DIV - 1) begin
                    divcnt <= {CW{1'b0}};
                    if (bit_idx == 4'd8) begin
                        // stop bit
                        tx     <= 1'b1;
                        busy   <= 1'b0;
                        bit_idx<= 4'd0;
                    end else begin
                        bit_idx<= bit_idx + 1'b1;
                        // data bit 0 = LSB first
                        tx     <= shreg[0];
                        shreg  <= {1'b0, shreg[7:1]};
                    end
                end else begin
                    divcnt <= divcnt + 1'b1;
                end
            end
        end
    end

endmodule
