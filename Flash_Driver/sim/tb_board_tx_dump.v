`timescale 1ns / 1ps
module tb_board_tx_dump;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg  cpu_resetn = 1'b0;
    wire led_busy, led_done, led_error;
    wire uart_tx, sck, csn, mosi, miso;

    board_top u_dut (
        .CLK100MHZ (clk),
        .CPU_RESETN (cpu_resetn),
        .BTN_START  (1'b0),
        .LED_BUSY   (led_busy),
        .LED_DONE   (led_done),
        .LED_ERROR  (led_error),
        .UART_TX    (uart_tx),
        .SPI_SCK    (sck),
        .SPI_CSN    (csn),
        .SPI_MOSI   (mosi),
        .SPI_MISO   (miso)
    );

    flash_model #(.MEM_DEPTH(64*1024)) u_flash (
        .sck (sck), .csn (csn), .mosi (mosi), .miso (miso)
    );

    localparam integer BIT_TICKS = 868;
    localparam integer MID_BIT   = 434;
    reg [9:0]  rx_sh      = 10'h3FF;
    reg [3:0]  rx_bit_idx = 4'd0;
    reg [15:0] rx_tick    = 16'd0;
    reg        rx_active  = 1'b0;
    reg        rx_prev    = 1'b1;

    always @(posedge clk) begin
        if (!rx_active) begin
            rx_tick    <= 16'd0;
            rx_bit_idx <= 4'd0;
            if (rx_prev && !uart_tx) begin
                rx_active  <= 1'b1;
                rx_tick    <= 16'd0;   // count up from this edge
            end
        end else if (rx_tick == MID_BIT) begin
            // Sample at the center of the current bit
            rx_tick <= 16'd0;
            if (rx_bit_idx == 0) begin
                // start bit - just verify it's 0
                if (uart_tx) rx_active <= 1'b0;
                else         rx_bit_idx <= 4'd1;
            end else if (rx_bit_idx <= 8) begin
                rx_sh      <= {uart_tx, rx_sh[9:1]};
                rx_bit_idx <= rx_bit_idx + 1'b1;
            end else begin
                rx_active <= 1'b0;
                $display("[%0t] RX byte = 0x%02h '%c' (stop=%b)", $time,
                         rx_sh[8:1], rx_sh[8:1], uart_tx);
            end
        end else begin
            rx_tick <= rx_tick + 1'b1;
        end
        rx_prev <= uart_tx;
    end

    initial begin
        cpu_resetn = 1'b0;
        repeat (50) @(posedge clk);
        cpu_resetn = 1'b1;
        wait (led_done || led_error);
        repeat (1_000_000) @(posedge clk);
        $display("[done] DONE=%b ERROR=%b", led_done, led_error);
        $finish;
    end
    initial begin
        #5_000_000_000; $display("TIMEOUT"); $finish;
    end

endmodule
