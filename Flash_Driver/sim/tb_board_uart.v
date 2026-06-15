`timescale 1ns / 1ps
//=============================================================================
// Module : tb_board_uart
// Purpose: Decodes board_top's UART output by sampling at the center of each
//          bit period (16x oversampling would be nicer; we just sample at
//          BAUD_TICKS/2 from the falling edge of start).
//=============================================================================
module tb_board_uart;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg  cpu_resetn = 1'b0;
    reg  btn_start  = 1'b0;
    wire led_busy, led_done, led_error;
    wire uart_tx, sck, csn, mosi, miso;

    board_top u_dut (
        .CLK100MHZ (clk),
        .CPU_RESETN (cpu_resetn),
        .BTN_START  (btn_start),
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

    // UART RX: 16x oversample on the bit period.
    localparam integer CLK_HZ    = 100_000_000;
    localparam integer BAUD      = 115_200;
    localparam integer BIT_TICKS = CLK_HZ / BAUD;   // 868
    localparam integer HALF_BIT  = BIT_TICKS / 2;

    reg [9:0] rx_sh = 10'h3FF;
    reg [3:0] rx_bit_idx = 4'd0;
    reg [15:0] rx_tick_cnt = 16'd0;
    reg rx_active = 1'b0;
    reg rx_prev = 1'b1;

    always @(posedge clk) begin
        if (!rx_active) begin
            rx_tick_cnt <= 16'd0;
            rx_bit_idx  <= 4'd0;
            // Detect falling edge of start bit
            if (rx_prev == 1'b1 && uart_tx == 1'b0) begin
                rx_active <= 1'b1;
                rx_tick_cnt <= 16'd0;
                rx_bit_idx  <= 4'd0;
            end
        end else begin
            if (rx_tick_cnt == BIT_TICKS - 1) begin
                rx_tick_cnt <= 16'd0;
                rx_sh <= {uart_tx, rx_sh[9:1]};
                if (rx_bit_idx == 4'd8) begin
                    rx_active <= 1'b0;
                    // Bits 0..7 of the byte (LSB first)
                    $write("%c", {rx_sh[8:1], uart_tx});
                    $fflush();
                end else begin
                    rx_bit_idx <= rx_bit_idx + 1'b1;
                end
            end else begin
                rx_tick_cnt <= rx_tick_cnt + 1'b1;
            end
        end
        rx_prev <= uart_tx;
    end

    initial begin
        cpu_resetn = 1'b0;
        repeat (50) @(posedge clk);
        cpu_resetn = 1'b1;
        repeat (6_000_000) @(posedge clk);
        // (Self-test auto-runs; just wait for done)
        wait (led_done || led_error);
        repeat (500_000) @(posedge clk);
        $display("\n[final] DONE=%b ERROR=%b", led_done, led_error);
        $finish;
    end

    initial begin
        #5_000_000_000; $display("TIMEOUT"); $finish;
    end

endmodule
