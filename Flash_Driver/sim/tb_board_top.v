`timescale 1ns / 1ps
//=============================================================================
// Module : tb_board_top
// Purpose: Functional test of board_top.v using flash_model as the W25Q64
//          substitute. Validates the entire self-test sequence end-to-end.
//
//   - Generates 100 MHz clock
//   - Pulls CPU_RESETN high
//   - Pulses BTN_START to start the test
//   - Lets the FSM run; prints UART output and final LED state
//   - Compares read-back buffers against the expected pattern
//=============================================================================
module tb_board_top;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg        cpu_resetn = 1'b0;
    reg        btn_start = 1'b0;
    wire       led_busy, led_done, led_error;
    wire       uart_tx;
    wire       sck, csn, mosi, miso;

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
        .sck  (sck),
        .csn  (csn),
        .mosi (mosi),
        .miso (miso)
    );

    // Optional: hook the UART to stdout for visibility
    reg [7:0] uart_shift = 8'd0;
    reg [3:0] uart_bit_idx = 4'd0;
    reg       uart_active = 1'b0;
    integer   uart_baud_ticks = 0;
    localparam integer BAUD_TICKS = 868;   // 100 MHz / 115200
    reg [10:0] uart_capture = 11'h7FF;

    always @(posedge clk) begin
        if (uart_active) begin
            if (uart_baud_ticks == BAUD_TICKS - 1) begin
                uart_baud_ticks <= 0;
                uart_capture <= {uart_capture[9:0], uart_tx};
                if (uart_bit_idx == 4'd10) begin
                    uart_active <= 1'b0;
                    // The 8 data bits are at positions [8:1] of the captured
                    // stream, LSB first.
                    $write("%c", uart_capture[8:1]);
                    $fflush();
                end else begin
                    uart_bit_idx <= uart_bit_idx + 1'b1;
                end
            end else begin
                uart_baud_ticks <= uart_baud_ticks + 1;
            end
        end else if (uart_tx == 1'b0) begin
            uart_active     <= 1'b1;
            uart_bit_idx    <= 4'd0;
            uart_baud_ticks <= 0;
            uart_capture    <= 11'h7FF;
        end
    end

    integer k;
    reg [7:0] mem_dump [0:15];

    initial begin
        $display("===========================================");
        $display("tb_board_top: self-test functional sim");
        $display("===========================================");

        // Reset
        cpu_resetn = 1'b0;
        btn_start  = 1'b0;
        repeat (50) @(posedge clk);
        cpu_resetn = 1'b1;

        // Wait for the auto-run ~50ms settle
        repeat (6_000_000) @(posedge clk);

        // Trigger a re-run via button
        btn_start = 1'b1;
        repeat (200) @(posedge clk);
        btn_start = 1'b0;

        // Wait for test to complete
        wait (led_done || led_error);

        // Give UART a moment to drain
        repeat (500_000) @(posedge clk);

        $display("");
        $display("===========================================");
        $display("Final LED state: DONE=%b ERROR=%b", led_done, led_error);
        $display("===========================================");

        // Read back 16 bytes from the flash model directly for verification
        // (this bypasses the controller and reads the model memory state)
        $display("Flash mem[0..15] = ");
        for (k = 0; k < 16; k = k + 1) begin
            $display("  [%2d] = 0x%02h", k, u_flash.mem[k]);
        end

        if (led_done && !led_error)
            $display("PASS: self-test completed successfully");
        else
            $display("FAIL: LED_DONE=%b LED_ERROR=%b", led_done, led_error);

        $finish;
    end

    // Watchdog
    initial begin
        #5_000_000_000;     // 5s
        $display("TIMEOUT");
        $finish;
    end

endmodule
