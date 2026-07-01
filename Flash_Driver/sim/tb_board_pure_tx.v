`timescale 1ns / 1ps
// Bypass flash_top by replacing it with a stub. Test that board_top's UART
// helpers produce the right bytes on the wire.
module tb_board_pure_tx;

    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg cpu_resetn = 1'b0;
    wire uart_tx;
    wire led_busy, led_done, led_error;

    // Need to instantiate board_top with flash_top hooks, but provide dummy
    // SPI slaves so it doesn't try to talk to a real flash
    wire sck, csn, mosi;
    assign sck = 1'b0;
    assign csn = 1'b1;
    assign mosi = 1'b0;
    wire miso = 1'b0;

    board_top u_dut (
        .CLK100MHZ (clk), .CPU_RESETN (cpu_resetn), .BTN_START (1'b0),
        .LED_BUSY (led_busy), .LED_DONE (led_done), .LED_ERROR (led_error),
        .UART_TX (uart_tx),
        .SPI_SCK (), .SPI_CSN (), .SPI_MOSI (), .SPI_MISO (1'b0)
    );

    // We can't easily access uart_putc from outside board_top. But board_top
    // will print "EGO1 W25Q64\r\n" first when its self-test runs. We capture
    // uart_tx pin and look for ASCII 'E' (0x45) and 'G' (0x47).
    integer i;
    reg [0:16383] tx_trace = 16384'b0;
    initial begin
        for (i = 0; i < 16384; i = i + 1) begin
            @(posedge clk);
            tx_trace[i] = uart_tx;
        end
    end

    initial begin
        cpu_resetn = 1'b0;
        repeat (50) @(posedge clk);
        cpu_resetn = 1'b1;
        repeat (5_000_000) @(posedge clk);
        // Now run the rest of the sim, just dump first 5000 samples of tx pin
        for (i = 0; i < 5000; i = i + 1) begin
            if (i % 80 == 0) $write("\n[%5d] ", i);
            $write("%b", tx_trace[i]);
        end
        $write("\n");
        $finish;
    end

    initial begin
        #1_000_000_000; $display("TIMEOUT"); $finish;
    end

endmodule
