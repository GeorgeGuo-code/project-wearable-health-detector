`timescale 1ns / 1ps
module tb_board_uart_single;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    // Minimal: just one uart_putc call via board_top's UART_TX path
    reg  cpu_resetn = 1'b0;
    wire uart_tx;
    wire sck, csn, mosi, miso;
    wire led_busy, led_done, led_error;

    // Replace flash_top with stubs to keep sim fast and deterministic
    reg [7:0]  rdata_q = 8'h00;
    reg        rdata_v_q = 1'b0;
    reg        busy_q = 1'b0;
    reg        done_q = 1'b0;
    reg        error_q = 1'b0;
    reg        wr_q = 1'b0;
    reg [7:0]  sr1_q = 8'h00;

    // Hook up board_top with a stub flash: tie sck=0, csn=1, mosi=0, miso=0
    board_top u_dut (
        .CLK100MHZ (clk),
        .CPU_RESETN (cpu_resetn),
        .BTN_START  (1'b0),
        .LED_BUSY   (led_busy),
        .LED_DONE   (led_done),
        .LED_ERROR  (led_error),
        .UART_TX    (uart_tx),
        .SPI_SCK    (),
        .SPI_CSN    (),
        .SPI_MOSI   (),
        .SPI_MISO   (1'b0)
    );

    // Override internal signals to make controller's tasks return done immediately
    // (so the test prints the header string and we can inspect its UART output)
    // - We need to hack into board_top... too complex.
    // Simpler: instantiate uart_tx directly and feed it one character to verify
    // the line discipline. Then trust the printout of header later.
    wire uart_tx2;
    reg  uart_start2 = 0;
    reg [7:0] uart_data2 = 0;
    uart_tx #(.CLK_HZ(100_000_000), .BAUD(115_200)) u_uart2 (
        .clk(clk), .rst_n(cpu_resetn), .start(uart_start2),
        .data(uart_data2), .tx(uart_tx2), .busy()
    );

    // Transmit one byte
    initial begin
        cpu_resetn = 1'b0;
        repeat (20) @(posedge clk);
        cpu_resetn = 1'b1;
        repeat (20) @(posedge clk);
        uart_data2 = 8'h45;  // 'E'
        uart_start2 = 1;
        @(posedge clk);
        uart_start2 = 0;
        // Wait long enough for full transmission
        repeat (10_000) @(posedge clk);
        $finish;
    end

    // Sample uart_tx2 at every clock
    integer i;
    reg [0:8191] tx_trace = 8192'b0;
    initial begin
        for (i = 0; i < 8192; i = i + 1) begin
            @(posedge clk);
            tx_trace[i] = uart_tx2;
        end
    end

    // After the run, dump first 2000 bits after start
    initial begin
        wait (cpu_resetn == 1);
        // Wait for first falling edge of uart_tx2
        while (uart_tx2 == 1'b1) @(posedge clk);
        $display("Start bit falling edge at t=%0t", $time);
        repeat (10_000) @(posedge clk);
        $display("Last 100 samples of uart_tx2:");
        for (i = 0; i < 8192; i = i + 1) begin
            if (i % 100 == 0) $write("\n[%4d] ", i);
            $write("%b", tx_trace[i]);
        end
        $write("\n");
        $finish;
    end

endmodule
