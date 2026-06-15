`timescale 1ns / 1ps
// Test the synthesizable board_top: verify LED_DONE asserts after the test
// completes successfully.
module tb_board_synth;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg  cpu_resetn = 1'b0;     // ACTIVE-HIGH reset button: idle=0
    reg  btn_start  = 1'b0;
    wire led_busy, led_done, led_error;
    wire sck, csn, mosi, miso;

    // We don't connect UART_TX since we removed it from the design
    board_top u_dut (
        .CLK100MHZ (clk),
        .CPU_RESETN (cpu_resetn),
        .BTN_START  (btn_start),
        .LED_BUSY   (led_busy),
        .LED_DONE   (led_done),
        .LED_ERROR  (led_error),
        .SPI_SCK    (sck),
        .SPI_CSN    (csn),
        .SPI_MOSI   (mosi),
        .SPI_MISO   (miso)
    );

    flash_model #(.MEM_DEPTH(64*1024)) u_flash (
        .sck (sck), .csn (csn), .mosi (mosi), .miso (miso)
    );

    initial begin
        $display("===========================================");
        $display("tb_board_synth: synthesizable board_top");
        $display("===========================================");
        // Reset button is active-HIGH: idle=0 (not pressed, design runs).
        cpu_resetn = 1'b0;
        btn_start  = 1'b0;
        repeat (20) @(posedge clk);   // hold reset
        cpu_resetn = 1'b0;            // release (still 0 = not pressed)
        repeat (20) @(posedge clk);
        // Periodically print FSM state so we can see if it's stuck
        fork
            begin
                forever begin
                    #1_000_000;  // every 10us
                    $display("t=%0t stage=%0d ss=%0d busy=%b fc_st=%h sm_st=%h wdrd=%b wdv=%b sck=%b csn=%b",
                             $time, u_dut.stage, u_dut.ss,
                             u_dut.busy,
                             u_dut.u_flash.u_flash_ctrl.state,
                             u_dut.u_flash.u_spi_master.state,
                             u_dut.wdata_ready, u_dut.wdata_valid,
                             u_dut.SPI_SCK, u_dut.SPI_CSN);
                end
            end
            begin
                // Wait for the ~50ms settle then the entire test
                wait (led_done || led_error);
                disable fork;
                repeat (500) @(posedge clk);
                $display("");
                $display("===========================================");
                $display("Final LED state: DONE=%b ERROR=%b BUSY=%b",
                         led_done, led_error, led_busy);
                $display("===========================================");
                if (led_done && !led_error)
                    $display("PASS: self-test completed");
                else
                    $display("FAIL: LED_DONE=%b LED_ERROR=%b", led_done, led_error);
                $finish;
            end
        join
    end

    initial begin
        #2_000_000_000; $display("TIMEOUT"); $finish;
    end
endmodule
