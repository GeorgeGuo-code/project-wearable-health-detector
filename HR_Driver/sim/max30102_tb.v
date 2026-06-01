`timescale 1ns / 1ps

module max30102_tb;

    reg  clk = 0, rst_n, start;
    wire scl, sda;
    wire [17:0] ir, red;
    wire dv_ok, idone;
    wire [3:0] st;

    max30102_driver dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .scl(scl), .sda(sda),
        .ir_data(ir), .red_data(red),
        .data_valid(dv_ok), .init_done(idone),
        .sda_link(), .sda_r(), .state(st)
    );

    pullup(scl); pullup(sda);
    always #5 clk = ~clk;

    initial begin
        $dumpfile("max30102_tb.vcd");
        $dumpvars(0, max30102_tb);
    end

    initial begin
        rst_n = 1; start = 0;
        #10 rst_n = 0;
        #100 rst_n = 1;
        #1000 start = 1;
    end

    // Monitor state + phase + sub_step
    reg [3:0] lst = 4'hF;
    reg [2:0] lph = 3'h7;
    reg [3:0] lss = 4'hF;

    always @(posedge clk) begin
        if (st !== lst) begin
            $display("t=%t ST=%d ph=%d ss=%d scl=%b sda=%b",
                     $time, st, dut.phase, dut.sub_step, scl, sda);
            lst <= st;
        end
        if (dut.phase !== lph || dut.sub_step !== lss) begin
            $display("t=%t     phase=%d sub_step=%d", $time, dut.phase, dut.sub_step);
            lph <= dut.phase; lss <= dut.sub_step;
        end
    end

    always @(posedge idone) $display("t=%t INIT DONE", $time);
    always @(posedge dv_ok) $display("t=%t DATA ir=%h red=%h", $time, ir, red);

    // Verification: check that state machine cycles correctly
    // Without a slave, ACK will fail → state goes 3→11 (ACK1→STOP1)
    // But the phase/sub_step should advance through init, flush, poll
    integer errs;
    initial errs = 0;

    always @(posedge clk) begin
        // After reset, phase should be PH_INIT=0
        if (st == 4'd0 && dut.phase != 3'd0 && $time > 1000)
            $display("WARN: expected PH_INIT(0) got %d", dut.phase);
    end

    initial begin
        #200000000;
        $display("=== END ===");
        $display("Init done: %b", idone);
        $display("Phase: %d  SubStep: %d  State: %d", dut.phase, dut.sub_step, st);
        $finish;
    end

endmodule