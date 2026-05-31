// DS18B20 Driver Testbench - Simple version
`timescale 1us / 1ns

module testbench;

    reg clk;
    reg rst_n;
    reg start;
    wire [15:0] temperature;
    wire data_valid;
    wire error;
    wire dq_out;
    wire dq_oe;
    wire dq;

    // DUT drives when dq_oe=1
    // Model drives when model_drive=1
    // Priority: DUT low > Model low > pull-up high
    assign dq = (dq_oe && !dq_out) ? 1'b0 :
                (model_drive && !model_val) ? 1'b0 :
                1'b1;

    ds18b20_driver #(.CLK_FREQ(50_000_000)) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .dq_in(dq),
        .temperature(temperature), .data_valid(data_valid),
        .error(error), .dq_out(dq_out), .dq_oe(dq_oe)
    );

    initial clk = 0;
    always #10 clk = ~clk;

    // ============================================
    // Simple Model
    // ============================================
    reg [3:0] model_state;
    localparam M_WAIT = 0;
    localparam M_RESET = 1;
    localparam M_PRESENCE = 2;
    localparam M_WAIT_CMD = 3;
    localparam M_CONVERT = 4;
    localparam M_READ = 5;

    reg model_drive;
    reg model_val;
    assign dq = model_drive ? model_val : 1'b1;

    reg last_dq_oe;
    always @(posedge clk) last_dq_oe <= dq_oe;

    reg [31:0] timer;
    reg [7:0] cmd;
    reg [3:0] bit_cnt;

    initial begin
        model_state = M_WAIT;
        model_drive = 0;
        model_val = 1;
        timer = 0;
        cmd = 0;
        bit_cnt = 0;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            model_state <= M_WAIT;
            model_drive <= 0;
            timer <= 0;
        end else begin
            case (model_state)
                M_WAIT: begin
                    model_drive <= 0;
                    timer <= 0;
                    if (dq_oe && !last_dq_oe) begin
                        model_state <= M_RESET;
                        $display("Time %0t: Reset start", $time);
                    end
                end

                M_RESET: begin
                    model_drive <= 1;
                    model_val <= 0;
                    if (!dq_oe && last_dq_oe) begin
                        $display("Time %0t: Reset end, start presence", $time);
                        model_state <= M_PRESENCE;
                        timer <= 0;
                    end
                end

                M_PRESENCE: begin
                    // Drive presence for 180us
                    model_drive <= 1;
                    model_val <= 0;
                    timer <= timer + 1;
                    if (timer >= 9000) begin
                        model_drive <= 0;
                        model_val <= 1;
                        model_state <= M_WAIT_CMD;
                        timer <= 0;
                        $display("Time %0t: Presence done", $time);
                    end
                end

                M_WAIT_CMD: begin
                    model_drive <= 0;
                    timer <= timer + 1;
                    if (timer >= 10000) begin
                        $display("Time %0t: Timeout waiting for cmd", $time);
                        model_state <= M_WAIT;
                    end
                end

                M_CONVERT: begin
                    model_drive <= 0;
                    timer <= timer + 1;
                    if (timer >= 750000) begin
                        $display("Time %0t: Convert done", $time);
                        model_state <= M_WAIT;
                    end
                end

                M_READ: begin
                    model_drive <= 0;
                    timer <= timer + 1;
                    if (timer >= 100000) begin
                        $display("Time %0t: Read done", $time);
                        model_state <= M_WAIT;
                    end
                end
            endcase
        end
    end

    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, testbench);

        rst_n = 0;
        start = 0;
        #100 rst_n = 1;
        #50;
        $display("Time %0t: Start", $time);
        start = 1; #50 start = 0;

        #3000000;

        if (data_valid) begin
            $display("PASS: Temp=0x%h", temperature);
        end else if (error) begin
            $display("ERROR");
        end else begin
            $display("TIMEOUT");
        end
        #1000 $finish;
    end

endmodule
