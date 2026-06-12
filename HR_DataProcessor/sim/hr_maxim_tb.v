//=============================================================================
// Testbench  : hr_maxim_tb
// Description: Self-checking testbench for the Maxim-style HR detector.
//              Generates realistic PPG-like raw IR waveforms and verifies
//              that hr_maxim correctly detects heart rate.
//
// Test scenarios:
//   Test 1: 72 BPM (1.2 Hz sine, DC=93000, amplitude=200 codes)
//   Test 2: 45 BPM (0.75 Hz sine, DC=93000, amplitude=200 codes)
//
// Success criteria:
//   - hr_valid fires at least once per test
//   - Final hr_locked = 1
//   - BPM within broad physiologically-plausible range
//=============================================================================

`timescale 1ns / 1ps

module hr_maxim_tb;

    // ---------------------------------------------------------------------------
    // DUT signals
    // ---------------------------------------------------------------------------
    reg              clk = 0;
    reg              rst_n = 0;
    reg              in_valid = 0;
    reg  [17:0]      ir_raw = 0;
    wire [7:0]       hr_bpm;
    wire             hr_valid;
    wire             hr_locked;
    wire signed [17:0] hamm_out;
    wire signed [17:0] threshold_out;
    wire             dec_valid;

    // ---------------------------------------------------------------------------
    // DUT instantiation
    // ---------------------------------------------------------------------------
    hr_maxim uut (
        .clk           (clk),
        .rst_n         (rst_n),
        .in_valid      (in_valid),
        .ir_ac         (ir_raw),
        .hr_bpm        (hr_bpm),
        .hr_valid      (hr_valid),
        .hr_locked     (hr_locked),
        .hamm_out      (hamm_out),
        .threshold_out (threshold_out),
        .dec_valid     (dec_valid)
    );

    // 100 MHz clock
    always #5 clk = ~clk;

    // ---------------------------------------------------------------------------
    // Precomputed sine tables (100 samples each)
    //   Test 1: 1.2 Hz -> 72 BPM, amplitude 200 codes
    //   Test 2: 0.75 Hz -> 45 BPM, amplitude 200 codes
    // ---------------------------------------------------------------------------
    integer sintab_120 [0:99];   // 1.2 Hz = 72 BPM
    integer sintab_075 [0:99];   // 0.75 Hz = 45 BPM
    integer active_sintab [0:99];
    integer i, j;

    // ---------------------------------------------------------------------------
    // Test tracking
    // ---------------------------------------------------------------------------
    integer       pass_count;
    integer       hr_pulse_count;
    reg [7:0]     captured_bpm;
    reg            captured_locked;

    // ---------------------------------------------------------------------------
    // Task: drive raw IR samples simulating 100 Hz data stream.
    //   Each sample: in_valid=1 for 1 cycle, then 3 cycles idle.
    //   Decimation 4:1 gives effective 25 Hz filter rate.
    // ---------------------------------------------------------------------------
    task automatic drive_samples;
        input integer n;              // number of raw samples
        input integer dc_level;       // DC offset (e.g., 93000)
        begin
            for (j = 0; j < n; j = j + 1) begin
                ir_raw = dc_level + active_sintab[j % 100];
                @(posedge clk);
                in_valid = 1;
                @(posedge clk);
                in_valid = 0;
                repeat (2) @(posedge clk);
            end
        end
    endtask

    // ---------------------------------------------------------------------------
    // Monitor hr_valid pulses
    // ---------------------------------------------------------------------------
    always @(posedge clk) begin
        if (hr_valid && rst_n) begin
            hr_pulse_count = hr_pulse_count + 1;
            captured_bpm    = hr_bpm;
            captured_locked = hr_locked;
            $display("  [t=%12t] hr_valid #%0d: BPM=%0d locked=%b",
                     $time, hr_pulse_count, hr_bpm, hr_locked);
        end
    end

    // ---------------------------------------------------------------------------
    // Main stimulus
    // ---------------------------------------------------------------------------
    initial begin
        $dumpfile("hr_maxim_tb.vcd");
        $dumpvars(0, hr_maxim_tb);

        pass_count = 0;

        // Build sine tables
        for (i = 0; i < 100; i = i + 1) begin
            sintab_120[i] = $rtoi(200.0 * $sin(2.0 * 3.14159265 * 1.2 * i / 100.0));
            sintab_075[i] = $rtoi(200.0 * $sin(2.0 * 3.14159265 * 0.75 * i / 100.0));
        end

        // -----------------------------------------------------------------------
        // Reset
        // -----------------------------------------------------------------------
        $display("\n============================================================");
        $display("===  hr_maxim Self-Checking Testbench                    ===");
        $display("============================================================\n");

        rst_n = 0; in_valid = 0; ir_raw = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (20) @(posedge clk);
        $display("[t=%12t] Reset released\n", $time);

        // -----------------------------------------------------------------------
        // Test 1: 72 BPM (1.2 Hz sine, DC=93000, amplitude=200)
        // -----------------------------------------------------------------------
        $display("--- Test 1: 72 BPM (1.2 Hz, DC=93000, amp=200) ---");
        $display("    Driving 4000 raw samples...");

        for (i = 0; i < 100; i = i + 1) active_sintab[i] = sintab_120[i];
        hr_pulse_count  = 0;
        captured_bpm    = 8'd0;
        captured_locked = 1'b0;

        drive_samples(4000, 93000);
        repeat (100) @(posedge clk);

        $display("    hr_valid pulses:  %0d", hr_pulse_count);
        $display("    final captured:   BPM=%0d locked=%b", captured_bpm, captured_locked);
        $display("    DUT sample_cnt:   %0d", uut.sample_cnt);
        $display("    DUT threshold:    %0d", $signed(threshold_out));

        if (hr_pulse_count > 0 && captured_locked) begin
            if (captured_bpm >= 40 && captured_bpm <= 120) begin
                $display("    PASS: BPM=%0d in [40,120], locked=1, hr_pulses=%0d\n",
                         captured_bpm, hr_pulse_count);
                pass_count = pass_count + 1;
            end else begin
                $display("    FAIL: BPM=%0d out of range [40,120]\n", captured_bpm);
            end
        end else begin
            $display("    FAIL: hr_pulses=%0d locked=%b (need >0 pulses, locked=1)\n",
                     hr_pulse_count, captured_locked);
        end

        // -----------------------------------------------------------------------
        // Test 2: 45 BPM (0.75 Hz, DC=93000, amplitude=200)
        // -----------------------------------------------------------------------
        $display("--- Test 2: 45 BPM (0.75 Hz, DC=93000, amp=200) ---");
        $display("    Driving 4000 raw samples...");

        // Reset DUT for clean start
        rst_n = 0; in_valid = 0; ir_raw = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (20) @(posedge clk);

        for (i = 0; i < 100; i = i + 1) active_sintab[i] = sintab_075[i];
        hr_pulse_count  = 0;
        captured_bpm    = 8'd0;
        captured_locked = 1'b0;

        drive_samples(4000, 93000);
        repeat (100) @(posedge clk);

        $display("    hr_valid pulses:  %0d", hr_pulse_count);
        $display("    final captured:   BPM=%0d locked=%b", captured_bpm, captured_locked);
        $display("    DUT sample_cnt:   %0d", uut.sample_cnt);
        $display("    DUT threshold:    %0d", $signed(threshold_out));

        if (hr_pulse_count > 0 && captured_locked) begin
            if (captured_bpm >= 30 && captured_bpm <= 100) begin
                $display("    PASS: BPM=%0d in [30,100], locked=1, hr_pulses=%0d\n",
                         captured_bpm, hr_pulse_count);
                pass_count = pass_count + 1;
            end else begin
                $display("    FAIL: BPM=%0d out of range [30,100]\n", captured_bpm);
            end
        end else begin
            $display("    FAIL: hr_pulses=%0d locked=%b (need >0 pulses, locked=1)\n",
                     hr_pulse_count, captured_locked);
        end

        // -----------------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------------
        $display("============================================================");
        $display("===  Summary: %0d / 2 tests passed                       ===", pass_count);
        if (pass_count == 2)
            $display("===  OVERALL: PASS                                       ===");
        else
            $display("===  OVERALL: FAIL                                       ===");
        $display("============================================================\n");
        $finish;
    end

    // ---------------------------------------------------------------------------
    // Watchdog timer
    // ---------------------------------------------------------------------------
    initial begin
        #20_000_000;   // 20 ms
        $display("\n============================================================");
        $display("===  WATCHDOG TIMEOUT at t=%0t ns                         ===", $time);
        $display("===  OVERALL: FAIL (timeout)                              ===");
        $display("============================================================\n");
        $finish;
    end

endmodule
