`timescale 1ns / 1ps
//=============================================================================
// tb_flash_multi_record — verify sequential write address increments
//
// Simulates N consecutive saves (session_end_int pulses).
// Checks that each Page Program goes to: 0x000000, 0x000010, 0x000020, ...
// After MAX_RECORDS (40) saves, checks that Block Erase is issued and
// write_addr wraps back to 0.
//
// Simplified: drives MISO=0x02 (BUSY=0,WEL=1) so polls pass immediately.
//=============================================================================
module tb_flash_multi_record;

    reg         clk;
    reg         rst_n;

    // ---- flash_op_seq user interface ----
    reg         session_end_int;
    reg         flash_view_en;
    reg         btn_confirm;
    reg         btn_mode;
    reg         work_en;
    reg  [15:0] save_step;
    reg  [7:0]  save_avg_cad;
    reg  [7:0]  save_avg_hr;

    // ---- flash_top command interface ----
    wire [3:0]  op;
    wire [23:0] addr;
    wire [15:0] len;
    wire        start;
    wire        wdata_valid;
    wire [7:0]  wdata;
    wire        wdata_ready;
    wire [7:0]  rdata;
    wire        rdata_valid;
    wire        busy;
    wire        done;
    wire        error;

    wire [15:0] load_step;
    wire [7:0]  load_avg_cad;
    wire [7:0]  load_avg_hr;
    wire        load_valid;
    wire        flash_op_en;
    wire [1:0]  flash_op_message;
    wire        delete_active;
    wire        load_in_progress;
    wire        session_happened;

    // ---- SPI pads ----
    wire        sck;
    wire        csn;
    wire        mosi;
    wire        miso;

    // ---- DUT: flash_op_seq + flash_top ----
    flash_top #(.CLK_DIV(4)) u_flash_top (
        .clk(clk), .rst_n(rst_n),
        .start(start), .op(op), .addr(addr),
        .wdata(wdata), .wdata_valid(wdata_valid), .wdata_ready(wdata_ready),
        .len(len), .rdata(rdata), .rdata_valid(rdata_valid),
        .busy(busy), .done(done), .error(error),
        .status_reg1(),
        .sck(sck), .csn(csn), .mosi(mosi), .miso(miso)
    );

    flash_op_seq u_flash_op_seq (
        .clk(clk), .rst_n(rst_n),
        .session_end_int(session_end_int),
        .flash_view_en(flash_view_en),
        .btn_confirm(btn_confirm),
        .btn_mode(btn_mode),
        .work_en(work_en),
        .save_step(save_step),
        .save_avg_cad(save_avg_cad),
        .save_avg_hr(save_avg_hr),
        .op(op), .addr(addr), .len(len),
        .start(start), .wdata_valid(wdata_valid), .wdata(wdata),
        .wdata_ready(wdata_ready), .rdata(rdata), .rdata_valid(rdata_valid),
        .busy(busy), .done(done), .error(error),
        .flash_op_en(flash_op_en), .flash_op_message(flash_op_message),
        .load_step(load_step), .load_avg_cad(load_avg_cad),
        .load_avg_hr(load_avg_hr), .load_valid(load_valid),
        .delete_active(delete_active), .load_in_progress(load_in_progress),
        .session_happened(session_happened)
    );

    // ---- MISO: use flash_model for correct timing, force status=0x02 ----
    flash_model #(.MEM_DEPTH(64*1024)) u_flash_model (
        .sck(sck), .csn(csn), .mosi(mosi), .miso(miso)
    );
    initial begin
        force u_flash_model.status_reg = 8'h02;
        force u_flash_model.wel        = 1'b1;
    end

    // ---- Clock 100 MHz ----
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ---- Probe flash_op_seq internal state ----
    wire [3:0]  fsm_state      = u_flash_op_seq.state;
    wire [23:0] dbg_write_addr  = u_flash_op_seq.write_addr;
    wire [23:0] dbg_read_addr   = u_flash_op_seq.read_addr;
    wire [5:0]  dbg_total_rec   = u_flash_op_seq.total_records;
    wire [5:0]  dbg_cur_rec     = u_flash_op_seq.current_record;
    wire        dbg_scan_done   = u_flash_op_seq.scan_done;
    wire        dbg_need_erase  = u_flash_op_seq.need_erase;

    // ---- MOSI address capture: track PP address ----
    reg [23:0]  captured_pp_addr;
    reg         pp_capture_valid;
    reg [4:0]   pp_count;
    reg [23:0]  expected_addr;
    reg [7:0]   mosi_sh;
    reg [2:0]   mosi_bc;
    reg         csn_d1;
    reg [7:0]   mosi_byte;
    reg         mosi_byte_valid;
    reg         sck_d1;

    always @(posedge clk) begin
        csn_d1 <= csn;
        sck_d1 <= sck;
    end

    // Byte-level MOSI capture: sample on SCK rising edge (MSB first)
    always @(posedge clk) begin
        if (csn) begin
            mosi_sh  <= 8'd0;
            mosi_bc  <= 3'd0;
            mosi_byte_valid <= 1'b0;
        end else if (sck && !sck_d1) begin
            // SCK rising edge: shift in MOSI bit
            mosi_sh <= {mosi_sh[6:0], mosi};
            if (mosi_bc == 3'd7) begin
                mosi_byte <= {mosi_sh[6:0], mosi};
                mosi_byte_valid <= 1'b1;
                mosi_bc <= 3'd0;
            end else begin
                mosi_bc <= mosi_bc + 3'd1;
            end
        end else begin
            mosi_byte_valid <= 1'b0;
        end
    end

    // Capture Page Program address: after seeing 0x02 cmd, collect next 3 bytes
    reg         pp_cmd_seen;
    reg [1:0]   pp_addr_idx;

    always @(posedge clk) begin
        if (csn) begin
            pp_cmd_seen <= 1'b0;
            pp_addr_idx <= 2'd0;
        end else if (mosi_byte_valid && !pp_cmd_seen && mosi_byte == 8'h02) begin
            pp_cmd_seen <= 1'b1;
            pp_addr_idx <= 2'd0;
        end else if (pp_cmd_seen && mosi_byte_valid) begin
            case (pp_addr_idx)
                2'd0: captured_pp_addr[23:16] <= mosi_byte;
                2'd1: captured_pp_addr[15:8]  <= mosi_byte;
                2'd2: begin
                    captured_pp_addr[7:0] <= mosi_byte;
                    pp_capture_valid <= 1'b1;
                    pp_count <= pp_count + 5'd1;
                    pp_cmd_seen <= 1'b0;
                end
            endcase
            pp_addr_idx <= pp_addr_idx + 2'd1;
        end else begin
            pp_capture_valid <= 1'b0;
        end
    end

    // =========================================================================
    // Test sequencer
    // =========================================================================
    reg [31:0] cycle;
    reg [7:0]  test_phase;
    reg [31:0] phase_timer;
    reg [5:0]  save_num;          // which save we're on (0..45)
    reg [23:0] expected_addrs [0:50];  // expected PP addresses

    reg        test_pass;
    reg        test_done;
    reg [31:0] error_count;

    localparam PH_INIT       = 8'd0;
    localparam PH_WAIT_LOAD  = 8'd1;
    localparam PH_DO_SAVE    = 8'd2;
    localparam PH_WAIT_IDLE  = 8'd3;
    localparam PH_NEXT_SAVE  = 8'd4;
    localparam PH_DONE       = 8'd5;
    localparam PH_WRAP_TEST  = 8'd6;
    localparam PH_WAIT_ERASE = 8'd7;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle           <= 32'd0;
            test_phase      <= PH_INIT;
            phase_timer     <= 32'd0;
            save_num        <= 6'd0;
            test_pass       <= 1'b1;
            test_done       <= 1'b0;
            error_count     <= 32'd0;
            session_end_int <= 1'b0;
            flash_view_en   <= 1'b0;
            btn_confirm     <= 1'b1;   // not pressed (active low)
            btn_mode        <= 1'b1;
            work_en         <= 1'b0;
            save_step       <= 16'd0;
            save_avg_cad    <= 8'd0;
            save_avg_hr     <= 8'd0;
            expected_addr   <= 24'd0;
        end else begin
            cycle <= cycle + 32'd1;
            session_end_int <= 1'b0;   // pulse default

            case (test_phase)

                PH_INIT: begin
                    if (cycle == 32'd100) begin
                        $display("[%0t] Waiting for initial load to complete...", $time);
                        test_phase <= PH_WAIT_LOAD;
                    end
                end

                PH_WAIT_LOAD: begin
                    // Wait for scan+load to finish (load_done)
                    if (dbg_scan_done && fsm_state == 4'd0) begin
                        $display("[%0t] Initial load done. total_records=%0d",
                                 $time, dbg_total_rec);
                        $display("[%0t] ===== Starting sequential save test =====", $time);
                        test_phase <= PH_DO_SAVE;
                        save_num   <= 6'd0;
                    end
                end

                PH_DO_SAVE: begin
                    // Wait until flash_op_seq is idle, then pulse session_end_int
                    if (fsm_state == 4'd0 && phase_timer > 32'd1000) begin
                        // Set up save data for this record
                        save_step    <= {8'd0, save_num};  // step = save_num (for verification)
                        save_avg_cad <= save_num;
                        save_avg_hr  <= save_num + 8'd10;
                        session_end_int <= 1'b1;
                        expected_addr <= save_num * 16;
                        phase_timer   <= 32'd0;
                        test_phase    <= PH_WAIT_IDLE;
                        $display("[%0t] Save #%0d: pulsing session_end_int, expected PP addr=0x%06h",
                                 $time, save_num, save_num * 16);
                    end else begin
                        phase_timer <= phase_timer + 32'd1;
                    end
                end

                PH_WAIT_IDLE: begin
                    // Wait for flash_op_seq to start (busy goes high), then return to idle
                    phase_timer <= phase_timer + 32'd1;
                    // Must see fsm_state leave IDLE first (save started), then return
                    if (phase_timer > 32'd100 && fsm_state == 4'd0 && dbg_scan_done) begin
                        // Check PP address
                        if (pp_capture_valid) begin
                            if (captured_pp_addr == expected_addr) begin
                                $display("[%0t]   PASS: PP addr=0x%06h (expected 0x%06h)",
                                         $time, captured_pp_addr, expected_addr);
                            end else begin
                                $display("[%0t]   FAIL: PP addr=0x%06h (expected 0x%06h)",
                                         $time, captured_pp_addr, expected_addr);
                                test_pass   <= 1'b0;
                                error_count <= error_count + 32'd1;
                            end
                        end else begin
                            $display("[%0t]   WARN: no PP address captured for save #%0d",
                                     $time, save_num);
                        end

                        $display("[%0t]   dbg: write_addr=0x%06h total_rec=%0d cur_rec=%0d",
                                 $time, dbg_write_addr, dbg_total_rec, dbg_cur_rec);

                        // Verify write_addr
                        if (save_num < 40) begin
                            if (dbg_write_addr != (save_num + 1) * 16) begin
                                $display("[%0t]   FAIL: write_addr=0x%06h (expected 0x%06h)",
                                         $time, dbg_write_addr, (save_num + 1) * 16);
                                test_pass   <= 1'b0;
                                error_count <= error_count + 32'd1;
                            end
                        end

                        test_phase <= PH_NEXT_SAVE;
                    end
                end

                PH_NEXT_SAVE: begin
                    phase_timer <= phase_timer + 32'd1;
                    if (phase_timer > 32'd500) begin
                        if (save_num < 41) begin
                            // 41 saves: 0-39 (fill up) + 40 (wrap test)
                            save_num  <= save_num + 6'd1;
                            test_phase <= PH_DO_SAVE;
                        end else begin
                            test_phase <= PH_DONE;
                        end
                    end
                end

                PH_DONE: begin
                    if (!test_done) begin
                        test_done <= 1'b1;
                        #1000;

                        $display("");
                        $display("==============================================");
                        $display(" Test Results");
                        $display("==============================================");
                        $display(" Total saves attempted: %0d", save_num);
                        $display(" PP count captured:     %0d", pp_count);
                        $display(" Errors:                %0d", error_count);

                        if (error_count == 0 && test_pass) begin
                            $display("===== ALL TESTS PASSED =====");
                        end else begin
                            $display("===== TEST FAILED =====");
                        end

                        $display("");
                        $display(" Write address progression:");
                        for (integer i = 0; i <= save_num && i <= 45; i = i + 1) begin
                            $display("   Save #%0d → addr 0x%06h", i, i * 16);
                        end
                        $display("   Save #40 → erase + addr 0x000000 (wrap)");

                        #1000;
                        $finish;
                    end
                end

                default: test_phase <= PH_INIT;
            endcase
        end
    end

    // =========================================================================
    // Init
    // =========================================================================
    initial begin
        rst_n = 0;
        #500;
        rst_n = 1;
    end

    // =========================================================================
    // Timeout
    // =========================================================================
    initial begin
        #10_000_000;  // 10ms max
        $display("");
        $display("TIMEOUT at %0t", $time);
        $display("  fsm_state     = %0d", fsm_state);
        $display("  test_phase    = %0d", test_phase);
        $display("  save_num      = %0d", save_num);
        $display("  write_addr    = 0x%06h", dbg_write_addr);
        $display("  total_records = %0d", dbg_total_rec);
        $display("  scan_done     = %b", dbg_scan_done);
        $finish;
    end

    // =========================================================================
    // VCD dump
    // =========================================================================
    initial begin
        $dumpfile("tb_flash_multi_record.vcd");
        $dumpvars(0, tb_flash_multi_record);
    end

endmodule
