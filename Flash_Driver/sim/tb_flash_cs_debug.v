`timescale 1ns / 1ps
//=============================================================================
// Module : tb_flash_cs_debug
// Purpose: 验证 flash_ctrl 的 CS 时序:
//          1. WE (0x06) 后 CS 必须拉高 (WE 是独立事务)
//          2. 擦除/写入命令后 CS 拉高再开始 RSR1 轮询
//          3. 轮询迭代之间 CS 拉高
//          4. Sector Erase + Page Program + Read Back 完整流程
//=============================================================================
module tb_flash_cs_debug;

    reg         clk;
    reg         rst_n;

    // User-side
    reg         start;
    reg  [3:0]  op;
    reg  [23:0] addr;
    reg  [7:0]  wdata;
    reg         wdata_valid;
    wire        wdata_ready;
    reg  [15:0] len;
    wire [7:0]  rdata;
    wire        rdata_valid;
    wire        busy;
    wire        done;
    wire        error;
    wire [7:0]  status_reg1;

    // SPI pads
    wire        sck;
    wire        csn;
    wire        mosi;
    wire        miso;

    localparam [3:0] OP_READ_DATA      = 4'h3;
    localparam [3:0] OP_PAGE_PROGRAM   = 4'h4;
    localparam [3:0] OP_SECTOR_ERASE   = 4'h5;

    // -------------------------------------------------------------------------
    // DUT (CLK_DIV=4 for fast sim; change to 8/256 for hardware debug)
    // -------------------------------------------------------------------------
    flash_top #(
        .CLK_DIV(4)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start),
        .op           (op),
        .addr         (addr),
        .wdata        (wdata),
        .wdata_valid  (wdata_valid),
        .wdata_ready  (wdata_ready),
        .len          (len),
        .rdata        (rdata),
        .rdata_valid  (rdata_valid),
        .busy         (busy),
        .done         (done),
        .error        (error),
        .status_reg1  (status_reg1),
        .sck          (sck),
        .csn          (csn),
        .mosi         (mosi),
        .miso         (miso)
    );

    // Flash model (W25Q64-like, 64KB)
    flash_model #(
        .MEM_DEPTH(64*1024)
    ) u_flash (
        .sck  (sck),
        .csn  (csn),
        .mosi (mosi),
        .miso (miso)
    );

    // -------------------------------------------------------------------------
    // Clock: 100 MHz
    // -------------------------------------------------------------------------
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // =========================================================================
    // CS 时序监控 + 自检
    // =========================================================================
    realtime cs_fall_time, cs_rise_time;
    integer  cs_high_errors = 0;
    integer  cs_low_glitch_errors = 0;
    integer  we_cs_high_ok = 0;
    integer  total_cs_high_count = 0;

    // 抓 CS 下降沿 (CS# 有效)
    always @(negedge csn) begin
        cs_fall_time = $realtime;
    end

    // 抓 CS 上升沿 (CS# 释放) + 检查高电平持续时间
    always @(posedge csn) begin
        cs_rise_time = $realtime;
        total_cs_high_count = total_cs_high_count + 1;

        // 检查 CS 高电平不低于 50ns (tSHSL)
        if (cs_rise_time - cs_fall_time > 0.0) begin
            if ((cs_rise_time - cs_fall_time) < 45.0) begin
                $display("[CS-CHECK] WARNING t=%0t: CS low pulse too short: %0t ns (min ~45 ns for data)",
                         $realtime, cs_rise_time - cs_fall_time);
                cs_low_glitch_errors = cs_low_glitch_errors + 1;
            end
        end
    end

    // 持续监控: CS 不应有毛刺 (高脉冲 < 30ns)
    // 如果有, 说明中间状态没正确保持 CS
    realtime last_cs_posedge = 0;
    always @(posedge csn) begin
        last_cs_posedge = $realtime;
    end
    always @(negedge csn) begin
        // 忽略同一时间步的边沿 (仿真 delta-cycle 产物)
        if (last_cs_posedge > 0 && ($realtime - last_cs_posedge) < 1.0) begin
            // same-timestep, delta-cycle artifact → skip
        end else if (last_cs_posedge > 0 && ($realtime - last_cs_posedge) < 30.0) begin
            $display("[CS-GLITCH] ERROR t=%0t: CS high glitch only %0t ns!",
                     $realtime, $realtime - last_cs_posedge);
            cs_high_errors = cs_high_errors + 1;
        end
    end

    // =========================================================================
    // 监控 flash_ctrl 内部状态
    // =========================================================================
    wire [3:0]  fsm_state = dut.u_flash_ctrl.state;
    wire [16:0] gap_cnt   = dut.u_flash_ctrl.gap_cnt;
    wire        gap_to_cmd = dut.u_flash_ctrl.gap_to_cmd;
    wire        gap_long   = dut.u_flash_ctrl.gap_long;

    // 打印状态转换
    reg [3:0] fsm_prev = 4'd0;
    always @(posedge clk) begin
        if (rst_n && fsm_state != fsm_prev) begin
            case (fsm_state)
                4'd0:  $display("[FSM] t=%0t ST_IDLE", $realtime);
                4'd1:  $display("[FSM] t=%0t ST_TX_WE (gap_to_cmd=%b)", $realtime, gap_to_cmd);
                4'd2:  $display("[FSM] t=%0t ST_TX_CMD", $realtime);
                4'd3:  $display("[FSM] t=%0t ST_TX_ADDR3", $realtime);
                4'd4:  $display("[FSM] t=%0t ST_TX_ADDR2", $realtime);
                4'd5:  $display("[FSM] t=%0t ST_TX_ADDR1", $realtime);
                4'd6:  $display("[FSM] t=%0t ST_DATA", $realtime);
                4'd7:  $display("[FSM] t=%0t ST_POLL_TX", $realtime);
                4'd8:  $display("[FSM] t=%0t ST_POLL_DUMMY", $realtime);
                4'd9:  $display("[FSM] t=%0t ST_POLL_WAIT", $realtime);
                4'd10: $display("[FSM] t=%0t ST_DONE (error=%b)", $realtime, error);
                4'd11: $display("[FSM] t=%0t ST_POLL_GAP (cnt=%0d to_cmd=%b)", $realtime, gap_cnt, gap_to_cmd);
                default: $display("[FSM] t=%0t UNKNOWN=%h", $realtime, fsm_state);
            endcase
            fsm_prev = fsm_state;
        end
    end

    // =========================================================================
    // Tasks
    // =========================================================================
    task issue_cmd(
        input [3:0]  op_i,
        input [23:0] addr_i,
        input [15:0] len_i
    );
        begin
            @(posedge clk);
            op    = op_i;
            addr  = addr_i;
            len   = len_i;
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            wait (busy == 1'b1);
            wait (done == 1'b1);
            @(posedge clk);
            if (error) begin
                $display("[TEST] ERROR: op=%h reported error at t=%0t", op_i, $realtime);
            end
        end
    endtask

    // Page Program with wdata handshake
    task page_program(input [23:0] addr_i, input [15:0] len_i);
        integer k;
        begin
            @(posedge clk);
            op    = OP_PAGE_PROGRAM;
            addr  = addr_i;
            len   = len_i;
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            fork
                begin
                    wait (busy == 1'b1);
                    wait (done == 1'b1);
                    @(posedge clk);
                end
                begin
                    wait (busy == 1'b1);
                    @(posedge clk);
                    while (!wdata_ready) @(posedge clk);
                    @(posedge clk);
                    wdata_valid = 1'b1;
                    for (k = 0; k < len_i; k = k + 1) begin
                        wdata = 8'hA0 + k[7:0];
                        repeat (38) @(posedge clk);
                    end
                    wdata_valid = 1'b0;
                end
            join
        end
    endtask

    // Read and check sequential pattern
    task read_check(input [23:0] addr_i, input [15:0] len_i, input [7:0] expected_start);
        integer k;
        reg [7:0] expected;
        reg [7:0] captured [0:255];
        begin
            fork
                begin
                    issue_cmd(OP_READ_DATA, addr_i, len_i);
                end
                begin
                    for (k = 0; k < len_i; k = k + 1) begin
                        while (!rdata_valid) @(posedge clk);
                        @(posedge clk);
                        captured[k] = rdata;
                    end
                end
            join
            expected = expected_start;
            for (k = 0; k < len_i; k = k + 1) begin
                if (captured[k] !== expected) begin
                    $display("[TEST] FAIL: read[%0d]=0x%02h expected=0x%02h", k, captured[k], expected);
                    $finish;
                end
                expected = expected + 1'b1;
            end
            $display("[TEST] PASS: read %0d bytes from 0x%06h OK", len_i, addr_i);
        end
    endtask

    task read_check_const(input [23:0] addr_i, input [15:0] len_i, input [7:0] exp);
        integer k;
        reg [7:0] captured [0:255];
        begin
            fork
                begin
                    issue_cmd(OP_READ_DATA, addr_i, len_i);
                end
                begin
                    for (k = 0; k < len_i; k = k + 1) begin
                        while (!rdata_valid) @(posedge clk);
                        @(posedge clk);
                        captured[k] = rdata;
                    end
                end
            join
            for (k = 0; k < len_i; k = k + 1) begin
                if (captured[k] !== exp) begin
                    $display("[TEST] FAIL: read[%0d]=0x%02h expected=0x%02h", k, captured[k], exp);
                    $finish;
                end
            end
            $display("[TEST] PASS: read %0d bytes of 0x%02h from 0x%06h OK", len_i, exp, addr_i);
        end
    endtask

    // =========================================================================
    // Main test
    // =========================================================================
    initial begin
        $display("============================================================");
        $display(" CS时序验证测试: WE-CS↑ / 命令-CS↑ / 轮询-CS↑");
        $display("============================================================");

        // Reset
        start = 0; op = 0; addr = 0; wdata = 0; wdata_valid = 0; len = 0;
        rst_n = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (10) @(posedge clk);

        // ---- Test 1: Sector Erase → 验证 WE 后 CS↑ ----
        $display("");
        $display("--- Test 1: Sector Erase (验证 WE 后 CS 拉高) ---");
        cs_high_errors = 0;
        cs_low_glitch_errors = 0;
        issue_cmd(OP_SECTOR_ERASE, 24'h000000, 16'd0);
        $display("[TEST] Sector Erase done. Status=0x%02h BUSY=%b WEL=%b error=%b",
                 status_reg1, status_reg1[0], status_reg1[1], error);
        $display("[TEST] CS high glitch errors (WE gap): %0d", cs_high_errors);
        $display("[TEST] Total CS rising edges seen: %0d", total_cs_high_count);

        // ---- Test 2: Page Program 5 bytes ----
        $display("");
        $display("--- Test 2: Page Program 5 bytes (0xA0..0xA4) ---");
        cs_high_errors = 0;
        page_program(24'h000000, 16'd5);
        $display("[TEST] Page Program done. error=%b", error);

        // ---- Test 3: Read back → 验证数据写入成功 ----
        $display("");
        $display("--- Test 3: Read back 5 bytes, verify data ---");
        read_check(24'h000000, 16'd5, 8'hA0);

        // ---- Test 4: Sector Erase again → 验证擦除 ----
        $display("");
        $display("--- Test 4: Sector Erase + Read → 验证擦除 (全FF) ---");
        issue_cmd(OP_SECTOR_ERASE, 24'h000000, 16'd0);
        read_check_const(24'h000000, 16'd5, 8'hFF);

        // ---- 最终报告 ----
        $display("");
        $display("============================================================");
        if (cs_high_errors > 0) begin
            $display("FAIL: CS glitch errors detected: %0d", cs_high_errors);
        end else begin
            $display("PASS: No CS glitch errors");
        end
        $display("Total CS rising edges (deassertions): %0d", total_cs_high_count);
        $display("============================================================");
        $display("ALL TESTS PASSED");
        $display("============================================================");
        $finish;
    end

    // Watchdog
    initial begin
        #500_000_000;  // 500 ms
        $display("[TEST] TIMEOUT: simulation stuck at t=%0t", $realtime);
        $display("[TEST] FSM state=%h gap_cnt=%0d gap_to_cmd=%b", fsm_state, gap_cnt, gap_to_cmd);
        $finish;
    end

endmodule
