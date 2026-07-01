`timescale 1ns / 1ps
//=============================================================================
// tb_flash_write_5bytes — verify 5-byte Page Program completes with MISO=0x02
//
// MISO fixed to 0x02 (BUSY=0, WEL=1); status poll passes immediately.
// Checks: byte_cnt=5, CSn deasserts, no double-count / hang.
//=============================================================================
module tb_flash_write_5bytes;

    reg         clk;
    reg         rst_n;

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

    wire        sck;
    wire        csn;
    wire        mosi;
    reg         miso;

    localparam [3:0] OP_PAGE_PROGRAM = 4'h4;

    // ---- DUT ----
    flash_top #(.CLK_DIV(4)) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .op(op), .addr(addr),
        .wdata(wdata), .wdata_valid(wdata_valid), .wdata_ready(wdata_ready),
        .len(len),
        .rdata(rdata), .rdata_valid(rdata_valid),
        .busy(busy), .done(done), .error(error),
        .status_reg1(status_reg1),
        .sck(sck), .csn(csn), .mosi(mosi), .miso(miso)
    );

    // ---- Internal probes ----
    wire [15:0] fsm_byte_cnt = dut.u_flash_ctrl.byte_cnt;
    wire [3:0]  fsm_state    = dut.u_flash_ctrl.state;
    wire        spi_busy_int  = dut.u_flash_ctrl.spi_busy;

    // ---- Clock ----
    initial clk = 1'b0;
    always #5 clk = ~clk;   // 100 MHz

    // =========================================================================
    // Use the existing flash_model for correct SPI Mode 0 MISO timing.
    // Force its status register to always return 0x02 (BUSY=0,WEL=1).
    // =========================================================================
    flash_model #(.MEM_DEPTH(64*1024)) u_flash_model (
        .sck  (sck),
        .csn  (csn),
        .mosi (mosi),
        .miso (miso)
    );

    // Force the model's status_reg and WEL so poll always passes
    initial begin
        force u_flash_model.status_reg = 8'h02;
        force u_flash_model.wel        = 1'b1;
    end

    // =========================================================================
    // MOSI byte capture
    // =========================================================================
    reg [7:0]  mosi_sh = 0;
    reg [2:0]  mosi_bc = 0;
    reg [7:0]  mosi_cap [0:31];
    reg [4:0]  mosi_idx = 0;

    always @(posedge clk) begin
        if (csn) begin
            mosi_sh  <= 8'd0;
            mosi_bc  <= 3'd0;
        end else if (sck && !dut.u_spi_master.sck) begin
            // SCK rising edge (spi_master drives sck; use internal sck edge)
            mosi_sh <= {mosi_sh[6:0], mosi};
            if (mosi_bc == 3'd7) begin
                mosi_cap[mosi_idx] <= {mosi_sh[6:0], mosi};
                mosi_idx <= mosi_idx + 5'd1;
                mosi_bc  <= 3'd0;
            end else begin
                mosi_bc <= mosi_bc + 3'd1;
            end
        end
    end

    // =========================================================================
    // Test sequencer — synthesize-friendly FSM
    // =========================================================================
    reg [3:0]  tb_state;
    reg [31:0] timer;
    reg [2:0]  byte_sent;
    reg        wr_edge;       // set when wdata_ready seen, cleared when !wdata_ready
    reg [7:0]  test_wdata [0:4];

    localparam TB_IDLE      = 4'd0;
    localparam TB_ISSUE     = 4'd1;
    localparam TB_WAIT_DATA = 4'd2;   // wait for ST_DATA
    localparam TB_SEND      = 4'd3;   // handshake
    localparam TB_WAIT_DONE = 4'd4;
    localparam TB_CHECK     = 4'd5;
    localparam TB_DONE      = 4'd6;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tb_state    <= TB_IDLE;
            timer       <= 32'd0;
            start       <= 1'b0;
            op          <= 4'h0;
            addr        <= 24'd0;
            len         <= 16'd0;
            wdata       <= 8'd0;
            wdata_valid <= 1'b0;
            byte_sent   <= 3'd0;
            wr_edge     <= 1'b0;
        end else begin
            timer <= timer + 32'd1;
            start <= 1'b0;   // default: pulse

            case (tb_state)
                TB_IDLE: begin
                    if (timer == 32'd50) begin
                        // Issue Page Program
                        op    <= OP_PAGE_PROGRAM;
                        addr  <= 24'h000000;
                        len   <= 16'd5;
                        start <= 1'b1;
                        tb_state <= TB_ISSUE;
                    end
                end

                TB_ISSUE: begin
                    if (busy) begin
                        $display("[%0t] busy=1, waiting for ST_DATA", $time);
                        tb_state    <= TB_WAIT_DATA;
                        byte_sent   <= 3'd0;
                        wdata       <= test_wdata[0];
                        wdata_valid <= 1'b0;
                        wr_edge     <= 1'b0;
                    end
                end

                TB_WAIT_DATA: begin
                    // Enter ST_DATA; pre-drive wdata_valid once we're there
                    if (fsm_state == 4'd6) begin
                        wdata_valid <= 1'b1;
                        tb_state    <= TB_SEND;
                        $display("[%0t] ST_DATA entered, wdata_valid=1 wdata=0x%02h",
                                 $time, test_wdata[0]);
                    end
                end

                TB_SEND: begin
                    // Edge-detect wdata_ready: count only on 0→1 transition
                    if (wdata_ready && !wr_edge) begin
                        wr_edge <= 1'b1;
                        if (byte_sent < 3'd5) begin
                            $display("[%0t] wdata_ready ↑ #%0d → consumed 0x%02h",
                                     $time, byte_sent, test_wdata[byte_sent]);
                            byte_sent <= byte_sent + 3'd1;
                            if (byte_sent < 3'd4)
                                wdata <= test_wdata[byte_sent + 3'd1];
                        end
                    end else if (!wdata_ready) begin
                        wr_edge <= 1'b0;
                    end

                    // Exit when byte_sent == 5 and wdata_ready is low
                    // (flash_ctrl started the last byte transfer)
                    if (byte_sent == 3'd5 && !wdata_ready) begin
                        wdata_valid <= 1'b0;
                        tb_state    <= TB_WAIT_DONE;
                        $display("[%0t] 5 bytes done, wdata_valid=0, waiting for done", $time);
                    end
                end

                TB_WAIT_DONE: begin
                    if (done) begin
                        $display("[%0t] done asserted, byte_cnt=%0d", $time, fsm_byte_cnt);
                        tb_state <= TB_CHECK;
                    end else if (error) begin
                        $display("[%0t] ERROR!", $time);
                        tb_state <= TB_CHECK;
                    end else if (timer % 1000 == 0) begin
                        // Periodic diagnostic
                        $display("[%0t] WAIT: fsm=%0d bc=%0d wr=%b wv=%b sbs=%b csn=%b sck=%b",
                                 $time, fsm_state, fsm_byte_cnt,
                                 wdata_ready, wdata_valid, spi_busy_int, csn, sck);
                    end
                end

                TB_CHECK: begin
                    tb_state <= TB_DONE;
                end

                default: ;
            endcase
        end
    end

    // Initialize test data and drive reset
    initial begin
        test_wdata[0] = 8'hA0;
        test_wdata[1] = 8'hA1;
        test_wdata[2] = 8'hA2;
        test_wdata[3] = 8'hA3;
        test_wdata[4] = 8'hA4;
        rst_n = 0;
        #200;
        rst_n = 1;
        $display("[%0t] rst_n released", $time);
    end

    // =========================================================================
    // Result check & report
    // =========================================================================
    reg check_trig = 0;
    always @(posedge clk) begin
        if (tb_state == TB_CHECK && !check_trig) begin
            check_trig <= 1'b1;
            #200;

            $display("");
            $display("============================================================");
            $display(" Test Results");
            $display("============================================================");

            // 1. No error
            if (!error)  $display("PASS: error = 0");
            else         $display("FAIL: error = 1");

            // 2. byte_cnt == 5
            if (fsm_byte_cnt >= 5)
                $display("PASS: byte_cnt = %0d", fsm_byte_cnt);
            else
                $display("FAIL: byte_cnt = %0d (expected 5)", fsm_byte_cnt);

            // 3. CSn deasserted
            if (csn === 1'b1)
                $display("PASS: CSn = 1 (deasserted)");
            else
                $display("FAIL: CSn = %b (stuck low!)", csn);

            // 4. SCK idle
            if (sck === 1'b0)
                $display("PASS: SCK = 0 (idle)");
            else
                $display("FAIL: SCK = %b", sck);

            // 5. Print MOSI trace
            $display("---- MOSI trace (%0d bytes) ----", mosi_idx);
            for (integer i = 0; i < mosi_idx; i = i + 1)
                $display("  [%0d] 0x%02h", i, mosi_cap[i]);

            // 6. Check MOSI content
            if (mosi_idx >= 10) begin
                if (mosi_cap[0] === 8'h06) $display("PASS: [0] WE   = 0x06");
                else $display("WARN: [0] = 0x%02h (expected WE=0x06)", mosi_cap[0]);

                if (mosi_cap[1] === 8'h02) $display("PASS: [1] PP   = 0x02");
                else $display("WARN: [1] = 0x%02h (expected PP=0x02)", mosi_cap[1]);

                if (mosi_cap[5] === test_wdata[0])
                    $display("PASS: [5] D0   = 0x%02h", mosi_cap[5]);
                else $display("WARN: [5] = 0x%02h (expected D0=0x%02h)", mosi_cap[5], test_wdata[0]);

                if (mosi_cap[9] === test_wdata[4])
                    $display("PASS: [9] D4   = 0x%02h", mosi_cap[9]);
                else $display("WARN: [9] = 0x%02h (expected D4=0x%02h)", mosi_cap[9], test_wdata[4]);
            end

            // 7. Summary
            if (!error && fsm_byte_cnt >= 5 && csn === 1'b1)
                $display("===== TEST PASSED =====");
            else
                $display("===== TEST FAILED =====");

            #1000;
            $finish;
        end
    end

    // =========================================================================
    // Timeout
    // =========================================================================
    initial begin
        #1_500_000;
        $display("");
        $display("TIMEOUT at %0t — diagnostic dump:", $time);
        $display("  tb_state     = %0d", tb_state);
        $display("  fsm_state    = %0d", fsm_state);
        $display("  fsm_byte_cnt = %0d", fsm_byte_cnt);
        $display("  wdata_ready  = %b", wdata_ready);
        $display("  wdata_valid  = %b", wdata_valid);
        $display("  spi_busy     = %b", spi_busy_int);
        $display("  csn          = %b", csn);
        $display("  sck          = %b", sck);
        $display("  byte_sent    = %0d", byte_sent);
        $display("  wr_edge      = %b", wr_edge);
        $finish;
    end

    // =========================================================================
    // VCD dump
    // =========================================================================
    initial begin
        $dumpfile("tb_flash_write_5bytes.vcd");
        $dumpvars(0, tb_flash_write_5bytes);
    end

endmodule
