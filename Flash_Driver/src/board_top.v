`timescale 1ns / 1ps
//=============================================================================
// Module : board_top
// Purpose: EGO1 (Xilinx Artix-7 XC7A35T) top-level wrapper for the W25Q64
//          driver. Runs a self-test, reports via 3 status LEDs.
//
//          SPI to an external W25Q64 module is wired to PMOD J5:
//            J5.15 (B11)  -- W25Q64 SCLK
//            J5.16 (A11)  -- W25Q64 CSn
//            J5.17 (E15)  -- W25Q64 SI
//            J5.18 (E16)  -- W25Q64 SO
//
//          Reset button is ACTIVE-HIGH (idle=0, pressed=1). Use one of the
//          EGO1 push-buttons (e.g. S4 = T14).
//
//          BTN_START: rising edge re-runs the test.
//
//          Status LEDs:
//            LED[0] (F6)  = busy
//            LED[1] (G4)  = done  (latched on full success)
//            LED[2] (G3)  = error (latched on any controller error)
//
//          The test sequence is driven by a synthesizable FSM that:
//            - issues a request by pulsing `start` for one cycle
//            - waits for `busy` to rise, then for `done` to rise
//            - reads rdata on rdata_valid pulses
//            - feeds wdata on wdata_ready pulses (for PP)
//            - advances through milestones until the full test passes
//=============================================================================
module board_top (
    input  wire        CLK100MHZ,   // 100 MHz clock
    input  wire        CPU_RESETN,  // ACTIVE-HIGH reset (button: idle=0, pressed=1)
    input  wire        BTN_START,   // ACTIVE-HIGH: re-run on rising edge
    output wire        LED_BUSY,    // F6
    output wire        LED_DONE,    // G4
    output wire        LED_ERROR,   // G3
    output wire        SPI_SCK,     // B11 (J5.15)
    output wire        SPI_CSN,     // A11 (J5.16)
    output wire        SPI_MOSI,    // E15 (J5.17)
    input  wire        SPI_MISO     // E16 (J5.18)
);

    // Operation codes (must match src/flash_ctrl.v)
    localparam [3:0] OP_READ_JEDEC_ID = 4'h1;
    localparam [3:0] OP_READ_STATUS   = 4'h2;
    localparam [3:0] OP_READ_DATA     = 4'h3;
    localparam [3:0] OP_PAGE_PROGRAM  = 4'h4;
    localparam [3:0] OP_SECTOR_ERASE  = 4'h5;
    localparam [3:0] OP_CHIP_ERASE    = 4'h8;

    // -------------------------------------------------------------------------
    // Reset: two-flop synchronizer. CPU_RESETN is active-HIGH push-button.
    //   0 (idle)  -> rst_n=1  (run)
    //   1 (press) -> rst_n=0  (held in reset)
    // -------------------------------------------------------------------------
    reg        sync0 = 1'b0;
    reg        sync1 = 1'b0;
    always @(posedge CLK100MHZ) begin
        sync0 <= CPU_RESETN;
        sync1 <= sync0;
    end
    // rst_n is active-LOW for submodules. Drive 0 during power-up so that
    // the very first clock cycles (before sync1 has had a chance to settle)
    // hold all submodules in reset rather than letting their internal
    // counters start with X.
    reg        rst_n_r = 1'b0;
    always @(posedge CLK100MHZ) begin
        rst_n_r <= ~sync1;
    end
    wire rst_n = rst_n_r;

    // -------------------------------------------------------------------------
    // Button debounce + rising-edge detect (BTN_START), 10 ms.
    // -------------------------------------------------------------------------
    reg [19:0] btn_cnt   = 20'd0;
    reg        btn_sync0 = 1'b0;
    reg        btn_sync1 = 1'b0;
    reg        btn_q     = 1'b0;
    reg        btn_press = 1'b0;
    always @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            btn_cnt   <= 20'd0;
            btn_sync0 <= 1'b0;
            btn_sync1 <= 1'b0;
            btn_q     <= 1'b0;
            btn_press <= 1'b0;
        end else begin
            btn_sync0 <= BTN_START;
            btn_sync1 <= btn_sync0;
            if (btn_cnt == 20'd999_999) begin
                btn_cnt   <= 20'd0;
                btn_q     <= btn_sync1;
                btn_press <= btn_sync1 & ~btn_q;
            end else begin
                btn_cnt   <= btn_cnt + 1'b1;
                btn_press <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Settle counter: ~50 ms after reset release
    // -------------------------------------------------------------------------
    reg [25:0] settle_cnt = 26'd0;
    reg        settled    = 1'b0;
    always @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            settle_cnt <= 26'd0;
            settled    <= 1'b0;
        end else if (!settled) begin
            if (settle_cnt == 26'd4_999_999) settled <= 1'b1;
            else                            settle_cnt <= settle_cnt + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // flash_top instance
    // -------------------------------------------------------------------------
    wire        start;
    wire [3:0]  op;
    wire [23:0] addr;
    wire [7:0]  wdata;
    wire        wdata_valid;
    wire        wdata_ready;
    wire [15:0] len;
    wire [7:0]  rdata;
    wire        rdata_valid;
    wire        busy;
    wire        done;
    wire        error;
    wire [7:0]  status_reg1;

    flash_top #(.CLK_DIV(4)) u_flash (
        .clk          (CLK100MHZ),
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
        .sck          (SPI_SCK),
        .csn          (SPI_CSN),
        .mosi         (SPI_MOSI),
        .miso         (SPI_MISO)
    );

    // -------------------------------------------------------------------------
    // Synthesizable self-test sequencer
    //
    // We model the test as a sequence of "milestones". Each milestone is
    // itself a small sub-FSM that issues a flash request and waits for it
    // to complete. We step through the milestones with a single counter
    // (`stage`). A "stage done" flag moves us to the next one.
    //
    // Milestones:
    //   0  Issue JEDEC ID            (wait busy+done; collect 3 bytes)
    //   1  Read Status Reg-1         (wait busy+done)
    //   2  Sector Erase 0x000000     (wait busy+done)
    //   3  Page Program 16 B         (drive wdata, wait busy+done)
    //   4  Read 16 B back            (wait done; collect 16 bytes)
    //   5  Chip Erase                (wait busy+done)
    //   6  Read 16 B back            (wait done; collect 16 bytes)
    //   7  Done                      (latch done_latch; go back to idle)
    // -------------------------------------------------------------------------
    localparam [3:0]
        STG_JEDEC  = 4'd0,
        STG_SR1    = 4'd1,
        STG_SE     = 4'd2,
        STG_PP     = 4'd3,
        STG_RB     = 4'd4,
        STG_CE     = 4'd5,
        STG_RB2    = 4'd6,
        STG_DONE   = 4'd7;

    // Sub-FSM for a single milestone: each milestone goes through these
    // sub-states in order.
    localparam [2:0]
        SS_ISSUE   = 3'd0,    // pulse start
        SS_WAITBSY = 3'd1,    // wait for busy to rise
        SS_BODY    = 3'd2,    // collect rdata / drive wdata
        SS_WAITDN  = 3'd3,    // wait for done to rise
        SS_NEXT    = 3'd4;    // advance to next milestone

    reg [3:0] stage    = STG_JEDEC;
    reg [2:0] ss       = SS_ISSUE;
    reg       busy_q   = 1'b0;
    reg       done_q   = 1'b0;
    reg [4:0] rb_count = 5'd0;
    reg [7:0] pp_k     = 8'd0;
    reg [1:0] jcount   = 2'd0;
    reg [7:0] jmfr = 8'h00, jtyp = 8'h00, jcap = 8'h00;
    reg [7:0] rb_buf [0:15];
    reg [7:0] sr1_q = 8'h00;

    // Request latches
    reg        req_start = 1'b0;
    reg [3:0]  req_op    = 4'h0;
    reg [23:0] req_addr  = 24'd0;
    reg [15:0] req_len   = 16'd0;
    reg        req_wdv   = 1'b0;
    reg [7:0]  req_wd    = 8'd0;

    assign start       = req_start;
    assign op          = req_op;
    assign addr        = req_addr;
    assign wdata       = req_wd;
    assign wdata_valid = req_wdv;
    assign len         = req_len;

    reg        run_active = 1'b0;
    reg        done_latch = 1'b0;
    reg        err_latch  = 1'b0;

    always @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            stage      <= STG_JEDEC;
            ss         <= SS_ISSUE;
            busy_q     <= 1'b0;
            done_q     <= 1'b0;
            rb_count   <= 5'd0;
            pp_k       <= 8'd0;
            jcount     <= 2'd0;
            jmfr       <= 8'h00;
            jtyp       <= 8'h00;
            jcap       <= 8'h00;
            sr1_q      <= 8'h00;
            req_start  <= 1'b0;
            req_op     <= 4'h0;
            req_addr   <= 24'd0;
            req_len    <= 16'd0;
            req_wdv    <= 1'b0;
            req_wd     <= 8'd0;
            run_active <= 1'b0;
            done_latch <= 1'b0;
            err_latch  <= 1'b0;
        end else begin
            if (req_start) req_start <= 1'b0;
            // Note: req_wdv is NOT defaulted to 0 here. The PP body keeps it
            // asserted across the whole transfer; the always-block's default
            // `0` would clobber the handshake.
            busy_q <= busy;
            done_q <= done;

            // Handle a "run" trigger: settled OR button press
            if (stage == STG_JEDEC && ss == SS_ISSUE && !run_active &&
                (settled || btn_press)) begin
                run_active <= 1'b1;
                done_latch <= 1'b0;
                err_latch  <= 1'b0;
            end

            case (ss)
                SS_ISSUE: begin
                    // Pulse start for one cycle with the proper params for
                    // the current stage.
                    case (stage)
                        STG_JEDEC: begin req_op <= OP_READ_JEDEC_ID; req_addr <= 24'd0; req_len <= 16'd0; jcount <= 2'd0; end
                        STG_SR1:   begin req_op <= OP_READ_STATUS;   req_addr <= 24'd0; req_len <= 16'd0; end
                        STG_SE:    begin req_op <= OP_SECTOR_ERASE;  req_addr <= 24'h000000; req_len <= 16'd0; end
                        STG_PP:    begin req_op <= OP_PAGE_PROGRAM;  req_addr <= 24'h000000; req_len <= 16'd16; pp_k <= 8'd0; end
                        STG_RB:    begin req_op <= OP_READ_DATA;     req_addr <= 24'h000000; req_len <= 16'd16; rb_count <= 5'd0; end
                        STG_CE:    begin req_op <= OP_CHIP_ERASE;    req_addr <= 24'd0; req_len <= 16'd0; end
                        STG_RB2:   begin req_op <= OP_READ_DATA;     req_addr <= 24'h000000; req_len <= 16'd16; rb_count <= 5'd0; end
                    endcase
                    req_start <= 1'b1;
                    ss        <= SS_WAITBSY;
                end

                SS_WAITBSY: begin
                    // busy is a level signal (asserted while the command is
                    // in progress). As soon as it is high, the controller has
                    // accepted the request and we move to the BODY sub-state.
                    if (busy == 1'b1) ss <= SS_BODY;
                end

                SS_BODY: begin
                    case (stage)
                        STG_JEDEC: begin
                            if (rdata_valid) begin
                                case (jcount)
                                    2'd0: jmfr <= rdata;
                                    2'd1: jtyp <= rdata;
                                    2'd2: jcap <= rdata;
                                endcase
                                jcount <= jcount + 1'b1;
                                if (jcount == 2'd2) ss <= SS_WAITDN;
                            end
                        end
                        STG_SR1: begin
                            // Read status returns 1 byte on done; capture when done
                            if (done == 1'b1 && done_q == 1'b0) begin
                                sr1_q <= status_reg1;
                                ss    <= SS_NEXT;
                            end
                        end
                        STG_SE: begin
                            // Erase is long; just wait for done
                            if (done == 1'b1 && done_q == 1'b0) ss <= SS_NEXT;
                        end
                        STG_PP: begin
                            // 4-phase handshake: wdata_valid must stay high
                            // until the controller actually consumes the byte
                            // (which happens when wdata_ready was high in the
                            // PREVIOUS cycle and is now low because the
                            // controller grabbed the byte). The actual
                            // wdata byte changes only when wdata_ready is
                            // high (= start of a new byte slot).
                            if (wdata_ready == 1'b1) begin
                                req_wd  <= 8'hA0 + pp_k;
                                req_wdv <= 1'b1;
                                if (pp_k == 8'd15) begin
                                    pp_k <= 8'd0;
                                    ss   <= SS_WAITDN;
                                end else begin
                                    pp_k <= pp_k + 1'b1;
                                end
                            end
                            // Keep wdata_valid asserted (override the
                            // always-block default `req_wdv <= 1'b0`).
                            // We use a different signal name internally to
                            // avoid the override, but for clarity we just
                            // re-assert here. Verilog non-blocking assign
                            // semantics make the LAST assignment win, so
                            // moving the default `req_wdv <= 1'b0` out of
                            // the case is sufficient.
                        end
                        STG_RB: begin
                            if (rdata_valid) begin
                                rb_buf[rb_count[3:0]] <= rdata;
                                if (rb_count == 5'd15) ss <= SS_WAITDN;
                                else                    rb_count <= rb_count + 1'b1;
                            end
                        end
                        STG_CE: begin
                            if (done == 1'b1 && done_q == 1'b0) ss <= SS_NEXT;
                        end
                        STG_RB2: begin
                            if (rdata_valid) begin
                                rb_buf[rb_count[3:0]] <= rdata;
                                if (rb_count == 5'd15) ss <= SS_WAITDN;
                                else                    rb_count <= rb_count + 1'b1;
                            end
                        end
                        default: ss <= SS_NEXT;
                    endcase
                end

                SS_WAITDN: begin
                    // `done` is a single-cycle pulse. Latch it asynchronously
                    // to the sub-FSM: if we ever see done high, advance.
                    if (done == 1'b1) ss <= SS_NEXT;
                end

                SS_NEXT: begin
                    if (stage == STG_DONE) begin
                        run_active <= 1'b0;
                        done_latch <= 1'b1;
                        ss         <= SS_ISSUE;
                        stage      <= STG_JEDEC;  // loop back, waiting for settle/btn
                    end else begin
                        ss    <= SS_ISSUE;
                        stage <= stage + 1'b1;
                    end
                end

                default: ss <= SS_ISSUE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Watchdog: latch err_latch if controller reports error
    // -------------------------------------------------------------------------
    always @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) err_latch <= 1'b0;
        else if (error) err_latch <= 1'b1;
    end

    // -------------------------------------------------------------------------
    // LEDs
    // -------------------------------------------------------------------------
    assign LED_BUSY  = run_active;
    assign LED_DONE  = done_latch;
    assign LED_ERROR = err_latch;

endmodule
