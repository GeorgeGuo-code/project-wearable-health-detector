`timescale 1ns / 1ps
//=============================================================================
// Module : tb_record
// Purpose: Multi-record W25Q64 driver test.
//
// Record layout (4 bytes per record, big-endian for the 16-bit field):
//   offset+0 : data16[15:8]   (high byte)
//   offset+1 : data16[7:0]    (low byte)
//   offset+2 : data8_a
//   offset+3 : data8_b
//
// Test sequence:
//   1. Read JEDEC ID  (sanity check, no assertion on values)
//   2. Sector Erase at 0x000000
//   3. Page Program 8 records (32 bytes) at 0x000000
//   4. Read 32 bytes from 0x000000
//   5. Decode each record and check the data16/data8_a/data8_b fields
//      match the values we wrote.
//=============================================================================
module tb_record;

    reg         clk;
    reg         rst_n;

    // User-side signals
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

    // Operation codes (mirror flash_ctrl.v)
    localparam [3:0] OP_READ_JEDEC_ID  = 4'h1;
    localparam [3:0] OP_READ_STATUS    = 4'h2;
    localparam [3:0] OP_READ_DATA      = 4'h3;
    localparam [3:0] OP_PAGE_PROGRAM   = 4'h4;
    localparam [3:0] OP_SECTOR_ERASE   = 4'h5;
    localparam [3:0] OP_CHIP_ERASE     = 4'h8;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    flash_top #(
        .CLK_DIV(4)   // 100 MHz / 4 = 25 MHz SCK
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

    // -------------------------------------------------------------------------
    // Test parameters
    // -------------------------------------------------------------------------
    localparam integer N_RECORDS    = 8;
    localparam integer BYTES_PER   = 4;
    localparam integer TOTAL_BYTES = N_RECORDS * BYTES_PER;  // 32
    localparam [23:0] BASE_ADDR    = 24'h000000;

    // Reference arrays (per record). We compute expected values for each
    // record index, then write/read back and compare.
    reg [15:0] exp_d16 [0:N_RECORDS-1];
    reg [7:0]  exp_a  [0:N_RECORDS-1];
    reg [7:0]  exp_b  [0:N_RECORDS-1];
    reg [7:0]  tx_buf [0:TOTAL_BYTES-1];   // bytes-to-write (built by build_payload)
    reg [7:0]  rx_buf [0:TOTAL_BYTES-1];

    integer i;

    // -------------------------------------------------------------------------
    // Tasks (mirrored from tb_flash_top.v, kept here so this testbench
    // is self-contained).
    // -------------------------------------------------------------------------
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
            if (error) $display("ERROR: op=%h reported error", op_i);
        end
    endtask

    // Page program `len` bytes from the module-level tx_buf. The wdata
    // driver is forked off the main thread and updates wdata once per
    // SPI-byte period (38 cycles) so the controller sees fresh data each
    // byte. Reading from a module-level array (rather than passing a
    // packed vector as a task argument) avoids quirks with variable
    // part-selects on packed inputs in iverilog.
    task page_program_buf(
        input [23:0] addr_i,
        input [15:0] len_i
    );
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
                        wdata = tx_buf[k];
                        repeat (38) @(posedge clk);
                    end
                    wdata_valid = 1'b0;
                end
            join
        end
    endtask

    // Read N bytes into rx_buf (caller is responsible for sizing).
    task read_bytes(
        input [23:0] addr_i,
        input [15:0] len_i
    );
        integer k;
        begin
            fork
                begin
                    issue_cmd(OP_READ_DATA, addr_i, len_i);
                end
                begin
                    for (k = 0; k < len_i; k = k + 1) begin
                        while (!rdata_valid) @(posedge clk);
                        @(posedge clk);
                        rx_buf[k] = rdata;
                    end
                end
            join
        end
    endtask

    task read_jedec_id;
        reg [7:0] mfr, typ, cap;
        begin
            fork
                begin
                    issue_cmd(OP_READ_JEDEC_ID, 24'd0, 16'd0);
                end
                begin
                    while (!rdata_valid) @(posedge clk); @(posedge clk); mfr = rdata;
                    while (!rdata_valid) @(posedge clk); @(posedge clk); typ = rdata;
                    while (!rdata_valid) @(posedge clk); @(posedge clk); cap = rdata;
                end
            join
            $display("JEDEC ID: MFR=0x%02h TYPE=0x%02h CAP=0x%02h", mfr, typ, cap);
        end
    endtask

    // -------------------------------------------------------------------------
    // Build tx_buf[] directly from exp_* arrays. Per record j:
    //   tx_buf[j*4 + 0] = exp_d16[j][15:8]   (data16 high byte)
    //   tx_buf[j*4 + 1] = exp_d16[j][7:0]    (data16 low byte)
    //   tx_buf[j*4 + 2] = exp_a[j]
    //   tx_buf[j*4 + 3] = exp_b[j]
    // -------------------------------------------------------------------------
    task build_payload;
        integer j;
        begin
            for (j = 0; j < N_RECORDS; j = j + 1) begin
                tx_buf[j*4 + 0] = exp_d16[j][15:8];
                tx_buf[j*4 + 1] = exp_d16[j][7:0];
                tx_buf[j*4 + 2] = exp_a[j];
                tx_buf[j*4 + 3] = exp_b[j];
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Decode rx_buf and verify against exp_* arrays. Print per-record results
    // and a single PASS/FAIL summary. Returns 0 on success, nonzero on fail.
    // -------------------------------------------------------------------------
    integer verify_status;
    task verify_records;
        integer j;
        reg [15:0] got_d16;
        reg [7:0]  got_a, got_b;
        begin
            verify_status = 0;
            for (j = 0; j < N_RECORDS; j = j + 1) begin
                got_d16 = {rx_buf[j*4 + 0], rx_buf[j*4 + 1]};
                got_a   =  rx_buf[j*4 + 2];
                got_b   =  rx_buf[j*4 + 3];
                $display("  rec[%0d] @ 0x%06h: d16=0x%04h a=0x%02h b=0x%02h  (expected d16=0x%04h a=0x%02h b=0x%02h)",
                         j, BASE_ADDR + j*4,
                         got_d16, got_a, got_b,
                         exp_d16[j], exp_a[j], exp_b[j]);
                if (got_d16 !== exp_d16[j] ||
                    got_a   !== exp_a[j]   ||
                    got_b   !== exp_b[j]) begin
                    $display("    FAIL on record %0d", j);
                    verify_status = verify_status + 1;
                end
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Main stimulus
    // -------------------------------------------------------------------------
    initial begin
        $display("=================================================");
        $display("W25Q64 record-mode test (8 records, 4B each)");
        $display("=================================================");

        // Reset
        start = 0; op = 0; addr = 0; wdata = 0; wdata_valid = 0; len = 0;
        rst_n = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (20) @(posedge clk);

        // ---- Sanity: JEDEC ID ----
        read_jedec_id();

        // ---- Generate test pattern ----
        // record i: d16 = 0x1000 + i, a = 0xA0 + i, b = 0xB0 + i
        for (i = 0; i < N_RECORDS; i = i + 1) begin
            exp_d16[i] = 16'h1000 + i[15:0];
            exp_a[i]   = 8'hA0 + i[7:0];
            exp_b[i]   = 8'hB0 + i[7:0];
        end
        build_payload();

        $display("Pattern: rec[i] = { d16=0x%04h+i, a=0x%02h+i, b=0x%02h+i }",
                 16'h1000, 8'hA0, 8'hB0);
        $display("Total payload: %0d bytes at 0x%06h", TOTAL_BYTES, BASE_ADDR);

        // ---- 1. Erase sector first so un-flashed bits are 0xFF ----
        issue_cmd(OP_SECTOR_ERASE, BASE_ADDR, 16'd0);
        $display("Sector Erase at 0x%06h done", BASE_ADDR);

        // ---- 2. Page Program 32 bytes ----
        page_program_buf(BASE_ADDR, TOTAL_BYTES[15:0]);
        $display("Page Program %0d bytes done", TOTAL_BYTES);

        // ---- 3. Read 32 bytes back ----
        read_bytes(BASE_ADDR, TOTAL_BYTES[15:0]);
        $display("Read %0d bytes done", TOTAL_BYTES);

        // ---- 4. Verify ----
        verify_records();
        if (verify_status == 0) begin
            $display("=================================================");
            $display("PASS: all %0d records match", N_RECORDS);
            $display("=================================================");
        end else begin
            $display("=================================================");
            $display("FAIL: %0d record(s) mismatched", verify_status);
            $display("=================================================");
        end

        $finish;
    end

    // Watchdog
    initial begin
        #200_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
