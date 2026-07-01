`timescale 1ns / 1ps
//=============================================================================
// Module : flash_ctrl
// Purpose: W25Q64 / W25Q128 SPI NOR flash controller.
//
// Higher-level "operation" interface:
//   1. Drive op / addr / len. For writes, also provide wdata + wdata_valid.
//   2. Pulse start.
//   3. Wait for busy to assert, then deassert (done).
//   4. For reads, capture rdata on each rdata_valid pulse.
//   5. For writes, ensure wdata is valid on the cycle wdata_ready is high.
//
// Internally, the controller sequences the byte-level SPI transactions
// required by each flash instruction (Write Enable -> command -> address ->
// data -> optional status polling).
//=============================================================================
module flash_ctrl #(
    parameter [15:0] POLL_TIMEOUT = 16'hFFFF  // max status-poll iterations
) (
    input  wire        clk,
    input  wire        rst_n,

    // ---- User command interface ----
    input  wire        start,
    input  wire [3:0]  op,
    input  wire [23:0] addr,
    input  wire [7:0]  wdata,
    input  wire        wdata_valid,
    output reg         wdata_ready,
    input  wire [15:0] len,
    output reg  [7:0]  rdata,
    output reg         rdata_valid,
    output reg         busy,
    output reg         done,
    output reg         error,

    // Last read of Status Register-1 (BUSY/WEL/BP0..2/TB/SEC/SRP0)
    output reg  [7:0]  status_reg1,

    // ---- SPI master interface (byte-level) ----
    output reg         spi_start,
    output reg         spi_csn_hold,
    output reg  [7:0]  spi_tx_data,
    input  wire [7:0]  spi_rx_data,
    input  wire        spi_busy,
    input  wire        spi_done
);

    // -------------------------------------------------------------------------
    // W25Q64 / W25Q128 instruction codes (per datasheet)
    // -------------------------------------------------------------------------
    localparam [7:0] INS_WE       = 8'h06;  // Write Enable
    localparam [7:0] INS_WD       = 8'h04;  // Write Disable
    localparam [7:0] INS_RSR1     = 8'h05;  // Read Status Reg-1
    localparam [7:0] INS_RSR2     = 8'h35;  // Read Status Reg-2
    localparam [7:0] INS_WSR      = 8'h01;  // Write Status Reg
    localparam [7:0] INS_READ     = 8'h03;  // Read Data
    localparam [7:0] INS_FAST_RD  = 8'h0B;  // Fast Read
    localparam [7:0] INS_PP       = 8'h02;  // Page Program
    localparam [7:0] INS_SE       = 8'h20;  // Sector Erase (4 KB)
    localparam [7:0] INS_BE32     = 8'h52;  // Block Erase (32 KB)
    localparam [7:0] INS_BE64     = 8'hD8;  // Block Erase (64 KB)
    localparam [7:0] INS_CE       = 8'hC7;  // Chip Erase
    localparam [7:0] INS_PD       = 8'hB9;  // Power-down
    localparam [7:0] INS_RPD      = 8'hAB;  // Release Power-down
    localparam [7:0] INS_JEDEC_ID = 8'h9F;  // Read JEDEC ID
    localparam [7:0] INS_UID      = 8'h4B;  // Read Unique ID
    localparam [7:0] INS_RSTEN    = 8'h66;  // Reset Enable
    localparam [7:0] INS_RST      = 8'h99;  // Reset
    localparam [7:0] INS_GBULK    = 8'h98;  // Global Block Unlock

    // -------------------------------------------------------------------------
    // User operation codes
    // -------------------------------------------------------------------------
    localparam [3:0] OP_NOP            = 4'h0;
    localparam [3:0] OP_READ_JEDEC_ID  = 4'h1;  // 3 bytes (MFR, TYPE, CAP)
    localparam [3:0] OP_READ_STATUS    = 4'h2;  // 1 byte -> status_reg1
    localparam [3:0] OP_READ_DATA      = 4'h3;  // `len` bytes from `addr`
    localparam [3:0] OP_PAGE_PROGRAM   = 4'h4;  // `len` bytes (1..256)
    localparam [3:0] OP_SECTOR_ERASE   = 4'h5;  // 4 KB
    localparam [3:0] OP_BLOCK_ERASE_32K= 4'h6;  // 32 KB
    localparam [3:0] OP_BLOCK_ERASE_64K= 4'h7;  // 64 KB
    localparam [3:0] OP_CHIP_ERASE     = 4'h8;
    localparam [3:0] OP_POWER_DOWN     = 4'h9;
    localparam [3:0] OP_RELEASE_PD     = 4'hA;
    localparam [3:0] OP_READ_UID       = 4'hB;  // 4 dummy + 8 UID bytes
    localparam [3:0] OP_WRITE_STATUS  = 4'hC;  // WE + 0x01 + 1 byte data → 清保护位
    localparam [3:0] OP_RESET_ENABLE  = 4'hD;  // Reset Enable (0x66)
    localparam [3:0] OP_RESET_DEVICE  = 4'hE;  // Reset Device (0x99)
    localparam [3:0] OP_GBLOCK_UNLOCK= 4'hF;  // Global Block Unlock (0x98)

    // Direction
    localparam [1:0] DIR_NONE  = 2'd0;
    localparam [1:0] DIR_READ  = 2'd1;
    localparam [1:0] DIR_WRITE = 2'd2;

    // States
    localparam [3:0] ST_IDLE        = 4'd0;
    localparam [3:0] ST_TX_WE       = 4'd1;
    localparam [3:0] ST_TX_CMD      = 4'd2;
    localparam [3:0] ST_TX_ADDR3    = 4'd3;
    localparam [3:0] ST_TX_ADDR2    = 4'd4;
    localparam [3:0] ST_TX_ADDR1    = 4'd5;
    localparam [3:0] ST_DATA        = 4'd6;
    localparam [3:0] ST_POLL_TX     = 4'd7;
    localparam [3:0] ST_POLL_DUMMY  = 4'd8;  // send 0xFF to clock out status
    localparam [3:0] ST_POLL_WAIT   = 4'd9;
    localparam [3:0] ST_POLL_GAP    = 4'd11;  // CS 高电平保持 ≥tSHSL, 然后重轮询
    localparam [3:0] ST_DONE        = 4'd10;

    reg [3:0]  state;
    reg [3:0]  op_reg;
    reg [23:0] addr_reg;
    reg [15:0] len_reg;
    reg [15:0] byte_cnt;
    reg [15:0] poll_cnt;
    reg [16:0] gap_cnt;       // ST_POLL_GAP 延时计数器 (最大1ms)
    reg        gap_to_cmd;    // ST_POLL_GAP 后: 0=ST_POLL_TX, 1=ST_TX_CMD
    reg        gap_long;      // (unused: gap now unified to 1µs, kept for compat)

    // Per-operation parameters
    reg        op_need_we;
    reg [7:0]  op_cmd;
    reg        op_has_addr;
    reg [1:0]  op_dir;
    reg        op_need_poll;

    always @(*) begin
        op_need_we   = 1'b0;
        op_cmd       = 8'h00;
        op_has_addr  = 1'b0;
        op_dir       = DIR_NONE;
        op_need_poll = 1'b0;
        case (op_reg)
            OP_READ_JEDEC_ID: begin
                op_cmd = INS_JEDEC_ID;
                op_dir = DIR_READ;
            end
            OP_READ_STATUS: begin
                op_cmd = INS_RSR1;
                op_dir = DIR_READ;
            end
            OP_READ_DATA: begin
                op_cmd      = INS_READ;
                op_has_addr = 1'b1;
                op_dir      = DIR_READ;
            end
            OP_PAGE_PROGRAM: begin
                op_need_we   = 1'b1;
                op_cmd       = INS_PP;
                op_has_addr  = 1'b1;
                op_dir       = DIR_WRITE;
                op_need_poll = 1'b1;
            end
            OP_SECTOR_ERASE: begin
                op_need_we   = 1'b1;
                op_cmd       = INS_SE;
                op_has_addr  = 1'b1;
                op_need_poll = 1'b1;
            end
            OP_BLOCK_ERASE_32K: begin
                op_need_we   = 1'b1;
                op_cmd       = INS_BE32;
                op_has_addr  = 1'b1;
                op_need_poll = 1'b1;
            end
            OP_BLOCK_ERASE_64K: begin
                op_need_we   = 1'b1;
                op_cmd       = INS_BE64;
                op_has_addr  = 1'b1;
                op_need_poll = 1'b1;
            end
            OP_CHIP_ERASE: begin
                op_need_we   = 1'b1;
                op_cmd       = INS_CE;
                op_need_poll = 1'b1;
            end
            OP_POWER_DOWN: op_cmd = INS_PD;
            OP_RELEASE_PD: op_cmd = INS_RPD;
            OP_READ_UID: begin
                op_cmd = INS_UID;
                op_dir = DIR_READ;
            end
            OP_WRITE_STATUS: begin
                op_need_we   = 1'b1;
                op_cmd       = INS_WSR;
                op_dir       = DIR_WRITE;
                op_need_poll = 1'b1;
            end
            OP_RESET_ENABLE: begin
                op_cmd = INS_RSTEN;
            end
            OP_RESET_DEVICE: begin
                op_cmd = INS_RST;
            end
            OP_GBLOCK_UNLOCK: begin
                op_need_we = 1'b1;
                op_cmd     = INS_GBULK;
            end
            default: ;
        endcase
    end

    // State that follows the command byte
    wire [3:0] state_after_cmd = op_has_addr              ? ST_TX_ADDR3 :
                                 (op_dir == DIR_READ)     ? ST_DATA     :
                                 (op_dir == DIR_WRITE)    ? ST_DATA     :
                                 op_need_poll             ? ST_POLL_TX  :
                                                           ST_DONE;

    // -------------------------------------------------------------------------
    // Main FSM
    //
    // The flash controller's flow for a typical write/erase op is:
    //   ST_TX_WE  -> ST_TX_CMD -> ST_TX_ADDR3 -> ST_TX_ADDR2 -> ST_TX_ADDR1
    //             -> ST_DATA (write `len` bytes with wdata handshake)
    //             -> ST_POLL_TX/ST_POLL_WAIT (poll RSR1.BUSY until 0)
    //             -> ST_DONE
    //
    // For a typical read op:
    //   ST_TX_CMD -> ST_TX_ADDR3 -> ST_TX_ADDR2 -> ST_TX_ADDR1
    //             -> ST_DATA (read `len` bytes)
    //             -> ST_DONE
    //
    // Each TX sub-state pulses spi_start and waits for spi_done.
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            op_reg       <= OP_NOP;
            addr_reg     <= 24'd0;
            len_reg      <= 16'd0;
            byte_cnt     <= 16'd0;
            poll_cnt     <= 16'd0;
            gap_cnt      <= 14'd0;
            gap_to_cmd   <= 1'b0;
            gap_long     <= 1'b0;
            spi_start    <= 1'b0;
            spi_csn_hold <= 1'b0;
            spi_tx_data  <= 8'd0;
            busy         <= 1'b0;
            done         <= 1'b0;
            error        <= 1'b0;
            rdata        <= 8'd0;
            rdata_valid  <= 1'b0;
            wdata_ready  <= 1'b0;
            status_reg1  <= 8'h00;
        end else begin
            // clear single-cycle pulses by default
            done        <= 1'b0;
            rdata_valid <= 1'b0;
            wdata_ready <= 1'b0;
            spi_start   <= 1'b0;
            // Default: keep CSn low between bytes during the multi-byte
            // cmd+addr+data transaction. States that should release CSn
            // (idle, each individual poll-byte transaction, done) override
            // below.
            spi_csn_hold <= 1'b1;

            case (state)
                // ------------------------------------------------------------
                ST_IDLE: begin
                    busy         <= 1'b0;
                    spi_csn_hold <= 1'b0;  // idle: CSn deasserted
                    if (start) begin
                        op_reg   <= op;
                        addr_reg <= addr;
                        // For fixed-length read ops, override `len`.
                        case (op)
                            OP_READ_JEDEC_ID: len_reg <= 16'd3;
                            OP_READ_STATUS:   len_reg <= 16'd1;
                            OP_READ_UID:      len_reg <= 16'd12;  // 4 dummy + 8 UID
                            default:          len_reg <= len;
                        endcase
                        byte_cnt <= 16'd0;
                        poll_cnt <= 16'd0;
                        busy     <= 1'b1;
                        error    <= 1'b0;
                        if (op_need_we_for_op(op)) state <= ST_TX_WE;
                        else                       state <= ST_TX_CMD;
                    end
                end

                // ------------------------------------------------------------
                // Each TX sub-state: set the byte, pulse spi_start, wait done.
                // ------------------------------------------------------------
                ST_TX_WE: begin
                    // WE 必须是独立 SPI 事务: CS↓→0x06→CS↑, 否则 Flash 不锁存 WEL
                    spi_csn_hold <= 1'b0;
                    if (!spi_busy) begin
                        spi_start   <= 1'b1;
                        spi_tx_data <= INS_WE;
                    end else if (spi_done) begin
                        // 等 CS 释放并保持高 ≥ tSHSL(50ns) → 跳转 ST_TX_CMD
                        state      <= ST_POLL_GAP;
                        gap_cnt    <= 17'd0;
                        gap_to_cmd <= 1'b1;
                    end
                end

                ST_TX_CMD: begin
                    if (!spi_busy) begin
                        spi_start   <= 1'b1;
                        spi_tx_data <= op_cmd;
                    end else if (spi_done) begin
                        state <= state_after_cmd;
                    end
                end

                ST_TX_ADDR3: begin
                    if (!spi_busy) begin
                        spi_start   <= 1'b1;
                        spi_tx_data <= addr_reg[23:16];
                    end else if (spi_done) begin
                        state <= ST_TX_ADDR2;
                    end
                end

                ST_TX_ADDR2: begin
                    if (!spi_busy) begin
                        spi_start   <= 1'b1;
                        spi_tx_data <= addr_reg[15:8];
                    end else if (spi_done) begin
                        state <= ST_TX_ADDR1;
                    end
                end

                ST_TX_ADDR1: begin
                    if (!spi_busy) begin
                        spi_start   <= 1'b1;
                        spi_tx_data <= addr_reg[7:0];
                    end else if (spi_done) begin
                        byte_cnt <= 16'd0;
                        state    <= ST_DATA;
                    end
                end

                // ------------------------------------------------------------
                // Data phase: read `len_reg` bytes or write `len_reg` bytes.
                // ------------------------------------------------------------
                ST_DATA: begin
                    if (byte_cnt >= len_reg) begin
                        // 写/擦除后 CS 必须先拉高 (≥tSHSL), 再开始轮询
                        if (op_need_poll) begin
                            state      <= ST_POLL_GAP;
                            gap_cnt    <= 17'd0;
                            gap_to_cmd <= 1'b0;
                        end else begin
                            state <= ST_DONE;
                        end
                    end else if (op_dir == DIR_READ) begin
                        if (!spi_busy) begin
                            spi_start   <= 1'b1;
                            spi_tx_data <= 8'hFF;
                        end else if (spi_done) begin
                            rdata       <= spi_rx_data;
                            rdata_valid <= 1'b1;
                            byte_cnt    <= byte_cnt + 1'b1;
                        end
                    end else begin
                        // DIR_WRITE: 4-phase handshake on wdata_ready.
                        if (spi_done) begin
                            byte_cnt    <= byte_cnt + 1'b1;
                            wdata_ready <= 1'b1;
                        end else if (wdata_ready && wdata_valid && !spi_start) begin
                            // ★ !spi_start guard: after B fires, spi_start stays 1
                            //   for 1 cycle (until default-clear), blocking C from
                            //   re-asserting wdata_ready before spi_busy rises
                            //   (1-cycle pipeline on start→busy).
                            spi_start   <= 1'b1;
                            spi_tx_data <= wdata;
                            wdata_ready <= 1'b0;
                        end else if (wdata_valid && !wdata_ready && !spi_busy && !spi_start) begin
                            // ★ wdata 预驱: 数据已就绪, 先握手再发送
                            wdata_ready <= 1'b1;
                        end else if (!wdata_valid) begin
                            wdata_ready <= 1'b1;
                        end
                    end
                end

                // ------------------------------------------------------------
                // Status poll: send RSR1 repeatedly until BUSY (bit 0) = 0.
                // After every write/erase, the flash sets BUSY=1. We send
                // RSR1, get a status byte, and on the next iteration we
                // re-send RSR1 if BUSY=1, or finish if BUSY=0.
                // ------------------------------------------------------------
                ST_POLL_TX: begin
                    // RSR1 + dummy byte 必须是一个连续 SPI 事务, CSn 不能中间释放
                    spi_csn_hold <= 1'b1;  // 保持 CSn 低, 等待 ST_POLL_DUMMY 的 dummy 字节
                    if (!spi_busy) begin
                        spi_start   <= 1'b1;
                        spi_tx_data <= INS_RSR1;
                        // Increase poll counter when we actually start a poll.
                        poll_cnt    <= poll_cnt + 1'b1;
                    end else if (spi_done) begin
                        // The byte we just sent was the INS_RSR1 cmd. The flash
                        // model will drive the status response on the NEXT byte
                        // (the dummy we send in ST_POLL_DUMMY).
                        state <= ST_POLL_DUMMY;
                    end
                end

                ST_POLL_DUMMY: begin
                    spi_csn_hold <= 1'b0;  // dummy 字节后释放 CSn, 结束本次 RSR1 事务
                    if (!spi_busy) begin
                        spi_start   <= 1'b1;
                        spi_tx_data <= 8'hFF;  // dummy to clock out status
                    end else if (spi_done) begin
                        status_reg1 <= spi_rx_data;
                        state       <= ST_POLL_WAIT;
                    end
                end

                ST_POLL_WAIT: begin
                    spi_csn_hold <= 1'b0;  // release CSn between poll iterations
                    // BUSY bit is bit 0 of status_reg1.
                    if (status_reg1[0] == 1'b0) begin
                        // Not busy -> op complete
                        state <= ST_DONE;
                    end else if (poll_cnt >= POLL_TIMEOUT) begin
                        // Timed out
                        error <= 1'b1;
                        state <= ST_DONE;
                    end else begin
                        // Re-poll: 先进入 ST_POLL_GAP 确保 CS 高 ≥ tSHSL (50ns)
                        state      <= ST_POLL_GAP;
                        gap_cnt    <= 17'd0;
                        gap_to_cmd <= 1'b0;
                    end
                end

                // ------------------------------------------------------------
                // ST_POLL_GAP: CSn 保持高 ≥1µs, 满足 tSHSL (50ns) 后继续
                // ------------------------------------------------------------
                ST_POLL_GAP: begin
                    spi_csn_hold <= 1'b0;  // CSn 保持高
                    // 统一 100 cycles = 1µs (tSHSL=50ns, 20x margin)
                    if (gap_cnt == 17'd99) begin
                        if (gap_to_cmd)
                            state <= ST_TX_CMD;   // WE 后 → 发操作命令
                        else
                            state <= ST_POLL_TX;  // 命令后 → 轮询状态
                    end else begin
                        gap_cnt <= gap_cnt + 17'd1;
                    end
                end

                // ------------------------------------------------------------
                ST_DONE: begin
                    done         <= 1'b1;
                    busy         <= 1'b0;
                    spi_csn_hold <= 1'b0;  // release CSn
                    state        <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // Synthesizable helper: returns 1 if the given op requires a Write Enable
    // cycle first.
    function op_needs_we(input [3:0] op_in);
        reg r;
        begin
            case (op_in)
                OP_PAGE_PROGRAM, OP_SECTOR_ERASE, OP_BLOCK_ERASE_32K,
                OP_BLOCK_ERASE_64K, OP_CHIP_ERASE, OP_WRITE_STATUS,
                OP_GBLOCK_UNLOCK: r = 1'b1;
                default:                            r = 1'b0;
            endcase
            op_needs_we = r;
        end
    endfunction

    // Wrapper used in ST_IDLE for the "needs WE?" decision.
    function op_need_we_for_op(input [3:0] op_in);
        begin
            op_need_we_for_op = op_needs_we(op_in);
        end
    endfunction

endmodule
