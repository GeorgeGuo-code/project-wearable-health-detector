`timescale 1ns / 1ps
// =============================================================================
// flash_op_seq — flash 操作序列器
//   把 flash_top 的命令式接口封装成 3 个用户流程:
//     1) 上电自动 read 4 数据字节 + 1 magic 字节 -> 写入 load_*
//     2) session_end_int ↑ -> sector_erase + page_program (保存)
//     3) flash_view_en && CONFIRM↑ -> 进入 DEL_REQ, 再次 CONFIRM 触发 erase
//
// 状态机 (单一时钟域, 单 always 块):
//   S_IDLE         默认; 监听 session_end_int 和 (flash_view_en && CONFIRM↑)
//   S_LOAD_INIT    rst_n 释放后自动, 15ms 计时
//   S_LOAD_ISSUE   拉 1 周期 start (READ_DATA, addr=0, len=5)
//   S_LOAD_WAIT    收 5 字节 rdata
//   S_DEL_REQ      等待 CONFIRM/MODE
//   S_ERASE_ISSUE  拉 1 周期 start (SECTOR_ERASE, addr=0)
//   S_ERASE_WAIT   等待 done
//   S_PROG_ISSUE   拉 1 周期 start (PAGE_PROGRAM, addr=0, len=5)
//   S_PROG_SEND    5 字节 wdata 握手
//   S_PROG_WAIT    等待 done
//   S_SHOW_MSG     1.5s 提示, 然后回 S_IDLE
//
// 任意 op 完成时 error=1 -> SHOW_MSG (Error)
//
// 写数据布局 (5 字节, 0x000000 起始):
//   [0] step_count[7:0]    little-endian
//   [1] step_count[15:8]
//   [2] avg_cadence
//   [3] avg_hr
//   [4] 8'hA5  (magic, 写死; load 时校验)
//
// 顶层在 session_end_int 触发时需要把 "刚结束的 session" 的 cached 值
// 写入 save_step / save_avg_cad / save_avg_hr 这 3 个 input,
// flash_op_seq 用它们装配 prog_buf (见端口注释).
// =============================================================================
module flash_op_seq (
    input  wire        clk, rst_n,

    // ---- 触发 ----
    input  wire        session_end_int,    // session_stats 下降沿单脉冲
    input  wire        flash_view_en,      // 当前是否在 FLASH 视图 (拨码)
    input  wire        btn_confirm,        // CONFIRM (顶层已取反, 0=按下)
    input  wire        btn_mode,           // MODE    (顶层已取反, 0=按下)
    input  wire        work_en,            // 来自 sm_main, 用于清 session_happened

    // ---- 要保存的数据 (顶层在 session_end_int 的同一拍把刚结束的 session
    //                    的 cached 值打到这三个 input) ----
    input  wire [15:0] save_step,
    input  wire [7:0]  save_avg_cad,
    input  wire [7:0]  save_avg_hr,

    // ---- flash_top 命令式接口 ----
    output reg  [3:0]  op,
    output reg  [23:0] addr,
    output reg  [15:0] len,
    output reg         start,
    output reg         wdata_valid,
    output reg  [7:0]  wdata,
    input  wire        wdata_ready,
    input  wire [7:0]  rdata,
    input  wire        rdata_valid,
    input  wire        busy,
    input  wire        done,
    input  wire        error,

    // ---- 显示 ----
    output reg         flash_op_en,
    output reg  [1:0]  flash_op_message,

    // ---- 加载数据 ----
    output reg  [15:0] load_step,
    output reg  [7:0]  load_avg_cad,
    output reg  [7:0]  load_avg_hr,
    output reg         load_valid,

    // ---- 门控 (供 top_integration 用) ----
    output reg         delete_active,      // DEL_REQ 期间拉高
    output reg         load_in_progress,   // rst_n 释放后 0..15ms 拉高
    output reg         session_happened    // save 完成后置 1, work_en 上升沿清 0
);

    // ---- 状态编码 (5-bit, 有足够空间) ----
    localparam [4:0]
        S_IDLE         = 5'd0,
        S_LOAD_INIT    = 5'd1,
        S_INIT_RSTEN      = 5'd16,  // Flash Reset Enable (0x66)
        S_INIT_RSTEN_WAIT = 5'd22,  // 等 RSTEN done
        S_INIT_RST        = 5'd17,  // Flash Reset (0x99)
        S_INIT_RST_WAIT  = 5'd23,  // 等 RST done
        S_INIT_RST_DELAY = 5'd24,  // RST 后硬延时 200µs
        S_INIT_GBULK     = 5'd26,  // Global Block Unlock
        S_INIT_GBULK_WAIT= 5'd27,  // 等 GBULK done
        S_INIT_WSR       = 5'd18,  // 清状态寄存器保护位
        S_INIT_WSR_WAIT   = 5'd19,  // 等 WSR 完成
        S_SCAN_NEXT       = 5'd20,  // 扫描下一条记录
        S_SCAN_READ       = 5'd21,  // 读当前扫描地址
        S_LOAD_ISSUE   = 5'd2,
        S_LOAD_WAIT    = 5'd3,
        S_DEL_REQ      = 5'd4,
        S_ERASE_ISSUE  = 5'd5,
        S_ERASE_WAIT   = 5'd6,
        S_PROG_ISSUE   = 5'd7,
        S_PROG_SEND    = 5'd8,
        S_PROG_WAIT    = 5'd9,
        S_SHOW_MSG     = 5'd10;

    // ---- 操作类型 ----
    localparam [1:0]
        OP_NONE   = 2'd0,
        OP_SAVE   = 2'd1,
        OP_DELETE = 2'd2;

    // ---- flash op codes (与 flash_ctrl.v 一致) ----
    localparam [3:0]
        FLASH_OP_READ_JEDEC_ID = 4'h1,   // 临时调试: 读 JEDEC ID
        FLASH_OP_READ_STATUS  = 4'h2,    // 读 Status Reg-1 (0x05)
        FLASH_OP_READ_DATA   = 4'h3,
        FLASH_OP_PAGE_PROGRAM= 4'h4,
        FLASH_OP_SECTOR_ERASE= 4'h5,
        FLASH_OP_BLOCK_ERASE32=4'h6,     // Block Erase 32KB (0x52)
        FLASH_OP_BLOCK_ERASE64=4'h7,     // Block Erase 64KB (0xD8)
        FLASH_OP_CHIP_ERASE  = 4'h8,     // 全片擦除, 无地址, 更简单
        FLASH_OP_WRITE_STATUS= 4'hC,     // WE + 0x01 + data → 写状态寄存器
        FLASH_OP_RESET_ENABLE= 4'hD,     // Reset Enable (0x66)
        FLASH_OP_RESET_DEVICE= 4'hE,     // Reset Device (0x99)
        FLASH_OP_GBLOCK_UNLOCK=4'hF;     // Global Block Unlock (0x98)

    // ---- 提示消息编码 ----
    localparam [1:0]
        MSG_SAVING  = 2'd0,
        MSG_SAVED   = 2'd1,
        MSG_DEL_REQ = 2'd2,
        MSG_DELETED = 2'd3;

    // ---- 计时常量 ----
    localparam [19:0] LOAD_WAIT_CYCLES = 20'h16E36;     // 1.5M = 15ms @ 100MHz
    localparam [31:0] SHOW_CYCLES      = 32'd500_000_000;  // 5s @ 100MHz
    localparam [23:0] DEL_DEBOUNCE_CYCLES = 24'd8000000;  // 8M = 80ms @ 100MHz

    // ---- 内部状态 ----
    reg  [4:0]  state;
    reg  [1:0]  pending_op;
    reg         op_err;
    reg  [31:0] show_timer;
    reg  [19:0] load_timer;
    reg  [23:0] del_debounce_cnt;   // S_DEL_REQ 按键消抖 (80ms)
    reg  [2:0]  rdata_cnt;
    reg  [2:0]  prog_byte_cnt;
    reg  [7:0]  prog_buf0, prog_buf1, prog_buf2, prog_buf3, prog_buf4;
    reg         session_happened_d;   // save 完成的"软"信号, 在 SHOW_MSG 入口锁存

    // ---- 多记录管理 ----
    localparam [23:0] RECORD_STRIDE = 24'd16;
    localparam [5:0]  MAX_RECORDS   = 6'd40;
    reg  [23:0] write_addr;
    reg  [23:0] read_addr;
    reg  [23:0] scan_addr;
    reg  [5:0]  total_records;
    reg  [5:0]  current_record;
    reg         scan_done;
    reg         load_done;

    // ---- 边沿检测 ----
    reg         work_en_d1;
    reg         btn_confirm_d1, btn_mode_d1;
    reg         wdata_ready_d1;
    wire        work_en_rise   = work_en & ~work_en_d1;
    wire        wdata_ready_rise = wdata_ready && !wdata_ready_d1;
    // btn_confirm/btn_mode 在顶层已取反 (0=按下), 用 negedge 检测按下瞬间
    wire        confirm_posedge= ~btn_confirm & btn_confirm_d1;
    wire        mode_posedge   = ~btn_mode    & btn_mode_d1;

    // ---- 清除 session_happened 的触发 (导航/load 时) ----
    reg         clear_session;

    // ---- 边沿采样 + work_en / clear_session 清 session_happened ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            work_en_d1        <= 1'b0;
            btn_confirm_d1    <= 1'b1;
            btn_mode_d1       <= 1'b1;
            wdata_ready_d1    <= 1'b0;
            session_happened  <= 1'b0;
        end else begin
            work_en_d1        <= work_en;
            btn_confirm_d1    <= btn_confirm;
            btn_mode_d1       <= btn_mode;
            wdata_ready_d1    <= wdata_ready;
            if (work_en_rise || clear_session)
                session_happened <= 1'b0;
            else if (session_happened_d)
                session_happened <= 1'b1;
        end
    end

    // ---- 主 FSM: 单 always 块, 状态 + 输出 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            pending_op       <= OP_NONE;
            op_err           <= 1'b0;
            show_timer       <= 32'd0;
            load_timer       <= 20'd0;
            del_debounce_cnt <= 24'd0;
            rdata_cnt        <= 3'd0;
            prog_byte_cnt    <= 3'd0;
            start            <= 1'b0;
            wdata_valid      <= 1'b0;
            wdata            <= 8'h00;
            addr             <= 24'h000000;
            len              <= 16'd0;
            op               <= FLASH_OP_READ_DATA;
            flash_op_en      <= 1'b0;
            flash_op_message <= MSG_SAVING;
            delete_active    <= 1'b0;
            load_in_progress <= 1'b0;
            load_step        <= 16'd0;
            load_avg_cad     <= 8'd0;
            load_avg_hr      <= 8'd0;
            load_valid       <= 1'b0;
            prog_buf0        <= 8'h00;
            prog_buf1        <= 8'h00;
            prog_buf2        <= 8'h00;
            prog_buf3        <= 8'h00;
            prog_buf4        <= 8'h00;
            session_happened_d <= 1'b0;
            clear_session    <= 1'b0;
            load_done        <= 1'b0;
            write_addr       <= 24'd0;
            read_addr        <= 24'd0;
            scan_addr        <= 24'd0;
            total_records    <= 6'd0;
            current_record   <= 6'd0;
            scan_done        <= 1'b0;
        end else begin
            // 默认值 (每拍重置, 然后按状态覆盖)
            start            <= 1'b0;
            wdata_valid      <= 1'b0;
            clear_session    <= 1'b0;
            session_happened_d <= 1'b0;

            case (state)
                // ============================================================
                S_IDLE: begin
                    delete_active    <= 1'b0;
                    load_in_progress <= 1'b0;
                    flash_op_en      <= 1'b0;

                    // ★ 优先级: session_end(save) > FLASH导航 > FLASH删除 > load
                    if (session_end_int) begin
                        // ★ 保存: write_addr==0 时先擦除整块, 后续顺序写入
                        pending_op    <= OP_SAVE;
                        op_err        <= 1'b0;
                        prog_buf0     <= save_step[7:0];
                        prog_buf1     <= save_step[15:8];
                        prog_buf2     <= save_avg_cad;
                        prog_buf3     <= save_avg_hr;
                        prog_buf4     <= 8'hA5;
                        prog_byte_cnt <= 3'd0;
                        if (write_addr == 24'd0) begin
                            // 块起始 → 先 Block Erase 64KB, 再 PP
                            state <= S_ERASE_ISSUE;
                            op    <= FLASH_OP_BLOCK_ERASE64;
                            addr  <= 24'h000000;
                            len   <= 16'd0;
                            start <= 1'b1;
                        end else begin
                            // 地址 > 0 → 已在已擦除区域, 直接 PP
                            state <= S_PROG_ISSUE;
                        end
                        flash_op_en   <= 1'b1;
                        flash_op_message <= MSG_SAVING;
                    end else if (flash_view_en && scan_done && total_records > 0 && mode_posedge) begin
                        // ★ FLASH视图 MODE: 循环切换记录 (0→1→...→N-1→0)
                        if (current_record < total_records - 6'd1) begin
                            current_record <= current_record + 6'd1;
                            read_addr <= RECORD_STRIDE * (current_record + 6'd1);
                        end else begin
                            current_record <= 6'd0;            // 回绕到最早
                            read_addr <= 24'd0;
                        end
                        state     <= S_LOAD_ISSUE;
                        rdata_cnt <= 3'd0;
                        clear_session <= 1'b1;   // ★ 导航时清 session_happened, OLED 显示 load 数据
                    end else if (flash_view_en && confirm_posedge) begin
                        // ★ FLASH视图 CONFIRM: 删除所有记录 (DEL_REQ)
                        pending_op    <= OP_NONE;
                        op_err        <= 1'b0;
                        state         <= S_DEL_REQ;
                        flash_op_en   <= 1'b1;
                        flash_op_message <= MSG_DEL_REQ;
                        delete_active <= 1'b1;
                        del_debounce_cnt <= 24'd0;
                    end else if (!load_done) begin
                        // 上电自动 scan + load (仅一次)
                        load_in_progress <= 1'b1;
                        load_timer    <= 20'd0;
                        state         <= S_LOAD_INIT;
                    end else begin
                        state <= S_IDLE;
                    end
                end

                // ============================================================
                S_LOAD_INIT: begin
                    load_in_progress <= 1'b1;
                    flash_op_en      <= 1'b0;
                    delete_active    <= 1'b0;
                    if (load_timer == LOAD_WAIT_CYCLES - 1) begin
                        state  <= S_INIT_RSTEN;  // ★ 先软件复位 Flash
                        load_timer <= 20'd0;
                    end else begin
                        load_timer <= load_timer + 20'd1;
                    end
                end

                // ============================================================
                // ★ Flash 软件复位: Reset Enable (0x66), 发一脉冲即等 done
                // ============================================================
                S_INIT_RSTEN: begin
                    load_in_progress <= 1'b0;
                    op    <= FLASH_OP_RESET_ENABLE;
                    start <= 1'b1;
                    state <= S_INIT_RSTEN_WAIT;
                end

                // ============================================================
                // S_INIT_RSTEN_WAIT: 等 RSTEN 完成 → 进 RST
                // ============================================================
                S_INIT_RSTEN_WAIT: begin
                    if (done) state <= S_INIT_RST;
                end

                // ============================================================
                // ★ Flash 软件复位: Reset Device (0x99), 发一脉冲即等 done
                // ============================================================
                S_INIT_RST: begin
                    load_in_progress <= 1'b0;
                    op    <= FLASH_OP_RESET_DEVICE;
                    start <= 1'b1;
                    state <= S_INIT_RST_WAIT;
                end

                // ============================================================
                // S_INIT_RST_WAIT: 等 RST 完成 → 硬延时等 Flash 恢复
                // ============================================================
                S_INIT_RST_WAIT: begin
                    if (done) begin
                        state <= S_INIT_RST_DELAY;
                        load_timer <= 20'd0;
                    end
                end

                // ============================================================
                // ★ RST 后硬延时 200µs (tRST max=100µs, 2x margin)
                // ============================================================
                S_INIT_RST_DELAY: begin
                    if (load_timer == 20'd20000)  // 200µs @ 100MHz
                        state <= S_INIT_GBULK;
                    else
                        load_timer <= load_timer + 20'd1;
                end

                // ============================================================
                // ★ Global Block Unlock (0x98): 清除独立块锁
                // ============================================================
                S_INIT_GBULK: begin
                    load_in_progress <= 1'b0;
                    op    <= FLASH_OP_GBLOCK_UNLOCK;
                    start <= 1'b1;
                    state <= S_INIT_GBULK_WAIT;
                end

                S_INIT_GBULK_WAIT: begin
                    if (done) state <= S_INIT_WSR;
                end

                // ============================================================
                // ★ 上电初始化: Write Status Register = 0x00, 清除 BP/SRP 保护
                // ============================================================
                // ★ 清 SR1+SR2+SR3: 3字节全写0x00, 彻底解除保护
                S_INIT_WSR: begin
                    load_in_progress <= 1'b0;
                    op            <= FLASH_OP_WRITE_STATUS;
                    addr          <= 24'h000000;
                    len           <= 16'd1;         // SR1 1字节
                    start         <= 1'b1;
                    wdata         <= 8'h00;         // 清空所有保护位
                    wdata_valid   <= 1'b0;
                    state         <= S_INIT_WSR_WAIT;
                end

                S_INIT_WSR_WAIT: begin
                    load_in_progress <= 1'b0;
                    if (!wdata_valid && wdata_ready_rise)
                        wdata_valid <= 1'b1;
                    else if (done) begin
                        wdata_valid <= 1'b0;
                        // ★ 扫描所有记录, 找最新一条
                        scan_addr     <= 24'd0;
                        total_records <= 6'd0;
                        state         <= S_SCAN_NEXT;
                    end
                end

                // ============================================================
                // ★ S_SCAN_NEXT: 检查当前 scan_addr 是否还有有效记录
                //   读5字节, 检查第5字节 magic=0xA5 判断有效性
                // ============================================================
                S_SCAN_NEXT: begin
                    load_in_progress <= 1'b0;
                    if (scan_addr >= RECORD_STRIDE * MAX_RECORDS) begin
                        // 扫描完成: 加载最新记录
                        scan_done <= 1'b1;
                        clear_session <= 1'b1;   // ★ load 时清 session_happened
                        // ★ 关键: write_addr 必须跳到下一条空闲地址
                        //   否则 reset 后 write_addr=0, 再次 save 会覆盖旧记录!
                        write_addr <= RECORD_STRIDE * total_records;
                        if (total_records > 0) begin
                            current_record <= total_records - 6'd1;
                            read_addr   <= RECORD_STRIDE * (total_records - 6'd1);
                            state       <= S_LOAD_ISSUE;
                            rdata_cnt   <= 3'd0;
                        end else begin
                            load_valid    <= 1'b1;
                            load_step     <= 16'hDEAD;  // 标记: 无数据
                            load_avg_cad  <= 8'hFF;
                            load_avg_hr   <= 8'hFF;
                            state         <= S_IDLE;
                            load_done     <= 1'b1;
                        end
                    end else if (!busy) begin
                        // ★ 等待 flash_ctrl 空闲后再发下一读 (避免时序竞争)
                        op     <= FLASH_OP_READ_DATA;
                        addr   <= scan_addr;
                        len    <= 16'd5;
                        start  <= 1'b1;
                        rdata_cnt <= 3'd0;
                        state  <= S_SCAN_READ;
                    end
                end

                // ============================================================
                // ★ S_SCAN_READ: 读回5字节, 检查magic=0xA5来判断记录有效性
                // ============================================================
                S_SCAN_READ: begin
                    load_in_progress <= 1'b0;
                    if (rdata_valid) begin
                        rdata_cnt <= rdata_cnt + 3'd1;
                        if (rdata_cnt == 3'd4 && rdata == 8'hA5) begin
                            total_records <= total_records + 6'd1;
                        end
                    end
                    if (done) begin
                        scan_addr <= scan_addr + RECORD_STRIDE;
                        state     <= S_SCAN_NEXT;
                    end
                end

                // ============================================================
                S_LOAD_ISSUE: begin
                    // ★ 读取 read_addr 处的 5 字节数据
                    load_in_progress <= 1'b0;
                    op     <= FLASH_OP_READ_DATA;
                    addr   <= read_addr;
                    len    <= 16'd5;
                    start  <= 1'b1;
                    rdata_cnt <= 3'd0;
                    state  <= S_LOAD_WAIT;
                end

                // ============================================================
                S_LOAD_WAIT: begin
                    // 捕获保存的数据: [0]step_lo [1]step_hi [2]cadence [3]hr [4]magic
                    load_in_progress <= 1'b0;
                    if (rdata_valid) begin
                        case (rdata_cnt)
                            3'd0: load_step[7:0]   <= rdata;  // step_count[7:0]
                            3'd1: load_step[15:8]  <= rdata;  // step_count[15:8]
                            3'd2: load_avg_cad     <= rdata;  // avg_cadence
                            3'd3: load_avg_hr      <= rdata;  // avg_hr
                            3'd4: ;  // magic byte (0xA5) — 仅校验, 不显示
                        endcase
                        rdata_cnt <= rdata_cnt + 3'd1;
                    end
                    if (rdata_cnt == 3'd5) begin
                        load_valid    <= 1'b1;
                        state         <= S_IDLE;
                        flash_op_en   <= 1'b0;
                        load_done     <= 1'b1;
                    end
                end

                // ============================================================
                S_DEL_REQ: begin
                    delete_active    <= 1'b1;
                    flash_op_en      <= 1'b1;
                    flash_op_message <= MSG_DEL_REQ;
                    // 消抖计数: 进入 S_DEL_REQ 后 50ms 内忽略按键, 防止 bounce 误触发
                    if (del_debounce_cnt < DEL_DEBOUNCE_CYCLES) begin
                        del_debounce_cnt <= del_debounce_cnt + 24'd1;
                    end else if (mode_posedge) begin
                        // 取消 (消抖后)
                        state         <= S_IDLE;
                        delete_active <= 1'b0;
                        flash_op_en   <= 1'b0;
                        del_debounce_cnt <= 24'd0;
                    end else if (confirm_posedge) begin
                        // 确认删除 (消抖后)
                        pending_op    <= OP_DELETE;
                        op_err        <= 1'b0;
                        state         <= S_ERASE_ISSUE;
                        op            <= FLASH_OP_BLOCK_ERASE64;
                        addr          <= 24'h000000;
                        len           <= 16'd0;
                        start         <= 1'b1;
                        flash_op_en   <= 1'b1;
                        flash_op_message <= MSG_SAVING;
                    end
                end

                // ============================================================
                S_ERASE_ISSUE: begin
                    // start 已在上一拍脉冲过, 这里直接进入等待
                    start            <= 1'b0;
                    flash_op_en      <= 1'b1;
                    flash_op_message <= MSG_SAVING;
                    state            <= S_ERASE_WAIT;
                end

                // ============================================================
                S_ERASE_WAIT: begin
                    flash_op_en      <= 1'b1;
                    flash_op_message <= MSG_SAVING;
                    if (done) begin
                        if (error) begin
                            op_err        <= 1'b1;
                            show_timer    <= 32'd0;
                            state         <= S_SHOW_MSG;
                        end else if (pending_op == OP_SAVE) begin
                            // 擦除完成: 整块已清空, 写指针和记录归零
                            write_addr    <= 24'd0;
                            total_records <= 6'd0;
                            current_record<= 6'd0;
                            scan_done     <= 1'b0;   // 下次需要重新扫描
                            prog_buf0     <= save_step[7:0];
                            prog_buf1     <= save_step[15:8];
                            prog_buf2     <= save_avg_cad;
                            prog_buf3     <= save_avg_hr;
                            prog_buf4     <= 8'hA5;
                            prog_byte_cnt <= 3'd0;
                            state         <= S_PROG_ISSUE;
                        end else begin
                            // OP_DELETE: 擦除后重置所有记录状态
                            write_addr    <= 24'd0;
                            total_records <= 6'd0;
                            current_record<= 6'd0;
                            read_addr     <= 24'd0;
                            scan_done     <= 1'b0;
                            load_step     <= 16'd0;
                            load_avg_cad  <= 8'd0;
                            load_avg_hr   <= 8'd0;
                            load_valid    <= 1'b0;
                            show_timer    <= 32'd0;
                            state         <= S_SHOW_MSG;
                        end
                    end else if (error) begin
                        op_err     <= 1'b1;
                        show_timer <= 32'd0;
                        state      <= S_SHOW_MSG;
                    end
                end

                // ============================================================
                S_PROG_ISSUE: begin
                    op            <= FLASH_OP_PAGE_PROGRAM;
                    addr          <= write_addr;   // ★ 使用写指针
                    len           <= 16'd5;
                    start         <= 1'b1;
                    flash_op_en   <= 1'b1;
                    flash_op_message <= MSG_SAVING;
                    state         <= S_PROG_SEND;
                end

                // ============================================================
                S_PROG_SEND: begin
                    flash_op_en      <= 1'b1;
                    flash_op_message <= MSG_SAVING;
                    wdata_valid      <= 1'b1;   // ★ 预驱, flash_ctrl 新条件接手
                    case (prog_byte_cnt)
                        3'd0: wdata <= prog_buf0;
                        3'd1: wdata <= prog_buf1;
                        3'd2: wdata <= prog_buf2;
                        3'd3: wdata <= prog_buf3;
                        3'd4: wdata <= prog_buf4;
                    endcase
                    if (wdata_ready_rise && prog_byte_cnt < 3'd5) begin
                        // ★ rising-edge detect: 每字节仅计数一次
                        prog_byte_cnt <= prog_byte_cnt + 3'd1;
                    end
                    if (prog_byte_cnt == 3'd5 && !wdata_ready) begin
                        wdata_valid <= 1'b0;
                        state       <= S_PROG_WAIT;
                    end
                end

                // ============================================================
                S_PROG_WAIT: begin
                    flash_op_en      <= 1'b1;
                    flash_op_message <= MSG_SAVING;
                    if (done) begin
                        if (error) begin
                            op_err  <= 1'b1;
                        end
                        show_timer    <= 32'd0;
                        state         <= S_SHOW_MSG;
                    end else if (error) begin
                        op_err  <= 1'b1;
                        show_timer <= 32'd0;
                        state    <= S_SHOW_MSG;
                    end
                end

                // ============================================================
                S_SHOW_MSG: begin
                    flash_op_en <= 1'b1;
                    if (op_err) begin
                        flash_op_message <= MSG_DEL_REQ;  // 复用 "Delete?" 槽显示 Error
                    end else if (pending_op == OP_SAVE) begin
                        flash_op_message <= MSG_SAVED;
                        session_happened_d <= 1'b1;
                        // ★ 写指针前进, 更新记录计数
                        if (show_timer == 32'd0) begin
                            write_addr     <= write_addr + RECORD_STRIDE;
                            total_records  <= total_records + 6'd1;
                            current_record <= total_records;  // 指向最新记录
                            read_addr      <= write_addr;     // 保存刚写入的地址
                            scan_done      <= 1'b1;
                        end
                    end else begin
                        flash_op_message <= MSG_DELETED;
                    end
                    if (show_timer == SHOW_CYCLES - 1) begin
                        state         <= S_IDLE;
                        flash_op_en   <= 1'b0;
                        delete_active <= 1'b0;
                        pending_op    <= OP_NONE;
                        show_timer    <= 32'd0;
                    end else begin
                        show_timer <= show_timer + 32'd1;
                    end
                end

                // ============================================================
                default: state <= S_IDLE;
            endcase
        end
    end

    // ---- 未使用 wire 抑制警告 ----
    wire _unused_ok = &{1'b0, busy};

endmodule
