`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// SM_Main — 主控状态机
//   管理待机/工作模式切换、按键消抖、超时检测
//   详细设计方案 §3.1
//////////////////////////////////////////////////////////////////////////////////

module sm_main (
    input  wire        clk,              // 100MHz 系统时钟
    input  wire        rst_n,            // 异步复位，低有效
    input  wire        btn_mode,         // 模式切换按键
    input  wire        btn_confirm,      // 确认/退出按键
    output reg         work_en,          // 工作使能 (→ SM_DataFlow)
    output reg  [2:0]  display_mode,     // 显示模式 (→ Display)
    output reg         standby_led       // 待机指示灯
);

    //===================================================================
    // 状态定义
    //===================================================================
    localparam STANDBY = 1'b0;
    localparam WORK    = 1'b1;

    //===================================================================
    // 参数 (100MHz 时钟基准)
    //===================================================================
    localparam DEBOUNCE_MS   = 20;               // 消抖时间 20ms
    localparam DEBOUNCE_CNT  = DEBOUNCE_MS * 100_000;  // 2,000,000

    localparam TIMEOUT_SEC   = 30;               // 超时时间 30s
    localparam TIMEOUT_CNT   = TIMEOUT_SEC * 100_000_000; // 3,000,000,000

    localparam STANDBY_MODES = 3;                // 待机显示模式数
    localparam WORK_MODES    = 6;                // 心率/体温/步频/运动/综合/评分

    //===================================================================
    // 内部信号
    //===================================================================
    reg         state, next_state;

    // ---- 按键消抖 ----
    reg         btn_mode_s1,   btn_mode_s2;
    reg         btn_confirm_s1, btn_confirm_s2;
    reg [20:0]  debounce_cnt;
    reg         debouncing;
    reg         btn_mode_stable,    btn_confirm_stable;
    reg         btn_mode_stable_d1, btn_confirm_stable_d1;
    wire        btn_mode_posedge,   btn_confirm_posedge;

    // ---- 超时检测 ----
    reg [31:0]  timeout_cnt;
    wire        standby_timeout;

    //===================================================================
    // 两级同步器 (消除亚稳态)
    //===================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_mode_s1    <= 1'b0;
            btn_mode_s2    <= 1'b0;
            btn_confirm_s1 <= 1'b0;
            btn_confirm_s2 <= 1'b0;
        end else begin
            btn_mode_s1    <= btn_mode;
            btn_mode_s2    <= btn_mode_s1;
            btn_confirm_s1 <= btn_confirm;
            btn_confirm_s2 <= btn_confirm_s1;
        end
    end

    //===================================================================
    // 按键消抖 (20ms)
    //===================================================================
    reg         btn_mode_s2_d1, btn_confirm_s2_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_mode_s2_d1    <= 1'b0;
            btn_confirm_s2_d1 <= 1'b0;
        end else begin
            btn_mode_s2_d1    <= btn_mode_s2;
            btn_confirm_s2_d1 <= btn_confirm_s2;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debounce_cnt        <= 21'd0;
            debouncing          <= 1'b0;
            btn_mode_stable     <= 1'b0;
            btn_confirm_stable  <= 1'b0;
        end else begin
            // 检测信号自身跳变 → 启动消抖
            if (btn_mode_s2 != btn_mode_s2_d1 || btn_confirm_s2 != btn_confirm_s2_d1) begin
                debouncing   <= 1'b1;
                debounce_cnt <= 21'd0;
            end else if (debouncing) begin
                if (debounce_cnt == DEBOUNCE_CNT - 1) begin
                    debouncing       <= 1'b0;
                    btn_mode_stable  <= btn_mode_s2;
                    btn_confirm_stable <= btn_confirm_s2;
                end else begin
                    debounce_cnt <= debounce_cnt + 21'd1;
                end
            end
        end
    end

    // 边沿检测 (按下瞬间产生单周期脉冲)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_mode_stable_d1    <= 1'b0;
            btn_confirm_stable_d1 <= 1'b0;
        end else begin
            btn_mode_stable_d1    <= btn_mode_stable;
            btn_confirm_stable_d1 <= btn_confirm_stable;
        end
    end

    assign btn_mode_posedge    = btn_mode_stable && !btn_mode_stable_d1;
    assign btn_confirm_posedge = btn_confirm_stable && !btn_confirm_stable_d1;

    //===================================================================
    // 超时检测 (30秒无操作 → 自动待机)
    //===================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timeout_cnt <= 32'd0;
        end else if (state != WORK || btn_mode_posedge || btn_confirm_posedge) begin
            timeout_cnt <= 32'd0;                      // 有操作就清零
        end else if (timeout_cnt < TIMEOUT_CNT) begin
            timeout_cnt <= timeout_cnt + 32'd1;
        end
    end

    assign standby_timeout = (timeout_cnt == TIMEOUT_CNT);

    //===================================================================
    // 状态寄存器
    //===================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= STANDBY;
        else
            state <= next_state;
    end

    //===================================================================
    // 次态逻辑 & 输出逻辑 (米利型)
    //===================================================================
    always @(*) begin
        // 默认值
        next_state   = state;
        work_en      = 1'b0;
        standby_led  = 1'b0;

        case (state)
            STANDBY: begin
                work_en     = 1'b0;
                standby_led = 1'b1;

                if (btn_confirm_posedge) begin
                    next_state  = WORK;
                    work_en     = 1'b1;
                    standby_led = 1'b0;
                end
            end

            WORK: begin
                work_en     = 1'b1;
                standby_led = 1'b0;

                if (standby_timeout || btn_confirm_posedge) begin
                    next_state  = STANDBY;
                    work_en     = 1'b0;
                    standby_led = 1'b1;
                end
            end

            default: begin
                next_state = STANDBY;
            end
        endcase
    end

    //===================================================================
    // 显示模式切换 (独立于状态机，响应按键)
    //===================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            display_mode <= 3'd0;
        end else if (btn_mode_posedge) begin
            if (state == STANDBY)
                display_mode <= (display_mode + 3'd1) % STANDBY_MODES;
            else
                display_mode <= (display_mode + 3'd1) % WORK_MODES;
        end
    end

endmodule
