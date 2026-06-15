// =============================================================================
// Module:  session_stats
// Project: EGO1 Health Monitoring Wearable
// Purpose: 在 work_en 拉高期间累加 heart_rate / cadence 的 sum 与 count，
//          work_en 下降沿触发 session_end 一拍脉冲，同时把三个汇总值
//          锁存到 cached_* 寄存器，供 flash 驱动写库 / OLED FLASH 视图显示。
//
// 锁存行为：
//   - work_en=0 期间: cached_* 保持上次锁存值不变
//   - work_en 上升沿: 累加器清零，重新开始本会话
//   - work_en=1 + data_valid: 累加 hr_sum / cad_sum / hr_cnt / cad_cnt
//   - work_en 下降沿 (session_end): 计算 mean = sum/cnt，锁存到 cached_*；
//                                  清零累加器
//
// 设计依据: health_monitor/integration_plan.md §4.6
// =============================================================================
`timescale 1ns / 1ps

module session_stats (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        work_en,           // from sm_main
    input  wire        data_valid,        // 1Hz 温度脉冲 (or anything that gates sampling)

    // ---- 实时输入 ----
    input  wire [7:0]  heart_rate,        // bpm
    input  wire [7:0]  cadence,           // spm
    input  wire [15:0] step_count,        // 累计步数, 16bit

    // ---- 锁存输出 (供 flash 写库 / OLED FLASH 视图) ----
    output reg  [15:0] cached_step_count,
    output reg  [7:0]  cached_avg_cadence,
    output reg  [7:0]  cached_avg_hr,

    // ---- 握手: 1 拍会话结束脉冲 ----
    output wire        session_end
);

    // ------------------------------------------------------------------
    // 累加器
    //   - hr_sum: 最大 200 bpm × 86400 sec × 1Hz = 17.28M, 24 bit 略紧,
    //             用 32 bit 保险 (24h 持续运动也 OK)
    //   - hr_cnt: 1 Hz 持续 24h = 86400, 17 bit 足够, 用 24 bit 留余
    //   - cad: 同样 32/24 bit
    // ------------------------------------------------------------------
    reg [31:0] hr_sum, cad_sum;
    reg [23:0] hr_cnt, cad_cnt;

    // ------------------------------------------------------------------
    // session_end 边沿检测
    // ------------------------------------------------------------------
    reg work_en_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) work_en_d1 <= 1'b0;
        else        work_en_d1 <= work_en;
    end
    assign session_end = !work_en && work_en_d1;

    // ------------------------------------------------------------------
    // 累加 + 锁存
    //   data_valid 在 work_en 拉高期间才有效 (与 sm_main 联动)
    // ------------------------------------------------------------------
    wire [7:0] hr_avg_w  = (hr_cnt  != 24'd0) ? (hr_sum  / {8'd0, hr_cnt }) : 8'd0;
    wire [7:0] cad_avg_w = (cad_cnt != 24'd0) ? (cad_sum / {8'd0, cad_cnt}) : 8'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hr_sum  <= 32'd0;
            hr_cnt  <= 24'd0;
            cad_sum <= 32'd0;
            cad_cnt <= 24'd0;

            cached_step_count  <= 16'd0;
            cached_avg_cadence <= 8'd0;
            cached_avg_hr      <= 8'd0;
        end else if (!work_en_d1 && work_en) begin
            // work_en 上升沿: 本会话开始, 累加器清零
            // cached_* 保持上次会话值, 不变 (让 OLED FLASH 视图能继续显示)
            hr_sum  <= 32'd0;
            hr_cnt  <= 24'd0;
            cad_sum <= 32'd0;
            cad_cnt <= 24'd0;
        end else if (work_en && data_valid) begin
            hr_sum  <= hr_sum  + {24'd0, heart_rate};
            hr_cnt  <= hr_cnt  + 24'd1;
            cad_sum <= cad_sum + {24'd0, cadence};
            cad_cnt <= cad_cnt + 24'd1;
        end else if (session_end) begin
            // 锁存 + 复位累加器
            cached_step_count  <= step_count;
            cached_avg_hr      <= hr_avg_w;
            cached_avg_cadence <= cad_avg_w;

            hr_sum  <= 32'd0;
            hr_cnt  <= 24'd0;
            cad_sum <= 32'd0;
            cad_cnt <= 24'd0;
        end
    end

endmodule
