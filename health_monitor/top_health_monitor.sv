`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// top_health_monitor — 健康监测顶层封装 (精简版, 无存储功能)
//   连接 SM_Main、Health_Status_Evaluator
//   预留与队友模块 (SM_DataFlow, Display) 的接口
//////////////////////////////////////////////////////////////////////////////////

module top_health_monitor (
    input  wire        clk,              // 100MHz
    input  wire        rst_n,            // 低有效复位

    // ---- 用户输入 ----
    input  wire        btn_mode,
    input  wire        btn_confirm,

    // ---- 传感器数据 (来自队友 SM_DataFlow) ----
    input  wire        data_valid,       // 数据有效脉冲
    input  wire [7:0]  heart_rate,
    input  wire [7:0]  temperature,      // °C ×2
    input  wire [2:0]  activity_level,

    // ---- 输出到 Display (队友) ----
    output wire        work_en,
    output wire [2:0]  display_mode,
    output wire [1:0]  health_status,
    output wire        alarm,
    output wire        standby_led,

    // ---- 评分输出 (来自 health_scorer) ----
    output wire [7:0]  health_score,
    output wire [7:0]  tach_count,
    output wire [7:0]  brad_count,
    output wire [7:0]  fever_count,
    output wire [1:0]  worst_status
);

    //===================================================================
    // SM_Main — 主控状态机
    //===================================================================
    sm_main u_sm_main (
        .clk          (clk),
        .rst_n        (rst_n),
        .btn_mode     (btn_mode),
        .btn_confirm  (btn_confirm),
        .work_en      (work_en),
        .display_mode (display_mode),
        .standby_led  (standby_led)
    );

    //===================================================================
    // Health_Status_Evaluator — 健康评估
    //===================================================================
    health_status_evaluator u_evaluator (
        .clk            (clk),
        .rst_n          (rst_n),
        .data_valid     (data_valid),
        .heart_rate     (heart_rate),
        .temperature    (temperature),
        .activity_level (activity_level),
        .health_status  (health_status),
        .alarm          (alarm)
    );

    //===================================================================
    // Health_Scorer — 实时健康评分
    //===================================================================
    health_scorer u_scorer (
        .clk            (clk),
        .rst_n          (rst_n),
        .work_en        (work_en),
        .data_valid     (data_valid),
        .heart_rate     (heart_rate),
        .temperature    (temperature),
        .health_status  (health_status),
        .health_score   (health_score),
        .tach_count     (tach_count),
        .brad_count     (brad_count),
        .fever_count    (fever_count),
        .worst_status   (worst_status)
    );

endmodule
