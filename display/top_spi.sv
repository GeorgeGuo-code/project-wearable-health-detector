`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// top_spi — 健康监测系统顶层 (cs13 + 渲染引擎版)
//////////////////////////////////////////////////////////////////////////////////

module top_spi (
    input  wire        clk,              // 100MHz
    input  wire        rst_n,

    input  wire        btn_mode,
    input  wire        btn_confirm,

    input  wire        data_valid,
    input  wire [7:0]  heart_rate,
    input  wire [7:0]  temperature,
    input  wire [2:0]  activity_level,
    input  wire [7:0]  cadence,

    // ---- OLED 接口 (cs13 命名) ----
    output wire        oled_csn,
    output wire        oled_rst,
    output wire        oled_dcn,
    output wire        oled_clk,
    output wire        oled_dat,

    output wire [2:0]  status_led,
    output wire        buzzer,

    // ---- Flash 视图 (本期新增, 顶层透传) ----
    input  wire        flash_view_en,        // 1 = OLED 切到 FLASH 视图
    input  wire [15:0] cached_step_count,    // session 缓存: 累计步数
    input  wire [7:0]  cached_avg_cadence,   // session 缓存: 平均步频
    input  wire [7:0]  cached_avg_hr,        // session 缓存: 平均心率
    input  wire [15:0] step_count            // 实时步数 (新2页模式用)
);

    wire        work_en;
    wire [2:0]  display_mode;
    wire [1:0]  health_status;
    wire        alarm;
    wire        standby_led;
    wire [7:0]  health_score;
    wire [7:0]  tach_count;
    wire [7:0]  brad_count;
    wire [7:0]  fever_count;
    wire [1:0]  worst_status;

    top_health_monitor u_health_monitor (
        .clk            (clk),
        .rst_n          (rst_n),
        .btn_mode       (btn_mode),
        .btn_confirm    (btn_confirm),
        .data_valid     (data_valid),
        .heart_rate     (heart_rate),
        .temperature    (temperature),
        .activity_level (activity_level),
        .work_en        (work_en),
        .display_mode   (display_mode),
        .health_status  (health_status),
        .alarm          (alarm),
        .standby_led    (standby_led),
        .health_score   (health_score),
        .tach_count     (tach_count),
        .brad_count     (brad_count),
        .fever_count    (fever_count),
        .worst_status   (worst_status)
    );

    display_top_spi u_display (
        .clk            (clk),
        .rst_n          (rst_n),
        .work_en        (work_en),
        .display_mode   (display_mode),
        .health_status  (health_status),
        .alarm          (alarm),
        .heart_rate     (heart_rate),
        .temperature    (temperature),
        .activity_level (activity_level),
        .cadence        (cadence),
        .health_score   (health_score),
        .tach_count     (tach_count),
        .brad_count     (brad_count),
        .fever_count    (fever_count),
        .worst_status   (worst_status),
        .oled_csn       (oled_csn),
        .oled_rst       (oled_rst),
        .oled_dcn       (oled_dcn),
        .oled_clk       (oled_clk),
        .oled_dat       (oled_dat),
        .status_led     (status_led),
        .buzzer         (buzzer),
        // ★ Flash 视图 (新增, 透传)
        .flash_view_en  (flash_view_en),
        .flash_step     (cached_step_count),
        .flash_avg_cad  (cached_avg_cadence),
        .flash_avg_hr   (cached_avg_hr),
        .step_count     (step_count)
    );

endmodule
