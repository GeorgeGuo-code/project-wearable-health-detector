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
    output wire        buzzer
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
        .buzzer         (buzzer)
    );

endmodule
