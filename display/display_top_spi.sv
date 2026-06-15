`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// display_top_spi — 显示模块顶层 (cs13 内部渲染版)
//   所有显示逻辑在 cs13 内部, 此处仅透传数据 + LED/蜂鸣器驱动
//////////////////////////////////////////////////////////////////////////////////
`include "test_defines.v"

module display_top_spi (
    input  wire        clk, rst_n,
    input  wire        work_en,
    input  wire [2:0]  display_mode,
    input  wire [1:0]  health_status,
    input  wire        alarm,

    input  wire [7:0]  heart_rate, temperature,
    input  wire [2:0]  activity_level,
    input  wire [7:0]  cadence,
    input  wire [7:0]  health_score, tach_count, brad_count, fever_count,
    input  wire [1:0]  worst_status,

    output wire        oled_csn, oled_rst, oled_dcn, oled_clk, oled_dat,
    output wire [2:0]  status_led,
    output wire        buzzer,

    // ---- Flash 视图 (新增, 透传给 cs13) ----
    input  wire        flash_view_en,
    input  wire [15:0] flash_step,
    input  wire [7:0]  flash_avg_cad,
    input  wire [7:0]  flash_avg_hr,
    input  wire [15:0] step_count         // 实时步数 (新2页模式用)
);

    // ---- cs13 OLED driver (all rendering internal) ----
    //   测试模式下: 强制 DANGER 让 OLED 也显示危险标记
    `ifdef TEST_FORCE_DANGER
        wire [1:0] health_status_eff = 2'b10;   // DANGER
        wire       alarm_eff         = 1'b1;
    `else
        wire [1:0] health_status_eff = health_status;
        wire       alarm_eff         = alarm;
    `endif

    cs13 u_cs13 (
        .clk(clk), .rst(rst_n), .en(1'b1),
        .heart_rate(heart_rate), .temperature(temperature),
        .activity_level(activity_level), .cadence(cadence),
        .health_status(health_status_eff), .health_score(health_score),
        .tach_count(tach_count), .brad_count(brad_count),
        .fever_count(fever_count), .worst_status(worst_status),
        .work_en(work_en), .display_mode(display_mode),
        .oled_csn(oled_csn), .oled_rst(oled_rst), .oled_dcn(oled_dcn),
        .oled_clk(oled_clk), .oled_dat(oled_dat),
        // ★ Flash 视图 (新增)
        .flash_view_en(flash_view_en),
        .flash_step   (flash_step),
        .flash_avg_cad(flash_avg_cad),
        .flash_avg_hr (flash_avg_hr),
        .step_count   (step_count)
    );

    // ---- LED ----
    reg [2:0] led; reg [16:0] blk;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin led<=3'b001; blk<=17'd0; end
        else begin
            blk<=blk+17'd1;
            if (!work_en) led<=blk[16]?3'b000:3'b010;
            else case(health_status_eff) 2'b00:led<=3'b001; 2'b01:led<=3'b010; 2'b10:led<=3'b100; default:led<=3'b001; endcase
        end
    end
    assign status_led=led;

    // ---- Buzzer ----
    //   EGO1 蜂鸣器在 3.3V 直驱下可能过压不振 (<3.3V 才能正常发声)
    //   改 bank 电压会影响同 bank 全部引脚, 不可行
    //   解决: 2kHz PWM 低占空比 → 平均电压 = 3.3V × duty
    //   - 2kHz 落在听觉范围内, 蜂鸣器发声
    //   - 占空比越低, 平均电压越低, 声音越轻
    //   - 调 BUZZER_DUTY 即可改变音量
    localparam [15:0] BUZZER_PERIOD = 16'd50000;  // 100MHz / 50000 = 2kHz
    localparam [15:0] BUZZER_DUTY   = 16'd20000;  // 40% 占空比 → 平均 ≈ 1.3V
    reg [15:0] bcnt; reg bpwm;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin bcnt<=16'd0; bpwm<=1'b0; end
        else if (alarm_eff && work_en) begin
            bpwm <= (bcnt < BUZZER_DUTY);
            bcnt <= (bcnt == BUZZER_PERIOD - 1) ? 16'd0 : bcnt + 16'd1;
        end else begin
            bpwm <= 1'b0;
            bcnt <= 16'd0;
        end
    end
    assign buzzer = bpwm;

endmodule
