`timescale 1ns / 1ps

module max30102_top #
(
    parameter CLK_FREQ = 100_000_000
)
(
    input  wire       clk,
    input  wire       rst_n_raw,     // 按键输入（按下=高电平）

    input  wire       start,         // 开关：开始采样
    input  wire       sel,           // 开关：0=IR数据, 1=RED数据

    // I2C
    output wire       scl,
    inout  wire       sda,

    // 数据输出
    output wire [7:0] sensor_data,   // 8位数据，直接接LED

    // 状态输出
    output wire       data_valid,    // LED: 新数据脉冲
    output wire       init_done      // LED: 初始化完成
);

    wire rst_n = ~rst_n_raw;         // 转换为低电平复位

    wire [17:0] ir_data_18;
    wire [17:0] red_data_18;

    // 高8位 [17:10]
    wire [7:0] ir_8b  = ir_data_18[17:10];
    wire [7:0] red_8b = red_data_18[17:10];

    assign sensor_data = sel ? red_8b : ir_8b;

    max30102_driver driver (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .scl        (scl),
        .sda        (sda),
        .ir_data    (ir_data_18),
        .red_data   (red_data_18),
        .data_valid (data_valid),
        .init_done  (init_done),
        .sda_link   (),
        .sda_r      (),
        .state      ()
    );

endmodule