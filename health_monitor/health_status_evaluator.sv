`timescale 1ns / 1ps
//   根据心率、体温、运动等级综合评估健康状态
//   详细设计方案 §3.2.4 阶段3
module health_status_evaluator (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        data_valid,        // 输入数据有效标志
    input  wire [7:0]  heart_rate,        // 心率 (bpm, 30-220)
    input  wire [7:0]  temperature,       // 体温 (°C ×2, 如 37.5→75)
    input  wire [2:0]  activity_level,    // 运动等级 (0-4)
    output reg  [1:0]  health_status,     // 健康状态: 00=正常 01=预警 10=危险
    output reg         alarm              // 报警标志 (危险时置1)
);

    localparam NORMAL  = 2'b00;
    localparam WARNING = 2'b01;
    localparam DANGER  = 2'b10;
    // 心率阈值 (bpm)

    localparam HR_TACHYCARDIA_DANGER  = 8'd120;   // 心动过速→危险
    localparam HR_TACHYCARDIA_WARNING = 8'd100;   // 心动过速→预警
    localparam HR_BRADYCARDIA_WARNING = 8'd60;    // 心动过缓→预警
    localparam HR_BRADYCARDIA_DANGER  = 8'd50;    // 心动过缓→危险
    // 体温阈值 (°C ×2)
    //   38.0°C→76, 37.5°C→75, 36.0°C→72
    localparam TEMP_FEVER_DANGER      = 8'd76;    // >38.0°C → 发热危险
    localparam TEMP_FEVER_WARNING     = 8'd75;    // >37.5°C → 低烧预警
    localparam TEMP_HYPOTHERMIA_WARN  = 8'd72;    // <36.0°C → 体温过低预警
    reg [1:0] hr_status;
    reg [1:0] temp_status;
    // 心率评估 
    always @(*) begin
        if (heart_rate > HR_TACHYCARDIA_DANGER)
            hr_status = DANGER;
        else if (heart_rate > HR_TACHYCARDIA_WARNING)
            hr_status = WARNING;
        else if (heart_rate < HR_BRADYCARDIA_DANGER)
            hr_status = DANGER;
        else if (heart_rate < HR_BRADYCARDIA_WARNING)
            hr_status = WARNING;
        else
            hr_status = NORMAL;
    end

    // 体温评估 
=
    always @(*) begin
        if (temperature > TEMP_FEVER_DANGER)
            temp_status = DANGER;
        else if (temperature > TEMP_FEVER_WARNING)
            temp_status = WARNING;
        else if (temperature < TEMP_HYPOTHERMIA_WARN)
            temp_status = WARNING;
        else
            temp_status = NORMAL;
    end
    // 综合评估: 取最严重的状态
    //   同时输出 alarm 标志
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            health_status <= NORMAL;
            alarm         <= 1'b0;
        end else if (data_valid) begin
            // 综合: 取心率、体温中最严重者
            if (hr_status == DANGER || temp_status == DANGER) begin
                health_status <= DANGER;
                alarm         <= 1'b1;
            end else if (hr_status == WARNING || temp_status == WARNING) begin
                health_status <= WARNING;
                alarm         <= 1'b0;
            end else begin
                health_status <= NORMAL;
                alarm         <= 1'b0;
            end
        end
    end

endmodule
