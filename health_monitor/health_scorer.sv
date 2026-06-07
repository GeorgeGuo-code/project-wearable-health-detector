`timescale 1ns / 1ps
// health_scorer — 实时健康评分模块
//   每次 data_valid 时累计异常事件, 实时计算 0-100 综合评分
//   会话结束时锁存最终评分 (work_en 下降沿)

module health_scorer (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        work_en,           
    input  wire        data_valid,        

    // 传感器数据 
    input  wire [7:0]  heart_rate,        // 心率 (bpm)
    input  wire [7:0]  temperature,       // 体温 (°C ×2)

    // 当前健康状态 (来自 evaluator) 
    input  wire [1:0]  health_status,     // 00=正常 01=预警 10=危险

    // 评分输出 
    output reg  [7:0]  health_score,      // 当前实时评分 (0-100)
    output reg  [7:0]  tach_count,        // 心动过速次数 (>120 bpm)
    output reg  [7:0]  brad_count,        // 心动过缓次数 (<60 bpm)
    output reg  [7:0]  fever_count,       // 发热次数 (>38.0°C)
    output reg  [1:0]  worst_status       // 本次会话最差健康状态
);

    // 阈值

    localparam HR_TACHYCARDIA  = 8'd120;   // 心动过速
    localparam HR_BRADYCARDIA  = 8'd60;    // 心动过缓
    localparam TEMP_FEVER      = 8'd76;    // >38.0°C (°C×2)
    reg [7:0]  tach_cnt;
    reg [7:0]  brad_cnt;
    reg [7:0]  fev_cnt;
    reg [1:0]  worst;

    // 心率统计 (用于奖励判定)
    reg [31:0] hr_sum;
    reg [23:0] hr_cnt;

    // 实时计算心率均值 (除零保护)
    wire [7:0] hr_avg;
    assign hr_avg = (hr_cnt > 0) ? (hr_sum / {8'd0, hr_cnt}) : 8'd0;
    reg        work_en_d1;
    wire       session_start;
    wire       session_end;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            work_en_d1 <= 1'b0;
        else
            work_en_d1 <= work_en;
    end

    assign session_start = work_en && !work_en_d1;
    assign session_end   = !work_en && work_en_d1;

    // 事件累计 + 最差状态追踪
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tach_cnt    <= 8'd0;
            brad_cnt    <= 8'd0;
            fev_cnt     <= 8'd0;
            worst       <= 2'd0;
            hr_sum      <= 32'd0;
            hr_cnt      <= 24'd0;
        end else if (session_start) begin
            tach_cnt    <= 8'd0;
            brad_cnt    <= 8'd0;
            fev_cnt     <= 8'd0;
            worst       <= 2'd0;
            hr_sum      <= 32'd0;
            hr_cnt      <= 24'd0;
        end else if (work_en && data_valid) begin
            // 心动过速 (>120 bpm)
            if (heart_rate > HR_TACHYCARDIA)
                tach_cnt <= tach_cnt + 8'd1;

            // 心动过缓 (<60 bpm)
            if (heart_rate < HR_BRADYCARDIA)
                brad_cnt <= brad_cnt + 8'd1;

            // 发热 (>38.0°C)
            if (temperature > TEMP_FEVER)
                fev_cnt <= fev_cnt + 8'd1;

            // 最差状态
            if (health_status > worst)
                worst <= health_status;

            // 心率累加 (用于计算均值)
            hr_sum <= hr_sum + {24'd0, heart_rate};
            hr_cnt <= hr_cnt + 24'd1;
        end
    end
    // 实时评分计算 
    wire [7:0] current_score;
    assign current_score = score_calc(tach_cnt, brad_cnt, fev_cnt, worst, hr_avg);

    // 输出锁存: 评分在会话结束后保持最终值
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            health_score <= 8'd0;
            tach_count   <= 8'd0;
            brad_count   <= 8'd0;
            fever_count  <= 8'd0;
            worst_status <= 2'd0;
        end else if (session_end) begin
            // 会话结束: 锁存最终评分
            health_score <= current_score;
            tach_count   <= tach_cnt;
            brad_count   <= brad_cnt;
            fever_count  <= fev_cnt;
            worst_status <= worst;
        end else begin
            // 工作中: 实时更新评分
            health_score <= current_score;
            tach_count   <= tach_cnt;
            brad_count   <= brad_cnt;
            fever_count  <= fev_cnt;
            worst_status <= worst;
        end
    end

    function [7:0] score_calc;
        input [7:0] tachy_cnt, brady_cnt, fevr_cnt;
        input [1:0] wst;
        input [7:0] hr_avg_val;
        reg [9:0] sc;
        begin
            sc = 10'd100;

            // 心动过速扣分: ≤10次 每次-5, >10次 直接-50
            if (tachy_cnt > 10)
                sc = (sc > 8'd50) ? (sc - 8'd50) : 0;
            else
                sc = (sc > tachy_cnt * 5) ? (sc - tachy_cnt * 5) : 0;

            // 心动过缓扣分: ≤20次 每次-3, >20次 直接-30
            if (brady_cnt > 20)
                sc = (sc > 8'd30) ? (sc - 8'd30) : 0;
            else
                sc = (sc > brady_cnt * 3) ? (sc - brady_cnt * 3) : 0;

            // 发热扣分: 每次 -10
            sc = (sc > fevr_cnt * 10) ? (sc - fevr_cnt * 10) : 0;

            // 最差状态扣分
            case (wst)
                2'd1: sc = (sc > 8'd10) ? (sc - 8'd10) : 0;   // WARNING
                2'd2: sc = (sc > 8'd30) ? (sc - 8'd30) : 0;   // DANGER
                default: sc = sc;
            endcase

            // 心率正常范围奖励 +5
            if (hr_avg_val >= 60 && hr_avg_val <= 100)
                sc = (sc + 8'd5 < 10'd100) ? (sc + 8'd5) : 10'd100;

            score_calc = sc[7:0];
        end
    endfunction

endmodule
