`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// top_oled_test_spi �? OLED 显示验证顶层 (SPI �?, 内置模拟传感器数�?)
//   不用接队友模�?, FPGA 自己生成测试数据循环变化
//   用法: Vivado 中把 top_oled_test_spi 设为顶层模块即可
//   �? top_oled_test.v 功能完全�?�?, 仅将 I2C 接口替换�? SPI 4-wire 接口
//////////////////////////////////////////////////////////////////////////////////

module top_oled_test_spi (
    input  wire        clk,              // 100MHz
    input  wire        rst_n_raw,          // 低有效复�?

    input  wire        btn_mode_raw,      // 按钮原始信号 (按下=高)
    input  wire        btn_confirm_raw,   // 按钮原始信号 (按下=高)

    // ---- SPI OLED 接口 (cs13 命名) ----
    output wire        oled_csn,          // SPI CS  (低有效)
    output wire        oled_rst,          // RESET   (低有效)
    output wire        oled_dcn,          // SPI DC  (0=命令, 1=数据)
    output wire        oled_clk,          // SPI SCLK
    output wire        oled_dat,          // SPI SDIN (MOSI)

    output wire [2:0]  status_led,
    output wire        buzzer
);

    wire rst_n       = ~rst_n_raw;
    wire btn_mode    = ~btn_mode_raw;
    wire btn_confirm = ~btn_confirm_raw;

    //===================================================================
    // 测试数据生成: 2 秒切换一�?, 循环 8 种场�?
    //===================================================================
    localparam CHANGE_CNT = 200_000_000;   // 2 �? @ 100MHz

    reg [31:0] test_cnt;
    reg [2:0]  test_phase;      // 0~7, 8 种场景循�?
    reg        data_valid;
    reg [7:0]  hr_test;
    reg [7:0]  temp_test;
    reg [2:0]  act_test;
    reg [7:0]  cad_test;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            test_cnt   <= 32'd0;
            test_phase <= 3'd0;
            data_valid <= 1'b0;
            hr_test    <= 8'd72;
            temp_test  <= 8'd74;
            act_test   <= 3'd0;
            cad_test   <= 8'd0;
        end else begin
            data_valid <= 1'b0;  // 默认�?, 仅切换时发一个脉�?

            if (test_cnt == CHANGE_CNT - 1) begin
                test_cnt   <= 32'd0;
                test_phase <= test_phase + 3'd1;
                data_valid <= 1'b1;

                // 8 种测试场�?: 正常→预警→危险→恢复→心动过缓→高热→剧烈运动→综�?
                case (test_phase)
                    // 场景0: 完全正常 �? 静坐
                    3'd0: begin
                        hr_test   <= 8'd72;     // 72 bpm
                        temp_test <= 8'd73;     // 36.5°C
                        act_test  <= 3'd0;      // 静坐
                        cad_test  <= 8'd0;      // 0 SPM
                    end
                    // 场景1: 轻度活动, 心率略高
                    3'd1: begin
                        hr_test   <= 8'd95;     // 95 bpm
                        temp_test <= 8'd74;     // 37.0°C
                        act_test  <= 3'd1;      // 轻度
                        cad_test  <= 8'd60;     // 60 SPM
                    end
                    // 场景2: 心率预警 + 低烧
                    3'd2: begin
                        hr_test   <= 8'd108;    // 108 bpm �? WARNING
                        temp_test <= 8'd75;     // 37.5°C �? WARNING
                        act_test  <= 3'd2;      // 中度
                        cad_test  <= 8'd100;    // 100 SPM
                    end
                    // 场景3: 危险! 心动过�?? + 高热
                    3'd3: begin
                        hr_test   <= 8'd128;    // 128 bpm �? DANGER
                        temp_test <= 8'd77;     // 38.5°C �? DANGER
                        act_test  <= 3'd3;      // 剧烈
                        cad_test  <= 8'd160;    // 160 SPM
                    end
                    // 场景4: 恢复�?, 心率仍略�?
                    3'd4: begin
                        hr_test   <= 8'd98;     // 98 bpm
                        temp_test <= 8'd74;     // 37.0°C
                        act_test  <= 3'd2;      // 中度
                        cad_test  <= 8'd80;     // 80 SPM
                    end
                    // 场景5: 心动过缓
                    3'd5: begin
                        hr_test   <= 8'd52;     // 52 bpm �? WARNING
                        temp_test <= 8'd73;     // 36.5°C
                        act_test  <= 3'd0;      // 静坐
                        cad_test  <= 8'd0;      // 0 SPM
                    end
                    // 场景6: 高热 + 剧烈运动
                    3'd6: begin
                        hr_test   <= 8'd135;    // 135 bpm �? DANGER
                        temp_test <= 8'd78;     // 39.0°C �? DANGER
                        act_test  <= 3'd4;      // 极剧�?
                        cad_test  <= 8'd180;    // 180 SPM
                    end
                    // 场景7: 恢复正常
                    3'd7: begin
                        hr_test   <= 8'd75;     // 75 bpm
                        temp_test <= 8'd73;     // 36.5°C
                        act_test  <= 3'd1;      // 轻度
                        cad_test  <= 8'd40;     // 40 SPM
                    end
                    default: begin
                        hr_test   <= 8'd72;
                        temp_test <= 8'd74;
                        act_test  <= 3'd0;
                        cad_test  <= 8'd0;
                    end
                endcase
            end else begin
                test_cnt <= test_cnt + 32'd1;
            end
        end
    end

    //===================================================================
    // 连接 SPI �? top 模块, 用测试数据替代传感器输入
    //===================================================================
    top_spi u_top (
        .clk                (clk),
        .rst_n              (rst_n),
        .btn_mode           (btn_mode),
        .btn_confirm        (btn_confirm),

        // ---- 传感�?: 接测试生成器 ----
        .data_valid         (data_valid),
        .heart_rate         (hr_test),
        .temperature        (temp_test),
        .activity_level     (act_test),
        .cadence            (cad_test),

        // ---- SPI OLED + LED + 蜂鸣器 ----
        .oled_csn           (oled_csn),
        .oled_rst           (oled_rst),
        .oled_dcn           (oled_dcn),
        .oled_clk           (oled_clk),
        .oled_dat           (oled_dat),
        .status_led         (status_led),
        .buzzer             (buzzer)
    );

endmodule
