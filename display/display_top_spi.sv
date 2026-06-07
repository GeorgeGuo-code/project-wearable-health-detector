`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// display_top_spi — 显示模块顶层 (cs13 内部渲染版)
//   所有显示逻辑在 cs13 内部, 此处仅透传数据 + LED/蜂鸣器驱动
//////////////////////////////////////////////////////////////////////////////////
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
    output wire        buzzer
);

    // ---- cs13 OLED driver (all rendering internal) ----
    cs13 u_cs13 (
        .clk(clk), .rst(rst_n), .en(1'b1),
        .heart_rate(heart_rate), .temperature(temperature),
        .activity_level(activity_level), .cadence(cadence),
        .health_status(health_status), .health_score(health_score),
        .tach_count(tach_count), .brad_count(brad_count),
        .fever_count(fever_count), .worst_status(worst_status),
        .work_en(work_en), .display_mode(display_mode),
        .oled_csn(oled_csn), .oled_rst(oled_rst), .oled_dcn(oled_dcn),
        .oled_clk(oled_clk), .oled_dat(oled_dat)
    );

    // ---- LED ----
    reg [2:0] led; reg [16:0] blk;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin led<=3'b001; blk<=17'd0; end
        else begin
            blk<=blk+17'd1;
            if (!work_en) led<=blk[16]?3'b000:3'b010;
            else case(health_status) 2'b00:led<=3'b001; 2'b01:led<=3'b010; 2'b10:led<=3'b100; default:led<=3'b001; endcase
        end
    end
    assign status_led=led;

    // ---- Buzzer ----
    reg [15:0] bcnt; reg bpwm;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin bcnt<=16'd0; bpwm<=1'b0; end
        else begin
            if (alarm&&work_en) begin
                bpwm<=(bcnt<16'd25000); bcnt<=(bcnt==16'd50000)?16'd0:bcnt+16'd1;
            end else begin bpwm<=1'b0; bcnt<=16'd0; end
        end
    end
    assign buzzer=bpwm;

endmodule
