//==============================================================================
// top_oled_test.v
// 顶层测试模块，包装 cs13 OLED 驱动。
// - 上电后延迟 ~200 ms（让 OLED VCC 稳定）再释放内部复位
// - 外部复位按键高有效（按下=1）-> 内部 cs13 复位为低有效，取反
// - OLED SPI 引脚经 OBUF 缓冲输出（Xilinx 7-series 习惯）
// - 一颗心跳 LED 指示设计在跑
//
// 需要在 XDC 里把端口名映射到具体 pin：
//   clk_100mhz -> 时钟脚（100 MHz 振荡器）
//   btn_reset  -> 按键脚（去抖后接进来更好）
//   OLED_CSN/RST/DCN/CLK/DAT -> 板子对应的 OLED 接口
//==============================================================================
module top_oled_test
(
    input  wire clk_100mhz,    // 100 MHz board clock
    input  wire btn_reset,     // active-high reset button (idle = 0)
    input  wire sw_oled_en,    // active-high OLED enable switch (1 = display on, 0 = OLED held in hardware reset, low power)
    output wire led_heartbeat, // status LED, blinks at ~3 Hz
    output wire OLED_CSN,      // OLED chip select   (active low)
    output wire OLED_RST,      // OLED reset         (active low)
    output wire OLED_DCN,      // OLED data/command  (1=data, 0=cmd)
    output wire OLED_CLK,      // OLED SPI clock
    output wire OLED_DAT       // OLED SPI data (MOSI)
);

    // ------------------------------------------------------------------------
    // Power-on delay: 2^23 / 100 MHz ≈ 84 ms, then release internal reset
    // (sticky — saturate the counter at 2^23 so por_done stays high forever)
    //
    // BUG history: originally used `por_cnt[23]` which is just a single bit
    // of a free-running 24-bit counter. That bit toggles every 2^23 cycles,
    // so por_done had a 50% duty cycle with 168 ms period. Every 84 ms the
    // cs13 module got reset, the OLED was re-initialized mid-scan, and the
    // display never had a chance to settle.  Logic-analyzer data confirmed
    // a reset pulse every 167.77 ms (= 2^24 / 100 MHz) on OLED_RST.
    // ------------------------------------------------------------------------
    reg [23:0] por_cnt = 24'd0;
    always @(posedge clk_100mhz) begin
        if (!por_cnt[23]) por_cnt <= por_cnt + 1'b1;  // saturate at 2^23
    end
    wire por_done = por_cnt[23];  // sticky: 1 after 2^23 cycles, never returns to 0

    // ------------------------------------------------------------------------
    // Reset inversion
    //   cs13 内部: if(!rst) -> 低有效复位
    //   用户按键: 高有效复位
    //   POR 没结束 -> 也要保持 cs13 复位
    // ------------------------------------------------------------------------
    wire cs13_rst = por_done & ~btn_reset;  // 1 = running, 0 = in reset

    // ------------------------------------------------------------------------
    // Heartbeat LED: toggle at ~3 Hz (toggle every 2^26 cycles / 2)
    // ------------------------------------------------------------------------
    reg [25:0] hb_cnt = 26'd0;
    always @(posedge clk_100mhz) hb_cnt <= hb_cnt + 1'b1;
    assign led_heartbeat = hb_cnt[25];  // 100 MHz / 2^26 ≈ 1.5 Hz blink

    // ------------------------------------------------------------------------
    // Internal wires from cs13
    // ------------------------------------------------------------------------
    wire oled_csn, oled_rst, oled_dcn, oled_clk, oled_dat;

    // ------------------------------------------------------------------------
    // Instantiate the OLED driver
    // ------------------------------------------------------------------------
    cs13 u_cs13
    (
        .clk      (clk_100mhz),
        .rst      (cs13_rst),
        .en       (sw_oled_en),
        .oled_csn (oled_csn),
        .oled_rst (oled_rst),
        .oled_dcn (oled_dcn),
        .oled_clk (oled_clk),
        .oled_dat (oled_dat)
    );

    // ------------------------------------------------------------------------
    // Output buffers (Xilinx 7-series). 3.3 V LVCMOS33, slow slew for clean edges.
    // ------------------------------------------------------------------------
    OBUF #(.DRIVE(12), .IOSTANDARD("LVCMOS33"), .SLEW("SLOW"))
        obuf_csn (.I(oled_csn), .O(OLED_CSN));

    OBUF #(.DRIVE(12), .IOSTANDARD("LVCMOS33"), .SLEW("SLOW"))
        obuf_rst (.I(oled_rst), .O(OLED_RST));

    OBUF #(.DRIVE(12), .IOSTANDARD("LVCMOS33"), .SLEW("SLOW"))
        obuf_dcn (.I(oled_dcn), .O(OLED_DCN));

    OBUF #(.DRIVE(12), .IOSTANDARD("LVCMOS33"), .SLEW("SLOW"))
        obuf_clk (.I(oled_clk), .O(OLED_CLK));

    OBUF #(.DRIVE(12), .IOSTANDARD("LVCMOS33"), .SLEW("SLOW"))
        obuf_dat (.I(oled_dat), .O(OLED_DAT));

endmodule
