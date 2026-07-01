// =============================================================================
// Module:  top_integration
// Project: EGO1 Health Monitoring Wearable
// Purpose: 顶层整合 - 把三条传感器链路 (MAX30102 / MPU6050 / DS18B20) + 转换
//          层 + session 缓存, 串接到 top_spi (sm_main + evaluator + scorer
//          + cs13 OLED + LED + 蜂鸣器).
//
// 数据流:
//   MAX30102 (I2C#1)  →  hr_maxim          →  hr_bpm[7:0]  ─┐
//   MPU6050  (I2C#2)  →  mpu6050_processor →  cadence/step  ─┤
//   DS18B20  (1-Wire) →  /8 转换           →  temp[7:0]     ─┼→ top_spi
//                                                            ─┘
//   session_stats 同时累加 → session_end → cached_*  →  顶层输出
//
// 顶层同时预留:
//   - flash_view_en (1 bit, 输入): 切到 FLASH 视图 OLED 页
//   - flash_csn/sck/mosi/miso: Flash SPI 占位 (本期不接驱动)
//   - cached_*: flash 写库使用
//   session_end_int  → Flash 驱动内部连线 (不占外部 IO)
//
// 设计依据: health_monitor/integration_plan.md
// =============================================================================
`timescale 1ns / 1ps

module top_integration (
    // ---- 时钟 / 复位 ----
    input  wire        clk_100MHz,         // EGO1 100MHz 主时钟 (P17)
    input  wire        reset_btn,          // EGO1 复位按钮 (P4, 高有效)

    // ---- 用户按键 (EGO1 拨码/按键, 按下=高) ----
    input  wire        btn_mode_raw,
    input  wire        btn_confirm_raw,

    // ---- MAX30102 (I2C #1) ----
    inout  wire        sda1,
    output wire        scl1,

    // ---- MPU6050 (I2C #2) ----
    inout  wire        sda2,
    output wire        scl2,

    // ---- DS18B20 (One-Wire, 外接 4.7kΩ 上拉) ----
    inout  wire        dq_temp,

    // ---- OLED SPI (cs13 4-wire + 片选) ----
    output wire        oled_csn,
    output wire        oled_rst,
    output wire        oled_dcn,
    output wire        oled_clk,
    output wire        oled_dat,

    // ---- 状态指示 + 报警 ----
    output wire [2:0]  status_led,
    output wire        buzzer,

    // ---- Flash 接口 (本期占位, 驱动未接) ----
    input  wire        flash_view_en,      // 1 = OLED 切到 FLASH 视图
    output wire        flash_csn,
    output wire        flash_sck,
    output wire        flash_mosi,
    input  wire        flash_miso
//    output wire [15:0] cached_step_count,
//    output wire [7:0]  cached_avg_cadence,
//    output wire [7:0]  cached_avg_hr
);


    wire [15:0] cached_step_count;
    wire [7:0]  cached_avg_cadence;
    wire [7:0]  cached_avg_hr;


    // ========================================================================
    // 按键取反
    // ========================================================================
    wire        rst_n         = ~reset_btn;
    wire        btn_mode      = ~btn_mode_raw;
    wire        btn_confirm   = ~btn_confirm_raw;

    // ========================================================================
    // 1. MAX30102 (I2C #1) + hr_maxim
    // ========================================================================
    wire [17:0] ir_raw;
    wire [17:0] red_raw_unused;
    wire        max_data_valid;
    wire        max_init_done;

    max30102_driver u_max_driver (
        .clk        (clk_100MHz),
        .rst_n      (rst_n),
        .start      (1'b1),                 // 上电即采样
        .scl        (scl1),
        .sda        (sda1),
        .ir_data    (ir_raw),
        .red_data   (red_raw_unused),
        .data_valid (max_data_valid),
        .init_done  (max_init_done),
        .sda_link   (),
        .sda_r      (),
        .state      ()
    );

    wire [7:0]  hr_bpm;
    wire        hr_valid_unused;
    wire        hr_locked_unused;
    wire signed [17:0] hamm_unused;
    wire signed [17:0] thresh_unused;
    wire        dec_valid_unused;

    hr_maxim u_hr_maxim (
        .clk           (clk_100MHz),
        .rst_n         (rst_n),
        .in_valid      (max_data_valid),     // 100Hz 内部驱动
        .ir_ac         (ir_raw),
        .hr_bpm        (hr_bpm),             // 8bit BPM (顶层只用这个)
        .hr_valid      (hr_valid_unused),
        .hr_locked     (hr_locked_unused),
        .hamm_out      (hamm_unused),
        .threshold_out (thresh_unused),
        .dec_valid     (dec_valid_unused)
    );

    // ========================================================================
    // 2. MPU6050 (I2C #2) via mpu6050_processor
    // ========================================================================
    wire [15:0] mpu_cadence;
    wire [15:0] mpu_step_count;
    wire        mpu_uart_tx_unused;

    mpu6050_processor u_mpu6050 (
        .clk        (clk_100MHz),
        .rst        (reset_btn),            // 注意: 顶层 reset_btn 是高有效
        .en         (1'b1),                 // 常读
        .sda        (sda2),
        .scl        (scl2),
        .uart_tx    (mpu_uart_tx_unused),   // 调试 UART, 本期不接 EGO1
        .cadence    (mpu_cadence),
        .step_count (mpu_step_count)
    );

    // ========================================================================
    // 3. DS18B20 (One-Wire) + 温度格式转换
    //    raw = 12bit signed, 0.0625°C/LSB
    //    →  cs13 期望 °C×2 (0.5°C/LSB)
    //    →  signed shift right 3 (= raw / 8)
    // ========================================================================
    wire [15:0] temp_raw;            // 12bit 实际有效 (signed)
    wire        temp_valid;
    wire        temp_dq_out;
    wire        temp_dq_oe;
    wire        dq_temp_in;

    ds18b20_driver u_temp (
        .clk        (clk_100MHz),
        .rst_n      (rst_n),
        .start      (1'b1),
        .dq_in      (dq_temp_in),
        .temperature(temp_raw),
        .data_valid (temp_valid),
        .error      (),
        .dq_out     (temp_dq_out),
        .dq_oe      (temp_dq_oe)
    );

    // IOBUF 双向 (综合时被推断为 IOBUF)
    assign dq_temp    = temp_dq_oe ? temp_dq_out : 1'bz;
    assign dq_temp_in = dq_temp;

    // 12bit signed 0.0625°C/LSB  →  8bit °C×2
    //   0.0625 / 0.5 = 1/8  →  arithmetic shift right 3 (signed)
    //   cs13 温度: 75 = 37.5°C
    wire [7:0]  temperature = $signed(temp_raw[11:0]) >>> 3;

    // ========================================================================
    // 4. 活动等级 (3bit, 0-4)  由 cadence 映射
    //    阈值与详细设计方案 §3.2.4 一致
    // ========================================================================
    function [2:0] cadence_to_level;
        input [7:0] cad;
        begin
            if (cad < 8'd60)        cadence_to_level = 3'd0;  // 静坐
            else if (cad < 8'd100)  cadence_to_level = 3'd1;  // 轻度
            else if (cad < 8'd140)  cadence_to_level = 3'd2;  // 中度
            else if (cad < 8'd180)  cadence_to_level = 3'd3;  // 剧烈
            else                    cadence_to_level = 3'd4;  // 极剧烈
        end
    endfunction
    wire [2:0]  activity_level = cadence_to_level(mpu_cadence[7:0]);
    wire [7:0]  cadence        = mpu_cadence[7:0];   // SPM, 顶层低 8 bit

    // ========================================================================
    // 5. data_valid 汇合
    //    选用 1Hz 温度脉冲作主心跳 (evaluator/scorer 每秒评估一次)
    //    hr_bpm / cadence / activity_level 是稳态寄存器, 不需要每拍脉冲
    // ========================================================================
    wire        data_valid = temp_valid;

    // ========================================================================
    // 6. sm_main 内部信号 (work_en / display_mode / standby_led)
    //    顶层实例化 top_health_monitor (内部含 sm_main + evaluator + scorer)
    //    顶层实例化 top_spi (内部含 top_health_monitor + display_top_spi)
    // ========================================================================
    wire        work_en;
    wire [2:0]  display_mode;
    wire [1:0]  health_status;
    wire        alarm;
    wire        standby_led_unused;
    wire [7:0]  health_score_unused;
    wire [7:0]  tach_count_unused;
    wire [7:0]  brad_count_unused;
    wire [7:0]  fever_count_unused;
    wire [1:0]  worst_status_unused;

    wire [2:0]  led_internal;

    top_spi u_top_spi (
        .clk                (clk_100MHz),
        .rst_n              (rst_n),
        .btn_mode           (btn_mode),
        .btn_confirm        (btn_confirm),
        .data_valid         (data_valid),
        .heart_rate         (hr_bpm),
        .temperature        (temperature),
        .activity_level     (activity_level),
        .cadence            (cadence),

        // OLED
        .oled_csn           (oled_csn),
        .oled_rst           (oled_rst),
        .oled_dcn           (oled_dcn),
        .oled_clk           (oled_clk),
        .oled_dat           (oled_dat),

        .status_led         (led_internal),
        .buzzer             (buzzer),
        .work_en            (work_en),           // ★ 从 top_spi 引出, 供 session_stats/flash_op_seq
        .display_mode       (),                  // 暂不接

        // ★ Flash 视图 + 缓存 (本期新增)
        .flash_view_en      (flash_view_en),
        .cached_step_count  (flash_disp_step),     // mux: session 完成后取 cached, 否则取 load
        .cached_avg_cadence (flash_disp_cad),
        .cached_avg_hr      (flash_disp_hr),
        // ★ 实时步数 (新2页模式用)
        .step_count         (mpu_step_count),
        // ★ Flash 操作提示 (新增, 优先级最高)
        .flash_op_en        (flash_op_en),
        .flash_op_message   (flash_op_message)
    );

    // ========================================================================
    // 7. session_stats — 累加心率/步频, 会话结束锁存, 供 flash 写库
    // ========================================================================
    wire [15:0] cached_step_count_in;
    wire [7:0]  cached_avg_cadence_in;
    wire [7:0]  cached_avg_hr_in;
    wire        session_end_int;

    session_stats u_session_stats (
        .clk                (clk_100MHz),
        .rst_n              (rst_n),
        .work_en            (work_en),
        .data_valid         (data_valid),
        .heart_rate         (hr_bpm),
        .cadence            (cadence),
        .step_count         (mpu_step_count),
        .cached_step_count  (cached_step_count_in),
        .cached_avg_cadence (cached_avg_cadence_in),
        .cached_avg_hr      (cached_avg_hr_in),
        .session_end        (session_end_int)
    );

    // ========================================================================
    // 8. 顶层输出回接
    // ========================================================================
    assign cached_step_count  = cached_step_count_in;
    assign cached_avg_cadence = cached_avg_cadence_in;
    assign cached_avg_hr      = cached_avg_hr_in;

    // ========================================================================
    // 9. Flash 驱动: flash_top + flash_op_seq
    //   替换之前 CSN/CLK/MOSI 拉死的占位
    //   flash_view_en 仍是顶层 input (XDC 绑拨码开关或 PULLDOWN)
    // ========================================================================
    // ---- flash_top 接口信号 (组合式连接到 flash_op_seq) ----
    wire        flash_start_pulse;
    wire        flash_busy, flash_done, flash_error;
    wire        flash_wdata_valid, flash_wdata_ready;
    wire [7:0]  flash_rdata;
    wire        flash_rdata_valid;
    wire [3:0]  flash_op_code;
    wire [23:0] flash_addr;
    wire [15:0] flash_len;
    wire [7:0]  flash_wdata;

    // ---- flash_op_seq 状态信号 ----
    wire        flash_op_en;
    wire [1:0]  flash_op_message;
    wire [15:0] load_step;
    wire [7:0]  load_avg_cad;
    wire [7:0]  load_avg_hr;
    wire        load_valid;
    wire        delete_active;
    wire        load_in_progress;
    wire        session_happened;

    // ---- flash_disp_*: FLASH 视图显示用 mux
    //      优先级: 1) save 完成后显示 cached (本次数据)
    //              2) load_valid 显示 load (上次保存的数据)
    //              3) 都无 → 0
    wire [15:0] flash_disp_step = session_happened   ? cached_step_count_in :
                                  load_valid         ? load_step            : 16'd0;
    wire [7:0]  flash_disp_cad  = session_happened   ? cached_avg_cadence_in :
                                  load_valid         ? load_avg_cad         : 8'd0;
    wire [7:0]  flash_disp_hr   = session_happened   ? cached_avg_hr_in      :
                                  load_valid         ? load_avg_hr          : 8'd0;

    // ---- save_*: session_end_int 触发时传给 flash_op_seq 的待写数据 ----
    //   (session_stats 已经在 session_end 锁存了 cached_*, 直接传即可)
    wire [15:0] save_step       = cached_step_count_in;
    wire [7:0]  save_avg_cad    = cached_avg_cadence_in;
    wire [7:0]  save_avg_hr     = cached_avg_hr_in;

    // ---- flash_top (CLK_DIV=256 → SCK≈390kHz, 方便逻辑分析仪抓取; 验证后改 4) ----
    flash_top #(.CLK_DIV(256)) u_flash_top (
        .clk           (clk_100MHz),
        .rst_n         (rst_n),
        .start         (flash_start_pulse),
        .op            (flash_op_code),
        .addr          (flash_addr),
        .len           (flash_len),
        .wdata         (flash_wdata),
        .wdata_valid   (flash_wdata_valid),
        .wdata_ready   (flash_wdata_ready),
        .rdata         (flash_rdata),
        .rdata_valid   (flash_rdata_valid),
        .busy          (flash_busy),
        .done          (flash_done),
        .error         (flash_error),
        .status_reg1   (),
        .sck           (flash_sck),
        .csn           (flash_csn),
        .mosi          (flash_mosi),
        .miso          (flash_miso)
    );

    // ---- flash_op_seq: 用户流程状态机 ----
    flash_op_seq u_flash_op_seq (
        .clk               (clk_100MHz),
        .rst_n             (rst_n),
        .session_end_int   (session_end_int),
        .flash_view_en     (flash_view_en),
        .btn_confirm       (btn_confirm),       // 已取反, 0=按下
        .btn_mode          (btn_mode),
        .work_en           (work_en),
        .save_step         (save_step),
        .save_avg_cad      (save_avg_cad),
        .save_avg_hr       (save_avg_hr),
        .op                (flash_op_code),
        .addr              (flash_addr),
        .len               (flash_len),
        .start             (flash_start_pulse),
        .wdata_valid       (flash_wdata_valid),
        .wdata             (flash_wdata),
        .wdata_ready       (flash_wdata_ready),
        .rdata             (flash_rdata),
        .rdata_valid       (flash_rdata_valid),
        .busy              (flash_busy),
        .done              (flash_done),
        .error             (flash_error),
        .flash_op_en       (flash_op_en),
        .flash_op_message  (flash_op_message),
        .load_step         (load_step),
        .load_avg_cad      (load_avg_cad),
        .load_avg_hr       (load_avg_hr),
        .load_valid        (load_valid),
        .delete_active     (delete_active),
        .load_in_progress  (load_in_progress),
        .session_happened  (session_happened)
    );

    // ★ DEBUG: status_led[2] = flash_ctrl busy, 确认 flash 控制器是否在跑
    //   闪存操作期间 LED2 会亮; 如果一直不亮, flash_ctrl 从未收到 start
    assign status_led = {led_internal[2] | flash_busy, led_internal[1:0]};

    // 未使用信号消警告 (保留观察)
    //   max_init_done:  MAX30102 初始化完成, 可送 LED 指示
    //   mpu_init_done_bd 暂未引出
    //   health_score/tach/brad/fever/worst_status: scorer 输出, 暂未用
    //   max_red_data_unused: 暂未使用
    //   mpu_uart_tx_unused:  BD UART, 暂未接 EGO1
    wire _unused = &{1'b0, max_init_done, standby_led_unused, health_score_unused,
                     tach_count_unused, brad_count_unused, fever_count_unused,
                     worst_status_unused, red_raw_unused, mpu_uart_tx_unused,
                     1'b0};

endmodule
