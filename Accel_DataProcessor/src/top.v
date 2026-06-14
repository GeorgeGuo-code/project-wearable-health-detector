// =============================================================================
// Module:  top
// Project: EGO1 Health Monitoring Wearable
// =============================================================================
//
// 顶层模块 (2026/06/14 简化版)：
//   1. MPU6050 I2C 驱动 (iic_mpu6050.v)  ── 采集原始数据
//   2. Block Design (MPU6050_central)    ── MicroBlaze 处理 (重力分离 + 步频检测)
//
// 信号流：
//   MPU6050 → iic_mpu6050.v → {mpu_data1, mpu_data2, mpu_data3, mpu_status}
//                            ↓
//                         BD (MicroBlaze)
//                            ↓
//              运动加速度 acc_mag (gpio3_o[31:16])
//                            ↓
//                  MicroBlaze C 端步频检测算法
//                            ↓
//                  uart_rtl_0_txd (BD UART @ 9600 baud)
//
// 注意：步频检测算法已从 step_detector.v 移植到 main.c (MicroBlaze C 端)。
//       cadence 通过 BD 内置 UART (T4 引脚) 打印。
//       cadence / step_count 顶层输出保留 (可选, 也可删)。
//
// =============================================================================
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/06/06 20:43:52
// Design Name:
// Module Name: top
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// 2026/06/14  简化：删除 step_detector.v 模块引用
//             - 步频检测算法全部在 MicroBlaze C 端实现
//             - cadence 通过 BD UART 输出 (uart_rtl_0_txd, 9600 baud)
//             - 顶层 cadence / step_count 端口保留供将来扩展
//
// Dependencies: iic_mpu6050.v
//
// Revision:
// Revision 0.03 - 移除 step_detector, 算法上移 MicroBlaze
// Revision 0.02 - 集成 step_detector
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module top (
      input          clk_100MHz,        // EGO1 100MHz 时钟 (P17)
      input          reset_btn,         // EGO1 按钮 (P4)
      input          read_en_switch,    // 拨码开关（可选）

      output         scl,               // PMOD SCL
      inout          sda,                // PMOD SDA

      // ==== BD UART 输出 (步频 / 调试) ====
      // BD 内置 UART TX 引脚, 由 MicroBlaze 通过 xil_printf 驱动
      // 9600 baud, 8N1
      output          bd_uart_txd,       // EGO1 T4 (来自 BD uart_rtl_0_txd)

      // ==== 可选: 顶层 cadence / step_count 输出 (供 LED/外部使用) ====
      output [15:0]  cadence,           // 步频 (spm), 来自 MicroBlaze GPIO_3[15:0]
      output [15:0]  step_count         // 累计步数 (16-bit), 来自 MicroBlaze GPIO_3[31:16]
  );

      // ============================================
      // BD 内部信号
      // ============================================
      wire [31:0] mpu_data1;
      wire [31:0] mpu_data2;
      wire [31:0] mpu_data3;
      wire [1:0]  mpu_status;

      // BD 输出（MicroBlaze → FPGA fabric）
      wire [31:0] gpio3_o;        // {acc_mag[31:16], ax_motion[15:0]}
      wire [31:0] gpio4_o;        // {ay_motion[31:16], az_motion[15:0]}

      // BD UART - 9600 baud, MicroBlaze 端 xil_printf 输出
      wire        bd_uart_rxd = 1'b1;  // BD UART RX tied idle - prevents RX interrupts

      // ============================================
      // 1. 例化 Block Design
      // ============================================
      MPU6050_central_wrapper MPU6050_central_i (
          .clk_100MHz      (clk_100MHz),
          .reset_rtl_0     (reset_btn),
          .mpu_data1       (mpu_data1),     // BD 里的 AXI GPIO 输入
          .mpu_data2       (mpu_data2),
          .mpu_data3       (mpu_data3),
          .mpu_status      (mpu_status),
          .gpio3_o_tri_o   (gpio3_o),       // 运动加速度模 + ax
          .gpio4_o_tri_o   (gpio4_o),       // ay + az
          .uart_rtl_0_txd  (bd_uart_txd),   // 顶层直接引出到 EGO1 T4
          .uart_rtl_0_rxd  (bd_uart_rxd)
      );

      // ============================================
      // 2. 例化 I2C 驱动
      // ============================================
      wire [7:0] acc_x_h, acc_x_l, acc_y_h, acc_y_l;
      wire [7:0] acc_z_h, acc_z_l, gyro_x_h, gyro_x_l;
      wire [7:0] gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l;
      wire       init_done, data_valid;

      iic_mpu6050 i_i2c_wrapper (
          .clk        (clk_100MHz),
          .rst_n      (~reset_btn),
          .read_en    (read_en_switch),
          .scl        (scl),
          .sda        (sda),
          .acc_x_h    (acc_x_h),
          .acc_x_l    (acc_x_l),
          .acc_y_h    (acc_y_h),
          .acc_y_l    (acc_y_l),
          .acc_z_h    (acc_z_h),
          .acc_z_l    (acc_z_l),
          .gyro_x_h   (gyro_x_h),
          .gyro_x_l   (gyro_x_l),
          .gyro_y_h   (gyro_y_h),
          .gyro_y_l   (gyro_y_l),
          .gyro_z_h   (gyro_z_h),
          .gyro_z_l   (gyro_z_l),
          .init_done  (init_done),
          .data_valid (data_valid)
      );

      // ============================================
      // 3. 数据打包 → 灌到 BD 的 AXI GPIO
      // ============================================
      assign mpu_data1     = {acc_x_h, acc_x_l, acc_y_h, acc_y_l};
      assign mpu_data2     = {acc_z_h, acc_z_l, gyro_x_h, gyro_x_l};
      assign mpu_data3     = {gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l};
      assign mpu_status    = {data_valid, init_done};

      // ============================================
      // 4. 顶层 cadence / step_count 输出
      //    现在 cadence 和 step_count 由 MicroBlaze C 端计算,
      //    打包到 GPIO_3: [31:16]=step_count, [15:0]=cadence
      //    顶层 cadence / step_count 引脚保留, 如需直接驱动 LED 可使用。
      //    如果不需要, 可在 XDC 中删去对应约束。
      // ============================================
      assign cadence    = gpio3_o[15:0];
      assign step_count = gpio3_o[31:16];

  endmodule
