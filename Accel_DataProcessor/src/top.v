  // =============================================================================
  // Module:  top
  // Project: EGO1 Health Monitoring Wearable
  // =============================================================================
  //
  // 顶层模块，连接：
  //   1. MPU6050 I2C 驱动（iic_mpu6050.v）── 采集原始数据
  //   2. Block Design (MPU6050_central)   ── 处理数据（MicroBlaze + 算法）
  //
  // 信号流：
  //   MPU6050 → iic_mpu6050.v → {mpu_data1, mpu_data2, mpu_data3, mpu_status}
  //                            ↓
  //                         BD (MicroBlaze)
  //                            ↓
  //              {gpio3_o_tri_o, gpio4_o_tri_o} = 运动加速度 + 模长
  //                            ↓
  //                         top.v 输出
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
// Dependencies: 
// 
// Revision:
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
      
      input          uart_rtl_0_rxd,
      output         uart_rtl_0_txd
  );

      // ============================================
      // BD 内部信号
      // ============================================
      wire [31:0] mpu_data1;
      wire [31:0] mpu_data2;
      wire [31:0] mpu_data3;
      wire [1:0]  mpu_status;

      // ============================================
      // 1. 例化 Block Design
      // ============================================
      MPU6050_central_wrapper MPU6050_central_i (
          .clk_100MHz      (clk_100MHz),
          .reset_rtl_0     (reset_btn),
          // ... UART 等其他端口
          .mpu_data1          (mpu_data1),     // BD 里的 AXI GPIO 输出
          .mpu_data2          (mpu_data2),
          .mpu_data3          (mpu_data3),
          .mpu_status         (mpu_status),
          .gpio3_o_tri_o      (),  // 暂时不接
          .gpio4_o_tri_o      (),
          .uart_rtl_0_txd     (uart_rtl_0_txd),
          .uart_rtl_0_rxd     (uart_rtl_0_rxd)
      );

      // ============================================
      // 2. 例化 I2C 驱动 wrapper
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

  endmodule
