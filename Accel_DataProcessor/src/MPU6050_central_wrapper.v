  // =============================================================================
  // Module:  MPU6050_central_wrapper
  // File:    <your_path>/MPU6050_central_wrapper.v
  // Project: EGO1 Health Monitoring Wearable
  // Version: 1.0
  // Date:    2026-06-07
  // =============================================================================
  //
  // -----------------------------------------------------------------------------
  // 功能概述
  // -----------------------------------------------------------------------------
  //   MPU6050 中央处理模块（Block Design 顶层 wrapper）
  //   - 通过 MicroBlaze 软核从 I2C 读取 MPU6050 6 轴数据
  //   - 在 MicroBlaze 中完成数据处理（互补滤波 + 重力补偿）
  //   - 输出运动加速度三轴分量 + 加速度模长
  //
  // -----------------------------------------------------------------------------
  // 时钟与复位
  // -----------------------------------------------------------------------------
  //   clk_100MHz       : 100 MHz 系统时钟（EGO1 P17 引脚）
  //   reset_rtl_0      : 复位信号（active high，由 EGO1 按钮提供）
  //
  // -----------------------------------------------------------------------------
  // 串口（UART @ 9600 baud）
  // -----------------------------------------------------------------------------
  //   uart_rtl_0_rxd   : 串口接收（FPGA RX，EGO1 N5）
  //   uart_rtl_0_txd   : 串口发送（FPGA TX，EGO1 T4）
  //
  //   串口协议：115200-8-N-1（注意：实际 BD 中配置为 9600）
  //   输出格式：每 160ms 一行，调试用
  //     RAW ax=<LSB> ay=<LSB> az=<LSB> gx=<LSB> gy=<LSB> gz=<LSB> |
  //     cg  ax=<0.01g> ay=<0.01g> az=<0.01g> |
  //     att r=<deg> p=<deg> |
  //     MOT ax=<0.01g> ay=<0.01g> az=<0.01g> mag=<0.01g>
  //
  // -----------------------------------------------------------------------------
  // 输入端口（来自 iic_mpu6050.v）
  // -----------------------------------------------------------------------------
  //   mpu_data1[31:0]  : GPIO_0 Ch1  ── {acc_x_h, acc_x_l, acc_y_h, acc_y_l}
  //                                          31..24  acc_x 高字节
  //                                          23..16  acc_x 低字节
  //                                           15..8  acc_y 高字节
  //                                            7..0  acc_y 低字节
  //                      物理含义：原始加速度 X/Y（16-bit signed，±4g 量程）
  //                                1g = 8192 LSB
  //
  //   mpu_data2[31:0]  : GPIO_0 Ch2  ── {acc_z_h, acc_z_l, gyro_x_h, gyro_x_l}
  //                                          31..24  acc_z 高字节
  //                                          23..16  acc_z 低字节
  //                                           15..8  gyro_x 高字节
  //                                            7..0  gyro_x 低字节
  //                      物理含义：原始加速度 Z + 陀螺仪 X
  //                                acc: ±4g（1g = 8192 LSB）
  //                                gyro: ±2000dps（1 dps = 16.4 LSB）
  //
  //   mpu_data3[31:0]  : GPIO_1      ── {gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l}
  //                                          31..24  gyro_y 高字节
  //                                          23..16  gyro_y 低字节
  //                                           15..8  gyro_z 高字节
  //                                            7..0  gyro_z 低字节
  //                      物理含义：原始陀螺仪 Y/Z（±2000dps，1 dps = 16.4 LSB）
  //
  //   mpu_status[1:0]  : GPIO_2      ── {data_valid, init_done}
  //                                          [1] init_done   (1 = 初始化完成)
  //                                          [0] data_valid  (1 = 本次数据有效)
  //
  // -----------------------------------------------------------------------------
  // 输出端口（送顶层 Verilog / 上层模块）
  // -----------------------------------------------------------------------------
  //
  //   gpio3_o_tri_o[31:0] : GPIO_3 ── {acc_mag[15:0], ax_motion[15:0]}
  //                              [31:16]  acc_mag   运动加速度模长
  //                              [15:0]   ax_motion 运动加速度 X 轴
  //
  //   gpio4_o_tri_o[31:0] : GPIO_4 ── {ay_motion[15:0], az_motion[15:0]}
  //                              [31:16]  ay_motion 运动加速度 Y 轴
  //                              [15:0]   az_motion 运动加速度 Z 轴
  //
  // -----------------------------------------------------------------------------
  // 输出数值转换公式
  // -----------------------------------------------------------------------------
  //   1. 加速度原始值 → 物理量（0.01g 单位）：
  //        acc_cg = raw_value / 82
  //      例：raw=8192 → 100 (0.01g) = 1.0g
  //          raw=4096 → 50  (0.01g) = 0.5g
  //
  //   2. 陀螺仪原始值 → 物理量（0.1dps 单位）：
  //        gyro_dps10 = raw_value / 2
  //      例：raw=164  → 82  (0.1dps) = 8.2 dps
  //
  //   3. 输出值单位（均已换算为 0.01g）：
  //        ax_motion, ay_motion, az_motion  → 0.01g 单位
  //        acc_mag                          → 0.01g 单位（已开方）
  //
  //      物理值(g) = 输出值 / 100
  //      例：ax_motion=23 → 0.23g
  //          acc_mag=150  → 1.50g
  //
  //   4. 数据范围：
  //        ax_motion, ay_motion, az_motion : -32768 ~ +32767 (±327g)
  //        acc_mag                          : 0 ~ 65535 (理论)
  //        实际有效范围                     : ±400 (0.01g) = ±4g
  //
  // -----------------------------------------------------------------------------
  // 内部算法（MicroBlaze C 代码）
  // -----------------------------------------------------------------------------
  //   1. 字节拼装：{h, l} → 16-bit signed
  //   2. 单位换算：LSB → 0.01g / 0.1dps
  //   3. 互补滤波（α=0.96, dt=10ms）：
  //        roll, pitch = α × gyro_integral + (1-α) × accel_angle
  //      注：姿态角 roll/pitch 为内部变量，不对外输出
  //   4. 重力补偿（查表法，sin/cos 表单位 = 0.01g）：
  //        gx = -sin(pitch)
  //        gy =  sin(roll) × cos(pitch) / 100
  //        gz =  cos(roll) × cos(pitch) / 100
  //   5. 运动加速度：
  //        ax_motion = ax_raw - gx
  //        ay_motion = ay_raw - gy
  //        az_motion = az_raw - gz
  //   6. 模长（整数 sqrt）：
  //        acc_mag = sqrt(ax_motion? + ay_motion? + az_motion?)
  //
  // -----------------------------------------------------------------------------
  // 资源占用（估算）
  // -----------------------------------------------------------------------------
  //   LUTs        : ~ 3000
  //   FFs         : ~ 2000
  //   BRAM        : 8 KB（LMB BRAM for MicroBlaze）
  //   DSP         : 5  (MicroBlaze 内置)
  //
  // -----------------------------------------------------------------------------
  // 约束（XDC）
  // -----------------------------------------------------------------------------
  //   clk_100MHz  : P17 (LVCMOS33)
  //   reset_btn   : R17 (LVCMOS33)  -- active high
  //   scl         : B16 (LVCMOS33)
  //   sda         : B17 (LVCMOS33)
  //   uart_rtl_0_rxd : N5  (LVCMOS33)
  //   uart_rtl_0_txd : T4  (LVCMOS33)
  //
  // =============================================================================
`timescale 1 ps / 1 ps

module MPU6050_central_wrapper
   (clk_100MHz,
    gpio3_o_tri_o,
    gpio4_o_tri_o,
    mpu_data1,
    mpu_data2,
    mpu_data3,
    mpu_status,
    reset_rtl_0,
    uart_rtl_0_rxd,
    uart_rtl_0_txd);
  input clk_100MHz;
  output [31:0]gpio3_o_tri_o;
  output [31:0]gpio4_o_tri_o;
  input [31:0]mpu_data1;
  input [31:0]mpu_data2;
  input [31:0]mpu_data3;
  input [1:0]mpu_status;
  input reset_rtl_0;
  input uart_rtl_0_rxd;
  output uart_rtl_0_txd;

  wire clk_100MHz;
  wire [31:0]gpio3_o_tri_o;
  wire [31:0]gpio4_o_tri_o;
  wire [31:0]mpu_data1;
  wire [31:0]mpu_data2;
  wire [31:0]mpu_data3;
  wire [1:0]mpu_status;
  wire reset_rtl_0;
  wire uart_rtl_0_rxd;
  wire uart_rtl_0_txd;

  MPU6050_central MPU6050_central_i
       (.clk_100MHz(clk_100MHz),
        .gpio3_o_tri_o(gpio3_o_tri_o),
        .gpio4_o_tri_o(gpio4_o_tri_o),
        .mpu_data1(mpu_data1),
        .mpu_data2(mpu_data2),
        .mpu_data3(mpu_data3),
        .mpu_status(mpu_status),
        .reset_rtl_0(reset_rtl_0),
        .uart_rtl_0_rxd(uart_rtl_0_rxd),
        .uart_rtl_0_txd(uart_rtl_0_txd));
endmodule