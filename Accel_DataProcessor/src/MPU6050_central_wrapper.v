//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2023.2 (win64) Build 4029153 Fri Oct 13 20:14:34 MDT 2023
//Date        : Sun Jun 14 14:53:15 2026
//Host        : GEORGEPC2 running 64-bit major release  (build 9200)
//Command     : generate_target MPU6050_central_wrapper.bd
//Design      : MPU6050_central_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
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
