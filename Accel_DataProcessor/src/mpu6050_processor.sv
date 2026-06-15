// =============================================================================
// Module:  mpu6050_processor
// Project: EGO1 Health Monitoring Wearable
// Purpose: Thin wrapper around Accel_DataProcessor/src/top.v (which is
//          `module top`, kept as the EGO1 standalone test top — DO NOT
//          rename it to avoid breaking standalone bitstream).
//
//          This wrapper exists so `top_integration.v` can instantiate the
//          MPU6050 processor under a non-conflicting module name.
//
//          Signal map (Verilog → SV wrapper):
//            clk_100MHz    → clk
//            reset_btn     → rst
//            read_en_switch→ en (1 = always read)
//            scl, sda      → scl, sda
//            bd_uart_txd   → uart_tx
//            cadence[15:0] → cadence
//            step_count[15:0] → step_count
//
// Build: Vivado adds this file as part of Accel_DataProcessor/src/ source set.
// =============================================================================
`timescale 1ns / 1ps

module mpu6050_processor (
    input  wire        clk,            // 100 MHz
    input  wire        rst,            // active-high (matches `top.v` reset_btn)
    input  wire        en,             // 1 = keep reading MPU6050

    inout  wire        sda,
    output wire        scl,

    output wire        uart_tx,        // BD uart_rtl_0_txd (MicroBlaze xil_printf)
    output wire [15:0] cadence,        // steps-per-minute, BD GPIO_3[15:0]
    output wire [15:0] step_count      // cumulative step count, BD GPIO_3[31:16]
);

    top u_mpu6050_top (
        .clk_100MHz     (clk),
        .reset_btn      (rst),
        .read_en_switch (en),

        .scl            (scl),
        .sda            (sda),

        .bd_uart_txd    (uart_tx),

        .cadence        (cadence),
        .step_count     (step_count)
    );

endmodule
