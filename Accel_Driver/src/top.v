module top (
    input clk,
    input rst_n,
    input [1:0] sel,      // 2'b00: X-axis, 2'b01: Y-axis, 2'b10: Z-axis
    output scl,
    inout sda,
    output [7:0] acc_h,   // Selected axis high byte
    output [7:0] acc_l,   // Selected axis low byte
    output data_valid     // Data ready flag
);

wire [7:0] acc_x_h, acc_x_l;
wire [7:0] acc_y_h, acc_y_l;
wire [7:0] acc_z_h, acc_z_l;
wire init_done;
wire iic_data_valid;

assign acc_h = (sel == 2'd0) ? acc_x_h :
               (sel == 2'd1) ? acc_y_h :
               (sel == 2'd2) ? acc_z_h : 8'h00;

assign acc_l = (sel == 2'd0) ? acc_x_l :
               (sel == 2'd1) ? acc_y_l :
               (sel == 2'd2) ? acc_z_l : 8'h00;

assign data_valid = init_done;

iic_mpu6050 iic_mpu6050_inst (
    .clk(clk),
    .rst_n(rst_n),
    .scl(scl),
    .sda(sda),
    .acc_x_h(acc_x_h),
    .acc_x_l(acc_x_l),
    .acc_y_h(acc_y_h),
    .acc_y_l(acc_y_l),
    .acc_z_h(acc_z_h),
    .acc_z_l(acc_z_l),
    .gyro_x_h(),
    .gyro_x_l(),
    .gyro_y_h(),
    .gyro_y_l(),
    .gyro_z_h(),
    .gyro_z_l(),
    .init_done(init_done),
    .data_valid(iic_data_valid)
);

endmodule