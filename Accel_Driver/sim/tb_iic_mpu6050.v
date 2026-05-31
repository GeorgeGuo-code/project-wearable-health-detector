`timescale 1ns/1ps

module tb_iic_mpu6050;

reg clk;
reg rst_n;
reg read_en;
wire scl;
wire sda;

wire [7:0] acc_x_h, acc_x_l, acc_y_h, acc_y_l, acc_z_h, acc_z_l;
wire [7:0] gyro_x_h, gyro_x_l, gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l;
wire init_done, data_valid;
wire sda_link, sda_r;
wire [3:0] state;

iic_mpu6050 dut (
    .clk(clk), .rst_n(rst_n), .read_en(read_en),
    .scl(scl), .sda(sda),
    .acc_x_h(acc_x_h), .acc_x_l(acc_x_l),
    .acc_y_h(acc_y_h), .acc_y_l(acc_y_l),
    .acc_z_h(acc_z_h), .acc_z_l(acc_z_l),
    .gyro_x_h(gyro_x_h), .gyro_x_l(gyro_x_l),
    .gyro_y_h(gyro_y_h), .gyro_y_l(gyro_y_l),
    .gyro_z_h(gyro_z_h), .gyro_z_l(gyro_z_l),
    .init_done(init_done), .data_valid(data_valid),
    .sda_link(sda_link), .sda_r(sda_r), .state(state)
);

// 100MHz clock
initial clk = 0;
always #5 clk = ~clk;

// I2C pull-up on SDA (weak)
pullup(sda);

// ============================================================
// MPU6050 I2C Slave Behavioral Model
// Uses @(posedge scl) / @(negedge scl) for I2C timing
// ============================================================
reg sda_oe;
reg sda_out;
assign sda = sda_oe ? sda_out : 1'bz;

// Slave state
reg [7:0] rx_byte;
reg [3:0] bit_idx;        // 0-7 = data bits, 8 = ACK
reg       addr_matched;
reg       is_read_op;
reg [7:0] cur_reg;
reg [1:0] wr_phase;       // write phase: 0=addr, 1=reg, 2=data

// Register file
reg [7:0] reg_mem[0:255];
reg [3:0] rd_bit;       // read data bit position
reg       ack_pending;   // ACK to be driven on next SCL negedge

initial begin
    reg_mem[8'h75] = 8'h68;  // WHO_AM_I
    reg_mem[8'h3B] = 8'hA1;  // ACCEL_XOUT_H
    reg_mem[8'h3C] = 8'hB2;  // ACCEL_XOUT_L
    reg_mem[8'h3D] = 8'hC3;  // ACCEL_YOUT_H
    reg_mem[8'h3E] = 8'hD4;  // ACCEL_YOUT_L
    reg_mem[8'h3F] = 8'hE5;  // ACCEL_ZOUT_H
    reg_mem[8'h40] = 8'hF6;  // ACCEL_ZOUT_L
    reg_mem[8'h43] = 8'h1A;  // GYRO_XOUT_H
    reg_mem[8'h44] = 8'h2B;  // GYRO_XOUT_L
    reg_mem[8'h45] = 8'h3C;  // GYRO_YOUT_H
    reg_mem[8'h46] = 8'h4D;  // GYRO_YOUT_L
    reg_mem[8'h47] = 8'h5E;  // GYRO_ZOUT_H
    reg_mem[8'h48] = 8'h6F;  // GYRO_ZOUT_L
end

// START condition detection: SDA falls while SCL is high
// Use @(negedge sda) for asynchronous detection
always @(negedge sda) begin
    // Only detect START if we are NOT the one driving SDA low (e.g. during ACK)
    if (scl == 1'b1 && !sda_oe) begin
        // START condition: master pulls SDA low while SCL=1
        $display("  [SLAVE] START detected at t=%0t", $time);
        bit_idx   = 0;
        rx_byte   = 0;
        addr_matched = 0;
        is_read_op   = 0;
        wr_phase     = 0;
        sda_oe       = 0;
    end
end

// STOP condition: SDA rises while SCL is high
always @(posedge sda) begin
    if (scl == 1'b1) begin
        // STOP condition - end of transaction
        addr_matched = 0;
        sda_oe       = 0;
    end
end

// Single merged posedge SCL block: sample data bits, then drive ACK
// Uses blocking assignments internally for correct ordering within this block
always @(posedge scl) begin
    if (bit_idx < 8) begin
        // Shift in data bit (MSB first) - blocking for immediate use
        rx_byte = {rx_byte[6:0], sda};
        bit_idx = bit_idx + 1;

        // If this was the 8th data bit (bit_idx just went 7→8),
        // rx_byte now has all 8 bits. Drive ACK on SDA immediately.
        if (bit_idx == 8) begin
            if (!addr_matched) begin
                // First byte: device address
                $display("  [SLAVE] Addr byte: rx=0x%02X [7:1]=0x%X", rx_byte, rx_byte[7:1]);
                if (rx_byte[7:1] == 7'h68) begin
                    addr_matched = 1;
                    is_read_op   = rx_byte[0];
                    ack_pending  = 1;  // Will drive ACK on next negedge
                    $display("  [SLAVE] ACK addr (R/W=%d)", rx_byte[0]);
                end else begin
                    $display("  [SLAVE] NACK - addr mismatch");
                end
            end
            else if (!is_read_op && wr_phase == 0) begin
                // First write byte after address: register address
                cur_reg   = rx_byte;
                wr_phase  = 1;
                ack_pending = 1;  // ACK on next negedge
            end
            else if (!is_read_op && wr_phase == 1) begin
                // Second write byte: data to write
                reg_mem[cur_reg] = rx_byte;
                wr_phase  = 0;
                ack_pending = 1;  // ACK on next negedge
                $display("  [SLAVE] Wrote reg 0x%02X = 0x%02X", cur_reg, rx_byte);
            end
            else if (is_read_op && wr_phase == 1) begin
                // Register address in read transaction
                wr_phase  = 0;
                ack_pending = 1;  // ACK on next negedge
            end
        end
    end
    // else: bit_idx==8 (ACK clock posedge) - ACK is already being driven
    //       No action needed; negedge will release it
end

// Negedge SCL: manage ACK timing + read data setup
// ACK timing: drive ACK at 8th negedge, hold through 9th SCL, release at 9th negedge
always @(negedge scl) begin
    if (ack_pending) begin
        // 8th negedge after ACK was committed: drive ACK on SDA
        sda_oe  = 1;
        sda_out = 1'b0;
        ack_pending = 0;
        // bit_idx stays 8 through the 9th SCL
    end
    else if (bit_idx == 8) begin
        // 9th negedge: release ACK, reset for next byte
        sda_oe  = 0;
        bit_idx = 0;
        rx_byte = 0;

        // For read: set up first data bit (MSB) during SCL low
        if (addr_matched && is_read_op) begin
            sda_oe  = 1;
            sda_out = reg_mem[cur_reg][7];
            rd_bit  = 1;
        end
    end
    else if (addr_matched && is_read_op) begin
        // During read data transfer: set up next bit on each SCL falling edge
        if (rd_bit < 8) begin
            sda_oe  = 1;
            sda_out = reg_mem[cur_reg][7 - rd_bit];
            rd_bit  = rd_bit + 1;
        end else begin
            sda_oe = 0;
            rd_bit = 0;
        end
    end
end

// ============================================================
// Test sequence
// ============================================================
integer sim_cycle;

initial begin
    $dumpfile("tb_iic_mpu6050.vcd");
    $dumpvars(0, tb_iic_mpu6050);

    rst_n   = 0;
    read_en = 0;

    // Initialize slave
    sda_oe  = 0;
    bit_idx = 0;
    addr_matched = 0;

    // Reset pulse
    #200;
    rst_n = 1;

    // Wait for INIT_WAIT (short for debug) + margin
    $display("=== Waiting for INIT_WAIT ===");
    #50_000;

    $display("State=%d times=%d at t=%0t", dut.state, dut.times, $time);

    // Enable reading
    read_en = 1;
    $display("read_en=1 at t=%0t", $time);

    // Monitor every 500us
    for (sim_cycle = 0; sim_cycle < 500; sim_cycle = sim_cycle + 1) begin
        #500_000;  // 500us

        // Print only key state transitions
        if (dut.state == 9 || init_done && data_valid || sim_cycle < 10) begin
            $display("[t=%0t] state=%d times=%d num=%d",
                     $time, dut.state, dut.times, dut.num);
        end

        if (init_done && data_valid) begin
            $display("\n========== SUCCESS ==========");
            $display("ACC_X: %02X %02X", acc_x_h, acc_x_l);
            $display("ACC_Y: %02X %02X", acc_y_h, acc_y_l);
            $display("ACC_Z: %02X %02X", acc_z_h, acc_z_l);
            $display("GYRO_X: %02X %02X", gyro_x_h, gyro_x_l);
            $display("GYRO_Y: %02X %02X", gyro_y_h, gyro_y_l);
            $display("GYRO_Z: %02X %02X", gyro_z_h, gyro_z_l);

            // Verify
            if (acc_x_h == 8'hA1) $display("PASS: ACC_X_H = A1");
            else $display("FAIL: ACC_X_H expected A1 got %02X", acc_x_h);
            if (acc_x_l == 8'hB2) $display("PASS: ACC_X_L = B2");
            else $display("FAIL: ACC_X_L expected B2 got %02X", acc_x_l);
            if (acc_y_h == 8'hC3) $display("PASS: ACC_Y_H = C3");
            else $display("FAIL: ACC_Y_H expected C3 got %02X", acc_y_h);
            if (gyro_z_l == 8'h6F) $display("PASS: GYRO_Z_L = 6F");
            else $display("FAIL: GYRO_Z_L expected 6F got %02X", gyro_z_l);

            #10_000_000;
            $finish;
        end
    end

    $display("\nFAIL: Timeout. state=%d times=%d", dut.state, dut.times);
    $finish;
end

// Global timeout
initial begin
    #20_000_000;  // 20ms
    $display("ERROR: Global timeout");
    $finish;
end

endmodule
