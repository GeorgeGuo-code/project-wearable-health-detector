//=============================================================================
// Module     : iic_mpu6050
// Author     :
// Date       :
// Description: I2C Master Driver for MPU6050 6-axis IMU (Accel + Gyro)
//              Performs device initialization then continuous register reading.
//
// Features:
//   - 100ms power-up delay for MPU6050 stabilization
//   - Automatic init: PWR_MGMT_1, SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACC_CONFIG
//   - Continuous 12-register burst read (6 accel + 6 gyro bytes)
//   - NACK auto-retry (failed transactions re-attempt on next cycle)
//   - 200kHz I2C SCL (standard-mode compatible)
//
// I2C Protocol:
//   - SCL frequency : 200 kHz  (5us period @ 100MHz = 500 clk cycles)
//   - Device address: 0x68 (7-bit, write=0xD0, read=0xD1)
//   - Data sampled on SCL high phase (scl_hig_pulse) for stability
//=============================================================================

//-----------------------------------------------------------------------------
// Port Descriptions
//-----------------------------------------------------------------------------
// clk         : System clock, 100MHz
// rst_n       : Async reset, active low
// read_en     : 1 = continuous reading, 0 = stop (resets to init)
// scl         : I2C serial clock output (200kHz)
// sda         : I2C serial data (bidirectional, open-drain w/ external pullup)
//
// acc_x_h/l   : Accelerometer X-axis, high/low byte (see data format below)
// acc_y_h/l   : Accelerometer Y-axis
// acc_z_h/l   : Accelerometer Z-axis
// gyro_x_h/l  : Gyroscope X-axis
// gyro_y_h/l  : Gyroscope Y-axis
// gyro_z_h/l  : Gyroscope Z-axis
// init_done   : High when initialization sequence complete
// data_valid  : Single-cycle pulse when all 12 data bytes are refreshed
//
// [debug] state       : Internal FSM state (see state encoding below)
// [debug] sda_link/r  : SDA output enable / output value
// [debug] scl_*_pulse : SCL phase indicator pulses (pos/hig/neg/low)

//-----------------------------------------------------------------------------
// Initialization Sequence (executed once after power-up)
//-----------------------------------------------------------------------------
//  1. 100ms delay (MPU6050 power-up stabilization)
//  2. PWR_MGMT_1  (0x6B) = 0x00  →  Wake up from sleep, clock = internal 8MHz
//  3. SMPLRT_DIV  (0x19) = 0x07  →  Sample rate = 8kHz / (1+7) = 1kHz
//  4. CONFIG      (0x1A) = 0x06  →  DLPF = 6 (5Hz BW for accel, 1kHz gyro)
//  5. GYRO_CONFIG (0x1B) = 0x18  →  Gyro FS = ±2000dps
//  6. ACC_CONFIG  (0x1C) = 0x09  →  Accel FS = ±4g
//  After init_done asserts, the FSM cycles through all 12 data registers.

//-----------------------------------------------------------------------------
// Data Format & Conversion
//-----------------------------------------------------------------------------
//  Each axis outputs a 16-bit two's-complement value: {_h[7:0], _l[7:0]}
//  Combine into signed integer before converting to physical units.
//
//  Accelerometer (@ ±4g FS, AFS_SEL=1, sensitivity = 8192 LSB/g):
//    int16_t raw = ((int16_t)acc_x_h << 8) | acc_x_l;
//    float   g   = (float)raw / 8192.0f;
//
//    Example: raw = 0x2000 (= 8192)  →  8192 / 8192 = +1.000 g
//             raw = 0xE000 (=-8192)  → -8192 / 8192 = -1.000 g
//
//  Gyroscope (@ ±2000dps FS, FS_SEL=3, sensitivity = 16.4 LSB/dps):
//    int16_t raw = ((int16_t)gyro_x_h << 8) | gyro_x_l;
//    float   dps = (float)raw / 16.4f;
//
//  Change ACC_CONFIG_VAL or GYRO_CONFIG_VAL to adjust range.
//  See MPU6050 datasheet for full register map and sensitivity tables.

//-----------------------------------------------------------------------------
// Timing & Sampling
//-----------------------------------------------------------------------------
//  System Clock    : 100 MHz
//  I2C SCL         : 200 kHz (500 cycles/bit, 5us period)
//  Per-transaction : ~50us (START + 2x addr + 2x data + ACKs + STOP)
//  Full scan cycle : ~9.4ms (5 init-write + 12 data-read transactions)
//                     Init writes only on first cycle; subsequent cycles
//                     also include init writes for register refresh.
//  Data update rate: ~100 Hz (each register refreshed every ~9.4ms)
//                     data_valid pulses once per complete scan cycle.

//-----------------------------------------------------------------------------
// Register Map (MPU6050)
//-----------------------------------------------------------------------------
//  Accel: 0x3B(ACCEL_XOUT_H) ~ 0x40(ACCEL_ZOUT_L)
//  Gyro : 0x43(GYRO_XOUT_H)  ~ 0x48(GYRO_ZOUT_L)
//  Init : 0x6B(PWR_MGMT_1), 0x19(SMPLRT_DIV), 0x1A(CONFIG),
//         0x1B(GYRO_CONFIG), 0x1C(ACCEL_CONFIG)

//-----------------------------------------------------------------------------
// State Encoding
//-----------------------------------------------------------------------------
//  IDLE(0) START1(1) ADD1(2) ACK1(3) ADD2(4) ACK2(5) START2(6)
//  ADD3(7) ACK3(8) DATA(9) ACK4(10) STOP1(11) STOP2(12)
//  ADD_EXT(13) ACK_EXT(14) INIT_WAIT(15)
//=============================================================================

module iic_mpu6050 (
    input clk,
    input rst_n,
    input read_en,           // High: continuous reading, Low: stop
    output scl,
    inout sda,
    output [7:0] acc_x_h,   // Accelerometer X high byte  [15:8]
    output [7:0] acc_x_l,   // Accelerometer X low byte   [7:0]
    output [7:0] acc_y_h,   // Accelerometer Y high byte
    output [7:0] acc_y_l,   // Accelerometer Y low byte
    output [7:0] acc_z_h,   // Accelerometer Z high byte
    output [7:0] acc_z_l,   // Accelerometer Z low byte
    output [7:0] gyro_x_h,  // Gyroscope X high byte
    output [7:0] gyro_x_l,  // Gyroscope X low byte
    output [7:0] gyro_y_h,  // Gyroscope Y high byte
    output [7:0] gyro_y_l,  // Gyroscope Y low byte
    output [7:0] gyro_z_h,  // Gyroscope Z high byte
    output [7:0] gyro_z_l,  // Gyroscope Z low byte
    output init_done,       // Init sequence complete (stays high)
    output data_valid,      // One-cycle pulse: all 12 bytes refreshed

// debug outputs
    output sda_link,
    output sda_r,
    output [3:0] state,
    output scl_hig_pulse,
    output scl_neg_pulse,
    output scl_pos_pulse,
    output scl_low_pulse,
    output start1_done
);

reg [2:0] cnt;
reg [8:0] cnt_sum;

reg scl_r;

reg [19:0] cnt_10ms;

always @(posedge clk or negedge rst_n)
if (!rst_n)
    cnt_10ms <= 20'd0;
else
    cnt_10ms <= cnt_10ms + 1'b1;

always @(posedge clk or negedge rst_n)
begin
    if (!rst_n)
        cnt_sum <= 0;
    else if (cnt_sum == 9'd499)  // 5us period @ 100MHz = 500 cycles → 200kHz SCL
        cnt_sum <= 0;
    else
        cnt_sum <= cnt_sum + 1'b1;
end

always @(posedge clk or negedge rst_n)
begin
    if (!rst_n)
        cnt <= 3'd0;
    else if (cnt_sum < 9'd124)  // 0-123: SCL low
        cnt <= 3'd3;
    else if (cnt_sum < 9'd248)  // 124-247: SCL pos
        cnt <= 3'd0;
    else if (cnt_sum < 9'd374)  // 248-373: SCL high
        cnt <= 3'd1;
    else if (cnt_sum < 9'd499)  // 374-498: SCL neg
        cnt <= 3'd2;
    else  // 499
        cnt <= 3'd2;
end

// Single-cycle pulse signals (ONE clock cycle pulse at transitions)
// Note: These are also exported as debug outputs
wire scl_pos_pulse;
wire scl_hig_pulse;
wire scl_neg_pulse;
wire scl_low_pulse;

`define SCL_POS (cnt==3'd0)  // SCL positive edge
`define SCL_HIG (cnt==3'd1)  // SCL high level
`define SCL_NEG (cnt==3'd2)  // SCL negative edge
`define SCL_LOW (cnt==3'd3)  // SCL low level

always @(posedge clk or negedge rst_n)
begin
    if (!rst_n)
        scl_r <= 1'b0;
    else if ((state != IDLE && state != INIT_WAIT && state != START1 && state != STOP2) && cnt == 3'd0)
        scl_r <= 1'b1;
    else if ((state != IDLE && state != INIT_WAIT && state != START1 && state != STOP2) && cnt == 3'd2)
        scl_r <= 1'b0;
    else if (state == IDLE || state == INIT_WAIT || state == START1 || state == STOP2)
        scl_r <= 1'b1;  // Hold SCL high: idle / start / post-stop
    // STOP1: SCL toggles normally for proper STOP sequence timing
end

assign scl = scl_r;

// I2C command definitions
`define DEVICE_READ  8'hD1
`define DEVICE_WRITE 8'hD0
`define ACC_XH 8'h3B   // ACCEL_XOUT_H register
`define ACC_XL 8'h3C
`define ACC_YH 8'h3D
`define ACC_YL 8'h3E
`define ACC_ZH 8'h3F
`define ACC_ZL 8'h40
`define GYRO_XH 8'h43
`define GYRO_XL 8'h44
`define GYRO_YH 8'h45
`define GYRO_YL 8'h46
`define GYRO_ZH 8'h47
`define GYRO_ZL 8'h48

// Initialization registers
`define PWR_MGMT_1 8'h6B
`define SMPLRT_DIV 8'h19
`define CONFIG1 8'h1A
`define GYRO_CONFIG 8'h1B
`define ACC_CONFIG 8'h1C

// Initialization values
`define PWR_MGMT_1_VAL 8'h00
`define SMPLRT_DIV_VAL 8'h07
`define CONFIG1_VAL 8'h06
`define GYRO_CONFIG_VAL 8'h18
`define ACC_CONFIG_VAL 8'h09   // AFS_SEL=1 → ±4g

parameter IDLE      = 4'd0;
parameter START1    = 4'd1;
parameter ADD1      = 4'd2;
parameter ACK1      = 4'd3;
parameter ADD2      = 4'd4;
parameter ACK2      = 4'd5;
parameter START2    = 4'd6;
parameter ADD3      = 4'd7;
parameter ACK3      = 4'd8;
parameter DATA      = 4'd9;
parameter ACK4      = 4'd10;
parameter STOP1     = 4'd11;
parameter STOP2     = 4'd12;
parameter ADD_EXT   = 4'd13;
parameter ACK_EXT   = 4'd14;
parameter INIT_WAIT = 4'd15;  // Power-up delay state

reg [3:0] state;
reg sda_r;
reg sda_link;
reg [3:0] num;
reg [7:0] db_r;

reg [7:0] ACC_XH_READ;
reg [7:0] ACC_XL_READ;
reg [7:0] ACC_YH_READ;
reg [7:0] ACC_YL_READ;
reg [7:0] ACC_ZH_READ;
reg [7:0] ACC_ZL_READ;
reg [7:0] GYRO_XH_READ;
reg [7:0] GYRO_XL_READ;
reg [7:0] GYRO_YH_READ;
reg [7:0] GYRO_YL_READ;
reg [7:0] GYRO_ZH_READ;
reg [7:0] GYRO_ZL_READ;
reg [4:0] times;

reg init_done_r;
reg data_valid_r;
reg [23:0] init_delay_cnt;  // Power-up delay counter (100ms @ 100MHz = 10,000,000 cycles)
    // Note: 24 bits needed for 10,000,000 (range 0-16,777,215)

// Flags for level-based state machine to prevent missing single-cycle pulses
reg start1_done;
reg start2_done;
reg stop1_low_done;
reg ack1_done;
reg ack2_done;
reg ack3_done;
reg ack4_done;
reg ack_ext_done;
reg start2_phase;   // 0: wait SCL low to release; 1: wait SCL high for re-START

reg [15:0] stop2_cnt;

assign init_done = init_done_r;
assign data_valid = data_valid_r;

assign acc_x_h = ACC_XH_READ;
assign acc_x_l = ACC_XL_READ;
assign acc_y_h = ACC_YH_READ;
assign acc_y_l = ACC_YL_READ;
assign acc_z_h = ACC_ZH_READ;
assign acc_z_l = ACC_ZL_READ;
assign gyro_x_h = GYRO_XH_READ;
assign gyro_x_l = GYRO_XL_READ;
assign gyro_y_h = GYRO_YH_READ;
assign gyro_y_l = GYRO_YL_READ;
assign gyro_z_h = GYRO_ZH_READ;
assign gyro_z_l = GYRO_ZL_READ;

always @(posedge clk or negedge rst_n)
begin
    if (!rst_n)
    begin
        sda_r <= 1'b1;
        sda_link <= 1'b0;
        num <= 4'd0;
        ACC_XH_READ <= 8'h00;
        ACC_XL_READ <= 8'h00;
        ACC_YH_READ <= 8'h00;
        ACC_YL_READ <= 8'h00;
        ACC_ZH_READ <= 8'h00;
        ACC_ZL_READ <= 8'h00;
        GYRO_XH_READ <= 8'h00;
        GYRO_XL_READ <= 8'h00;
        GYRO_YH_READ <= 8'h00;
        GYRO_YL_READ <= 8'h00;
        GYRO_ZH_READ <= 8'h00;
        GYRO_ZL_READ <= 8'h00;
        times <= 5'd1;  // Start at 1 (PWR_MGMT_1 init)
        init_done_r <= 1'b0;
        data_valid_r <= 1'b0;
        init_delay_cnt <= 23'd0;
        start1_done <= 1'b0;
        start2_done <= 1'b0;
        stop1_low_done <= 1'b0;
        ack1_done <= 1'b0;
        ack2_done <= 1'b0;
        ack3_done <= 1'b0;
        ack4_done <= 1'b0;
        ack_ext_done <= 1'b0;
        start2_phase <= 1'b0;
        stop2_cnt <= 16'd0;
        state <= INIT_WAIT;  // Start with power-up delay
    end
    else
        case (state)
        INIT_WAIT: begin
            // Wait 100ms for MPU6050 power-up (100ms = 10,000,000 cycles @ 100MHz)
            if (init_delay_cnt >= 24'd10000000)  // 100ms delay
            begin
                init_delay_cnt <= 23'd0;
                state <= IDLE;
            end
            else
            begin
                init_delay_cnt <= init_delay_cnt + 1'b1;
                state <= INIT_WAIT;
            end
        end

        IDLE: begin
            start1_done <= 1'b0;
            start2_done <= 1'b0;
            start2_phase <= 1'b0;
            stop1_low_done <= 1'b0;
            ack1_done <= 1'b0;
            ack2_done <= 1'b0;
            ack3_done <= 1'b0;
            ack4_done <= 1'b0;
            ack_ext_done <= 1'b0;
            if (read_en) begin
                // times incremented on successful txn completion (ACK_EXT/ACK4)
                sda_link <= 1'b1;
                sda_r <= 1'b1;
                db_r <= `DEVICE_WRITE;
                state <= START1;
            end else begin
                times <= 5'd1;
                sda_link <= 1'b0;
                state <= IDLE;
            end
        end

        START1: begin
            if (!start1_done && `SCL_HIG) begin
                sda_link <= 1'b1;
                sda_r   <= 1'b0;     // SDA falls while SCL HIGH → valid START
                state   <= ADD1;
                num     <= 4'd0;
                start1_done <= 1'b1;
            end else if (!`SCL_HIG) begin
                start1_done <= 1'b0;
                sda_link <= 1'b1;
                sda_r   <= 1'b1;
            end else begin
                sda_link <= 1'b1;
                sda_r   <= 1'b1;
            end
        end

        ADD1: begin
            if (scl_low_pulse)
            begin
                if (num == 4'd8)     // all 8 bits sent, release SDA for ACK
                begin
                    num <= 4'd0;
                    sda_r <= 1'b1;
                    sda_link <= 1'b0;
                    state <= ACK1;
                end
                else
                begin
                    num <= num + 1'b1;
                    if (num == 4'd7)
                        sda_r <= db_r[0];     // bit 0 (last data bit)
                    else
                        sda_r <= db_r[4'd7 - num];
                    state <= ADD1;
                end
            end
            else
                state <= ADD1;
        end

        ACK1: begin
            if (scl_neg_pulse) begin
                if (sda == 1'b0) begin  // ACK received
                    state <= ADD2;
                    case (times)
                        5'd1: db_r <= `PWR_MGMT_1;
                        5'd2: db_r <= `SMPLRT_DIV;
                        5'd3: db_r <= `CONFIG1;
                        5'd4: db_r <= `GYRO_CONFIG;
                        5'd5: db_r <= `ACC_CONFIG;
                        5'd6: db_r <= `ACC_XH;
                        5'd7: db_r <= `ACC_XL;
                        5'd8: db_r <= `ACC_YH;
                        5'd9: db_r <= `ACC_YL;
                        5'd10: db_r <= `ACC_ZH;
                        5'd11: db_r <= `ACC_ZL;
                        5'd12: db_r <= `GYRO_XH;
                        5'd13: db_r <= `GYRO_XL;
                        5'd14: db_r <= `GYRO_YH;
                        5'd15: db_r <= `GYRO_YL;
                        5'd16: db_r <= `GYRO_ZH;
                        5'd17: db_r <= `GYRO_ZL;
                        default: db_r <= `PWR_MGMT_1;
                    endcase
                end else
                    state <= STOP1;  // NACK
            end else
                state <= ACK1;
        end

        ADD2: begin
            if (scl_low_pulse)
            begin
                if (num == 4'd8)
                begin
                    num <= 4'd0;
                    sda_r <= 1'b1;
                    sda_link <= 1'b0;
                    state <= ACK2;
                end
                else
                begin
                    sda_link <= 1'b1;
                    num <= num + 1'b1;
                    if (num == 4'd7)
                        sda_r <= db_r[0];
                    else
                        sda_r <= db_r[4'd7 - num];
                    state <= ADD2;
                end
            end
            else
                state <= ADD2;
        end

        ACK2: begin
            if (scl_neg_pulse) begin
                if (sda == 1'b0) begin  // ACK received
                    case (times)
                        5'd1: db_r <= `PWR_MGMT_1_VAL;
                        5'd2: db_r <= `SMPLRT_DIV_VAL;
                        5'd3: db_r <= `CONFIG1_VAL;
                        5'd4: db_r <= `GYRO_CONFIG_VAL;
                        5'd5: db_r <= `ACC_CONFIG_VAL;
                        5'd6: db_r <= `DEVICE_READ;
                        default: db_r <= `DEVICE_READ;
                    endcase
                    if (times >= 5'd6)
                        state <= START2;
                    else
                        state <= ADD_EXT;
                end else
                    state <= STOP1;  // NACK
            end else
                state <= ACK2;
        end

        ADD_EXT: begin
            if (scl_low_pulse)
            begin
                if (num == 4'd8)
                begin
                    num <= 4'd0;
                    sda_r <= 1'b1;
                    sda_link <= 1'b0;
                    state <= ACK_EXT;
                end
                else
                begin
                    sda_link <= 1'b1;
                    num <= num + 1'b1;
                    if (num == 4'd7)
                        sda_r <= db_r[0];
                    else
                        sda_r <= db_r[4'd7 - num];
                    state <= ADD_EXT;
                end
            end
            else
                state <= ADD_EXT;
        end

        ACK_EXT: begin
            if (scl_neg_pulse) begin
                sda_r <= 1'b1;
                times <= times + 1'b1;  // Init write completed successfully
                state <= STOP1;
            end
        end

        START2: begin
            if (!start2_phase) begin
                // Phase 0: release SDA, let SCL go low so slave releases ACK
                sda_link <= 1'b0;
                if (`SCL_LOW)
                    start2_phase <= 1'b1;
            end else begin
                // Phase 1: SDA high->low while SCL high = repeated START
                if (!start2_done && `SCL_HIG) begin
                    sda_link <= 1'b1;
                    sda_r   <= 1'b0;
                    state   <= ADD3;
                    num     <= 4'd0;
                    start2_done <= 1'b1;
                    start2_phase <= 1'b0;
                end else if (!`SCL_HIG) begin
                    start2_done <= 1'b0;
                    sda_link <= 1'b1;
                    sda_r   <= 1'b1;
                end else begin
                    sda_link <= 1'b1;
                    sda_r   <= 1'b1;
                end
            end
        end

        ADD3: begin
            if (scl_low_pulse)
            begin
                if (num == 4'd8)
                begin
                    num <= 4'd0;
                    sda_r <= 1'b1;
                    sda_link <= 1'b0;
                    state <= ACK3;
                end
                else
                begin
                    sda_link <= 1'b1;
                    num <= num + 1'b1;
                    if (num == 4'd7)
                        sda_r <= db_r[0];
                    else
                        sda_r <= db_r[4'd7 - num];
                    state <= ADD3;
                end
            end
            else
                state <= ADD3;
        end

        ACK3: begin
            if (scl_neg_pulse) begin
                if (sda == 1'b0) begin
                    state <= DATA;
                    sda_link <= 1'b0;
                end else
                    state <= STOP1;
            end else
                state <= ACK3;
        end

        DATA: begin
            if (num <= 4'd7)
            begin
                if (scl_hig_pulse)  // Sample during SCL high (stable data)
                begin
                    case (times)
                        5'd6: ACC_XH_READ[4'd7-num] <= sda;
                        5'd7: ACC_XL_READ[4'd7-num] <= sda;
                        5'd8: ACC_YH_READ[4'd7-num] <= sda;
                        5'd9: ACC_YL_READ[4'd7-num] <= sda;
                        5'd10: ACC_ZH_READ[4'd7-num] <= sda;
                        5'd11: ACC_ZL_READ[4'd7-num] <= sda;
                        5'd12: GYRO_XH_READ[4'd7-num] <= sda;
                        5'd13: GYRO_XL_READ[4'd7-num] <= sda;
                        5'd14: GYRO_YH_READ[4'd7-num] <= sda;
                        5'd15: GYRO_YL_READ[4'd7-num] <= sda;
                        5'd16: GYRO_ZH_READ[4'd7-num] <= sda;
                        5'd17: GYRO_ZL_READ[4'd7-num] <= sda;
                        default: ;
                    endcase
                    num <= num + 1'b1;
                end
                state <= DATA;
            end
            else if (num == 4'd8 && scl_low_pulse)
            begin
                sda_link <= 1'b1;
                num <= 4'd0;
                state <= ACK4;
            end
            else
                state <= DATA;
        end

        ACK4: begin
            if (scl_neg_pulse) begin
                if (times == 5'd17) begin
                    sda_r <= 1'b1;
                    init_done_r <= 1'b1;
                    data_valid_r <= 1'b1;
                end else begin
                    sda_r <= 1'b1;
                end
                times <= times + 1'b1;  // Data read completed successfully
                state <= STOP1;
            end else
                state <= ACK4;
        end

        STOP1: begin
            if (!stop1_low_done && scl_low_pulse) begin
                sda_link <= 1'b1;
                sda_r   <= 1'b0;       // Drive SDA low while SCL LOW (NOT a START!)
                stop1_low_done <= 1'b1;
            end else if (stop1_low_done && scl_hig_pulse) begin
                sda_r   <= 1'b1;       // Raise SDA while SCL HIGH → valid STOP
                state   <= STOP2;
                stop1_low_done <= 1'b0;
            end else begin
                sda_link <= 1'b1;
            end
        end

        STOP2: begin
            if (stop2_cnt < 16'd50000) begin   // 500us @ 100MHz
                stop2_cnt <= stop2_cnt + 1'b1;
                sda_link <= 1'b0;              // Release SDA
            end else begin
                stop2_cnt <= 16'd0;
                if (read_en) begin
                    if (times >= 5'd18)
                        times <= 5'd1;
                end
                state <= IDLE;
                data_valid_r <= 1'b0;
            end
        end

        default: state <= IDLE;
        endcase
end

assign sda = sda_link ? sda_r : 1'bz;

// Debug outputs
assign scl_hig_pulse = (cnt == 3'd0) && (cnt_sum == 9'd248); // cnt about to become 1
assign scl_neg_pulse = (cnt == 3'd1) && (cnt_sum == 9'd374); // cnt about to become 2
assign scl_pos_pulse = (cnt == 3'd3) && (cnt_sum == 9'd124); // cnt about to become 0
assign scl_low_pulse = (cnt == 3'd2) && (cnt_sum == 9'd0);   // cnt about to become 3

endmodule