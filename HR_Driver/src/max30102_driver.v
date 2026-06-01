//=============================================================================
// Module     : max30102_driver
// Description: I2C Driver for MAX30102 Pulse Oximeter & Heart-Rate Sensor
//
// Phase-based state machine:
//   INIT        : Write 5 config registers
//   FLUSH_PTR   : Read FIFO_WR_PTR, compute bytes to drain
//   FLUSH_RD    : Read & discard FIFO_DATA until FIFO empty
//   FLUSH_WR    : Write FIFO_RD_PTR → init_done=1 → enter POLL
//   POLL        : Read FIFO_WR_PTR, compute avail; if 0→retry
//   READ_DATA   : Read 6 bytes FIFO_DATA → fill IR/RED
//   WRITE_PTR   : Write FIFO_RD_PTR → back to POLL
//
// I2C sub-states (unchanged from MPU6050 pattern):
//   START1→ADD1→ACK1→ADD2→ACK2→[ADD_EXT→ACK_EXT | START2→ADD3→ACK3→DATA→ACK4]
//   →STOP1→STOP2→IDLE
//=============================================================================

module max30102_driver (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,

    output wire       scl,
    inout  wire       sda,

    output wire [17:0] ir_data,
    output wire [17:0] red_data,
    output wire        init_done,
    output wire        data_valid,

    output wire        sda_link,
    output wire        sda_r,
    output wire [3:0]  state
);

//=============================================================================
// I2C Commands
//=============================================================================
`define DEVICE_READ   8'hAF
`define DEVICE_WRITE  8'hAE

`define FIFO_WR_PTR   8'h04
`define FIFO_RD_PTR   8'h06
`define FIFO_DATA     8'h07
`define FIFO_CONFIG   8'h08
`define MODE_CONFIG   8'h09
`define SPO2_CONFIG   8'h0A
`define LED1_PA       8'h0C
`define LED2_PA       8'h0D

`define FIFO_CONFIG_VAL  8'h1F     // No avg, FIFO_A_FULL=31
`define MODE_CONFIG_VAL  8'h03     // SpO2 mode
`define MODE_RESET_VAL   8'h40     // Reset all registers
`define INT_STATUS      8'h00
`define PART_ID          8'hFF
`define SPO2_CONFIG_VAL  8'h27
`define LED1_PA_VAL      8'h69     // RED LED ~25mA
`define LED2_PA_VAL      8'h69     // IR LED ~25mA

//=============================================================================
// Phase states (top-level scheduler)
//=============================================================================
parameter PH_INIT       = 3'd0;
parameter PH_FLUSH_PTR  = 3'd1;
parameter PH_FLUSH_RD   = 3'd2;
parameter PH_FLUSH_WR   = 3'd3;
parameter PH_POLL       = 3'd4;
parameter PH_READ       = 3'd5;
parameter PH_WRITE      = 3'd6;

//=============================================================================
// I2C sub-states (bit-level protocol engine)
//=============================================================================
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

//=============================================================================
// SCL Timing (200kHz = 5us period @ 100MHz = 500 cycles)
//=============================================================================
reg [2:0] cnt;
reg [8:0] cnt_sum;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)       cnt_sum <= 0;
    else if (cnt_sum == 9'd499) cnt_sum <= 0;
    else              cnt_sum <= cnt_sum + 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)       cnt <= 3'd0;
    else if (cnt_sum < 9'd124)  cnt <= 3'd3;   // SCL low
    else if (cnt_sum < 9'd248)  cnt <= 3'd0;   // SCL pos
    else if (cnt_sum < 9'd374)  cnt <= 3'd1;   // SCL high
    else if (cnt_sum < 9'd499)  cnt <= 3'd2;   // SCL neg
    else                        cnt <= 3'd2;
end

`define SCL_HIG (cnt==3'd1)
`define SCL_LOW (cnt==3'd3)

reg scl_r;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        scl_r <= 1'b1;
    else if ((state_r != IDLE && state_r != START1 && state_r != STOP2) && cnt == 3'd0)
        scl_r <= 1'b1;
    else if ((state_r != IDLE && state_r != START1 && state_r != STOP2) && cnt == 3'd2)
        scl_r <= 1'b0;
    else if (state_r == IDLE || state_r == START1 || state_r == STOP2)
        scl_r <= 1'b1;
end

assign scl = scl_r;

// Pulse signals
wire scl_hig_pulse = (cnt == 3'd0) && (cnt_sum == 9'd248);
wire scl_neg_pulse = (cnt == 3'd1) && (cnt_sum == 9'd374);
wire scl_low_pulse = (cnt == 3'd2) && (cnt_sum == 9'd0);

//=============================================================================
// Hardware registers
//=============================================================================
reg [3:0] state_r;
reg sda_drive, sda_oe;
reg [3:0] num;
reg [7:0] db_r;

// Phase & sub-step
reg [2:0] phase;
reg [3:0] sub_step;

// FIFO data bytes
reg [7:0] byte0, byte1, byte2, byte3, byte4, byte5;
reg [7:0] wr_ptr, rd_ptr;     // local copies of FIFO pointers
reg [7:0] avail;               // available sample count
reg [7:0] flush_target;        // total bytes to flush = wr_ptr*6
reg [2:0] rd_byte_cnt;         // 0-5: byte index in current read burst

reg init_done_r, data_valid_r;

// I2C flags
reg start1_done, start2_done, stop1_low_done, start2_phase;
reg [15:0] stop2_cnt;

assign init_done  = init_done_r;
assign data_valid = data_valid_r;
assign sda_link   = sda_oe;
assign sda_r      = sda_drive;
assign state      = state_r;

// Data extraction: MAX30102 FIFO {byte0=ADC[17:10], byte1=ADC[9:2], byte2={ADC[1:0],6'b0}}
assign ir_data  = {byte0,     byte1,     byte2[7:6]};   // bytes 0-2 (working channel)
assign red_data = {byte3,     byte4,     byte5[7:6]};   // bytes 3-5

// Combinational: available samples = (wr_ptr - rd_ptr) mod 32
wire [7:0] avail_tmp = (wr_ptr >= rd_ptr) ? (wr_ptr - rd_ptr) : (8'd32 - rd_ptr + wr_ptr);

//=============================================================================
// Main State Machine
//=============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state_r   <= IDLE;
        sda_drive <= 1'b1;  sda_oe <= 1'b0;
        num       <= 4'd0;
        phase     <= PH_INIT;
        sub_step  <= 4'd0;

        byte0<=0; byte1<=0; byte2<=0; byte3<=0; byte4<=0; byte5<=0;
        wr_ptr<=0; rd_ptr<=0; avail<=0; flush_target<=0; rd_byte_cnt<=0;

        init_done_r <= 0;  data_valid_r <= 0;
        start1_done<=0; start2_done<=0; stop1_low_done<=0; start2_phase<=0;
        stop2_cnt  <= 0;
    end else begin
        case (state_r)

        //=====================================================================
        // IDLE - decide next I2C transaction based on phase
        //=====================================================================
        IDLE: begin
            start1_done<=0; start2_done<=0; start2_phase<=0; stop1_low_done<=0;
            if (start) begin
                sda_oe<=1; sda_drive<=1; db_r<=`DEVICE_WRITE; state_r<=START1;
            end else begin
                sda_oe<=0; state_r<=IDLE;
            end
        end

        //=====================================================================
        // START1 - generate START condition
        //=====================================================================
        START1: begin
            if (!start1_done && `SCL_HIG) begin
                sda_oe<=1; sda_drive<=0; state_r<=ADD1; num<=0; start1_done<=1;
            end else if (!`SCL_HIG) begin
                start1_done<=0; sda_oe<=1; sda_drive<=1;
            end else begin sda_oe<=1; sda_drive<=1; end
        end

        //=====================================================================
        // ADD1 - send DEVICE_WRITE address
        //=====================================================================
        ADD1: begin
            if (scl_low_pulse) begin
                if (num == 4'd8) begin
                    num<=0; sda_drive<=1; sda_oe<=0; state_r<=ACK1;
                end else begin
                    num<=num+1;
                    sda_drive <= (num==4'd7) ? db_r[0] : db_r[4'd7-num];
                end
            end
        end

        //=====================================================================
        // ACK1 - check slave ACK, prepare register address
        //=====================================================================
        ACK1: begin
            if (scl_neg_pulse) begin
                if (sda == 1'b0) begin
                    state_r <= ADD2;
                    // Choose register address based on phase
                    case (phase)
                        PH_INIT: begin
                            // Reset first, config, then MODE last to start sensing
                            case (sub_step)
                                4'd0: db_r <= `MODE_CONFIG;
                                4'd1: db_r <= `FIFO_CONFIG;
                                4'd2: db_r <= `SPO2_CONFIG;
                                4'd3: db_r <= `LED1_PA;
                                4'd4: db_r <= `LED2_PA;
                                4'd5: db_r <= `MODE_CONFIG;
                                default: db_r <= `FIFO_CONFIG;
                            endcase
                        end
                        PH_FLUSH_PTR:  db_r <= `FIFO_WR_PTR;
                        PH_FLUSH_RD:   db_r <= `FIFO_DATA;
                        PH_FLUSH_WR:   db_r <= `FIFO_RD_PTR;
                        PH_POLL:       db_r <= `FIFO_WR_PTR;
                        PH_READ:       db_r <= `FIFO_DATA;
                        PH_WRITE:      db_r <= `FIFO_RD_PTR;
                        default:       db_r <= `FIFO_DATA;
                    endcase
                end else
                    state_r <= STOP1;
            end
        end

        //=====================================================================
        // ADD2 - send register address
        //=====================================================================
        ADD2: begin
            if (scl_low_pulse) begin
                if (num == 4'd8) begin
                    num<=0; sda_drive<=1; sda_oe<=0; state_r<=ACK2;
                end else begin
                    sda_oe<=1; num<=num+1;
                    sda_drive <= (num==4'd7) ? db_r[0] : db_r[4'd7-num];
                end
            end
        end

        //=====================================================================
        // ACK2 - branch: write (ADD_EXT) or read (START2)
        //=====================================================================
        ACK2: begin
            if (scl_neg_pulse) begin
                if (sda == 1'b0) begin
                    // Set data-to-write or read-address based on phase
                    case (phase)
                        PH_INIT: begin
                            case (sub_step)
                                4'd0: db_r <= `MODE_RESET_VAL;
                                4'd1: db_r <= `FIFO_CONFIG_VAL;
                                4'd2: db_r <= `SPO2_CONFIG_VAL;
                                4'd3: db_r <= `LED1_PA_VAL;
                                4'd4: db_r <= `LED2_PA_VAL;
                                4'd5: db_r <= `MODE_CONFIG_VAL;
                                default: db_r <= `FIFO_CONFIG_VAL;
                            endcase
                        end
                        PH_FLUSH_WR: db_r <= rd_ptr;
                        PH_WRITE:    db_r <= rd_ptr;
                        default:     db_r <= `DEVICE_READ;  // all read phases
                    endcase

                    // Branch: write vs read
                    if (phase == PH_INIT || phase == PH_FLUSH_WR || phase == PH_WRITE)
                        state_r <= ADD_EXT;    // write data
                    else
                        state_r <= START2;      // read data
                end else
                    state_r <= STOP1;
            end
        end

        //=====================================================================
        // ADD_EXT - send extra data byte (for writes)
        //=====================================================================
        ADD_EXT: begin
            if (scl_low_pulse) begin
                if (num == 4'd8) begin
                    num<=0; sda_drive<=1; sda_oe<=0; state_r<=ACK_EXT;
                end else begin
                    sda_oe<=1; num<=num+1;
                    sda_drive <= (num==4'd7) ? db_r[0] : db_r[4'd7-num];
                end
            end
        end

        //=====================================================================
        // ACK_EXT - after write byte → advance sub_step → STOP
        //=====================================================================
        ACK_EXT: begin
            if (scl_neg_pulse) begin
                sda_drive <= 1'b1;
                sub_step  <= sub_step + 1'b1;
                state_r   <= STOP1;
            end
        end

        //=====================================================================
        // START2 - repeated START → prepare for DEVICE_READ
        //=====================================================================
        START2: begin
            if (!start2_phase) begin
                sda_oe<=0;
                if (`SCL_LOW) start2_phase<=1;
            end else begin
                if (!start2_done && `SCL_HIG) begin
                    sda_oe<=1; sda_drive<=0; state_r<=ADD3; num<=0;
                    start2_done<=1; start2_phase<=0;
                end else if (!`SCL_HIG) begin
                    start2_done<=0; sda_oe<=1; sda_drive<=1;
                end else begin sda_oe<=1; sda_drive<=1; end
            end
        end

        //=====================================================================
        // ADD3 - send DEVICE_READ address
        //=====================================================================
        ADD3: begin
            if (scl_low_pulse) begin
                if (num == 4'd8) begin
                    num<=0; sda_drive<=1; sda_oe<=0; state_r<=ACK3;
                end else begin
                    sda_oe<=1; num<=num+1;
                    sda_drive <= (num==4'd7) ? db_r[0] : db_r[4'd7-num];
                end
            end
        end

        //=====================================================================
        // ACK3 - wait slave ACK for read address
        //=====================================================================
        ACK3: begin
            if (scl_neg_pulse) begin
                if (sda == 1'b0) begin state_r<=DATA; sda_oe<=0; end
                else              state_r<=STOP1;
            end
        end

        //=====================================================================
        // DATA - read one byte from slave
        //=====================================================================
        DATA: begin
            if (num <= 4'd7) begin
                if (scl_hig_pulse) begin
                    case (phase)
                        PH_FLUSH_PTR: wr_ptr[4'd7-num] <= sda;
                        PH_FLUSH_RD:  ;  // discard
                        PH_POLL:      wr_ptr[4'd7-num] <= sda;
                        PH_READ: begin
                            // rd_byte_cnt: 0→byte0, 1→byte1, ..., 5→byte5
                            case (rd_byte_cnt)
                                3'd0: byte0[4'd7-num] <= sda;
                                3'd1: byte1[4'd7-num] <= sda;
                                3'd2: byte2[4'd7-num] <= sda;
                                3'd3: byte3[4'd7-num] <= sda;
                                3'd4: byte4[4'd7-num] <= sda;
                                3'd5: byte5[4'd7-num] <= sda;
                                default: ;
                            endcase
                        end
                        default: ;
                    endcase
                    num <= num + 1'b1;
                end
            end else if (num == 4'd8 && scl_low_pulse) begin
                sda_oe<=1; num<=0; state_r<=ACK4;
            end
        end

        //=====================================================================
        // ACK4 - master sends ACK/NACK → advance → STOP
        //=====================================================================
        ACK4: begin
            if (scl_neg_pulse) begin
                // Always NACK (single-byte read per transaction)
                sda_drive <= 1'b1;

                case (phase)
                    PH_FLUSH_PTR: begin
                        avail       <= wr_ptr;
                        flush_target <= wr_ptr * 8'd6;
                    end
                    PH_FLUSH_RD: begin
                        rd_byte_cnt <= rd_byte_cnt + 1'b1;
                    end
                    PH_READ: begin
                        rd_byte_cnt <= rd_byte_cnt + 1'b1;
                    end
                    default: ;
                endcase

                if (phase == PH_INIT || phase == PH_FLUSH_WR || phase == PH_WRITE)
                    sub_step <= sub_step + 1'b1;

                state_r <= STOP1;   // always STOP (MPU6050 pattern)
            end
        end

        //=====================================================================
        // STOP1 - generate STOP condition
        //=====================================================================
        STOP1: begin
            if (!stop1_low_done && scl_low_pulse) begin
                sda_oe<=1; sda_drive<=0; stop1_low_done<=1;
            end else if (stop1_low_done && scl_hig_pulse) begin
                sda_drive<=1; state_r<=STOP2; stop1_low_done<=0;
            end else sda_oe<=1;
        end

        //=====================================================================
        // STOP2 - bus free delay, then phase transition
        //=====================================================================
        STOP2: begin
            if (stop2_cnt < 16'd50000) begin   // 500us bus-free
                stop2_cnt <= stop2_cnt + 1'b1;
                sda_oe <= 1'b0;
            end else begin
                stop2_cnt <= 16'd0;
                data_valid_r <= 1'b0;

                // Phase transition logic
                case (phase)

                    PH_INIT: begin
                        if (sub_step >= 4'd6) begin   // 6 steps: reset+4config+start
                            phase    <= PH_FLUSH_PTR;
                            sub_step <= 4'd0;
                        end
                    end

                    PH_FLUSH_PTR: begin
                        rd_byte_cnt <= 0;
                        if (wr_ptr > 8'd0) begin
                            phase    <= PH_FLUSH_RD;
                            sub_step <= 4'd0;
                        end else begin
                            phase    <= PH_FLUSH_WR;  // FIFO empty, skip
                            sub_step <= 4'd0;
                        end
                    end

                    PH_FLUSH_RD: begin
                        // Check if all bytes flushed (rd_byte_cnt incremented in ACK4)
                        if (rd_byte_cnt >= flush_target) begin
                            rd_ptr   <= wr_ptr;
                            phase    <= PH_FLUSH_WR;
                            sub_step <= 4'd0;
                            rd_byte_cnt <= 0;
                        end
                        // else: stay in PH_FLUSH_RD for next byte
                    end

                    PH_FLUSH_WR: begin
                        init_done_r <= 1'b1;    // init complete!
                        phase    <= PH_POLL;
                        sub_step <= 4'd0;
                    end

                    PH_POLL: begin
                        avail <= avail_tmp;
                        rd_byte_cnt <= 0;
                        if (avail_tmp > 8'd0) begin
                            phase    <= PH_READ;
                            sub_step <= 4'd0;
                        end else begin
                            phase    <= PH_POLL;  // retry
                            sub_step <= 4'd0;
                        end
                    end

                    PH_READ: begin
                        // Each transaction reads 1 byte. rd_byte_cnt incremented in ACK4.
                        if (rd_byte_cnt >= 3'd6) begin   // 6 bytes = 1 sample
                            data_valid_r <= 1'b1;
                            rd_byte_cnt <= 0;
                            rd_ptr <= rd_ptr + 1'b1;
                            if (rd_ptr >= 8'd31) rd_ptr <= 8'd0;
                            if (avail_tmp > 8'd1) begin
                                avail <= avail_tmp - 1'b1;
                                phase <= PH_READ;
                            end else begin
                                avail <= 8'd0;
                                phase <= PH_WRITE;
                            end
                        end
                        // else: stay in PH_READ for next byte
                        sub_step <= 4'd0;
                    end

                    PH_WRITE: begin
                        phase    <= PH_POLL;
                        sub_step <= 4'd0;
                    end

                    default: begin
                        phase <= PH_INIT;
                        sub_step <= 4'd0;
                    end
                endcase

                if (!start) begin
                    phase    <= PH_INIT;
                    sub_step <= 4'd0;
                end

                state_r <= IDLE;
            end
        end

        default: state_r <= IDLE;
        endcase
    end
end

assign sda = sda_oe ? sda_drive : 1'bz;

endmodule