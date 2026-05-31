//=============================================================================
// Module     : ds18b20_driver
// Author     :
// Date       :
// Description: 1-Wire Master Driver for DS18B20 Digital Temperature Sensor
//              Performs reset/presence detection, ROM/function commands, and
//              9-byte scratchpad readout with CRC-8 verification.
//
// Features:
//   - Full 1-Wire protocol: reset pulse (480us), presence detection, bit-banged
//     write/read time slots with configurable timing parameters
//   - 12-bit temperature resolution (0.0625 C/LSB, signed 16-bit output)
//   - CRC-8/MAXIM verification over scratchpad bytes 0-7
//   - Continuous sampling mode (start=1 loops S_DONE→S_RESET automatically)
//   - SIM_FAST parameter for accelerated simulation (reduces T_CONVERT to 100us)
//   - Debug output ports: state, send_byte, bit_cnt, crc_error
//
// 1-Wire Protocol:
//   - Reset   : master pulls bus low for 480us, then releases
//   - Presence: DS18B20 pulls bus low for 60-240us after detecting reset end
//   - ROM cmd : 8-bit command sent LSB-first (0xCC = Skip ROM)
//   - Func cmd: 8-bit command sent LSB-first (0x44 = Convert T, 0xBE = Read SP)
//   - Read    : 9 bytes (72 bits) read LSB-first, byte 8 is CRC-8 of bytes 0-7
//   - Convert : 750ms wait for 12-bit temperature conversion (750ms @ 100MHz)
//
//   Write Slot: [10us low] [55us data: low=0, high=1] [5us recovery] = 70us
//   Read  Slot: [10us low] [10us release] [sample@20us] [55us wait] [5us recov]
//   Slot timing conforms to DS18B20 datasheet (min 60us active + min 1us recovery)
//=============================================================================

//-----------------------------------------------------------------------------
// Port Descriptions
//-----------------------------------------------------------------------------
// clk         : System clock, 100MHz (set CLK_FREQ to match)
// rst_n       : Async reset, active low
// start       : 1 = start/continuous measurement, 0 = stop after current cycle
// dq_in       : 1-Wire bus input (connect to bidirectional dq pin)
// dq_out      : 1-Wire bus output value (0 = pull low)
// dq_oe       : 1-Wire bus output enable (1 = output, 0 = input/tri-state)
// temperature : Signed 16-bit temperature, 0.0625 C/LSB (e.g. 0x0191 = 25.0625C)
// data_valid  : Single-cycle pulse when a new temperature reading is ready
// error       : Stays high on CRC/presence error, cleared on next valid reading
//
// [debug] debug_state : Current FSM state (see state encoding below)
// [debug] debug_send  : Command byte being sent during write states
// [debug] debug_bit   : Bit counter (0-7 per byte)
// [debug] debug_err   : CRC error flag (1 = CRC mismatch)

//-----------------------------------------------------------------------------
// 1-Wire Bus Wiring
//-----------------------------------------------------------------------------
//  FPGA dq_out ────┐
//  FPGA dq_oe  ────┤▸ open-drain buffer ──── 1-Wire bus (dq) ──── DS18B20 DQ
//  FPGA dq_in  ────┤                              │
//                  └──────────────────────────────┤
//                                              4.7kΩ pull-up to 3.3V
//  Bidirectional: dq_oe=1 drives bus (dq_out), dq_oe=0 releases (dq_in reads)

//-----------------------------------------------------------------------------
// Measurement Sequence (12-bit, 750ms conversion)
//-----------------------------------------------------------------------------
//  1. Reset (480us low) → Presence detect
//  2. Skip ROM  (0xCC)  → ROM command
//  3. Convert T (0x44)  → Function command, starts temperature conversion
//  4. Wait 750ms        → 12-bit conversion time (bus released)
//  5. Reset (480us low) → Presence detect (2nd)
//  6. Skip ROM  (0xCC)  → ROM command (MUST send before each function cmd!)
//  7. Read SP  (0xBE)   → Function command, prepare scratchpad readout
//  8. Read 9 bytes      → 72 bits LSB-first, byte8 = CRC-8 of bytes0-7
//  9. CRC-8 verify      → CRC mismatch → error=1; CRC match → data_valid=1

//-----------------------------------------------------------------------------
// Temperature Data Format & Conversion
//-----------------------------------------------------------------------------
//  DS18B20 scratchpad byte 0 (LSB) and byte 1 (MSB) form a signed 12-bit value:
//    Byte1[3:0] = {S, 2^6, 2^5, 2^4}   (upper bits + sign)
//    Byte0[7:0] = {2^3, 2^2, 2^1, 2^0, 2^-1, 2^-2, 2^-3, 2^-4}
//
//  12-bit value = {Byte1[3:0], Byte0[7:0]}, sign-extended to 16 bits.
//  Resolution: 0.0625 C/LSB (2^-4).
//
//  For integer Celsius:
//    int16_t raw = (int16_t)temperature;     // signed 16-bit in 0.0625C units
//    float   c   = (float)raw * 0.0625f;     // OR: raw >> 4 for integer part
//
//  Examples:
//    +10.125 C → Byte0=0xA2, Byte1=0x00 → 12-bit=0x0A2 → 16-bit=0x00A2 (+162)
//    -10.125 C → Byte0=0x5E, Byte1=0xFF → 12-bit=0xF5E → 16-bit=0xFF5E (-162)
//    +25.0625C → Byte0=0x91, Byte1=0x01 → output 0x0191 (401 * 0.0625 = 25.0625)
//      0.000 C → Byte0=0x00, Byte1=0x00 → output 0x0000

//-----------------------------------------------------------------------------
// Timing Parameters (@ 100MHz, CLK_FREQ=100_000_000)
//-----------------------------------------------------------------------------
//  System Clock    : 100 MHz (10ns period)
//  US              : CLK_FREQ / 1_000_000 = 100 cycles per microsecond
//
//  Reset low       : 480 us  (T_RESET_LOW, min 480us per datasheet)
//  Presence start  :  30 us  (T_PRESENCE_START, check after this delay)
//  Presence max    : 300 us  (T_PRESENCE_MAX, timeout if no response)
//  Post-presence   :  10 us  (T_POST_PRESENCE, wait after presence ends)
//
//  Write slot      :  75 us  (T_SLOT, min 60us per datasheet)
//  Write-1 low     :  10 us  (T_WRITE_LOW, 1-15us per datasheet)
//  Read slot       :  75 us  (T_SLOT, same as write)
//  Read sample     :  20 us  (T_SAMPLE, after slave takeover within 15us)
//  Slot recovery   :   5 us  (T_RECOVERY, min 1us per datasheet)
//
//  Convert wait    : 750 ms  (T_CONVERT, 12-bit resolution)
//  Per-measurement : ~800 ms  (2x reset/presence/cmds + convert + read 72 bits)

//-----------------------------------------------------------------------------
// CRC-8/MAXIM
//-----------------------------------------------------------------------------
//  Polynomial : x^8 + x^5 + x^4 + 1 = 0x8C (reflected: 0x31)
//  Computed over scratchpad bytes 0-7 (8 bytes), compared with byte 8.
//  Algorithm processes LSB first (matching 1-Wire bit order).

//-----------------------------------------------------------------------------
// State Encoding
//-----------------------------------------------------------------------------
//  IDLE(0) RESET(1) PRESENCE(2) WR_SKIP_ROM(3) WR_CONVERT(4) WAIT_CONV(5)
//  RESET2(6) PRESENCE2(7) WR_SKIP_ROM2(8) WR_READ_SP(9) READ(10)
//  CRC_CHECK(11) DONE(12) ERROR(13)
//=============================================================================

module ds18b20_driver #(
    parameter CLK_FREQ = 100_000_000, // 100MHz clock
    parameter SIM_FAST = 0            // Set to 1 for fast simulation (reduces T_CONVERT)
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,      // Start conversion
    input  wire dq_in,      // Data pin input (external wire with pull-up)

    output reg  [15:0] temperature,  // Temperature: signed 12-bit, 0.0625 C/LSB
    output reg         data_valid,   // Data ready pulse
    output reg         error,        // Error flag (includes CRC error)
    output reg         dq_out,       // Data pin output value
    output reg         dq_oe,        // Data pin output enable (1=output, 0=input)

    output wire [3:0]  debug_state,  // Debug: current state
    output wire [7:0]  debug_send,   // Debug: command byte being sent
    output wire [3:0]  debug_bit,    // Debug: bit counter
    output wire        debug_err     // Debug: crc_error flag
);

    // ============================================
    // Timing Constants
    // ============================================
    localparam US = CLK_FREQ / 1_000_000;  // cycles per microsecond

    // Reset timing
    localparam T_RESET_LOW     = 480 * US;  // 480us min
    localparam T_PRESENCE_START = 30 * US;  // 15-60us before presence
    localparam T_PRESENCE_MAX   = 300 * US; // 240us max presence

    // Write slot: minimum 60us duration (per DS18B20 datasheet)
    // Phase1: master pulls low 1-15us for write-1, or holds through for write-0
    // Phase2: write-1 releases bus (high via pullup), write-0 keeps low
    // Recovery: minimum 1us between slots
    localparam T_SLOT      = 75 * US;   // Active slot time (75us > 60us min)
    localparam T_WRITE_LOW = 10 * US;   // Master low time (10us per datasheet)

    // Read slot: minimum 60us duration (per DS18B20 datasheet)
    // Master pulls low >= 1us, releases, slave drives within 15us
    // Master samples after slave has time to take over
    localparam T_READ_LOW  = 10 * US;   // Master low time (10us > 1us min)
    localparam T_SAMPLE    = 20 * US;   // Sample at 20us (after 15us slave takeover)

    // Recovery: bus must be high >= 1us between slots (per DS18B20 datasheet)
    localparam T_RECOVERY  = 5  * US;   // 5us recovery (> 1us min)

    // Post-presence: wait after presence pulse ends before first command
    localparam T_POST_PRESENCE = 10 * US;  // 10us wait after presence

    // Conversion time for 12-bit
    localparam T_CONVERT = SIM_FAST ? 100 * US : 750_000 * US;  // 750ms (100us for sim)

    // ============================================
    // Commands (LSB first)
    // ============================================
    localparam CMD_SKIP_ROM = 8'hCC;  // 11001100
    localparam CMD_CONVERT  = 8'h44;  // 00100100
    localparam CMD_READ_SP  = 8'hBE;  // 10111110

    // ============================================
    // States
    // ============================================
    localparam [3:0] S_IDLE         = 4'd0;
    localparam [3:0] S_RESET        = 4'd1;
    localparam [3:0] S_PRESENCE     = 4'd2;
    localparam [3:0] S_WR_SKIP_ROM  = 4'd3;  // Send 0xCC
    localparam [3:0] S_WR_CONVERT   = 4'd4;  // Send 0x44
    localparam [3:0] S_WAIT_CONV    = 4'd5;
    localparam [3:0] S_RESET2       = 4'd6;
    localparam [3:0] S_PRESENCE2    = 4'd7;
    localparam [3:0] S_WR_SKIP_ROM2 = 4'd8;  // Send 0xCC (2nd)
    localparam [3:0] S_WR_READ_SP   = 4'd9;  // Send 0xBE
    localparam [3:0] S_READ         = 4'd10;
    localparam [3:0] S_CRC_CHECK    = 4'd11;
    localparam [3:0] S_DONE         = 4'd12;
    localparam [3:0] S_ERROR        = 4'd13;

    reg [3:0] state, next_state;

    // ============================================
    // CRC-8/MAXIM Calculation (LSB first - DS18B20)
    // Polynomial: x^8 + x^5 + x^4 + 1 = 0x8C
    // Standard Dallas/Maxim 1-Wire CRC-8
    // Fixed: use proper 8-bit algorithm with data shift
    // ============================================
    function [7:0] crc8_maxim;
        input [7:0] data;
        input [7:0] crc;
        integer i;
        reg [7:0] d, c;
        begin
            d = data;
            c = crc;
            for (i = 0; i < 8; i = i + 1) begin
                if (c[0] ^ d[0])
                    c = {1'b0, c[7:1]} ^ 8'h8C;
                else
                    c = {1'b0, c[7:1]};
                d = {1'b0, d[7:1]};
            end
            crc8_maxim = c;
        end
    endfunction

    // ============================================
    // Temperature Parsing
    // DS18B20 12-bit format:
    //   Byte1[3:0] = {S, 2^6, 2^5, 2^4}  (upper bits)
    //   Byte0[7:0] = {2^3, 2^2, 2^1, 2^0, 2^-1, 2^-2, 2^-3, 2^-4}
    // Full 12-bit signed value = {Byte1[3:0], Byte0[7:0]}
    // Output: sign-extended to 16 bits, value in 0.0625 C/LSB
    // ============================================
    function [15:0] parse_temperature;
        input [15:0] raw;
        reg [11:0] temp12;
        begin
            // Extract full 12-bit value: {Byte1[3:0], Byte0[7:0]}
            temp12 = {raw[11:8], raw[7:0]};

            // Sign-extend to 16 bits (two's complement)
            if (temp12[11])
                parse_temperature = {4'hF, temp12};
            else
                parse_temperature = {4'h0, temp12};
        end
    endfunction

    // ============================================
    // Internal Signals
    // ============================================
    reg [31:0] timer;
    reg [7:0]  send_byte;
    reg [3:0]  bit_cnt;      // 0-8 bit counter (needs 4 bits for value 8)
    reg [3:0]  byte_cnt;     // 0-8 byte counter for reading
    reg        presence_detected;
    reg [71:0] scratchpad;   // 9 bytes received
    reg        crc_error;
    reg [7:0]  calc_crc;

    // ============================================
    // Combinational CRC chain for scratchpad
    // Computes CRC-8 over bytes 0-7 (first 8 bytes)
    // The 9th byte (scratchpad[71:64]) is the CRC itself
    // ============================================
    wire [7:0] crc_b0 = crc8_maxim(scratchpad[ 0+:8], 8'h00);
    wire [7:0] crc_b1 = crc8_maxim(scratchpad[ 8+:8], crc_b0);
    wire [7:0] crc_b2 = crc8_maxim(scratchpad[16+:8], crc_b1);
    wire [7:0] crc_b3 = crc8_maxim(scratchpad[24+:8], crc_b2);
    wire [7:0] crc_b4 = crc8_maxim(scratchpad[32+:8], crc_b3);
    wire [7:0] crc_b5 = crc8_maxim(scratchpad[40+:8], crc_b4);
    wire [7:0] crc_b6 = crc8_maxim(scratchpad[48+:8], crc_b5);
    // crc_calc: CRC of bytes 0-7 (should match scratchpad[71:64])
    wire [7:0] crc_calc  = crc8_maxim(scratchpad[56+:8], crc_b6);

    // ============================================
    // State Machine Register
    // ============================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
        end else begin
            state <= next_state;
        end
    end

    // ============================================
    // Next State Logic (combinational)
    // ============================================
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:       if (start) next_state = S_RESET;
            S_RESET:      if (timer >= T_RESET_LOW) next_state = S_PRESENCE;
            S_PRESENCE:   if (presence_detected && dq_in && timer >= T_POST_PRESENCE)
                              next_state = S_WR_SKIP_ROM;
                          else if (!presence_detected && timer >= T_PRESENCE_MAX)
                              next_state = S_ERROR;
            S_WR_SKIP_ROM: if (bit_cnt >= 8) next_state = S_WR_CONVERT;
            S_WR_CONVERT: if (bit_cnt >= 8) next_state = S_WAIT_CONV;
            S_WAIT_CONV:  if (timer >= T_CONVERT) next_state = S_RESET2;
            S_RESET2:     if (timer >= T_RESET_LOW) next_state = S_PRESENCE2;
            S_PRESENCE2:  if (presence_detected && dq_in && timer >= T_POST_PRESENCE)
                              next_state = S_WR_SKIP_ROM2;
                          else if (!presence_detected && timer >= T_PRESENCE_MAX)
                              next_state = S_ERROR;
            S_WR_SKIP_ROM2:if (bit_cnt >= 8) next_state = S_WR_READ_SP;
            S_WR_READ_SP: if (bit_cnt >= 8) next_state = S_READ;
            S_READ:       if (byte_cnt >= 9) next_state = S_CRC_CHECK;
            // Use combinational CRC chain directly - avoids 1-cycle stall
            S_CRC_CHECK:  next_state = (crc_calc == scratchpad[71:64]) ? S_DONE : S_ERROR;
            // Continuous sampling: if switch still on, loop to S_RESET
            S_DONE:       next_state = start ? S_RESET : S_IDLE;
            // Error: retry immediately if switch still on
            S_ERROR:      next_state = start ? S_RESET : S_IDLE;
            default:      next_state = S_IDLE;
        endcase
    end

    // ============================================
    // Datapath (sequential)
    // ============================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dq_out     <= 1'b0;
            dq_oe      <= 1'b0;   // FIXED: release bus on reset
            timer      <= 0;
            bit_cnt    <= 0;
            byte_cnt   <= 0;
            send_byte  <= 8'h00;
            presence_detected <= 0;
            scratchpad <= 72'h0;
            temperature<= 16'h0;
            data_valid <= 1'b0;
            error      <= 1'b0;
            crc_error  <= 1'b0;
            calc_crc   <= 8'h00;
        end else begin
            // Defaults
            data_valid <= 1'b0;
            // error holds until cleared by S_DONE or S_IDLE

            case (state)

                // -------- IDLE --------
                S_IDLE: begin
                    dq_oe  <= 1'b0;
                    dq_out <= 1'b0;
                    timer  <= 0;
                    bit_cnt <= 0;
                    byte_cnt <= 0;
                    error <= 1'b0;
                    crc_error <= 1'b0;
                    calc_crc <= 8'h00;
                end

                // -------- Reset Pulse --------
                S_RESET, S_RESET2: begin
                    dq_oe  <= 1'b1;  // Drive low
                    dq_out <= 1'b0;
                    timer  <= timer + 1;
                    if (timer >= T_RESET_LOW) begin
                        timer <= 0;
                    end
                end

                // -------- Presence Check --------
                S_PRESENCE, S_PRESENCE2: begin
                    dq_oe  <= 1'b0;  // Release bus
                    dq_out <= 1'b0;
                    if (!dq_in && timer >= T_PRESENCE_START) begin
                        presence_detected <= 1'b1;
                    end

                    if (presence_detected && dq_in) begin
                        // Presence done - wait T_POST_PRESENCE before next state
                        if (timer < T_POST_PRESENCE)
                            timer <= timer + 1;
                        else
                            timer <= 0;
                    end else if (presence_detected || timer >= T_PRESENCE_MAX) begin
                        // Holding during presence pulse or timeout
                        timer <= 0;
                    end else begin
                        timer <= timer + 1;
                    end
                end

                // -------- Write States (merged: set command + bit-bang) --------
                S_WR_SKIP_ROM, S_WR_SKIP_ROM2, S_WR_CONVERT, S_WR_READ_SP: begin
                    // Set command byte on first entry (timer==0, bit_cnt==0)
                    if (timer == 0 && bit_cnt == 0) begin
                        case (state)
                            S_WR_SKIP_ROM,
                            S_WR_SKIP_ROM2: send_byte <= CMD_SKIP_ROM;
                            S_WR_CONVERT:   send_byte <= CMD_CONVERT;
                            S_WR_READ_SP:   send_byte <= CMD_READ_SP;
                        endcase
                    end

                    if (bit_cnt < 8) begin
                        if (timer < T_WRITE_LOW) begin
                            // Phase 1: master pulls bus low
                            dq_oe  <= 1'b1;
                            dq_out <= 1'b0;
                            timer  <= timer + 1;
                        end else if (timer < T_SLOT) begin
                            // Phase 2: master releases for write-1, holds for write-0
                            if (send_byte[bit_cnt]) begin
                                // Write 1: release bus
                                dq_oe  <= 1'b0;
                                dq_out <= 1'b0;
                            end else begin
                                // Write 0: keep driving low
                                dq_oe  <= 1'b1;
                                dq_out <= 1'b0;
                            end
                            timer <= timer + 1;
                        end else if (timer < T_SLOT + T_RECOVERY) begin
                            // Phase 3: recovery - release bus for >= 1us
                            dq_oe  <= 1'b0;
                            dq_out <= 1'b0;
                            timer  <= timer + 1;
                        end else begin
                            // End of slot, advance to next bit
                            dq_oe  <= 1'b0;
                            dq_out <= 1'b0;
                            timer  <= 0;
                            bit_cnt <= bit_cnt + 1;
                        end
                    end else begin
                        // Byte complete: reset for next state
                        bit_cnt <= 0;
                        timer   <= 0;
                    end
                end

                // -------- Wait for Conversion --------
                S_WAIT_CONV: begin
                    dq_oe  <= 1'b0;
                    dq_out <= 1'b0;
                    if (timer >= T_CONVERT) begin
                        // Prepare for second reset sequence
                        timer <= 0;
                        presence_detected <= 1'b0;  // FIXED: clear before S_PRESENCE2
                    end else begin
                        timer <= timer + 1;
                    end
                end

                // -------- Read 9 Bytes (72 bits) --------
                S_READ: begin
                    if (byte_cnt < 9) begin
                        if (timer < T_READ_LOW) begin
                            // Phase 1: master pulls bus low
                            dq_oe  <= 1'b1;
                            dq_out <= 1'b0;
                            timer  <= timer + 1;
                        end else if (timer < T_SAMPLE) begin
                            // Phase 2: master releases, bus settles
                            dq_oe  <= 1'b0;
                            dq_out <= 1'b0;
                            timer  <= timer + 1;
                        end else if (timer == T_SAMPLE) begin
                            // Phase 3: sample data from slave
                            scratchpad[byte_cnt * 8 + bit_cnt] <= dq_in;
                            timer  <= timer + 1;
                        end else if (timer < T_SLOT) begin
                            // Phase 4: wait for end of slot
                            dq_oe  <= 1'b0;
                            timer  <= timer + 1;
                        end else if (timer < T_SLOT + T_RECOVERY) begin
                            // Phase 5: recovery - release bus
                            dq_oe  <= 1'b0;
                            timer  <= timer + 1;
                        end else begin
                            // End of slot: advance bit counter
                            timer  <= 0;
                            dq_oe  <= 1'b0;
                            if (bit_cnt >= 7) begin
                                bit_cnt <= 0;
                                byte_cnt <= byte_cnt + 1;
                            end else begin
                                bit_cnt <= bit_cnt + 1;
                            end
                        end
                    end
                end

                // -------- CRC Check --------
                S_CRC_CHECK: begin
                    // Capture combinational CRC result for debug
                    calc_crc  <= crc_calc;
                    // CRC comparison done combinationally in next_state;
                    // crc_calc != scratchpad[71:64] → S_ERROR
                    // crc_calc == scratchpad[71:64] → S_DONE
                end

                // -------- Done --------
                S_DONE: begin
                    temperature <= parse_temperature({scratchpad[15:8], scratchpad[7:0]});
                    data_valid <= 1'b1;
                    dq_oe  <= 1'b0;
                    // Clear for next conversion loop
                    error     <= 1'b0;
                    crc_error <= 1'b0;
                    calc_crc  <= 8'h00;
                    presence_detected <= 1'b0;
                    byte_cnt  <= 0;
                    bit_cnt   <= 0;
                    timer     <= 0;
                end

                // -------- Error --------
                S_ERROR: begin
                    error     <= 1'b1;
                    crc_error <= 1'b1;
                    dq_oe     <= 1'b0;
                    // Clear for retry
                    presence_detected <= 1'b0;
                    calc_crc  <= 8'h00;
                    byte_cnt  <= 0;
                    bit_cnt   <= 0;
                    timer     <= 0;
                end

            endcase
        end
    end

    // Debug outputs
    assign debug_state = state;
    assign debug_send  = send_byte;
    assign debug_bit   = bit_cnt;
    assign debug_err   = crc_error;

endmodule