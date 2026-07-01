`timescale 1ns / 1ps
//=============================================================================
// Module : spi_master
// Purpose: Generic 8-bit SPI master, Mode 0 (CPOL=0, CPHA=0).
//          - MSB first
//          - SCK idles low, MOSI/MISO sampled on rising edge
//          - 4-wire SPI: SCK, CSn, MOSI, MISO
//          - Byte-level handshake: pulse `start` to begin,
//            `done` pulses for one cycle when the byte is fully shifted.
//          - `rx_data` is valid on the cycle `done` is high.
//
// Parameter CLK_DIV sets the SCK period in system-clock cycles.
//   Example: 100 MHz system clock, CLK_DIV=4 -> 25 MHz SCK.
//   CLK_DIV must be >= 2 (so that both half-periods are at least 1 cycle).
//   CLK_DIV must be a power of 2 for clean $clog2 sizing.
//
// `csn_hold`: when high, CSn is kept asserted across byte boundaries
//             (no deassertion between bytes). This matches the way real
//             W25Q64 commands are sent: the entire instruction + address
//             + data phase is one CSn-low transaction. The flash model
//             relies on CSn being held low during multi-byte operations
//             (Page Program, Read Data) so it can correctly distinguish
//             data bytes from new commands.
//=============================================================================
module spi_master #(
    parameter CLK_DIV = 4   // SCK period = CLK_DIV * system_clock_period
) (
    input  wire        clk,        // system clock
    input  wire        rst_n,      // active-low async reset
    // Byte-level user interface
    input  wire        start,      // start a byte transaction
    input  wire        csn_hold,   // 1: keep CSn low between bytes
    input  wire [7:0]  tx_data,    // byte to transmit (MSB first)
    output reg  [7:0]  rx_data,    // received byte (valid when done=1)
    output reg         busy,       // high while a transaction is in progress
    output reg         done,       // one-cycle pulse on completion
    // SPI pads (Mode 0: SCK idles low, CSn active low)
    output reg         sck,
    output reg         csn,
    output reg         mosi,
    input  wire        miso
);

    localparam HALF = CLK_DIV >> 1;                       // half-period in clk cycles
    localparam CW   = (HALF <= 1) ? 1 : $clog2(HALF);     // half-period counter width

    // FSM states
    localparam [2:0] S_IDLE    = 3'd0,
                     S_ASSERT  = 3'd1,  // CSn low, wait tSLCH
                     S_XFER    = 3'd2,  // shifting 8 bits
                     S_HOLD    = 3'd3,  // SCK low, CSn low, wait tCHSH
                     S_RELOAD  = 3'd4,  // CSn-hold: wait one cycle for controller to update tx_data, then reload
                     S_RELEASE = 3'd5,  // CSn high, wait tSHSL
                     S_DONE    = 3'd6;

    reg [2:0]    state;
    reg [CW-1:0] half_cnt;     // half-period tick counter
    reg [3:0]    bit_cnt;      // bit index, 0..7
    reg [7:0]    tx_sh;
    reg [7:0]    rx_sh;

    wire half_done = (half_cnt == HALF - 1);

    // ----------------------------------------------------------------------
    // Main FSM
    // ----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            sck      <= 1'b0;
            csn      <= 1'b1;
            mosi     <= 1'b0;
            busy     <= 1'b0;
            done     <= 1'b0;
            rx_data  <= 8'd0;
            half_cnt <= {CW{1'b0}};
            bit_cnt  <= 4'd0;
            tx_sh    <= 8'd0;
            rx_sh    <= 8'd0;
        end else begin
            // default: clear single-cycle pulses
            done <= 1'b0;

            case (state)
                // ------------------------------------------------------------
                S_IDLE: begin
                    sck  <= 1'b0;
                    // Default: release CSn so the next byte starts a fresh
                    // transaction. But if csn_hold=1, keep CSn low: the next
                    // byte is part of the same multi-byte transaction and we
                    // must not glitch CSn high (that would confuse the flash
                    // model, which would treat the next data byte as a new
                    // command).
                    if (!csn_hold) csn <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        csn     <= 1'b0;        // assert CSn
                        mosi    <= tx_data[7];  // present MSB
                        tx_sh   <= tx_data;
                        rx_sh   <= 8'd0;
                        bit_cnt <= 4'd0;
                        busy    <= 1'b1;
                        state   <= S_ASSERT;
                    end
                end

                // ------------------------------------------------------------
                // CSn is low, wait tSLCH (>= 5 ns) before first SCK rising edge.
                S_ASSERT: begin
                    if (half_done) begin
                        half_cnt <= {CW{1'b0}};
                        sck      <= 1'b1;   // first rising edge
                        rx_sh    <= {rx_sh[6:0], miso};
                        state    <= S_XFER;
                    end else begin
                        half_cnt <= half_cnt + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // Shift 8 bits. On each rising edge of SCK sample MISO;
                // on each falling edge update MOSI for the next bit.
                S_XFER: begin
                    if (half_done) begin
                        half_cnt <= {CW{1'b0}};
                        if (sck == 1'b1) begin
                            // Falling edge: update MOSI for next bit
                            sck <= 1'b0;
                            if (bit_cnt == 4'd7) begin
                                state <= S_HOLD;
                            end else begin
                                bit_cnt <= bit_cnt + 1'b1;
                                mosi    <= tx_sh[6 - bit_cnt[2:0]];
                            end
                        end else begin
                            // Rising edge: sample MISO
                            sck   <= 1'b1;
                            rx_sh <= {rx_sh[6:0], miso};
                        end
                    end else begin
                        half_cnt <= half_cnt + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // SCK low, CSn still low, wait tCHSH (>= 5 ns).
                // If csn_hold=1, do NOT deassert CSn: the next byte is
                // part of the same transaction. Hand off to S_RELOAD,
                // which gives the controller one cycle to update tx_data
                // (it sees spi_done=1 the cycle we're in S_DONE), then
                // we latch the new byte and start shifting.
                S_HOLD: begin
                    if (half_done) begin
                        half_cnt <= {CW{1'b0}};
                        if (csn_hold) begin
                            sck     <= 1'b0;
                            state   <= S_RELOAD;
                        end else begin
                            csn      <= 1'b1;
                            state    <= S_RELEASE;
                        end
                    end else begin
                        half_cnt <= half_cnt + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // CSn-hold bridge: latch the received byte (which would
                // normally happen in S_RELEASE), then wait for the
                // controller to update tx_data before going to S_DONE.
                S_RELOAD: begin
                    rx_data <= rx_sh;
                    state   <= S_DONE;
                end

                // ------------------------------------------------------------
                // CSn high, wait tSHSL (>= 10 ns for read, 50 ns for write).
                // One full SCK period is more than enough.
                S_RELEASE: begin
                    if (half_done) begin
                        rx_data <= rx_sh;
                        state   <= S_DONE;
                    end else begin
                        half_cnt <= half_cnt + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                S_DONE: begin
                    done  <= 1'b1;
                    // Release CSn at the end of a byte, but only if the
                    // controller isn't asking us to hold CSn for the next
                    // byte in the same transaction.
                    if (!csn_hold) csn <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
