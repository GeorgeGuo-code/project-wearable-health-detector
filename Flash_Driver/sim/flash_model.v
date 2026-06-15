`timescale 1ns / 1ps
//=============================================================================
// Module : flash_model
// Purpose: Simple behavioral model of a W25Q64 SPI flash for simulation.
//          APPROXIMATE - intended for driver validation, not accuracy vs.
//          the real part.
//
// Design:
//   - Byte-level processing: each CSn-low period is one byte.
//   - State is reset to S_CMD on posedge csn so each byte starts fresh.
//   - current_cmd persists across CSn transitions (it's set by the first
//     byte and used to interpret subsequent bytes within the same multi-byte
//     transaction like Page Program or Read Data).
//   - The controller must send dummy bytes (0xFF) after read cmds to clock
//     out the response. The model drives the response on MISO for the byte
//     immediately following the cmd.
//=============================================================================
module flash_model #(
    parameter MEM_DEPTH = 64*1024
) (
    input  wire        sck,
    input  wire        csn,
    input  wire        mosi,
    output reg         miso
);

    reg [7:0] mem [0:MEM_DEPTH-1];
    integer i;
    initial begin
        for (i = 0; i < MEM_DEPTH; i = i + 1) mem[i] = 8'hFF;
        miso       = 1'b0;
        drive_miso = 1'b0;
        tx_byte    = 8'h00;
        state      = S_CMD;
        current_addr    = 24'd0;
        current_cmd     = 8'h00;
        addr_bytes_left = 3'd0;
        data_count      = 16'd0;
        bit_cnt         = 4'd0;
        shreg           = 8'd0;
        byte_done       = 1'b0;
    end

    reg [7:0]  status_reg = 8'h00;
    reg        wel        = 1'b0;

    reg [7:0]  shreg;
    reg [3:0]  bit_cnt;
    reg        byte_done;

    // FSM
    localparam [2:0] S_IDLE    = 3'd0;
    localparam [2:0] S_CMD     = 3'd1;
    localparam [2:0] S_ADDR    = 3'd2;
    localparam [2:0] S_DATA_TX = 3'd3;
    localparam [2:0] S_DATA_RX = 3'd4;

    reg [2:0]  state;
    reg [7:0]  current_cmd;
    reg [23:0] current_addr;
    reg [2:0]  addr_bytes_left;
    reg [15:0] data_count;

    reg        drive_miso;
    reg [7:0]  tx_byte;

    localparam [7:0] JEDEC_MFR = 8'hEF;
    localparam [7:0] JEDEC_TYP = 8'h40;
    localparam [7:0] JEDEC_CAP = 8'h17;

    // Is the byte a known flash command? Used to detect end of multi-byte
    // data transfer (Page Program, Read Data).
    function is_known_cmd;
        input [7:0] b;
        begin
            case (b)
                8'h9F, 8'h05, 8'h35, 8'h01,
                8'h03, 8'h0B, 8'h02,
                8'h20, 8'h52, 8'hD8, 8'hC7, 8'h60,
                8'h06, 8'h04,
                8'hB9, 8'h66, 8'h99,
                8'h4B: is_known_cmd = 1'b1;
                default: is_known_cmd = 1'b0;
            endcase
        end
    endfunction

    // -------------------------------------------------------------------------
    // On SCK rising edge: shift in MOSI
    // -------------------------------------------------------------------------
    always @(posedge sck) begin
        if (!csn) begin
            shreg <= {shreg[6:0], mosi};
            if (bit_cnt == 4'd7) begin
                byte_done <= 1'b1;
            end else begin
                bit_cnt <= bit_cnt + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // On SCK falling edge: drive MISO bit, and if byte_done, process the byte.
    // -------------------------------------------------------------------------
    always @(negedge sck) begin
        if (!csn) begin
            if (drive_miso) begin
                miso <= tx_byte[7 - bit_cnt];
            end
            if (byte_done) begin
                byte_done <= 1'b0;
                bit_cnt   <= 4'd0;
                on_byte_complete(shreg);
            end
        end
    end

    // -------------------------------------------------------------------------
    // Handle a fully received byte. Process based on current state.
    // -------------------------------------------------------------------------
    task on_byte_complete(input [7:0] b);
        begin
            case (state)
                S_IDLE, S_CMD: begin
                    // First byte of a transaction: it's a command
                    current_cmd <= b;
                    case (b)
                        8'h9F: begin
                            // JEDEC ID: response is 3 bytes
                            drive_miso <= 1'b1;
                            tx_byte    <= JEDEC_MFR;
                            miso       <= JEDEC_MFR[7];
                            state      <= S_DATA_RX;
                            data_count <= 16'd0;  // 0 response bytes sent
                        end
                        8'h05: begin
                            // Read Status: response is 1 byte
                            drive_miso <= 1'b1;
                            tx_byte    <= status_reg;
                            miso       <= status_reg[7];
                            state      <= S_DATA_RX;
                            data_count <= 16'd0;
                        end
                        8'h03: begin
                            // Read Data: 3 addr bytes follow
                            current_addr    <= 24'd0;
                            addr_bytes_left <= 3'd3;
                            state           <= S_ADDR;
                        end
                        8'h02: begin
                            // Page Program: 3 addr bytes follow, then data
                            if (wel) begin
                                current_addr    <= 24'd0;
                                addr_bytes_left <= 3'd3;
                                state           <= S_ADDR;
                            end else begin
                                state <= S_IDLE;
                            end
                        end
                        8'h20, 8'h52, 8'hD8: begin
                            // Erase: 3 addr bytes follow
                            if (wel) begin
                                current_addr    <= 24'd0;
                                addr_bytes_left <= 3'd3;
                                state           <= S_ADDR;
                            end else begin
                                state <= S_IDLE;
                            end
                        end
                        8'hC7, 8'h60: begin
                            if (wel) do_chip_erase();
                            state <= S_IDLE;
                        end
                        8'h06: begin
                            wel   <= 1'b1;
                            state <= S_IDLE;
                        end
                        8'h04: begin
                            wel   <= 1'b0;
                            state <= S_IDLE;
                        end
                        8'hB9, 8'hAB, 8'h66, 8'h99: state <= S_IDLE;
                        default: state <= S_IDLE;
                    endcase
                end

                S_ADDR: begin
                    if (addr_bytes_left == 3'd1) begin
                        // Last addr byte
                        current_addr <= {current_addr[15:0], b};
                        case (current_cmd)
                            8'h03: begin
                                // Read Data: prepare first data byte response
                                drive_miso <= 1'b1;
                                tx_byte    <= mem[{current_addr[15:0], b}];
                                miso       <= mem[{current_addr[15:0], b}][7];
                                state      <= S_DATA_RX;
                                data_count <= 16'd0;  // 0 bytes sent yet
                            end
                            8'h02: begin state <= S_DATA_TX; data_count <= 16'd0; end
                            8'h20: begin do_sector_erase({current_addr[15:0], b}); state <= S_IDLE; end
                            8'h52: begin do_block_erase_32k({current_addr[15:0], b}); state <= S_IDLE; end
                            8'hD8: begin do_block_erase_64k({current_addr[15:0], b}); state <= S_IDLE; end
                            default: state <= S_IDLE;
                        endcase
                    end else begin
                        current_addr    <= {current_addr[15:0], b};
                        addr_bytes_left <= addr_bytes_left - 1'b1;
                    end
                end

                S_DATA_TX: begin
                    if (current_cmd == 8'h02) begin
                        // Page Program data byte. End-of-data is detected on
                        // posedge csn (see always @(posedge csn) below), not
                        // by the value of the data byte, so a data value
                        // that happens to match a flash command opcode
                        // (e.g. 0x01) is still treated as data.
                        mem[current_addr + data_count] <= b;
                        data_count <= data_count + 1;
                    end
                end

                S_DATA_RX: begin
                    case (current_cmd)
                        8'h9F: begin
                            // JEDEC ID response
                            case (data_count)
                                16'd0: begin
                                    tx_byte <= JEDEC_TYP;
                                    miso    <= JEDEC_TYP[7];
                                end
                                16'd1: begin
                                    tx_byte <= JEDEC_CAP;
                                    miso    <= JEDEC_CAP[7];
                                end
                                default: begin
                                    drive_miso <= 1'b0;
                                    state      <= S_IDLE;
                                end
                            endcase
                            data_count <= data_count + 1;
                        end
                        8'h05: begin
                            // Read Status: 1 byte response
                            drive_miso <= 1'b0;
                            state      <= S_IDLE;
                        end
                        8'h03: begin
                            // Read Data: prepare next data byte. End-of-read
                            // is detected on posedge csn.
                            data_count <= data_count + 1;
                            tx_byte    <= mem[current_addr + data_count + 1];
                            miso       <= mem[current_addr + data_count + 1][7];
                        end
                        default: state <= S_IDLE;
                    endcase
                end

                default: state <= S_IDLE;
            endcase
        end
    endtask

    // -------------------------------------------------------------------------
    // CSn rising edge: end of byte transaction.
    //   - For S_DATA_TX (Page Program): the controller releases CSn to
    //     indicate end of data. Clear WEL (per W25Q64 spec, /WEL
    //     auto-clears on /CS high after a write) and return to S_IDLE
    //     so the next CSn-low starts a fresh command.
    //   - For S_DATA_RX (Read Data, JEDEC ID, Read Status): the controller
    //     releases CSn to indicate end of read; we stop driving MISO and
    //     return to S_IDLE.
    // -------------------------------------------------------------------------
    always @(posedge csn) begin
        case (state)
            S_DATA_TX: begin
                wel  = 1'b0;
                state = S_IDLE;
            end
            S_DATA_RX: begin
                drive_miso <= 1'b0;
                state      <= S_IDLE;
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // Erase / program tasks
    // -------------------------------------------------------------------------
    task do_sector_erase(input [23:0] a);
        integer idx;
        begin
            for (idx = 0; idx < 4096; idx = idx + 1)
                mem[(a & 24'hFFF000) + idx[23:0]] = 8'hFF;
            wel = 1'b0;
        end
    endtask

    task do_block_erase_32k(input [23:0] a);
        integer idx;
        begin
            for (idx = 0; idx < 32768; idx = idx + 1)
                mem[(a & 24'hFFFF8000) + idx[23:0]] = 8'hFF;
            wel = 1'b0;
        end
    endtask

    task do_block_erase_64k(input [23:0] a);
        integer idx;
        begin
            for (idx = 0; idx < 65536; idx = idx + 1)
                mem[(a & 24'hFF000000) | (a & 24'hFFFF0000) + idx[23:0]] = 8'hFF;
            wel = 1'b0;
        end
    endtask

    task do_chip_erase;
        integer idx;
        begin
            for (idx = 0; idx < MEM_DEPTH; idx = idx + 1)
                mem[idx] = 8'hFF;
            wel = 1'b0;
        end
    endtask

    always @(*) begin
        status_reg[0] = 1'b0;
        status_reg[1] = wel;
    end

endmodule
