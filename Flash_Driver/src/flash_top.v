`timescale 1ns / 1ps
//=============================================================================
// Module : flash_top
// Purpose: Top-level wrapper for the W25Q64 / W25Q128 driver.
//
// Instantiates the SPI master and the flash controller, and exposes a single
// flat user interface (start, op, addr, wdata, rdata, etc.) plus the four
// SPI pads (sck, csn, mosi, miso).
//
// System clock -> SCK divider is set by CLK_DIV. For 100 MHz system clock
// and CLK_DIV=4, SCK = 25 MHz (W25Q64 max is 80 MHz standard SPI).
//=============================================================================
module flash_top #(
    parameter CLK_DIV = 4   // SCK period in system clocks
) (
    input  wire        clk,
    input  wire        rst_n,

    // ---- User interface ----
    input  wire        start,
    input  wire [3:0]  op,
    input  wire [23:0] addr,
    input  wire [7:0]  wdata,
    input  wire        wdata_valid,
    output wire        wdata_ready,
    input  wire [15:0] len,
    output wire [7:0]  rdata,
    output wire        rdata_valid,
    output wire        busy,
    output wire        done,
    output wire        error,

    // Status register-1 last read by controller (BUSY/WEL/BP/TB/SEC/SRP0)
    output wire [7:0]  status_reg1,

    // ---- SPI pads to flash chip ----
    output wire        sck,
    output wire        csn,
    output wire        mosi,
    input  wire        miso
);

    // SPI master <-> flash controller internal nets
    wire        spi_start;
    wire        spi_csn_hold;
    wire [7:0]  spi_tx_data;
    wire [7:0]  spi_rx_data;
    wire        spi_busy;
    wire        spi_done;

    // -------------------------------------------------------------------------
    // SPI master instance
    // -------------------------------------------------------------------------
    spi_master #(
        .CLK_DIV(CLK_DIV)
    ) u_spi_master (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (spi_start),
        .csn_hold (spi_csn_hold),
        .tx_data  (spi_tx_data),
        .rx_data  (spi_rx_data),
        .busy     (spi_busy),
        .done     (spi_done),
        .sck      (sck),
        .csn      (csn),
        .mosi     (mosi),
        .miso     (miso)
    );

    // -------------------------------------------------------------------------
    // Flash controller instance
    // -------------------------------------------------------------------------
    flash_ctrl u_flash_ctrl (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start),
        .op           (op),
        .addr         (addr),
        .wdata        (wdata),
        .wdata_valid  (wdata_valid),
        .wdata_ready  (wdata_ready),
        .len          (len),
        .rdata        (rdata),
        .rdata_valid  (rdata_valid),
        .busy         (busy),
        .done         (done),
        .error        (error),
        .status_reg1  (status_reg1),
        .spi_start    (spi_start),
        .spi_csn_hold (spi_csn_hold),
        .spi_tx_data  (spi_tx_data),
        .spi_rx_data  (spi_rx_data),
        .spi_busy     (spi_busy),
        .spi_done     (spi_done)
    );

endmodule
