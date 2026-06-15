`timescale 1ns / 1ps
// Test flash_top directly: pulse start with OP_READ_JEDEC_ID, see if done fires
module tb_flash_top_oneshot;
    reg clk = 0;
    always #5 clk = ~clk;

    reg        start = 0;
    reg [3:0]  op    = 0;
    reg [23:0] addr  = 0;
    reg [7:0]  wdata = 0;
    reg        wdata_valid = 0;
    reg [15:0] len   = 0;
    wire       wdata_ready, busy, done, error, rdata_valid;
    wire [7:0] rdata, status_reg1;
    wire       sck, csn, mosi, miso;

    flash_top #(.CLK_DIV(4)) u_flash (
        .clk(clk), .rst_n(1'b1),
        .start(start), .op(op), .addr(addr), .wdata(wdata),
        .wdata_valid(wdata_valid), .wdata_ready(wdata_ready),
        .len(len), .rdata(rdata), .rdata_valid(rdata_valid),
        .busy(busy), .done(done), .error(error), .status_reg1(status_reg1),
        .sck(sck), .csn(csn), .mosi(mosi), .miso(miso)
    );
    flash_model #(.MEM_DEPTH(64*1024)) u_flash_mem (
        .sck(sck), .csn(csn), .mosi(mosi), .miso(miso)
    );

    initial begin
        #100;
        @(posedge clk);
        op    = 4'h1;     // OP_READ_JEDEC_ID
        addr  = 0;
        len   = 0;
        start = 1;
        @(posedge clk);
        start = 0;
        // Wait up to 1 ms for done
        fork
            begin
                wait (busy == 1);
                $display("[%0t] busy=1", $time);
                wait (done == 1);
                $display("[%0t] done=1, rdata_valid=%b", $time, rdata_valid);
            end
            begin
                #1_000_000;
                $display("[%0t] TIMEOUT. busy=%b done=%b csn=%b sck=%b mosi=%b",
                         $time, busy, done, csn, sck, mosi);
                $finish;
            end
        join_any
        disable fork;
        #1000;
        $finish;
    end
endmodule
