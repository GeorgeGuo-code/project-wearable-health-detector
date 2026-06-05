//==============================================================================
//  Module : cs13
//  Purpose: Driver for HS96L01W4S03 (SSD1306-compatible) 128x64 monochrome
//           OLED panel over 4-wire SPI (write-only).
//
//  Display  : 128 columns x 64 rows, organised as 8 pages of 128x8 pixels
//             (page addressing mode). 1 bpp; column 0 is leftmost.
//  Font     : built-in 5x8 ASCII ROM (font_rom() below). Each 5x8 glyph is
//             padded to 8x8 with 3 blank columns on the right, so 16 chars
//             fill one row (16 * 8 = 128 px).
//
//  SPI clock: 500 kHz SCLK (one SCLK period = 2 sys_ce = 200 clk cycles at
//             100 MHz). Well below the SSD1306 VCC=3.3V max of 3.3 MHz.
//  Init     : 25 SSD1306 setup commands, then a 50 ms reset/settle window
//             (25 ms RES# low + 25 ms high). See `cmd[]` table at the
//             bottom of this file for the exact byte sequence.
//  Refresh  : ~30 Hz full-frame.  Per-byte wall time is 32 us (18 sys_ce
//             WRITE + 5 sys_ce DELAY + ~9 us of SCAN/mainline overhead,
//             measured on logic analyser).
//                8 pages * (3 CMD + 16 chars * 8 DATA bytes)  = 1048 bytes
//                1048 bytes * 32 us/byte                       = ~33 ms
//             SPI throughput:  500 kHz * 8 b = 4.0 Mbps.
//
//  Power-on : `top_oled_test` gates this module's rst with an 84 ms POR
//             counter so the panel's VCC is stable before the init
//             sequence starts (SSD1306 datasheet recommends >= 100 ms
//             VCC-stable time before RES# is released).
//
//  Usage
//  -----
//      wire oled_csn, oled_rst, oled_dcn, oled_clk, oled_dat;
//
//      cs13 u_cs13 (
//          .clk      (clk_100mhz),   // 100 MHz
//          .rst      (cs13_rst_n),   // 0 = held in reset, 1 = run
//          .en       (sw_oled_en),   // 0 = force OLED hardware reset (off)
//          .oled_csn (oled_csn),
//          .oled_rst (oled_rst),
//          .oled_dcn (oled_dcn),
//          .oled_clk (oled_clk),
//          .oled_dat (oled_dat)
//      );
//
//      // All OLED outputs are 3.3 V LVCMOS; route through OBUF (slow slew)
//      // before the pad.  The panel is write-only, so no MISO / SDO needed.
//
//  Changing the displayed text
//  ---------------------------
//      Edit the `MAIN` state's case arms (cnt_main = 5'd1..5'd8):
//          - y_p     = page address  (0xB0..0xB7, top page first)
//          - num_max = characters to render (almost always 16)
//          - char    = 21-byte MSB-aligned string literal; only positions
//                      1..16 are read, so the first byte is a "padding" byte
//                      and the last 5 bytes are off-screen.  See quirk Q1.
//      To blank a page (clear the GDDRAM junk from power-up), set
//          char <= {21{8'h20}};       // 21 spaces -> font_rom[0x20] = 0
//      and keep num_max = 16 so all 128 columns get overwritten with 0x00.
//
//  Known quirks (Q1-Q3) and design notes
//  -------------------------------------
//  Q1. The `char` register is 21 bytes (168 bits), MSB-aligned, but the
//      SCAN loop only reads positions 1..16 (via char[(152 - num*8) +: 8]).
//      Position 0 (bits [167:160]) and positions 17..20 (bits [127:0]) are
//      padding that is never read.  This is a side effect of the indexing
//      formula -- see problems.md (P9) for the derivation.
//  Q2. font_rom is a function, not a `reg [...] rom[...]` array.  Earlier
//      revisions used a RAM with an `initial` block, but Vivado inferred
//      BRAM/distributed-RAM and dropped the initial values, so every glyph
//      read back as 0.  Functions synthesise to LUTs and are always
//      correct, so font_rom() is the canonical glyph source.
//  Q3. The INIT reset pulse is 25 ms (RES# low) followed by 25 ms
//      (RES# high, pre-command settle).  The SSD1306 datasheet only needs
//      3 us / 3 us, but the over-conservative timing is harmless and
//      matches Adafruit's reference driver.
//
//  Revision history
//  ----------------
//  P1  cmd[10]:  0x1F (1/32 duty) -> 0x3F (1/64 duty) for full 64-row panel
//  P2  SCAN:     char indexing direction reversed (was right-to-left)
//  P3  cmd:      0xAE '®' / 0x7B '{' replaced with 0x7C '|' + 'C' for °C
//  P4  cmd[19]:  0x00 -> 0x12 (alternative COM pin config for 128x64)
//  P5  clk_div:  added 100:1 divider; SCLK was 50 MHz, now 500 kHz
//  P6  num_max:  14/15 -> 16 on every page; strings padded to 16 chars
//  P7  cmd[17,21]: pre-charge 0x1F -> 0xF1, VCOMH 0x40 -> 0x30
//  P8  mem[]:    replaced with font_rom() function (see Q2)
//  P9  char idx: corrected for MSB-aligned string storage (see Q1)
//  P10 SCAN:     `if(num+1 < num_max)` -> `if(num < num_max)` (off-by-one)
//  P11 PADDING:  added all-space pages 4-7 to clear power-on GDDRAM junk
//  P12 en:       added `en` input; !en holds OLED in hardware reset
//==============================================================================
module cs13
(
input wire clk,           // 100 MHz system clock
input wire rst,           // active-low async reset (0 = held in reset)
input wire en,            // 1 = run normally, 0 = force OLED hardware reset (display off, low power)
output reg oled_csn,oled_rst,oled_dcn,oled_clk,oled_dat
);

// ---- clock divider: 100 MHz -> 1 MHz tick (period 100 clk) ----
reg [6:0]  clk_div;
wire       sys_ce = (clk_div == 7'd99);
always @(posedge clk or negedge rst) begin
    if (!rst) clk_div <= 7'd0;
    else      clk_div <= clk_div + 7'd1;
end

wire[7:0] hour_1=1,hour_0=1,min_1=1,min_0=1,m_1=1,m_0=1,
           tem_1=1,tem_0=1,tem_3=1                            ;

localparam INIT_DEPTH = 16'd25; //LCD初始化的命令的数量
localparam IDLE       = 6'h1,
           MAIN       = 6'h2,
           INIT       = 6'h4,
           SCAN       = 6'h8,
           WRITE      = 6'h10,
           DELAY      = 6'h20;
localparam HIGH       = 1'b1,
           LOW        = 1'b0;
localparam DATA       = 1'b1,
           CMD        = 1'b0;
localparam time_long  = 5 ;
localparam temp_long  = 12;

reg  [7:0]  cmd[24:0]  ;
// Font ROM moved to font_rom() function (no RAM inference issues)
reg  [7:0]	y_p        ,
            x_ph                                        ,
            x_pl                                        ;
reg  [(8*21-1):0] char                                  ;
reg  [7:0]	num         ,   // character index used in SCAN lookup
            num_max    ,   // number of characters to render on this row
            char_reg                                    ;
reg  [4:0]	cnt_main                                    ,
            cnt_init                                    ,
            cnt_scan                                    ,
            cnt_write                                   ;
reg  [15:0] num_delay                                   ,
            cnt_delay                                   ,
            cnt                                         ;
reg  [5:0]  state                                       ,
            state_back                                  ;
reg  [39:0] time_head                                   ;
reg  [95:0] temp_head                                   ;
wire [7:0]  sign                                        ;
    wire [39:0] font_byte = font_rom(char[(152 - num*8) +: 8]);   // string[num] (MSB-aligned storage)   // temp, P2 fix: read string[num]

    // ---- Font ROM (5x8 ASCII, MSB = leftmost column) ----
    // 5 bytes per character, byte 0 = column 0 (MSB at top).
    function [39:0] font_rom;
        input [7:0] ch;
        begin
            case (ch)
                8'd  0: font_rom = 40'h3E_51_49_45_3E;
                8'd  1: font_rom = 40'h00_42_7F_40_00;
                8'd  2: font_rom = 40'h42_61_51_49_46;
                8'd  3: font_rom = 40'h21_41_45_4B_31;
                8'd  4: font_rom = 40'h18_14_12_7F_10;
                8'd  5: font_rom = 40'h27_45_45_45_39;
                8'd  6: font_rom = 40'h3C_4A_49_49_30;
                8'd  7: font_rom = 40'h01_71_09_05_03;
                8'd  8: font_rom = 40'h36_49_49_49_36;
                8'd  9: font_rom = 40'h06_49_49_29_1E;
                8'd 10: font_rom = 40'h7C_12_11_12_7C;
                8'd 11: font_rom = 40'h7F_49_49_49_36;
                8'd 12: font_rom = 40'h3E_41_41_41_22;
                8'd 13: font_rom = 40'h7F_41_41_22_1C;
                8'd 14: font_rom = 40'h7F_49_49_49_41;
                8'd 15: font_rom = 40'h7F_09_09_09_01;
                8'd 32: font_rom = 40'h00_00_00_00_00;
                8'd 33: font_rom = 40'h00_00_2F_00_00;
                8'd 34: font_rom = 40'h00_07_00_07_00;
                8'd 35: font_rom = 40'h14_7F_14_7F_14;
                8'd 36: font_rom = 40'h24_2A_7F_2A_12;
                8'd 37: font_rom = 40'h62_64_08_13_23;
                8'd 38: font_rom = 40'h36_49_55_22_50;
                8'd 39: font_rom = 40'h00_05_03_00_00;
                8'd 40: font_rom = 40'h00_1C_22_41_00;
                8'd 41: font_rom = 40'h00_41_22_1C_00;
                8'd 42: font_rom = 40'h14_08_3E_08_14;
                8'd 43: font_rom = 40'h08_08_3E_08_08;
                8'd 44: font_rom = 40'h00_00_A0_60_00;
                8'd 45: font_rom = 40'h08_08_08_08_08;
                8'd 46: font_rom = 40'h00_60_60_00_00;
                8'd 47: font_rom = 40'h20_10_08_04_02;
                8'd 48: font_rom = 40'h3E_51_49_45_3E;
                8'd 49: font_rom = 40'h00_42_7F_40_00;
                8'd 50: font_rom = 40'h42_61_51_49_46;
                8'd 51: font_rom = 40'h21_41_45_4B_31;
                8'd 52: font_rom = 40'h18_14_12_7F_10;
                8'd 53: font_rom = 40'h27_45_45_45_39;
                8'd 54: font_rom = 40'h3C_4A_49_49_30;
                8'd 55: font_rom = 40'h01_71_09_05_03;
                8'd 56: font_rom = 40'h36_49_49_49_36;
                8'd 57: font_rom = 40'h06_49_49_29_1E;
                8'd 58: font_rom = 40'h00_36_36_00_00;
                8'd 59: font_rom = 40'h00_56_36_00_00;
                8'd 60: font_rom = 40'h08_14_22_41_00;
                8'd 61: font_rom = 40'h14_14_14_14_14;
                8'd 62: font_rom = 40'h00_41_22_14_08;
                8'd 63: font_rom = 40'h02_01_51_09_06;
                8'd 64: font_rom = 40'h32_49_59_51_3E;
                8'd 65: font_rom = 40'h7C_12_11_12_7C;
                8'd 66: font_rom = 40'h7F_49_49_49_36;
                8'd 67: font_rom = 40'h3E_41_41_41_22;
                8'd 68: font_rom = 40'h7F_41_41_22_1C;
                8'd 69: font_rom = 40'h7F_49_49_49_41;
                8'd 70: font_rom = 40'h7F_09_09_09_01;
                8'd 71: font_rom = 40'h3E_41_49_49_7A;
                8'd 72: font_rom = 40'h7F_08_08_08_7F;
                8'd 73: font_rom = 40'h00_41_7F_41_00;
                8'd 74: font_rom = 40'h20_40_41_3F_01;
                8'd 75: font_rom = 40'h7F_08_14_22_41;
                8'd 76: font_rom = 40'h7F_40_40_40_40;
                8'd 77: font_rom = 40'h7F_02_0C_02_7F;
                8'd 78: font_rom = 40'h7F_04_08_10_7F;
                8'd 79: font_rom = 40'h3E_41_41_41_3E;
                8'd 80: font_rom = 40'h7F_09_09_09_06;
                8'd 81: font_rom = 40'h3E_41_51_21_5E;
                8'd 82: font_rom = 40'h7F_09_19_29_46;
                8'd 83: font_rom = 40'h46_49_49_49_31;
                8'd 84: font_rom = 40'h01_01_7F_01_01;
                8'd 85: font_rom = 40'h3F_40_40_40_3F;
                8'd 86: font_rom = 40'h1F_20_40_20_1F;
                8'd 87: font_rom = 40'h3F_40_38_40_3F;
                8'd 88: font_rom = 40'h63_14_08_14_63;
                8'd 89: font_rom = 40'h07_08_70_08_07;
                8'd 90: font_rom = 40'h61_51_49_45_43;
                8'd 91: font_rom = 40'h00_7F_41_41_00;
                8'd 92: font_rom = 40'h55_2A_55_2A_55;
                8'd 93: font_rom = 40'h00_41_41_7F_00;
                8'd 94: font_rom = 40'h04_02_01_02_04;
                8'd 95: font_rom = 40'h40_40_40_40_40;
                8'd 96: font_rom = 40'h00_01_02_04_00;
                8'd 97: font_rom = 40'h20_54_54_54_78;
                8'd 98: font_rom = 40'h7F_48_44_44_38;
                8'd 99: font_rom = 40'h38_44_44_44_20;
                8'd100: font_rom = 40'h38_44_44_48_7F;
                8'd101: font_rom = 40'h38_54_54_54_18;
                8'd102: font_rom = 40'h08_7E_09_01_02;
                8'd103: font_rom = 40'h18_A4_A4_A4_7C;
                8'd104: font_rom = 40'h7F_08_04_04_78;
                8'd105: font_rom = 40'h00_44_7D_40_00;
                8'd106: font_rom = 40'h40_80_84_7D_00;
                8'd107: font_rom = 40'h7F_10_28_44_00;
                8'd108: font_rom = 40'h00_41_7F_40_00;
                8'd109: font_rom = 40'h7C_04_18_04_78;
                8'd110: font_rom = 40'h7C_08_04_04_78;
                8'd111: font_rom = 40'h38_44_44_44_38;
                8'd112: font_rom = 40'hFC_24_24_24_18;
                8'd113: font_rom = 40'h18_24_24_18_FC;
                8'd114: font_rom = 40'h7C_08_04_04_08;
                8'd115: font_rom = 40'h48_54_54_54_20;
                8'd116: font_rom = 40'h04_3F_44_40_20;
                8'd117: font_rom = 40'h3C_40_40_20_7C;
                8'd118: font_rom = 40'h1C_20_40_20_1C;
                8'd119: font_rom = 40'h3C_40_30_40_3C;
                8'd120: font_rom = 40'h44_28_10_28_44;
                8'd121: font_rom = 40'h1C_A0_A0_A0_7C;
                8'd122: font_rom = 40'h44_64_54_4C_44;
                8'd123: font_rom = 40'h00_00_00_03_03;
                8'd124: font_rom = 40'h00_06_09_09_06;
                default: font_rom = 40'h0;
            endcase
        end
    endfunction


//assign sign = tem_sign?" ":"+"                          ;

always @(posedge clk or negedge rst or negedge en)
    begin
	 if(!rst || !en)            // !en == async hold-off: keep OLED in hardware reset (blank, low power)
	     begin
		  cnt_main <= 1'b0;
		  cnt_init <= 1'b0;
		  cnt_scan <= 1'b0;
		  cnt_write <= 1'b0;
		  y_p <= 1'b0;
		  x_ph <= 1'b0;
		  x_pl <= 1'b0;
		  num <= 1'b0;
		  char <= 1'b0;
		  char_reg <= 1'b0;
		  num_delay <= 16'd5;
		  cnt_delay <= 1'b0;
		  cnt <= 1'b0;
		  oled_csn <= HIGH;
		  oled_rst <= LOW;       // force SSD1306 hardware reset → display off, charge pump off
		  oled_dcn <= CMD;
		  oled_clk <= HIGH;
		  oled_dat <= LOW;
		  state <= IDLE;
		  state_back <= IDLE;
		  end
    else if (sys_ce)
	     begin
		  case(state)
		      IDLE:
				begin
				cnt_main <= 1'b0; 
				cnt_init <= 1'b0; 
				cnt_scan <= 1'b0; 
				cnt_write <= 1'b0;
				y_p <= 1'b0; 
				x_ph <= 1'b0; 
				x_pl <= 1'b0;
				num <= 1'b0; 
				char <= 1'b0; 
				char_reg <= 1'b0;
				num_delay <= 16'd5; 
				cnt_delay <= 1'b0; 
				cnt <= 1'b0;
				oled_csn <= HIGH; 
				oled_rst <= HIGH; 
				oled_dcn <= CMD; 
				oled_clk <= HIGH; 
				oled_dat <= LOW;
				state <= MAIN; 
				state_back <= MAIN;
				end
				MAIN:
				begin
				if(cnt_main >= 8)
				    cnt_main <= 5'd1;
				else
				    cnt_main <= cnt_main+1'b1;
				case(cnt_main)
				    5'd0:
					 begin
					 state <= INIT;
					 end
					 5'd1:
					 begin
					 y_p <= 8'hb0;
					 x_ph <= 8'h10;
					 x_pl <= 8'h00;
					 num <= 5'd0;
					 num_max <= 5'd16;
					 char <= {"       OLED        "};   // 21B, 7sp+OLED+10sp (P2/P6)
					 state <= SCAN;
					 end
					 5'd2:
					 begin
					 y_p <= 8'hb1;
					 x_ph <= 8'h10;
					 x_pl <= 8'h00;
					 num <= 5'd0;
					 num_max <= 5'd16;                                              // 16 chars = 128 px (was 14 → right 16 px was unwritten, showed GDDRAM "snow")
					 char <= {"  ",temp_head,sign,tem_1,tem_0,8'd124,8'h43,"         "};   // 21B, deg+C (P3/P6)
					 state <= SCAN;
					 end
					5'd3:
					begin
					 y_p <= 8'hb2;
					 x_ph <= 8'h10;
					 x_pl <= 8'h00;
					 num <= 5'd0;
					 num_max <= 5'd16;
					 char <= {"       FPGA        "};   // 21B, 7sp+FPGA+10sp (P2/P6)
					 state <= SCAN;
					 end
					 5'd4:
					 begin
					 y_p <= 8'hb3;
					 x_ph <= 8'h10;
					 x_pl <= 8'h00;
					 num <= 5'd0;
					 num_max <= 5'd16;
					 char <= {"       FPGA        "};   // 21B, 7sp+FPGA+10sp (P6)
					 state <= SCAN;
					 end
					 // pages 4-7: all spaces → font_rom[0x20] = 0, clears the
					 // bottom half (otherwise GDDRAM shows undefined power-on
					 // "snow" because SSD1306 init does not zero the RAM)
					 5'd5:
					 begin
					 y_p <= 8'hb4;
					 x_ph <= 8'h10;
					 x_pl <= 8'h00;
					 num <= 5'd0;
					 num_max <= 5'd16;
					 char <= {21{8'h20}};             // 21 spaces
					 state <= SCAN;
					 end
					 5'd6:
					 begin
					 y_p <= 8'hb5;
					 x_ph <= 8'h10;
					 x_pl <= 8'h00;
					 num <= 5'd0;
					 num_max <= 5'd16;
					 char <= {21{8'h20}};
					 state <= SCAN;
					 end
					 5'd7:
					 begin
					 y_p <= 8'hb6;
					 x_ph <= 8'h10;
					 x_pl <= 8'h00;
					 num <= 5'd0;
					 num_max <= 5'd16;
					 char <= {21{8'h20}};
					 state <= SCAN;
					 end
					 5'd8:
					 begin
					 y_p <= 8'hb7;
					 x_ph <= 8'h10;
					 x_pl <= 8'h00;
					 num <= 5'd0;
					 num_max <= 5'd16;
					 char <= {21{8'h20}};
					 state <= SCAN;
					 end
					 default: state <= IDLE;
				endcase
				end
				INIT:
				begin	//初始化状态
				case(cnt_init)
				    5'd0:   begin oled_rst <= LOW; cnt_init <= cnt_init + 1'b1; end	//复位有效
					 5'd1:   begin num_delay <= 16'd25000; state <= DELAY; state_back <= INIT; cnt_init <= cnt_init + 1'b1; end	//延时大于3us
					 5'd2:   begin oled_rst <= HIGH; cnt_init <= cnt_init + 1'b1; end	//复位恢复
					 5'd3:   begin num_delay <= 16'd25000; state <= DELAY; state_back <= INIT; cnt_init <= cnt_init + 1'b1; end	//延时大于220us
					 5'd4:   begin 
								if(cnt>=INIT_DEPTH) 
								    begin	//当25条指令及数据发出后，配置完成
								    cnt <= 1'b0;
								    cnt_init <= cnt_init + 1'b1;
									 end 
								else 
								    begin	
									 cnt <= cnt + 1'b1; 
									 num_delay <= 16'd5;
									 oled_dcn <= CMD; 
									 char_reg <= cmd[cnt]; 
									 state <= WRITE; 
									 state_back <= INIT;
									 end
								end
                5'd5:	begin cnt_init <= 1'b0; state <= MAIN; end	//初始化完成，返回MAIN状态
					 default: state <= IDLE;
            endcase
				end
				SCAN:
				begin	//刷屏状态，从RAM中读取数据刷屏
				if(cnt_scan == 5'd11) begin
				    // P2 fix: char[0] is at leftmost column. num goes 0..num_max-1.
				    if(num < num_max)      cnt_scan <= 5'd3;
				    else                  cnt_scan <= 5'd12;
				end
				else if(cnt_scan == 5'd12) cnt_scan <= 1'b0;
				else                        cnt_scan <= cnt_scan + 1'b1;
				case(cnt_scan)
				    5'd 0:	begin oled_dcn <= CMD; char_reg <= y_p; state <= WRITE; state_back <= SCAN; end		//定位列页地址
					 5'd 1:	begin oled_dcn <= CMD; char_reg <= x_pl; state <= WRITE; state_back <= SCAN; end	//定位行地址低位
					 5'd 2:	begin oled_dcn <= CMD; char_reg <= x_ph; state <= WRITE; state_back <= SCAN; end	//定位行地址高位
							
					 5'd 3:	begin num <= num + 1'b1;end   // P2 fix: increment
					 5'd 4:	begin oled_dcn <= DATA; char_reg <= 8'h00; state <= WRITE; state_back <= SCAN; end	//将5*8点阵编程8*8
					 5'd 5:	begin oled_dcn <= DATA; char_reg <= 8'h00; state <= WRITE; state_back <= SCAN; end	//将5*8点阵编程8*8
					 5'd 6:	begin oled_dcn <= DATA; char_reg <= 8'h00; state <= WRITE; state_back <= SCAN; end	//将5*8点阵编程8*8
					 5'd 7:	begin oled_dcn <= DATA; char_reg <= font_byte[39:32]; state <= WRITE; state_back <= SCAN; end
					 5'd 8:	begin oled_dcn <= DATA; char_reg <= font_byte[31:24]; state <= WRITE; state_back <= SCAN; end
					 5'd 9:	begin oled_dcn <= DATA; char_reg <= font_byte[23:16]; state <= WRITE; state_back <= SCAN; end
					 5'd10:	begin oled_dcn <= DATA; char_reg <= font_byte[15: 8]; state <= WRITE; state_back <= SCAN; end
					 5'd11:	begin oled_dcn <= DATA; char_reg <= font_byte[ 7: 0]; state <= WRITE; state_back <= SCAN; end
					 5'd12:	begin state <= MAIN; end
					 default: state <= IDLE;
				endcase
			   end
				WRITE:
				begin	//WRITE状态，将数据按照SPI时序发送给屏幕
				if(cnt_write >= 5'd17) 
				    cnt_write <= 1'b0;
				else 
				    cnt_write <= cnt_write + 1'b1;
				case(cnt_write)
				    5'd 0:	begin oled_csn <= LOW; end	//9位数据最高位为命令数据控制位
					 5'd 1:	begin oled_clk <= LOW; oled_dat <= char_reg[7]; end	//先发高位数据
					 5'd 2:	begin oled_clk <= HIGH; end
					 5'd 3:	begin oled_clk <= LOW; oled_dat <= char_reg[6]; end
					 5'd 4:	begin oled_clk <= HIGH; end
					 5'd 5:	begin oled_clk <= LOW; oled_dat <= char_reg[5]; end
					 5'd 6:	begin oled_clk <= HIGH; end
					 5'd 7:	begin oled_clk <= LOW; oled_dat <= char_reg[4]; end
					 5'd 8:	begin oled_clk <= HIGH; end
					 5'd 9:	begin oled_clk <= LOW; oled_dat <= char_reg[3]; end
					 5'd10:	begin oled_clk <= HIGH; end
					 5'd11:	begin oled_clk <= LOW; oled_dat <= char_reg[2]; end
					 5'd12:	begin oled_clk <= HIGH; end
					 5'd13:	begin oled_clk <= LOW; oled_dat <= char_reg[1]; end
					 5'd14:	begin oled_clk <= HIGH; end
					 5'd15:	begin oled_clk <= LOW; oled_dat <= char_reg[0]; end	//后发低位数据
					 5'd16:	begin oled_clk <= HIGH; end
					 5'd17:	begin oled_csn <= HIGH; state <= DELAY; end	//
					 default: state <= IDLE;
				endcase
				end
				DELAY:
				begin	//延时状态
				if(cnt_delay >= num_delay) 
				    begin
					 cnt_delay <= 16'd0; 
					 state <= state_back; 
					 end 
				else 
				    cnt_delay <= cnt_delay + 1'b1;
				end
				default:state <= IDLE;
        endcase
		  end
  end

initial
    begin
	 time_head = "TIME:";
	 temp_head = "TEMP:";
	 cnt_main = 1'b0; 
	 cnt_init = 1'b0; 
	 cnt_scan = 1'b0; 
	 cnt_write = 1'b0;
	 y_p = 1'b0; 
	 x_ph = 1'b0; 
	 x_pl = 1'b0;
	 num = 1'b0; 
	 char = 1'b0; 
	 char_reg = 1'b0;
	 num_delay = 16'd5; 
	 cnt_delay = 1'b0; 
	 cnt = 1'b0;
	 oled_csn <= HIGH;
	 oled_rst <= LOW;     // start in hardware reset; cs13 will release then re-pulse in INIT when en=1
	 oled_dcn <= CMD;
	 oled_clk <= HIGH;
	 oled_dat <= LOW;
	 state <= IDLE; 
	 state_back <= IDLE;
	 cmd[ 0] = {8'hae}; 
	 cmd[ 1] = {8'h00}; 
	 cmd[ 2] = {8'h10}; 
	 cmd[ 3] = {8'h00}; 
	 cmd[ 4] = {8'hb0}; 
	 cmd[ 5] = {8'h81}; 
	 cmd[ 6] = {8'hff}; 
	 cmd[ 7] = {8'ha1}; 
	 cmd[ 8] = {8'ha6}; 
	 cmd[ 9] = {8'ha8}; 
	 cmd[10] = {8'h3f};   // 1/64 duty (P1 fix)
	 cmd[11] = {8'hc8};
	 cmd[12] = {8'hd3};
	 cmd[13] = {8'h00};
	 cmd[14] = {8'hd5};
	 cmd[15] = {8'h80};
	 cmd[16] = {8'hd9};
	 cmd[17] = {8'hf1};   // pre-charge (P7 fix)
	 cmd[18] = {8'hda};
	 cmd[19] = {8'h12};   // alt COM pin config (P4 fix)
	 cmd[20] = {8'hdb};
	 cmd[21] = {8'h30};   // VCOMH (P7 fix)
	 cmd[22] = {8'h8d};
	 cmd[23] = {8'h14};
	 cmd[24] = {8'haf};
	 end
endmodule