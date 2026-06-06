//==============================================================================
// cs13 — SSD1306 SPI OLED driver (health monitor edition)
//   Uses internal font_rom() for all rendering.
//   No external font_rom.sv needed.
//   Health data inputs → MAIN constructs char strings → SCAN → WRITE → SPI
//   INIT / WRITE / DELAY framework unchanged from proven working version.
//==============================================================================
module cs13
(
    input wire clk,            // 100 MHz
    input wire rst,            // active-low async reset
    input wire en,             // 1 = run, 0 = force OLED hardware reset

    // ---- health data inputs ----
    input wire [7:0]  heart_rate,      // bpm (30-220)
    input wire [7:0]  temperature,     // °C × 2 (e.g. 75 = 37.5°C)
    input wire [2:0]  activity_level,  // 0-4
    input wire [7:0]  cadence,         // steps per minute
    input wire [1:0]  health_status,   // 00=NORMAL 01=WARNING 10=DANGER
    input wire [7:0]  health_score,    // 0-100
    input wire [7:0]  tach_count,      // tachycardia event count
    input wire [7:0]  brad_count,      // bradycardia event count
    input wire [7:0]  fever_count,     // fever event count
    input wire [1:0]  worst_status,    // worst status this session
    input wire        work_en,         // 0=standby 1=work
    input wire [2:0]  display_mode,    // 0-5 screen selection

    output reg oled_csn, oled_rst, oled_dcn, oled_clk, oled_dat
);

    // ---- clock divider: 100 MHz -> 1 MHz tick (unchanged) ----
    reg [6:0]  clk_div;
    wire       sys_ce = (clk_div == 7'd99);
    always @(posedge clk or negedge rst) begin
        if (!rst) clk_div <= 7'd0;
        else      clk_div <= clk_div + 7'd1;
    end

    localparam INIT_DEPTH = 16'd25;
    localparam IDLE   = 6'h1,  MAIN  = 6'h2,  INIT  = 6'h4,
               SCAN   = 6'h8,  WRITE = 6'h10, DELAY = 6'h20;
    localparam HIGH   = 1'b1,  LOW   = 1'b0;
    localparam DATA   = 1'b1,  CMD   = 1'b0;

    reg [7:0] cmd[24:0];
    reg [7:0] y_p, x_ph, x_pl;
    reg [(8*21-1):0] char;
    reg [7:0] num, num_max, char_reg;
    reg [4:0] cnt_main, cnt_init, cnt_scan, cnt_write;
    reg [15:0] num_delay, cnt_delay, cnt;
    reg [5:0] state, state_back;

    // ---- screen type (0=standby, 1-6=work screens) ----
    wire [2:0] scr;
    assign scr = work_en ? (display_mode + 3'd1) : 3'd0;

    // ---- digit to ASCII function ----
    function [7:0] d2a;
        input [7:0] val;
        begin
            if (val < 8'd10) d2a = 8'd48 + val;
            else d2a = 8'd48 + (val % 8'd10);
        end
    endfunction

    // ---- pre-compute display digits from sensor data ----
    wire [7:0] hr_h = (heart_rate >= 8'd200) ? "2" : (heart_rate >= 8'd100) ? "1" : 8'h20;
    wire [7:0] hr_t = (heart_rate >= 8'd10)  ? d2a((heart_rate % 8'd100) / 8'd10) : 8'h20;
    wire [7:0] hr_o = d2a(heart_rate % 8'd10);

    wire [7:0] tp_t = d2a((temperature / 8'd2) / 8'd10);
    wire [7:0] tp_o = d2a((temperature / 8'd2) % 8'd10);
    wire [7:0] tp_f = (temperature % 8'd2) ? "5" : "0";

    wire [7:0] cd_h = (cadence >= 8'd200) ? "2" : (cadence >= 8'd100) ? "1" : 8'h20;
    wire [7:0] cd_t = (cadence >= 8'd10)  ? d2a((cadence % 8'd100) / 8'd10) : 8'h20;
    wire [7:0] cd_o = d2a(cadence % 8'd10);

    wire [7:0] al_d = d2a({5'd0, activity_level});

    wire [7:0] sc_h = (health_score >= 8'd100) ? "1" : 8'h20;
    wire [7:0] sc_t = (health_score >= 8'd10)  ? d2a((health_score % 8'd100) / 8'd10) : 8'h20;
    wire [7:0] sc_o = d2a(health_score % 8'd10);

    wire [7:0] ta_d = (tach_count  > 8'd9) ? "9" : d2a(tach_count);
    wire [7:0] br_d = (brad_count  > 8'd9) ? "9" : d2a(brad_count);
    wire [7:0] fv_d = (fever_count > 8'd9) ? "9" : d2a(fever_count);

    // ---- status strings (6 chars each) ----
    wire [47:0] st_now;
    assign st_now = (health_status == 2'b01) ? "WARN!!" :
                    (health_status == 2'b10) ? "DANGER" : "NORMAL";

    wire [47:0] st_worst;
    assign st_worst = (worst_status == 2'b01) ? "WARN!!" :
                      (worst_status == 2'b10) ? "DANGER" : "NORMAL";

    // ---- font_rom (5x8, unchanged from working original) ----
    wire [39:0] font_byte = font_rom(char[(152 - num*8) +: 8]);

    // ---- Font ROM (5x8 ASCII, complete set from working original) ----
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

    // ---- main state machine (unchanged logic) ----
    always @(posedge clk or negedge rst or negedge en) begin
        if (!rst || !en) begin
            cnt_main <= 1'b0; cnt_init <= 1'b0; cnt_scan <= 1'b0; cnt_write <= 1'b0;
            y_p <= 1'b0; x_ph <= 1'b0; x_pl <= 1'b0;
            num <= 1'b0; char <= 1'b0; char_reg <= 1'b0;
            num_delay <= 16'd5; cnt_delay <= 1'b0; cnt <= 1'b0;
            oled_csn <= HIGH; oled_rst <= LOW; oled_dcn <= CMD;
            oled_clk <= HIGH; oled_dat <= LOW;
            state <= IDLE; state_back <= IDLE;
        end else if (sys_ce) begin
            case (state)
                IDLE: begin
                    cnt_main <= 1'b0; cnt_init <= 1'b0; cnt_scan <= 1'b0; cnt_write <= 1'b0;
                    y_p <= 1'b0; x_ph <= 1'b0; x_pl <= 1'b0;
                    num <= 1'b0; char <= 1'b0; char_reg <= 1'b0;
                    num_delay <= 16'd5; cnt_delay <= 1'b0; cnt <= 1'b0;
                    oled_csn <= HIGH; oled_rst <= HIGH; oled_dcn <= CMD;
                    oled_clk <= HIGH; oled_dat <= LOW;
                    state <= MAIN; state_back <= MAIN;
                end

                MAIN: begin
                    if (cnt_main >= 8) cnt_main <= 5'd1;
                    else cnt_main <= cnt_main + 1'b1;
                    // common page setup
                    x_ph <= 8'h10; x_pl <= 8'h00; num <= 5'd0; num_max <= 5'd16;
                    // ---- dispatch by screen + page ----
                    if (cnt_main == 5'd0) state <= INIT;
                    else case ({scr, cnt_main})
                        // 每行21字符: [0-1]=空格(不显示) [2-17]=16个显示字符 [18-20]=空格(不显示)
                        //========== SCREEN 0: STANDBY ==========
                        {3'd0,5'd1}: begin y_p<=8'hb0; char<={"  "," ","H","E","A","L","T","H"," ","M","O","N","I","T","O","R"," ","   "}; state<=SCAN; end
                        {3'd0,5'd2}: begin y_p<=8'hb1; char<={"  "," "," ","S","T","A","N","D","B","Y"," ","M","O","D","E"," ","   "}; state<=SCAN; end
                        {3'd0,5'd3}: begin y_p<=8'hb2; char<={"  ","P","R","E","S","S"," ","C","O","N","F","I","R","M"," "," "," ","   "}; state<=SCAN; end
                        {3'd0,5'd4}: begin y_p<=8'hb3; char<={"  "," ","v","1",".","0"," "," ","2","0","2","5",".","0","6"," ","   "}; state<=SCAN; end
                        //========== SCREEN 1: HEART RATE ==========
                        {3'd1,5'd1}: begin y_p<=8'hb0; char<={"  ","<","3"," ","H","E","A","R","T"," ","R","A","T","E"," "," "," ","   "}; state<=SCAN; end
                        {3'd1,5'd2}: begin y_p<=8'hb1; char<={"  "," "," "," ",hr_h,hr_t,hr_o," ","B","P","M"," "," "," "," "," ","   "}; state<=SCAN; end
                        //========== SCREEN 2: TEMPERATURE ==========
                        {3'd2,5'd1}: begin y_p<=8'hb0; char<={"  ","~"," ","T","E","M","P","E","R","A","T","U","R","E"," "," ","   "}; state<=SCAN; end
                        {3'd2,5'd2}: begin y_p<=8'hb1; char<={"  "," "," "," ",tp_t,tp_o,".",tp_f," ","C"," "," "," "," "," "," ","   "}; state<=SCAN; end
                        //========== SCREEN 3: CADENCE ==========
                        {3'd3,5'd1}: begin y_p<=8'hb0; char<={"  ","/"," ","C","A","D","E","N","C","E"," "," "," "," "," "," ","   "}; state<=SCAN; end
                        {3'd3,5'd2}: begin y_p<=8'hb1; char<={"  "," "," "," ",cd_h,cd_t,cd_o," ","S","P","M"," "," "," "," "," ","   "}; state<=SCAN; end
                        //========== SCREEN 4: ACTIVITY ==========
                        {3'd4,5'd1}: begin y_p<=8'hb0; char<={"  ","="," ","A","C","T","I","V","I","T","Y"," "," "," "," "," ","   "}; state<=SCAN; end
                        {3'd4,5'd2}: begin y_p<=8'hb1; char<={"  "," "," "," ","L","E","V","E","L"," ",al_d," "," "," "," "," ","   "}; state<=SCAN; end
                        //========== SCREEN 5: SUMMARY ==========
                        {3'd5,5'd1}: begin y_p<=8'hb0; char<={"  ","H","R",":",hr_h,hr_t,hr_o," ","T",":",tp_t,tp_o,".",tp_f,"C"," ","   "}; state<=SCAN; end
                        {3'd5,5'd2}: begin y_p<=8'hb1; char<={"  ","A","C","T",":",al_d," ","C","A","D",":",cd_h,cd_t,cd_o," "," ","   "}; state<=SCAN; end
                        {3'd5,5'd3}: begin y_p<=8'hb2; char<={"  ","S","T",":",st_now[47:40],st_now[39:32],st_now[31:24],st_now[23:16],st_now[15:8],st_now[7:0]," "," "," "," "," "," ","   "}; state<=SCAN; end
                        //========== SCREEN 6: HEALTH SCORE ==========
                        {3'd6,5'd1}: begin y_p<=8'hb0; char<={"  "," ","H","E","A","L","T","H"," ","S","C","O","R","E"," "," ","   "}; state<=SCAN; end
                        {3'd6,5'd2}: begin y_p<=8'hb1; char<={"  "," "," "," "," ",sc_h,sc_t,sc_o,"/","1","0","0"," "," "," "," ","   "}; state<=SCAN; end
                        {3'd6,5'd3}: begin y_p<=8'hb2; char<={"  ","T",":",ta_d," ","B",":",br_d," ","F",":",fv_d," "," "," "," ","   "}; state<=SCAN; end
                        {3'd6,5'd4}: begin y_p<=8'hb3; char<={"  ","S","T",":",st_worst[47:40],st_worst[39:32],st_worst[31:24],st_worst[23:16],st_worst[15:8],st_worst[7:0]," "," "," "," "," "," ","   "}; state<=SCAN; end
                        //========== BLANK / DEFAULT ==========
                        default: begin y_p <= 8'hb0 | {5'd0, cnt_main[2:0]}; char <= {"  ",{16{8'h20}},"   "}; state <= SCAN; end
                    endcase
                end

                INIT: begin
                    case (cnt_init)
                        5'd0: begin oled_rst <= LOW;  cnt_init <= cnt_init + 1'b1; end
                        5'd1: begin num_delay <= 16'd25000; state <= DELAY; state_back <= INIT; cnt_init <= cnt_init + 1'b1; end
                        5'd2: begin oled_rst <= HIGH; cnt_init <= cnt_init + 1'b1; end
                        5'd3: begin num_delay <= 16'd25000; state <= DELAY; state_back <= INIT; cnt_init <= cnt_init + 1'b1; end
                        5'd4: begin
                                if (cnt >= INIT_DEPTH) begin cnt <= 1'b0; cnt_init <= cnt_init + 1'b1; end
                                else begin
                                    cnt <= cnt + 1'b1; num_delay <= 16'd5;
                                    oled_dcn <= CMD; char_reg <= cmd[cnt];
                                    state <= WRITE; state_back <= INIT;
                                end
                               end
                        5'd5: begin cnt_init <= 1'b0; state <= MAIN; end
                        default: state <= IDLE;
                    endcase
                end

                SCAN: begin
                    if (cnt_scan == 5'd11) begin
                        if (num < num_max) cnt_scan <= 5'd3; else cnt_scan <= 5'd12;
                    end else if (cnt_scan == 5'd12) cnt_scan <= 1'b0;
                    else cnt_scan <= cnt_scan + 1'b1;
                    case (cnt_scan)
                        5'd 0: begin oled_dcn <= CMD;  char_reg <= y_p;  state <= WRITE; state_back <= SCAN; end
                        5'd 1: begin oled_dcn <= CMD;  char_reg <= x_pl; state <= WRITE; state_back <= SCAN; end
                        5'd 2: begin oled_dcn <= CMD;  char_reg <= x_ph; state <= WRITE; state_back <= SCAN; end
                        5'd 3: begin num <= num + 1'b1; end
                        5'd 4: begin oled_dcn <= DATA; char_reg <= 8'h00; state <= WRITE; state_back <= SCAN; end
                        5'd 5: begin oled_dcn <= DATA; char_reg <= 8'h00; state <= WRITE; state_back <= SCAN; end
                        5'd 6: begin oled_dcn <= DATA; char_reg <= 8'h00; state <= WRITE; state_back <= SCAN; end
                        5'd 7: begin oled_dcn <= DATA; char_reg <= font_byte[39:32]; state <= WRITE; state_back <= SCAN; end
                        5'd 8: begin oled_dcn <= DATA; char_reg <= font_byte[31:24]; state <= WRITE; state_back <= SCAN; end
                        5'd 9: begin oled_dcn <= DATA; char_reg <= font_byte[23:16]; state <= WRITE; state_back <= SCAN; end
                        5'd10: begin oled_dcn <= DATA; char_reg <= font_byte[15: 8]; state <= WRITE; state_back <= SCAN; end
                        5'd11: begin oled_dcn <= DATA; char_reg <= font_byte[ 7: 0]; state <= WRITE; state_back <= SCAN; end
                        5'd12: begin state <= MAIN; end
                        default: state <= IDLE;
                    endcase
                end

                WRITE: begin
                    if (cnt_write >= 5'd17) cnt_write <= 1'b0;
                    else cnt_write <= cnt_write + 1'b1;
                    case (cnt_write)
                        5'd 0: begin oled_csn <= LOW; end
                        5'd 1: begin oled_clk <= LOW;  oled_dat <= char_reg[7]; end
                        5'd 2: begin oled_clk <= HIGH; end
                        5'd 3: begin oled_clk <= LOW;  oled_dat <= char_reg[6]; end
                        5'd 4: begin oled_clk <= HIGH; end
                        5'd 5: begin oled_clk <= LOW;  oled_dat <= char_reg[5]; end
                        5'd 6: begin oled_clk <= HIGH; end
                        5'd 7: begin oled_clk <= LOW;  oled_dat <= char_reg[4]; end
                        5'd 8: begin oled_clk <= HIGH; end
                        5'd 9: begin oled_clk <= LOW;  oled_dat <= char_reg[3]; end
                        5'd10: begin oled_clk <= HIGH; end
                        5'd11: begin oled_clk <= LOW;  oled_dat <= char_reg[2]; end
                        5'd12: begin oled_clk <= HIGH; end
                        5'd13: begin oled_clk <= LOW;  oled_dat <= char_reg[1]; end
                        5'd14: begin oled_clk <= HIGH; end
                        5'd15: begin oled_clk <= LOW;  oled_dat <= char_reg[0]; end
                        5'd16: begin oled_clk <= HIGH; end
                        5'd17: begin oled_csn <= HIGH; state <= DELAY; end
                        default: state <= IDLE;
                    endcase
                end

                DELAY: begin
                    if (cnt_delay >= num_delay) begin
                        cnt_delay <= 16'd0; state <= state_back;
                    end else
                        cnt_delay <= cnt_delay + 1'b1;
                end

                default: state <= IDLE;
            endcase
        end
    end

    initial begin
        cnt_main=1'b0; cnt_init=1'b0; cnt_scan=1'b0; cnt_write=1'b0;
        y_p=1'b0; x_ph=1'b0; x_pl=1'b0; num=1'b0; char=1'b0; char_reg=1'b0;
        num_delay=16'd5; cnt_delay=1'b0; cnt=1'b0;
        oled_csn=HIGH; oled_rst=LOW; oled_dcn=CMD; oled_clk=HIGH; oled_dat=LOW;
        state=IDLE; state_back=IDLE;
        cmd[ 0]=8'hae; cmd[ 1]=8'h00; cmd[ 2]=8'h10; cmd[ 3]=8'h00;
        cmd[ 4]=8'hb0; cmd[ 5]=8'h81; cmd[ 6]=8'hff; cmd[ 7]=8'ha1;
        cmd[ 8]=8'ha6; cmd[ 9]=8'ha8; cmd[10]=8'h3f; cmd[11]=8'hc8;
        cmd[12]=8'hd3; cmd[13]=8'h00; cmd[14]=8'hd5; cmd[15]=8'h80;
        cmd[16]=8'hd9; cmd[17]=8'hf1; cmd[18]=8'hda; cmd[19]=8'h12;
        cmd[20]=8'hdb; cmd[21]=8'h30; cmd[22]=8'h8d; cmd[23]=8'h14;
        cmd[24]=8'haf;
    end
endmodule
