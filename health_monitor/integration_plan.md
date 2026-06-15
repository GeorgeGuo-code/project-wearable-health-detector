# 整合方案：心率 + 加速度 + 温度 → top_spi

> 目标：把三条已验证的传感器链路（MAX30102 / MPU6050 / DS18B20）汇入现成的
> `top_spi.v`，再加一层 `top_integration.v` 串接。完成后端到端
> `传感器 → 处理 → 评估 → OLED` 跑通，Vivado 综合 + 上板可见。

---

# 整合方案：心率 + 加速度 + 温度 → top_spi + Flash 视图

> 目标：把三条已验证的传感器链路（MAX30102 / MPU6050 / DS18B20）汇入现成的
> `top_spi.v`，再加一层 `top_integration.v` 串接。完成后端到端
> `传感器 → 处理 → 评估 → OLED` 跑通，Vivado 综合 + 上板可见。
>
> 另：Flash 驱动另行编写。本 plan **只预留 1 bit `flash_view_en` 切换信号 + 3 个
> session 缓存寄存器**（步数 / 平均步频 / 平均心率），供后续 flash 读写调用。
> Flash SPI 接口本身（CS/SCK/MOSI/MISO）也在顶层预留。

---

### 1.1 各传感器已有顶层（已独立验证可读数据）

| 传感器 | 顶层文件 | 输出（关键位） | 备注 |
|--------|----------|----------------|------|
| **MAX30102** | `HR_Driver/src/max30102_top.v` | `sensor_data[7:0]`, `data_valid`, `init_done` | 仅暴露 8bit IR/RED 高字节，**未算心率** |
| **MAX30102 + 心率** | `HR_DataProcessor/src/hr_test_top.v` | `hr_bpm_bin[7:0]`, `hr_valid`, `hr_locked` + UART | 内含 `hr_maxim`，已能算 BPM |
| **MPU6050（带算法）** | `Accel_DataProcessor/src/top.v` | `cadence[15:0]`, `step_count[15:0]` + BD UART | 走 MicroBlaze BD，验证过 ✅ step_count 已扩到 16 bit |
| **MPU6050（裸）** | `Accel_Driver/src/top.v` | `acc_h/l` + 6 轴字节 | 纯 RTL |
| **DS18B20** | `Temp_Driver/src/ds18b20_driver.v` | `temperature[15:0]` (12bit, 0.0625°C/LSB, signed), `data_valid` | 1Hz 异步 |
| **OLED SPI** | `display/cs13.v` | `oled_csn/rst/dcn/clk/dat` | 内置渲染 + 字库 |

### 1.2 `top_spi.v` 期望的输入（5 个数据信号）

```verilog
input  wire        data_valid;        // 统一 1Hz/异步脉冲
input  wire [7:0]  heart_rate;        // BPM (30-220)
input  wire [7:0]  temperature;       // °C × 2  (37.5°C → 75)
input  wire [2:0]  activity_level;    // 0-4
input  wire [7:0]  cadence;           // SPM, 0-240
```

### 1.3 `top_spi.v` 已经做好（不动它）

- `top_health_monitor`：sm_main + evaluator + scorer
- `display_top_spi`：cs13 OLED + LED + 蜂鸣器
- 全部走 SPI OLED

---

## 2. 顶层架构

```
                          ┌─────────────────────────────────────────────┐
                          │           top_integration.v                │
                          │   (新增, 把三路传感器 + 转换层串起来)         │
                          └─────────────────────────────────────────────┘
                                  │
   ┌────────────────┐             │             ┌────────────────┐
   │ HR_Processor   │──heart_rate─┤             │ Display Chain  │
   │ max30102_top   │──hr_valid──┐│             │ top_spi        │
   │ + hr_maxim     │            ││             │   ↳ top_health_│
   │ I2C #1         │            ││             │     monitor    │
   │ scl1/sda1      │            ││             │   ↳ display_   │
   └────────────────┘            ││             │     top_spi    │
                                ││             │ SPI OLED       │
   ┌────────────────┐            ││             │ + LED × 3      │
   │ Accel          │──cadence───┤│             │ + buzzer       │
   │ DataProcessor  │──activity──┘│             └────────────────┘
   │ (MicroBlaze BD)│             │
   │ I2C #2         │             │
   │ scl2/sda2      │             │
   └────────────────┘             │
                                  │
   ┌────────────────┐             │
   │ DS18B20 + temp │──temp_fmt───│  ← 把 12bit 0.0625°C 换成 °C×2
   │ One-Wire       │             │
   │ dq             │             │
   └────────────────┘             │
                                  │
                                data_valid (任一源脉冲 OR 起来)
```

---

## 3. 顶层 IO 清单（Vivado XDC 需要约束的引脚）

| 名称 | 方向 | 位宽 | 用途 |
|------|------|------|------|
| `clk_100MHz` | in | 1 | EGO1 100MHz 主时钟 (P17) |
| `reset_btn` | in | 1 | EGO1 复位按钮 (P4)，低有效 |
| `btn_mode`, `btn_confirm` | in | 2 | EGO1 拨码/按键（按下=高，需要取反） |
| `scl1`, `sda1` | out/inout | 1+1 | **I2C #1** → MAX30102 |
| `scl2`, `sda2` | out/inout | 1+1 | **I2C #2** → MPU6050 |
| `dq_temp` | inout | 1 | One-Wire → DS18B20（外加上拉 4.7kΩ） |
| `oled_csn/rst/dcn/clk/dat` | out | 5 | **SPI OLED** (cs13) |
| `status_led[2:0]` | out | 3 | 绿/黄/红 |
| `buzzer` | out | 1 | 蜂鸣器 PWM |
| `flash_view_en` | in | 1 | **Flash 视图选择**（高=OLED 切到 FLASH 页显示 3 个缓存值，低=正常轮播） |
| `flash_csn`, `flash_sck`, `flash_mosi`, `flash_miso` | out/inout/inout/in | 4 | **Flash SPI 预留**（CSN/CLK/MOSI 输出，MISO 输入；本期只占位，flash 驱动未接） |
| `session_save_req` | out | 1 | **会话结束脉冲**（work_en 下降沿触发一拍，flash 驱动可据此发起保存） |

**给 Flash 驱动的"软接口"（输出只读，flash 驱动可随时采样）：**

| 信号 | 位宽 | 说明 |
|------|------|------|
| `cached_step_count` | 16 | 会话结束时的累计步数（来自 MPU6050 BD） |
| `cached_avg_cadence` | 8 | 会话期间 cadence 的算术平均（spm） |
| `cached_avg_hr` | 8 | 会话期间 heart_rate 的算术平均（bpm） |

> **IO 总计：13（业务）+ 1（flash_view_en）+ 4（Flash SPI 预留）+ 1（save_req）= 19 个 IO**。
> 19 / 32 ≈ 60% 占用，FPGA 资源宽裕。

---

## 4. 各模块在 `top_integration.v` 里的实例化

### 4.1 MAX30102 + 心率（I2C #1）

```verilog
wire [7:0]  hr_bpm_bin;
wire        hr_valid, hr_locked;

// 只暴露 I2C + 18bit IR，避免重复实现 I2C
max30102_driver u_max_driver (
    .clk       (clk_100MHz),
    .rst_n     (~reset_btn),
    .start     (1'b1),                  // 上电即开始
    .scl       (scl1),
    .sda       (sda1),
    .ir_data   (ir_raw),
    .red_data  (red_raw),               // 不连
    .data_valid(data_valid_max),
    .init_done (init_done_max),
    .sda_link  (),
    .sda_r     (),
    .state     ()
);

hr_maxim u_hr_maxim (
    .clk           (clk_100MHz),
    .rst_n         (~reset_btn),
    .in_valid      (data_valid_max),     // 100Hz 脉冲
    .ir_ac         (ir_raw),             // 18bit IR
    .hr_bpm        (hr_bpm_bin),         // 8bit BPM
    .hr_valid      (hr_valid),
    .hr_locked     (hr_locked),
    .hamm_out      (),
    .threshold_out (),
    .dec_valid     ()
);
```

**注意**：`max30102_driver.v` 的 I2C 时序要求 100kHz 模式（默认），与 MPU6050 兼容。
两路 I2C 物理完全隔离，不需要总线仲裁。

### 4.2 MPU6050 + 步频（I2C #2，复用 `Accel_DataProcessor`）

`Accel_DataProcessor/src/top.v` 已经把 BD 封好了，直接套用：

```verilog
wire [15:0] mpu_cadence;
wire [15:0] mpu_step_count;        // 16 bit ✅
wire        mpu_data_valid_bd;    // BD 侧的 data_valid
wire        mpu_init_done_bd;

accel_dataprocessor_top u_accel (   // 见下方"包装说明"
    .clk_100MHz     (clk_100MHz),
    .reset_btn      (reset_btn),
    .read_en_switch (1'b1),
    .scl            (scl2),
    .sda            (sda2),
    .bd_uart_txd    (),              // 调试 UART，可不接
    .cadence        (mpu_cadence),
    .step_count     (mpu_step_count)
);
```

**包装说明**：当前 `Accel_DataProcessor/src/top.v` 是顶层，不能直接被
`top_integration` 引用。建议做法：
- **方案 A（推荐）**：把 `top.v` 改名为 `mpu6050_processor.v`，模块名也改成
  `mpu6050_processor`（保持接口不变），让 `top_integration` 实例化它。
- **方案 B**：保留 `top.v` 作为 EGO1 单独验证顶；新写一个
  `mpu6050_processor.sv` 包一层供集成用。

### 4.3 DS18B20 + 温度格式转换（One-Wire）

```verilog
wire [15:0] temp_raw;    // 12bit, 0.0625°C/LSB, signed
wire        temp_valid;
wire        temp_dq_out, temp_dq_oe;

ds18b20_driver u_temp (
    .clk        (clk_100MHz),
    .rst_n      (~reset_btn),
    .start      (1'b1),
    .dq_in      (dq_temp_in),
    .temperature(temp_raw),
    .data_valid (temp_valid),
    .error      (),
    .dq_out     (temp_dq_out),
    .dq_oe      (temp_dq_oe)
);

// IOBUF 双向
assign dq_temp     = temp_dq_oe ? temp_dq_out : 1'bz;
assign dq_temp_in  = dq_temp;

// 12bit signed 0.0625°C/LSB  →  8bit °C×2
//   temp_raw = 0x0191 (= 401) → 25.0625°C
//   25.0625 × 2 = 50.125  →  取整 50 →  8'd50
//   -0.5°C: 0xFF8 → -8/16 = -0.5 →  ×2 = -1 →  8'hFF (有符号 -1)
wire [15:0] temp_cdeg2 = $signed(temp_raw) >>> 3;   // /8 ≈ ×0.125, 再 ×2 直接 shift
// 上面算式: temp_raw*0.0625/0.5 = temp_raw/8 → 单位 0.5°C → ×2 是显示用
// 简化: temp_cdeg2 = temp_raw[15:0] / 8 (signed) → 物理量是 0.5°C/LSB
// cs13 期望 °C×2, 即 1 LSB = 0.5°C → 上面算式恰好一致
wire [7:0]  temperature = temp_cdeg2[7:0];   // Q4.3 截断到 8bit
```

**DS18B20 数据格式**（数据手册）：12 bit 补码，LSB = 0.0625°C
- 0x0000 → 0°C
- 0x0191 (401) → +25.0625°C
- 0xFF8 (-8) → -0.5°C
- 转换公式：°C = raw × 0.0625
- cs13 期望单位：0.5°C/LSB（°C×2），所以 `temperature = raw / 8`（有符号右移 3）

**✅ 温度处理决策**：本期采用 **直接输出原始温度**（`raw / 8` 转为 °C×2 格式），
**不实现**详细设计方案 §3.2.7 里的"基线差值法"（不建立 3s 静息基线、不算 ΔT、
不做防突变检测）。`health_status_evaluator` 用绝对阈值（>38°C→DANGER），
这个组合够用。基线差值法在评估精度收益不大（DS18B20 接触式测量本身误差
±2°C），留待后续版本。

### 4.4 活动等级映射

`activity_level` 是 3bit (0-4)，由 cadence 决定：

```verilog
// 0=静坐, 1=轻度, 2=中度, 3=剧烈, 4=极剧烈
// 阈值与详细设计方案 §3.2.4 一致
function [2:0] cadence_to_level;
    input [7:0] cad;
    begin
        if (cad < 8'd60)        cadence_to_level = 3'd0;  // 静坐
        else if (cad < 8'd100)  cadence_to_level = 3'd1;  // 轻度
        else if (cad < 8'd140)  cadence_to_level = 3'd2;  // 中度
        else if (cad < 8'd180)  cadence_to_level = 3'd3;  // 剧烈
        else                    cadence_to_level = 3'd4;  // 极剧烈
    end
endfunction

wire [2:0] activity_level = cadence_to_level(mpu_cadence[7:0]);
```

### 4.5 data_valid 汇合

`top_spi` 期望一个统一的数据有效脉冲。三个源频率不同：
- 心率 `hr_valid`：~7.7s 一次（locked 之后）
- 步频 `mpu_data_valid_bd`：~每步一次（100Hz 噪声触发）—— **不能用这个**
- 温度 `temp_valid`：1Hz

**建议**：用 1Hz 温度有效脉冲作为主 `data_valid`（保证 evaluator/scorer
每秒更新一次）。心率、cadence 是持续刷新（`hr_bpm_bin`、
`mpu_cadence` 是稳态寄存器），不需要每次都脉冲。

```verilog
assign data_valid = temp_valid;   // 1Hz 温度脉冲作为统一心跳
```

`hr_bpm_bin` 和 `mpu_cadence[7:0]` 是持续更新的稳态值，evaluator
会读到最新的数。

### 4.6 `session_stats` —— 会话级缓存（Flash 写库 + Flash 视图的数据源）

`work_en` 拉高期间持续累加，`work_en` 下降沿把三个汇总值**锁存**到
`cached_*` 寄存器。整个会话期间这三个值保持不变，供 flash 驱动读取
（写库）或 OLED flash 视图显示。

```verilog
// session_stats.sv — 放在 health_monitor/ 下
module session_stats (
    input  wire        clk, rst_n,
    input  wire        work_en,           // from sm_main
    input  wire        data_valid,        // 1Hz temp_valid

    // 实时数据 (来自传感器)
    input  wire [7:0]  heart_rate,        // bpm
    input  wire [7:0]  cadence,           // spm, 8bit
    input  wire [15:0] step_count,        // 累计步数, 16bit

    // 锁存的会话级缓存 (供 flash 写库 / OLED flash 视图使用)
    output reg  [15:0] cached_step_count,
    output reg  [7:0]  cached_avg_cadence,
    output reg  [7:0]  cached_avg_hr,

    // 握手: 会话结束一拍脉冲 (给 flash 驱动)
    output wire        session_end
);
    // 累加器
    reg [31:0] hr_sum, cad_sum;
    reg [23:0] hr_cnt, cad_cnt;
    reg        work_en_d1;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) work_en_d1 <= 1'b0;
        else        work_en_d1 <= work_en;

    assign session_end = !work_en && work_en_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hr_sum <= 0; hr_cnt <= 0;
            cad_sum <= 0; cad_cnt <= 0;
            cached_step_count  <= 16'd0;
            cached_avg_cadence <= 8'd0;
            cached_avg_hr      <= 8'd0;
        end else if (work_en && data_valid) begin
            // 累加求平均 (cnt 上限约 16M, 跑满 1Hz 也能跑 194 天)
            hr_sum  <= hr_sum  + {24'd0, heart_rate};
            hr_cnt  <= hr_cnt  + 24'd1;
            cad_sum <= cad_sum + {24'd0, cadence};
            cad_cnt <= cad_cnt + 24'd1;
        end else if (session_end) begin
            // 锁存: 步数直接取累计, 心率/步频取算术平均
            cached_step_count  <= step_count;
            cached_avg_hr      <= (hr_cnt  != 0) ? hr_sum [31:24]  // 取高 8bit = sum/cnt 的近似
                                                 : 8'd0;            // 简化: 1Hz × 200bpm = 200/255 误差可接受
            cached_avg_cadence <= (cad_cnt != 0) ? cad_sum[31:24]
                                                 : 8'd0;
            // 复位累加器
            hr_sum  <= 0; hr_cnt  <= 0;
            cad_sum <= 0; cad_cnt <= 0;
        end
    end
endmodule
```

**关于 `sum >> 24` 作为平均值**：

- `hr_sum` 是 `cnt × avg` 的累积，每秒 +1 次。
- 会话跑 1 小时：cnt = 3600，sum ≈ 3600 × 80 = 288000（< 2^19）
- 取 `sum >> 24` 不对（太小）。
- **正确做法**：`avg = sum / cnt`，需要除法器。

**修正**（更稳妥）：

```verilog
// 用 IP Divider Generator 或手写 32/24 定点除法
// 例: 用 Vivado IP: Divider Generator, latency 32, 几乎不占资源
// 或者: 用组合逻辑除法器 (32/24 → 8, 1 拍, LUT 较多)

wire [7:0] hr_avg_w      = (hr_cnt  != 0) ? (hr_sum  / {8'd0, hr_cnt})  : 8'd0;
wire [7:0] cad_avg_w     = (cad_cnt != 0) ? (cad_sum / {8'd0, cad_cnt})  : 8'd0;

always @(posedge clk or negedge rst_n) begin
    ...
    end else if (session_end) begin
        cached_step_count  <= step_count;
        cached_avg_hr      <= hr_avg_w;
        cached_avg_cadence <= cad_avg_w;
        ...
    end
end
```

> 32/24 定点除法器用 Vivado DivGen IP 即可（1 个 IP，~150 LUT，~30 级流水线），
> 也可手写一个 long-division（~300 LUT，组合）。这里**只是缓存**，
> 不在 100Hz 路径上，资源压力小。

---

## 5. 顶层信号映射总表

| `top_spi` 输入 | 来源 |
|----------------|------|
| `clk` | `clk_100MHz` |
| `rst_n` | `~reset_btn` |
| `btn_mode` | `~btn_mode_raw` (取反) |
| `btn_confirm` | `~btn_confirm_raw` |
| `data_valid` | `temp_valid` (1Hz 温度脉冲) |
| `heart_rate[7:0]` | `hr_bpm_bin` (来自 `hr_maxim`) |
| `temperature[7:0]` | `temp_raw >> 3` (12bit→°C×2) |
| `activity_level[2:0]` | `cadence_to_level(mpu_cadence[7:0])` |
| `cadence[7:0]` | `mpu_cadence[7:0]` |

**`top_integration` 顶层额外输出（flash 驱动用）：**

| 顶层输出 | 来源 | 用途 |
|----------|------|------|
| `cached_step_count[15:0]` | `session_stats.cached_step_count` | flash 写库使用 |
| `cached_avg_cadence[7:0]` | `session_stats.cached_avg_cadence` | flash 写库使用 |
| `cached_avg_hr[7:0]` | `session_stats.cached_avg_hr` | flash 写库使用 |
| `session_save_req` | `session_stats.session_end` | flash 驱动触发保存 |

**`top_integration` 顶层额外输入（flash 驱动 + 用户选择）：**

| 顶层输入 | 用途 |
|----------|------|
| `flash_view_en` | 拉高时 OLED 切到 FLASH 视图（3 个缓存值） |
| `flash_csn / flash_sck / flash_mosi / flash_miso` | Flash SPI（本期预留，驱动未接时浮空/低） |

---

## 6. 实施步骤（建议 4 个 PR / 4 步走）

### Step 1：把 `Accel_DataProcessor/src/top.v` 改成可被引用的封装

- 把 `module top` 改名为 `mpu6050_processor`（接口不动）
- 或者新写一个 `Accel_DataProcessor/src/mpu6050_processor.sv` 把 `top.v` 包一层
- 验证：原 EGO1 单独烧写还能用

### Step 2：编写 `health_monitor/top_integration.v`

内容：
- 实例化 `max30102_driver` + `hr_maxim`
- 实例化 `mpu6050_processor`（包好的）
- 实例化 `ds18b20_driver` + 温度格式转换
- `activity_level` 映射函数
- 实例化 `session_stats`（§4.6）
- 接到 `top_spi`（不动 `top_spi`）
- 引出 `flash_*` 接口和 `cached_*` 给外部 flash 驱动

文件包含列表：
```verilog
// IP 库
max30102_driver    // 来自 HR_Driver/src
hr_maxim           // 来自 HR_DataProcessor/src
mpu6050_processor  // 来自 Accel_DataProcessor/src (新建的封装)
ds18b20_driver     // 来自 Temp_Driver/src
// 业务层
session_stats      // 来自 health_monitor/session_stats.sv (新建)
top_spi            // 来自 health_monitor/top_spi.sv
```

### Step 2.5：扩展 `cs13.v` / `display_top_spi.sv` 支持 FLASH 视图

`cs13.v` 当前 7 个 scr（0=STANDBY, 1-6=WORK）。FLASH 视图需要一个独立的页，
**不影响现有 scr 编号**。

**接口扩展**：

```verilog
// cs13.v 新增输入
input wire        flash_view_en,    // 1=FLASH 视图, 0=原 scr 逻辑
input wire [15:0] flash_step,       // 缓存的累计步数
input wire [7:0]  flash_avg_cad,    // 缓存的平均步频
input wire [7:0]  flash_avg_hr,     // 缓存的平均心率
```

**显示逻辑**：

```verilog
// 在 cs13 的 scr 计算处:
wire [3:0] scr;
assign scr = flash_view_en ? 4'd7 : (work_en ? (display_mode + 3'd1) : 3'd0);
//                              ^^^^^^^^
//                              FLASH 视图, 优先级最高
```

**scr=7 FLASH 视图 4 行布局**（每行 21 字符，2 字符空 + 16 显示 + 3 空）：

```
行 0:  "  * FLASH STATS  "
行 1:  "  STEP=     1234  "   ← 4 位十进制 (max 9999 步)
行 2:  "  AVG_CAD=   85  "   ← 2 位十进制 (spm)
行 3:  "  AVG_HR =   72  "   ← 2 位十进制 (bpm)
```

> 步数显示 4 位（max 9999）能覆盖 ~1.5 小时慢跑 @ 100 spm。如果想存
> 5+ 小时会话，需要扩展到 5 位或滚动显示。建议**先 4 位**，flash
> 库自己支持 16 bit，写 OLED 时只显示低 4 位。

**新增 ASCII 字符需求**（检查现有 font_rom）：`F L A S H S T E P = A V G C D H _`
几乎都在 ASCII 0x41-0x5F 范围，现有 font_rom 都覆盖。无需扩展字库。

**`display_top_spi.sv` 修改**：

```verilog
// 新增输入
input  wire        flash_view_en,
input  wire [15:0] flash_step,
input  wire [7:0]  flash_avg_cad,
input  wire [7:0]  flash_avg_hr,

// 透传给 cs13
cs13 u_cs13 (
    ...
    .flash_view_en(flash_view_en),
    .flash_step   (flash_step),
    .flash_avg_cad(flash_avg_cad),
    .flash_avg_hr (flash_avg_hr),
    ...
);
```

**`top_spi.sv` 修改**：

```verilog
// 新增输入
input  wire        flash_view_en,
input  wire [15:0] cached_step_count,
input  wire [7:0]  cached_avg_cadence,
input  wire [7:0]  cached_avg_hr,

// 透传给 display_top_spi
display_top_spi u_display (
    ...
    .flash_view_en    (flash_view_en),
    .flash_step       (cached_step_count),
    .flash_avg_cad    (cached_avg_cadence),
    .flash_avg_hr     (cached_avg_hr),
    ...
);
```

### Step 3：Vivado 工程设置

- 添加源文件路径：`HR_Driver/src`, `HR_DataProcessor/src`,
  `Accel_DataProcessor/src`, `Temp_Driver/src`, `health_monitor/`
- 把 `top_integration` 设为顶层
- 配 XDC：19 个 IO（见 §3）

### Step 4：联调

- 上电：3 颗 LED 全亮 → OLED 显 `STANDBY` → 按 `btn_confirm` 进 `WORK`
- 静止：心率 ~70-80, 温度 ~36.5, cadence=0, 活动=0, 状态=绿
- 走两步：cadence 跳到 60-100, 活动=1, 状态保持绿
- 走快：cadence 140+, 活动=3, 状态可能转黄
- 拉高 `flash_view_en`：OLED 切到 FLASH 视图，显示 3 个缓存值（初始 0）
- 关闭 `flash_view_en`：回到原显示模式
- 按 `btn_confirm` 退到 STANDBY：`session_end` 触发一拍脉冲，flash 驱动可据此保存
- UART 验证（可选）：BD 端 `bd_uart_txd` 接 USB-UART，看 MicroBlaze 打印的
  `STEP spm=...`，与 OLED 显示对照

---

## 7. 待确认事项

1. **MPU6050 BD 单独烧写 vs 集成后跑** 的资源/LUT/BRAM 差异？
   MicroBlaze + 6 AXI GPIO 约 ~3000 LUT，加上 BD 内部还要 1-2 个 BRAM。
   集成后 `health_score` 这类 LUT 会有冲突吗？→ 上板跑一次才知道。

2. **DS18B20 上拉电阻** 4.7kΩ 必须外接，不能省。

3. **心率/温度/活动 数据有效脉冲要不要分开**？
   现在的 `data_valid = temp_valid` (1Hz) 是最简方案，evaluator 每秒
   评估一次。`scorer` 也按这个 1Hz 心跳累加。要不要做更高频评估？
   → 详细设计方案说 100Hz 采样，但 evaluator 本来就是组合逻辑，1Hz
   也够用。

4. **MicroBlaze C 代码改动**：`main.c` 当前用 UART 打印 `cadence`。
   集成后还能正常打印吗？BD UART TX 还是引出来接到 EGO1 的 T4 即可。

5. **EGO1 引脚分配**：最终需要一份 XDC 约束文件，把 §3 的 19 个 IO
   分配到 EGO1 实际可用的引脚（注意 scl2/sda2 与 scl1/sda1 不能
   共用同一 PMOD 上拉电阻，最好分两个 PMOD）。

6. **`flash_view_en` 怎么触发**？本期只暴露信号，不写驱动。常见做法：
   - 第三个按键（EGO1 还有拨码开关可用）
   - 长按 `btn_mode` ≥ 2s 切换
   - flash 驱动自己拉高（"正在读 flash，OLED 显示一下缓存"）
   - UART 命令（MicroBlaze C 收到指令后置位）
   建议**先用拨码开关**验证通路，后面再优化。

7. **MPU6050 `step_count` 扩到 16 bit** 后，BD 需要重新生成（AXI GPIO 位宽
   从 4 改到 16）。`main.c` 内部已经 16 bit，只是 GPIO 输出口限制。
   改 wrapper 时记得同步改 `gpio3_o` / `gpio4_o` 的 `tri_o` 分配。

8. **`session_stats` 用 `sum / cnt` 还是 `sum >> 24`**？
   上面 §4.6 给了两个方案。1Hz × 24 小时 × 200bpm ≈ 17M，cnt 用 24 bit
   够，但 sum 用 32 bit 可能溢出（24h × 200 ≈ 17M，sum 17M，cnt 86400，
   17M / 86K = 200，整数除法 OK）。建议**用 32/24 定点除法器**（Vivado IP），
   ~150 LUT。

---

## 8. 不在本次集成范围内

- Flash 驱动本体（SPI 控制器、擦写协议、坏块管理）—— 你正在写
- 锂电池充放电管理 —— 详细设计方案 §3.5，留待 PCB 阶段
- 蓝牙 / 扩展功能 —— 详细设计方案 §六

本次目标：
1. **传感器数据进 → top_spi 出 OLED 显示**（主链路）
2. **3 个会话级缓存寄存器**（供 flash 写库使用）
3. **FLASH 视图 OLED 页 + 1 bit `flash_view_en` 切换**（供 flash 读出时显示）
