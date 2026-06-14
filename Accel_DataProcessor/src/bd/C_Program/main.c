/*
 * main.c - MPU6050 运动加速度模长 + 步频检测 (全 C 实现)
 *
 * 版本: v2.1 (debug)
 *   备份: main.c.v2_stuck_at_init  ← 卡在 init, 待修复
 *   状态: 串口能打印启动信息, 然后卡在 "Waiting for MPU6050 init..."
 *         说明 iic_mpu6050.v 一直没把 init_done 拉高
 *   Debug: 加了等待循环里的状态打印, 重新烧完看 st 值
 *
 * v2.0 (2026/06/14)
 *   - step_detector 全部 C 实现 (从 step_detector.v 移植)
 *   - UART 输出 cadence / step_count / conf
 *   - GPIO_3 回写 cadence (低 16) / step_count (高 16)
 *
 * 算法:
 *   1) 字节拼装 → 16-bit signed
 *   2) 单位换算 (0.01g)
 *   3) 模长 sqrt(ax²+ay²+az²)
 *   4) 重力基线追踪 (16 点 MA, Q16 LPF, τ≈2.5s)
 *   5) acc_mag = raw_mag - gravity_baseline
 *   6) 步频检测: 4 态 FSM (TRACK/RISE/FALL/REFRACT)
 *        - 16 点 MA 动态基线
 *        - 阈值 MA + 0.06g
 *        - 2 个下降样本确认峰
 *        - 25 拍 (250ms) 不应期
 *        - 最近 8 步算 cadence (spm)
 *   7) 信号强度 confidence (peak - ma) × 4, 8-bit 饱和
 *   8) 看门狗: 200 拍 (2s) 无步 → cadence/confidence = 0
 *
 * UART 输出 (9600 baud, BD uart_rtl_0):
 *   10 Hz:  MOT mag=<0.01g>
 *   1 Hz:   STEP spm=<cadence> conf=<conf> cnt=<step_count>
 *   步事件: STEP! t=<sample> (限流 5 Hz 防止 UART 堵死)
 *
 * 数据流:
 *   MPU6050 → iic_mpu6050.v → BD GPIO_0/1/2 → MicroBlaze (本文件)
 *                                                   ↓
 *                                            xil_printf → BD UART → T4
 */

#include "xparameters.h"
#include "xgpio.h"
#include "xil_printf.h"
#include "sleep.h"
#include <stdint.h>

/* ====================================================================
 * GPIO 输入 (收 MPU6050 数据) + 1 路输出 (回写 cadence / step_count)
 * ==================================================================== */
XGpio GpioAccLow;     // GPIO_0 Ch1: acc_x_h, acc_x_l, acc_y_h, acc_y_l
XGpio GpioAccHigh;    // GPIO_0 Ch2: acc_z_h, acc_z_l, gyro_x_h, gyro_x_l
XGpio GpioGyro;       // GPIO_1:      gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l
XGpio GpioStatus;     // GPIO_2:      data_valid[0], init_done[1]
XGpio GpioOutMag;     // GPIO_3 out:  step_count[31:16], cadence[15:0]

/* ====================================================================
 * MPU6050 解析常量
 * ==================================================================== */
#define RAW2CG              82      // 1g = 8192 LSB → 0.01g
#define GRAVITY_LP_SHIFT    8       // Q16 LPF, /256 per sample
#define GRAVITY_INIT        100     // 初始 1g = 100 (0.01g)

/* ====================================================================
 * 步频检测常量 (与原 step_detector.v 参数一致)
 * ==================================================================== */
#define SAMPLE_PERIOD_MS    10      // 100 Hz
#define REFRACTORY_SAMP     25      // 250 ms 不应期 → 上限 240 spm
#define MA_WINDOW           16      // 160 ms 移动平均
#define THRESH_OFFSET       6       // 0.06 g 高于 MA
#define FALL_DELTA          4       // 0.04 g 下降触发 FALL
#define PEAK_HOLD           2       // 抗抖: 2 个下降样本确认
#define CADENCE_AVG_N       8       // 步频平滑窗口
#define SAT_CEIL            240     // 步频上饱和
#define STEP_GAP_RESET      200     // 2 s 无步 → 归零

/* ====================================================================
 * 重力基线追踪状态 (Q16 定点)
 * ==================================================================== */
static uint32_t gravity_est_q16 = (uint32_t)GRAVITY_INIT << 16;

/* ====================================================================
 * 步频检测 FSM 状态
 * ==================================================================== */
typedef enum { ST_TRACK = 0, ST_RISE = 1, ST_FALL = 2, ST_REFRACT = 3 } fsm_state_t;

static fsm_state_t step_state = ST_TRACK;

// Moving average (16 槽环形)
static uint16_t ma_buf[16];
static uint8_t  ma_ptr      = 0;
static uint32_t ma_sum      = 0;
static uint8_t  ma_full     = 0;
static uint16_t ma_val      = 0;
static uint16_t ma_at_peak  = 0;

// 峰值检测
static uint16_t local_max   = 0;
static uint16_t peak_val    = 0;
static uint8_t  fall_cnt    = 0;

// 不应期
static uint8_t  refr_cnt    = 0;

// 步时间戳环形缓冲 (24-bit sample_cnt / slot)
static uint32_t step_time_buf[8];
static uint8_t  step_time_ptr = 0;
static uint8_t  step_time_n   = 0;

// 样本计数 (24-bit, 用 uint32 简化)
static uint32_t sample_cnt = 0;

// 信号强度
static uint8_t  conf_decay_cnt = 0;
static uint8_t  confidence     = 0;

// 步间隔看门狗
static uint8_t  step_gap_cnt   = 0;

// 步频 / 步数
static uint16_t cadence    = 0;
static uint16_t step_count = 0;     // 16-bit, max 65535 步
                                    //   跑 30 min @ 170 spm = 5100 步, 充裕
static uint8_t  step_event = 0;     // 单拍脉冲

// 步事件打印节流 (5 Hz 上限)
static uint32_t last_step_print_sample = 0;
#define STEP_PRINT_MIN_INTERVAL  20   // 20 samples = 200 ms

/* ====================================================================
 * 整数 sqrt (牛顿法)
 * ==================================================================== */
static uint16_t isqrt(uint32_t x) {
    if (x == 0) return 0;
    uint32_t r = x;
    uint32_t b = 1u << 30;
    uint32_t res = 0;
    while (b > r) b >>= 2;
    while (b != 0) {
        if (r >= res + b) {
            r -= res + b;
            res = (res >> 1) + b;
        } else {
            res >>= 1;
        }
        b >>= 2;
    }
    return (uint16_t)res;
}

static inline int16_t pack16(uint8_t h, uint8_t l) {
    return (int16_t)((uint16_t)h << 8 | l);
}

/* ====================================================================
 * 步频检测单拍 (100 Hz 调用一次, 在 data_valid 上升沿)
 * 与原 Verilog step_detector.v 算法一致, 行为对等
 * ==================================================================== */
static void step_detector_tick(uint16_t acc_mag) {
    sample_cnt++;

    // 默认: 每拍 step_gap_cnt++, 确认步时清零
    if (step_gap_cnt < 0xFF) step_gap_cnt++;

    /* ---------- 1) 移动平均 (16 槽) ---------- */
    if (ma_full) {
        ma_sum = ma_sum - ma_buf[ma_ptr] + acc_mag;
        ma_val = (uint16_t)(ma_sum >> 4);
    } else {
        ma_sum = ma_sum + acc_mag;
        if (ma_ptr == MA_WINDOW - 1) {
            // 第 16 个样本, 升级为 full
            ma_full = 1;
            ma_val  = (uint16_t)(ma_sum >> 4);
        } else {
            uint8_t count = ma_ptr + 1;  // 1..15
            ma_val = (uint16_t)(ma_sum / count);
        }
    }
    ma_buf[ma_ptr] = acc_mag;
    ma_ptr = (ma_ptr + 1) & 0x0F;

    /* ---------- 2) 信号强度衰减 (每 16 拍 -1) ---------- */
    if (conf_decay_cnt > 0) {
        conf_decay_cnt--;
    } else {
        conf_decay_cnt = 15;
        if (confidence > 0) confidence--;
    }

    /* ---------- 3) 4 态 FSM ---------- */
    step_event = 0;
    switch (step_state) {
    case ST_TRACK:
        local_max = acc_mag;
        fall_cnt  = 0;
        if (ma_full && acc_mag > (ma_val + THRESH_OFFSET)) {
            step_state = ST_RISE;
            local_max  = acc_mag;
        }
        break;

    case ST_RISE:
        if (acc_mag > local_max) {
            local_max = acc_mag;
            fall_cnt  = 0;
        } else if ((local_max - acc_mag) >= FALL_DELTA) {
            step_state = ST_FALL;
            fall_cnt  = 1;
            peak_val  = local_max;
        }
        break;

    case ST_FALL: {
        // 用旧 fall_cnt 做确认判断 (对齐 Verilog 的非阻塞语义)
        uint8_t prev_fall = fall_cnt;
        if (acc_mag < local_max) {
            local_max = acc_mag;
            if (fall_cnt < 0x0F) fall_cnt++;
        } else {
            // 中途反弹, 重新武装
            local_max = acc_mag;
            fall_cnt  = 1;
        }

        if (prev_fall >= PEAK_HOLD) {
            /* ---- 步确认 ---- */
            ma_at_peak = ma_val;
            step_time_buf[step_time_ptr] = sample_cnt;
            step_time_ptr = (step_time_ptr + 1) & 0x07;
            if (step_time_n < 0x0F) step_time_n++;
            if (step_count  < 0xFFFF) step_count++;   // 16-bit 累加
            step_event    = 1;
            step_gap_cnt  = 0;

            // confidence = (peak - ma) << 2, 8-bit 饱和
            if (peak_val > ma_val) {
                uint16_t cv = (uint16_t)(peak_val - ma_val) << 2;
                confidence = (cv > 255) ? 255 : (uint8_t)cv;
            } else {
                confidence = 0;
            }
            conf_decay_cnt = 15;
            refr_cnt       = REFRACTORY_SAMP;
            step_state     = ST_REFRACT;
            fall_cnt       = 0;
        }
        break;
    }

    case ST_REFRACT:
        // 当前为 1 时本拍减为 0, 下一拍切回 TRACK
        if (refr_cnt == 1) {
            step_state = ST_TRACK;
            fall_cnt   = 0;
        }
        if (refr_cnt > 0) refr_cnt--;
        break;
    }

    /* ---------- 4) 步频计算 (最近 CADENCE_AVG_N 步) ---------- */
    if (step_time_n >= 2) {
        uint8_t oldest_idx, newest_idx;
        if (step_time_n < CADENCE_AVG_N) {
            oldest_idx = 0;
            newest_idx = step_time_n - 1;
        } else {
            oldest_idx = step_time_ptr;
            newest_idx = (step_time_ptr == 0) ? 7 : (step_time_ptr - 1);
        }

        uint32_t interval_diff = step_time_buf[newest_idx] - step_time_buf[oldest_idx];
        if (interval_diff == 0) interval_diff = 1;

        uint8_t  n = (step_time_n < CADENCE_AVG_N) ? step_time_n : CADENCE_AVG_N;
        uint32_t numerator = (uint32_t)(n - 1) * 6000;
        uint16_t cadence_raw = (uint16_t)(numerator / interval_diff);

        if (cadence_raw > SAT_CEIL) cadence_raw = SAT_CEIL;
        if (cadence_raw != 0)        cadence     = cadence_raw;  // 0 不更新
    }

    /* ---------- 5) 2 秒无步 → 归零 (停下的人不显示陈旧值) ---------- */
    if (step_gap_cnt >= STEP_GAP_RESET) {
        cadence    = 0;
        confidence = 0;
    }
}

/* ====================================================================
 * main
 * ==================================================================== */
int main(void) {
    /* GPIO 输入初始化 */
    XGpio_Initialize(&GpioAccLow,  XPAR_AXI_GPIO_0_BASEADDR);
    XGpio_Initialize(&GpioAccHigh, XPAR_AXI_GPIO_0_BASEADDR);
    XGpio_Initialize(&GpioGyro,    XPAR_AXI_GPIO_1_BASEADDR);
    XGpio_Initialize(&GpioStatus,  XPAR_AXI_GPIO_2_BASEADDR);
    XGpio_Initialize(&GpioOutMag,  XPAR_AXI_GPIO_3_BASEADDR);

    XGpio_SetDataDirection(&GpioAccLow,  1, 0xFFFFFFFF);
    XGpio_SetDataDirection(&GpioAccHigh, 2, 0xFFFFFFFF);
    XGpio_SetDataDirection(&GpioGyro,    1, 0xFFFFFFFF);
    XGpio_SetDataDirection(&GpioStatus,  1, 0x00000003);
    XGpio_SetDataDirection(&GpioOutMag,  1, 0x00000000);  // 全输出

    xil_printf("\r\n=== MPU6050 + step detector (C) ===\r\n");
    xil_printf("10 Hz MOT mag | 1 Hz STEP spm/conf/cnt | 步事件 STEP!\r\n");
    xil_printf("Waiting for MPU6050 init...\r\n");

    /* 等待 MPU6050 初始化 (debug: 每 200 拍打印一次 st 值) */
    {
        uint32_t wait_cnt = 0;
        while (1) {
            uint32_t st = XGpio_DiscreteRead(&GpioStatus, 1);
            if (st & 0x02) break;
            if ((++wait_cnt % 200) == 0) {
                xil_printf("[init] st=0x%08x cnt=%u\r\n", st, wait_cnt);
            }
        }
    }
    xil_printf("MPU6050 ready.\r\n\r\n");

    uint32_t tick = 0;

    while (1) {
        /* 检查 data_valid (bit0) */
        uint32_t st = XGpio_DiscreteRead(&GpioStatus, 1);
        if (!(st & 0x01)) continue;

        /* ---------- 读 6 字节加速度 ---------- */
        uint32_t d01 = XGpio_DiscreteRead(&GpioAccLow,  1);
        uint32_t d02 = XGpio_DiscreteRead(&GpioAccHigh, 2);

        uint8_t axh = (uint8_t)(d01 >> 24);
        uint8_t axl = (uint8_t)(d01 >> 16);
        uint8_t ayh = (uint8_t)(d01 >> 8);
        uint8_t ayl = (uint8_t)(d01 >> 0);
        uint8_t azh = (uint8_t)(d02 >> 24);
        uint8_t azl = (uint8_t)(d02 >> 16);

        /* ---------- 拼装 + 0.01g 换算 ---------- */
        int16_t ax_cg = pack16(axh, axl) / RAW2CG;
        int16_t ay_cg = pack16(ayh, ayl) / RAW2CG;
        int16_t az_cg = pack16(azh, azl) / RAW2CG;

        /* ---------- 总模长 (含重力 ~1g) ---------- */
        uint32_t raw_sq = (uint32_t)((int32_t)ax_cg * ax_cg)
                        + (uint32_t)((int32_t)ay_cg * ay_cg)
                        + (uint32_t)((int32_t)az_cg * az_cg);
        uint16_t raw_mag = isqrt(raw_sq);

        /* ---------- 重力基线追踪 (Q16 LPF) ---------- */
        gravity_est_q16 += ((int32_t)(raw_mag << 16) - (int32_t)gravity_est_q16) >> GRAVITY_LP_SHIFT;
        uint16_t gravity_est = (uint16_t)(gravity_est_q16 >> 16);

        /* ---------- 运动加速度 (防下溢) ---------- */
        uint16_t acc_mag;
        if (raw_mag > gravity_est)
            acc_mag = raw_mag - gravity_est;
        else
            acc_mag = 0;

        /* ---------- 步频检测 (100 Hz) ---------- */
        step_detector_tick(acc_mag);

        /* ---------- 顶层 GPIO 输出 (每拍 100 Hz) ----------
         * GPIO_3 = { step_count[15:0], cadence[15:0] }
         * 顶层 cadence = gpio3_o[15:0], step_count = gpio3_o[31:16]
         */
        {
            uint32_t gpio3_val = ((uint32_t)step_count << 16) | (uint32_t)cadence;
            XGpio_DiscreteWrite(&GpioOutMag, 1, gpio3_val);
        }

        /* ---------- UART 输出 ---------- */
        // 10 Hz: acc_mag
        if ((tick % 10) == 0) {
            xil_printf("MOT mag=%u\r\n", acc_mag);
        }

        // 1 Hz: cadence / confidence / step_count
        if ((tick % 100) == 0) {
            xil_printf("STEP spm=%u conf=%u cnt=%u\r\n",
                       cadence, confidence, step_count);
        }

        // 步事件: 立即打, 但限流 5 Hz 防止 UART 堵死
        if (step_event &&
            (sample_cnt - last_step_print_sample) >= STEP_PRINT_MIN_INTERVAL) {
            xil_printf("STEP! t=%u spm=%u conf=%u\r\n",
                       sample_cnt, cadence, confidence);
            last_step_print_sample = sample_cnt;
        }

        tick++;
        usleep(10000);   // 10 ms
    }

    return 0;
}
