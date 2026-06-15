// =============================================================================
// test_defines.v — 硬件测试开关
//   取消下面这行 `define 的注释 → 进入测试模式 (红 LED 常亮 + 蜂鸣器常响)
//   注释掉 / 删掉        → 正常模式
//
// 用法: 在 display_top_spi.sv 顶部 `include "test_defines.v" 即可
// =============================================================================

`ifndef TEST_DEFINES_V
`define TEST_DEFINES_V

// ---- 测试开关 ----
//   启用: 强制 health_status = DANGER (2'b10)
//         强制 alarm = 1
//   目的: 验证红 LED 硬件通路 + 蜂鸣器硬件通路, 不依赖传感器实测 DANGER
//`define TEST_FORCE_DANGER       // ← 取消行首 // 即可启用测试

`endif
