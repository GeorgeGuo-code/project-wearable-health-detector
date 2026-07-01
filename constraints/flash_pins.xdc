# =============================================================================
# flash_pins.xdc — W25Q64 SPI 引脚约束 (PMOD J5)
#
#   board_top.v 验证过这套引脚可用, 这里直接复用:
#     SCK  = B11   (J5.15)
#     CSn  = A11   (J5.16)
#     MOSI = E15   (J5.17)
#     MISO = E16   (J5.18)
# =============================================================================

# ---- W25Q64 SPI (PMOD J5) ----
set_property PACKAGE_PIN B11 [get_ports flash_sck]
set_property PACKAGE_PIN A11 [get_ports flash_csn]
set_property PACKAGE_PIN E15 [get_ports flash_mosi]
set_property PACKAGE_PIN E16 [get_ports flash_miso]
set_property IOSTANDARD LVCMOS33 [get_ports {flash_sck flash_csn flash_mosi flash_miso}]

# ---- flash_view_en (调试用拨码开关) ----
#   TODO: 根据实际板子绑到具体拨码 (SW0..SW15).
#   若暂时无空闲拨码, 可注释掉 PACKAGE_PIN 改用 PULLDOWN 保持低 (正常模式):
#     set_property PULLDOWN true [get_ports flash_view_en]
#   或在顶层 assign 0
set_property IOSTANDARD LVCMOS33 [get_ports flash_view_en]
# set_property PACKAGE_PIN ... [get_ports flash_view_en]
