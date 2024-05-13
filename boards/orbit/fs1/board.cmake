
board_runner_args(jlink "--device=TMS570LS1224" "--iface=jtag" "--speed=4000")
board_runner_args(openocd --target-handle=_CHIPNAME.cpu0)

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
