
board_runner_args(jlink "--device=TMS570LS1224" "--iface=jtag" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
