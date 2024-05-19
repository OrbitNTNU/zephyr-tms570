
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_UART_TMS570 uart_tms570.c)
