
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_TMS570_TIMER tms570_timer.c)
