
zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_I2C_TMS570 i2c_tms570.c)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/i2c)
