
zephyr_library()

zephyr_library_sources_ifdef(CONFIG_I2C_TMS570 i2c_tms570.c)
