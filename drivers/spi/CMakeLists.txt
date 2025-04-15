
zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_SPI_TMS570 spi_tms570.c)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/spi)
