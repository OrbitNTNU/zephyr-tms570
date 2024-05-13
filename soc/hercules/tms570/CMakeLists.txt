
zephyr_include_directories(.)
set(SOC_LINKER_SCRIPT
    ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_a_r/scripts/linker.ld
    CACHE
    INTERNAL
    ""
)

