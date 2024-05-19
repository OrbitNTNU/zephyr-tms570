cmake_minimum_required(VERSION 3.20)

set(ENV{BE_ARM_NONE_EABI_TOOLCHAIN_PATH}
    ${CMAKE_CURRENT_LIST_DIR}/toolchain/be-gcc-arm-none-eabi)

zephyr_include_directories(include)
add_subdirectory(drivers)
