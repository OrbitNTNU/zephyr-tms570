# SPDX-License-Identifier: Apache-2.0

zephyr_get(BE_ARM_NONE_EABI_TOOLCHAIN_PATH)
assert(    BE_ARM_NONE_EABI_TOOLCHAIN_PATH "BE_ARM_NONE_EABI_TOOLCHAIN_PATH is not set")

if(NOT EXISTS ${BE_ARM_NONE_EABI_TOOLCHAIN_PATH})
    message(FATAL_ERROR
        "Nothing found at BE_ARM_NONE_EABI_TOOLCHAIN_PATH:
        '${BE_ARM_NONE_EABI_TOOLCHAIN_PATH}'"
    )
endif()

set(TOOLCHAIN_HOME ${BE_ARM_NONE_EABI_TOOLCHAIN_PATH})

set(COMPILER gcc)
set(LINKER ld)
set(BINTOOLS gnu)

set(CROSS_COMPILE_TARGET arm-none-eabi)
set(SYSROOT_TARGET       arm-none-eabi)

# Required for the big endian toolchain to work. Without this, the compiler
# will silently fail
set(GCC_M_FPU vfpv3-d16)

set(CROSS_COMPILE ${TOOLCHAIN_HOME}/bin/${CROSS_COMPILE_TARGET}-)
set(SYSROOT_DIR   ${TOOLCHAIN_HOME}/${SYSROOT_TARGET})
set(TOOLCHAIN_HAS_NEWLIB ON CACHE BOOL "True if toolchain supports newlib")

set(CMAKE_C_BYTE_ORDER BIG_ENDIAN)
list(APPEND TOOLCHAIN_C_FLAGS -mbig-endian -mbe32)

message(STATUS "Found toolchain: be-arm-none-eabi (${BE_ARM_NONE_EABI_TOOLCHAIN_PATH})")
