
This repository acts as a Zephyr module, implementing support for custom
Orbit NTNU hardware. This includes custom SoC support, such as for the
`TI TMS570LS1224` SoC, aswell as board configuration files.

## Usage
In order to use the `TMS570LS1224` SoC, the included big-endian `arm-none-eabi`
toolchain must be used. To do so, add the below to your `CMakeLists.txt`:
```cmake
set(TOOLCHAIN_ROOT <path/to/this/module>)
set(ZEPHYR_TOOLCHAIN_VARIANT be-arm-none-eabi)
```

The module must also be included in the Zephyr build system, which can be done
with:
```cmake
set(EXTRA_ZEPHYR_MODULES <path/to/this/module>)
```
