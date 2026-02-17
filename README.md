
This repository acts as a Zephyr module, implementing support for the
FramSat-1 and FramSat1.5 on board computer. This includes custom SoC
support for the `TI TMS570LS1224` SoC, aswell as board configuration files.

## Usage
The TMS570 requires the `-mbe32` flags to be passed to compiler and linker,
which is not passed through Zephyr. Until a proper way of handling that is
introduced to Zephyr, the following must be added to your `CMakeLists.txt`
before calling `find_package(Zephyr)`:

```cmake
list(APPEND TOOLCHAIN_C_FLAGS -mbe32)
list(APPEND TOOLCHAIN_LD_FLAGS -mbe32)
```

Alternatively, this can be achieved by passing
`-DTOOLCHAIN_C_FLAGS="-mbe32" -DTOOLCHAIN_LD_FLAGS="-mbe32"`
to `west build`

The module must also be included in the Zephyr build system, which can be done
with:
```cmake
set(EXTRA_ZEPHYR_MODULES <path/to/this/module>)
```

## Supported version

This is tested on Zephyr SDK 17.4.0, using Zephyr 4.3.0. Python version is
`3.11`.
