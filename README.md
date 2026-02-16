
This repository acts as a Zephyr module, implementing support for the
FramSat-1 and FramSat1.5 on board computer. This includes custom SoC
support for the `TI TMS570LS1224` SoC, aswell as board configuration files.

## Usage
In order to use the `TMS570LS1224` SoC, the included big-endian `arm-none-eabi`
toolchain must be used. To do so, add the below to your `CMakeLists.txt`:
```cmake
set(TOOLCHAIN_ROOT <path/to/this/module>)
set(ZEPHYR_TOOLCHAIN_VARIANT be-arm-none-eabi)
```

Alternatively, using Zephyr SDK>=17.4.0 and Zephyr 4.3.0, compilation can be
done by simply passing `-DTOOLCHAIN_C_FLAGS="-mbe32" -DTOOLCHAIN_LD_FLAGS="-mbe32"`
to `west build`

The module must also be included in the Zephyr build system, which can be done
with:
```cmake
set(EXTRA_ZEPHYR_MODULES <path/to/this/module>)
```
