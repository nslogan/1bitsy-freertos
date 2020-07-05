# 1bitsy FreeRTOS

This project contains my experiments with FreeRTOS on the [1bitsy](https://1bitsy.org/).

## Build

### Dependencies

* [meson](https://mesonbuild.com/) build system
* ARM toolchain (version used during development: `arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]`)

Additionally, this repository has three submodules:

* [CMSIS 5](https://github.com/ARM-software/CMSIS_5.git)
* [CMSIS Device STM32F4](https://github.com/STMicroelectronics/cmsis_device_f4.git)
* [FreeRTOS-Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V10.3.1-kernel-only)

Be sure to initialize these before building.

```
# During clone
$ git clone https://github.com/nslogan/1bitsy-freertos.git --recurse-submodules

# After clone
$ git submodule update --init --recrusive
```

### Build Binary

```
# Configure build directory / build options
meson <build-dir> --cross-file tools/meson/gcc-arm-cortex-m7.txt

# Build target
$ ninja -C <build-dir>
# or
$ cd <build-dir> && ninja
```

This will generate `<build-dir>/usbtmc-demo.elf`.

### Deploy

Assumes you have a [blackmagic probe](https://1bitsquared.com/products/black-magic-probe). If you have a python-enabled binary of gdb you will be able to use [FreeRTOS-GDB](https://github.com/autolycus/FreeRTOS-GDB) which lets you examine various FreeRTOS structures like threads and mutexes.

```
$ arm-none-eabi-gdb[-py] <build-dir>/usbtmc-demo.elf
# Determine ttyACM number from `dmesg` or other means, e.g.
# 
# [    9.644649] usb 5-1.4.2.4.2: Product: Black Magic Probe
# [    9.644649] usb 5-1.4.2.4.2: Manufacturer: Black Sphere Technologies
# [    9.644650] usb 5-1.4.2.4.2: SerialNumber: C1DA9FEB
# [    9.723503] cdc_acm 5-1.4.2.4.2:1.0: ttyACM1: USB ACM device
# [    9.724004] cdc_acm 5-1.4.2.4.2:1.2: ttyACM2: USB ACM device
#
# In this example it is ttyACM1
(gdb) tar ext /dev/ttyACM1
Remote debugging using /dev/ttyACM1
(gdb) mon swdp_scan
Target voltage: 3.3V
Available Targets:
No. Att Driver
 1      STM32F4xx
(gdb) attach 1
Attaching to program: <binary-path>, Remote target
(gdb) load
# ... TODO ...
(gdb) r
# ... TODO ...
```
