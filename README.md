# ReLeva Controller Firmware

Firmware for the ReLeva espresso machine controller.  
The system manages temperature control, boiler water levels, heater switching,
and communication with external devices. For more information see: 
[ReLeva Project](http://github.com)

The firmware targets an **STM32L431** microcontroller and is built using
CMake and the GNU Arm Embedded toolchain.

---

## Features

- PID temperature control for brew and steam boilers
- AC heater control with zero-cross synchronization
- Water-level monitoring and refill logic
- Bluetooth Low Energy (BLE) telemetry and configuration
- Modular architecture for experimentation and tuning

---

## Toolchain Requirements

- CMake ≥ 3.16
- GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`)
- OpenOCD (for flashing and debugging)
- ST-Link (or compatible SWD programmer)

---

## Build Instructions

Clone the repository and initialize submodules if used:

```sh
git clone https://github.com/maknig/re-leva
cd re-leva
```

Create a build directory:

```sh
mkdir build
cd build
```

Configure the project:

```sh
cmake  ..

```

Build the firmware:

```sh
cmake --build .

```

The resulting .elf and .bin files will be located in the build/ directory.

## Flashing Instructions

Flashing is done via SWD using an ST-Link and OpenOCD.

### Using OpenOCD

Connect the programmer and run:

```sh
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg

```

In a separate terminal:

```sh
arm-none-eabi-gdb releva.elf
```

Inside GDB:

```sh
target remote localhost:3333
monitor reset halt
load
monitor reset run

```

Alternatively, flashing can be done directly:

```sh
openocd \
  -f interface/stlink.cfg \
  -f target/stm32l4x.cfg \
  -c "program releva.bin verify reset exit"

```

## Status

This firmware is under active development.
APIs, configuration formats, and control parameters may change without notice.

The code is intended for experimentation and learning and is not certified
for commercial or safety-critical use.
