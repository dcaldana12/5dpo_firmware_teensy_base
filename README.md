# [5dpo_firmware_teensy_base](https://github.com/5dpo/5dpo_firmware_teensy_base)

This repository is a generic implementation of the firmware for a mobile robot
with up-to four wheels using the
[Teensy 4.1](https://www.pjrc.com/store/teensy41.html) development board as the
microcontroller to drive the motors.

See the header configuration file [robotconfig.h](/include/robotconfig.h)
to set up properly your robot!

**Version 0.0.0** _(NOT TESTED)_

- Serial communication using the library channels
- Read encoders channel A and B for wheeled odometry
- Motors angular speed control with a generic PID controller
- Motors PWM control
  ([TimerOne](https://github.com/PaulStoffregen/TimerOne) and
  [TimerThree](https://github.com/PaulStoffregen/TimerThree) PWM)
- Watchdog timer to disable the motors if the PC does not send anything for a
  certain time
- Limitation on the PWM change

**The next version will add these features:**

- TBD

**Bugs identified in the current version:**

- TBC

## Hardware

- Teensy 4.1
  ([datasheet](https://www.pjrc.com/store/teensy41.html),
  [store](https://mauser.pt/catalog/product_info.php?products_id=096-9109))
- DC Motor Controllers
  - Cytron 13A, 5-30V Single DC Motor Controller
    ([info](https://www.cytron.io/p-10amp-5v-30v-dc-motor-driver),
    [store](https://eu.robotshop.com/products/cytron-13a-5-30v-single-dc-motor-controller))
  - Cytron 10A 5-30V Dual Channel DC Motor Driver
    ([info](https://www.cytron.io/p-10amp-5v-30v-dc-motor-driver-2-channels),
    [store](https://eu.robotshop.com/products/cytron-10a-5-30v-dual-channel-dc-motor-driver))
- TXB0104 4-Bit Bidirectional Voltage-Level Shifter with Auto Direction Sensing
  and +/-15 kV ESD Protect
  ([datasheet](https://www.ti.com/lit/ds/symlink/txb0104.pdf?ts=1708816249554))
  - Logic translator module with 4 bidirectional channels
    ([store](https://mauser.pt/catalog/product_info.php?products_id=096-7592))

## Serial Communication (channels)

**Teensy > USB**

- `[g]`: encoders total count of motor 0 (ticks)
- `[h]`: encoders total count of motor 1 (ticks)
- `[i]`: encoders total count of motor 2 (ticks)
- `[j]`: encoders total count of motor 3 (ticks)
- `[k]`: interval time between control cycles (us)
- `[r]`: reset signal

**USB > Teensy**

- `[G]`: reference angular speed of motor 0 (rad/s)
- `[H]`: reference angular speed of motor 1 (rad/s)
- `[I]`: reference angular speed of motor 2 (rad/s)
- `[J]`: reference angular speed of motor 3 (rad/s)
- `[K]`: PWM value of motors
  - `(value >> 24) & 0x03`: motor index (0..3)
  - `value & 0xFFFF`: PWM value (0..`kMotPWMmax`)

## Usage

**Requirements**

- PlatformIO extension for VS Code

### Compilation

1. Clone the repository
   ```sh
   # Clone repository
   git clone git@github.com:5dpo/5dpo_firmware_teensy_base.git
   cd 5dpo_firmware_teensy_base

   # Open the folder as a project
   code .
   ```
2. Open `src/main.cpp`
3. Compile the project using the PlatformIO extension in VS Code
   (PlatformIO on left-side navigation bar > Project Tasks > Build)

### Run

1. Open `src/main.cpp`
2. Upload the project using the PlatformIO extension in VS Code
   (PlatformIO on left-side navigation bar > Project Tasks > Upload)
   - **Note:** by default, it shows the Teensy GUI upload tool that
     automatically runs the boatloader and allows to upload the program
     (wo/ the need to press the physical button...)

## Acknowledges

- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)

## Contacts

If you have any questions or you want to know more about this work, please
contact any member of the [5dpo Robotics Team](https://5dpo.github.io/).
