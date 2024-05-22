# [5dpo_firmware_ratf_teensy](https://github.com/5dpo/5dpo_firmware_ratf_teensy)

This repository implements the firmware required to the four-wheeled
omnidirectional robot for the Robot@Factory 4.0 competition using the
[Teensy 4.1](https://www.pjrc.com/store/teensy41.html) development board as the
microcontroller to drive the motors.
The schematics of the PCB that should be using in conjunction with the
[Teensy 4.1](https://www.pjrc.com/store/teensy41.html) microcontroller is
available in the [doc/](/doc/teensy-4.1_5dpo-pcb_schematic.pdf) folder.
Lastly, this firmware is based on the generic implementation of the
[5dpo_firmware_teensy_base](https://github.com/5dpo/5dpo_firmware_teensy_base).

See the header configuration file [robotconfig.h](/include/robotconfig.h)
to set up properly your robot!

**Version 1.0.0** _(NOT TESTED)_

- Serial communication using the library channels
- Read encoders channel A and B for wheeled odometry
- Motors angular speed control with a generic PID controller
- Motors PWM control
  ([TimerOne](https://github.com/PaulStoffregen/TimerOne) and
  [TimerThree](https://github.com/PaulStoffregen/TimerThree) PWM)
- Limitation on the PWM change
- Reset signal
- Solenoid actuator
- Switch to sense the presence of a box
- Watchdog timer to disable the motors if the PC does not send anything for a
  certain time
- Switch off solenoid actuator when the motors are disabled due to watchdog
  timer

**The next version will add these features:**

- TBD

**Bugs identified in the current version:**

- TBC

## Hardware

### Mechanics

- 60mm Aluminum Mecanum Wheels Set (2x left + 2x right, 45° rotational axis)
  ([shop](https://eu.robotshop.com/products/60mm-mecanum-wheel-set-2x-left-2x-right))
- Metal DC Geared Motor w/Encoder - 12V 251RPM 18Kg.cm (DFRobot)
  ([shop-1](https://eu.robotshop.com/products/12v-dc-motor-251rpm-encoder),
  [shop-2](https://www.dfrobot.com/product-634.html))

### Electronics

- Microcontroller
  - Teensy 4.1
    ([datasheet](https://www.pjrc.com/store/teensy41.html),
    [store](https://mauser.pt/catalog/product_info.php?products_id=096-9109))
  - Teensy 4.1 Shield (Drive Four Motors + GPIO)
    ([schematics](/doc/teensy-4.1_5dpo-pcb_schematic.pdf))
    - TXB0104 4-Bit Bidirectional Voltage-Level Shifter with Auto Direction
      Sensing and +/-15 kV ESD Protect
      ([datasheet](https://www.ti.com/lit/ds/symlink/txb0104.pdf?ts=1708816249554))
      - Logic translator module with 4 bidirectional channels
        ([store](https://mauser.pt/catalog/product_info.php?products_id=096-7592))
- Actuators
  - DC Motor Controllers (must support 3.3V logic, see
    [schematics](/doc/teensy-4.1_5dpo-pcb_schematic.pdf) for further
    information)
    - Cytron 13A 5-30V Single DC Motor Controller
      ([info](https://www.cytron.io/p-10amp-5v-30v-dc-motor-driver),
      [store](https://eu.robotshop.com/products/cytron-13a-5-30v-single-dc-motor-controller))
    - Cytron 10A 5-30V Dual Channel DC Motor Driver
      ([info](https://www.cytron.io/p-10amp-5v-30v-dc-motor-driver-2-channels),
      [store](https://eu.robotshop.com/products/cytron-10a-5-30v-dual-channel-dc-motor-driver))
  - Grove Electromagnet
    ([shop](https://eu.robotshop.com/products/grove-electromagnet))
- Sensors
  - 10 DoF Inertial Measurement Unit (IMU)
    ([shop](https://www.waveshare.com/10-dof-imu-sensor-b.htm))
  - Micro Switch with Roller Lever Arm
    ([shop](https://mauser.pt/catalog/product_info.php?products_id=010-1473))
  - Raspberry Pi Camera Board v2.1 (8MP, 1080p)
    ([shop-1](https://mauser.pt/catalog/product_info.php?products_id=096-4061),
    [shop-2](https://pt.farnell.com/raspberry-pi/rpi-8mp-camera-board/raspberry-pi-camera-board-v2/dp/2510728),
    [shop-3](https://pt.mouser.com/ProductDetail/Raspberry-Pi/SC0023?qs=T%252BzbugeAwjgRU4vb4%252BbLIg%3D%3D),
    [shop-4](https://www.digikey.pt/en/products/detail/raspberry-pi/SC0023/6152810?s=N4IgTCBcDaIIwDYCcAGAtHFc5oHIBEQBdAXyA))
  - YDLIDAR X4 360° Laser Scanner
    ([shop](https://eu.robotshop.com/products/ydlidar-x4-360-laser-scanner))
- Power
  - DC-DC Buck Converter 300W 6-40V to 1.2-36V 20A Step Dowm
    ([shop](https://www.amazon.com/DIANN-300W-DC-DC-Buck-Converter/dp/B0B4CZ6GRV))
  - LiPo Battery 11.1V 3S
    - Gens ace 5000mAh 11.1V 3S1P 60C Lipo Battery Pack with XT90 Plug Bashing
      Series
      ([shop](https://www.gensace.de/gens-ace-5000mah-11-1v-3s1p-60c-lipo-battery-pack-with-xt-90-plug-bashing-series.html))
    - Tattu 11.1V 15C 3S 10000mAh Lipo Battery Pack
      ([shop](https://www.gensace.de/tattu-11-1v-15c-3s-10000mah-lipo-battery-pack.html))
  - Power ON/OFF Switch
    ([shop](https://mauser.pt/catalog/product_info.php?products_id=010-0626))

## Electronic Connections

### Teensy 4.1 Shield (Drive Four Motors + GPIO)

**Front View**

![teensy-4.1_5dpo-pcb_front](/doc/teensy-4.1_5dpo-pcb_front.jpeg)

**Back View**

![teensy-4.1_5dpo-pcb_front](/doc/teensy-4.1_5dpo-pcb_back.jpeg)

### Observations


**ENC_\<BR|BL|FR|FL\>**

- 5V logic
- JST connectors
- Vcc + CHA + CHB + GND

**MOT_\<BR|BL|FR|FL\>**

- 3.3V logic
- JST connectors
- GND + DIR + PWM

**\<ENC|MOT\> Headers**

- headers (e.g., w/ jumpers)
- only for debug purposes with a oscilloscope
  (e.g., see the quadrature of the encoders' signals, check the PWM generation)

**\<I1C|RX/TX|SPI\>**

- 3.3V logic
- headers (e.g., w/ jumpers)
- always available 1x5V + 1x3.3V + 2xGND
- not necessarily for communication, can be setup as IO
  (considering 3.3V logic!)

**Teensy 4.1 Pinout**

| Teensy Pin | Name | Description |
| :---: | :---: | :--- |
| 0 | ENC_BR_CHA | Channel A of the encoder of the back-right wheel |
| 1 | ENC_BR_CHB | Channel B of the encoder of the back-right wheel |
| 2 | ENC_BL_CHA | Channel A of the encoder of the back-left wheel |
| 3 | ENC_BL_CHB | Channel B of the encoder of the back-left wheel |
| 4 | ENC_FR_CHA | Channel A of the encoder of the front-right wheel |
| 5 | ENC_FR_CHB | Channel B of the encoder of the front-right wheel |
| 30 | ENC_FL_CHA | Channel A of the encoder of the front-left wheel |
| 31 | ENC_FL_CHB | Channel B of the encoder of the front-left wheel |
| 23 | MOT_BR_DIR | Direction of the back-right motor driver |
| 22 | MOT_BL_DIR | Direction of the back-left motor driver |
| 21 | MOT_FR_DIR | Direction of the front-right motor driver |
| 20 | MOT_FL_DIR | Direction of the front-left motor driver |
| 6 | MOT_BR_PWM | PWM of the back-right motor driver |
| 7 | MOT_BL_PWM | PWM of the back-left motor driver |
| 8 | MOT_FR_PWM | PWM of the front-right motor driver |
| 9 | MOT_FL_PWM | PWM of the front-left motor driver |
| 40 | GPIO_1 | General Purpose Input/Output 1 |
| 39 | GPIO_2 | General Purpose Input/Output 2 |
| 38 | GPIO_3 | General Purpose Input/Output 3 |
| 37 | GPIO_4 | General Purpose Input/Output 4 |
| 36 | GPIO_5 | General Purpose Input/Output 5 |
| 35 | GPIO_6 | General Purpose Input/Output 6 |
| 34 | GPIO_7 | General Purpose Input/Output 7 |
| 33 | GPIO_8 | General Purpose Input/Output 8 |


Please always refer to the original documentation of the
[Teensy 4.1](https://www.pjrc.com/store/teensy41.html) board to confirm if your
connections are according to the shield schematics and the program setup.

_Additional observations:_

- Encoders pins set accordingly to the
  [_Hardware Quadrature Library for the Teensy 4.x_](https://github.com/mjs513/Teensy-4.x-Quad-Encoder-Library)
  to allow future implementation of hardware quadrature encoders reading
- PWM pins of the motor drivers set accordingly to the
  [TimerOne](https://github.com/PaulStoffregen/TimerOne) and
  [TimerThree](https://github.com/PaulStoffregen/TimerThree) libraries for the
  Teensy 4.1 microcontroller (see
  [TimerOne & TimerThree Libraries](https://www.pjrc.com/teensy/td_libs_TimerOne.html)
  for further information)


## Serial Communication (channels)

**Teensy > USB**

- `[g]`: encoders total count of motor 0 - back-right (ticks)
- `[h]`: encoders total count of motor 1 - back-left (ticks)
- `[i]`: encoders total count of motor 2 - front-right (ticks)
- `[j]`: encoders total count of motor 3 - front-left (ticks)
- `[k]`: interval time between control cycles (us)
- `[r]`: reset signal
- `[s]`: switch (0|1)

**USB > Teensy**

- `[G]`: reference angular speed of motor 0 - back-right (rad/s)
- `[H]`: reference angular speed of motor 1 - back-left (rad/s)
- `[I]`: reference angular speed of motor 2 - front-right (rad/s)
- `[J]`: reference angular speed of motor 3 - front-left (rad/s)
- `[K]`: PWM value of motors
  - `(value >> 24) & 0x03`: motor index (0..3)
  - `value & 0xFFFF`: PWM value (0..`kMotPWMmax`)
- `[L]`: solenoid (0|1)

## Usage

**Requirements**

- PlatformIO extension for VS Code

### Compilation

1. Clone the repository
   ```sh
   # Clone repository
   git clone git@github.com:5dpo/5dpo_firmware_ratf_teensy.git
   cd 5dpo_firmware_ratf_teensy

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
