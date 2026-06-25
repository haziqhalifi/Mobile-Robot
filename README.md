# Mobile Robot

PlatformIO firmware for an Arduino Uno based mecanum mobile robot. The active firmware lives in `src/main.cpp` and controls line following, intersection-based routes, timed movement, strafing, IR remote commands, and emergency stop handling.

## Hardware Target

- Board: Arduino Uno
- Framework: Arduino
- PlatformIO environment: `uno`
- Main robot library: `lib/MecanumCar_v2`
- IR receiver: Arduino-IRremote `2.8.0`
- Extra libraries: Servo, Adafruit NeoPixel, Adafruit PWM Servo Driver

## Project Layout

```text
src/
  main.cpp                         Active firmware compiled by PlatformIO

test/
  arm_pick_and_place_test.cpp       Arm and PCA9685 pick/place experiment
  auto_docking_left_test.cpp        Left-side auto docking experiment
  auto_docking_right_test.cpp       Right-side auto docking experiment
  auto_route_line_strafe_test.cpp   Line tracking and strafe route experiment
  intersection_parking_route_test.cpp
  ir_route_led_test.cpp
  line_block_rotation_test.cpp
  robot_self_audit_line_tracking_test.cpp
  sensor_test.cpp
  ultrasonic_distance_test.cpp

lib/
  MecanumCar_v2/                    Local mecanum robot driver
  ir/                               Local IR helper code
  Servo/                            Bundled Servo library
  Adafruit_NeoPixel/                Bundled NeoPixel library

PROJECT_GUIDE.md                    Detailed notes about the project
platformio.ini                      PlatformIO build configuration
```

## Build And Upload

Install PlatformIO, then run these commands from the project root:

```powershell
pio run
pio run --target upload
pio device monitor
```

If `pio` is not available in the terminal, open the project in VS Code with the PlatformIO extension and use the PlatformIO Build, Upload, and Monitor actions.

## IR Remote Commands

The active firmware maps remote buttons to robot routes:

- `1`: follow the line and stop at intersection 3
- `2`: stop at intersection 3, then turn right
- `3`: run button 2 route, then strafe left
- `4`: timed forward, timed right turn, then strafe left
- `5`: route through repeated intersection turns
- `6`: button 5 route, then 180 degree turn
- `7`: button 6 route, then return path
- Left arrow: manual strafe left
- `0`: stop manual movement
- `*`: emergency stop while moving

## Notes

- `src/main.cpp` is the only application file compiled by the default PlatformIO environment.
- Files in `test/` are hardware experiments and reference sketches, not a conventional automated unit test suite.
- The project includes a detailed [PROJECT_GUIDE.md](PROJECT_GUIDE.md) with deeper notes on structure and libraries.
