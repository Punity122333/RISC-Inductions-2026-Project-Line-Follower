## Optional / Bonus Components
- Encoder sensors for wheel odometry feedback
- Engineering drawings / CAD model (submitted separately for bonus credit)

## Parts Used in This Build
The build is a 4WD differential-drive robot. It uses four motors, not the two in the core list.

| Ref | Part | Qty | Notes |
|---|---|---|---|
| U1 | Orange Pi Zero 3, 4 GB | 1 | Runs the camera and high-level control |
| U2 | ESP32 DevKitC (ESP-WROOM-32) | 1 | Drives the motors and reads the encoders |
| U3 | TB6612FNG dual motor driver | 1 | Left side |
| U4 | TB6612FNG dual motor driver | 1 | Right side |
| M1 to M4 | N20 motor with encoder | 4 | Pins: M1, M2, C1, C2, VCC, GND |
| J1 | 2S Li-ion battery, 7.4 V nominal | 1 | Protection and charger are external |
| F1 | Blade fuse, 2 A | 1 | Check against motor stall current |
| SW1 | SPST toggle switch | 1 | Master switch |
| PS1 | Adjustable buck converter | 1 | 5 V logic branch |
| PS2 | Adjustable buck converter | 1 | 6 V motor branch |
| J2 | USB connector | 1 | Camera port |
| J3 | USB connector | 1 | Orange Pi to ESP32 serial link |
| J4 | USB connector block | 1 | ESP32 USB serial side |
| Camera | USB webcam | 1 | |
| R5 | Resistor, 10 kΩ | 1 | STBY pull-down |
| C3, C4 | Ceramic capacitor, 100 nF | 2 | Logic rail decoupling |
| Cdecouple | Electrolytic capacitor, 1 µF | 1 | Logic rail decoupling |
| Cbulk | Electrolytic capacitor | 1 | Motor rail bulk, value not chosen yet |
| C5 | Electrolytic capacitor | 1 | At U4 VM, value not chosen yet |

## Connections

### Motor control (ESP32 to drivers)
| ESP32 pin | Driver pin | Function |
|---|---|---|
| IO16 | U3 PWMA | M1 front-left PWM |
| IO17 | U3 PWMB | M2 rear-left PWM |
| IO21 | U3 AIN1 and BIN1 | Left direction 1 |
| IO22 | U3 AIN2 and BIN2 | Left direction 2 |
| IO18 | U4 PWMA | M3 front-right PWM |
| IO19 | U4 PWMB | M4 rear-right PWM |
| IO23 | U4 AIN1 and BIN1 | Right direction 1 |
| IO25 | U4 AIN2 and BIN2 | Right direction 2 |
| IO26 | U3 STBY, U4 STBY, R5 pin 1 | Shared standby net |

### Driver outputs to motors
| Driver pins | Motor |
|---|---|
| U3 A01, A02 | M1 (terminals M1, M2) |
| U3 B01, B02 | M2 (terminals M1, M2) |
| U4 A01, A02 | M3 (terminals M1, M2) |
| U4 B01, B02 | M4 (terminals M1, M2) |

### Encoders
| Motor | C1 | C2 |
|---|---|---|
| M1 front-left | IO34 | IO35 |
| M2 rear-left | GPIO36 (VN) | GPIO39 (VP) |
| M3 front-right | IO13 | IO14 |
| M4 rear-right | IO27 | IO32 |

Encoder VCC pins go to the ESP32 3.3 V net. Encoder GND pins go to common ground.

### Power
- Battery J1 VCC to fuse F1 IN.
- F1 OUT to switch SW1 COM.
- SW1 L1 to the input + of PS1 and PS2 (4.5 to 28 V input range).
- PS2 output + to U3 VM, U4 VM, Cbulk + and C5 +.
- ESP32 3V3 to U3 VCC, U4 VCC, M1 to M4 VCC, C3, C4 and Cdecouple +.
- PS1 output + to U1 5 V, U2 5V, and VBUS on the USB connectors and camera.
- Common ground: battery GND, both converters (input and output negatives), U1, U2, both drivers, all motors and encoders, USB connector GND pins, camera, capacitor negatives, R5 pin 2.

### USB
| Link | Connections |
|---|---|
| Camera data | U1 USB2 DP to camera D+ and J2 D+. U1 USB2 DM to camera D- and J2 D-. |
| Serial data | U1 USB3 DP to J3 D+ and J4 D+. U1 USB3 DM to J3 D- and J4 D-. |

## Notes
- Set PS1 to 5.0 V and PS2 to the motor voltage with a multimeter before connecting anything.
- PS1 output shares a net with the USB VBUS pins. Do not power the ESP32 over USB while the battery is on.
- IO34, IO35, GPIO36 and GPIO39 are input-only with no internal pull-ups. Open-collector encoders need external 10 kΩ pull-ups to 3V3.
- IO16 and IO17 are PSRAM pins on WROVER modules, so this wiring needs a WROOM-32.