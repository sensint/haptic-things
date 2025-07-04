# HAPTICUBES 🧊

![alt text](Pictures/IMG_7376.webp)

## Project
A self-contained 3D-printed cube tangible enabling compliance illusion and friction illusion based on motion-coupled vibrotactile rendering with WIFI control.

## Components
1. [Actuator: HAPCOIL-ONE](https://tactilelabs.com/product/hapcoil-one/)
2. [IMU: BNO085](https://www.adafruit.com/product/4754)
3. [RGB LED: APA102](https://www.adafruit.com/product/3341)
4. [Main MCU: Daisy Seed](https://electro-smith.com/products/daisy-seed)
5. [Class D Amplifier: PAM8302](https://www.adafruit.com/product/2130)
6. [Wifi MCU: Wemos D1 Mini](https://www.wemos.cc/en/latest/d1/d1_mini.html)
7. [Force Sensing Resistors: FSR03](https://www.ohmite.com/catalog/fsr-series/FSR03CE)
8. [Battery management: Powerboost 1000](https://www.adafruit.com/product/2465)
9. [Battery: Generic 3.7v Lipo](https://www.adafruit.com/product/1578)

# Wiring 

                           +-----------------------------+
                           |        Daisy Seed MCU       |
                           |                             |
                           |  A0 ──> FSR 1               |
                           |  A1 ──> FSR 2               |
                           |  A2 ──> FSR 3               |
                           |  A3 ──> FSR 4               |
                           |  A4 ──> FSR 5               |
                           |  A5 ──> FSR 6               |
                           |                             |
                           |  SCL1 ─────┐                |
                           |  SDA1 ─────┘──> BNO085 I2C  |
                           |                             |
                           |  TX1 ◄─────────> Weemos D2  |
                           |  RX1 ◄─────────> Weemos D3  |
                           |                             |
                           |  D0 ──> LED DI             |
                           |  D1 ──> LED CLK             |
                           |                             |
                           |  AUDIO OUT 1 ──> Class D Amp|
                           |                             |
                           |  VIN ────┬────────┐         |
                           |  DGND ───┘── AGND │         |
                           +-------------------│---------+
                                               │
                              +----------------▼----------------+
                              |        PowerBoost 1000          |        +---------------------+
                              |   5V OUT ─────┬─────────────────┐ ◄───── |    3.7V LIPO CELL   |
                              |               │                 │        +---------------------+                      
                              |              GND────────────────┘                              
                              +---------------┼-----------------+
                                              ▼
                                  +---------------------+
                                  |     Wemos D1        |
                                  |   3V3 ◄─────────────┘
                                  |   GND ◄─────────────┘
                                  +---------------------+

                                  +---------------------+
                                  |   Class D Amp       |
                                  |   3V3 ◄─────────────┘
                                  |   GND ◄─────────────┘
                                  +---------------------+
                                  
                                  +---------------------+
                                  |     BNO085          |
                                  |   VIN ◄─────────────┘
                                  |   GND ◄─────────────┘
                                  +---------------------+
                                  
                                  +---------------------+
                                  |       LED Strip     |
                                  |   VCC ◄─────────────┘
                                  |   GND ◄─────────────┘
                                  +---------------------+

