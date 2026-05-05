# arc-drone

A modular, ESP32-based flight stack for a custom quadcopter. `arc-drone` bundles a small set of Arduino/C++ drivers — ESC (DShot), IMU, OLED telemetry display, and a Bluetooth gamepad controller — behind clean interfaces so they can be combined inside a single sketch (`drone.ino`) or reused independently.

## Features

- **Digital ESC driver** with DShot150 / DShot300 / DShot600 support
- **Bluetooth gamepad input** (PS4/PS5/Xbox/Switch controllers via Bluepad32)
- **MPU6050 IMU driver** for accelerometer, gyroscope, and temperature readings over I²C
- **SPI OLED driver** (Inland 0.96" 128×64) for live on-board telemetry
- **Single entry-point sketch** (`drone.ino`) that wires everything together

## Hardware

| Component | Notes |
|---|---|
| MCU | ESP32 (uses ESP32-only RMT peripheral and Bluetooth stack) |
| ESC | Any 4-in-1 / single ESC supporting DShot150/300/600 |
| IMU | MPU6050 (I²C) |
| Display | Inland 0.96" SPI OLED (SSD1306-class) |
| Input | Bluetooth gamepad supported by [Bluepad32](https://github.com/ricardoquesada/bluepad32) |

### Default pin map

| Peripheral | Signal | GPIO |
|---|---|---|
| MPU6050 | SDA | 21 |
| MPU6050 | SCL | 22 |
| OLED | (5 pins, SPI-style) | 27, 26, 32, 33, 25 |
| ESC | DShot output | configured via the `ESC` constructor |

Pins are passed into the driver constructors, so you can remap them to fit your wiring.

## Repository layout

```
arc-drone/
├── controller/     # Bluetooth gamepad driver (Bluepad32 wrapper)
├── esc/            # DShot ESC driver
├── mpu6050/        # IMU driver (accel / gyro / temperature)
├── inlandOled/     # SPI OLED display driver
├── example/        # Standalone example sketches per driver
├── drone.ino       # Top-level sketch that ties the drivers together
└── .vscode/        # Editor settings
```

## Getting started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) (or [arduino-cli](https://arduino.github.io/arduino-cli/))
- ESP32 board support package installed via the Arduino Boards Manager
- Required libraries:
  - **Bluepad32** — Bluetooth gamepad input
  - **Adafruit MPU6050** + **Adafruit Unified Sensor** — IMU
  - A DShot library for ESP32 (e.g. `DShotRMT`) — ESC output
  - **U8g2** — OLED rendering

> Exact library names and versions depend on what's `#include`d inside each driver's `.h` / `.cpp`. Open the corresponding folder for specifics.

### Build & flash

1. Clone the repo:
   ```bash
   git clone https://github.com/ArterioRodrigues/arc-drone.git
   ```
2. Open `drone.ino` in the Arduino IDE.
3. Select your ESP32 board and the correct serial port.
4. Click **Upload**.

## Usage

The top-level sketch (`drone.ino`) shows the intended pattern: each driver is constructed at file scope, initialized in `setup()`, and driven from `loop()`.

```cpp
#include "controller/controller.h"
#include "esc/esc.h"
#include "mpu6050/mpu6050.h"
#include "inlandOLED/inlandOLED.h"

drone::Controller controller;
ESC               esc(DSHOT::DSHOT300);
MPU6050           mpu6050;                          // default I²C pins (SDA=21, SCL=22)
InlandOLED        oled(27, 26, 32, 33, 25);

void setup() {
  Serial.begin(115200);
  controller.setup();
  esc.setup();
  mpu6050.setup();
  oled.setup();
}

void loop() {
  controller.processController([]() {
    ControllerPtr ctl = controller.getController();
    if (ctl->isConnected() && ctl->hasData()) {
      esc.sendDShotPacket(1000);                    // throttle value
    }
  });

  sensors_vec_t accel = mpu6050.getAcceleration();
  sensors_vec_t gyro  = mpu6050.getGyro();
  float         temp  = mpu6050.getTemperature();
  // ... render to OLED, run control loop, etc.
}
```

### Driver overview

#### ESC (`esc/`)
Sends DShot frames to a brushless ESC over the ESP32's RMT peripheral.
```cpp
ESC esc(DSHOT::DSHOT300);   // also: DSHOT150, DSHOT600
esc.setup();
esc.sendDShotPacket(1000);  // 0 = disarmed, 48–2047 = throttle range
```

#### Controller (`controller/`)
Thin wrapper around Bluepad32. Pair a gamepad to the ESP32 once, then poll it each loop.
```cpp
drone::Controller controller;
controller.setup();
controller.processController([]() {
  ControllerPtr ctl = controller.getController();
  if (ctl->isConnected() && ctl->hasData()) {
    // read sticks / buttons here
  }
});
```

#### MPU6050 (`mpu6050/`)
I²C IMU driver returning accelerometer / gyroscope vectors and chip temperature.
```cpp
MPU6050 mpu;            // defaults to SDA=21, SCL=22
mpu.setup();
auto accel = mpu.getAcceleration();
auto gyro  = mpu.getGyro();
float t    = mpu.getTemperature();
```

#### InlandOLED (`inlandOled/`)
Buffered drawing API for an SPI 128×64 OLED. Useful for flight-side telemetry.
```cpp
InlandOLED oled(27, 26, 32, 33, 25);
oled.setup();
oled.flipScreen(true);

oled.clearBuffer();
oled.setFont(FONT_SIZE_6);
oled.drawStr(25, 10, "SENSOR READINGS");
oled.drawLine(0, 14, 128, 14);
oled.sendBuffer();
```

## Examples

Standalone sketches for each driver live in `example/`. They're the fastest way to verify wiring before plugging everything into `drone.ino`.

## Status

⚠️ Active development — APIs and pin maps may shift. The current `drone.ino` runs the ESC at a fixed throttle for bench testing; the gamepad → ESC control path and IMU-based stabilization loop are commented out as they're integrated.

## Safety

Always bench-test ESCs and motors **with propellers removed**. Brushless motors at full throttle can cause serious injury. Verify throttle range and arming behavior before mounting props.

## License

No license file is currently included. Add one (MIT / Apache-2.0 / etc.) before others can reuse the code.
