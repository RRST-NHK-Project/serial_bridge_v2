# esp32_serial_bridge

## 1. Overview

esp32_serial_bridge is a reference firmware implementation for microcontrollers used with the serial_bridge ROS 2 package.

This program runs on an ESP32-based microcontroller and communicates with a PC-side ROS 2 node via a custom binary serial protocol.
It receives control commands from ROS 2 and outputs them to motors, servos, and GPIOs, while also sending encoder values and sensor data back to ROS 2.

This firmware is designed for real-time robot hardware control and is actively used in the NHK Project, RRST, at Ritsumeikan University.

---

## 2. Target Hardware

MCU           : ESP32 series
Framework     : Arduino / PlatformIO
Communication : USB Serial (CDC)
Power         : External or USB

Note:
UART is not required. Communication is performed via USB serial.

---

## 3. Features

- Custom binary serial protocol compatible with serial_bridge
- Fixed-length int16_t data frame handling
- Real-time motor and servo control
- Multi-axis encoder reading
- Deterministic periodic control loop
- Designed to be portable to other MCU platforms (STM32, RP2040, etc.)

---

## 4. Firmware Architecture

```
Serial (USB)
 |
 +-- Frame Parser
 |    +-- START / LENGTH / CHECKSUM validation
 |    +-- int16_t payload decoding
 |
 +-- Control Task
 |    +-- DC motor PWM output
 |    +-- RC servo control
 |    +-- GPIO control
 |
 +-- Feedback Task
      +-- Encoder value acquisition
      +-- Sensor reading
      +-- Serial frame transmission
```

## 5. Communication Protocol
```
Frame Structure:

[START][DEVICE_ID][LENGTH][DATA...][CHECKSUM]

START     : Fixed value 0xAA
DEVICE_ID: Unique MCU identifier
LENGTH   : Payload length in bytes
DATA     : int16_t array (little-endian)
CHECKSUM : XOR of all previous bytes

RX (PC -> MCU):
- Motor target values
- Servo angles
- Control flags and parameters

TX (MCU -> PC):
- Encoder values
- Sensor data
- Status or debug values
```
---

## 6. Pin Assignment

Pin configuration depends on the specific robot hardware.
All pin definitions are centralized in the source code for easy modification.

Typical assignments include:
- Motor PWM outputs
- Servo control pins
- Quadrature encoder inputs
- Digital I/O

---

## 7. Integration with ROS 2

This firmware is designed to work with the PC-side ROS 2 package:
serial_bridge/

Ensure the following parameters match on both sides:
- DEVICE_ID
- TX / RX data length
- Frame structure and byte order
- Communication period

---

## 9. Credits
Developed by NHK Project, RRST, Ritsumeikan University, Japan (2024)
- Official Website: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC

![Logo](https://www.rrst.jp/img/logo.png)
