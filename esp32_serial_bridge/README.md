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

## 8. Serial-CAN Bridge Mode (TWAI + MCP2561)

This repository now supports `MODE_CAN_BRIDGE` for a gateway MCU connected to PC via USB serial.

Use case:
- 1 bridge ESP32 is connected to PC
- The bridge converts serial frames to CAN (TWAI)
- Multiple node MCUs are connected on the same CAN bus

Enable in `src/config.hpp`:
- `#define MODE_CAN_BRIDGE`

### 8.1 Bridge Wiring (ESP32 + MCP2561)

- ESP32 `GPIO4` -> MCP2561 `TXD`
- ESP32 `GPIO2` <- MCP2561 `RXD`
- MCP2561 `CANH`, `CANL` -> CAN bus
- Shared GND between all nodes
- Use 120 ohm termination at both ends of CAN bus

CAN pin definitions are in `src/defs.hpp`:
- `CAN_TX = 4`
- `CAN_RX = 2`

### 8.2 Bridge Behavior

- Serial input uses existing frame format:
      `[START][DEVICE_ID][LENGTH][DATA...][CHECKSUM]`
- Bridge validates checksum and forwards to CAN
- CAN response frames are reassembled into serial frame and sent back to PC

### 8.3 CAN Fragment Format (Bridge Internal Protocol)

- CAN ID (bridge -> node): `0x500 + target_device_id`
- CAN ID (node -> bridge): `0x600 + source_device_id`
- Standard 11-bit ID, DLC=8

Data bytes:
- `data[0]`: fragment sequence index (`seq`)
- `data[1]`: total fragment count (`total_chunks`)
- `data[2..7]`: serial-frame payload bytes (6 bytes per fragment)

Notes:
- Serial frame is sent as-is (including `START`, `ID`, `LEN`, `CHECKSUM`)
- TWAI speed is set to 1 Mbps

### 8.4 CAN Client Node (Slave Side)

Node MCU (actuator side) can use CAN as transport while keeping existing control modes.

In `src/config.hpp` on node MCU:
- Select one existing functional mode (example: `MODE_OUTPUT` or `MODE_IO`)
- Set `USE_CAN_CLIENT` to `1`
- Do not enable `MODE_CAN_BRIDGE`

Behavior:
- RX: accepts only CAN ID `0x500 + DEVICE_ID`, reassembles frame, updates `Rx_16Data`
- TX: serial-compatible status frame from `Tx_16Data` is fragmented and sent on `0x600 + DEVICE_ID`

This means one PC-connected bridge MCU can control multiple node MCUs on one CAN bus,
as long as each node has a unique `DEVICE_ID`.

---

## 9. Credits
Developed by NHK Project, RRST, Ritsumeikan University, Japan (2024)
- Official Website: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC

![Logo](https://www.rrst.jp/img/logo.png)
