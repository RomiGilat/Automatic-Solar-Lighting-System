# Automatic Solar Lighting System

The Automatic Solar Lighting System is a smart, low-power outdoor lighting solution designed to intelligently manage energy usage and light control based on environmental conditions. It is optimized for off-grid locations and uses solar energy for sustainable operation. The system combines multiple microcontrollers, sensors, and a cloud backend to deliver reliable, autonomous lighting control.

## Key Features

- Motion-activated lighting via PIR sensors
- Ambient light-based logic with VEML7700 
- Solar-powered system with real-time voltage/current monitoring
- Power-efficient FreeRTOS logic with intelligent cooldown timers
- Firebase integration for remote event logging and control
- Modular MCU-PIC24 + ESP32 architecture with RS-485 comms

---

## System Architecture Overview

### 1. MCU & Sensor Subsystem

This subsystem includes the microcontrollers (PIC24 and ESP32), communication buses, and environmental sensors.


- **PIC24**:
  - Interfaces with PIR sensor(s)
  - Sends motion events over RS-485 to the ESP32

- **ESP32**:
  - Receives trigger messages
  - Executes light control logic
  - Logs events to Firestore
  - Controls GPIOs for lighting
  
[ PIR Sensor(s) ] [ Ambient Light Sensor ] | | [ PIC24 MCU ] ── UART/RS-485 ──> [ ESP32 MCU ]

---

### 2. App & Cloud Database

This layer handles logging and monitoring via Firebase.

[ ESP32 ] ────────> [ Firebase Firestore ] - Motion Events - Power Logs - Remote Config (switch state, test mode, etc.)


- Firebase Firestore:
  - Stores timestamped event logs (motion, power)
  - Supports future expansion to remote control

---

### 3. Power Generation Subsystem

Handles energy harvesting and charging.


[ Solar Panel ] ↓ [ TP4056 ] ←── USB fallback (optional) ↓ [ 3.7V Li-ion Battery ]


- TP4056 regulates charging current and protects battery
- Solar panel provides primary daytime charging

---

### 4. Power Distribution & Monitoring

Regulates and monitors power delivery to system components.

[ Lead-Acid Battery ] ↓ [ ADP1853 Buck Controller Buck Converter ] ↓ ↘ [ 3.3V Rail ] [ INA260 Current/Voltage Sensor ] | Logs power to Firebase through MCU



- INA260 measures system power usage and uploads data periodically
- Buck converter provides stable 3.3V to ESP32 and peripherals
- Modified Sine Wave Inverter provides 220VAC to operate LED Lights 40W/60W/100W
- SPI Controlled Relay Driver communicates between the ESP32 and Modified Sine Wave Inverter to operate relay switches depending on sensor trigger or user override

---

## Directory Structure

Automatic-Solar-Lighting-System/ ├── main/ # ESP32 application source code ├── MCU_CODE/ # PIC24 firmware ├── .devcontainer/ # Development environment configs ├── .vscode/ # Editor configuration ├── sdkconfig # ESP-IDF configuration file ├── CMakeLists.txt # CMake project entry point └── README.md # Project documentation




---

## Getting Started

To build and flash the ESP32 firmware:

```bash
idf.py set-target esp32
idf.py build
idf.py flash monitor
