# Distributed Vehicle Safety & Telematics Network (vECU & CAN Emulation)

An automotive-grade, distributed Electronic Control Unit (ECU) vehicle safety system simulated on ESP32 microcontrollers using FreeRTOS, CAN 2.0B bus protocol emulation, and an end-to-end cloud telematics pipeline connected to HiveMQ MQTT and ThingSpeak IoT.

---

##  Architecture Overview

```
                          ┌────────────────────────┐
                          │  Cloud Telematics Hub  │
                          │ - HiveMQ Public MQTT   │
                          │ - ThingSpeak Dashboard │
                          └───────────▲────────────┘
                                      │ (WiFi / MQTT & REST)
                   ┌──────────────────┴──────────────────┐
                   │  vECU 0: Gateway & Safety Controller│
                   │  - Priority CAN Frame Consumer      │
                   │  - TTC Calculation & Arbitration    │
                   │  - AEB Brake Actuator (Servo Motor) │
                   │  - Audible / Visual Alarm Warning   │
                   └───────────▲───────────────▲─────────┘
                               │               │
       (Simulated CAN Bus /    │               │  (Remote Two-Way
       CAN 2.0B 8-Byte Frames) │               │   Control Command)
                               │               │
      ┌────────────────────────┴─┐   ┌─────────┴────────────────┐
      │  vECU 1: Front ADAS Node │   │  Fleet Monitor Station   │
      │  - Ultrasonic Sensor     │   │  (Second ESP32 Node)     │
      │  - Vehicle Speed Pot     │   │  - I2C OLED Telemetry HUD│
      │  - FreeRTOS 10Hz Sampler │   │  - Remote Brake Overrides│
      └──────────────────────────┘   └──────────────────────────┘
```

---

##  Key Features

- **Automotive CAN 2.0B Bus Protocol**: Implemented deterministic ISO 11898-1 frame structures (`CAN_Frame_t` with 11-bit IDs, DLC, and 8-byte payload packing) across FreeRTOS prioritized mailbox queues.
- **Concurrent FreeRTOS Multitasking**: Pinned tasks across dual Xtensa cores:
  - `Task_ADAS_ECU`: High-rate ($10\text{ Hz}$) distance & wheel speed acquisition with signal serialization (`CAN_ID 0x101`).
  - `Task_SafetyGateway_ECU`: Safety arbitration, computing Time-To-Collision (TTC) to command Autonomous Emergency Braking (AEB) via micro servo caliper clamping and alarm indicators.
  - `Task_Telematics_Cloud`: Non-blocking WiFi management, publishing real-time telemetry over MQTT.
- **Cloud Telematics & Predictive Monitoring**:
  - Live JSON telemetry streamed to HiveMQ public MQTT broker (`vehicle/telematics/live`).
  - Periodic IoT field logging to ThingSpeak (Speed, Distance, Brake State, TTC).
  - Two-way command ingestion enabling remote brake actuation over MQTT.
- **Hardware-in-the-Loop Simulation**: Modeled with Wokwi and PlatformIO for VS Code.

---

##  Hardware & Pin Configuration

| Component | ESP32 GPIO | Description |
|---|---|---|
| **HC-SR04 Trigger** | `GPIO 5` | Front obstacle sonar pulse |
| **HC-SR04 Echo** | `GPIO 18` | Sonar echo reception |
| **Speed Potentiometer** | `GPIO 34` (ADC1) | Simulated vehicle speed (0–140 km/h) |
| **Brake Servo Motor** | `GPIO 13` (PWM) | Mechanical brake caliper ($0^\circ$ released, $90^\circ$ clamped) |
| **Warning Buzzer** | `GPIO 12` | Collision hazard acoustic alert |
| **Red Alert LED** | `GPIO 2` | Emergency braking indicator |
| **Green OK LED** | `GPIO 15` | Normal system status indicator |

---

## 📊 CAN Frame Specifications

### Frame 1: ADAS Front Perception (`0x101`)
- **DLC:** 8 bytes
- **Byte 0–1:** Distance in cm ($\times 10$ resolution)
- **Byte 2–3:** Speed in km/h ($\times 10$ resolution)
- **Byte 4:** Hazard Flag (`0x01` if distance $< 25\text{ cm}$, else `0x00`)
- **Byte 5–7:** Reserved

### Frame 2: Brake Actuator Status (`0x050`)
- **DLC:** 2 bytes
- **Byte 0:** Brake state (`0x00` = Released, `0x02` = Emergency Actuated)
- **Byte 1:** Time-To-Collision in seconds

---

##  How to Run Locally

### Prerequisites
- [VS Code](https://code.visualstudio.com/) with **PlatformIO IDE** and **Wokwi Simulator** extensions installed.

### Steps
1. Clone the repository:
   ```bash
   git clone https://github.com/<your-username>/distributed-ecu-telematics-network.git
   cd distributed-ecu-telematics-network
   ```
2. Build the firmware:
   ```bash
   pio run
   ```
3. Open `diagram.json` in VS Code and press **F1** $\rightarrow$ `Wokwi: Start Simulator` (or click the green Play icon).
4. View live CAN traces in the Serial Monitor and test obstacle collision by dragging the HC-SR04 sensor target.
#
