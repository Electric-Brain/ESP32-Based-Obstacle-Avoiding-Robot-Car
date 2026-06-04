# ESP32 Multi-Mode Robot Car – GHOST DRIVE

An ESP32-based robot car project built around the theme of progressive expansion. What started as a simple obstacle-avoiding car has been transformed into a **7-mode intelligent robot car** using WebSocket-based communication and a premium web dashboard.

This repository contains two firmware files:
1. **Standalone Obstacle Avoidance** — [ObstacleAvoidingCar.ino](ObstacleAvoidingCar.ino)
2. **Multi-Mode GHOST DRIVE** — [GHOST_DRIVE_DUAL.ino](GHOST_DRIVE_DUAL.ino) *(main firmware)*

---

## 🎮 7 Planned Car Modes

| # | Mode | Status | Description |
|---|------|--------|-------------|
| 1 | **Obstacle Avoiding** | ✅ Done | Autonomous self-driving with servo scan |
| 2 | **RC Web Control** | ✅ Done | 360° joystick browser control |
| 3 | **Gesture Control** | ✅ Done | Phone IMU tilt steering via WebSocket |
| 4 | **Human Following** | ✅ Done | Servo-lock tracking + follow logic |
| 5 | **Line Following** | 🔜 Planned | IR-based track following |
| 6 | **Light Controlled** | 🔜 Planned | LDR sensor-based steering |
| 7 | **Sound Controlled** | 🔜 Planned | Microphone-triggered commands |

---

## 📂 Firmware Variations

### 1. ObstacleAvoidingCar.ino (Autonomous Only)
A lightweight, standalone firmware focusing solely on self-driving collision avoidance. Uses a servo sweep pattern to look left, right, and center when blocked, then selects the best path.

### 2. GHOST_DRIVE_DUAL.ino — The Main Firmware
Full multi-mode firmware with a Glassmorphism Web Dashboard hosted directly on the ESP32, communicating via WebSocket (port 81) for real-time low-latency control.

#### Implemented Modes:
- **AUTOPILOT** — Autonomous obstacle avoidance with IR cliff detection.
- **RC DRIVE** — Web-based 360° vector joystick. Speed and servo sliders included.
- **TILT (Gesture)** — Tilt your phone to steer! Uses the browser `DeviceOrientation` API. Speed scales linearly with tilt angle (10°–40° deadzone to full speed). Diagonal tilts produce diagonal movement.
- **FOLLOW** — Servo-mounted ultrasonic sensor sweeps 45°–135°, locks onto a target at 15–50 cm, and steers/drives the car to maintain 20–30 cm of tracking distance.

---

## 🚀 Features

*   **Dual Mode Capability**: Switch seamlessly between Autopilot and RC Drive from the web dashboard.
*   **360° Vector Joystick**: Dynamic and fluid movement commands translated instantly over HTTP.
*   **Real-time Canvas Radar**: Fully custom web radar rendering obstacles with warning indicators.
*   **High-frequency PWM**: LEDC configuration running at 18 kHz for silent and smooth motor operation.
*   **Per-Motor Inversion Flags**: Soft-configure motor spin directions directly in code without changing physical wires.
*   **IR Cliff/Edge Detection**: Analog IR sensing on ADC (GPIO 34) stops the car and backs away to prevent falls.
*   **Hardware Conflicts Solved**: Timers allocated dynamically for the servo to prevent LEDC/PWM channel interference.

---

## 🛠 Hardware Used

*   **Microcontroller**: ESP32 DevKit (WROOM-32)
*   **Motor Drivers**: 2× TB6612FNG dual motor drivers
*   **Motors**: 4× DC gear motors
*   **Actuators**: SG90 micro servo (for scanner deck)
*   **Ultrasonic Sensor**: HC-SR04
*   **Edge Sensor**: Analog IR sensor
*   **Power**: 2× 18650 Li-ion cells (7.4V - 8.4V)
*   **Voltage Regulation**: Buck converter (5.0V regulated output for ESP32 and logic)
*   **Buzzer**: Active buzzer (for feedback and horn)
*   **Level Shifting**: Resistor divider for ultrasonic ECHO pin (5V → 3.3V)

---

## 🔌 Wiring / Pin Mapping

### Motor Drivers

#### TB6612 A (Left Motors)
*   **VM** → 7.4V (+)
*   **GND** → GND (-)
*   **AO1 / AO2** → Front Left Motor (+/-)
*   **BO1 / BO2** → Back Left Motor (+/-)
*   **STBY** → GPIO 27
*   **AIN1 / AIN2** → GPIO 16 / GPIO 17
*   **PWMA** → GPIO 14 (Channel 0)
*   **PWMB** → GPIO 22 (Channel 1)
*   **BIN1 / BIN2** → GPIO 19 / GPIO 21

#### TB6612 B (Right Motors)
*   **VM** → 7.4V (+)
*   **GND** → GND (-)
*   **AO1 / AO2** → Front Right Motor (+/-)
*   **BO1 / BO2** → Back Right Motor (+/-)
*   **STBY** → GPIO 27
*   **AIN1 / AIN2** → GPIO 23 / GPIO 25
*   **PWMA** → GPIO 26 (Channel 2)
*   **PWMB** → GPIO 13 (Channel 3)
*   **BIN1 / BIN2** → GPIO 32 / GPIO 33

### Sensors & Peripherals
*   **Servo Control**: GPIO 4
*   **Ultrasonic TRIG**: GPIO 5
*   **Ultrasonic ECHO**: GPIO 18 (use a 1kΩ / 2kΩ voltage divider to step down to 3.3V)
*   **IR Cliff Sensor**: GPIO 34 (Analog)
*   **Buzzer**: GPIO 15
*   **Buck Converter**: 7.4V Input → 5V Output to supply ESP32 and servo rails.

---

## ⚙️ How to Build & Upload

1.  Open the Arduino IDE.
2.  Install the **ESP32** board support package (by Espressif Systems) via Boards Manager.
3.  Install the **ESP32Servo** library.
4.  Open either `ObstacleAvoidingCar.ino` (for autonomous mode only) or `GHOST_DRIVE_DUAL.ino` (for dual-mode).
5.  Set your Board to **ESP32 Dev Module** and choose the correct Port.
6.  Click **Upload**.

---

## 📶 Accessing the Control Deck (For Dual-Mode)

1.  Turn on the robot car.
2.  Search for the WiFi network named **"GhostDrive"** on your phone or computer.
3.  Connect using the password **`12345678`**.
4.  Open your browser and navigate to `http://192.168.4.1/`.
5.  You will be greeted by the **GHOST DRIVE // SCANNER DECK** control panel!

---

## 📄 License

This project is licensed under the MIT License. Feel free to use and adapt it for personal or educational projects!
