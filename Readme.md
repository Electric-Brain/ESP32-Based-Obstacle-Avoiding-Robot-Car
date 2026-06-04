# ESP32 Based Obstacle-Avoiding & RC Web Robot Car

This repository contains firmware for a feature-rich ESP32-based robot car. It supports two different control variations:
1. **Fully Autonomous Obstacle Avoidance** ([ObstacleAvoidingCar.ino](file:///C:/Users/Gautam/.gemini/antigravity/scratch/ESP32-Based-Obstacle-Avoiding-Robot-Car/ObstacleAvoidingCar.ino))
2. **Dual-Mode Autopilot + RC Web Car with Premium Glassmorphism Dashboard** ([GHOST_DRIVE_DUAL.ino](file:///C:/Users/Gautam/.gemini/antigravity/scratch/ESP32-Based-Obstacle-Avoiding-Robot-Car/GHOST_DRIVE_DUAL.ino))

Both projects utilize dual TB6612FNG motor drivers (driving 4 motors independently), a servo-mounted ultrasonic scanner, an IR edge/cliff detector, a buzzer, and high-frequency quiet PWM (18 kHz) for smooth motor control.

---

## 📂 Firmware Variations

### 1. ObstacleAvoidingCar.ino (Autonomous Only)
A lightweight firmware focusing solely on self-driving collision avoidance. It uses a servo sweep pattern to look left, right, and center when blocked, selecting the most open path.

### 2. GHOST_DRIVE_DUAL.ino (Dual Mode: Autopilot + RC Web Control)
An advanced, feature-rich firmware featuring a real-time web control panel hosted directly on the ESP32.
*   **Autopilot (Mode A)**: Self-driving with speed adjustment and collision avoidance.
*   **RC Drive (Mode B)**: Controlled via a premium 360-degree vector joystick in your browser.
*   **Glassmorphism Dashboard**: Fully responsive web interface with a futuristic design.
*   **Active Telemetry Radar**: Live canvas-based sweeping radar visualizes obstacle directions with decaying red warning dots.
*   **Live Sensor Statuses**: Range and cliff/edge sensing telemetry update in real-time.
*   **Interactive Controls**: Adjustable drive velocity slider, servo position control slider, and a triggerable horn.
*   **WiFi Integration**: Supports Access Point (AP) mode (creates its own "GhostDrive" network) or client mode (connects to your router).

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
