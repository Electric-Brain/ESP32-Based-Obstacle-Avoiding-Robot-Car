# ESP32-Based Obstacle Avoiding Robot Car

An ESP32-based autonomous robot car that steers away from obstacles using an SG90 servo-mounted HC-SR04 ultrasonic sensor and incorporates edge protection.

This repository contains the standalone autonomous firmware for the robot car.

---

## 📂 Firmware

The primary firmware is:
* **[ObstacleAvoidingCar.ino](ObstacleAvoidingCar.ino)** — Standalone obstacle avoidance logic that sweeps a servo-mounted ultrasonic sensor to scan the surroundings, detects edges (cliffs), and navigates the environment autonomously.

---

## 🚀 Obstacle Avoidance Behavior

When the car is driving forward:
1. It continuously monitors the front distance using the ultrasonic sensor.
2. If an obstacle is detected within 30 cm:
   - It stops and backs up slightly.
   - It rotates the servo-mounted ultrasonic sensor to scan 150° (Left) and 30° (Right).
   - It compares the distances and turns the car towards the direction with the clearest path (Left or Right).
   - It centers the scanner and continues driving forward.
3. If the analog IR sensor detects a cliff (distance to ground increases significantly, value falls below threshold):
   - It instantly stops, reverses, and steers away from the edge to protect the chassis.

---

## 🛠 Hardware Used

* **Microcontroller**: ESP32 DevKit (WROOM-32)
* **Motor Drivers**: 2× TB6612FNG dual motor drivers
* **Motors**: 4× DC gear motors
* **Actuators**: SG90 micro servo (for scanner deck)
* **Ultrasonic Sensor**: HC-SR04
* **Edge Sensor**: Analog IR sensor
* **Power**: 2× 18650 Li-ion cells (7.4V - 8.4V)
* **Voltage Regulation**: Buck converter (5.0V regulated output for ESP32 and servo logic)
* **Buzzer**: Active buzzer (for backing alerts and warnings)
* **Level Shifting**: Resistor divider for ultrasonic ECHO pin (5V → 3.3V)

---

## 🔌 Wiring / Pin Mapping

### Motor Drivers

#### TB6612 A (Left Motors)
* **VM** → 7.4V (+)
* **GND** → GND (-)
* **AO1 / AO2** → Front Left Motor (+/-)
* **BO1 / BO2** → Back Left Motor (+/-)
* **STBY** → GPIO 27
* **AIN1 / AIN2** → GPIO 16 / GPIO 17
* **PWMA** → GPIO 14 (Channel 0)
* **PWMB** → GPIO 22 (Channel 1)
* **BIN1 / BIN2** → GPIO 19 / GPIO 21

#### TB6612 B (Right Motors)
* **VM** → 7.4V (+)
* **GND** → GND (-)
* **AO1 / AO2** → Front Right Motor (+/-)
* **BO1 / BO2** → Back Right Motor (+/-)
* **STBY** → GPIO 27
* **AIN1 / AIN2** → GPIO 23 / GPIO 25
* **PWMA** → GPIO 26 (Channel 2)
* **PWMB** → GPIO 13 (Channel 3)
* **BIN1 / BIN2** → GPIO 32 / GPIO 33

### Sensors & Peripherals
* **Servo Control**: GPIO 4
* **Ultrasonic TRIG**: GPIO 5
* **Ultrasonic ECHO**: GPIO 18 (use a 1kΩ / 2kΩ voltage divider to step down to 3.3V)
* **IR Cliff Sensor**: GPIO 34 (Analog)
* **Buzzer**: GPIO 15
* **Buck Converter**: 7.4V Input → 5V Output to supply ESP32 and servo rails.

---

## ⚙️ How to Build & Upload

1. Open the Arduino IDE.
2. Install the **ESP32** board support package (by Espressif Systems) via Boards Manager.
3. Install the **ESP32Servo** library.
4. Open `ObstacleAvoidingCar.ino`.
5. Set your Board to **ESP32 Dev Module** and choose the correct Port.
6. Click **Upload**.

---

## 📄 License

This project is licensed under the MIT License. Feel free to use and adapt it for personal or educational projects!
