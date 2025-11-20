ESP32 Based Obstacle-Avoiding Robot Car
Dual TB6612 • Ultrasonic + IR Sensing • Servo Scanning • LEDC PWM (18 kHz)

This project implements a fully autonomous obstacle-avoiding robot car based on the ESP32 microcontroller.
The system uses dual TB6612FNG motor drivers, a servo-mounted ultrasonic sensor, an IR edge/cliff detector, and high-frequency LEDC PWM for smooth motor control.
This version is fully self-driving.

🚀 Features

Autonomous obstacle avoidance

Servo scanning: left → right → center obstacle detection

Ultrasonic sensing (HC-SR04) with 5-sample median filtering

IR analog cliff detection on ADC pin (GPIO34)

Dual TB6612 motor driver support (4 motors independently driven)

High-frequency LEDC PWM at 18 kHz (quiet and smooth)

Per-motor inversion flags (fix direction without rewiring)

Safe ECHO level shifting (5V → 3.3V)

Adaptive forward speed based on distance

Reverse → scan → pivot → resume workflow

🛠 Hardware Used

ESP32 DevKit (WROOM-32)

2× TB6612FNG dual motor drivers

4× DC gear motors

SG90 micro servo

HC-SR04 ultrasonic sensor

Analog IR sensor

2× 18650 Li-ion cells (7.4–8.4 V)

Buck converter (5.0 V regulated output)

Active buzzer (GPIO controlled)

Resistor divider for ultrasonic ECHO

Common GND (mandatory)

🔌 Wiring / Pin Mapping

##Motor Drivers

##TB6612 A

VM → 7.4V (+)
GND → 7.4 (-)
AO1 → Front Left (+ve)
AO2 → Front Left (-ve)
BO1 → Back Left (+ve)
BO2 → Back Left (-ve)
STBY → GPIO27
AIN1 → GPIO16
AIN2 → GPIO17
PWMA → GPIO14
BIN1 → GPIO19
BIN2 → GPIO21
PWMB → GPIO22


##TB6612 B

VM → 7.4V (+)
GND → 7.4 (-)
AO1 → Front Right (+ve)
AO2 → Front Right (-ve)
BO1 → Back Right (+ve)
BO2 → Back Right (-ve)
STBY → GPIO27
AIN1 → GPIO23
AIN2 → GPIO25
PWMA → GPIO26
BIN1 → GPIO32
BIN2 → GPIO33
PWMB → GPIO13

STBY (both drivers) → GPIO27  (or tie directly to 5V to always enable)

##Servo Motor 
Servo → GPIO4

##Ultrasonic Sensor
Ultrasonic TRIG → GPIO5
Ultrasonic ECHO → 1K → GPIO18 → 2K → GND 

##IR Sensor
IR Sensor (ADC) → GPIO34

##Buzzer
Buzzer → GPIO15

##Buck Converter

I/P → 7.4V (+)
I/P → 7.4 (-)
O/P → 5V (+) → Positive Power Rail
O/P → 5V (-) → Negative Power Rail


•2X 470uF Electrolytic Capacitor Parallel with 7.4V
•4X 0.1uF Ceramic Capacitor Parallel with each Motor Terminal

##⚙️ PWM Configuration (LEDC)

Frequency: 18 kHz
Resolution: 10-bit (0–1023)

Channels:

CH0 → L_PWMA

CH1 → L_PWMB

CH2 → R_PWMA

CH3 → R_PWMB

🤖 Behavior Logic
1. Normal forward movement

Full speed when clear distance ≥ CLEAR_DISTANCE_CM

Reduced speed when semi-blocked

2. Obstacle detected (Ultrasonic ≤ SAFE_DISTANCE_CM)

Stop motor

Beep

Reverse briefly

Servo scans left → right → center

Selects best direction

Pivots to that side

Resumes forward movement

3. Edge/Cliff detected (IR < threshold)

Same behavior as obstacle detection

Prevents falling off edges

📡 Distance Measurement (Median Filter)

Reads 5 ultrasonic samples and returns the median:

d[0..4] = singlePing();
sort(d);
return d[2];   // median


This reduces false readings and noise.




🧪 How to Build & Upload

Install ESP32 by Espressif Systems in Boards Manager

Install ESP32Servo library

Select:

Board: ESP32 Dev Module

Upload Speed: 115200

Flash Frequency: 80 MHz

Connect via CP2102 USB driver

Upload the sketch normally

📄 License

This project is licensed under the MIT License.
Free for personal and commercial use.
