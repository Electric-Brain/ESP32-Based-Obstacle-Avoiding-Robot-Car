#include <Arduino.h>
#include <ESP32Servo.h>

// ---------------- Pin Mapping ----------------
// Left TB6612 (two motors on left side)
const int L_AIN1 = 16;
const int L_AIN2 = 17;
const int L_PWMA = 14;  // LEDC ch 0
const int L_BIN1 = 19;
const int L_BIN2 = 21;
const int L_PWMB = 22;  // LEDC ch 1

// Right TB6612 (two motors on right side)
const int R_AIN1 = 23;
const int R_AIN2 = 25;
const int R_PWMA = 26;  // LEDC ch 2
const int R_BIN1 = 32;
const int R_BIN2 = 33;
const int R_PWMB = 13;  // LEDC ch 3

// Standby: both TB6612 STBY tied to this GPIO (or tie to 3.3V and comment lines)
const int STBY_PIN = 27;

// Sensors / Actuators
const int SERVO_PIN = 4;
const int TRIG_PIN  = 5;
const int ECHO_PIN  = 18; // MUST be level-shifted to 3.3V
const int IR_PIN    = 34; // analog input-only
const int BUZZER    = 15; // active buzzer (HIGH = beep)

// ---------------- Motor PWM (LEDC) ----------------
const int PWM_FREQ     = 18000; // quiet
const int PWM_RES_BITS = 10;    // 0..1023
const int PWM_MAX      = (1 << PWM_RES_BITS) - 1;

// LEDC channels
const int CH_L_PWMA = 0;
const int CH_L_PWMB = 1;
const int CH_R_PWMA = 2;
const int CH_R_PWMB = 3;

// ---------------- Per-motor inversion ----------------
// If a wheel spins the wrong way when moving FORWARD, flip its flag.
bool L_A_INVERT = true; // Left driver channel A (AO1/AO2)
bool L_B_INVERT = false;  // Left driver channel B (BO1/BO2)  <-- common case: set true
bool R_A_INVERT = true; // Right driver channel A
bool R_B_INVERT = false;  // Right driver channel B           <-- common case: set true

// ---------------- Behavior Tunables ----------------
const int   BASE_SPEED        = 500; // 0..1023
const int   TURN_SPEED        = 300;
const int   REVERSE_SPEED     = 400;
const int   SAFE_DISTANCE_CM  = 30;  // stop/avoid threshold (as requested)
const int   CLEAR_DISTANCE_CM = 18;  // “good to go” distance
const int   TURN_ANGLE_L      = 150; // servo look left
const int   TURN_ANGLE_C      = 90;  // center
const int   TURN_ANGLE_R      = 30;  // right
const int   REVERSE_MS        = 300;
const int   PIVOT_MS          = 260;
const int   STOP_BEEP_MS      = 60;
const int   IR_EDGE_THRESHOLD = 1600; // tune per your IR sensor/surface

// ---------------- Globals ----------------
Servo scanServo;

// ---------------- Utilities ----------------
void beep(int ms) {
  digitalWrite(BUZZER, HIGH);
  delay(ms);
  digitalWrite(BUZZER, LOW);
}

long singlePingCM(unsigned long timeout_us = 20000UL) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, timeout_us);
  if (dur == 0) return 9999;
  return dur / 58; // us -> cm
}

long distanceCM() {
  const int N=5;
  long d[N];
  for (int i=0;i<N;i++){ d[i]=singlePingCM(); delay(8); }
  for (int i=0;i<N-1;i++) for (int j=i+1;j<N;j++) if(d[j]<d[i]) { long t=d[i]; d[i]=d[j]; d[j]=t; }
  return d[N/2];
}

void writeChannel(bool forward, int AIN1, int AIN2, int pwmCh, int pwmVal, bool invert){
  bool f = invert ? !forward : forward;
  digitalWrite(AIN1, f ? HIGH : LOW);
  digitalWrite(AIN2, f ? LOW  : HIGH);
  ledcWrite(pwmCh, pwmVal);
}

// Drive both motors on a side with possible different inversions for A/B
void setSide(int speed,
             int AIN1_A, int AIN2_A, int PWM_A_ch, bool INV_A,
             int AIN1_B, int AIN2_B, int PWM_B_ch, bool INV_B) {
  int mag = abs(speed);
  if (mag > PWM_MAX) mag = PWM_MAX;
  bool forward = (speed >= 0);
  writeChannel(forward, AIN1_A, AIN2_A, PWM_A_ch, mag, INV_A);
  writeChannel(forward, AIN1_B, AIN2_B, PWM_B_ch, mag, INV_B);
}

void driveLeft (int speed){ setSide(speed, L_AIN1,L_AIN2,CH_L_PWMA,L_A_INVERT, L_BIN1,L_BIN2,CH_L_PWMB,L_B_INVERT); }
void driveRight(int speed){ setSide(speed, R_AIN1,R_AIN2,CH_R_PWMA,R_A_INVERT, R_BIN1,R_BIN2,CH_R_PWMB,R_B_INVERT); }

void stopAll(){
  ledcWrite(CH_L_PWMA,0); ledcWrite(CH_L_PWMB,0);
  ledcWrite(CH_R_PWMA,0); ledcWrite(CH_R_PWMB,0);
}

void forward(int s=BASE_SPEED){ driveLeft(+s); driveRight(+s); }
void reverse(int s=REVERSE_SPEED){ driveLeft(-s); driveRight(-s); }
void pivotLeft(int s=TURN_SPEED){ driveLeft(-s); driveRight(+s); }
void pivotRight(int s=TURN_SPEED){ driveLeft(+s); driveRight(-s); }

void aimServo(int angle){ scanServo.write(angle); delay(150); }

int chooseBestTurn(){
  aimServo(TURN_ANGLE_L); long dL = distanceCM();
  aimServo(TURN_ANGLE_R); long dR = distanceCM();
  aimServo(TURN_ANGLE_C);
  return (dL >= dR) ? +1 : -1; // +1 left, -1 right
}

void setup() {
  Serial.begin(115200);

  // Motor control pins
  pinMode(L_AIN1,OUTPUT); pinMode(L_AIN2,OUTPUT);
  pinMode(L_BIN1,OUTPUT); pinMode(L_BIN2,OUTPUT);
  pinMode(R_AIN1,OUTPUT); pinMode(R_AIN2,OUTPUT);
  pinMode(R_BIN1,OUTPUT); pinMode(R_BIN2,OUTPUT);

  // PWM
  ledcSetup(CH_L_PWMA, PWM_FREQ, PWM_RES_BITS); ledcSetup(CH_L_PWMB, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_R_PWMA, PWM_FREQ, PWM_RES_BITS); ledcSetup(CH_R_PWMB, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(L_PWMA, CH_L_PWMA); ledcAttachPin(L_PWMB, CH_L_PWMB);
  ledcAttachPin(R_PWMA, CH_R_PWMA); ledcAttachPin(R_PWMB, CH_R_PWMB);

  // Standby enable
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

  // Sensors / actuators
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  // Servo
  scanServo.setPeriodHertz(50);
  scanServo.attach(SERVO_PIN, 500, 2500);
  aimServo(TURN_ANGLE_C);

  stopAll();
  delay(300);
  beep(100);

  Serial.println("Ready. If a wheel turns wrong way, flip its *_INVERT flag.");
}

void loop() {
  long dC = distanceCM();
  int ir = analogRead(IR_PIN);

  // Edge/cliff protection (IR)
  if (ir < IR_EDGE_THRESHOLD) {
    stopAll(); beep(STOP_BEEP_MS);
    reverse(REVERSE_SPEED); delay(REVERSE_MS);
    int turnDir = chooseBestTurn();
    (turnDir > 0) ? pivotLeft() : pivotRight();
    delay(PIVOT_MS); stopAll();
    return;
  }

  // Ultrasonic avoidance
  if (dC <= SAFE_DISTANCE_CM) {
    stopAll(); beep(STOP_BEEP_MS);
    reverse(REVERSE_SPEED); delay(REVERSE_MS); stopAll();
    int turnDir = chooseBestTurn();
    (turnDir > 0) ? pivotLeft() : pivotRight();
    delay(PIVOT_MS); stopAll();
  } else {
    if (dC >= CLEAR_DISTANCE_CM) forward(BASE_SPEED);
    else forward(BASE_SPEED * 3 / 4);
  }

  delay(20);
}