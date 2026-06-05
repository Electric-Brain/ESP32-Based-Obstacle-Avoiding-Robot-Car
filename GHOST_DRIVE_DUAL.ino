/*  ============================================================
    DUAL-MODE OBSTACLE-AVOIDING + RC WEB CAR
    ESP32  |  TB6612 x2  |  Servo  |  HC-SR04  |  IR  |  Buzzer
    ============================================================
    MODE A  –  Obstacle Avoidance (autonomous, slow speed)
    MODE B  –  RC Web Control     (WiFi browser joystick, fast)
    How to switch modes
    -------------------
    1. Connect to the ESP32's WiFi AP ("GhostDrive") or router
    2. Open http://<IP>/ in your phone or computer browser
    3. Enjoy the premium futuristic Glassmorphism Control Panel!
    ============================================================ */
#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
// ─── WiFi Configuration ──────────────────────────────────────
#define WIFI_AP_MODE true  // true = creates own hotspot, false = connects to router
const char* WIFI_SSID = "GhostDrive";
const char* WIFI_PASS = "12345678";
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
// ─── Mode ────────────────────────────────────────────────────
enum CarMode { MODE_OA, MODE_RC, MODE_GESTURE, MODE_FOLLOW };
CarMode carMode = MODE_OA;   // default: OA on boot
// ─── Pin Mapping ─────────────────────────────────────────────
// Left TB6612 (two motors on left side)
const int L_AIN1 = 16, L_AIN2 = 17, L_PWMA = 14;
const int L_BIN1 = 19, L_BIN2 = 21, L_PWMB = 22;
// Right TB6612 (two motors on right side)
const int R_AIN1 = 23, R_AIN2 = 25, R_PWMA = 26;
const int R_BIN1 = 32, R_BIN2 = 33, R_PWMB = 13;
const int STBY_PIN   = 27;
const int SERVO_PIN  =  4;
const int TRIG_PIN   =  5;
const int ECHO_PIN   = 18;   // level-shift to 3.3 V!
const int IR_PIN     = 34;   // analog input-only
const int BUZZER     = 15;
// ─── Motor PWM (LEDC Channels) ───────────────────────────────
const int PWM_FREQ     = 18000; // quiet high frequency
const int PWM_RES_BITS = 10;    // 0..1023
const int PWM_MAX      = (1 << PWM_RES_BITS) - 1;
// Core v2.x Channel numbers (Ignored on Core v3.x)
const int CH_L_PWMA = 0;
const int CH_L_PWMB = 1;
const int CH_R_PWMA = 2;
const int CH_R_PWMB = 3;
// ─── Per-motor inversion ──────────────────────────────────────
bool L_A_INVERT = true;
bool L_B_INVERT = false;
bool R_A_INVERT = true;
bool R_B_INVERT = false;
// ─── Speed constants ──────────────────────────────────────────
const int OA_SPEED      = 380;   // OA forward (0-1023) - Reduced for safety
const int OA_TURN       = 280;   // OA pivot - Reduced for safety
const int OA_REVERSE    = 320;   // OA reverse - Reduced for safety
const int RC_SPEED_DEF  = 800;   // RC default speed (faster)
const int RC_TURN_DEF   = 650;   // RC default pivot
// RC runtime speed (changed by slider)
int rcSpeed = RC_SPEED_DEF;
// ─── OA Behaviour ─────────────────────────────────────────────
const int SAFE_DIST_CM  = 30;
const int CLEAR_DIST_CM = 18;
const int TURN_ANGLE_L  = 150;
const int TURN_ANGLE_C  =  90;
const int TURN_ANGLE_R  =  30;
const int REVERSE_MS    = 300;
const int PIVOT_MS      = 260;
const int IR_THRESHOLD  = 1600;
// ─── RC state ────────────────────────────────────────────────
int  rcServoAngle = 90;
bool rcHornOn     = false;
char rcCmd[4]     = "S";  // Drive command: F, B, L, R, FL, FR, BL, BR, S
// ─── Telemetry Globals ────────────────────────────────────────
volatile long lastDistance = 999;
volatile int lastIR = 4095;
int currentServoAngle = 90;
volatile long dL_dist = 999;
volatile long dC_dist = 999;
volatile long dR_dist = 999;
// ─── Actuator Globals ─────────────────────────────────────────
Servo scanServo;
// ─── Human Follower state ────────────────────────────────────
int followAngle = 90;
bool followFound = false;
int followSweepDir = 8;
unsigned long lastFollowTime = 0;
unsigned long lostFollowTime = 0;
// ─────────────────────────────────────────────────────────────
//  MOTOR DRIVER HELPERS (Compatible with Core v2.x and v3.x)
// ─────────────────────────────────────────────────────────────
void writeChannel(bool forward, int AIN1, int AIN2, int pwmPin, int pwmCh, int pwmVal, bool invert) {
  bool f = invert ? !forward : forward;
  digitalWrite(AIN1, f ? HIGH : LOW);
  digitalWrite(AIN2, f ? LOW  : HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(pwmPin, pwmVal);
#else
  ledcWrite(pwmCh, pwmVal);
#endif
}
void setSide(int speed,
             int AIN1_A, int AIN2_A, int PWM_A_pin, int PWM_A_ch, bool INV_A,
             int AIN1_B, int AIN2_B, int PWM_B_pin, int PWM_B_ch, bool INV_B) {
  int mag = abs(speed);
  if (mag > PWM_MAX) mag = PWM_MAX;
  bool fwd = (speed >= 0);
  writeChannel(fwd, AIN1_A, AIN2_A, PWM_A_pin, PWM_A_ch, mag, INV_A);
  writeChannel(fwd, AIN1_B, AIN2_B, PWM_B_pin, PWM_B_ch, mag, INV_B);
}
void driveLeft (int s){ setSide(s, L_AIN1, L_AIN2, L_PWMA, CH_L_PWMA, L_A_INVERT, L_BIN1, L_BIN2, L_PWMB, CH_L_PWMB, L_B_INVERT); }
void driveRight(int s){ setSide(s, R_AIN1, R_AIN2, R_PWMA, CH_R_PWMA, R_A_INVERT, R_BIN1, R_BIN2, R_PWMB, CH_R_PWMB, R_B_INVERT); }
void stopAll() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(L_PWMA, 0); ledcWrite(L_PWMB, 0);
  ledcWrite(R_PWMA, 0); ledcWrite(R_PWMB, 0);
#else
  ledcWrite(CH_L_PWMA, 0); ledcWrite(CH_L_PWMB, 0);
  ledcWrite(CH_R_PWMA, 0); ledcWrite(CH_R_PWMB, 0);
#endif
}
void forward   (int s){ driveLeft(+s); driveRight(+s); }
void reverse   (int s){ driveLeft(-s); driveRight(-s); }
void pivotLeft (int s){ driveLeft(-s); driveRight(+s); }
void pivotRight(int s){ driveLeft(+s); driveRight(-s); }
// ─────────────────────────────────────────────────────────────
//  SENSOR / ACTUATOR HELPERS
// ─────────────────────────────────────────────────────────────
void beep(int ms) {
  digitalWrite(BUZZER, HIGH);
  delay(ms);
  digitalWrite(BUZZER, LOW);
}
long singlePingCM(unsigned long timeout_us = 20000UL) {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, timeout_us);
  return dur == 0 ? 9999 : dur / 58;
}
long distanceCM() {
  const int N = 5;
  long d[N];
  for (int i = 0; i < N; i++) { d[i] = singlePingCM(); delay(8); }
  for (int i = 0; i < N-1; i++)
    for (int j = i+1; j < N; j++)
      if (d[j] < d[i]) { long t = d[i]; d[i] = d[j]; d[j] = t; }
  return d[N/2];
}
void aimServo(int angle, bool wait = true) {
  if (angle != currentServoAngle) {
    scanServo.write(angle);
    currentServoAngle = angle;
    if (wait) {
      delay(150); // delay only when waiting is needed (avoid blocking RC loop)
    }
  }
}
int chooseBestTurn() {
  aimServo(TURN_ANGLE_L, true); dL_dist = distanceCM();
  aimServo(TURN_ANGLE_R, true); dR_dist = distanceCM();
  aimServo(TURN_ANGLE_C, true); dC_dist = distanceCM();
  return (dL_dist >= dR_dist) ? +1 : -1;
}
void applyRcCmd(const char* cmd, int spd) {
  int t = spd * 75 / 100; // soft turn speed coefficient
  if      (strcmp(cmd, "F")  == 0) { forward(spd); }
  else if (strcmp(cmd, "B")  == 0) { reverse(spd); }
  else if (strcmp(cmd, "L")  == 0) { pivotLeft(t); }
  else if (strcmp(cmd, "R")  == 0) { pivotRight(t); }
  else if (strcmp(cmd, "FL") == 0) { driveLeft(spd); driveRight(t); }
  else if (strcmp(cmd, "FR") == 0) { driveLeft(t);   driveRight(spd); }
  else if (strcmp(cmd, "BL") == 0) { driveLeft(-t);  driveRight(-spd); }
  else if (strcmp(cmd, "BR") == 0) { driveLeft(-spd);driveRight(-t); }
  else                             { stopAll(); }
}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %s\n", num, ip.toString().c_str());
      break;
    }
    case WStype_TEXT: {
      String msg = String((char*)payload);
      if (msg.startsWith("g:")) {
        int colon1 = msg.indexOf(':', 2);
        if (colon1 != -1) {
          String cmd = msg.substring(2, colon1);
          int speed = msg.substring(colon1 + 1).toInt();
          if (carMode == MODE_GESTURE) {
            applyRcCmd(cmd.c_str(), speed);
          }
        }
      } else if (msg.startsWith("cmd:")) {
        String cmd = msg.substring(4);
        if (carMode == MODE_RC) {
          strlcpy(rcCmd, cmd.c_str(), sizeof(rcCmd));
        }
      } else if (msg.startsWith("speed:")) {
        int pct = msg.substring(6).toInt();
        pct = constrain(pct, 10, 100);
        rcSpeed = (int)(RC_SPEED_DEF * pct / 100);
      } else if (msg.startsWith("servo:")) {
        rcServoAngle = constrain(msg.substring(6).toInt(), 0, 180);
      } else if (msg.startsWith("horn:")) {
        rcHornOn = (msg.substring(5) == "1");
      } else if (msg.startsWith("mode:")) {
        String m = msg.substring(5);
        stopAll();
        strlcpy(rcCmd, "S", sizeof(rcCmd));
        if (m == "rc") {
          carMode = MODE_RC;
          beep(80);
        } else if (m == "gesture") {
          carMode = MODE_GESTURE;
          beep(80);
        } else if (m == "follow") {
          carMode = MODE_FOLLOW;
          beep(80);
        } else {
          carMode = MODE_OA;
          beep(80); delay(80); beep(80);
          rcServoAngle = TURN_ANGLE_C;
        }
      }
      break;
    }
    default:
      break;
  }
}
void handleRoot() {
  String page = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no'>
  <title>GHOST DRIVE // SCANNER DECK</title>
  <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@400;600;700;800&family=JetBrains+Mono:wght@700&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-color: #06070d;
      --card-bg: rgba(16, 18, 35, 0.55);
      --card-border: rgba(255, 255, 255, 0.05);
      --accent-primary: #6366f1; /* Indigo for RC */
      --accent-primary-glow: rgba(99, 102, 241, 0.25);
      --accent-success: #10b981; /* Emerald for Autopilot */
      --accent-success-glow: rgba(16, 185, 129, 0.25);
      --accent-gesture: #06b6d4; /* Cyan for Gesture Control */
      --accent-gesture-glow: rgba(6, 182, 212, 0.25);
      --accent-follow: #a855f7; /* Purple for Human Following */
      --accent-follow-glow: rgba(168, 85, 247, 0.25);
      --accent-danger: #f43f5e; /* Rose for Alerts/Horn */
      --accent-danger-glow: rgba(244, 63, 94, 0.3);
      --text-main: #f9fafb;
      --text-muted: #6b7280;
      
      /* Dynamic theme targets */
      --accent-active: var(--accent-success);
      --accent-active-glow: var(--accent-success-glow);
    }
    
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      -webkit-tap-highlight-color: transparent;
      user-select: none;
    }
    
    body {
      font-family: 'Outfit', sans-serif;
      background-color: var(--bg-color);
      color: var(--text-main);
      background-image: 
      	radial-gradient(circle at 10% 20%, rgba(99, 102, 241, 0.12) 0%, transparent 40%),
      	radial-gradient(circle at 90% 80%, rgba(16, 185, 129, 0.08) 0%, transparent 40%);
      background-attachment: fixed;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 16px;
      overflow-x: hidden;
    }
    .app-container {
      width: 100%;
      max-width: 440px;
      display: flex;
      flex-direction: column;
      gap: 16px;
    }
    /* Active theme modifiers */
    .app-container.theme-oa {
      --accent-active: var(--accent-success);
      --accent-active-glow: var(--accent-success-glow);
    }
    .app-container.theme-rc {
      --accent-active: var(--accent-primary);
      --accent-active-glow: var(--accent-primary-glow);
    }
    .app-container.theme-gesture {
      --accent-active: var(--accent-gesture);
      --accent-active-glow: var(--accent-gesture-glow);
    }
    .app-container.theme-follow {
      --accent-active: var(--accent-follow);
      --accent-active-glow: var(--accent-follow-glow);
    }
    /* Header styling */
    .header {
      width: 100%;
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 4px 6px;
    }
    
    .logo-area {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    
    .pulse-indicator {
      width: 8px;
      height: 8px;
      background-color: var(--accent-active);
      border-radius: 50%;
      box-shadow: 0 0 8px var(--accent-active);
      transition: all 0.3s ease;
      animation: pulse 1.8s infinite;
    }
    
    @keyframes pulse {
      0% { transform: scale(0.95); opacity: 0.6; }
      50% { transform: scale(1.15); opacity: 1; }
      100% { transform: scale(0.95); opacity: 0.6; }
    }
    
    .logo-title {
      font-size: 13px;
      font-weight: 800;
      letter-spacing: 0.16em;
      background: linear-gradient(135deg, #ffffff 30%, rgba(255,255,255,0.6) 100%);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
    }
    
    .connection-badge {
      font-size: 9px;
      font-weight: 700;
      letter-spacing: 0.08em;
      padding: 5px 10px;
      border-radius: 20px;
      background: rgba(16, 185, 129, 0.08);
      color: var(--accent-success);
      border: 1px solid rgba(16, 185, 129, 0.2);
      transition: all 0.3s ease;
    }
    /* Cards */
    .card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 24px;
      padding: 20px;
      box-shadow: 0 10px 30px rgba(0, 0, 0, 0.25);
      backdrop-filter: blur(20px);
      -webkit-backdrop-filter: blur(20px);
      transition: border-color 0.3s ease;
    }
    
    .card-title {
      font-size: 10px;
      font-weight: 800;
      letter-spacing: 0.12em;
      color: var(--text-muted);
      text-transform: uppercase;
      margin-bottom: 14px;
    }
    /* Radar Section centerpiece */
    .radar-wrapper {
      position: relative;
      width: 200px;
      height: 200px;
      margin: 0 auto 16px;
    }
    
    #radar-canvas {
      display: block;
      background: transparent;
      width: 200px;
      height: 200px;
    }
    /* Sliding Mode Selector */
    .mode-selector {
      position: relative;
      display: flex;
      background: rgba(16, 18, 35, 0.7);
      border: 1px solid var(--card-border);
      border-radius: 18px;
      padding: 4px;
    }
    
    .mode-tab {
      position: relative;
      flex: 1;
      background: none;
      border: none;
      color: var(--text-muted);
      padding: 12px 6px;
      font-family: inherit;
      font-size: 9px;
      font-weight: 700;
      letter-spacing: 0.08em;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 4px;
      z-index: 2;
      transition: color 0.3s ease;
    }
    
    .mode-tab.active {
      color: var(--text-main);
    }
    
    .tab-icon {
      width: 13px;
      height: 13px;
      fill: none;
      stroke: currentColor;
      stroke-width: 2.2;
    }
    
    .tab-glider {
      position: absolute;
      left: 4px;
      top: 4px;
      width: calc(25% - 4px);
      height: calc(100% - 8px);
      background: linear-gradient(135deg, var(--accent-success) 0%, rgba(16, 185, 129, 0.8) 100%);
      border-radius: 14px;
      z-index: 1;
      transition: all 0.35s cubic-bezier(0.25, 1, 0.4, 1);
      box-shadow: 0 4px 12px var(--accent-success-glow);
    }
    
    .mode-selector.rc-active .tab-glider {
      transform: translateX(100%);
      background: linear-gradient(135deg, var(--accent-primary) 0%, rgba(99, 102, 241, 0.8) 100%);
      box-shadow: 0 4px 12px var(--accent-primary-glow);
    }
    .mode-selector.gesture-active .tab-glider {
      transform: translateX(200%);
      background: linear-gradient(135deg, var(--accent-gesture) 0%, rgba(6, 182, 212, 0.8) 100%);
      box-shadow: 0 4px 12px var(--accent-gesture-glow);
    }
    .mode-selector.follow-active .tab-glider {
      transform: translateX(300%);
      background: linear-gradient(135deg, var(--accent-follow) 0%, rgba(168, 85, 247, 0.8) 100%);
      box-shadow: 0 4px 12px var(--accent-follow-glow);
    }
    /* Telemetry Panel */
    .metrics-container {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(110px, 1fr));
      gap: 12px;
    }
    
    .metric-widget {
      background: rgba(30, 35, 60, 0.25);
      border: 1px solid rgba(255, 255, 255, 0.02);
      border-radius: 16px;
      padding: 12px 14px;
      display: flex;
      flex-direction: column;
      gap: 4px;
      position: relative;
      overflow: hidden;
    }
    
    .metric-icon-wrap {
      position: absolute;
      right: 10px;
      top: 10px;
      opacity: 0.05;
      color: var(--text-main);
    }
    
    .metric-icon-wrap svg {
      width: 26px;
      height: 26px;
      stroke: currentColor;
      stroke-width: 2;
      fill: none;
    }
    
    .metric-label {
      font-size: 8px;
      font-weight: 800;
      letter-spacing: 0.1em;
      color: var(--text-muted);
    }
    
    .metric-value {
      font-size: 16px;
      font-weight: 700;
      font-family: 'JetBrains Mono', monospace;
    }
    
    .gauge-bar-container {
      width: 100%;
      height: 3px;
      background: rgba(255, 255, 255, 0.06);
      border-radius: 2px;
      overflow: hidden;
      margin-top: 6px;
    }
    
    .gauge-bar {
      height: 100%;
      background: var(--accent-active);
      border-radius: 2px;
      transition: width 0.30s ease, background-color 0.30s ease;
    }
    
    .status-safe { color: var(--accent-success); }
    .status-warning { color: #f59e0b; }
    .status-danger { 
      color: var(--accent-danger); 
      animation: alert-flash 0.8s ease infinite alternate; 
    }
    
    @keyframes alert-flash {
      from { opacity: 0.5; }
      to { opacity: 1; }
    }
    /* Tab Pages */
    .panel-oa, .panel-rc, .panel-gesture, .panel-follow {
      display: none;
    }
    
    .panel-oa.active, .panel-rc.active, .panel-gesture.active, .panel-follow.active {
      display: block;
      animation: tab-switch-in 0.3s cubic-bezier(0.25, 1, 0.5, 1) forwards;
    }
    
    @keyframes tab-switch-in {
      from { opacity: 0; transform: translateY(6px); }
      to { opacity: 1; transform: translateY(0); }
    }
    
    .oa-status-msg {
      text-align: center;
      font-size: 11px;
      color: var(--text-muted);
      line-height: 1.6;
      padding: 10px 15px;
    }
    /* RC Controls Joystick Layout */
    .controls-layout {
      display: flex;
      flex-direction: column;
      gap: 20px;
    }
    
    .joystick-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 8px;
    }
    
    .joystick-zone {
      width: 100%;
      height: 180px;
      display: flex;
      align-items: center;
      justify-content: center;
      position: relative;
      touch-action: none;
    }
    
    .joystick-base {
      width: 150px;
      height: 150px;
      background: rgba(30, 35, 60, 0.2);
      border: 1.5px dashed rgba(255, 255, 255, 0.08);
      border-radius: 50%;
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: inset 0 4px 12px rgba(0, 0, 0, 0.3);
    }
    
    .joystick-handle {
      width: 54px;
      height: 54px;
      background: radial-gradient(circle at 30% 30%, #ffffff 0%, var(--accent-active) 85%);
      border-radius: 50%;
      position: absolute;
      cursor: grab;
      box-shadow: 0 8px 20px rgba(0, 0, 0, 0.4), 0 0 15px var(--accent-active-glow);
      transition: background-color 0.3s ease;
      touch-action: none;
    }
    
    .joystick-handle:active {
      cursor: grabbing;
      background: radial-gradient(circle at 30% 30%, #ffffff 0%, #ffffff 100%);
      box-shadow: 0 4px 10px rgba(0,0,0,0.4), 0 0 25px #ffffff;
    }
    /* Sliders and Action Buttons */
    .actions-panel {
      display: flex;
      flex-direction: column;
      gap: 16px;
    }
    
    .btn-horn {
      width: 100%;
      padding: 14px;
      background: rgba(244, 63, 94, 0.08);
      border: 1px solid rgba(244, 63, 94, 0.2);
      border-radius: 16px;
      color: var(--accent-danger);
      font-family: inherit;
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.08em;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      outline: none;
      transition: all 0.15s ease;
    }
    
    .btn-horn svg {
      width: 16px;
      height: 16px;
      fill: none;
      stroke: currentColor;
      stroke-width: 2.2;
    }
    
    .btn-horn:active, .btn-horn.active {
      background: var(--accent-danger);
      border-color: var(--accent-danger);
      color: #ffffff;
      transform: scale(0.97);
      box-shadow: 0 0 16px var(--accent-danger-glow);
    }
    
    .slider-group {
      display: flex;
      flex-direction: column;
      gap: 6px;
    }
    
    .slider-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 0 2px;
    }
    
    .slider-title {
      display: flex;
      align-items: center;
      gap: 6px;
      font-size: 9px;
      font-weight: 800;
      letter-spacing: 0.08em;
      color: var(--text-muted);
    }
    
    .slider-icon {
      width: 13px;
      height: 13px;
      fill: none;
      stroke: currentColor;
      stroke-width: 2.2;
    }
    
    .slider-value {
      font-size: 11px;
      font-weight: 700;
      color: var(--accent-active);
      font-family: 'JetBrains Mono', monospace;
      transition: color 0.3s ease;
    }
    
    .range-input-wrap {
      padding: 4px 0;
    }
    
    input[type='range'] {
      -webkit-appearance: none;
      width: 100%;
      height: 5px;
      background: rgba(255, 255, 255, 0.07);
      border-radius: 4px;
      outline: none;
      border: none;
    }
    
    input[type='range']::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 18px;
      height: 18px;
      border-radius: 50%;
      background: var(--accent-active);
      box-shadow: 0 0 8px var(--accent-active-glow);
      cursor: pointer;
      border: 2.5px solid var(--bg-color);
      transition: transform 0.12s, background-color 0.12s, border-color 0.12s;
    }
    
    input[type='range']::-webkit-slider-thumb:hover {
      transform: scale(1.15);
    }
    
    input[type='range']::-webkit-slider-thumb:active {
      background: #ffffff;
      border-color: var(--accent-active);
      box-shadow: 0 0 12px #ffffff;
    }
    /* D-pad fallback buttons */
    .dpad-btn {
      background: rgba(6, 182, 212, 0.07);
      border: 1.5px solid rgba(6, 182, 212, 0.18);
      border-radius: 12px;
      color: var(--accent-gesture);
      font-size: 18px;
      height: 52px;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      outline: none;
      font-family: inherit;
      transition: background 0.12s, transform 0.1s;
      touch-action: none;
      -webkit-tap-highlight-color: transparent;
    }
    .dpad-btn:active {
      background: rgba(6, 182, 212, 0.25);
      transform: scale(0.93);
      box-shadow: 0 0 12px rgba(6, 182, 212, 0.3);
    }
    /* RC Transmitter Control Buttons */
    .rc-ctrl-btn {
      background: rgba(99, 102, 241, 0.07);
      border: 1.5px solid rgba(99, 102, 241, 0.22);
      border-radius: 14px;
      color: var(--accent-primary);
      font-size: 20px;
      font-weight: 800;
      height: 72px;
      cursor: pointer;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 2px;
      outline: none;
      font-family: inherit;
      transition: background 0.1s, transform 0.08s, box-shadow 0.1s;
      touch-action: none;
      -webkit-tap-highlight-color: transparent;
      user-select: none;
      width: 100%;
    }
    .rc-ctrl-btn small { font-size: 8px; font-weight: 700; letter-spacing: 0.08em; opacity: 0.7; }
    .rc-ctrl-btn:active, .rc-ctrl-btn.pressed {
      background: rgba(99, 102, 241, 0.28);
      transform: scale(0.91);
      box-shadow: 0 0 18px rgba(99, 102, 241, 0.4);
    }
    .rc-stop-btn {
      background: rgba(244, 63, 94, 0.07);
      border-color: rgba(244, 63, 94, 0.22);
      color: var(--accent-danger);
    }
    .rc-stop-btn:active, .rc-stop-btn.pressed {
      background: rgba(244, 63, 94, 0.28);
      box-shadow: 0 0 18px rgba(244, 63, 94, 0.4);
    }
  </style>
</head>
<body>
<div class='app-container theme-oa'>
  
  <!-- Top Header Deck -->
  <header class='header'>
    <div class='logo-area'>
      <span class='pulse-indicator'></span>
      <h1 class='logo-title'>GHOST DRIVE</h1>
    </div>
    <div class='connection-badge' id='conn-badge'>READY</div>
  </header>
  <!-- Radar centerpiece card -->
  <section class='card'>
    <h2 class='card-title'>ACTIVE TELEMETRY SCANNER</h2>
    <div class='radar-wrapper'>
      <canvas id='radar-canvas' width='200' height='200'></canvas>
    </div>
    <div class='metrics-container'>
      <!-- Distance Widget -->
      <div class='metric-widget'>
        <div class='metric-icon-wrap'>
          <svg viewBox='0 0 24 24'><path d='M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z'/></svg>
        </div>
        <span class='metric-label'>LIDAR RANGE</span>
        <span class='metric-value' id='val-dist'>-- cm</span>
        <div class='gauge-bar-container'>
          <div class='gauge-bar' id='dist-gauge' style='width: 100%'></div>
        </div>
      </div>
      <!-- Cliff Widget -->
      <div class='metric-widget'>
        <div class='metric-icon-wrap'>
          <svg viewBox='0 0 24 24'><path d='M12 2L2 22h20L12 2zm0 14h.01M12 10v4'/></svg>
        </div>
        <span class='metric-label'>CLIFF SCAN</span>
        <span class='metric-value status-safe' id='val-cliff'>SAFE</span>
        <div class='gauge-bar-container'>
          <div class='gauge-bar' id='cliff-gauge' style='width: 100%'></div>
        </div>
      </div>
      <!-- Vector Widget -->
      <div class='metric-widget'>
        <div class='metric-icon-wrap'>
          <svg viewBox='0 0 24 24'><polygon points='12 2 22 8.5 22 15.5 12 22 2 15.5 2 8.5 12 2'/></svg>
        </div>
        <span class='metric-label'>VECTOR CHN</span>
        <span class='metric-value' id='val-cmd'>STOP</span>
        <div class='gauge-bar-container'>
          <div class='gauge-bar' id='vector-gauge' style='width: 100%'></div>
        </div>
      </div>
    </div>
  </section>
  
  <!-- Sliding Selector Tab -->
  <div class='mode-selector'>
    <button class='mode-tab active' id='tab-oa' onclick='changeMode("oa")'>
      <svg class='tab-icon' viewBox='0 0 24 24'><circle cx='12' cy='12' r='3'/><path d='M12 2v3M12 19v3M4.22 4.22l2.12 2.12M17.66 17.66l2.12 2.12M2 12h3M19 12h3M4.22 19.78l2.12-2.12M17.66 6.34l2.12-2.12'/></svg>
      <span>AUTO</span>
    </button>
    <button class='mode-tab' id='tab-rc' onclick='changeMode("rc")'>
      <svg class='tab-icon' viewBox='0 0 24 24'><rect x='2' y='7' width='20' height='14' rx='3'/><path d='M12 7V3M8 3h8'/><line x1='9' y1='14' x2='9' y2='14'/><line x1='6' y1='14' x2='12' y2='14'/><line x1='9' y1='11' x2='9' y2='17'/><circle cx='15' cy='14' r='1'/></svg>
      <span>RC</span>
    </button>
    <button class='mode-tab' id='tab-gesture' onclick='changeMode("gesture")'>
      <svg class='tab-icon' viewBox='0 0 24 24'><rect x='5' y='2' width='14' height='20' rx='2' ry='2'/><line x1='12' y1='18' x2='12.01' y2='18'/><path d='M17 6c2 1 3 3 3 5s-1 4-3 5'/><path d='M7 6c-2 1-3 3-3 5s1 4 3 5'/></svg>
      <span>TILT</span>
    </button>
    <button class='mode-tab' id='tab-follow' onclick='changeMode("follow")'>
      <svg class='tab-icon' viewBox='0 0 24 24'><path d='M16 21v-2a4 4 0 0 0-4-4H6a4 4 0 0 0-4 4v2'/><circle cx='9' cy='7' r='4'/><path d='M22 9l-3 3 3 3'/></svg>
      <span>FOLLOW</span>
    </button>
    <div class='tab-glider'></div>
  </div>

  <!-- Autonomous Tab Page -->
  <section class='card panel-oa active' id='panel-oa'>
    <h2 class='card-title'>AUTONOMOUS NAVIGATOR</h2>
    <div class='oa-status-msg'>
      Car running in autonomous safety mode. Reduced speeds engaged for maximum reaction windows. Scans will run dynamically at path obstructions.
    </div>
  </section>

  <!-- RC Drive Tab Page -->
  <section class='card panel-rc' id='panel-rc'>
    <h2 class='card-title'>RC TRANSMITTER</h2>
    <!-- Direction pad: 3x3 grid, centre = STOP -->
    <div style='display:grid; grid-template-columns:repeat(3,1fr); gap:10px; max-width:260px; margin:0 auto 18px;'>
      <div></div>
      <button class='rc-ctrl-btn' id='rc-fwd'>&#9650;<br><small>FORWARD</small></button>
      <div></div>
      <button class='rc-ctrl-btn' id='rc-left'>&#9668;<br><small>LEFT</small></button>
      <button class='rc-ctrl-btn rc-stop-btn' id='rc-stop'>&#9632;<br><small>STOP</small></button>
      <button class='rc-ctrl-btn' id='rc-right'>&#9658;<br><small>RIGHT</small></button>
      <div></div>
      <button class='rc-ctrl-btn' id='rc-rev'>&#9660;<br><small>REVERSE</small></button>
      <div></div>
    </div>
    <!-- Horn & sliders -->
    <div class='actions-panel'>
      <button class='btn-horn' id='btn-horn'>
        <svg viewBox='0 0 24 24'><path d='M11 5L6 9H2v6h4l5 4V5z'/><path d='M19.07 4.93a10 10 0 0 1 0 14.14'/><path d='M15.54 8.46a5 5 0 0 1 0 7.07'/></svg>
        <span>TRIGGER HORN</span>
      </button>
      <div class='slider-group'>
        <div class='slider-header'>
          <div class='slider-title'>
            <svg class='slider-icon' viewBox='0 0 24 24'><polygon points='13 2 3 14 12 14 11 22 21 10 12 10 13 2'/></svg>
            <span>RC DRIVE VELOCITY</span>
          </div>
          <span class='slider-value' id='val-speed'>78%</span>
        </div>
        <div class='range-input-wrap'>
          <input type='range' id='slide-speed' min='20' max='100' value='78'>
        </div>
      </div>
      <div class='slider-group'>
        <div class='slider-header'>
          <div class='slider-title'>
            <svg class='slider-icon' viewBox='0 0 24 24'><path d='M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z'/><circle cx='12' cy='12' r='3'/></svg>
            <span>RADAR DECK SERVO</span>
          </div>
          <span class='slider-value' id='val-servo'>90&deg;</span>
        </div>
        <div class='range-input-wrap'>
          <input type='range' id='slide-servo' min='0' max='180' value='90'>
        </div>
      </div>
    </div>
  </section>

  <!-- Gesture Tab Page -->
  <section class='card panel-gesture' id='panel-gesture'>
    <h2 class='card-title'>IMU GESTURE CONTROLLER</h2>
    <div class="joystick-container">
      <div class="imu-visualizer" style="display: flex; justify-content: center; margin: 10px 0; width: 100%;">
        <canvas id="imu-canvas" width="140" height="140" style="background: transparent; border-radius: 50%; display: block;"></canvas>
      </div>
      <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 12px; width: 100%; text-align: center; margin-bottom: 16px;">
        <div class="metric-widget" style="padding: 10px;">
          <span class="metric-label">PITCH TILT</span>
          <span class="metric-value" id="val-pitch">0&deg;</span>
        </div>
        <div class="metric-widget" style="padding: 10px;">
          <span class="metric-label">ROLL TILT</span>
          <span class="metric-value" id="val-roll">0&deg;</span>
        </div>
      </div>
      <button class="btn-horn" id="btn-imu-toggle" onclick="requestOrientationPermission()" style="background: rgba(6, 182, 212, 0.08); border-color: rgba(6, 182, 212, 0.2); color: var(--accent-gesture);">
        START TILT STEERING
      </button>
      <!-- IMU warning shown if browser blocks DeviceOrientation -->
      <div id="imu-warning" style="display:none; text-align:center; padding:12px 14px; background:rgba(244,63,94,0.07); border:1px solid rgba(244,63,94,0.2); border-radius:14px; margin-top:14px; font-size:10px; line-height:1.7; color:var(--accent-danger);">
        &#9888; No IMU events detected.<br>
        Android Chrome blocks sensors on HTTP.<br>
        <b>Fix:</b> Go to <code style="font-size:9px;">chrome://flags</code> &rarr; search <b>Insecure origins</b> &rarr; add <code style="font-size:9px;">http://192.168.4.1</code><br>
        Or use the D-pad below as fallback.
      </div>
      <!-- Fallback D-pad (always visible, works without IMU) -->
      <div style="margin-top:18px;">
        <div style="text-align:center; font-size:9px; font-weight:800; letter-spacing:0.1em; color:var(--text-muted); margin-bottom:10px;">MANUAL FALLBACK PAD</div>
        <div style="display:grid; grid-template-columns:1fr 1fr 1fr; gap:8px; max-width:180px; margin:0 auto;">
          <div></div>
          <button class="dpad-btn" id="dpf" onpointerdown="gestureManual('F')" onpointerup="gestureManual('S')" onpointerleave="gestureManual('S')">&#9650;</button>
          <div></div>
          <button class="dpad-btn" id="dpl" onpointerdown="gestureManual('L')" onpointerup="gestureManual('S')" onpointerleave="gestureManual('S')">&#9668;</button>
          <button class="dpad-btn" id="dps" onpointerdown="gestureManual('S')">&#9632;</button>
          <button class="dpad-btn" id="dpr" onpointerdown="gestureManual('R')" onpointerup="gestureManual('S')" onpointerleave="gestureManual('S')">&#9658;</button>
          <div></div>
          <button class="dpad-btn" id="dpb" onpointerdown="gestureManual('B')" onpointerup="gestureManual('S')" onpointerleave="gestureManual('S')">&#9660;</button>
          <div></div>
        </div>
      </div>
    </div>
  </section>

  <!-- Follow Tab Page -->
  <section class='card panel-follow' id='panel-follow'>
    <h2 class='card-title'>HUMAN TARGET FOLLOWER</h2>
    <div class="oa-status-msg" style="margin-bottom: 16px;">
      The car will dynamically seek out a human at a close distance range (15cm to 50cm).
      The scanning deck tracks the target, steering and driving the car to maintain its distance.
    </div>
    <div class="metrics-container" style="grid-template-columns: 1fr;">
      <div class="metric-widget" style="align-items: center; padding: 20px;">
        <span class="metric-label">TRACKED TARGET DISTANCE</span>
        <span class="metric-value" id="val-follow-dist" style="font-size: 24px; color: var(--accent-follow);">STANDBY</span>
        <div class="gauge-bar-container" style="width: 80%; margin-top: 10px;">
          <div class="gauge-bar" id="follow-gauge" style="width: 0%; background: var(--accent-follow);"></div>
        </div>
      </div>
    </div>
  </section>

</div>
<script>
  let activeCmd = 'S';
  let socket;
  let imuActive = false;
  let imuEventsReceived = false;
  let lastGestureTime = 0;
  
  // Custom throttle to prevent ESP32 network starvation
  const throttle = (func, limit) => {
    let inThrottle;
    return (...args) => {
      const context = this;
      if (!inThrottle) {
        func.apply(context, args);
        inThrottle = true;
        setTimeout(() => inThrottle = false, limit);
      }
    }
  };
  
  // WebSockets setup
  const initWebSocket = () => {
    const wsUrl = 'ws://' + window.location.hostname + ':81/';
    socket = new WebSocket(wsUrl);
    socket.onopen = () => {
      const badge = document.getElementById('conn-badge');
      badge.textContent = 'CONNECTED';
      badge.style.background = 'rgba(16, 185, 129, 0.08)';
      badge.style.color = 'var(--accent-success)';
      badge.style.borderColor = 'rgba(16, 185, 129, 0.2)';
    };
    socket.onclose = () => {
      const badge = document.getElementById('conn-badge');
      badge.textContent = 'OFFLINE';
      badge.style.background = 'rgba(244, 63, 94, 0.08)';
      badge.style.color = 'var(--accent-danger)';
      badge.style.borderColor = 'rgba(244, 63, 94, 0.2)';
      setTimeout(initWebSocket, 2000);
    };
    socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        updateTelemetryUI(data);
      } catch(e) {}
    };
  };
  
  // Primary WS communication with HTTP fallbacks
  const sendSpeed = throttle((val) => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send('speed:' + val);
    } else {
      fetch('/speed?v=' + val).catch(() => {});
    }
  }, 100);
  
  const sendServo = throttle((val) => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send('servo:' + val);
    } else {
      fetch('/servo?a=' + val).catch(() => {});
    }
  }, 80);
  
  const sendCmd = (cmd) => {
    if (cmd !== activeCmd) {
      activeCmd = cmd;
      if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send('cmd:' + cmd);
      } else {
        fetch('/cmd?d=' + cmd).catch(() => {});
      }
      const names = { F: 'FORWARD', B: 'REVERSE', L: 'LEFT', R: 'RIGHT', FL: 'FWD-LEFT', FR: 'FWD-RIGHT', BL: 'REV-LEFT', BR: 'REV-RIGHT', S: 'STOP' };
      document.getElementById('val-cmd').textContent = names[cmd] || 'STOP';
      
      const vGauge = document.getElementById('vector-gauge');
      if (cmd === 'S') {
        vGauge.style.width = '20%';
        vGauge.style.background = 'var(--text-muted)';
      } else {
        vGauge.style.width = '100%';
        vGauge.style.background = 'var(--accent-active)';
      }
    }
  };
  
  const sendGestureCmd = (cmd, speed) => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send('g:' + cmd + ':' + speed);
    }
  };
  
  // RC Transmitter Button State Tracking
  // Supports simultaneous presses: FWD+LEFT = FL, FWD+RIGHT = FR, etc.
  const rcState = { fwd: false, rev: false, left: false, right: false };
  
  const calcRcCmd = () => {
    if (rcState.fwd  && rcState.left)  return 'FL';
    if (rcState.fwd  && rcState.right) return 'FR';
    if (rcState.rev  && rcState.left)  return 'BL';
    if (rcState.rev  && rcState.right) return 'BR';
    if (rcState.fwd)   return 'F';
    if (rcState.rev)   return 'B';
    if (rcState.left)  return 'L';
    if (rcState.right) return 'R';
    return 'S';
  };
  
  const attachRcBtn = (id, key) => {
    const btn = document.getElementById(id);
    if (!btn) return;
    btn.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      btn.setPointerCapture(e.pointerId);
      rcState[key] = true;
      btn.classList.add('pressed');
      sendCmd(calcRcCmd());
    });
    const release = () => {
      rcState[key] = false;
      btn.classList.remove('pressed');
      sendCmd(calcRcCmd());
    };
    btn.addEventListener('pointerup', release);
    btn.addEventListener('pointerleave', release);
    btn.addEventListener('pointercancel', release);
  };
  
  attachRcBtn('rc-fwd',   'fwd');
  attachRcBtn('rc-rev',   'rev');
  attachRcBtn('rc-left',  'left');
  attachRcBtn('rc-right', 'right');
  
  const rcStopBtn = document.getElementById('rc-stop');
  if (rcStopBtn) {
    rcStopBtn.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      Object.keys(rcState).forEach(k => rcState[k] = false);
      sendCmd('S');
    });
  }
  
  // Horn Button Touch/Pointer listeners
  const horn = document.getElementById('btn-horn');
  const hornStart = (e) => {
    e.preventDefault();
    horn.classList.add('active');
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send('horn:1');
    } else {
      fetch('/horn?on=1').catch(() => {});
    }
  };
  
  const hornEnd = (e) => {
    e.preventDefault();
    horn.classList.remove('active');
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send('horn:0');
    } else {
      fetch('/horn?on=0').catch(() => {});
    }
  };
  
  horn.addEventListener('pointerdown', hornStart);
  horn.addEventListener('pointerup', hornEnd);
  horn.addEventListener('pointerleave', hornEnd);
  
  // Sliders Input Bindings
  const speedSlider = document.getElementById('slide-speed');
  const speedVal = document.getElementById('val-speed');
  speedSlider.addEventListener('input', () => {
    const txt = speedSlider.value + '%';
    speedVal.textContent = txt;
    sendSpeed(speedSlider.value);
  });
  
  speedSlider.addEventListener('change', () => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send('speed:' + speedSlider.value);
    } else {
      fetch('/speed?v=' + speedSlider.value).catch(() => {});
    }
  });
  
  const servoSlider = document.getElementById('slide-servo');
  const servoVal = document.getElementById('val-servo');
  servoSlider.addEventListener('input', () => {
    const txt = servoSlider.value + '°';
    servoVal.textContent = txt;
    sendServo(servoSlider.value);
  });
  
  const servoSliderChange = () => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send('servo:' + servoSlider.value);
    } else {
      fetch('/servo?a=' + servoSlider.value).catch(() => {});
    }
  };
  servoSlider.addEventListener('change', servoSliderChange);
  
  // Mode Selection Tabs Switcher
  const changeMode = (mode) => {
    if (mode !== 'gesture' && imuActive) {
      stopOrientationListener();
    }
    syncModeTab(mode);
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send('mode:' + mode);
    } else {
      fetch('/set?mode=' + mode).catch(() => {});
    }
  };
  
  const syncModeTab = (mode) => {
    const container = document.querySelector('.app-container');
    const selector = document.querySelector('.mode-selector');
    const tabs = ['oa', 'rc', 'gesture', 'follow'];
    
    tabs.forEach(t => {
      document.getElementById('tab-' + t).classList.remove('active');
      document.getElementById('panel-' + t).classList.remove('active');
    });
    
    document.getElementById('tab-' + mode).classList.add('active');
    document.getElementById('panel-' + mode).classList.add('active');
    
    selector.className = 'mode-selector';
    if (mode === 'rc') {
      selector.classList.add('rc-active');
      container.className = 'app-container theme-rc';
    } else if (mode === 'gesture') {
      selector.classList.add('gesture-active');
      container.className = 'app-container theme-gesture';
      drawImuIndicator(0, 0);
    } else if (mode === 'follow') {
      selector.classList.add('follow-active');
      container.className = 'app-container theme-follow';
    } else {
      container.className = 'app-container theme-oa';
    }
  };
  
  // Telemetry updates
  const updateTelemetryUI = (data) => {
    // Distance Lidar
    const distVal = document.getElementById('val-dist');
    const distBar = document.getElementById('dist-gauge');
    if (data.dist >= 999) {
      distVal.textContent = 'CLEAR';
      distVal.className = 'metric-value status-safe';
      distBar.style.width = '100%';
      distBar.style.background = 'var(--accent-success)';
    } else {
      distVal.textContent = data.dist + ' cm';
      const pct = Math.min(Math.max(data.dist, 0), 100);
      distBar.style.width = pct + '%';
      if (data.dist > 50) {
        distVal.className = 'metric-value status-safe';
        distBar.style.background = 'var(--accent-success)';
      } else if (data.dist > 30) {
        distVal.className = 'metric-value status-warning';
        distBar.style.background = '#f59e0b';
      } else {
        distVal.className = 'metric-value status-danger';
        distBar.style.background = 'var(--accent-danger)';
      }
    }
    
    // Cliff IR
    const cliffVal = document.getElementById('val-cliff');
    const cliffBar = document.getElementById('cliff-gauge');
    if (data.ir < 1600) {
      cliffVal.textContent = 'CLIFF!';
      cliffVal.className = 'metric-value status-danger';
      cliffBar.style.width = '20%';
      cliffBar.style.background = 'var(--accent-danger)';
    } else {
      cliffVal.textContent = 'SAFE';
      cliffVal.className = 'metric-value status-safe';
      cliffBar.style.width = '100%';
      cliffBar.style.background = 'var(--accent-success)';
    }
    
    // Follow Mode telemetries
    const followVal = document.getElementById('val-follow-dist');
    const followBar = document.getElementById('follow-gauge');
    if (data.mode === 'follow') {
      if (data.dist >= 999) {
        followVal.textContent = 'SEEKING...';
        followVal.style.color = 'var(--text-muted)';
        followBar.style.width = '0%';
      } else {
        followVal.textContent = data.dist + ' cm';
        followVal.style.color = 'var(--accent-active)';
        const fPct = Math.min(Math.max((data.dist - 15) / (50 - 15) * 100, 0), 100);
        followBar.style.width = fPct + '%';
      }
    }
    
    // Update radar sweeps target trackers
    window._liveRadarDist = (data.dist && data.dist < 999) ? data.dist : 0;
    currentAngle = (data.angle && data.angle > 0) ? data.angle : 90;
    
    // Push target readings to decay array
    addRadarTarget(data.angle, data.dist);
    addRadarTarget(150, data.dL);
    addRadarTarget(90, data.dC);
    addRadarTarget(30, data.dR);
    
    // Sync mode tab selection
    const tabs = ['oa', 'rc', 'gesture', 'follow'];
    const activeTab = tabs.find(t => document.getElementById('tab-' + t).classList.contains('active'));
    if (data.mode !== activeTab) {
      syncModeTab(data.mode);
    }
  };
  
  const fetchTelemetry = () => {
    fetch('/status')
      .then(res => res.json())
      .then(data => {
        const badge = document.getElementById('conn-badge');
        badge.textContent = 'CONNECTED';
        badge.style.background = 'rgba(16, 185, 129, 0.08)';
        badge.style.color = 'var(--accent-success)';
        badge.style.borderColor = 'rgba(16, 185, 129, 0.2)';
        updateTelemetryUI(data);
      })
      .catch(() => {
        if (!socket || socket.readyState !== WebSocket.OPEN) {
          const badge = document.getElementById('conn-badge');
          badge.textContent = 'OFFLINE';
          badge.style.background = 'rgba(244, 63, 94, 0.08)';
          badge.style.color = 'var(--accent-danger)';
          badge.style.borderColor = 'rgba(244, 63, 94, 0.2)';
        }
      });
  };
  
  // Canvas radar renderer logic
  const canvas = document.getElementById('radar-canvas');
  const ctx = canvas.getContext('2d');
  const cx = canvas.width / 2;
  const cy = canvas.height / 2;
  const maxR = 90; // maximum range scale radius (px)
  
  let currentAngle = 90;
  let targetSpots = []; // holds target markers {angle, dist, life}
  
  const addRadarTarget = (angle, dist) => {
    if (dist <= 0 || dist >= 250) return;   // show obstacles up to 250 cm
    const existing = targetSpots.find(t => Math.abs(t.angle - angle) < 15);
    if (existing) {
      existing.dist = dist;
      existing.life = 1.0;
    } else {
      targetSpots.push({ angle: angle, dist: dist, life: 1.0 });
    }
  };
  
  const renderRadar = () => {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Outer scope radial backing
    ctx.beginPath();
    ctx.arc(cx, cy, maxR, 0, 2 * Math.PI);
    ctx.fillStyle = 'rgba(16, 185, 129, 0.012)';
    ctx.fill();
    
    // Grid circles
    ctx.strokeStyle = 'rgba(16, 185, 129, 0.12)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    [50, 100, 200].forEach(rCm => {       // rings at 50 / 100 / 200 cm
      ctx.beginPath();
      ctx.arc(cx, cy, (rCm / 250) * maxR, 0, 2 * Math.PI);
      ctx.stroke();
    });
    
    // Crosshairs lines
    ctx.setLineDash([]);
    ctx.strokeStyle = 'rgba(16, 185, 129, 0.06)';
    ctx.beginPath();
    ctx.moveTo(cx - maxR, cy);
    ctx.lineTo(cx + maxR, cy);
    ctx.moveTo(cx, cy - maxR);
    ctx.lineTo(cx, cy + maxR);
    ctx.stroke();
    
    // Field of view lines (Left and Right bounds: 30deg and 150deg)
    ctx.strokeStyle = 'rgba(16, 185, 129, 0.04)';
    [30, 150].forEach(deg => {
      const rad = deg * Math.PI / 180;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + maxR * Math.cos(rad), cy - maxR * Math.sin(rad));
      ctx.stroke();
    });
    
    // Dynamic active theme color determination
    const container = document.querySelector('.app-container');
    let rgbColor = '16, 185, 129';
    if (container.classList.contains('theme-rc')) rgbColor = '99, 102, 241';
    else if (container.classList.contains('theme-gesture')) rgbColor = '6, 182, 212';
    else if (container.classList.contains('theme-follow')) rgbColor = '168, 85, 247';
    
    // Sensor flashlight sweeping cone (16deg width)
    const beamAngle = currentAngle * Math.PI / 180;
    const beamGrad = ctx.createRadialGradient(cx, cy, 0, cx, cy, maxR);
    beamGrad.addColorStop(0, `rgba(${rgbColor}, 0.2)`);
    beamGrad.addColorStop(0.3, `rgba(${rgbColor}, 0.06)`);
    beamGrad.addColorStop(1, `rgba(${rgbColor}, 0)`);
    
    ctx.fillStyle = beamGrad;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, maxR, -beamAngle - (8 * Math.PI/180), -beamAngle + (8 * Math.PI/180));
    ctx.closePath();
    ctx.fill();
    
    // Sweeping beam centerline indicator
    ctx.strokeStyle = `rgba(${rgbColor}, 0.4)`;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + maxR * Math.cos(beamAngle), cy - maxR * Math.sin(beamAngle));
    ctx.stroke();
    
    // Live distance dot at current servo position — always visible
    const liveDist = window._liveRadarDist || 0;
    if (liveDist > 0 && liveDist < 300) {
      const dotR = Math.min((liveDist / 250) * maxR, maxR - 3);
      const dotX = cx + dotR * Math.cos(beamAngle);
      const dotY = cy - dotR * Math.sin(beamAngle);
      ctx.beginPath();
      ctx.arc(dotX, dotY, 5.5, 0, 2 * Math.PI);
      ctx.fillStyle = 'rgba(239, 68, 68, 0.95)';
      ctx.shadowColor = 'rgba(239, 68, 68, 0.9)';
      ctx.shadowBlur = 14;
      ctx.fill();
      ctx.shadowBlur = 0;
    } else {
      // No obstacle: faint dot at beam tip so beam is always visible
      const tipX = cx + (maxR - 4) * Math.cos(beamAngle);
      const tipY = cy - (maxR - 4) * Math.sin(beamAngle);
      ctx.beginPath();
      ctx.arc(tipX, tipY, 3, 0, 2 * Math.PI);
      ctx.fillStyle = `rgba(${rgbColor}, 0.18)`;
      ctx.fill();
    }
    
    // Decay update & draw warning markers
    targetSpots.forEach(t => t.life -= 0.012);
    targetSpots = targetSpots.filter(t => t.life > 0);
    targetSpots.forEach(t => {
      const r = (Math.min(t.dist, 250) / 250) * maxR;  // scaled to 250cm
      const rAngle = t.angle * Math.PI / 180;
      const tx = cx + r * Math.cos(rAngle);
      const ty = cy - r * Math.sin(rAngle);
      
      // Warning obstacle glow ring
      ctx.beginPath();
      ctx.arc(tx, ty, 4, 0, 2 * Math.PI);
      ctx.fillStyle = `rgba(244, 63, 94, ${t.life})`;
      ctx.shadowColor = '#f43f5e';
      ctx.shadowBlur = 8;
      ctx.fill();
      ctx.shadowBlur = 0;
      
      // Pulsing outer halo rings
      ctx.beginPath();
      ctx.arc(tx, ty, 12 * (1 - t.life), 0, 2 * Math.PI);
      ctx.strokeStyle = `rgba(244, 63, 94, ${t.life * 0.45})`;
      ctx.lineWidth = 1.2;
      ctx.stroke();
    });
    requestAnimationFrame(renderRadar);
  };
  
  // IMU canvas indicator bubble drawing
  const drawImuIndicator = (pitch, roll) => {
    const imuCanvas = document.getElementById('imu-canvas');
    if (!imuCanvas) return;
    const imuCtx = imuCanvas.getContext('2d');
    const icx = imuCanvas.width / 2;
    const icy = imuCanvas.height / 2;
    const maxOffset = 50;
    
    imuCtx.clearRect(0, 0, imuCanvas.width, imuCanvas.height);
    
    // Draw outer circle
    imuCtx.strokeStyle = 'rgba(6, 182, 212, 0.15)';
    imuCtx.lineWidth = 2;
    imuCtx.beginPath();
    imuCtx.arc(icx, icy, maxOffset, 0, 2 * Math.PI);
    imuCtx.stroke();
    
    // Draw crosshairs
    imuCtx.strokeStyle = 'rgba(6, 182, 212, 0.05)';
    imuCtx.beginPath();
    imuCtx.moveTo(icx - maxOffset, icy);
    imuCtx.lineTo(icx + maxOffset, icy);
    imuCtx.moveTo(icx, icy - maxOffset);
    imuCtx.lineTo(icx, icy + maxOffset);
    imuCtx.stroke();
    
    // Calculate bubble position based on pitch/roll (clamp between -45 and 45)
    const px = Math.min(Math.max(roll / 45, -1), 1) * maxOffset;
    const py = Math.min(Math.max(-pitch / 45, -1), 1) * maxOffset;
    
    // Draw bubble
    imuCtx.beginPath();
    imuCtx.arc(icx + px, icy + py, 10, 0, 2 * Math.PI);
    imuCtx.fillStyle = 'var(--accent-active)';
    imuCtx.shadowColor = 'var(--accent-active)';
    imuCtx.shadowBlur = 10;
    imuCtx.fill();
    imuCtx.shadowBlur = 0;
  };
  
  // Process device orientation and send packets to server
  const processOrientationData = (pitch, roll) => {
    const now = Date.now();
    if (now - lastGestureTime < 100) return;
    lastGestureTime = now;
    
    let cmd = 'S';
    const deadzone = 10;
    
    const absPitch = Math.abs(pitch);
    const absRoll = Math.abs(roll);
    
    if (absPitch > deadzone || absRoll > deadzone) {
      if (pitch > deadzone && roll > deadzone) cmd = 'FR';
      else if (pitch > deadzone && roll < -deadzone) cmd = 'FL';
      else if (pitch < -deadzone && roll > deadzone) cmd = 'BR';
      else if (pitch < -deadzone && roll < -deadzone) cmd = 'BL';
      else if (pitch > deadzone) cmd = 'F';
      else if (pitch < -deadzone) cmd = 'B';
      else if (roll > deadzone) cmd = 'R';
      else if (roll < -deadzone) cmd = 'L';
    }
    
    let speed = 0;
    if (cmd !== 'S') {
      const maxTilt = Math.min(Math.max(Math.max(absPitch, absRoll), deadzone), 40);
      const pct = (maxTilt - deadzone) / (40 - deadzone);
      const limitPct = parseInt(document.getElementById('slide-speed').value) / 100.0;
      speed = Math.round(350 + (pct * (800 - 350) * limitPct));
    }
    
    sendGestureCmd(cmd, speed);
  };
  
  const startOrientationListener = () => {
    if (imuActive) return;
    imuActive = true;
    imuEventsReceived = false;
    window.addEventListener('deviceorientation', handleDeviceOrientation);
    document.getElementById('btn-imu-toggle').textContent = 'STOP TILT STEERING';
    document.getElementById('btn-imu-toggle').classList.add('active');
    // After 2 seconds, check if any events actually arrived
    setTimeout(() => {
      if (imuActive && !imuEventsReceived) {
        const warn = document.getElementById('imu-warning');
        if (warn) warn.style.display = 'block';
      }
    }, 2000);
  };
  
  const stopOrientationListener = () => {
    if (!imuActive) return;
    imuActive = false;
    imuEventsReceived = false;
    window.removeEventListener('deviceorientation', handleDeviceOrientation);
    document.getElementById('btn-imu-toggle').textContent = 'START TILT STEERING';
    document.getElementById('btn-imu-toggle').classList.remove('active');
    const warn = document.getElementById('imu-warning');
    if (warn) warn.style.display = 'none';
    sendGestureCmd('S', 0);
    drawImuIndicator(0, 0);
  };
  
  const handleDeviceOrientation = (event) => {
    if (!imuActive || document.querySelector('.app-container').classList.contains('theme-gesture') === false) return;
    imuEventsReceived = true;
    // Hide warning if events start coming in
    const warn = document.getElementById('imu-warning');
    if (warn && warn.style.display !== 'none') warn.style.display = 'none';
    const pitch = event.beta;
    const roll = event.gamma;
    document.getElementById('val-pitch').textContent = Math.round(pitch) + '&deg;';
    document.getElementById('val-roll').textContent = Math.round(roll) + '&deg;';
    drawImuIndicator(pitch, roll);
    processOrientationData(pitch, roll);
  };
  
  // Fallback D-pad for when browser blocks DeviceOrientation (e.g. Android Chrome on HTTP)
  const gestureManual = (cmd) => {
    const spdPct = parseInt(document.getElementById('slide-speed').value);
    const speed = cmd === 'S' ? 0 : Math.round(350 + ((spdPct - 20) / 80) * (800 - 350));
    sendGestureCmd(cmd, speed);
  };
  
  const requestOrientationPermission = () => {
    if (imuActive) {
      stopOrientationListener();
      return;
    }
    
    // Check permission logic
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
      DeviceOrientationEvent.requestPermission()
        .then(response => {
          if (response === 'granted') {
            startOrientationListener();
          } else {
            alert('Device orientation permission is required for tilt control.');
          }
        })
        .catch(console.error);
    } else {
      startOrientationListener();
    }
  };
  
  // Initialize Websocket connection
  initWebSocket();
  
  // Fallback HTTP status polling loop (if WebSocket is down)
  setInterval(() => {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      fetchTelemetry();
    }
  }, 400);
</script>
</body>
</html>
)rawhtml";
  server.send(200, "text/html", page);
}
void handleSetMode() {
  String m = server.arg("mode");
  stopAll();
  strlcpy(rcCmd, "S", sizeof(rcCmd));
  if (m == "rc") {
    carMode = MODE_RC;
    beep(80);
  } else if (m == "gesture") {
    carMode = MODE_GESTURE;
    beep(80);
  } else if (m == "follow") {
    carMode = MODE_FOLLOW;
    beep(80);
  } else {
    carMode = MODE_OA;
    beep(80); delay(80); beep(80);
    rcServoAngle = TURN_ANGLE_C; // Request centering in the main loop thread
  }
  server.send(200, "text/plain", "ok");
}
void handleCmd() {
  String d = server.arg("d");
  strlcpy(rcCmd, d.c_str(), sizeof(rcCmd));
  server.send(200, "text/plain", "ok");
}
void handleHorn() {
  rcHornOn = (server.arg("on") == "1");
  server.send(200, "text/plain", "ok");
}
void handleSpeed() {
  int pct = server.arg("v").toInt();
  pct = constrain(pct, 10, 100);
  rcSpeed = (int)(RC_SPEED_DEF * pct / 100);
  server.send(200, "text/plain", "ok");
}
void handleServo() {
  rcServoAngle = constrain(server.arg("a").toInt(), 0, 180);
  server.send(200, "text/plain", "ok");
}
void handleStatus() {
  String modeStr = "oa";
  if (carMode == MODE_RC) modeStr = "rc";
  else if (carMode == MODE_GESTURE) modeStr = "gesture";
  else if (carMode == MODE_FOLLOW) modeStr = "follow";
  
  String json = "{";
  json += "\"mode\":\"" + modeStr + "\",";
  json += "\"dist\":" + String(lastDistance) + ",";
  json += "\"angle\":" + String(currentServoAngle) + ",";
  json += "\"ir\":" + String(lastIR) + ",";
  json += "\"dL\":" + String(dL_dist) + ",";
  json += "\"dC\":" + String(dC_dist) + ",";
  json += "\"dR\":" + String(dR_dist);
  json += "}";
  server.send(200, "application/json", json);
}
// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  // Motor pins
  pinMode(L_AIN1, OUTPUT); pinMode(L_AIN2, OUTPUT);
  pinMode(L_BIN1, OUTPUT); pinMode(L_BIN2, OUTPUT);
  pinMode(R_AIN1, OUTPUT); pinMode(R_AIN2, OUTPUT);
  pinMode(R_BIN1, OUTPUT); pinMode(R_BIN2, OUTPUT);
  // LEDC – Setup (Compatible with both Core v2.x and v3.x)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(L_PWMA, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(L_PWMB, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(R_PWMA, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(R_PWMB, PWM_FREQ, PWM_RES_BITS);
#else
  ledcSetup(CH_L_PWMA, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_L_PWMB, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_R_PWMA, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_R_PWMB, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(L_PWMA, CH_L_PWMA);
  ledcAttachPin(L_PWMB, CH_L_PWMB);
  ledcAttachPin(R_PWMA, CH_R_PWMA);
  ledcAttachPin(R_PWMB, CH_R_PWMB);
#endif
  // Standby pin
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
  // Sensors & Horn
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  // Servo timer allocation to prevent LEDC conflict with motor channels 0-3
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  // Servo
  scanServo.setPeriodHertz(50);
  scanServo.attach(SERVO_PIN, 500, 2500);
  aimServo(TURN_ANGLE_C, true);
  stopAll();
  // WiFi Setup (Hotspot AP or Station Router mode)
  if (WIFI_AP_MODE) {
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.print("Access Point started. IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Connected to WiFi. IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("Connection failed! Defaulting to Hotspot Mode...");
      WiFi.softAP(WIFI_SSID, WIFI_PASS);
      Serial.print("Access Point started. IP: ");
      Serial.println(WiFi.softAPIP());
    }
  }
  // Web routes
  server.on("/",       handleRoot);
  server.on("/rc",     []() { server.sendHeader("Location", "/"); server.send(302, "text/plain", "Redirecting..."); });
  server.on("/set",    handleSetMode);
  server.on("/cmd",    handleCmd);
  server.on("/horn",   handleHorn);
  server.on("/speed",  handleSpeed);
  server.on("/servo",  handleServo);
  server.on("/status", handleStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "Not Found"); });
  server.begin();
  
  // Start WebSocket Server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  beep(100);
  Serial.println("System initialized. HTTP & WebSocket servers running.");
}
// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();   // Always handle incoming HTTP requests
  webSocket.loop();        // Keep the WebSocket server listening
  
  // Update servo angle safely in loop context
  if (carMode == MODE_OA) {
    aimServo(TURN_ANGLE_C, false);
  } else if (carMode == MODE_RC || carMode == MODE_GESTURE) {
    aimServo(rcServoAngle, false);
  }
  
  // Broadcast WebSocket telemetry at 150ms intervals
  static unsigned long lastWsTelemetry = 0;
  if (millis() - lastWsTelemetry > 150) {
    lastWsTelemetry = millis();
    String modeStr = "oa";
    if (carMode == MODE_RC) modeStr = "rc";
    else if (carMode == MODE_GESTURE) modeStr = "gesture";
    else if (carMode == MODE_FOLLOW) modeStr = "follow";
    
    String json = "{";
    json += "\"mode\":\"" + modeStr + "\",";
    json += "\"dist\":" + String(lastDistance) + ",";
    json += "\"angle\":" + String(currentServoAngle) + ",";
    json += "\"ir\":" + String(lastIR) + ",";
    json += "\"dL\":" + String(dL_dist) + ",";
    json += "\"dC\":" + String(dC_dist) + ",";
    json += "\"dR\":" + String(dR_dist);
    json += "}";
    webSocket.broadcastTXT(json);
  }
  
  if (carMode == MODE_OA) {
    // ── Obstacle Avoidance (Autopilot Mode) ───────────────────
    long dC = distanceCM();
    int ir = analogRead(IR_PIN);
    lastDistance = dC;
    lastIR = ir;
    dC_dist = dC;
    // Edge/cliff protection
    if (ir < IR_THRESHOLD) {
      stopAll(); beep(60);
      reverse(OA_REVERSE); delay(REVERSE_MS);
      int dir = chooseBestTurn();
      (dir > 0) ? pivotLeft(OA_TURN) : pivotRight(OA_TURN);
      delay(PIVOT_MS); stopAll();
      return;
    }
    // Ultrasonic collision avoidance
    if (dC <= SAFE_DIST_CM) {
      stopAll(); beep(60);
      reverse(OA_REVERSE); delay(REVERSE_MS); stopAll();
      int dir = chooseBestTurn();
      (dir > 0) ? pivotLeft(OA_TURN) : pivotRight(OA_TURN);
      delay(PIVOT_MS); stopAll();
    } else {
      int spd = (dC >= CLEAR_DIST_CM) ? OA_SPEED : OA_SPEED * 3 / 4;
      forward(spd);
    }
    delay(20);
  } else if (carMode == MODE_RC) {
    // ── RC Web Control Mode ──────────────────────────────────
    digitalWrite(BUZZER, rcHornOn ? HIGH : LOW);
    applyRcCmd(rcCmd, rcSpeed);
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 150) {
      lastSensorUpdate = millis();
      lastDistance = singlePingCM(15000UL);
      lastIR = analogRead(IR_PIN);
    }
    delay(15);
  } else if (carMode == MODE_GESTURE) {
    // ── Gesture Control Mode ─────────────────────────────────
    digitalWrite(BUZZER, rcHornOn ? HIGH : LOW);
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 150) {
      lastSensorUpdate = millis();
      lastDistance = singlePingCM(15000UL);
      lastIR = analogRead(IR_PIN);
    }
    delay(15);
  } else if (carMode == MODE_FOLLOW) {
    // ── Human / Hand Following Mode ───────────────────────────
    // Follows hand forward when detected (5-35 cm). Stops when close (< 5 cm) or lost. No reverse. No IR reads.

    unsigned long now = millis();
    static unsigned long lastScanTime = 0;
    static long lastValidDist = 999;

    if (!followFound) {
      // ── Search State (Sweeping) ───────────────────────────
      stopAll();

      // Sweep servo slowly
      followAngle += followSweepDir;
      if (followAngle >= 120) {
        followAngle = 120;
        followSweepDir = -4;
      } else if (followAngle <= 60) {
        followAngle = 60;
        followSweepDir = 4;
      }
      aimServo(followAngle, false);
      currentServoAngle = followAngle;

      // Ping to find targets
      long d = singlePingCM(15000UL);
      lastDistance = d;
      dC_dist = d;

      // Lock on if hand is detected between 5 cm and 35 cm
      if (d >= 5 && d <= 35) {
        followFound = true;
        lostFollowTime = now;
        lastValidDist = d;
        lastScanTime = 0; // reset scan cooldown
        beep(70); // Lock feedback
      }
      delay(60);
    } else {
      // ── Locked Tracking State (Quiet search-on-loss with 600ms Cooldown) ──
      // Servo points straight at followAngle. No wiggling if hand is steady!
      aimServo(followAngle, false);
      currentServoAngle = followAngle;
      
      long d = singlePingCM(12000UL);
      dC_dist = d;

      if (d >= 5 && d <= 35) {
        // Hand is still directly in front of the servo!
        lostFollowTime = now;
        lastValidDist = d;
        lastDistance = d;
        dL_dist = 999;
        dR_dist = 999;

        // Drive to maintain 7cm distance (stops if < 5cm, no reverse)
        if (d < 5) {
          stopAll();
        } else {
          if (followAngle > 105) {
            pivotLeft(OA_TURN * 6 / 10);  // slow left turn
          } else if (followAngle < 75) {
            pivotRight(OA_TURN * 6 / 10); // slow right turn
          } else {
            forward(OA_SPEED * 6 / 10);   // drive straight forward
          }
        }
      } else {
        // Center reading is lost! Stop motors.
        stopAll();

        // Scan left/right ONLY once every 600ms to prevent vibration
        if (now - lastScanTime > 600) {
          lastScanTime = now;

          // Scan Left
          int angleL = constrain(followAngle + 18, 45, 135);
          aimServo(angleL, false);
          delay(80); // allow servo to settle
          long dL_s = singlePingCM(12000UL);
          dL_dist = dL_s;

          // Scan Right
          int angleR = constrain(followAngle - 18, 45, 135);
          aimServo(angleR, false);
          delay(80); // allow servo to settle
          long dR_s = singlePingCM(12000UL);
          dR_dist = dR_s;

          // Restore center servo position
          aimServo(followAngle, false);

          if (dL_s >= 5 && dL_s <= 35 && (dR_s > dL_s || dR_s > 35)) {
            // Hand is on the left
            followAngle = constrain(followAngle + 15, 60, 120);
            lostFollowTime = now;
            lastValidDist = dL_s;
          } else if (dR_s >= 5 && dR_s <= 35 && (dL_s > dR_s || dL_s > 35)) {
            // Hand is on the right
            followAngle = constrain(followAngle - 15, 60, 120);
            lostFollowTime = now;
            lastValidDist = dR_s;
          }
        }

        // If target lost for more than 1500 ms, break lock
        if (now - lostFollowTime > 1500) {
          followFound = false;
          beep(50);
          delay(50);
          beep(50);
        }
      }
      delay(40); // 25Hz loop
    }
  }
}
