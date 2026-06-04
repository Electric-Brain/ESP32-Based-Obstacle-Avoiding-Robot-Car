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
// ─── WiFi Configuration ──────────────────────────────────────
#define WIFI_AP_MODE true  // true = creates own hotspot, false = connects to router
const char* WIFI_SSID = "GhostDrive";
const char* WIFI_PASS = "12345678";
WebServer server(80);
// ─── Mode ────────────────────────────────────────────────────
enum CarMode { MODE_OA, MODE_RC };
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
int currentServoAngle = -1;
volatile long dL_dist = 999;
volatile long dC_dist = 999;
volatile long dR_dist = 999;
// ─── Actuator Globals ─────────────────────────────────────────
Servo scanServo;
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
      padding: 12px;
      font-family: inherit;
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.08em;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      z-index: 2;
      transition: color 0.3s ease;
    }
    
    .mode-tab.active {
      color: var(--text-main);
    }
    
    .tab-icon {
      width: 15px;
      height: 15px;
      fill: none;
      stroke: currentColor;
      stroke-width: 2.2;
    }
    
    .tab-glider {
      position: absolute;
      left: 4px;
      top: 4px;
      width: calc(50% - 4px);
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
    .panel-oa, .panel-rc {
      display: none;
    }
    
    .panel-oa.active, .panel-rc.active {
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
      <span>AUTOPILOT</span>
    </button>
    <button class='mode-tab' id='tab-rc' onclick='changeMode("rc")'>
      <svg class='tab-icon' viewBox='0 0 24 24'><rect x='2' y='7' width='20' height='14' rx='3'/><path d='M12 7V3M8 3h8'/><line x1='9' y1='14' x2='9' y2='14'/><line x1='6' y1='14' x2='12' y2='14'/><line x1='9' y1='11' x2='9' y2='17'/><circle cx='15' cy='14' r='1'/></svg>
      <span>RC DRIVE</span>
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
  
  <!-- Manual Tab Page -->
  <section class='card panel-rc' id='panel-rc'>
    <div class='controls-layout'>
      <!-- Interactive Joystick Controller -->
      <div class='joystick-container'>
        <h2 class='card-title'>360 VECTOR JOYSTICK</h2>
        <div class='joystick-zone'>
          <div class='joystick-base'>
            <div class='joystick-handle' id='joy-handle'></div>
          </div>
        </div>
      </div>
      
      <!-- Range sliders & horn button -->
      <div class='actions-panel'>
        <button class='btn-horn' id='btn-horn'>
          <svg viewBox='0 0 24 24'><path d='M11 5L6 9H2v6h4l5 4V5z'/><path d='M19.07 4.93a10 10 0 0 1 0 14.14'/><path d='M15.54 8.46a5 5 0 0 1 0 7.07'/></svg>
          <span>TRIGGER HORN</span>
        </button>
        
        <!-- Speed Limit Slider -->
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
        
        <!-- Camera Servo Angle Slider -->
        <div class='slider-group'>
          <div class='slider-header'>
            <div class='slider-title'>
              <svg class='slider-icon' viewBox='0 0 24 24'><path d='M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z'/><circle cx='12' cy='12' r='3'/></svg>
              <span>RADAR DECK SERVO</span>
            </div>
            <span class='slider-value' id='val-servo'>90°</span>
          </div>
          <div class='range-input-wrap'>
            <input type='range' id='slide-servo' min='0' max='180' value='90'>
          </div>
        </div>
      </div>
    </div>
  </section>
  
</div>
<script>
  let activeCmd = 'S';
  
  // Custom throttle to prevent ESP32 TCP network starvation
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
  // API wrappers
  const sendSpeed = throttle((val) => {
    fetch('/speed?v=' + val).catch(() => {});
  }, 100);
  const sendServo = throttle((val) => {
    fetch('/servo?a=' + val).catch(() => {});
  }, 80);
  const sendCmd = (cmd) => {
    if (cmd !== activeCmd) {
      activeCmd = cmd;
      fetch('/cmd?d=' + cmd).catch(() => {});
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
  // Virtual Joystick Logic
  const handle = document.getElementById('joy-handle');
  const joyBase = handle.parentElement;
  let isDragging = false;
  let startX = 0, startY = 0;
  const maxDistance = 45; // max displacement boundary (px)
  const initJoystick = () => {
    // Pointerdown down handler
    joyBase.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      isDragging = true;
      joyBase.setPointerCapture(e.pointerId);
      
      const rect = joyBase.getBoundingClientRect();
      startX = rect.left + rect.width / 2;
      startY = rect.top + rect.height / 2;
      
      updateJoyPos(e.clientX, e.clientY);
    });
    joyBase.addEventListener('pointermove', (e) => {
      if (!isDragging) return;
      e.preventDefault();
      updateJoyPos(e.clientX, e.clientY);
    });
    const endDrag = (e) => {
      if (!isDragging) return;
      isDragging = false;
      joyBase.releasePointerCapture(e.pointerId);
      
      // return knob to center
      handle.style.transform = 'translate(0px, 0px)';
      sendCmd('S');
    };
    joyBase.addEventListener('pointerup', endDrag);
    joyBase.addEventListener('pointercancel', endDrag);
  };
  const updateJoyPos = (clientX, clientY) => {
    let dx = clientX - startX;
    let dy = clientY - startY;
    
    const distance = Math.sqrt(dx * dx + dy * dy);
    if (distance > maxDistance) {
      dx = (dx / distance) * maxDistance;
      dy = (dy / distance) * maxDistance;
    }
    
    handle.style.transform = `translate(${dx}px, ${dy}px)`;
    
    // Normalize coordinates
    const nx = dx / maxDistance;
    const ny = -dy / maxDistance; // invert Y axis for coordinates
    
    let cmd = 'S';
    const deadzone = 0.35;
    
    if (Math.abs(nx) < deadzone && Math.abs(ny) < deadzone) {
      cmd = 'S';
    } else {
      const angle = Math.atan2(ny, nx) * 180 / Math.PI; // angle range -180 to 180
      
      if (angle >= -22.5 && angle < 22.5) {
        cmd = 'R';
      } else if (angle >= 22.5 && angle < 67.5) {
        cmd = 'FR';
      } else if (angle >= 67.5 && angle < 112.5) {
        cmd = 'F';
      } else if (angle >= 112.5 && angle < 157.5) {
        cmd = 'FL';
      } else if (angle >= 157.5 || angle < -157.5) {
        cmd = 'L';
      } else if (angle >= -157.5 && angle < -112.5) {
        cmd = 'BL';
      } else if (angle >= -112.5 && angle < -67.5) {
        cmd = 'B';
      } else if (angle >= -67.5 && angle < -22.5) {
        cmd = 'BR';
      }
    }
    sendCmd(cmd);
  };
  initJoystick();
  // Horn Button Touch/Pointer listeners
  const horn = document.getElementById('btn-horn');
  const hornStart = (e) => {
    e.preventDefault();
    horn.classList.add('active');
    fetch('/horn?on=1').catch(() => {});
  };
  const hornEnd = (e) => {
    e.preventDefault();
    horn.classList.remove('active');
    fetch('/horn?on=0').catch(() => {});
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
    fetch('/speed?v=' + speedSlider.value).catch(() => {});
  });
  const servoSlider = document.getElementById('slide-servo');
  const servoVal = document.getElementById('val-servo');
  servoSlider.addEventListener('input', () => {
    const txt = servoSlider.value + '°';
    servoVal.textContent = txt;
    sendServo(servoSlider.value);
  });
  servoSlider.addEventListener('change', () => {
    fetch('/servo?a=' + servoSlider.value).catch(() => {});
  });
  // Mode Selection Tabs Switcher
  const changeMode = (mode) => {
    syncModeTab(mode);
    fetch('/set?mode=' + mode).catch(() => {});
  };
  const syncModeTab = (mode) => {
    const container = document.querySelector('.app-container');
    const selector = document.querySelector('.mode-selector');
    const tabOa = document.getElementById('tab-oa');
    const tabRc = document.getElementById('tab-rc');
    const panelOa = document.getElementById('panel-oa');
    const panelRc = document.getElementById('panel-rc');
    
    if (mode === 'oa') {
      tabOa.classList.add('active');
      tabRc.classList.remove('active');
      selector.classList.remove('rc-active');
      panelOa.classList.add('active');
      panelRc.classList.remove('active');
      container.className = 'app-container theme-oa';
    } else {
      tabOa.classList.remove('active');
      tabRc.classList.add('active');
      selector.classList.add('rc-active');
      panelOa.classList.remove('active');
      panelRc.classList.add('active');
      container.className = 'app-container theme-rc';
    }
  };
  // Telemetry updates
  const fetchTelemetry = () => {
    fetch('/status')
      .then(res => res.json())
      .then(data => {
        // Connected Badge
        const badge = document.getElementById('conn-badge');
        badge.textContent = 'CONNECTED';
        badge.style.background = 'rgba(16, 185, 129, 0.08)';
        badge.style.color = 'var(--accent-success)';
        badge.style.borderColor = 'rgba(16, 185, 129, 0.2)';
        
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
        
        // Update radar sweeps target trackers
        currentAngle = data.angle;
        
        // Push target readings to decay array
        addRadarTarget(data.angle, data.dist);
        addRadarTarget(150, data.dL);
        addRadarTarget(90, data.dC);
        addRadarTarget(30, data.dR);
        // Sync mode tab selection
        if (data.mode === 'oa' && !tabOa.classList.contains('active')) {
          syncModeTab('oa');
        } else if (data.mode === 'rc' && !tabRc.classList.contains('active')) {
          syncModeTab('rc');
        }
      })
      .catch(() => {
        const badge = document.getElementById('conn-badge');
        badge.textContent = 'OFFLINE';
        badge.style.background = 'rgba(244, 63, 94, 0.08)';
        badge.style.color = 'var(--accent-danger)';
        badge.style.borderColor = 'rgba(244, 63, 94, 0.2)';
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
    if (dist <= 0 || dist >= 100) return;
    const radAngle = angle;
    // check if point already plotted at this angle range (e.g. within 15deg)
    const existing = targetSpots.find(t => Math.abs(t.angle - radAngle) < 15);
    if (existing) {
      existing.dist = dist;
      existing.life = 1.0; // refresh decay path
    } else {
      targetSpots.push({ angle: radAngle, dist: dist, life: 1.0 });
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
    [30, 60, 90].forEach(rVal => {
      ctx.beginPath();
      ctx.arc(cx, cy, (rVal / 100) * maxR, 0, 2 * Math.PI);
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
    const isRC = document.querySelector('.app-container').classList.contains('theme-rc');
    const rgbColor = isRC ? '99, 102, 241' : '16, 185, 129';
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
    // Decay update & draw warning markers
    targetSpots.forEach(t => t.life -= 0.012); // slow fade out
    targetSpots = targetSpots.filter(t => t.life > 0);
    targetSpots.forEach(t => {
      const r = (t.dist / 100) * maxR;
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
      ctx.shadowBlur = 0; // reset shadow parameter
      // Pulsing outer halo rings
      ctx.beginPath();
      ctx.arc(tx, ty, 12 * (1 - t.life), 0, 2 * Math.PI);
      ctx.strokeStyle = `rgba(244, 63, 94, ${t.life * 0.45})`;
      ctx.lineWidth = 1.2;
      ctx.stroke();
    });
    requestAnimationFrame(renderRadar);
  };
  // Initialize loop
  requestAnimationFrame(renderRadar);
  // Poll server state every 350ms
  setInterval(fetchTelemetry, 350);
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
  String json = "{";
  json += "\"mode\":\"" + String(carMode == MODE_OA ? "oa" : "rc") + "\",";
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
  beep(100);
  Serial.println("System initialized. HTTP server running.");
}
// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();   // Always handle incoming HTTP requests
  // Update servo angle safely in loop context
  if (carMode == MODE_OA) {
    aimServo(TURN_ANGLE_C, false);
  } else {
    aimServo(rcServoAngle, false);
  }
  if (carMode == MODE_OA) {
    // ── Obstacle Avoidance (Autopilot Mode) ───────────────────
    long dC = distanceCM();
    int ir = analogRead(IR_PIN);
    lastDistance = dC;
    lastIR = ir;
    dC_dist = dC; // Update center distance for radar
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
  } else {
    // ── RC Web Control Mode ──────────────────────────────────
    // Buzzer control
    digitalWrite(BUZZER, rcHornOn ? HIGH : LOW);
    // Apply motor drive commands
    applyRcCmd(rcCmd, rcSpeed);
    // Update telemetry parameters asynchronously in the background
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 150) {
      lastSensorUpdate = millis();
      lastDistance = singlePingCM(15000UL); // Single ping with 15ms timeout (approx 250cm range)
      lastIR = analogRead(IR_PIN);
    }
    delay(15);
  }
}
