#include <SD.h>
#include <SPI.h>
#include <math.h>

// === Control Pins ===
const int PIN_DOWN   = 8;
const int PIN_LEFT   = 9;
const int PIN_UP     = 10;
const int PIN_RIGHT  = 11;

// === Monopulse Tuning Parameters ===
const float theta3dB    = 30.0;
const float thetaSquint = 13.0;
const float km = 2.0 * sqrt(2.0) * (thetaSquint/theta3dB) * log(2.0);

// === Motor Speeds (s/deg) ===
const float ELEV_SEC_PER_DEG = 67.0 / 180.0;
const float AZIM_SEC_PER_DEG = 67.0 / 450.0;

// === SD card ===
const int chipSelect = 53;
bool    sdOK        = false;

// === RSSI & target ===
float g0dBm, g1dBm, g2dBm, g3dBm;
float targetAz, targetEl;

void setup() {
  delay(5000);
  Serial.begin(115200);    // SX1280 #0
  Serial1.begin(115200);   // SX1280 #1
  Serial2.begin(115200);   // SX1280 #2
  Serial3.begin(115200);   // SX1280 #3

  pinMode(PIN_DOWN,  OUTPUT);
  pinMode(PIN_UP,    OUTPUT);
  pinMode(PIN_LEFT,  OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);
  stopAll();

  pinMode(chipSelect, OUTPUT);
  if (SD.begin(chipSelect)) {
    sdOK = true;
    if (SD.exists("data.txt")) SD.remove("data.txt");
    File f = SD.open("data.txt", FILE_WRITE);
    f.println("time_ms,g0dBm,g1dBm,g2dBm,g3dBm,targetAz,targetEl");
    f.close();
  }
}

void loop() {
  readRSSI();

  // --- flat‐field check: skip if all within 2 dBm ---
  float mx = max( max(g0dBm, g1dBm), max(g2dBm, g3dBm) );
  float mn = min( min(g0dBm, g1dBm), min(g2dBm, g3dBm) );
  if (mx - mn <= 2.0) {
    // all four are within ±2 dBm → ignore and reread
    targetAz = 0;
    targetEl = 0;
    stopAll();
    
  }
  else{
    computeTarget();
    moveToTarget();
  }

  
  logData();

  delay(20);  // ~50 Hz
}

void readRSSI() {
  if (Serial.available())   g0dBm = Serial.parseFloat();
  if (Serial1.available())  g1dBm = Serial1.parseFloat();
  if (Serial2.available())  g2dBm = Serial2.parseFloat();
  if (Serial3.available())  g3dBm = Serial3.parseFloat();
}

void computeTarget() {
  float g0 = pow(10.0, g0dBm/10.0) * 1e6;
  float g1 = pow(10.0, g1dBm/10.0) * 1e6;
  float g2 = pow(10.0, g2dBm/10.0) * 1e6;
  float g3 = pow(10.0, g3dBm/10.0) * 1e6;

  float sum    = g0 + g1 + g2 + g3;
  float diffAz = (g0 + g3) - (g1 + g2);
  float diffEl = (g0 + g1) - (g2 + g3);

  targetAz = (diffAz * theta3dB) / (sum * km);
  targetEl = (diffEl * theta3dB) / (sum * km);
}

void moveToTarget() {
  float dAz = targetAz;
  float dEl = targetEl;

  unsigned long tAz = min((unsigned long)(abs(dAz)*AZIM_SEC_PER_DEG*1000.0), 1000UL);
  unsigned long tEl = min((unsigned long)(abs(dEl)*ELEV_SEC_PER_DEG*1000.0), 1000UL);

  unsigned long t0 = millis();

  if (dAz > 0)      digitalWrite(PIN_RIGHT, HIGH);
  else if (dAz < 0) digitalWrite(PIN_LEFT,  HIGH);
  if (dEl > 0)      digitalWrite(PIN_UP,    HIGH);
  else if (dEl < 0) digitalWrite(PIN_DOWN,  HIGH);

  while (millis() - t0 < max(tAz, tEl)) {
    unsigned long dt = millis() - t0;
    if (dt >= tAz) {
      digitalWrite(PIN_RIGHT, LOW);
      digitalWrite(PIN_LEFT,  LOW);
    }
    if (dt >= tEl) {
      digitalWrite(PIN_UP,   LOW);
      digitalWrite(PIN_DOWN, LOW);
    }
  }

  stopAll();
}

void logData() {
  if (!sdOK) return;
  File f = SD.open("data.txt", FILE_WRITE);
  if (!f) return;
  f.print(millis());    f.print(',');
  f.print(g0dBm,2);     f.print(',');
  f.print(g1dBm,2);     f.print(',');
  f.print(g2dBm,2);     f.print(',');
  f.print(g3dBm,2);     f.print(',');
  f.print(targetAz,2);  f.print(',');
  f.println(targetEl,2);
  f.close();
}

void stopAll() {
  digitalWrite(PIN_DOWN,  LOW);
  digitalWrite(PIN_UP,    LOW);
  digitalWrite(PIN_LEFT,  LOW);
  digitalWrite(PIN_RIGHT, LOW);
}
