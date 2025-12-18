#include <Arduino.h>
#include <stdlib.h>

/*
  SlipGuard — Personal Safety Module (Standalone)

  This sketch keeps ONLY my module for defense/Wokwi:
  - HX711 pressure sampling
  - Slip detection (pressure drop / loss of contact)
  - Sound feedback (non-blocking, millis-based)
  - Emergency button sound (highest priority)
  - Serial Plotter output for pressure visualization

  Priority of sound output: emergency > slip > obstacle
*/

// ---------------------------- Hardware pins ----------------------------
const uint8_t PIN_HX711_DOUT = 3;          // HX711 DT
const uint8_t PIN_HX711_SCK = 2;           // HX711 SCK
const uint8_t PIN_BUZZER = 9;              // Active/passive buzzer (tone-capable pin)
const uint8_t PIN_EMERGENCY_BTN = 7;       // Push button to GND (uses INPUT_PULLUP)

// ---------------------------- Serial plotter ----------------------------
const unsigned long SERIAL_BAUD = 115200;
const unsigned long SERIAL_PLOT_INTERVAL_MS = 40; // Keep plot stable and readable

// ---------------------------- Pressure model ----------------------------
// If your HX711 reads "more negative" when pressing, set this to -1.
const int8_t PRESSURE_SIGN = 1;

// Raw HX711 units (must be tuned for your sensor + setup).
const long PRESSURE_CONTACT_THRESHOLD = 10000L;   // "Foot is on ground" threshold
const long PRESSURE_DROP_DELTA = 8000L;           // Sudden drop needed to flag slip
const unsigned long CONTACT_LOSS_TIMEOUT_MS = 180; // Contact lost too long => slip

// Slow baseline tracking: used as the "expected" pressure to detect drops.
const float BASELINE_ALPHA = 0.02f;

// Prevents repeated slip alarms from noise/vibration.
const unsigned long SLIP_REARM_MS = 600;

// ---------------------------- Obstacle sound ----------------------------
// This module keeps ONLY the sound mapping (no ultrasonic code).
// For a standalone demo you can set distance via Serial input (send a number + Enter).
const uint16_t OBSTACLE_DISTANCE_NEAR_CM = 15;
const uint16_t OBSTACLE_DISTANCE_FAR_CM = 150;
const unsigned long OBSTACLE_PERIOD_MIN_MS = 120; // Fast beeps (near)
const unsigned long OBSTACLE_PERIOD_MAX_MS = 900; // Slow beeps (far)
const unsigned long OBSTACLE_ON_MS = 30;
const uint16_t OBSTACLE_TONE_HZ = 1200;

// ---------------------------- Emergency sound ----------------------------
const unsigned long EMERGENCY_ON_MS = 260;
const unsigned long EMERGENCY_OFF_MS = 260;
const uint16_t EMERGENCY_TONE_HZ = 220; // Low-frequency repeating warning

// ---------------------------- Slip sound ----------------------------
const unsigned long SLIP_BEEP_ON_MS = 70;
const unsigned long SLIP_BEEP_GAP_MS = 70;
const uint16_t SLIP_TONE_HZ = 2400; // High-frequency double beep

// ---------------------------- Global state ----------------------------
long g_pressureRaw = 0;
float g_pressureBaseline = 0.0f;
bool g_baselineInitialized = false;

unsigned long g_contactBelowSinceMs = 0;
unsigned long g_lastSlipTriggerMs = 0;

// Slip alert is split into "pending" (detected) and "playing" (sound state machine),
// so a slip event isn't lost when emergency sound has priority.
bool g_slipAlertPending = false;
enum SlipSoundState : uint8_t { SLIP_IDLE, SLIP_BEEP1_ON, SLIP_GAP, SLIP_BEEP2_ON, SLIP_DONE };
SlipSoundState g_slipSoundState = SLIP_IDLE;
unsigned long g_slipNextChangeMs = 0;

// Emergency sound toggling state
bool g_emergencyToneOn = false;
unsigned long g_emergencyNextChangeMs = 0;

// Obstacle sound toggling state
bool g_obstacleToneOn = false;
unsigned long g_obstacleNextChangeMs = 0;

// Debounced emergency button state (stable "pressed" boolean)
const unsigned long BUTTON_DEBOUNCE_MS = 30;
bool g_btnLastReadingPressed = false;
bool g_btnStablePressed = false;
unsigned long g_btnLastEdgeMs = 0;

// Obstacle distance (cm). Default: "far away" => no obstacle beeps.
uint16_t g_obstacleDistanceCm = 999;
char g_distanceBuf[8];
uint8_t g_distanceBufLen = 0;

unsigned long g_lastPlotMs = 0;

// ---------------------------- HX711 (minimal, non-blocking) ----------------------------
// WHY: We avoid waiting loops so the sound scheduler stays responsive.
long readPressure() {
  // HX711 pulls DOUT LOW when a new 24-bit sample is ready.
  if (digitalRead(PIN_HX711_DOUT) == HIGH) {
    return g_pressureRaw; // Keep last value when not ready (non-blocking)
  }

  unsigned long value = 0;
  noInterrupts(); // Reduces timing jitter during the 24-bit shift
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(PIN_HX711_SCK, HIGH);
    value = (value << 1) | (digitalRead(PIN_HX711_DOUT) & 0x1);
    digitalWrite(PIN_HX711_SCK, LOW);
  }

  // 25th pulse sets gain/channel (1 pulse => Channel A, gain 128).
  digitalWrite(PIN_HX711_SCK, HIGH);
  digitalWrite(PIN_HX711_SCK, LOW);
  interrupts();

  // Sign-extend 24-bit two's complement to 32-bit.
  if (value & 0x800000UL) {
    value |= 0xFF000000UL;
  }

  g_pressureRaw = (long)value * PRESSURE_SIGN;
  return g_pressureRaw;
}

// ---------------------------- Input helpers ----------------------------
bool readEmergencyButton(unsigned long nowMs) {
  const bool pressedNow = (digitalRead(PIN_EMERGENCY_BTN) == LOW);

  // WHY: Debounce prevents rapid mode switching that would corrupt sound patterns.
  if (pressedNow != g_btnLastReadingPressed) {
    g_btnLastReadingPressed = pressedNow;
    g_btnLastEdgeMs = nowMs;
  }

  if (nowMs - g_btnLastEdgeMs >= BUTTON_DEBOUNCE_MS) {
    g_btnStablePressed = pressedNow;
  }

  return g_btnStablePressed;
}

void readObstacleDistanceFromSerial() {
  // WHY: Keep sketch standalone (no ultrasonic code) while still demonstrating obstacle beeps.
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (g_distanceBufLen == 0) {
        continue;
      }

      g_distanceBuf[g_distanceBufLen] = '\0';
      long v = strtol(g_distanceBuf, nullptr, 10);
      if (v < 0) v = 0;
      if (v > 500) v = 500;
      g_obstacleDistanceCm = (uint16_t)v;
      g_distanceBufLen = 0;
      continue;
    }

    if (c >= '0' && c <= '9') {
      if (g_distanceBufLen < sizeof(g_distanceBuf) - 1) {
        g_distanceBuf[g_distanceBufLen++] = c;
      }
      continue;
    }

    // Ignore any non-numeric bytes so Serial Plotter stays purely numeric.
  }
}

// ---------------------------- Slip detection ----------------------------
bool slipDetection(unsigned long nowMs, long pressureRaw) {
  // WHY: A slow baseline gives us an "expected" pressure to detect sudden drops.
  if (!g_baselineInitialized) {
    g_pressureBaseline = (float)pressureRaw;
    g_baselineInitialized = true;
  }

  const bool inContactNow = (pressureRaw >= PRESSURE_CONTACT_THRESHOLD);
  if (inContactNow) {
    g_pressureBaseline += BASELINE_ALPHA * ((float)pressureRaw - g_pressureBaseline);
    g_contactBelowSinceMs = 0;
  } else if (g_contactBelowSinceMs == 0) {
    g_contactBelowSinceMs = nowMs;
  }

  const long baseline = (long)g_pressureBaseline;
  const bool baselineIsValid = (baseline >= PRESSURE_CONTACT_THRESHOLD);

  // Two independent slip triggers:
  // 1) Sudden drop relative to the baseline (fast loss of traction/contact).
  const bool suddenDrop = baselineIsValid && ((baseline - pressureRaw) >= PRESSURE_DROP_DELTA);

  // 2) Pressure stays below threshold long enough (loss of stable ground contact).
  const bool lostContactTooLong =
    baselineIsValid &&
    (g_contactBelowSinceMs != 0) &&
    (nowMs - g_contactBelowSinceMs >= CONTACT_LOSS_TIMEOUT_MS);

  const bool slipCondition = suddenDrop || lostContactTooLong;
  const bool rearmed = (nowMs - g_lastSlipTriggerMs >= SLIP_REARM_MS);
  const bool slipSoundBusy = (g_slipSoundState != SLIP_IDLE) || g_slipAlertPending;

  if (slipCondition && rearmed && !slipSoundBusy) {
    g_slipAlertPending = true;
    g_lastSlipTriggerMs = nowMs;
  }

  return (g_slipAlertPending || (g_slipSoundState != SLIP_IDLE));
}

// ---------------------------- Sound engines (millis-based) ----------------------------
void emergencySound(unsigned long nowMs, bool active) {
  if (!active) {
    g_emergencyToneOn = false;
    g_emergencyNextChangeMs = 0;
    return;
  }

  if (g_emergencyNextChangeMs == 0) {
    g_emergencyToneOn = true;
    tone(PIN_BUZZER, EMERGENCY_TONE_HZ);
    g_emergencyNextChangeMs = nowMs + EMERGENCY_ON_MS;
    return;
  }

  if ((long)(nowMs - g_emergencyNextChangeMs) < 0) {
    return;
  }

  if (g_emergencyToneOn) {
    noTone(PIN_BUZZER);
    g_emergencyToneOn = false;
    g_emergencyNextChangeMs = nowMs + EMERGENCY_OFF_MS;
  } else {
    tone(PIN_BUZZER, EMERGENCY_TONE_HZ);
    g_emergencyToneOn = true;
    g_emergencyNextChangeMs = nowMs + EMERGENCY_ON_MS;
  }
}

void slipSound(unsigned long nowMs) {
  // WHY: This is a millis-based double-beep (no delay), so pressure sampling keeps running.
  if (g_slipSoundState == SLIP_IDLE) {
    if (!g_slipAlertPending) {
      noTone(PIN_BUZZER);
      return;
    }

    g_slipAlertPending = false;
    g_slipSoundState = SLIP_BEEP1_ON;
    g_slipNextChangeMs = nowMs;
  }

  if ((long)(nowMs - g_slipNextChangeMs) < 0) {
    return;
  }

  switch (g_slipSoundState) {
    case SLIP_BEEP1_ON:
      tone(PIN_BUZZER, SLIP_TONE_HZ);
      g_slipSoundState = SLIP_GAP;
      g_slipNextChangeMs = nowMs + SLIP_BEEP_ON_MS;
      break;

    case SLIP_GAP:
      noTone(PIN_BUZZER);
      g_slipSoundState = SLIP_BEEP2_ON;
      g_slipNextChangeMs = nowMs + SLIP_BEEP_GAP_MS;
      break;

    case SLIP_BEEP2_ON:
      tone(PIN_BUZZER, SLIP_TONE_HZ);
      g_slipSoundState = SLIP_DONE;
      g_slipNextChangeMs = nowMs + SLIP_BEEP_ON_MS;
      break;

    case SLIP_DONE:
      noTone(PIN_BUZZER);
      g_slipSoundState = SLIP_IDLE;
      g_slipNextChangeMs = 0;
      break;

    default:
      g_slipSoundState = SLIP_IDLE;
      g_slipNextChangeMs = 0;
      noTone(PIN_BUZZER);
      break;
  }
}

unsigned long obstaclePeriodMs(uint16_t distanceCm) {
  if (distanceCm <= OBSTACLE_DISTANCE_NEAR_CM) return OBSTACLE_PERIOD_MIN_MS;
  if (distanceCm >= OBSTACLE_DISTANCE_FAR_CM) return OBSTACLE_PERIOD_MAX_MS;

  const unsigned long span = (unsigned long)(OBSTACLE_DISTANCE_FAR_CM - OBSTACLE_DISTANCE_NEAR_CM);
  const unsigned long offset = (unsigned long)(distanceCm - OBSTACLE_DISTANCE_NEAR_CM);
  return OBSTACLE_PERIOD_MIN_MS + (offset * (OBSTACLE_PERIOD_MAX_MS - OBSTACLE_PERIOD_MIN_MS)) / span;
}

void obstacleSound(unsigned long nowMs, bool active, uint16_t distanceCm) {
  if (!active) {
    g_obstacleToneOn = false;
    g_obstacleNextChangeMs = 0;
    return;
  }

  if (distanceCm > OBSTACLE_DISTANCE_FAR_CM) {
    noTone(PIN_BUZZER);
    g_obstacleToneOn = false;
    g_obstacleNextChangeMs = 0;
    return;
  }

  const unsigned long periodMs = obstaclePeriodMs(distanceCm);
  const unsigned long onMs = min(OBSTACLE_ON_MS, periodMs / 2);
  const unsigned long offMs = periodMs - onMs;

  if (g_obstacleNextChangeMs == 0) {
    g_obstacleToneOn = true;
    tone(PIN_BUZZER, OBSTACLE_TONE_HZ);
    g_obstacleNextChangeMs = nowMs + onMs;
    return;
  }

  if ((long)(nowMs - g_obstacleNextChangeMs) < 0) {
    return;
  }

  if (g_obstacleToneOn) {
    noTone(PIN_BUZZER);
    g_obstacleToneOn = false;
    g_obstacleNextChangeMs = nowMs + offMs;
  } else {
    tone(PIN_BUZZER, OBSTACLE_TONE_HZ);
    g_obstacleToneOn = true;
    g_obstacleNextChangeMs = nowMs + onMs;
  }
}

// ---------------------------- Serial plot output ----------------------------
void plotPressure(unsigned long nowMs, long pressureRaw) {
  if (nowMs - g_lastPlotMs < SERIAL_PLOT_INTERVAL_MS) return;
  g_lastPlotMs = nowMs;

  // WHY: Multiple lines help a defense demo (pressure, baseline, threshold, slip marker).
  Serial.print(pressureRaw);
  Serial.print('\t');
  Serial.print((long)g_pressureBaseline);
  Serial.print('\t');
  Serial.print(PRESSURE_CONTACT_THRESHOLD);
  Serial.print('\t');
  Serial.println((g_slipAlertPending || g_slipSoundState != SLIP_IDLE) ? (PRESSURE_CONTACT_THRESHOLD + PRESSURE_DROP_DELTA) : 0);
}

void setup() {
  pinMode(PIN_HX711_SCK, OUTPUT);
  digitalWrite(PIN_HX711_SCK, LOW);
  pinMode(PIN_HX711_DOUT, INPUT);

  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  pinMode(PIN_EMERGENCY_BTN, INPUT_PULLUP);

  Serial.begin(SERIAL_BAUD);

  // Initialize debounce state from the current electrical level.
  g_btnLastReadingPressed = (digitalRead(PIN_EMERGENCY_BTN) == LOW);
  g_btnStablePressed = g_btnLastReadingPressed;
  g_btnLastEdgeMs = millis();
}

void loop() {
  const unsigned long nowMs = millis();

  // Optional: allows obstacle beep demo without ultrasonic code.
  readObstacleDistanceFromSerial();

  const long pressure = readPressure();
  const bool emergencyPressed = readEmergencyButton(nowMs);
  const bool slipActive = slipDetection(nowMs, pressure);

  // Priority: emergency > slip > obstacle
  if (emergencyPressed) {
    emergencySound(nowMs, true);
    obstacleSound(nowMs, false, g_obstacleDistanceCm);
  } else if (slipActive) {
    emergencySound(nowMs, false);
    slipSound(nowMs);
    obstacleSound(nowMs, false, g_obstacleDistanceCm);
  } else {
    emergencySound(nowMs, false);
    obstacleSound(nowMs, true, g_obstacleDistanceCm);
  }

  plotPressure(nowMs, pressure);
}
