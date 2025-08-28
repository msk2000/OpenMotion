/******************************************************************************
 * @file    StewartServoReceiver.ino
 * @brief   Serial-controlled 6-servo receiver for a Stewart (6-DOF) platform.
 *
 * Overview
 *   - Receives newline-terminated CSV: "a,b,c,d,e,f\n" where each a..f ∈ [0,180].
 *   - Applies per-servo inversion, trim, and mechanical clamps before commanding.
 *   - Optional input convention: treat 0° as horizontal (adds +90° offset).
 *   - Optional microsecond pulse mode for fine calibration (writeMicroseconds).
 *   - Non-blocking line parser; echoes applied physical angles for debugging.
 *
 * Hardware & Pins
 *   - Servo signal pins: 3, 5, 6, 9, 10, 11 (see SERVO_PINS[]).
 *   - Use a dedicated 5–6 V servo supply; DO NOT power servos from Arduino 5 V.
 *   - MUST share a common GND between the servo PSU and the Arduino.
 *   - Consider bulk decoupling near each servo rail branch; keep signal returns short.
 *
 * Serial Link
 *   - Baud: 115200 8N1.
 *   - Boot banner: "READY".
 *   - On valid CSV: replies "OK:x,x,x,x,x,x\r\n" (post-inversion/trim/clamp).
 *   - Text commands:
 *       • "HOME" → drives all servos to HOME_ANGLE (default 90 = horizontal).
 *       • "PING" → replies "PONG".
 *   - Errors: "ERR:PARSE" for malformed lines or wrong field count.
 *
 * Coordinate & Mapping Conventions
 *   - HOME_ANGLE = 90° corresponds to horn horizontal (full up/down headroom).
 *   - INVERT[i] flips odd indices so paired horns face each other mechanically.
 *   - INPUT_ZERO_IS_HORIZONTAL=true interprets incoming 0° as physical 90°
 *     (cannot command below horizontal in that mode).
 *   - TRIM_DEG[i] applies per-channel fine offsets after inversion.
 *   - MIN_DEG[i]/MAX_DEG[i] enforce mechanical safety limits after trim.
 *   - USE_US=true switches to microsecond pulses; tune SERVO_MIN_US/MAX_US.
 *
 * Timing & Throughput
 *   - The Arduino Servo library updates ≈50 Hz; sending faster (e.g., 60–100 Hz)
 *     is acceptable but servos won’t actuate faster than their internal refresh.
 *   - Parser is non-blocking; processes complete lines as bytes arrive.
 *   - Line length is bounded to prevent runaway buffers.
 *
 * Integration Tips
 *   - Send CSV at a steady rate (e.g., 30–60 Hz) with a trailing '\n'.
 *   - Issue "HOME" once at session start to align with the mechanical neutral.
 *   - If using microseconds, characterize each servo for linearity and range.
 *
 * Safety Notes
 *   - Software clamps mitigate but do not eliminate mechanical interference.
 *   - Verify linkage limits offline before live motion; keep emergency power cut.
 *
 * Part of the Open Motion project (6-DOF Motion Simulator).
 ******************************************************************************/


#include <Servo.h>

// -------------------- Config --------------------
static const uint8_t NUM_SERVOS = 6;
static const uint8_t SERVO_PINS[NUM_SERVOS] = {3,5,6,9,10,11};

// Home pose at power‑on (use 90 = horizontal for full up/down travel)
static const int HOME_ANGLE = 90;

// Invert odd indices: 0,2,4 normal; 1,3,5 inverted
static const bool INVERT[NUM_SERVOS] = {false, true, false, true, false, true};

// Fine trims (deg) applied after inversion: Once we get thigns level in hardware
// and boot, this helps us fine tune any offsets due to mechanical imperfections
static const int8_t TRIM_DEG[NUM_SERVOS] = {0,0,0,0,0,0};

// Mechanical safety limits (deg) after trim
static const uint8_t MIN_DEG[NUM_SERVOS] = {0,0,0,0,0,0};
static const uint8_t MAX_DEG[NUM_SERVOS] = {180,180,180,180,180,180};

// --- Choose input convention ---
static const bool INPUT_ZERO_IS_HORIZONTAL = false;
// If true: logical 0 = physical 90 (adds +90). You cannot go below horizontal.

// For future fine tuning: drive by microseconds(US) instead of degrees
static const bool USE_US = false;
static const int  SERVO_MIN_US = 1000;
static const int  SERVO_MAX_US = 2000;

// -------------------- State --------------------
Servo servos[NUM_SERVOS];
int current_deg[NUM_SERVOS];  // last commanded physical degrees

// -------------------- Helpers ------------------
static inline int clampi(int v, int lo, int hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int degToMicros(int deg)
{
  const long span = (long)SERVO_MAX_US - (long)SERVO_MIN_US;
  return SERVO_MIN_US + (int)(span * (long)clampi(deg,0,180) / 180L);
}

// Map a single logical input to a physical horn angle
int logicalToPhysical(uint8_t idx, int logical_deg)
{
  // 0) optional shift if data sender still uses 0 = horizontal
  if (INPUT_ZERO_IS_HORIZONTAL) logical_deg += 90;

  // 1) clamp input
  int a = clampi(logical_deg, 0, 180);

  // 2) paired inversion (so mates face each other)
  if (INVERT[idx]) a = 180 - a;

  // 3) trim and final clamp to protek mechanics!
  a += TRIM_DEG[idx];
  a = clampi(a, MIN_DEG[idx], MAX_DEG[idx]);
  return a;
}

// -------------------- Serial parsing -----------
String lineBuf;

bool parseLine6(const char* s, int out[NUM_SERVOS])
{
  if (!s) return false;
  char buf[64]; // enough for "180,180,180,180,180,180"
  size_t n = 0;
  while (s[n] && s[n] != '\n' && s[n] != '\r' && n < sizeof(buf)-1) buf[n++] = s[n];
  buf[n] = 0;

  uint8_t i = 0;
  char* tok = strtok(buf, ", ");
  while (tok && i < NUM_SERVOS)
  {
    char* endp = nullptr;
    long v = strtol(tok, &endp, 10);
    if (endp == tok) return false;
    out[i++] = (int)v;
    tok = strtok(nullptr, ", ");
  }
  return i == NUM_SERVOS;
}

void processIfLineComplete()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == '\n')
    {
      int raw[NUM_SERVOS];
      if (parseLine6(lineBuf.c_str(), raw))
      {
        for (uint8_t i=0;i<NUM_SERVOS;++i)
        {
          const int phys = logicalToPhysical(i, raw[i]);
          current_deg[i] = phys;
          if (USE_US) servos[i].writeMicroseconds(degToMicros(phys));
          else        servos[i].write(phys);
        }
        // echo for debugging
        Serial.print("OK:");
        for (uint8_t i=0;i<NUM_SERVOS;++i){ Serial.print(current_deg[i]); if(i+1<NUM_SERVOS) Serial.print(','); }
        Serial.print("\r\n");
      }
      else
      {
        // Simple text commands
        if (lineBuf.equalsIgnoreCase("HOME"))
        {
          for (uint8_t i=0;i<NUM_SERVOS;++i)
          {
            const int phys = logicalToPhysical(i, HOME_ANGLE);
            current_deg[i] = phys;
            if (USE_US) servos[i].writeMicroseconds(degToMicros(phys));
            else        servos[i].write(phys);
          }
          Serial.println("OK:HOME");
        }
        else if (lineBuf.equalsIgnoreCase("PING"))
        {
          Serial.println("PONG");
        }
        else
        {
          Serial.println("ERR:PARSE");
        }
      }
      lineBuf.remove(0);
    }
    else if (c != '\r')
    {
      if (lineBuf.length() < 60) lineBuf += c;
      else lineBuf.remove(0); // prevent runaway
    }
  }
}

// -------------------- Arduino lifecycle --------
void setup()
{
  Serial.begin(115200);
  delay(200);

  for (uint8_t i=0;i<NUM_SERVOS;++i)
  {
    servos[i].attach(SERVO_PINS[i]);
  }

  // Power‑on HOME: use 90 = horizontal so we have up and down headroom
  for (uint8_t i=0;i<NUM_SERVOS;++i)
  {
    const int phys = logicalToPhysical(i, HOME_ANGLE);
    current_deg[i] = phys;
    if (USE_US) servos[i].writeMicroseconds(degToMicros(phys));
    else        servos[i].write(phys);
  }

  Serial.println("READY");
}

void loop()
{
  processIfLineComplete();
}
