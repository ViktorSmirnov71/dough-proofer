// Reaction-chamber control panel - Analog button matrix + Grove RGB LCD +
// ultrasonic dough-height tracking on UNO R4 WiFi.
//
// Hardware:
//   5V --[1k]--+-- A0                       (R1 = series resistor)
//              |
//              +--[ SW1 ]-- GND             (short,  R2 = 0       -> Vout ~ 0.00 V, ADC ~   0)
//              +--[180R]--[ SW2 ]-- GND     (R2 = 180             -> Vout ~ 0.76 V, ADC ~ 156)
//              +--[240R]--[ SW3 ]-- GND     (R2 = 240             -> Vout ~ 0.97 V, ADC ~ 198)
//              +--[330R]--[ SW4 ]-- GND     (R2 = 330             -> Vout ~ 1.24 V, ADC ~ 254)
//   Nothing pressed -> A0 pulled up via 1k  (Vout ~ 5.00 V, ADC ~ 1023)
//
//   Grove RGB LCD 2x16 plugs into any I2C socket on the Grove Base Shield
//   (SDA = A4, SCL = A5 on the UNO R4 header; the shield routes both).
//
//   Grove Ultrasonic Distance Sensor v2 (single-wire signal):
//       Plug a Grove cable from the sensor's socket to the D2 socket on the
//       Base Shield. The cable carries VCC, GND, and the bidirectional
//       signal line; only the primary signal (yellow, = D2) is used. The
//       module handles its own logic-level conversion internally.
//
// Design notes:
//   - One ADC pin for N buttons frees digital pins for the heater relay
//     and additional sensors that the later iterations bring in.
//   - Thresholds come straight from the voltage-divider math; the wider
//     upper band on B4 absorbs resistor tolerance and series-resistor drift.
//   - 5 ms sampling x 6 stable samples = ~30 ms debounce: rides out tactile
//     switch bounce while still feeling instant.
//   - Edge-trigger only on idle->press; release fires nothing so holding a
//     button never spams events.
//
// UI semantics:
//   B1 STOP : reset state to defaults (setpoint 32 C, TEMP mode, sim restart,
//             height baseline cleared)
//   B2 MODE : cycle pages TEMP -> HUMIDITY -> HEIGHT -> TEMP ...
//             Entering HEIGHT auto-tares so the display starts at 0 mm.
//   B3  -   : TEMP page    -> setpoint -1 C  (min 20 C)
//             HEIGHT page  -> re-tare the height baseline at current reading
//   B4  +   : TEMP page    -> setpoint +1 C  (max 45 C)
//             HEIGHT page  -> no-op

#include <Wire.h>
#include <math.h>
#include "rgb_lcd.h"

// ---- Pin map ---------------------------------------------------------------
static const uint8_t PIN_BUTTONS = A0;
static const uint8_t PIN_SONAR   = 2;     // Grove Ultrasonic v2 single-wire signal

// ---- ADC -> voltage conversion ---------------------------------------------
static const float ADC_TO_V = 5.0f / 1024.0f;

// ---- Button classification -------------------------------------------------
enum Button : uint8_t { BTN_NONE, BTN_B1, BTN_B2, BTN_B3, BTN_B4 };

struct Band { Button id; int lo; int hi; };
static const Band BANDS[] = {
  { BTN_B1,    0,   60 },
  { BTN_B2,  120,  185 },
  { BTN_B3,  186,  220 },
  { BTN_B4,  240,  450 },
  { BTN_NONE, 900, 1023 }
};

static Button classify(int adc) {
  for (const Band& b : BANDS) {
    if (adc >= b.lo && adc <= b.hi) return b.id;
  }
  return BTN_NONE;
}

// ---- Debounce state --------------------------------------------------------
static const unsigned long SAMPLE_MS = 5;
static const uint8_t       STABLE_SAMPLES = 6;

static Button   candidate    = BTN_NONE;
static uint8_t  stableCount  = 0;
static Button   confirmed    = BTN_NONE;
static unsigned long nextSampleMs = 0;

// ---- App state -------------------------------------------------------------
enum Mode : uint8_t { MODE_TEMP = 0, MODE_HUM = 1, MODE_HEIGHT = 2, MODE_N };
static const char* MODE_NAME[MODE_N] = { "TEMP", "HUM ", "HGHT" };

static int      setpointC      = 32;
static Mode     displayMode    = MODE_TEMP;
static uint32_t pressCount     = 0;
static Button   lastPressed    = BTN_NONE;
static int      lastAdc        = 0;
static bool     verbose        = false;
static bool     distStream     = false;
static unsigned long nextVerboseMs = 0;
static unsigned long nextDistMs    = 0;

// Stubbed chamber readings -- replaced by real sensor reads in the next
// iteration. They live in the same units the LCD will show so the swap-in
// is mechanical.
static float    currentTempC   = 22.0f;
static float    currentHumPct  = 58.0f;
static unsigned long nextSimMs = 0;

// Distance / dough-height state. currentDistanceMm is the latest raw read
// (-1 = no echo / sensor not present). baselineMm is the "empty / starting"
// reference set when entering HEIGHT mode or re-tared with B3 -- dough rise
// is then baselineMm - currentDistanceMm.
static long currentDistanceMm = -1;
static long baselineMm        = -1;

static Mode     bgMode         = MODE_N;

// ---- LCD -------------------------------------------------------------------
rgb_lcd lcd;

static const char* labelOf(Button b) {
  switch (b) {
    case BTN_B1: return "STOP";
    case BTN_B2: return "MODE";
    case BTN_B3: return " -  ";
    case BTN_B4: return " +  ";
    default:     return "----";
  }
}

static void applyBacklight() {
  if (displayMode == bgMode) return;
  bgMode = displayMode;
  if      (displayMode == MODE_TEMP)   lcd.setRGB(200, 110,  20);  // warm amber
  else if (displayMode == MODE_HUM)    lcd.setRGB( 20, 150, 200);  // cool cyan
  else                                 lcd.setRGB( 40, 200, 100);  // fresh green
}

static void renderLCD() {
  applyBacklight();
  char l1[17], l2[17];
  if (displayMode == MODE_TEMP) {
    int t10 = (int)lroundf(currentTempC * 10.0f);
    snprintf(l1, sizeof(l1), "TEMP*  %2d.%1d C   ", t10 / 10, abs(t10) % 10);
    snprintf(l2, sizeof(l2), "Set %2d C   [+/-]", setpointC);
  } else if (displayMode == MODE_HUM) {
    int h10 = (int)lroundf(currentHumPct * 10.0f);
    snprintf(l1, sizeof(l1), "HUM*   %2d.%1d %%   ", h10 / 10, abs(h10) % 10);
    snprintf(l2, sizeof(l2), "display only    ");
  } else {  // MODE_HEIGHT
    if (currentDistanceMm < 0) {
      snprintf(l1, sizeof(l1), "HEIGHT  -- mm   ");
      snprintf(l2, sizeof(l2), "no echo signal  ");
    } else {
      long rise = (baselineMm >= 0) ? (baselineMm - currentDistanceMm) : 0;
      snprintf(l1, sizeof(l1), "HEIGHT %+5ld mm ", rise);
      snprintf(l2, sizeof(l2), "raw %4ld  retare", currentDistanceMm);
    }
  }
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

// Grove Ultrasonic v2 single-pin ping. The signal line is bidirectional:
// pulse HIGH ~5 us to trigger, then switch the pin to INPUT and measure
// the echo pulse width. Returns distance in mm or -1 on timeout / no echo.
// The 30 ms timeout caps worst-case blocking (~5 m round trip).
static long readDistanceMm() {
  pinMode(PIN_SONAR, OUTPUT);
  digitalWrite(PIN_SONAR, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONAR, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_SONAR, LOW);
  pinMode(PIN_SONAR, INPUT);
  unsigned long durUs = pulseIn(PIN_SONAR, HIGH, 30000UL);
  if (durUs == 0) return -1;
  // Speed of sound ~ 343 m/s -> mm = us * 343 / 2 / 1000
  // Integer-safe form: mm = us * 343 / 2000.
  return (long)durUs * 343L / 2000L;
}

// Simulated chamber dynamics + live sonar sample. Temperature relaxes
// first-order toward the setpoint (heater bringing the chamber up);
// humidity wobbles +/-3 % around a 58 % baseline on a 30 s period
// (deterministic so the demo is reproducible). The distance read is
// always live -- real temp/humidity sensors will replace the stub lines.
static void stepSimulation(unsigned long now) {
  currentTempC      += (setpointC - currentTempC) * 0.05f;
  currentHumPct      = 58.0f + 3.0f * sinf(now / 30000.0f * 6.2831853f);
  currentDistanceMm  = readDistanceMm();
}

static void onPress(Button b, int adcAtPress) {
  pressCount++;
  lastPressed = b;
  bool ignored = false;
  switch (b) {
    case BTN_B1:
      setpointC      = 32;
      displayMode    = MODE_TEMP;
      currentTempC   = 22.0f;
      baselineMm     = -1;
      break;
    case BTN_B2:
      displayMode = (Mode)((displayMode + 1) % MODE_N);
      // Entering HEIGHT auto-tares to the current distance so the display
      // starts at 0 mm and counts up as the dough rises.
      if (displayMode == MODE_HEIGHT) {
        long mm = readDistanceMm();
        if (mm >= 0) { baselineMm = mm; currentDistanceMm = mm; }
      }
      break;
    case BTN_B3:
      if (displayMode == MODE_TEMP) {
        if (setpointC > 20) setpointC--;
      } else if (displayMode == MODE_HEIGHT) {
        long mm = readDistanceMm();
        if (mm >= 0) { baselineMm = mm; currentDistanceMm = mm; }
      } else {
        ignored = true;
      }
      break;
    case BTN_B4:
      if (displayMode == MODE_TEMP) {
        if (setpointC < 45) setpointC++;
      } else {
        ignored = true;
      }
      break;
    default: break;
  }
  Serial.print(F("press="));     Serial.print(labelOf(b));
  Serial.print(F(" adc="));      Serial.print(adcAtPress);
  Serial.print(F(" V="));        Serial.print(adcAtPress * ADC_TO_V, 3);
  Serial.print(F(" mode="));     Serial.print(MODE_NAME[displayMode]);
  Serial.print(F(" set="));      Serial.print(setpointC);
  if (ignored) Serial.print(F(" (no-op in this mode)"));
  Serial.print(F(" n="));        Serial.println(pressCount);
  renderLCD();
}

// ---- Arduino entry points --------------------------------------------------
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 1500) { /* spin */ }

  // Single-wire Grove sonar: direction is flipped per ping inside readDistanceMm().
  pinMode(PIN_SONAR, INPUT);

  Wire.begin();
  lcd.begin(16, 2);
  lcd.setRGB(0, 200, 80);
  lcd.print("Dough proofer");
  lcd.setCursor(0, 1);
  lcd.print("warming up...");
  delay(800);
  renderLCD();

  Serial.println(F("Proofer panel ready."));
  Serial.println(F("  STOP  : reset to defaults"));
  Serial.println(F("  MODE  : cycle TEMP -> HUMIDITY -> HEIGHT"));
  Serial.println(F("  + / - : TEMP page  -> adjust setpoint"));
  Serial.println(F("          HEIGHT (B3 only) -> re-tare baseline"));
  Serial.println(F("  c     : one-shot raw A0 read"));
  Serial.println(F("  v     : toggle continuous A0 stream (every 200 ms)"));
  Serial.println(F("  p     : single ultrasonic ping with full diagnostics"));
  Serial.println(F("  d     : toggle continuous distance stream (every 500 ms)"));
}

void loop() {
  unsigned long now = millis();

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c') {
      int raw = analogRead(PIN_BUTTONS);
      Serial.print(F("A0 raw = "));
      Serial.print(raw);
      Serial.print(F("  ("));
      Serial.print(raw * ADC_TO_V, 3);
      Serial.println(F(" V)"));
    } else if (ch == 'v') {
      verbose = !verbose;
      Serial.print(F("verbose=")); Serial.println(verbose ? F("ON") : F("OFF"));
    } else if (ch == 'd') {
      distStream = !distStream;
      Serial.print(F("distStream=")); Serial.println(distStream ? F("ON") : F("OFF"));
    } else if (ch == 'p') {
      pinMode(PIN_SONAR, INPUT);
      int idleBefore = digitalRead(PIN_SONAR);
      pinMode(PIN_SONAR, OUTPUT);
      digitalWrite(PIN_SONAR, LOW);  delayMicroseconds(2);
      digitalWrite(PIN_SONAR, HIGH); delayMicroseconds(5);
      digitalWrite(PIN_SONAR, LOW);
      pinMode(PIN_SONAR, INPUT);
      unsigned long t0 = micros();
      unsigned long us = pulseIn(PIN_SONAR, HIGH, 30000UL);
      unsigned long elapsed = micros() - t0;
      int idleAfter = digitalRead(PIN_SONAR);
      Serial.print(F("ping: sig_idle_before="));    Serial.print(idleBefore);
      Serial.print(F("  pulseIn_us="));             Serial.print(us);
      Serial.print(F("  elapsed_us="));             Serial.print(elapsed);
      Serial.print(F("  sig_after="));              Serial.print(idleAfter);
      if (us > 0) {
        Serial.print(F("  mm="));
        Serial.println((long)us * 343L / 2000L);
      } else {
        Serial.println(F("  -> NO ECHO (sensor not pinging or out of range)"));
      }
    }
  }

  if (verbose && now >= nextVerboseMs) {
    nextVerboseMs = now + 200;
    int raw = analogRead(PIN_BUTTONS);
    Button thinks = classify(raw);
    Serial.print(F("[stream] A0="));
    Serial.print(raw);
    Serial.print(F("  V="));
    Serial.print(raw * ADC_TO_V, 3);
    Serial.print(F("  thinks="));
    Serial.println(labelOf(thinks));
  }

  if (now >= nextSimMs) {
    nextSimMs = now + 1000;
    stepSimulation(now);
    renderLCD();
  }

  if (distStream && now >= nextDistMs) {
    nextDistMs = now + 500;
    long mm = readDistanceMm();
    Serial.print(F("[dist] mm="));
    if (mm < 0) Serial.println(F("-- (no echo)"));
    else        Serial.println(mm);
  }

  if (now < nextSampleMs) return;
  nextSampleMs = now + SAMPLE_MS;

  int     adc   = analogRead(PIN_BUTTONS);
  lastAdc       = adc;
  Button  fresh = classify(adc);

  if (fresh == candidate) {
    if (stableCount < 255) stableCount++;
  } else {
    candidate   = fresh;
    stableCount = 1;
  }

  if (stableCount == STABLE_SAMPLES && candidate != confirmed) {
    Button prev = confirmed;
    confirmed   = candidate;
    if (prev == BTN_NONE && confirmed != BTN_NONE) onPress(confirmed, adc);
  }
}
