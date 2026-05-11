// Workshop 1 - Analog-divider button input + Grove RGB LCD on UNO R4 WiFi.
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
// Design notes:
//   - One ADC pin for N buttons frees digital pins for the heater relay
//     and additional sensors that the later workshops will bring in.
//   - Thresholds come straight from the slide-deck math; the wider upper
//     band on B4 absorbs resistor tolerance and series-resistor drift.
//   - 5 ms sampling x 6 stable samples = ~30 ms debounce: rides out tactile
//     switch bounce while still feeling instant.
//   - Edge-trigger only on idle->press; release fires nothing so holding a
//     button never spams events.

#include <Wire.h>
#include "rgb_lcd.h"

// ---- Pin map ---------------------------------------------------------------
static const uint8_t PIN_BUTTONS = A0;

// ---- ADC -> voltage conversion ---------------------------------------------
// Slide convention: 5 V / 1024 = 4.88 mV per ADC count (V_ref = 5 V, 10-bit).
static const float ADC_TO_V = 5.0f / 1024.0f;

// ---- Button classification -------------------------------------------------
enum Button : uint8_t { BTN_NONE, BTN_B1, BTN_B2, BTN_B3, BTN_B4 };

// Bands are tuned to the expected ADC codes from the slide examples, with
// gaps between them so a sample mid-transition resolves to NONE rather
// than a phantom button. Re-tune by sending 'c' over serial while a button
// is held (see calibration block in loop()).
struct Band { Button id; int lo; int hi; };
static const Band BANDS[] = {
  { BTN_B1,    0,   60 },   // ~0
  { BTN_B2,  120,  185 },   // ~156
  { BTN_B3,  186,  220 },   // ~198
  { BTN_B4,  240,  450 },   // ~254 (wide: tolerates ~+/-30% on the 330R leg)
  { BTN_NONE, 900, 1023 }   // pulled-up idle
};

static Button classify(int adc) {
  for (const Band& b : BANDS) {
    if (adc >= b.lo && adc <= b.hi) return b.id;
  }
  return BTN_NONE;  // dead-zone between bands -> treat as idle
}

// ---- Debounce state --------------------------------------------------------
static const unsigned long SAMPLE_MS = 5;
static const uint8_t       STABLE_SAMPLES = 6;

static Button   candidate    = BTN_NONE;
static uint8_t  stableCount  = 0;
static Button   confirmed    = BTN_NONE;
static unsigned long nextSampleMs = 0;

// ---- App state -------------------------------------------------------------
static int      setpointC   = 32;
static uint32_t pressCount  = 0;
static Button   lastPressed = BTN_NONE;

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

static void renderLCD() {
  // Pad to full 16 chars per line so leftover text from a longer line never
  // lingers when state changes.
  char l1[17], l2[17];
  snprintf(l1, sizeof(l1), "Setpoint: %2d C  ", setpointC);
  snprintf(l2, sizeof(l2), "[%s] n=%-7lu",     labelOf(lastPressed),
                                               (unsigned long)pressCount);
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

static void onPress(Button b, int adcAtPress) {
  pressCount++;
  lastPressed = b;
  switch (b) {
    case BTN_B1:                                  lcd.setRGB(200,  40,  40); break;
    case BTN_B2:                                  lcd.setRGB( 40,  40, 200); break;
    case BTN_B3: if (setpointC > 20) setpointC--; lcd.setRGB( 80,  40, 160); break;
    case BTN_B4: if (setpointC < 45) setpointC++; lcd.setRGB( 40, 180,  80); break;
    default: break;
  }
  // Log the raw ADC and its equivalent voltage alongside the classified
  // button so any mis-banding is immediately visible in the serial monitor.
  Serial.print(F("press="));     Serial.print(labelOf(b));
  Serial.print(F(" adc="));      Serial.print(adcAtPress);
  Serial.print(F(" V="));        Serial.print(adcAtPress * ADC_TO_V, 3);
  Serial.print(F(" setpoint=")); Serial.print(setpointC);
  Serial.print(F(" n="));        Serial.println(pressCount);
  renderLCD();
}

// ---- Arduino entry points --------------------------------------------------
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 1500) { /* spin */ }

  Wire.begin();
  lcd.begin(16, 2);
  lcd.setRGB(0, 200, 80);
  lcd.print("W1 button rig");
  lcd.setCursor(0, 1);
  lcd.print("ready.");
  delay(800);
  renderLCD();

  Serial.println(F("W1 ready. Type 'c' for a raw A0 sample (calibration)."));
}

void loop() {
  unsigned long now = millis();

  // Calibration helper: prints the live ADC value on demand without
  // disturbing the debounce loop. Handy when matching the band table to
  // the actual resistors on the bench.
  if (Serial.available() && Serial.read() == 'c') {
    int raw = analogRead(PIN_BUTTONS);
    Serial.print(F("A0 raw = "));
    Serial.print(raw);
    Serial.print(F("  ("));
    Serial.print(raw * ADC_TO_V, 3);
    Serial.println(F(" V)"));
  }

  if (now < nextSampleMs) return;
  nextSampleMs = now + SAMPLE_MS;

  int     adc   = analogRead(PIN_BUTTONS);
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
