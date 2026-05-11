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
//
// UI semantics:
//   B1 STOP : reset state to defaults (setpoint 32 C, TEMP mode, sim restart)
//   B2 MODE : cycle display between TEMP and HUMIDITY pages
//   B3  -   : decrement setpoint by 1 C  (only in TEMP mode, min 20 C)
//   B4  +   : increment setpoint by 1 C  (only in TEMP mode, max 45 C)
//
// Live readings are stubbed (marked with * on the LCD). Real sensors will
// replace stepSimulation() in the next iteration.

#include <Wire.h>
#include <math.h>
#include "rgb_lcd.h"

// ---- Pin map ---------------------------------------------------------------
static const uint8_t PIN_BUTTONS = A0;

// ---- ADC -> voltage conversion ---------------------------------------------
// Slide convention: 5 V / 1024 = 4.88 mV per ADC count (V_ref = 5 V, 10-bit).
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
enum Mode : uint8_t { MODE_TEMP = 0, MODE_HUM = 1, MODE_N };
static const char* MODE_NAME[MODE_N] = { "TEMP", "HUM " };

static int      setpointC      = 32;
static Mode     displayMode    = MODE_TEMP;
static uint32_t pressCount     = 0;
static Button   lastPressed    = BTN_NONE;
static int      lastAdc        = 0;
static bool     verbose        = false;
static unsigned long nextVerboseMs = 0;

// Stubbed chamber readings -- replaced by real sensor reads in a later step.
static float    currentTempC   = 22.0f;
static float    currentHumPct  = 58.0f;
static unsigned long nextSimMs = 0;

// Track the last mode actually pushed to the LCD so we only re-issue the
// (relatively slow) setRGB call when it actually changes.
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

// Backlight reflects the active page so a glance tells you which mode you
// are in even without reading the text. Only re-issue setRGB on change to
// avoid hammering the LCD over I2C every render.
static void applyBacklight() {
  if (displayMode == bgMode) return;
  bgMode = displayMode;
  if (displayMode == MODE_TEMP) lcd.setRGB(200, 110,  20);   // warm amber
  else                          lcd.setRGB( 20, 150, 200);   // cool cyan
}

static void renderLCD() {
  applyBacklight();
  char l1[17], l2[17];
  if (displayMode == MODE_TEMP) {
    int t10 = (int)lroundf(currentTempC * 10.0f);
    snprintf(l1, sizeof(l1), "TEMP*  %2d.%1d C   ", t10 / 10, abs(t10) % 10);
    snprintf(l2, sizeof(l2), "Set %2d C   [+/-]", setpointC);
  } else {  // MODE_HUM
    int h10 = (int)lroundf(currentHumPct * 10.0f);
    snprintf(l1, sizeof(l1), "HUM*   %2d.%1d %%   ", h10 / 10, abs(h10) % 10);
    snprintf(l2, sizeof(l2), "display only    ");
  }
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

// Simulated proofer dynamics. Temperature relaxes first-order toward the
// setpoint (a heater bringing the chamber up); humidity wobbles +/-3 %
// around a 58 % baseline on a 30 s period -- deterministic so the demo is
// reproducible. The real sensors will replace this body.
static void stepSimulation(unsigned long now) {
  currentTempC  += (setpointC - currentTempC) * 0.05f;
  currentHumPct  = 58.0f + 3.0f * sinf(now / 30000.0f * 6.2831853f);
}

static void onPress(Button b, int adcAtPress) {
  pressCount++;
  lastPressed = b;
  bool ignored = false;
  switch (b) {
    case BTN_B1:  // STOP / RESET: snap all state back to defaults.
      setpointC      = 32;
      displayMode    = MODE_TEMP;
      currentTempC   = 22.0f;
      break;
    case BTN_B2:  // MODE: cycle through the display pages.
      displayMode = (Mode)((displayMode + 1) % MODE_N);
      break;
    case BTN_B3:
      if (displayMode == MODE_TEMP && setpointC > 20) setpointC--;
      else if (displayMode != MODE_TEMP)              ignored = true;
      break;
    case BTN_B4:
      if (displayMode == MODE_TEMP && setpointC < 45) setpointC++;
      else if (displayMode != MODE_TEMP)              ignored = true;
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
  Serial.println(F("  MODE  : cycle TEMP <-> HUMIDITY page"));
  Serial.println(F("  + / - : adjust temperature setpoint (TEMP page only)"));
  Serial.println(F("  c     : one-shot raw A0 read"));
  Serial.println(F("  v     : toggle continuous A0 stream (every 200 ms)"));
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
    }
  }

  // Continuous live stream when verbose mode is on. Each line also runs
  // the same classifier the press-edge logic uses, so holding a button
  // continuously reports thinks=MODE etc. without going silent.
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

  // Tick the simulated chamber dynamics once per second and refresh the LCD
  // so the displayed temperature drifts toward the setpoint and the humidity
  // gently wobbles -- gives the panel a "live" feel before real sensors land.
  if (now >= nextSimMs) {
    nextSimMs = now + 1000;
    stepSimulation(now);
    renderLCD();
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
