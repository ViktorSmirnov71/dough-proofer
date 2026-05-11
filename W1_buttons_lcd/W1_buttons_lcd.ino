// Reaction-chamber control panel - Analog button matrix + Grove RGB LCD +
// ultrasonic dough-height tracking + DHT22 temperature/humidity, on
// Arduino UNO R4 WiFi.
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
//       Base Shield. That's the whole hookup -- the cable carries VCC, GND,
//       and the bidirectional signal line. Only the primary signal (yellow,
//       = D2) is used; the secondary line (white, = D3) is unconnected on
//       this module. The sensor handles its own logic-level conversion so
//       the shield's 3.3V/5V switch position is not critical.
//
//   Grove Temperature & Humidity Sensor Pro v1.3 (DHT22 / AM2302):
//       Plug a Grove cable from the sensor's socket to the D4 socket on the
//       Base Shield. Single-wire 1-wire-style protocol on D4 (yellow); the
//       D5 secondary line is unused. Datasheet limits read rate to ~0.5 Hz,
//       so we sample every 2.5 s and cache the last good values.
//
//   Grove SPDT Relay (active-HIGH) driving the silicone heater pad:
//       Signal pin on D8. Drive HIGH to energise the coil and close the
//       contacts (heater receives mains/PSU power); drive LOW to break.
//       The relay's coil is isolated from the Arduino's 5 V rail by the
//       module's onboard optocoupler/transistor stage.
//
//   Grove Speaker on A0 (12-bit DAC output):
//       The UNO R4 WiFi has a real analogue DAC on pin A0. Driving the
//       LM386 amplifier on the Grove Speaker with the DAC gives clean
//       analogue audio with no PWM carrier artefacts. We use TalkiePCM
//       (Linear Predictive Coding speech synthesis, ~250-word vocabulary)
//       to generate 8 kHz speech samples and stream them straight into
//       the DAC. The same path is also used for system beeps -- the DAC
//       drives a square wave whose amplitude is the software volume.
//
// Design notes:
//   - One ADC pin for N buttons frees digital pins for the heater relay
//     and additional sensors.
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
#include <DHT.h>
#include <TalkiePCM.h>
#include "rgb_lcd.h"

// ---- Pin map ---------------------------------------------------------------
static const uint8_t PIN_BUTTONS = A1;    // analog voltage-divider button matrix
static const uint8_t PIN_SONAR   = 2;     // Grove Ultrasonic v2 single-wire signal
static const uint8_t PIN_DHT     = 4;     // Grove Temp+Humidity Pro signal (D4)
static const uint8_t PIN_HEATER  = 8;     // Grove relay driving the heater pad
// PIN_AUDIO is intentionally typed as int, not uint8_t, so it carries A0
// as the variant-specific DAC pin (analogWrite uses the DAC when called
// on A0 at 12-bit resolution -- not a PWM channel).
static const int     PIN_AUDIO   = A0;    // R4 WiFi 12-bit DAC -> Grove Speaker

// ---- Heater control --------------------------------------------------------
// Bang-bang controller with a 0.5 C deadband around the setpoint -- wide
// enough that the relay does not chatter on sensor noise, narrow enough
// that the chamber holds within +/-1 C of target on a slow thermal mass
// like a proofing jar.
static const float HEATER_DEADBAND_C = 0.5f;
// Hard upper safety cap. Crumpet dough proofs around 28-32 C; anything
// over this is either a sensor fault or a runaway and the relay opens
// regardless of the setpoint.
static const float HEATER_MAX_C      = 45.0f;
static bool        heaterOn          = false;

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
enum Mode : uint8_t { MODE_TEMP = 0, MODE_HUM = 1, MODE_HEIGHT = 2, MODE_VOLUME = 3, MODE_N };
static const char* MODE_NAME[MODE_N] = { "TEMP", "HUM ", "HGHT", "VOL " };

// Software speaker level. 0 = mute (beep() skips, but still consumes the
// nominal beep duration so speakNumber's cadence stays predictable).
// 1 = soft (shorter, lower-pitch tones), 2 = normal, 3 = emphatic
// (slightly higher-pitch tones). Real analogue loudness comes from the
// potentiometer on the speaker module itself; this is the digital mute /
// trim on top of it.
static const uint8_t VOLUME_MAX = 3;
static uint8_t       volumeLevel = 2;

static int      setpointC      = 32;
static Mode     displayMode    = MODE_TEMP;
static uint32_t pressCount     = 0;
static Button   lastPressed    = BTN_NONE;
static int      lastAdc        = 0;
static bool     verbose        = false;
static bool     distStream     = false;
static unsigned long nextVerboseMs = 0;
static unsigned long nextDistMs    = 0;

// Live readings from the DHT22. Pre-seeded with sensible defaults so the
// LCD has something to show before the first successful read lands, but a
// 'valid' flag tracks whether real numbers have arrived yet.
static float    currentTempC   = 22.0f;
static float    currentHumPct  = 58.0f;
static bool     dhtValid       = false;
static unsigned long nextSonarMs = 0;
static unsigned long nextDhtMs   = 0;

// Distance / dough-height state. currentDistanceMm is the latest raw read
// (-1 = no echo / sensor not present). baselineMm is the "empty / starting"
// reference set when entering HEIGHT mode or re-tared with B3 -- dough rise
// is then baselineMm - currentDistanceMm.
static long currentDistanceMm = -1;
static long baselineMm        = -1;

static Mode     bgMode         = MODE_N;

// ---- LCD -------------------------------------------------------------------
rgb_lcd lcd;

// ---- DHT22 -----------------------------------------------------------------
DHT dht(PIN_DHT, DHT22);

// ---- Speech synthesis (TalkiePCM LPC vocoder) ------------------------------
// TalkiePCM produces 8 kHz int16 samples and calls our callback with them.
// We map each sample to the DAC's 0..4095 range and clock them out at the
// requested rate via a busy-wait. The Grove Speaker's LM386 then turns
// the analogue voltage into sound. Quality is robotic but intelligible --
// the same vocoder the 1980s Speak-and-Spell used.
TalkiePCM voice;

static void dacCallback(int16_t* data, int len) {
  for (int i = 0; i < len; i++) {
    int32_t v = ((int32_t)data[i] + 32768) >> 4;  // -32768..32767 -> 0..4095
    if (v < 0)    v = 0;
    if (v > 4095) v = 4095;
    analogWrite(PIN_AUDIO, (uint16_t)v);
    delayMicroseconds(125);                        // ~8 kHz cadence
  }
}

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
  else if (displayMode == MODE_HEIGHT) lcd.setRGB( 40, 200, 100);  // fresh green
  else                                 lcd.setRGB(180,  60, 200);  // violet for volume
}

static void renderLCD() {
  applyBacklight();
  char l1[17], l2[17];
  if (displayMode == MODE_TEMP) {
    // Show a small heater indicator on line 1 -- one * means the relay is
    // currently closed and the heater pad is drawing current.
    char hot = heaterOn ? '*' : ' ';
    if (dhtValid) {
      int t10 = (int)lroundf(currentTempC * 10.0f);
      snprintf(l1, sizeof(l1), "TEMP   %2d.%1d C %c ", t10 / 10, abs(t10) % 10, hot);
    } else {
      snprintf(l1, sizeof(l1), "TEMP   --.- C %c ", hot);
    }
    snprintf(l2, sizeof(l2), "Set %2d C   [+/-]", setpointC);
  } else if (displayMode == MODE_HUM) {
    if (dhtValid) {
      int h10 = (int)lroundf(currentHumPct * 10.0f);
      snprintf(l1, sizeof(l1), "HUM    %2d.%1d %%   ", h10 / 10, abs(h10) % 10);
    } else {
      snprintf(l1, sizeof(l1), "HUM    --.- %%   ");
    }
    snprintf(l2, sizeof(l2), "display only    ");
  } else if (displayMode == MODE_HEIGHT) {
    if (currentDistanceMm < 0) {
      snprintf(l1, sizeof(l1), "DIST   -- mm    ");
      snprintf(l2, sizeof(l2), "no echo signal  ");
    } else if (baselineMm < 0) {
      // Not tared yet: just show the live distance and prompt the user.
      snprintf(l1, sizeof(l1), "DIST  %4ld mm   ", currentDistanceMm);
      snprintf(l2, sizeof(l2), "B3 to tare      ");
    } else {
      // Tared: show raw distance up top, rise from baseline below.
      long rise = baselineMm - currentDistanceMm;
      snprintf(l1, sizeof(l1), "DIST  %4ld mm   ", currentDistanceMm);
      snprintf(l2, sizeof(l2), "rise %+5ld mm  ", rise);
    }
  } else {  // MODE_VOLUME
    snprintf(l1, sizeof(l1), "VOLUME    %u/%u   ", volumeLevel, VOLUME_MAX);
    if (volumeLevel == 0) snprintf(l2, sizeof(l2), "MUTED -- + ups  ");
    else                   snprintf(l2, sizeof(l2), "use +/- to tune ");
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
  return (long)durUs * 343L / 2000L;
}

// ---- Audio annunciator -----------------------------------------------------
// System beep generated by driving the DAC as a square wave around the
// midpoint. The amplitude swing IS the volume here -- because we have a
// real DAC, software volume genuinely controls loudness (unlike the
// previous tone()-based implementation which could only mute or pass).
//   0 = silent (DAC parked at midpoint, no swing)
//   1 = soft   (narrow swing around midpoint)
//   2 = normal
//   3 = emphatic (full-rail swing)
static void beep(unsigned freq, unsigned durMs) {
  if (volumeLevel == 0) { delay(durMs + 50); return; }
  uint16_t hi = 2048 + 1600, lo = 2048 - 1600;          // ~78% swing
  if (volumeLevel == 1) { hi = 2048 +  400; lo = 2048 -  400; }   // ~20% swing
  if (volumeLevel == 3) { hi = 4095;         lo =    0; }         // full rail
  unsigned halfPeriodUs = 500000UL / freq;
  unsigned long endMs = millis() + durMs;
  bool high = false;
  while ((long)(millis() - endMs) < 0) {
    high = !high;
    analogWrite(PIN_AUDIO, high ? hi : lo);
    delayMicroseconds(halfPeriodUs);
  }
  analogWrite(PIN_AUDIO, 2048);                          // park midpoint
  delay(50);
}

// Speak whatever metric the current LCD page is showing -- real spoken
// words now, via the LPC vocoder. TalkiePCM.sayNumber() handles
// "twenty five" / "fifty eight" expansion automatically. Distance is
// converted from mm to cm so the spoken value sits in a sensible range
// for a proofing chamber. Sensor faults produce a single low growl
// instead of attempting to say nonsense.
static void speakCurrentPage() {
  if (volumeLevel == 0) return;            // muted -> say nothing at all
  if (displayMode == MODE_TEMP) {
    if (!dhtValid) { beep(120, 500); return; }
    voice.sayNumber((long)lroundf(currentTempC));
    voice.silence(120);
    voice.say(sp2_DEGREES);
  } else if (displayMode == MODE_HUM) {
    if (!dhtValid) { beep(120, 500); return; }
    voice.sayNumber((long)lroundf(currentHumPct));
    voice.silence(120);
    voice.say(sp2_PERCENT);
  } else if (displayMode == MODE_HEIGHT) {
    if (currentDistanceMm < 0) { beep(120, 500); return; }
    long cm = currentDistanceMm / 10;
    if (cm > 999) cm = 999;
    voice.sayNumber(cm);
  } else {                                  // MODE_VOLUME -- speak the level
    voice.sayNumber(volumeLevel);
  }
  analogWrite(PIN_AUDIO, 2048);             // park DAC so we leave silent
}

// Bang-bang heater controller with deadband. Forces the relay open if the
// DHT22 has not produced a valid reading yet (so we never heat blind) or
// if the chamber temp has gone above the safety cap (sensor fault / heater
// runaway). The deadband stops the relay clicking on every 0.1 C wobble.
// Logs every state change so the serial trail tells you exactly when the
// heater turned on/off and which temperature triggered it.
static void updateHeater() {
  bool want = heaterOn;  // default: hold current state inside the deadband
  if (!dhtValid)                                        want = false;
  else if (currentTempC >= HEATER_MAX_C)                want = false;
  else if (currentTempC <  setpointC - HEATER_DEADBAND_C) want = true;
  else if (currentTempC >  setpointC + HEATER_DEADBAND_C) want = false;

  if (want != heaterOn) {
    heaterOn = want;
    digitalWrite(PIN_HEATER, heaterOn ? HIGH : LOW);
    Serial.print(F("[heater] "));
    Serial.print(heaterOn ? F("ON ") : F("OFF"));
    Serial.print(F(" (current="));
    Serial.print(currentTempC, 1);
    Serial.print(F(" C, set="));
    Serial.print(setpointC);
    Serial.println(F(" C)"));
  }
}

// Two-rate sensor sampler. Sonar runs at 1 Hz (pulseIn blocks up to 30 ms
// so we keep it cheap); the DHT22 runs at ~0.4 Hz because its datasheet
// requires >=2 s between reads. NaN returns from the DHT mean the sensor
// missed a frame -- common, harmless, just hold the last good value.
// Returns true if any reading changed (caller decides to repaint).
static bool stepSensors(unsigned long now) {
  bool changed = false;

  if (now >= nextSonarMs) {
    nextSonarMs = now + 1000;
    currentDistanceMm = readDistanceMm();
    changed = true;
  }

  if (now >= nextDhtMs) {
    nextDhtMs = now + 2500;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      currentTempC  = t;
      currentHumPct = h;
      if (!dhtValid) {
        dhtValid = true;
        Serial.println(F("[dht] first valid reading"));
      }
      changed = true;
    } else {
      Serial.println(F("[dht] read failed (NaN) -- holding previous values"));
    }
    // Re-evaluate the heater every time we have fresh chamber temperature.
    updateHeater();
  }

  return changed;
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
      // No auto-tare on entry -- the page shows raw distance by default
      // and the user explicitly presses B3 when they want to set the zero.
      // Announce the new page's current value over the speaker. This is
      // deferred until after the LCD repaint at the end of onPress() so
      // the user sees the page change before the audio starts.
      break;
    case BTN_B3:
      if (displayMode == MODE_TEMP) {
        if (setpointC > 20) setpointC--;
      } else if (displayMode == MODE_HEIGHT) {
        long mm = readDistanceMm();
        if (mm >= 0) { baselineMm = mm; currentDistanceMm = mm; }
      } else if (displayMode == MODE_VOLUME) {
        if (volumeLevel > 0) volumeLevel--;
      } else {
        ignored = true;
      }
      break;
    case BTN_B4:
      if (displayMode == MODE_TEMP) {
        if (setpointC < 45) setpointC++;
      } else if (displayMode == MODE_VOLUME) {
        if (volumeLevel < VOLUME_MAX) volumeLevel++;
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
  // Setpoint may have moved -- re-evaluate the heater immediately so the
  // operator does not wait up to 2.5 s for the next DHT tick.
  updateHeater();
  renderLCD();
  // After the visual feedback, play audio feedback. MODE press announces
  // the value of the page just landed on. +/- press in VOLUME mode plays
  // a sample beep so the user can hear the new level immediately.
  // Deliberately last in onPress so the speaker block does not delay the
  // LCD refresh.
  if (b == BTN_B2) {
    speakCurrentPage();
  } else if (displayMode == MODE_VOLUME && (b == BTN_B3 || b == BTN_B4)) {
    beep(800, 120);
  }
}

// ---- Arduino entry points --------------------------------------------------
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 1500) { /* spin */ }

  pinMode(PIN_SONAR, INPUT);

  // Heater relay defaults OFF -- if the firmware crashes between here and
  // updateHeater(), the chamber simply does not heat.
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);

  // Audio path: 12-bit resolution selects the DAC on A0 (the Renesas core
  // routes >=12-bit analogWrite on this pin through DAC12 instead of GPT
  // PWM). Park the DAC at midpoint so the speaker amplifier is quiescent.
  analogWriteResolution(12);
  analogWrite(PIN_AUDIO, 2048);
  voice.setDataCallback(dacCallback);

  dht.begin();

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
  Serial.println(F("  MODE  : cycle TEMP -> HUMIDITY -> HEIGHT -> VOLUME"));
  Serial.println(F("  + / - : TEMP   -> adjust setpoint"));
  Serial.println(F("          HEIGHT -> B3 re-tares baseline"));
  Serial.println(F("          VOLUME -> adjust speaker level (0..3)"));
  Serial.println(F("  c     : one-shot raw A0 read"));
  Serial.println(F("  v     : toggle continuous A0 stream (every 200 ms)"));
  Serial.println(F("  p     : single ultrasonic ping with full diagnostics"));
  Serial.println(F("  d     : toggle continuous distance stream (every 500 ms)"));
  Serial.println(F("  s     : speak the current page value over the speaker"));
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
    } else if (ch == 's') {
      Serial.print(F("[speak] page=")); Serial.println(MODE_NAME[displayMode]);
      speakCurrentPage();
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

  // Drive the real sensors. Each one has its own cadence inside stepSensors().
  if (stepSensors(now)) renderLCD();

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
