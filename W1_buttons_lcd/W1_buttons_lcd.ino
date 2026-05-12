// Reaction-chamber control panel - Analog button matrix + Grove RGB LCD +
// laser ToF sample-height tracking + DHT22 temperature/humidity, on
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
//   Grove Time-of-Flight Distance Sensor (VL53L0X):
//       I2C laser ranging module. Plug the Grove cable into ANY I2C socket
//       on the Base Shield (NOT a digital socket -- the chip is I2C only).
//       Shares the bus with the LCD; address 0x29 vs LCD's 0x3E/0x62.
//       Range ~30..2000 mm, ~30 ms per measurement.
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
#include <OneWire.h>
#include <DallasTemperature.h>
#include <VL53L0X.h>
#include "rgb_lcd.h"

// ---- Pin map ---------------------------------------------------------------
static const uint8_t PIN_BUTTONS = A1;    // analog voltage-divider button matrix
// Grove ToF VL53L0X is I2C-only -- it sits on SDA/SCL via any Grove I2C
// socket on the Base Shield. No dedicated GPIO; default address 0x29.
static const uint8_t PIN_DHT     = 4;     // Grove Temp+Humidity Pro signal (D4)
static const uint8_t PIN_PROBE   = 6;     // DS18B20 1-Wire probe (waterproof, external sample)
static const uint8_t PIN_HEATER  = 8;     // Grove relay driving the heater pad
// PIN_AUDIO is intentionally typed as int, not uint8_t, so it carries A0
// as the variant-specific DAC pin (analogWrite uses the DAC when called
// on A0 at 12-bit resolution -- not a PWM channel).
static const int     PIN_AUDIO   = A0;    // R4 WiFi 12-bit DAC -> Grove Speaker

// ---- Heater control --------------------------------------------------------
// Time-proportional control. Instead of slamming the heater fully on until
// it crosses the setpoint (bang-bang -> big overshoot on a thermally-laggy
// chamber), we run slow PWM on the relay with a duty cycle that scales
// down as we approach target.
//
//   error = setpoint - temp
//
//                duty
//                1.0 |__________
//                    |          \                              .
//                    |           \                             .
//                    |            \                            .
//                0.0 |_____________\___________________ error
//                    0            +HEATER_BAND_C
//                    (over)            (far below)
//
// Snaps at the extremes (<5 % -> off, >95 % -> continuous on) so the relay
// is not chattering uselessly when we are deep inside or far outside the
// band. The HEATER_WINDOW_MS window is the slow-PWM period -- long enough
// that contact wear is negligible (mechanical relays hate kHz switching),
// short enough that the chamber air does not see the pulses.
static const float         HEATER_BAND_C       = 3.0f;     // proportional band size
static const unsigned long HEATER_WINDOW_MS    = 20000UL;  // 20 s slow-PWM period
static const float         HEATER_MIN_DUTY     = 0.05f;    // < 5 %  -> snap to off
static const float         HEATER_MAX_DUTY     = 0.95f;    // > 95 % -> snap to on
static const float         HEATER_MAX_C        = 45.0f;    // hard safety cap
// Integral gain. Ki = 0.002 (1/(C*s)) means an error of 1 C, sustained
// for 1 second, adds 0.2 % to the duty. After ~5 minutes of a 0.6 C
// steady-state offset the integral has accumulated enough to bring the
// duty up to whatever is needed to close the gap exactly. Larger Ki ->
// faster correction but more risk of oscillation.
static const float         HEATER_KI           = 0.002f;
// Cap |Ki*integral| at 1.0 so a long stall cannot drive the controller
// into unrecoverable wind-up regardless of anti-windup logic.
static const float         HEATER_INTEGRAL_MAX = 1.0f / HEATER_KI;
static const float         TARGET_TOL_C        = 0.3f;     // chime trigger band
static bool                heaterOn            = false;
static unsigned long       heaterWindowStartMs = 0;
static float               heaterCommandedDuty = 0.0f;
static unsigned long       nextHeaterMs        = 0;
static float               heaterIntegral      = 0.0f;     // C*s accumulator
static unsigned long       heaterLastTickMs    = 0;
static int                 heaterLastSrcKind   = 0;        // 0=none, 1=probe, 2=dht

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
enum Mode : uint8_t { MODE_TEMP = 0, MODE_HUM = 1, MODE_HEIGHT = 2, MODE_GAME = 3, MODE_DOOM = 4, MODE_N };
static const char* MODE_NAME[MODE_N] = { "TEMP", "HUM ", "HGHT", "GAME", "DOOM" };

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
// "Internal" = DS18B20 stainless-steel probe (the primary control sensor --
//   sits inside / against the sample, so it reads the actual reaction temp).
// "External" = DHT22 reading chamber air around the sample.
static float    currentTempC   = 22.0f;
static float    currentHumPct  = 58.0f;
static bool     dhtValid       = false;
static float    probeTempC     = 22.0f;
static bool     probeValid     = false;
// Non-blocking DS18B20 state machine. Conversion at 12-bit resolution
// takes ~750 ms; doing it synchronously stalls the button sampler and
// makes MODE presses feel sluggish. Instead we kick off a conversion,
// move on, and read the result on the next loop tick after the conversion
// window has elapsed.
enum ProbeState : uint8_t { PROBE_IDLE, PROBE_CONVERTING };
static ProbeState    probeState   = PROBE_IDLE;
static unsigned long probeReadyMs = 0;
static unsigned long nextTofMs    = 0;
static unsigned long nextDhtMs    = 0;
static unsigned long nextProbeMs  = 0;
// Target-reached chime is requested by updateHeater() but fired from the
// main loop, so the 360 ms blocking burst never lands inside onPress
// (which would visibly stall the MODE flip).
static bool          chimePending = false;

// VL53L0X laser ToF reading. -1 means "no valid reading this cycle".
// baselineMm is captured when B3 is pressed; rise = baselineMm - currentTofMm.
static long currentTofMm = -1;
static long baselineMm   = -1;

static Mode     bgMode         = MODE_N;

// ---- Dino runner game state ------------------------------------------------
// Pixel-art "Chrome dino" using four HD44780 custom characters in CGRAM.
// Single button to jump (B4 / "+"). Cacti scroll in from the right and the
// dino has to clear them. Score increments once per game tick (~150 ms).
// State machine is intentionally tiny so it adds no measurable load to the
// PI heater loop or button debounce -- the heater keeps regulating while
// you play.
static const uint8_t GAME_W            = 16;
static const uint8_t GAME_JUMP_FRAMES  = 4;
static const unsigned long GAME_TICK_MS = 150;
enum GameState : uint8_t { GAME_TITLE, GAME_PLAY, GAME_OVER };
static GameState     gameState         = GAME_TITLE;
static uint16_t      gameScore         = 0;
static uint16_t      gameHigh          = 0;
static uint8_t       gameJumpRem       = 0;     // frames of jump remaining
static bool          gameObs[GAME_W]   = {0};   // cacti at each column
static uint8_t       gameSpawnCD       = 6;
static uint8_t       gameRunFrame      = 0;     // 0/1 for running animation
static unsigned long nextGameMs        = 0;

// Custom 5x8 sprites. Each byte = one row; bits 4..0 = leftmost..rightmost.
static uint8_t DINO_A[8] = {
  0b00111, 0b00101, 0b00111, 0b01100,
  0b11111, 0b11111, 0b01010, 0b01010
};
static uint8_t DINO_B[8] = {
  0b00111, 0b00101, 0b00111, 0b01100,
  0b11111, 0b11111, 0b01100, 0b01100
};
static uint8_t DINO_JUMP[8] = {
  0b00111, 0b00101, 0b00111, 0b01100,
  0b11111, 0b11111, 0b01100, 0b00000
};
static uint8_t CACTUS[8] = {
  0b00100, 0b00100, 0b10101, 0b10101,
  0b10101, 0b11111, 0b00100, 0b00100
};
enum : uint8_t {
  CHAR_DINO_A    = 0,
  CHAR_DINO_B    = 1,
  CHAR_DINO_JUMP = 2,
  CHAR_CACTUS    = 3,
  CHAR_GUN       = 4,
  CHAR_GUN_FIRE  = 5,
  CHAR_IMP       = 6,
  CHAR_BULLET    = 7,
};

// ---- Doom 1-D shooter -----------------------------------------------------
// Two-row corridor. Player has a vertical position (top row or bottom row)
// and a gun pointing right. Imps spawn on either row at the right edge
// and walk left. Press B4 to fire a bullet along your current row, press
// B3 to hop to the other row. Imps reaching column 0 of YOUR row hit
// you; imps reaching column 0 of the other row escape harmlessly behind
// you. Real Doom is impossible (RA4M1 has 32 KB of RAM vs the original
// game's ~4 MB working set and there is no pixel display) but the dodge-
// and-shoot loop captures the spirit on a 16x2 grid.
static const uint8_t DOOM_W            = 16;
static const uint8_t DOOM_IMP_DAMAGE   = 20;
static const unsigned long DOOM_TICK_MS = 200;
enum DoomState : uint8_t { DOOM_TITLE, DOOM_PLAY, DOOM_OVER };
static DoomState     doomState     = DOOM_TITLE;
static uint8_t       doomHp        = 100;
static uint16_t      doomKills     = 0;
static bool          doomImp[2][DOOM_W] = {{0}};   // [row][col]
static int8_t        doomBulletCol = -1;           // -1 = no bullet in flight
static uint8_t       doomBulletRow = 1;
static uint8_t       doomPlayerRow = 1;            // 0=top, 1=bottom (start low)
static uint8_t       doomSpawnCD[2] = {5, 8};      // per-row independent spawn
static uint8_t       doomImpStepCD = 2;            // imps step left every 2 ticks
static uint8_t       doomFireFlash = 0;
static uint8_t       doomHurtFlash = 0;            // backlight white-flash on hit
static unsigned long nextDoomMs    = 0;

static uint8_t GUN[8] = {
  0b00000, 0b00000, 0b00100, 0b01111,
  0b11111, 0b01111, 0b00100, 0b00000
};
static uint8_t GUN_FIRE[8] = {
  0b00010, 0b00100, 0b01101, 0b11111,
  0b01101, 0b00100, 0b00010, 0b00000
};
static uint8_t IMP[8] = {
  0b00000, 0b01110, 0b10101, 0b11111,
  0b10101, 0b01110, 0b01010, 0b10001
};
static uint8_t BULLET[8] = {
  0b00000, 0b00000, 0b00000, 0b00111,
  0b00000, 0b00000, 0b00000, 0b00000
};

// ---- LCD -------------------------------------------------------------------
// Declared here (rather than below in a "globals" block) because the CGRAM
// helpers that follow reference `lcd` directly.
rgb_lcd lcd;

// ---- Humidity history graph ------------------------------------------------
// Ring buffer of the last HUM_HIST_LEN samples. Pushed once every
// HUM_SAMPLE_MS regardless of which mode is on screen, so the graph
// keeps building even while the user is playing the games. Rendered in
// MODE_HUM as a 12-column dual-row bar chart, giving 16 vertical pixel
// levels per sample (8 pixels per LCD cell x 2 stacked cells).
// Graph width: 12 LCD cells x 5 pixel columns each = 60 sample slots.
// At one sample every 5 s that's 5 minutes of rolling history.
static const uint8_t HUM_GRAPH_CELLS  = 12;
static const uint8_t HUM_HIST_LEN     = HUM_GRAPH_CELLS * 5;
static const unsigned long HUM_SAMPLE_MS = 5000UL;
static uint8_t       humHistory[HUM_HIST_LEN] = {0}; // % humidity per sample
static uint8_t       humHistoryCount = 0;
static unsigned long nextHumSampleMs = 0;

// CGRAM ownership tracking. The dino and doom sprites together fill
// slots 0..7; the humidity bar chart also wants slots 0..7. Whoever
// most recently displayed sprites is "owner"; we swap on demand when
// the user crosses the boundary between graphics modes.
enum CgramSet : uint8_t { CGRAM_GAMES = 0, CGRAM_BARS = 1 };
static CgramSet currentCgram = CGRAM_GAMES;

// Upload the dino+doom sprite pack to CGRAM slots 0..7.
static void loadGameSprites() {
  lcd.createChar(CHAR_DINO_A,    DINO_A);
  lcd.createChar(CHAR_DINO_B,    DINO_B);
  lcd.createChar(CHAR_DINO_JUMP, DINO_JUMP);
  lcd.createChar(CHAR_CACTUS,    CACTUS);
  lcd.createChar(CHAR_GUN,       GUN);
  lcd.createChar(CHAR_GUN_FIRE,  GUN_FIRE);
  lcd.createChar(CHAR_IMP,       IMP);
  lcd.createChar(CHAR_BULLET,    BULLET);
  currentCgram = CGRAM_GAMES;
}

// Per-cell custom bitmaps built dynamically each render. Five 1-pixel-wide
// bars fit in a single 5-wide cell, so 12 cells x 5 = 60 samples shown.
// The HD44700 only has 8 CGRAM slots, so we dedupe identical cell bitmaps
// before upload; with slowly-changing humidity data adjacent cells usually
// repeat. If we still need more than 8 unique sprites the overflow cells
// render blank (effectively dropped samples on screen for that frame).
static uint8_t humCellSlot[HUM_GRAPH_CELLS * 2];   // top row first, then bottom
static const uint8_t HUM_SLOT_BLANK = 0xFE;        // marker for "render space"
static uint8_t humCellBitmap[HUM_GRAPH_CELLS * 2][8];
static uint8_t humUniqueBitmap[8][8];
static uint8_t humUniqueCount = 0;

static bool humBitmapsEqual(const uint8_t* a, const uint8_t* b) {
  for (uint8_t i = 0; i < 8; i++) if (a[i] != b[i]) return false;
  return true;
}

static bool humBitmapBlank(const uint8_t* a) {
  for (uint8_t i = 0; i < 8; i++) if (a[i] != 0) return false;
  return true;
}

// Build the CGRAM contents and per-cell slot assignment for one frame.
// Heights are expressed in 0..16 pixel-rows (bottom cell takes 0..8,
// top cell shows the overshoot 0..8 above the bottom cell's max).
static void buildHumGraph() {
  // Map each of HUM_HIST_LEN samples to a pixel-column height 0..16,
  // right-justified inside the graph window so partially-filled history
  // appears on the right side (newest) and the left fills in later.
  uint8_t pxHeight[HUM_HIST_LEN];
  for (uint8_t i = 0; i < HUM_HIST_LEN; i++) pxHeight[i] = 0;
  uint8_t startPx = (humHistoryCount >= HUM_HIST_LEN) ? 0 : (HUM_HIST_LEN - humHistoryCount);
  for (uint8_t i = 0; i < humHistoryCount && i < HUM_HIST_LEN; i++) {
    uint8_t pct = humHistory[i];
    if (pct > 100) pct = 100;
    pxHeight[startPx + i] = (uint16_t)pct * 16 / 100;
  }

  // Build a cell bitmap for each of the 24 cells (12 top + 12 bottom).
  for (uint8_t row = 0; row < 2; row++) {
    for (uint8_t col = 0; col < HUM_GRAPH_CELLS; col++) {
      uint8_t cellIdx = row * HUM_GRAPH_CELLS + col;
      uint8_t h[5];
      for (uint8_t c = 0; c < 5; c++) {
        uint8_t full = pxHeight[col * 5 + c];
        if (row == 1) {                    // bottom cell: 0..8 pixels filled
          h[c] = full > 8 ? 8 : full;
        } else {                            // top cell: overshoot above 8
          h[c] = full > 8 ? (full - 8) : 0;
        }
      }
      // For each pixel row r in the cell (0=top of cell, 7=bottom), set
      // the bit for each pixel column whose bar reaches that row.
      for (uint8_t r = 0; r < 8; r++) {
        uint8_t b = 0;
        for (uint8_t c = 0; c < 5; c++) {
          if (h[c] > 0 && r >= (uint8_t)(8 - h[c])) {
            b |= (uint8_t)(1 << (4 - c));   // bit 4 = leftmost
          }
        }
        humCellBitmap[cellIdx][r] = b;
      }
    }
  }

  // Dedupe. Blank cells render as a plain space (no CGRAM slot needed)
  // which frees slots for non-trivial bitmaps.
  humUniqueCount = 0;
  for (uint8_t i = 0; i < HUM_GRAPH_CELLS * 2; i++) {
    if (humBitmapBlank(humCellBitmap[i])) {
      humCellSlot[i] = HUM_SLOT_BLANK;
      continue;
    }
    int8_t found = -1;
    for (uint8_t j = 0; j < humUniqueCount; j++) {
      if (humBitmapsEqual(humCellBitmap[i], humUniqueBitmap[j])) { found = (int8_t)j; break; }
    }
    if (found >= 0) {
      humCellSlot[i] = (uint8_t)found;
    } else if (humUniqueCount < 8) {
      for (uint8_t k = 0; k < 8; k++) humUniqueBitmap[humUniqueCount][k] = humCellBitmap[i][k];
      humCellSlot[i] = humUniqueCount;
      humUniqueCount++;
    } else {
      humCellSlot[i] = HUM_SLOT_BLANK;     // overflow -> drop this cell
    }
  }

  // Push the unique bitmaps into CGRAM. ~1 ms per slot over I2C so up to
  // ~8 ms per frame -- comfortably below the 5 s sample interval.
  for (uint8_t j = 0; j < humUniqueCount; j++) {
    lcd.createChar(j, humUniqueBitmap[j]);
  }
  currentCgram = CGRAM_BARS;
}

// Legacy entry point kept for symmetry with useGamesCgram(). Building
// the graph from current data also primes CGRAM, so nothing else to do.
static void loadBarSprites() { buildHumGraph(); }

// Mode-specific CGRAM loaders. Split into separate helpers (rather than a
// dispatcher that takes a Mode argument) so the Arduino IDE's automatic
// prototype generator does not need to know the Mode enum before its
// definition appears further down the file.
static void useBarsCgram()  { if (currentCgram != CGRAM_BARS)  loadBarSprites();  }
static void useGamesCgram() { if (currentCgram != CGRAM_GAMES) loadGameSprites(); }

// ---- DHT22 -----------------------------------------------------------------
DHT dht(PIN_DHT, DHT22);

// ---- Grove Time-of-Flight distance sensor (VL53L0X) ------------------------
// I2C laser ToF, range ~30..2000 mm, ~30 ms per measurement in default mode.
// Sits on the same I2C bus as the LCD (0x29 default for VL53L0X, no collision
// with the LCD's 0x3E / 0x62). Returns 65535 / sets timeoutOccurred() on
// out-of-range or no-return.
VL53L0X tof;
static bool tofValid = false;

// ---- DS18B20 external probe ------------------------------------------------
// 1-Wire stainless-steel probe on D6 via converter board (which carries the
// usual 4.7 kohm pull-up from data to VCC). Resolution defaults to 12 bits
// (~0.0625 C, ~750 ms conversion). We poll at ~1 Hz so the conversion has
// always finished before the next request.
OneWire           probeBus(PIN_PROBE);
DallasTemperature probe(&probeBus);

// ---- Target-reached beep ---------------------------------------------------
// Latches true once the controlling temperature crosses into the setpoint
// deadband from below, fires a short chime, and stays latched until the
// temp wanders back out (or STOP / setpoint change resets it). One beep
// per arrival -- no nagging.
static bool targetReached = false;

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
  else if (displayMode == MODE_GAME)   lcd.setRGB(255, 220,  60);  // retro yellow
  else                                 lcd.setRGB(180,  20,  20);  // hell-red for DOOM
}

// ---- Dino runner ----------------------------------------------------------
static void gameReset() {
  gameState     = GAME_TITLE;
  gameScore     = 0;
  gameJumpRem   = 0;
  gameSpawnCD   = 6;
  gameRunFrame  = 0;
  for (uint8_t i = 0; i < GAME_W; i++) gameObs[i] = false;
  nextGameMs    = millis() + GAME_TICK_MS;
}

static void gameJump() {
  // B4 doubles as "start" on the title screen and "retry" on the over
  // screen, so the user only ever needs one button to play.
  if (gameState == GAME_TITLE) { gameState = GAME_PLAY; return; }
  if (gameState == GAME_OVER)  { gameReset(); gameState = GAME_PLAY; return; }
  if (gameJumpRem == 0) gameJumpRem = GAME_JUMP_FRAMES;
}

// One tick of the world: shift cacti left, maybe spawn a new one at the
// right edge, advance jump/run animation, check for a collision.
static void gameTick() {
  if (gameState != GAME_PLAY) return;
  for (uint8_t i = 0; i < GAME_W - 1; i++) gameObs[i] = gameObs[i + 1];
  gameObs[GAME_W - 1] = false;
  if (gameSpawnCD == 0) {
    gameObs[GAME_W - 1] = true;
    gameSpawnCD = 4 + (uint8_t)random(8);   // 4..11 ticks until next spawn
  } else {
    gameSpawnCD--;
  }
  if (gameJumpRem > 0) gameJumpRem--;
  // Collision: dino lives at column 0; if a cactus is there and we are
  // not airborne, it's over.
  if (gameObs[0] && gameJumpRem == 0) {
    gameState = GAME_OVER;
    if (gameScore > gameHigh) gameHigh = gameScore;
    Serial.print(F("[game] over score=")); Serial.print(gameScore);
    Serial.print(F(" high="));             Serial.println(gameHigh);
  } else {
    gameScore++;
    gameRunFrame ^= 1;
  }
}

static void renderGame() {
  useGamesCgram();
  applyBacklight();
  uint8_t r0[GAME_W], r1[GAME_W];
  for (uint8_t i = 0; i < GAME_W; i++) { r0[i] = ' '; r1[i] = ' '; }

  if (gameState == GAME_TITLE) {
    // Title: name on top, controls on bottom. A static dino sprite sits
    // in the gameplay position so the player previews where they will
    // be standing.
    const char* t = "DINO RUNNER";
    for (uint8_t i = 0; t[i] && i < GAME_W; i++) r0[i + 2] = (uint8_t)t[i];
    r1[0] = CHAR_DINO_A;
    const char* h = " B4=jump start";
    for (uint8_t i = 0; h[i] && (i + 1) < GAME_W; i++) r1[i + 1] = (uint8_t)h[i];
  } else if (gameState == GAME_OVER) {
    const char* msg = "GAME OVER";
    for (uint8_t i = 0; msg[i] && i < GAME_W; i++) r0[i] = (uint8_t)msg[i];
    char buf[17];
    snprintf(buf, sizeof(buf), "S:%4u  HI:%4u", gameScore, gameHigh);
    for (uint8_t i = 0; buf[i] && i < GAME_W; i++) r1[i] = (uint8_t)buf[i];
  } else {
    // Dino: row 1 col 0 normally (alternating frames); row 0 col 0 while
    // airborne. The square it vacates becomes empty ground.
    if (gameJumpRem > 0) r0[0] = CHAR_DINO_JUMP;
    else                 r1[0] = gameRunFrame ? CHAR_DINO_B : CHAR_DINO_A;
    for (uint8_t i = 1; i < GAME_W; i++) {
      if (gameObs[i]) r1[i] = CHAR_CACTUS;
    }
    char sbuf[8];
    snprintf(sbuf, sizeof(sbuf), "S:%04u", gameScore);
    for (uint8_t k = 0; k < 6; k++) r0[10 + k] = (uint8_t)sbuf[k];
  }

  lcd.setCursor(0, 0);
  for (uint8_t i = 0; i < GAME_W; i++) lcd.write(r0[i]);
  lcd.setCursor(0, 1);
  for (uint8_t i = 0; i < GAME_W; i++) lcd.write(r1[i]);
}

// ---- Doom 1-D shooter -----------------------------------------------------
static void doomReset() {
  doomState     = DOOM_TITLE;
  doomHp        = 100;
  doomKills     = 0;
  doomBulletCol = -1;
  doomBulletRow = 1;
  doomPlayerRow = 1;
  doomSpawnCD[0] = 5;
  doomSpawnCD[1] = 8;
  doomImpStepCD = 2;
  doomFireFlash = 0;
  doomHurtFlash = 0;
  for (uint8_t r = 0; r < 2; r++)
    for (uint8_t i = 0; i < DOOM_W; i++) doomImp[r][i] = false;
  nextDoomMs    = millis() + DOOM_TICK_MS;
}

static void doomFire() {
  if (doomState == DOOM_TITLE) { doomState = DOOM_PLAY; return; }
  if (doomState == DOOM_OVER)  { doomReset(); doomState = DOOM_PLAY; return; }
  if (doomBulletCol >= 0) return;                  // one bullet in flight
  doomBulletCol = 1;                               // muzzle at col 0; bullet starts at col 1
  doomBulletRow = doomPlayerRow;                   // bullet inherits player's row
  doomFireFlash = 1;
}

static void doomSwapRow() {
  if (doomState == DOOM_OVER)  { doomReset(); doomState = DOOM_PLAY; return; }
  if (doomState != DOOM_PLAY)  return;
  doomPlayerRow ^= 1;
}

// Resolve a bullet/imp overlap. Called after bullet move and again after
// imp move so fast crossings still register on the bullet's row.
static void doomCheckHit() {
  if (doomBulletCol < 0) return;
  if (doomBulletCol >= (int8_t)DOOM_W) { doomBulletCol = -1; return; }
  if (doomImp[doomBulletRow][doomBulletCol]) {
    doomImp[doomBulletRow][doomBulletCol] = false;
    doomBulletCol = -1;
    doomKills++;
  }
}

static void doomTick() {
  if (doomState != DOOM_PLAY) return;

  if (doomFireFlash > 0) doomFireFlash--;
  if (doomHurtFlash > 0) doomHurtFlash--;

  // Bullet step (right) and collision check.
  if (doomBulletCol >= 0) { doomBulletCol++; doomCheckHit(); }

  // Imps step (left) every doomImpStepCD ticks. Both rows step together.
  if (--doomImpStepCD == 0) {
    doomImpStepCD = 2;
    for (uint8_t r = 0; r < 2; r++) {
      // Any imp reaching col 0 hits the player, regardless of row -- you
      // have to shoot every imp before it gets there. Dodging only buys
      // you time to line up the shot, not free escape.
      if (doomImp[r][0]) {
        doomImp[r][0] = false;
        if (doomHp >= DOOM_IMP_DAMAGE) doomHp -= DOOM_IMP_DAMAGE; else doomHp = 0;
        doomHurtFlash = 2;
      }
      for (uint8_t i = 0; i < DOOM_W - 1; i++) doomImp[r][i] = doomImp[r][i + 1];
      doomImp[r][DOOM_W - 1] = false;
    }
    doomCheckHit();
  }

  // Per-row spawn. Slightly slower than the single-row version because
  // density doubles when both rows are firing simultaneously.
  uint8_t base = 8;
  if (doomKills > 20) base = 6;
  if (doomKills > 40) base = 4;
  for (uint8_t r = 0; r < 2; r++) {
    if (doomSpawnCD[r] == 0) {
      doomImp[r][DOOM_W - 1] = true;
      doomSpawnCD[r] = base + (uint8_t)random(5);
    } else {
      doomSpawnCD[r]--;
    }
  }

  if (doomHp == 0) doomState = DOOM_OVER;
}

static void renderDoom() {
  useGamesCgram();
  // Backlight: doom red normally, white flash when the player takes a
  // hit (visual feedback now that HP is no longer drawn on a HUD row).
  if (doomState == DOOM_PLAY && doomHurtFlash > 0) {
    lcd.setRGB(255, 255, 255);
    bgMode = MODE_N;             // force the red to be re-applied next frame
  } else {
    applyBacklight();
  }

  uint8_t r0[DOOM_W], r1[DOOM_W];
  for (uint8_t i = 0; i < DOOM_W; i++) { r0[i] = ' '; r1[i] = ' '; }

  if (doomState == DOOM_TITLE) {
    const char* t = "  D O O M  ";
    for (uint8_t i = 0; t[i] && i < DOOM_W; i++) r0[i + 2] = (uint8_t)t[i];
    const char* h = "B4=fire B3=move";
    for (uint8_t i = 0; h[i] && i < DOOM_W; i++) r1[i] = (uint8_t)h[i];
  } else if (doomState == DOOM_OVER) {
    const char* t = "YOU DIED";
    for (uint8_t i = 0; t[i] && i < DOOM_W; i++) r0[i] = (uint8_t)t[i];
    char b[17];
    snprintf(b, sizeof(b), "K:%-3u  B4 retry", doomKills);
    for (uint8_t i = 0; b[i] && i < DOOM_W; i++) r1[i] = (uint8_t)b[i];
  } else {
    // Both rows are corridor. Gun lives on the player's row at col 0.
    // The opposite row's col 0 carries an HP indicator (a single digit
    // 0..8 for HP/12) so the player has some HUD without losing a row.
    uint8_t* gunRow = (doomPlayerRow == 0) ? r0 : r1;
    uint8_t* otherRow = (doomPlayerRow == 0) ? r1 : r0;
    gunRow[0]   = doomFireFlash > 0 ? CHAR_GUN_FIRE : CHAR_GUN;
    uint8_t hpDigit = doomHp / 12;            // 100/12=8, 0/12=0
    if (hpDigit > 9) hpDigit = 9;
    otherRow[0] = (uint8_t)('0' + hpDigit);

    // Imps + bullet on their respective rows.
    for (uint8_t i = 1; i < DOOM_W; i++) {
      if (doomImp[0][i]) r0[i] = CHAR_IMP;
      if (doomImp[1][i]) r1[i] = CHAR_IMP;
    }
    if (doomBulletCol >= 1 && doomBulletCol < (int8_t)DOOM_W) {
      if (doomBulletRow == 0) r0[doomBulletCol] = CHAR_BULLET;
      else                    r1[doomBulletCol] = CHAR_BULLET;
    }
  }

  lcd.setCursor(0, 0);
  for (uint8_t i = 0; i < DOOM_W; i++) lcd.write(r0[i]);
  lcd.setCursor(0, 1);
  for (uint8_t i = 0; i < DOOM_W; i++) lcd.write(r1[i]);
}

static void renderLCD() {
  // The mini-games own their rendering -- routing through them means
  // sensor ticks during play still update the screen consistently.
  if (displayMode == MODE_GAME) { renderGame(); return; }
  if (displayMode == MODE_DOOM) { renderDoom(); return; }
  applyBacklight();
  char l1[17], l2[17];
  if (displayMode == MODE_TEMP) {
    // Side-by-side internal (DHT22 chamber air) vs external (DS18B20 probe
    // touching the sample) temperature. Heater indicator '*' on the far
    // right of line 1 -- relay closed = pad drawing current.
    //   "IN 25.4  EX 24.1"  (line 1, 16 cols incl. heater marker)
    //   "Set 32 C    [+/-]" (line 2 -- setpoint still drives the heater)
    char hot = heaterOn ? '*' : ' ';
    char in[6], ex[6];
    if (probeValid) {
      int p10 = (int)lroundf(probeTempC * 10.0f);
      snprintf(in, sizeof(in), "%2d.%1d", p10 / 10, abs(p10) % 10);
    } else {
      snprintf(in, sizeof(in), "--.-");
    }
    if (dhtValid) {
      int t10 = (int)lroundf(currentTempC * 10.0f);
      snprintf(ex, sizeof(ex), "%2d.%1d", t10 / 10, abs(t10) % 10);
    } else {
      snprintf(ex, sizeof(ex), "--.-");
    }
    snprintf(l1, sizeof(l1), "IN %4s EX %4s%c", in, ex, hot);
    snprintf(l2, sizeof(l2), "Set %2d C   [+/-]", setpointC);
  } else if (displayMode == MODE_HUM) {
    // 12 cells of dense graph + 4 cells of numeric readout on the right.
    // Each LCD cell packs 5 one-pixel-wide bars, so the graph carries 60
    // samples worth of history (5 minutes at HUM_SAMPLE_MS = 5 s). The
    // graph builder uploads dynamic sprites to CGRAM each frame.
    buildHumGraph();
    // Bar cells.
    lcd.setCursor(0, 0);
    for (uint8_t c = 0; c < HUM_GRAPH_CELLS; c++) {
      uint8_t s = humCellSlot[0 * HUM_GRAPH_CELLS + c];
      lcd.write(s == HUM_SLOT_BLANK ? ' ' : s);
    }
    // Numeric overlay on cols 12..15: " NN%" current value.
    char buf[5];
    if (dhtValid) {
      int hPct = (int)lroundf(currentHumPct);
      if (hPct < 0)  hPct = 0;
      if (hPct > 99) hPct = 99;
      snprintf(buf, sizeof(buf), " %2d%%", hPct);
    } else {
      buf[0] = ' '; buf[1] = '-'; buf[2] = '-'; buf[3] = '%'; buf[4] = 0;
    }
    for (uint8_t i = 0; i < 4; i++) lcd.write(buf[i]);
    // Bottom row: bars + " HUM" label.
    lcd.setCursor(0, 1);
    for (uint8_t c = 0; c < HUM_GRAPH_CELLS; c++) {
      uint8_t s = humCellSlot[1 * HUM_GRAPH_CELLS + c];
      lcd.write(s == HUM_SLOT_BLANK ? ' ' : s);
    }
    lcd.write(' '); lcd.write('H'); lcd.write('U'); lcd.write('M');
    return;
  } else {  // MODE_HEIGHT
    if (currentTofMm < 0) {
      snprintf(l1, sizeof(l1), "DIST   -- mm    ");
      snprintf(l2, sizeof(l2), "no return       ");
    } else if (baselineMm < 0) {
      snprintf(l1, sizeof(l1), "DIST  %4ld mm   ", currentTofMm);
      snprintf(l2, sizeof(l2), "B3 to tare      ");
    } else {
      long rise = baselineMm - currentTofMm;
      snprintf(l1, sizeof(l1), "DIST  %4ld mm   ", currentTofMm);
      snprintf(l2, sizeof(l2), "rise %+5ld mm  ", rise);
    }
  }
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

// Grove VL53L0X ToF read. readRangeSingleMillimeters() runs one ranging
// cycle (~30 ms blocking by default) and returns the distance in mm.
// 8190+ or the timeoutOccurred() flag means out-of-range / no return; we
// surface that as -1 to the rest of the sketch.
static long readTofMm() {
  if (!tofValid) return -1;
  uint16_t mm = tof.readRangeSingleMillimeters();
  if (tof.timeoutOccurred() || mm == 65535 || mm >= 8190) return -1;
  return (long)mm;
}

// ---- Audio annunciator -----------------------------------------------------
// One-shot chime when the chamber reaches its setpoint. Three short rising
// tones, full-rail DAC swing, ~360 ms total. Total blocking is small
// enough not to disturb the bang-bang loop or button debounce, and only
// fires once per arrival -- there is no per-mode-change audio at all so
// flipping pages is instantaneous.
static void targetBeep() {
  static const uint16_t tones[3] = { 880, 1175, 1568 };   // A5, D6, G6
  for (uint8_t t = 0; t < 3; t++) {
    unsigned halfPeriodUs = 500000UL / tones[t];
    unsigned long endMs = millis() + 100;
    bool high = false;
    while ((long)(millis() - endMs) < 0) {
      high = !high;
      analogWrite(PIN_AUDIO, high ? 4095 : 0);
      delayMicroseconds(halfPeriodUs);
    }
    analogWrite(PIN_AUDIO, 2048);
    delay(20);
  }
}

// Wipe the integral state. Called on STOP, on any setpoint change, and
// whenever the control source (probe/dht) switches -- the accumulated
// error history would otherwise be mis-applied to a new operating point.
static void resetHeaterIntegral() {
  heaterIntegral   = 0.0f;
  heaterLastTickMs = 0;
}

// PI heater controller with time-proportional output and integral
// anti-windup. P = error / HEATER_BAND_C; I accumulates error*dt but only
// while the controller is in its linear (unsaturated) region, so the long
// 100 %-duty ramp from cold start does not pile up integral mass that
// would later overshoot. Safety overrides force duty to zero. Output is
// run through slow PWM on the relay with a HEATER_WINDOW_MS period.
static void updateHeater() {
  // Prefer the probe (internal, sample-side) as the control signal. Fall
  // back to the DHT22 (chamber air) if the probe is missing -- avoids the
  // heater going dark just because the cable is unplugged.
  bool        haveTemp = probeValid || dhtValid;
  float       tempC    = probeValid ? probeTempC : currentTempC;
  const char* src      = probeValid ? "probe"    : "dht";
  int         srcKind  = probeValid ? 1 : (dhtValid ? 2 : 0);

  // Source change (probe <-> dht <-> none) invalidates the accumulator.
  if (srcKind != heaterLastSrcKind) {
    resetHeaterIntegral();
    heaterLastSrcKind = srcKind;
  }

  unsigned long now = millis();

  // PI evaluation in the unsaturated region. Time step from the last
  // update; first tick after a reset gets dt=0 so we do not absorb a
  // bogus dt across the reset boundary.
  float dt = 0.0f;
  if (heaterLastTickMs != 0) {
    dt = (float)(now - heaterLastTickMs) / 1000.0f;
    if (dt < 0.0f || dt > 5.0f) dt = 0.0f;  // skip on weird gaps
  }
  heaterLastTickMs = now;

  float duty = 0.0f;
  if (haveTemp && tempC < HEATER_MAX_C) {
    float error = (float)setpointC - tempC;
    float pTerm = error / HEATER_BAND_C;
    float iTerm = heaterIntegral * HEATER_KI;
    float raw   = pTerm + iTerm;
    // Anti-windup: only integrate when integration would not push the
    // output further into saturation. Outside this window the integral
    // freezes (a.k.a. conditional integration).
    bool windingUp   = (raw >= 1.0f && error > 0.0f);
    bool windingDown = (raw <= 0.0f && error < 0.0f);
    if (!windingUp && !windingDown && dt > 0.0f) {
      heaterIntegral += error * dt;
      // Hard clamp |Ki*integral| <= 1.0 as a belt-and-braces guard.
      if (heaterIntegral >  HEATER_INTEGRAL_MAX) heaterIntegral =  HEATER_INTEGRAL_MAX;
      if (heaterIntegral < -HEATER_INTEGRAL_MAX) heaterIntegral = -HEATER_INTEGRAL_MAX;
    }
    duty = pTerm + heaterIntegral * HEATER_KI;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
  } else {
    // Safety stop -- also bleed the integral so we resume with a clean
    // slate the moment the fault clears.
    heaterIntegral = 0.0f;
  }
  // Snap dead zones at both ends so the relay is not pulsed pointlessly.
  if (duty < HEATER_MIN_DUTY) duty = 0.0f;
  if (duty > HEATER_MAX_DUTY) duty = 1.0f;
  // Start a fresh slow-PWM window every HEATER_WINDOW_MS. The commanded
  // duty is latched at the start of the window -- it does not chase the
  // temperature mid-window, which would cause the heater to chatter near
  // the band edge.
  if (now - heaterWindowStartMs >= HEATER_WINDOW_MS) {
    heaterWindowStartMs = now;
    heaterCommandedDuty = duty;
    if (haveTemp) {
      float iPct = heaterIntegral * HEATER_KI * 100.0f;
      Serial.print(F("[heater] window duty="));
      Serial.print(heaterCommandedDuty * 100.0f, 0);
      Serial.print(F("%  I="));
      Serial.print(iPct, 0);
      Serial.print(F("%  (src="));
      Serial.print(src);
      Serial.print(F(", temp="));
      Serial.print(tempC, 1);
      Serial.print(F(" C, set="));
      Serial.print(setpointC);
      Serial.println(F(" C)"));
    }
  }
  // Within the window: relay ON for the first duty*window milliseconds,
  // OFF for the rest. duty == 0 -> always OFF; duty == 1 -> always ON.
  unsigned long elapsed = now - heaterWindowStartMs;
  unsigned long onMs    = (unsigned long)(heaterCommandedDuty * HEATER_WINDOW_MS);
  bool want = (elapsed < onMs);

  if (want != heaterOn) {
    heaterOn = want;
    digitalWrite(PIN_HEATER, heaterOn ? HIGH : LOW);
  }

  // Target-reached chime: triggers once when the control temp first
  // crosses inside +/-TARGET_TOL_C of the setpoint. Reset latch when we
  // drift back outside, so a long session that wanders chimes again.
  if (haveTemp) {
    float absErr = fabsf((float)setpointC - tempC);
    if (absErr <= TARGET_TOL_C && !targetReached) {
      targetReached = true;
      Serial.print(F("[target] reached at "));
      Serial.print(tempC, 1);
      Serial.println(F(" C"));
      chimePending = true;
    } else if (absErr > HEATER_BAND_C) {
      targetReached = false;
    }
  }
}

// Two-rate sensor sampler. ToF runs at 1 Hz (~30 ms blocking per read),
// so we keep it cheap); the DHT22 runs at ~0.4 Hz because its datasheet
// requires >=2 s between reads. NaN returns from the DHT mean the sensor
// missed a frame -- common, harmless, just hold the last good value.
// Returns true if any reading changed (caller decides to repaint).
static bool stepSensors(unsigned long now) {
  bool changed = false;

  if (now >= nextTofMs) {
    nextTofMs = now + 1000;
    currentTofMm = readTofMm();
    changed = true;
  }

  // Non-blocking DS18B20: kick off a conversion, come back ~800 ms later
  // to read the result, then wait until the next 1 Hz slot.
  if (probeState == PROBE_IDLE && now >= nextProbeMs) {
    probe.requestTemperatures();
    probeReadyMs = now + 800;          // 12-bit conversion is ~750 ms
    probeState   = PROBE_CONVERTING;
  } else if (probeState == PROBE_CONVERTING && now >= probeReadyMs) {
    float p = probe.getTempCByIndex(0);
    if (p > -50.0f && p < 125.0f) {
      probeTempC = p;
      if (!probeValid) {
        probeValid = true;
        Serial.println(F("[probe] first valid reading"));
      }
      changed = true;
    } else if (probeValid) {
      probeValid = false;
      Serial.println(F("[probe] disconnected"));
      changed = true;
    }
    probeState  = PROBE_IDLE;
    nextProbeMs = now + 1000;
    updateHeater();
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
    case BTN_B1: {
      // Reset setpoint to the current measured temperature so the heater
      // stops chasing whatever the previous target was. Prefer the probe
      // (internal); fall back to DHT22 (chamber air); if neither has a
      // valid reading yet, hold the existing setpoint untouched.
      float refC;
      bool  haveRef = true;
      if      (probeValid) refC = probeTempC;
      else if (dhtValid)   refC = currentTempC;
      else                 haveRef = false;
      if (haveRef) {
        int s = (int)lroundf(refC);
        if (s < 20) s = 20;
        if (s > 45) s = 45;
        setpointC = s;
      }
      displayMode    = MODE_TEMP;
      baselineMm     = -1;
      targetReached  = false;
      resetHeaterIntegral();
      break;
    }
    case BTN_B2:
      displayMode = (Mode)((displayMode + 1) % MODE_N);
      if      (displayMode == MODE_GAME) gameReset();
      else if (displayMode == MODE_DOOM) doomReset();
      break;
    case BTN_B3:
      if (displayMode == MODE_TEMP) {
        if (setpointC > 20) {
          setpointC--;
          targetReached = false;
          resetHeaterIntegral();
        }
      } else if (displayMode == MODE_HEIGHT) {
        long mm = readTofMm();
        if (mm >= 0) { baselineMm = mm; currentTofMm = mm; }
      } else if (displayMode == MODE_GAME) {
        gameReset();
      } else if (displayMode == MODE_DOOM) {
        doomSwapRow();
      } else {
        ignored = true;
      }
      break;
    case BTN_B4:
      if (displayMode == MODE_TEMP) {
        if (setpointC < 45) {
          setpointC++;
          targetReached = false;
          resetHeaterIntegral();
        }
      } else if (displayMode == MODE_GAME) {
        gameJump();
      } else if (displayMode == MODE_DOOM) {
        doomFire();
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
  // operator does not wait up to 2.5 s for the next sensor tick.
  updateHeater();
  renderLCD();
}

// ---- Arduino entry points --------------------------------------------------
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 1500) { /* spin */ }

  // Heater relay defaults OFF -- if the firmware crashes between here and
  // updateHeater(), the chamber simply does not heat.
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);

  // Audio path: 12-bit resolution selects the DAC on A0 (the Renesas core
  // routes >=12-bit analogWrite on this pin through DAC12 instead of GPT
  // PWM). Park the DAC at midpoint so the speaker amplifier is quiescent.
  analogWriteResolution(12);
  analogWrite(PIN_AUDIO, 2048);

  dht.begin();
  probe.begin();
  probe.setWaitForConversion(false);   // non-blocking -- we time it ourselves

  // Seed the RNG used by the dino game. A2 is unconnected, so its ADC
  // floats on coupling noise -- entropy enough for cactus spawn timing.
  randomSeed((unsigned long)analogRead(A2) ^ micros());

  Wire.begin();

  // VL53L0X ToF distance sensor. init() returns false if the chip does not
  // ACK on the I2C bus -- usually means the Grove cable is in a digital
  // socket instead of an I2C socket, or the wiring is loose. We tolerate
  // a failed init (tofValid stays false; HEIGHT mode just shows "--").
  tof.setTimeout(500);
  if (tof.init()) {
    tof.setMeasurementTimingBudget(33000);    // 33 ms = balance speed/accuracy
    tofValid = true;
    Serial.println(F("[tof] VL53L0X ready"));
  } else {
    Serial.println(F("[tof] VL53L0X init failed -- check I2C wiring"));
  }

  lcd.begin(16, 2);
  // Upload dino-runner sprites into CGRAM slots 0..3. These persist until
  // power-off, so the game just writes the slot index (0..3) wherever it
  // wants a sprite to appear.
  lcd.createChar(CHAR_DINO_A,    DINO_A);
  lcd.createChar(CHAR_DINO_B,    DINO_B);
  lcd.createChar(CHAR_DINO_JUMP, DINO_JUMP);
  lcd.createChar(CHAR_CACTUS,    CACTUS);
  lcd.createChar(CHAR_GUN,       GUN);
  lcd.createChar(CHAR_GUN_FIRE,  GUN_FIRE);
  lcd.createChar(CHAR_IMP,       IMP);
  lcd.createChar(CHAR_BULLET,    BULLET);
  lcd.setRGB(0, 200, 80);
  lcd.print("isotherm-r4");
  lcd.setCursor(0, 1);
  lcd.print("warming up...");
  delay(800);
  renderLCD();

  Serial.println(F("isotherm-r4 panel ready."));
  Serial.println(F("  STOP  : reset to defaults"));
  Serial.println(F("  MODE  : cycle TEMP -> HUMIDITY -> HEIGHT -> GAME -> DOOM"));
  Serial.println(F("  + / - : TEMP   -> adjust setpoint"));
  Serial.println(F("          HEIGHT -> B3 re-tares baseline"));
  Serial.println(F("          GAME   -> B4 jump,  B3 restart"));
  Serial.println(F("          DOOM   -> B4 fire,  B3 swap rows"));
  Serial.println(F("  c     : one-shot raw A0 read"));
  Serial.println(F("  v     : toggle continuous A0 stream (every 200 ms)"));
  Serial.println(F("  p     : single ToF range measurement with diagnostics"));
  Serial.println(F("  d     : toggle continuous distance stream (every 500 ms)"));
  Serial.println(F("  b     : fire the target-reached chime now (test)"));
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
    } else if (ch == 'b') {
      Serial.println(F("[beep] test chime"));
      targetBeep();
    } else if (ch == 'p') {
      // ToF single-shot diagnostic: prints raw mm, timeout flag, init state.
      unsigned long t0 = micros();
      long mm = readTofMm();
      unsigned long elapsed = micros() - t0;
      Serial.print(F("tof: init="));        Serial.print(tofValid ? F("OK") : F("FAIL"));
      Serial.print(F(" timeout="));         Serial.print(tof.timeoutOccurred() ? F("Y") : F("N"));
      Serial.print(F(" elapsed_us="));      Serial.print(elapsed);
      if (mm < 0) Serial.println(F("  -> OUT OF RANGE / NO RETURN"));
      else { Serial.print(F("  mm=")); Serial.println(mm); }
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

  // Push a humidity sample into the rolling history once per minute.
  // First sample lands as soon as the DHT22 produces its first valid
  // reading (nextHumSampleMs starts at 0).
  if (dhtValid && now >= nextHumSampleMs) {
    nextHumSampleMs = now + HUM_SAMPLE_MS;
    int p = (int)lroundf(currentHumPct);
    if (p < 0) p = 0; if (p > 100) p = 100;
    if (humHistoryCount < HUM_HIST_LEN) {
      humHistory[humHistoryCount++] = (uint8_t)p;
    } else {
      for (uint8_t i = 0; i < HUM_HIST_LEN - 1; i++) humHistory[i] = humHistory[i + 1];
      humHistory[HUM_HIST_LEN - 1] = (uint8_t)p;
    }
    if (displayMode == MODE_HUM) renderLCD();
  }

  // Dino game tick. Runs only while MODE_GAME is on screen; heater and
  // sensors continue regulating in parallel. Each tick advances the
  // world by one column and redraws the whole 16x2 frame.
  if (displayMode == MODE_GAME && gameState == GAME_PLAY && now >= nextGameMs) {
    nextGameMs = now + GAME_TICK_MS;
    gameTick();
    renderGame();
  }

  // Doom mini-shooter tick. Only ticks while the player has dismissed
  // the title screen (state == DOOM_PLAY) -- otherwise the title and
  // game-over screens hold still until B4/B3 is pressed.
  if (displayMode == MODE_DOOM && doomState == DOOM_PLAY && now >= nextDoomMs) {
    nextDoomMs = now + DOOM_TICK_MS;
    doomTick();
    renderDoom();
  }

  // Tick the heater at 4 Hz so the slow-PWM window resolution is ~250 ms.
  // updateHeater() is cheap; the slow-PWM logic just compares elapsed vs
  // window*duty and flips the relay if needed.
  if (now >= nextHeaterMs) {
    nextHeaterMs = now + 250;
    bool prev = heaterOn;
    updateHeater();
    if (prev != heaterOn) renderLCD();   // refresh the '*' indicator
  }

  // Target-reached chime: only fire while no button is held, so the 360 ms
  // blocking tone never lands on top of a press or release transition.
  if (chimePending && confirmed == BTN_NONE) {
    chimePending = false;
    targetBeep();
  }

  if (distStream && now >= nextDistMs) {
    nextDistMs = now + 500;
    long mm = readTofMm();
    Serial.print(F("[dist] mm="));
    if (mm < 0) Serial.println(F("-- (no return)"));
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
