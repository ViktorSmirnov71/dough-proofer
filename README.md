# Dough Proofer

A self-contained reaction-chamber controller for monitoring and steering
yeast fermentation during dough proofing. An Arduino UNO R4 WiFi
orchestrates a small sensor stack — air temperature and humidity from a
DHT22, dough rise from an ultrasonic ranger — and exposes a three-page
panel on a 16x2 RGB LCD driven from a custom analog button matrix.

The project explores how far a single low-cost microcontroller can be
pushed for closed-loop control of a slow biological process, with an
emphasis on careful analog electronics, robust software debouncing, and
sensor fusion at the embedded level.

## Why dough proofing

Yeast dough proofing is a slow, autocatalytic reaction. Carbon dioxide
production by *Saccharomyces cerevisiae* both raises the dough volume and
feeds back into the metabolic rate via local pH and substrate
availability. The reaction is exquisitely sensitive to temperature
(Q10 ≈ 2-3) and humidity (skin formation on the surface arrests rise),
making it an ideal candidate for actively controlled instrumentation
rather than open-loop kitchen timing.

## Hardware

| Module | Bus | Notes |
|---|---|---|
| Arduino UNO R4 WiFi | host | Renesas RA4M1, 5 V I/O, 10-bit ADC default |
| Seeed Grove Base Shield | passthrough | voltage selector set to 5 V |
| 4-button voltage-divider matrix | **A0** | single ADC pin, four discrete codes |
| Grove RGB LCD 2x16 | I2C | Wire library, addresses 0x3E + 0x62 |
| Grove Ultrasonic Distance Sensor v2 | **D2** | single-wire bidirectional protocol |
| Grove Temperature + Humidity Pro v1.3 (DHT22) | **D4** | proprietary 1-wire, 0.5 Hz max |
| Grove SPDT relay + silicone heater pad | **D8** | active-HIGH; bang-bang temperature loop |

Active components consume ~80 mA at 5 V from the USB-C connection; the
heater itself runs from its own supply through the relay's isolated
contacts.

## Analog button matrix

Four push-buttons share one ADC pin via a switched voltage divider:

```
   5V --[ 1 kohm ]--+-- A0
                    |
                    +--[ SW1 ]------------- GND     (short,    V ~ 0.00 V)
                    +--[ 180 ohm ]--[ SW2 ]-- GND   (          V ~ 0.76 V)
                    +--[ 240 ohm ]--[ SW3 ]-- GND   (          V ~ 0.97 V)
                    +--[ 330 ohm ]--[ SW4 ]-- GND   (          V ~ 1.24 V)
```

With nothing pressed, A0 is pulled up to 5 V through the 1 kohm. Pressing
a button drops A0 to a leg-specific voltage and the firmware classifies
the resulting ADC code into one of four bands. The 10-bit ADC at
V_ref = 5 V resolves the bands cleanly — minimum gap between adjacent codes
is ~40 counts, well above the noise floor.

Net cost: one analog pin for four buttons (instead of four digital pins),
which frees the digital side for the relay, additional sensors, and the
ultrasonic ranger. See `W1_buttons_lcd/schematic.svg` for the hand-drawn
circuit.

## Firmware

`W1_buttons_lcd/W1_buttons_lcd.ino` is a single sketch (~400 lines) that:

- **reads four buttons** off A0 with 5 ms x 6-sample software debounce
  (~30 ms effective) and edge-only press detection — holding a button
  never auto-repeats,
- **drives the LCD** as a three-page panel with mode-driven backlight
  colour (warm amber for TEMP, cool cyan for HUM, fresh green for HEIGHT),
- **reads the DHT22** at 0.4 Hz (datasheet minimum), tolerates the
  occasional NaN return by silently holding the previous good value,
- **pulses the ultrasonic sensor** at 1 Hz, converts the echo flight time
  to millimetres, and reports dough rise from an auto-tared baseline,
- **streams a structured log** at 115200 baud — every press includes the
  raw ADC code and equivalent voltage, every sensor read failure is
  flagged, and on-demand diagnostic commands (`c`, `v`, `p`, `d`) dump
  calibration data without disturbing the rest of the loop.

Memory footprint with the Arduino UNO R4 (Renesas) core: ~25 % flash
(64 KB / 256 KB) and ~28 % SRAM (9 KB / 32 KB), leaving headroom for the
smoothing and forecast code planned next.

## Build and flash

Tested with **Arduino IDE 2.3.8** and **arduino-cli 1.4.1** on macOS,
targeting `arduino:renesas_uno:unor4wifi` (core 1.5.3).

Required libraries:

| Library | Source |
|---|---|
| Grove - LCD RGB Backlight | Seeed Studio |
| DHT sensor library | Adafruit |
| Adafruit Unified Sensor | Adafruit |

```sh
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi W1_buttons_lcd
arduino-cli upload  --fqbn arduino:renesas_uno:unor4wifi \
                    -p /dev/cu.usbmodem... W1_buttons_lcd
arduino-cli monitor -p /dev/cu.usbmodem... -c baudrate=115200
```

Day-to-day operation, serial commands, and a 60-second health check live
in [`W1_buttons_lcd/MANUAL.md`](W1_buttons_lcd/MANUAL.md). The breadboard
build, sensor cabling, and a troubleshooting table are in
[`W1_buttons_lcd/WIRING.md`](W1_buttons_lcd/WIRING.md).

## Roadmap

- **Moving-average smoothing** on the ultrasonic height signal. Raw
  ultrasonic readings have +/-3 mm jitter, which is large enough to bury
  the slow dough-rise gradient (typically 5-10 mm / minute at peak rise);
  a short trailing window collapses the jitter without compromising
  responsiveness.
- **Linear least-squares fit** on the height time series. With the dough
  rise approximated as `H(t) = a*t` over its near-linear middle band, a
  running fit of the slope `a` lets us forecast time-to-target without
  waiting for the dough to actually reach it.
- **3D-printed insulated enclosure** mounting the sensors at fixed
  positions above the proofing jar, isolating the chamber thermally from
  the surrounding air.

## Repository layout

```
.
+- README.md                     this file
+- .gitignore
+- W1_buttons_lcd/
   +- W1_buttons_lcd.ino         the sketch
   +- WIRING.md                  bench build + troubleshooting
   +- MANUAL.md                  operation manual
   +- schematic.svg              hand-drawn circuit
   +- schematic.png              rasterised at 1600 px
```
