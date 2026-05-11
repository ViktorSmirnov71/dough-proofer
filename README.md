# Dough Proofer — Emerging Technologies Group Project

Imperial College Year-3 *Emerging Technologies* coursework. The brief is to
build a heated chamber that proves yeasted dough (case study: crumpets),
gathering temperature, humidity, and dough-rise data and controlling the
chamber accordingly.

This repository covers the **Workshop 1** deliverable — the electronics and
firmware that turn an Arduino UNO R4 WiFi + a Grove sensor stack into a
working control panel. The 3D-printed enclosure (Workshop 2) and the
closed-loop heater control + dough-rise forecast (Workshop 3) build on top
of this base.

## Hardware

| Component | Where it lives | Notes |
|---|---|---|
| Arduino UNO R4 WiFi | base board | 5 V I/O, 10-bit ADC, USB-C |
| Seeed Grove Base Shield | stacked | switch left on **5 V** |
| Grove RGB LCD 2x16 | I²C socket | uses the `Wire` bus (`A4`/`A5`) |
| Push-button array | breadboard via Grove cable on **A0** | 4 buttons in a switched voltage divider |
| Grove Ultrasonic Distance Sensor v2 | Grove socket **D2** | single-wire signal |
| Grove Temperature + Humidity Sensor Pro v1.3 (DHT22) | Grove socket **D4** | 2 s minimum read interval |

The button divider is described in `W1_buttons_lcd/WIRING.md` and drawn in
`W1_buttons_lcd/schematic.svg`. Other sensors are Grove modules — one cable
per sensor, no breadboard work.

## Firmware

`W1_buttons_lcd/W1_buttons_lcd.ino` is a single sketch that:

- reads four buttons off a single analog pin using a voltage-divider trick
  (slide-deck math, software debounce, edge-only press events),
- drives a Grove RGB LCD as a three-page proofer panel:
  - **TEMP** — live air temperature + editable setpoint,
  - **HUM** — live humidity (read-only),
  - **HEIGHT** — dough rise in mm, auto-tared on entry,
- streams a structured log on `Serial` at 115200 baud for debugging.

Button layout: **STOP** (reset) · **MODE** (cycle pages) · **−** / **+**
(adjust setpoint on TEMP, re-tare on HEIGHT).

## Build

Requires Arduino IDE 2 or `arduino-cli` with the `arduino:renesas_uno` core
and these libraries:

- `Grove - LCD RGB Backlight` (Seeed)
- `DHT sensor library` (Adafruit) + `Adafruit Unified Sensor`

```sh
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi W1_buttons_lcd
arduino-cli upload  --fqbn arduino:renesas_uno:unor4wifi -p /dev/cu.usbmodem... W1_buttons_lcd
```

See `W1_buttons_lcd/MANUAL.md` for serial-monitor usage, calibration, and
the diagnostic commands (`c`, `v`, `p`, `d`).
