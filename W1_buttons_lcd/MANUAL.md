# Operation manual

Day-to-day cheat sheet for running the dough-proofer firmware
(`W1_buttons_lcd.ino`). For the electrical build see
[`WIRING.md`](WIRING.md) and [`schematic.svg`](schematic.svg).

## 1. Compile and upload

You can use either the GUI or the command line. Pick one.

### GUI route — Arduino IDE 2

1. Open the sketch: it should already be loaded; if not, `File > Open` and
   pick `W1_buttons_lcd/W1_buttons_lcd.ino`.
2. Plug the UNO R4 WiFi into the Mac with a USB-C cable.
3. **Tools > Board > Arduino UNO R4 Boards > Arduino UNO R4 WiFi**.
4. **Tools > Port > /dev/cu.usbmodem...** (the entry appears only when the
   board is connected).
5. Click the right-arrow **Upload** icon. The status bar reports
   "Done uploading" when finished.

### CLI route — arduino-cli

From this folder:

```
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi W1_buttons_lcd
arduino-cli board list                                            # find the port
arduino-cli upload  --fqbn arduino:renesas_uno:unor4wifi -p /dev/cu.usbmodem... W1_buttons_lcd
arduino-cli monitor -p /dev/cu.usbmodem... -c baudrate=115200
```

The first command also catches syntax errors without touching the board, so
it is the fastest way to validate a change.

## 2. Open the Serial Monitor

In the IDE, click the magnifying-glass icon in the top right, **set the baud
rate to 115200**, and press the on-board reset button once. You should see:

```
W1 ready. Type 'c' for a raw A0 sample (calibration).
```

At the same time the LCD prints:

```
W1 button rig
ready.
```

for ~0.8 seconds, then switches to the live display.

## 3. What each button does

| Button | Resistor leg | LCD label | Backlight  | Action                          |
|--------|--------------|-----------|------------|---------------------------------|
| **B1** | direct short | `[STOP]`  | red        | reset setpoint to 32 C          |
| **B2** | 180 ohm      | `[MODE]`  | blue       | placeholder (mode cycle hook)   |
| **B3** | 240 ohm      | `[ - ]`   | purple     | decrement setpoint by 1 C (min 20) |
| **B4** | 330 ohm      | `[ + ]`   | green      | increment setpoint by 1 C (max 45) |

Only the press *edge* fires an event. Holding a button down does **not**
auto-repeat. To change the setpoint by 5 C you tap B4 five times.

## 4. Reading the LCD

```
+----------------+
|Setpoint: 32 C  |   <- line 1: current target temperature
|[ +  ] n=7      |   <- line 2: last button + cumulative press count
+----------------+
```

The press counter `n` increments on every accepted press regardless of which
button. Useful for confirming the debounce is working: one physical tap
should add exactly **1** to `n`, never 2 or 3.

## 5. Reading the Serial Monitor

Every accepted press also prints a one-line summary, e.g.:

```
press= +   setpoint=33 n=4
press=STOP setpoint=32 n=5
press=MODE setpoint=32 n=6
```

This is the easiest place to confirm the band table is mapping ADC codes to
the correct button.

## 6. Calibration: `c` command

If a press lands on the wrong button (e.g. B3 keeps registering as B4), hold
the button down and type `c` followed by Enter in the Serial Monitor. The
sketch prints:

```
A0 raw = 213
```

That number is the live ADC reading for the held button. Repeat for every
button and write the four values down. Then in `W1_buttons_lcd.ino` update
the `BANDS[]` table near the top so each `lo`..`hi` window centres on the
measured value with a comfortable margin (about +/-30 counts is plenty).
Re-upload.

Typical re-tuned bands after one calibration pass:

```c++
{ BTN_B1,    0,   raw_b1 + 30 },
{ BTN_B2, raw_b2 - 30, raw_b2 + 25 },
{ BTN_B3, raw_b3 - 25, raw_b3 + 25 },
{ BTN_B4, raw_b4 - 25, raw_b4 + 80 },   // wider upper margin
{ BTN_NONE, 900, 1023 }
```

## 7. Quick health check (60 seconds)

1. Open Serial Monitor. Confirm the `W1 ready.` banner.
2. Press each button once in order B1 -> B2 -> B3 -> B4. The LCD backlight
   should change colour each time and `n` should advance by exactly 1 per
   tap.
3. Release every button. The LCD should hold the last state — pressing
   nothing for several seconds must not generate any spurious events.
4. Hold B4 down for 3 seconds. `n` must increase by exactly 1, not climb
   continuously.
5. Type `c` + Enter while no button is pressed. Expect a printed `A0 raw`
   value above 900 (close to 1023).

If all five pass, the rig is ready for the closed-loop control work
(relay + heater, signal smoothing, dough-rise forecast).

## 8. Common runtime issues

| Symptom | Fix |
|---|---|
| Serial Monitor empty after upload | Baud rate not 115200, or you opened the monitor on the wrong port. |
| `n` jumps by 2-3 on one tap | Increase `STABLE_SAMPLES` in the sketch from 6 to 8 or 10. |
| First press after upload is missed | The 1.5-second `Serial` wait in `setup()` is still running — press again. |
| Press fires but no LCD change | LCD Grove cable not in an I2C port, or library not installed. |
| Setpoint never crosses 32 C downward | Reset (B1) was tapped after every decrement — check you are pressing the correct leg. |
