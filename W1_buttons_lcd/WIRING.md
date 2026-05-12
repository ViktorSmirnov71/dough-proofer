# Wiring guide

Hardware target: Arduino UNO R4 WiFi + Seeed Grove Base Shield + Grove RGB LCD 2x16 + 4 push-buttons on a mini breadboard.

## 0. Power off before wiring

Do not have the UNO plugged into USB while you push wires in. Plug it in *after* the circuit is built and you have double-checked it.

## 1. Stack the Grove Base Shield onto the UNO R4

- Line up the headers and push down evenly. Do not flex it.
- Slide the on-shield voltage selector to **5V** (this is the default; verify the tiny switch near the SCL/SDA labels). The LCD and the 1k divider both expect 5V.

## 2. Connect the Grove RGB LCD

- Use a 4-pin Grove cable from the LCD to **any port marked I2C** on the Base Shield. There are usually four I2C ports — they are wired in parallel, pick whichever is most convenient.
- Cable orientation only goes in one way; do not force it.

## 3. Connect the breadboard for the button divider

The slides show a "mini breadboard with Grove connector" — that is a small breadboard whose top row exposes the four wires of a Grove cable as power/ground/signal rails. Two options:

### Option A (matches the lecture, recommended)

- Grove cable from **port A0** on the Base Shield -> Grove socket on the mini breadboard.
- The mini breadboard's rails now carry: GND, VCC (5V), A0 signal, A1 signal. Only the first three are needed here.

### Option B (no mini breadboard)

- Use a normal breadboard with jumper wires direct to the UNO R4 header: a wire from `5V`, a wire from `GND`, a wire from `A0`.

Either way, downstream of this point the wiring is identical.

## 4. Build the voltage divider

Schematic (re-stating the comment block at the top of the .ino):

```
   5V ----[ 1 kohm ]----+----> A0
                        |
                        +---[ SW1 ]----------------- GND      (short, ADC ~ 0)
                        +---[ 180 ohm ]---[ SW2 ]--- GND      (ADC ~ 156)
                        +---[ 240 ohm ]---[ SW3 ]--- GND      (ADC ~ 198)
                        +---[ 330 ohm ]---[ SW4 ]--- GND      (ADC ~ 254)
```

Idle (no button pressed): A0 floats high through the 1 kohm, ADC reads ~1023.

### Step-by-step on the breadboard

Pick a row (call it row X) that will be the **common node** carrying A0 + the top of every switch leg.

1. **Series resistor**: 1 kohm from the 5V rail to row X. This is R1; it stays in circuit at all times.
2. **A0 tap**: jumper from row X to the A0 rail of the mini breadboard (Option A) or to UNO pin A0 (Option B).
3. **SW1 (short / "STOP")**: a push-button from row X to GND, with **no** resistor in series. This is your B1 button.
4. **SW2 ("MODE")**: 180 ohm from row X to one leg of a push-button; the button's other leg to GND. This is B2.
5. **SW3 ("-" / decrement)**: 240 ohm from row X to one leg of a push-button; other leg to GND. This is B3.
6. **SW4 ("+" / increment)**: 330 ohm from row X to one leg of a push-button; other leg to GND. This is B4.

Tactile push-buttons are 4-pin but actually only 2 nets: the pins on each *short* side are tied together inside the body. The convention that always works is to bridge the breadboard's centre gap with the button so the two electrical nets sit on opposite sides of the gap. If a press has no effect, rotate the button 90 degrees.

### Resistor colour-band cheat (4-band, +/-5%)

- 1 kohm (1.0 kohm): brown - black - red - gold
- 180 ohm: brown - grey - brown - gold
- 240 ohm: red - yellow - brown - gold
- 330 ohm: orange - orange - brown - gold

There is a colour-band chart on slide 3 of the W1 deck if you have 5-band resistors instead.

## 5. Sanity-check before powering up

- The 1 kohm is in series between 5V and A0 *and only A0*. Never let a button short 5V to GND through anything less than that 1 kohm — that is what stops B1 (the "short" button) from being a dead short across the supply.
- Each of B2/B3/B4 has its own resistor between row X and the button; only the 180/240/330 ohm legs lead into a switch, not directly into GND.
- No bare wire ends touching anything they should not.

## 6. Power up and verify

1. Plug the UNO R4 WiFi into your Mac with a USB-C cable.
2. In the Arduino IDE:
   - Select **Tools > Board > Arduino UNO R4 Boards > Arduino UNO R4 WiFi**.
   - Select **Tools > Port > /dev/cu.usbmodem...** (the entry that appears only when the board is plugged in).
3. Click **Upload** (right-arrow icon).
4. Open the Serial Monitor at **115200 baud**. You should see `W1 ready. Type 'c' for a raw A0 sample (calibration).` and the LCD should display `Setpoint: 32 C` on line 1.
5. Press each button in turn. Expected behaviour:
   - **B1 (short)**: backlight turns red, line 2 shows `[STOP]`.
   - **B2 (180 ohm)**: backlight turns blue, line 2 shows `[MODE]`.
   - **B3 (240 ohm)**: backlight turns purple, setpoint counts **down** by 1.
   - **B4 (330 ohm)**: backlight turns green, setpoint counts **up** by 1.

If a button is detected as the wrong one, hold it down and type `c` + Enter in the Serial Monitor. The reported `A0 raw` value tells you which `BANDS[]` entry to tweak in the sketch. Resistor tolerance is +/-5% on the cheap ones, so the codes can drift by ~10 counts.

## 7. Troubleshooting

| Symptom | Likely cause |
|---|---|
| LCD backlight on but no characters | Library not installed, or Wire never started. Reinstall "Grove - LCD RGB Backlight" and re-upload. |
| LCD totally dark | Grove cable in a non-I2C port; Base Shield voltage switch on 3.3V. |
| Every press registers as B1 | Series 1k resistor missing or shorted; row X is going straight to GND through the button regardless of the leg resistor. |
| Press of B2/B3/B4 registers as B1 | The button's leg resistor is between the button and *GND* instead of between row X and the *button*; current still flows through the button short. Move the resistor to the row-X side. |
| Random presses appear when nothing is touched | Idle A0 is not sitting near 1023; series resistor not connected to 5V, or there is a short elsewhere on row X. |
| `Board at /dev/cu.usbmodem... is not available` | The R4's bootloader port disappears for a second during upload; just retry. If it keeps failing, double-tap the on-board reset to force DFU mode. |

## 8. Grove Ultrasonic Distance Sensor v2

Used for tracking sample rise inside the chamber.

- Plug a 4-wire Grove cable from the sensor's onboard socket to the **D2** socket on the Base Shield. That's the whole hookup.
- The sensor uses a **single bidirectional signal line** — the firmware pulses it HIGH for 5 us to fire a ping, then switches the pin to INPUT and measures the returned echo pulse width. There is no separate Trig/Echo split as on a bare HC-SR04.
- VCC and GND travel through the same Grove cable. The shield's 3.3 V / 5 V selector is **not critical** for this module — its onboard PCB handles logic-level conversion.

Range: ~2 cm to ~4 m, ±3 mm typical. Mount the sensor face-down ~10-20 cm above the sample surface; the ~15° cone needs a target wider than ~5 cm at that distance to return a clean echo.

## 9. Grove Temperature & Humidity Sensor Pro v1.3 (DHT22 / AM2302)

The chamber's air temperature and relative humidity probe.

- Plug a 4-wire Grove cable from the sensor's onboard socket to the **D4** socket on the Base Shield.
- Single-wire proprietary protocol on the primary signal line (yellow = D4). The secondary line (D5) is unused.
- Powered through the Grove cable; tolerates the shield being set to either 3.3 V or 5 V.

Sensor characteristics:

| Parameter | Value |
|---|---|
| Temperature range | -40 °C to +80 °C, ±0.5 °C accuracy |
| Humidity range | 0–100 % RH, ±2 % accuracy |
| Minimum read interval | 2 s (datasheet limit) |
| First valid reading after power-on | ~1 s warm-up |

The firmware samples the DHT22 every 2.5 s and silently holds the previous good value on a NaN return (which the sensor produces occasionally — usually less than 1 in 50 reads, harmless).

## 10. Grove SPDT Relay + silicone heater pad

Closes the temperature loop: the relay switches a silicone heater pad (or any mains/PSU-driven heater of your choice) on and off under firmware control.

- Plug a 4-wire Grove cable from the relay module's onboard socket to the **D8** signal pin on the Base Shield. The relay is active-HIGH — driving D8 HIGH energises the coil and closes the NO contacts.
- The relay's onboard optocoupler/transistor stage isolates the coil from the Arduino's 5 V rail, so the inductive kickback never reaches the MCU side.
- **Wire the heater across the NO and COM screw terminals** of the relay. Use mains-rated cabling if the heater is mains-powered; for the silicone 12 V pad supplied with the kit, any insulated 0.5 mm² stranded wire is fine.

The firmware runs a bang-bang controller around the temperature setpoint:

| Condition | Heater state |
|---|---|
| current < setpoint − 0.5 °C | ON |
| setpoint − 0.5 °C ≤ current ≤ setpoint + 0.5 °C | hold (no change) |
| current > setpoint + 0.5 °C | OFF |
| current ≥ 45 °C (hard cap) | OFF (regardless of setpoint) |
| DHT22 has not produced a valid reading yet | OFF (safety lockout) |

A `*` next to the temperature on the LCD TEMP page indicates the heater is currently drawing power. Every transition is also logged on serial as `[heater] ON / OFF (current=X.X C, set=Y C)`.

## 11. Grove Speaker (audio annunciator on A0 / DAC)

A passive piezo with an onboard LM386 amplifier. The Grove Speaker takes an analogue audio waveform on its signal pin and reproduces it. We feed it the UNO R4 WiFi's true 12-bit DAC output on **A0** — no PWM carrier artefacts to fight.

- Plug the speaker's Grove cable into the **A0** Grove socket on the Base Shield. The primary signal line of that socket is the DAC output.
- The module has a small onboard **potentiometer** for analogue volume — turn it down if it is too loud for the room. The firmware-side volume (page **VOLUME**, levels 0–3) sits on top of this and genuinely controls amplitude because it now drives the DAC swing.

The firmware uses [TalkiePCM](https://github.com/pschatzmann/TalkiePCM) — a Linear Predictive Coding speech synthesiser modelled on the 1980s Texas Instruments TMS5220 (the chip in the original Speak & Spell). It produces 8 kHz PCM samples that we stream straight into the DAC, so MODE press now causes the panel to actually say "twenty five degrees" / "fifty eight percent" / "fifteen centimetres" instead of beeping out a digit count.

Speech characteristics:

| Property | Value |
|---|---|
| Vocabulary | ~250 pre-built words + an integer-to-words expander |
| Sample rate | 8 kHz |
| Audio quality | Recognisable but robotic ("Speak & Spell" timbre) |
| Vocoder cost | ~150 bytes per word in flash; ~5 % CPU during playback |
| Announce trigger | MODE press lands on a new page; `s` over serial replays |

A short system **beep** for events that are not speech (button confirmation, sensor fault) is generated by driving the DAC as a square wave around midpoint; volume level 1–3 sets the amplitude swing, level 0 keeps the DAC parked at midpoint (silent).
