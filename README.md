# Stereo dummy load with true RMS wattmeter

Two-channel resistive dummy load for audio amplifier testing (4 ohm / 8 ohm per channel), with true RMS power measurement, dual OLED display, thermal management and serial data output.

## Specifications

| Parameter | Value |
|---|---|
| Channels | 2 (L / R), independent |
| Load | 4 ohm or 8 ohm per channel, selected by switch |
| Measurement | Differential, true RMS, AC + DC |
| Measurement range | +/-70 V peak at the load (49 Vrms sine, approx. 300 W into 8 ohm) |
| Common-mode range | approx. -70 V to +90 V at the load terminals |
| Residual common-mode error | < 0.35 V for 30 V common mode (measured, after trim) |
| Resolution | 1 ADC LSB = 0.22 V at the load (Vcc = 5 V), averaged over 2048 samples |
| Sampling | 2048 samples per channel, interleaved L/R, approx. 460 ms window |
| Update rate | approx. 2 Hz |
| Load terminal impedance | 47 kohm to measurement ground per terminal (BTL safe) |
| Supply | 5 V, 2 A min. |
| Calibration | Against DMM, 16 V DC into 8 ohm |

Power dissipation is limited by the load resistors and the cooling, not by the measurement chain.

## Measurement principle

Each channel uses one LM324 section as a difference amplifier with a gain of 22k / 1M = 0.022. The output is centered on Vref = 0.32 x Vcc, provided by a buffered divider (third LM324 section) and read on A2.

<img width="200" alt="IMG_1085" src="https://github.com/user-attachments/assets/caeae74e-7e05-4670-a487-0ec3beafe68d" />

Schematics (made with claude, lazy to open eagle) :

<p align="center">
  <img src="https://github.com/user-attachments/assets/d74fc22f-117a-4ca5-8052-24aa84518d7c" width="49%" alt="Difference amplifier, channel L">
  <img src="https://github.com/user-attachments/assets/0f778467-bc4d-4b9c-842a-8325fbe25bcc" width="49%" alt="Vref buffer, ground reference, LM324 pinout">
</p>


Signal processing (firmware):

- AVcc measured each cycle against the internal 1.1 V reference, compensates 5 V rail variations.
- Per channel: sum and sum of squares over the window, variance computed exactly in 64-bit integers.
- DC component = channel mean - Vref (A2) - calibrated offset.
- Vrms = sqrt(variance + DC^2), scaled by gain and calibration factor, residual noise removed in quadrature.
- P = Vrms^2 / R_load, displayed for both the 4 ohm and 8 ohm values.
- Saturation flagged when the LM324 output reaches its rails (low limit 10 LSB, high limit Vcc - 1.6 V).

## Bill of materials

### Control and display

| Qty | Component | Reference / Value |
|---|---|---|
| 1 | Microcontroller board | Arduino Nano, ATmega328P |
| 1 | Temperature sensor | DS18B20 + 4.7 kohm pull-up |
| 2 | OLED display | SSD1306 128x64 I2C, addresses 0x3C / 0x3D |
| 1 | Fan | 5 V, 120 mm |
| 1 | Relay, fan | 5 V |
| 2 | Relay, protection | 5 V |
| 1 | LED, thermal status | Red + 10 kohm |
| 2 | LED, saturation | + 1 kohm each |
| 1 | Power supply | 5 V, 2 A min. |
| 1 | Quad op-amp | LM324N, DIP-14, socketed |
| 4 | Resistor, input | 1 Mohm, 1 % metal film recommended |
| 2 | Resistor, feedback | 22 kohm, 1 % |
| 2 | Resistor, reference leg | 20 kohm, 1 % |
| 2 | Trimmer, CMRR adjust | 5 kohm multi-turn |
| 4 | Resistor, ground reference | 47 kohm, 1/4 W |
| 1 | Resistor, Vref divider | 10 kohm |
| 1 | Resistor, Vref divider | 4.7 kohm |
| 1 | Capacitor, Vref filter | 10 uF electrolytic |
| 1 | Capacitor, decoupling | 100 nF ceramic |
| 4 | Power resistor | 2.7 ohm, 100 W |
| 2 | Power resistor | 15 ohm, 100 W |
| 2 | Power resistor | 2.5 ohm, 100 W |
| 2 | Switch, load select | SPDT ON-ON, 1321S |
| 2 | Input terminals | Jack / banana |

## Pinout

| Arduino pin | Function |
|---|---|
| A0 | Channel L measurement (LM324 pin 1) |
| A1 | Channel R measurement (LM324 pin 7) |
| A2 | Vref (LM324 pin 8) |
| A4 / A5 | I2C SDA / SCL, OLED displays |
| D2 | Protection relay, channel R |
| D3 | Protection relay, channel L |
| D4 | Fan relay |
| D6 | Saturation LED, channel R |
| D7 | Saturation LED, channel L |
| D8 | Thermal status LED |
| D12 | DS18B20 data |

## Power and grounding rules

1. PSU 5 V output to the Nano 5V pin. Do not use VIN: the onboard regulator drops the rail to approx. 4.2 V.
2. LM324 pin 4 and the top of the Vref divider are supplied from the same 5V node as the Nano. The measurement is ratiometric only if Vref and the ADC share the same supply.
3. Fan and relays are wired directly to the PSU output, with their own wires.
4. USB can stay connected with the PSU on (the Nano USB input is diode-protected). Do not run on USB alone with the PSU wired but off.
5. Load terminals connect to the measurement ground only through the 47 kohm resistors. Never connect a load terminal directly to ground (BTL / class D amplifiers).
6. Load current must never flow through the measurement ground. With a bench supply: power leads directly to the load terminals, separate thin wire from the supply negative to the measurement ground.
7. The 1 Mohm input nodes are leakage sensitive. Remove flux residue with isopropyl alcohol after any soldering near the LM324, then apply conformal coating. Lock the trimmers after adjustment.

## Firmware

### Configuration flags

| Flag | Default | Function |
|---|---|---|
| `CFG_WATCHDOG` | 0 | 2 s watchdog, requires Optiboot |
| `CFG_SERIAL_MONITOR` | 1 | CSV output at 115200 baud |
| `CFG_DC_COUPLED` | 1 | 1: AC + DC RMS (requires Vref on A2), 0: AC only |

### Thermal management

| State | Condition | Fan | Protection relays | Thermal LED |
|---|---|---|---|---|
| Normal | T < 25 C (falling) | Off | Off | Off |
| Cooling | T >= 27 C | On | Off | Steady |
| Protection | T >= 35 C, released below 30 C | On | On | Blinking |
| Sensor fault | DS18B20 missing, CRC error or timeout | On | On | Blinking |

Sensor fault is fail-safe: fan and protection relays are energized until valid readings resume.

### Display

Per channel: Vrms, P (4 ohm), P (8 ohm), temperature and thermal state. `SAT` is shown when the measurement chain saturates. Saturation LEDs stay on for 1 s after the last event.

<img width="300" alt="IMG_1088" src="https://github.com/user-attachments/assets/bbed2a5d-c325-4710-919c-3e2b886b0a7e" />

### Serial output

One CSV line per measurement cycle, 115200 baud, `;` separator. A header line is sent at startup.

| Column | Unit | Description |
|---|---|---|
| `u_l` / `u_r` | V | RMS voltage at the load |
| `p4_l` / `p4_r` | W | Power, 4 ohm load value |
| `p8_l` / `p8_r` | W | Power, 8 ohm load value |
| `sat_l` / `sat_r` | 0/1 | Saturation flag |
| `min_l`, `max_l` / `min_r`, `max_r` | LSB | Raw ADC extremes over the window |
| `dc_l` / `dc_r` | LSB | Raw mean - Vref, before offset correction |
| `vcc` | V | Measured AVcc |
| `temp` | C | DS18B20 temperature |
| `state` | - | 0 Normal, 1 Cooling, 2 Protection, 3 Sensor fault |

## Calibration

The `dc` columns are raw values, independent of the calibration constants: the procedure can be run with any firmware revision. Average 5 to 20 lines for each reading. Warm up for 5 minutes before starting.

1. Common-mode trim, one channel at a time.
   - Bench supply positive on one terminal of the channel only (e.g. L+), negative to measurement ground, current limit 50 mA.
   - Read `dc` at 0 V and at 30 V.
   - Adjust the channel trimmer until |dc(30 V) - dc(0 V)| < 2 LSB.
2. Offset. Bench supply disconnected, no signal: average `dc_l` / `dc_r` into `OFFSET_L_COUNTS` / `OFFSET_R_COUNTS`.
3. Gain. 16 V DC into 8 ohm, current limit 2.5 A, DMM across the load terminals.
   - U_raw = (dc - OFFSET) x vcc / 1024 / 0.022
   - `CAL_x` = U_DMM / U_raw
   - Repeat with reversed polarity: both directions must agree.
4. Load values. `LOAD_8R` = U_load / I_supply at 16 V, `LOAD_4R` likewise at 8 V.
5. Noise floor. No signal, residual Vrms into `NOISE_FLOOR_V`.

## Photos

![IMG_4635](https://github.com/user-attachments/assets/8eafffcc-8349-46ea-a188-e03a429097c2)
<img width="2635" height="780" alt="IMG_1087" src="https://github.com/user-attachments/assets/bf4614a1-a671-47e1-a8d1-12ffe020745f" />


## References

- [Compact stereo dummy load for amplifier testing (tolisdiy)](http://tolisdiy.com/2021/03/30/compact-stereo-dummy-load-for-amplifier-testing/)
- [Arduino audio wattmeter (andydoz)](https://andydoz.blogspot.com/2016/04/arduino-audio-wattmeter.html)
- [TI SBOA582, CMRR of difference amplifiers vs. resistor matching](https://www.ti.com/lit/an/sboa582/sboa582.pdf)
- [LM324 datasheet](https://www.ti.com/lit/ds/symlink/lm324.pdf)
