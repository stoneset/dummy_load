/*
 * arduino_sketch.ino
 *
 * Stereo dummy load - true RMS power measurement and thermal management
 *
 * Target       : Arduino Nano, ATmega328P 16 MHz (old bootloader: CFG_WATCHDOG 0)
 * Measurement  : one LM324 difference amplifier per channel, true RMS AC+DC
 *                Vout = Vref + G * (V+ - V-), G = 22k / 1M, Vref = 0.32 x Vcc
 *                (+) side: 1M from terminal, 20k + 5k trimmer to Vref (CMRR adjust)
 *                (-) side: 1M from terminal, 22k feedback
 *                Vref (LM324 pin 8, follower) read back on A2
 *                47k from each load terminal to measurement ground
 * Display      : 2 x SSD1306 128x64 I2C, 0x3C channel L, 0x3D channel R
 * Thermal      : DS18B20 on D12, fan relay D4, protection relays D2 (R) / D3 (L)
 * Indicators   : D8 thermal, D7 saturation L, D6 saturation R
 * Dependencies : OneWire, DallasTemperature, U8g2
 *
 * Calibration (raw dc column, independent of CAL and OFFSET):
 *   1. Common mode, one channel at a time: supply (+) on one terminal only,
 *      (-) to ground. Adjust trimmer until |dc(30 V) - dc(0 V)| < 2 LSB.
 *   2. Idle, supply disconnected: mean of dc_x -> OFFSET_x_COUNTS.
 *   3. 16 V DC into 8 ohm: CAL_x = U_dmm / U_raw,
 *      U_raw = (dc_x - OFFSET_x) * Vcc / 1024 / G.
 *   4. LOAD_xR = U_load / I_supply.
 *   5. Idle: residual Vrms -> NOISE_FLOOR_V.
 */

#include <avr/wdt.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>

/* ---------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

#define CFG_WATCHDOG        0   /* 1: 2 s watchdog, requires Optiboot (not "Old Bootloader") */
#define CFG_SERIAL_MONITOR  1   /* 1: CSV line at 115200 baud after each measurement */
#define CFG_DC_COUPLED      1   /* 1: total AC+DC power, requires Vref wired to A2
                                   0: AC component only */

/* Pin map */
constexpr uint8_t PIN_RELAY_PROT_R = 2;
constexpr uint8_t PIN_RELAY_PROT_L = 3;
constexpr uint8_t PIN_RELAY_FAN    = 4;
constexpr uint8_t PIN_LED_SAT_R    = 6;
constexpr uint8_t PIN_LED_SAT_L    = 7;
constexpr uint8_t PIN_LED          = 8;
constexpr uint8_t PIN_DS18B20      = 12;
constexpr uint8_t PIN_MEAS_L       = A0;
constexpr uint8_t PIN_MEAS_R       = A1;
constexpr uint8_t PIN_MEAS_VREF    = A2;

/* Relay active level (LOW for active-low relay modules) */
constexpr uint8_t RELAY_ON  = HIGH;
constexpr uint8_t RELAY_OFF = LOW;

/* Thermal thresholds in deg C, with hysteresis */
constexpr float TEMP_FAN_ON   = 27.0f;
constexpr float TEMP_FAN_OFF  = 25.0f;
constexpr float TEMP_PROT_ON  = 35.0f;
constexpr float TEMP_PROT_OFF = 30.0f;
constexpr float TEMP_MIN_VALID = -55.0f;   /* DS18B20 range */
constexpr float TEMP_MAX_VALID = 125.0f;

/* Measured load resistances in ohm, wiring included */
constexpr float LOAD_4R = 4.09f;    /* 8.18 V / 2.000 A */
constexpr float LOAD_8R = 8.015f;   /* 15.83 V / 1.975 A */

/* Measurement chain */
constexpr float DIFF_GAIN     = 22.0e3f / 1.0e6f;
constexpr float CAL_L         = 0.988f;
constexpr float CAL_R         = 0.990f;
constexpr float NOISE_FLOOR_V = 0.15f;     /* subtracted in quadrature */
constexpr float OFFSET_L_COUNTS = -0.1f;   /* idle dc_l */
constexpr float OFFSET_R_COUNTS = 3.8f;    /* idle dc_r */
constexpr float BANDGAP_V     = 1.100f;    /* nominal internal reference */
constexpr float VCC_NOMINAL   = 5.000f;
constexpr float VCC_MIN_VALID = 4.0f;
constexpr float VCC_MAX_VALID = 5.6f;

constexpr uint16_t ADC_SAMPLES   = 2048;   /* per channel, ~460 ms window */
constexpr uint8_t  VREF_SAMPLES  = 64;     /* Vref reads before and after the window */
constexpr uint16_t ADC_FULLSCALE = 1024;
constexpr uint16_t ADC_CLIP_LOW  = 10;     /* LM324 output at low rail (~0 V) */
constexpr float    LM324_HEADROOM_V = 1.5f; /* max output swing: Vcc - 1.5 V */
constexpr float    CLIP_MARGIN_V    = 0.1f; /* high SAT threshold below the rail */
constexpr uint32_t SAT_HOLD_MS   = 1000;   /* indicator hold time after last saturation */

static_assert((uint64_t)ADC_SAMPLES * 1023u * 1023u <= UINT32_MAX,
              "ADC_SAMPLES too large for the 32-bit accumulator");

/* DS18B20 */
constexpr uint8_t  TEMP_RESOLUTION_BITS = 10;     /* 0.25 C, 188 ms conversion */
constexpr uint32_t TEMP_TIMEOUT_MS      = 1000;   /* unfinished conversion = fault */

constexpr uint32_t SERIAL_BAUD    = 115200;
constexpr uint32_t I2C_CLOCK_HZ   = 400000;
constexpr uint16_t LAMP_TEST_MS   = 1000;

/* ---------------------------------------------------------------------------
 * Types
 * ------------------------------------------------------------------------- */

enum class ThermalState : uint8_t {
  Normal,
  Cooling,
  Protection,
  SensorFault
};

struct SatIndicator {
  uint8_t  pin;
  bool     seen;       /* at least one saturation since power-up */
  uint32_t lastMs;     /* time of last saturation */
};

struct ChannelMeas {
  float    vrms;       /* RMS voltage across the load, V */
  float    dcCounts;   /* mean deviation from Vref in ADC LSB, before offset correction */
  bool     overrange;  /* LM324 output at a rail during the window */
  uint16_t rawMin;     /* raw ADC extremes over the window, diagnostics */
  uint16_t rawMax;
};

/* ---------------------------------------------------------------------------
 * Objects and global state
 * ------------------------------------------------------------------------- */

static OneWire           oneWire(PIN_DS18B20);
static DallasTemperature tempSensor(&oneWire);

/* Page mode (_1_): 128-byte buffer per display instead of 1024 */
static U8G2_SSD1306_128X64_NONAME_1_HW_I2C displayL(U8G2_R0, U8X8_PIN_NONE);
static U8G2_SSD1306_128X64_NONAME_1_HW_I2C displayR(U8G2_R0, U8X8_PIN_NONE);

static ThermalState thermalState     = ThermalState::SensorFault;
static float        temperature      = DEVICE_DISCONNECTED_C;
static bool         temperatureValid = false;
static uint32_t     tempRequestMs    = 0;
static bool         ledBlink         = false;
static SatIndicator satL             = { PIN_LED_SAT_L, false, 0 };
static SatIndicator satR             = { PIN_LED_SAT_R, false, 0 };

/* ---------------------------------------------------------------------------
 * Measurement
 * ------------------------------------------------------------------------- */

/* AVcc measured against the internal 1.1 V reference. Compensates
 * 5 V rail variations (fan and relay switching). */
static float readVcc()
{
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);

  uint16_t raw = 0;
  for (uint8_t i = 0; i < 2; i++) {       /* first conversion discarded */
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {
    }
    raw = ADC;
  }

  if (raw == 0) {
    return VCC_NOMINAL;
  }
  const float vcc = BANDGAP_V * ADC_FULLSCALE / raw;
  return (vcc > VCC_MIN_VALID && vcc < VCC_MAX_VALID) ? vcc : VCC_NOMINAL;
}

static uint32_t sumVref()
{
  uint32_t s = 0;
  for (uint8_t i = 0; i < VREF_SAMPLES; i++) {
    s += analogRead(PIN_MEAS_VREF);
  }
  return s;
}

/* Variance in LSB^2, exact 64-bit computation: (N * S2 - S1^2) / N^2 */
static float varianceCounts(uint32_t s1, uint32_t s2)
{
  const int64_t num = (int64_t)ADC_SAMPLES * s2 - (int64_t)s1 * s1;
  return (num > 0) ? (float)num / ((float)ADC_SAMPLES * ADC_SAMPLES) : 0.0f;
}

/* RMS value in LSB. In AC+DC mode, the DC component is the difference
 * between the channel mean and Vref, corrected for the op-amp offset. */
static float rmsCounts(uint32_t s1, uint32_t s2, float vref, float offset, float &dcRaw)
{
  dcRaw = (float)s1 / ADC_SAMPLES - vref;
  const float var = varianceCounts(s1, s2);
#if CFG_DC_COUPLED
  const float dc = dcRaw - offset;
  return sqrtf(var + dc * dc);
#else
  (void)offset;
  return sqrtf(var);
#endif
}

static float countsToLoadVolts(float counts, float vcc, float cal)
{
  const float v = counts * (vcc / ADC_FULLSCALE) / DIFF_GAIN * cal;
  const float v2 = v * v - NOISE_FLOOR_V * NOISE_FLOOR_V;
  return (v2 > 0.0f) ? sqrtf(v2) : 0.0f;
}

/* Interleaved L/R acquisition over the same window. Vref is read before
 * and after the window to average out slow drift. */
static void measureChannels(float vcc, ChannelMeas &l, ChannelMeas &r)
{
  uint32_t s1L = 0, s2L = 0, s1R = 0, s2R = 0;
  uint16_t minL = 1023, maxL = 0, minR = 1023, maxR = 0;

  const uint32_t vrefBefore = sumVref();

  for (uint16_t i = 0; i < ADC_SAMPLES; i++) {
    const uint16_t a = analogRead(PIN_MEAS_L);
    const uint16_t b = analogRead(PIN_MEAS_R);

    s1L += a;
    s2L += (uint32_t)a * a;
    s1R += b;
    s2R += (uint32_t)b * b;

    if (a < minL) minL = a;
    if (a > maxL) maxL = a;
    if (b < minR) minR = b;
    if (b > maxR) maxR = b;
  }

  const uint32_t vrefAfter = sumVref();
  const float vref = (float)(vrefBefore + vrefAfter) / (2u * VREF_SAMPLES);

  l.vrms = countsToLoadVolts(rmsCounts(s1L, s2L, vref, OFFSET_L_COUNTS, l.dcCounts), vcc, CAL_L);
  r.vrms = countsToLoadVolts(rmsCounts(s1R, s2R, vref, OFFSET_R_COUNTS, r.dcCounts), vcc, CAL_R);

  /* LM324 high rail in ADC LSB, tracks the supply voltage */
  const uint16_t clipHigh =
      (uint16_t)((vcc - LM324_HEADROOM_V - CLIP_MARGIN_V) / vcc * ADC_FULLSCALE);

  l.overrange = (minL < ADC_CLIP_LOW) || (maxL > clipHigh);
  r.overrange = (minR < ADC_CLIP_LOW) || (maxR > clipHigh);
  l.rawMin = minL;
  l.rawMax = maxL;
  r.rawMin = minR;
  r.rawMax = maxR;
}

/* Saturation indicator, held for SAT_HOLD_MS so short clipping stays visible. */
static void updateSatLed(SatIndicator &ind, bool sat)
{
  const uint32_t now = millis();
  if (sat) {
    ind.seen = true;
    ind.lastMs = now;
  }
  const bool on = ind.seen && (now - ind.lastMs < SAT_HOLD_MS);
  digitalWrite(ind.pin, on ? HIGH : LOW);
}

/* ---------------------------------------------------------------------------
 * Thermal
 * ------------------------------------------------------------------------- */

/* Non-blocking read: the conversion runs during the ADC acquisition. */
static void temperatureService()
{
  const uint32_t now = millis();

  if (tempSensor.isConversionComplete()) {
    const float t = tempSensor.getTempCByIndex(0);
    temperatureValid = (t > TEMP_MIN_VALID) && (t < TEMP_MAX_VALID);
    temperature = t;
  } else if (now - tempRequestMs < TEMP_TIMEOUT_MS) {
    return;
  } else {
    temperatureValid = false;           /* bus stuck */
  }

  tempSensor.requestTemperatures();
  tempRequestMs = now;
}

static ThermalState nextThermalState(ThermalState cur, float t, bool valid)
{
  if (!valid) {
    return ThermalState::SensorFault;
  }

  switch (cur) {
  case ThermalState::Normal:
    if (t >= TEMP_PROT_ON) return ThermalState::Protection;
    if (t >= TEMP_FAN_ON)  return ThermalState::Cooling;
    return ThermalState::Normal;

  case ThermalState::Cooling:
    if (t >= TEMP_PROT_ON) return ThermalState::Protection;
    if (t <  TEMP_FAN_OFF) return ThermalState::Normal;
    return ThermalState::Cooling;

  case ThermalState::Protection:
    if (t < TEMP_PROT_OFF) return ThermalState::Cooling;
    return ThermalState::Protection;

  case ThermalState::SensorFault:
  default:
    /* Sensor back: resume in the safest state */
    return (t >= TEMP_PROT_OFF) ? ThermalState::Protection : ThermalState::Cooling;
  }
}

/* LED steady while cooling, blinking in protection or sensor fault. */
static void applyOutputs(ThermalState s)
{
  const bool fan  = (s != ThermalState::Normal);
  const bool prot = (s == ThermalState::Protection) || (s == ThermalState::SensorFault);

  digitalWrite(PIN_RELAY_FAN,    fan  ? RELAY_ON : RELAY_OFF);
  digitalWrite(PIN_RELAY_PROT_L, prot ? RELAY_ON : RELAY_OFF);
  digitalWrite(PIN_RELAY_PROT_R, prot ? RELAY_ON : RELAY_OFF);

  ledBlink = !ledBlink;
  const bool led = prot ? ledBlink : fan;
  digitalWrite(PIN_LED, led ? HIGH : LOW);
}

static const char *thermalLabel(ThermalState s)
{
  switch (s) {
  case ThermalState::Normal:     return "OK";
  case ThermalState::Cooling:    return "FAN";
  case ThermalState::Protection: return "PROTECTION";
  default:                       return "SENSOR FAULT";
  }
}

/* ---------------------------------------------------------------------------
 * Display
 * ------------------------------------------------------------------------- */

static void drawChannel(U8G2 &d, const char *title, const ChannelMeas &m, const char *status)
{
  char u[8], p4[8], p8[8];
  dtostrf(m.vrms, 6, 2, u);
  dtostrf(m.vrms * m.vrms / LOAD_4R, 6, 1, p4);
  dtostrf(m.vrms * m.vrms / LOAD_8R, 6, 1, p8);

  d.firstPage();
  do {
    d.drawStr(0, 9, title);
    if (m.overrange) {
      d.drawStr(110, 9, "SAT");
    }
    d.drawHLine(0, 12, 128);

    d.drawStr(0, 24, "U");
    d.drawStr(30, 24, u);
    d.drawStr(72, 24, "Vrms");

    d.drawStr(0, 36, "P 4R");
    d.drawStr(30, 36, p4);
    d.drawStr(72, 36, "W");

    d.drawStr(0, 48, "P 8R");
    d.drawStr(30, 48, p8);
    d.drawStr(72, 48, "W");

    d.drawStr(0, 62, status);
  } while (d.nextPage());
}

static void buildStatus(char *buf, size_t len)
{
  if (thermalState == ThermalState::SensorFault) {
    strlcpy(buf, thermalLabel(thermalState), len);
    return;
  }
  char t[8];
  dtostrf(temperature, 4, 1, t);
  snprintf(buf, len, "T %sC %s", t, thermalLabel(thermalState));
}

static void initDisplay(U8G2 &d, uint8_t addr7)
{
  d.setI2CAddress(addr7 << 1);
  d.setBusClock(I2C_CLOCK_HZ);
  d.begin();
  d.setFont(u8g2_font_6x10_tr);
}

/* ---------------------------------------------------------------------------
 * Serial output
 * ------------------------------------------------------------------------- */

#if CFG_SERIAL_MONITOR
static void serialReport(const ChannelMeas &l, const ChannelMeas &r, float vcc)
{
  const ChannelMeas *ch[2] = { &l, &r };
  for (uint8_t i = 0; i < 2; i++) {
    const float v2 = ch[i]->vrms * ch[i]->vrms;
    Serial.print(ch[i]->vrms, 3);  Serial.print(';');
    Serial.print(v2 / LOAD_4R, 2); Serial.print(';');
    Serial.print(v2 / LOAD_8R, 2); Serial.print(';');
    Serial.print(ch[i]->overrange ? 1 : 0); Serial.print(';');
    Serial.print(ch[i]->rawMin); Serial.print(';');
    Serial.print(ch[i]->rawMax); Serial.print(';');
    Serial.print(ch[i]->dcCounts, 2); Serial.print(';');
  }
  Serial.print(vcc, 3);          Serial.print(';');
  Serial.print(temperature, 2);  Serial.print(';');
  Serial.println(static_cast<uint8_t>(thermalState));
}
#endif

/* ---------------------------------------------------------------------------
 * Entry points
 * ------------------------------------------------------------------------- */

void setup()
{
#if CFG_WATCHDOG
  MCUSR = 0;
  wdt_disable();
#endif

  /* Output level set before pin direction: no relay glitch at boot */
  digitalWrite(PIN_RELAY_FAN,    RELAY_OFF);
  digitalWrite(PIN_RELAY_PROT_L, RELAY_OFF);
  digitalWrite(PIN_RELAY_PROT_R, RELAY_OFF);
  digitalWrite(PIN_LED,          HIGH);       /* lamp test */
  digitalWrite(PIN_LED_SAT_L,    HIGH);
  digitalWrite(PIN_LED_SAT_R,    HIGH);
  pinMode(PIN_RELAY_FAN,    OUTPUT);
  pinMode(PIN_RELAY_PROT_L, OUTPUT);
  pinMode(PIN_RELAY_PROT_R, OUTPUT);
  pinMode(PIN_LED,          OUTPUT);
  pinMode(PIN_LED_SAT_L,    OUTPUT);
  pinMode(PIN_LED_SAT_R,    OUTPUT);

  analogReference(DEFAULT);

#if CFG_SERIAL_MONITOR
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("u_l;p4_l;p8_l;sat_l;min_l;max_l;dc_l;u_r;p4_r;p8_r;sat_r;min_r;max_r;dc_r;vcc;temp;state"));
#endif

  initDisplay(displayL, 0x3C);
  initDisplay(displayR, 0x3D);

  /* First conversion blocking, then asynchronous mode */
  tempSensor.begin();
  tempSensor.setResolution(TEMP_RESOLUTION_BITS);
  tempSensor.setWaitForConversion(true);
  tempSensor.requestTemperatures();
  temperature = tempSensor.getTempCByIndex(0);
  temperatureValid = (temperature > TEMP_MIN_VALID) && (temperature < TEMP_MAX_VALID);
  tempSensor.setWaitForConversion(false);
  tempSensor.requestTemperatures();
  tempRequestMs = millis();

  thermalState = nextThermalState(ThermalState::SensorFault, temperature, temperatureValid);
  if (thermalState == ThermalState::Cooling && temperature < TEMP_FAN_OFF) {
    thermalState = ThermalState::Normal;
  }

  delay(LAMP_TEST_MS);
  applyOutputs(thermalState);
  digitalWrite(PIN_LED_SAT_L, LOW);
  digitalWrite(PIN_LED_SAT_R, LOW);

#if CFG_WATCHDOG
  wdt_enable(WDTO_2S);
#endif
}

void loop()
{
#if CFG_WATCHDOG
  wdt_reset();
#endif

  ChannelMeas l;
  ChannelMeas r;
  const float vcc = readVcc();
  measureChannels(vcc, l, r);
  updateSatLed(satL, l.overrange);
  updateSatLed(satR, r.overrange);

  temperatureService();
  thermalState = nextThermalState(thermalState, temperature, temperatureValid);
  applyOutputs(thermalState);

  char status[24];
  buildStatus(status, sizeof(status));
  drawChannel(displayL, "LEFT CHANNEL", l, status);
  drawChannel(displayR, "RIGHT CHANNEL", r, status);

#if CFG_SERIAL_MONITOR
  serialReport(l, r, vcc);
#endif
}
