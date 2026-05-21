/*
 * Environmental Data Logger — Sensor Polling State Machine
 * Arduino Nano 33 IoT  (SAMD21G18A, 3.3 V)
 *
 * Each sensor is polled through a state machine that handles retries
 * and power-cycling before permanently marking a sensor invalid.
 *
 * State machine flow (per sensor):
 *
 *   valid == false ──────────────────────────────────────────────► END
 *
 *   valid == true
 *       │
 *   PS_SENSOR_ON  (D9 HIGH)
 *       │
 *   PS_COMMUNICATE  ◄──────────────────────────────────────────┐
 *       │                                                       │
 *   PS_CHECK_VALUE                                              │
 *       ├── reading OK ──► PS_STORE ──► END                    │
 *       │                                                       │
 *       └── reading FAIL                                        │
 *               count_poll++                                    │
 *               │                                              │
 *               ├── count_poll < 2  ─────────────────────────►─┘  (soft retry)
 *               │
 *               ├── 2 ≤ count_poll < 4 ──► PS_POWER_CYCLE         (hard retry)
 *               │                               D9 LOW
 *               │                               delay 50 ms
 *               │                               D9 HIGH
 *               │                               └──► PS_COMMUNICATE ──► PS_CHECK_VALUE
 *               │                                        (same branching as above)
 *               │
 *               └── count_poll ≥ 4 ──► sensor valid = false ──► END
 *
 * Wiring
 * ------
 *   BMP388    SCK=D13  SDI=D11  SDO=D12  CSB=D10
 *   DS18B20   DQ=D8   (4.7 kΩ pull-up to 3.3 V required)
 *   DS3231    SCL=A3  SDA=D3   INT/SQW=D2
 *   D9 — HIGH powers BMP388 + DS18B20; DS3231 is always powered.
 *
 * Libraries (Arduino Library Manager)
 *   Adafruit BMP3XX Library
 *   OneWire  by Paul Stoffregen
 *   DallasTemperature  by Miles Burton
 *
 * First upload: uncomment #define SET_RTC_TIME, upload, comment out, re-upload.
 */

// ---------------------------------------------------------------------------
// LogTime struct — MUST be before #includes so the Arduino IDE auto-prototype
// pass sees the type before generating the prototype for rtc_read().
// Named LogTime to avoid the SAMD21 core's RTC_ namespace (RTC_IRQn etc.).
// ---------------------------------------------------------------------------
struct LogTime {
  uint8_t  second, minute, hour;
  uint8_t  dayOfWeek, date, month;
  uint16_t year;
};

// ---------------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------------
#include <SPI.h>
#include <Adafruit_BMP3XX.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---------------------------------------------------------------------------
// Pin / address definitions
// ---------------------------------------------------------------------------
#define BMP_CS          10
#define DS18B20_PIN      8
#define ENABLE_PIN_SENSORS       9      // HIGH = BMP388 + DS18B20 powered
#define RTC_INT_PIN      2      // DS3231 INT/SQW — open-drain, active LOW
#define RTC_SDA          3
#define RTC_SCL         A3
#define DS3231_ADDR     0x68
#define ENABLE_PIN_MOTOR     A2
#define ENABLE_PIN_ESP     A6
#define ELECTRODE_1     4
#define ELECTRODE_2     5
#define CONDUCT_SENSE     A0
#define UART_BAUD       115200
#define COMPLETE_TIMEOUT_MS       30000   //30 seconds


// ---------------------------------------------------------------------------
// Sensor objects
// ---------------------------------------------------------------------------
Adafruit_BMP3XX   bmp;
OneWire           oneWire(DS18B20_PIN);
DallasTemperature ds18(&oneWire);

// ---------------------------------------------------------------------------
// Global sensor validity flags
// Initialised true; set false if a sensor exceeds the retry limit.
// Once false the sensor is permanently skipped for all future cycles.
// ---------------------------------------------------------------------------
bool g_bmp_valid  = true;
bool g_ds18_valid = true;

// Last confirmed-valid readings (NAN until first successful read)
float g_bmp_pres  = -10.0f;
float g_ds18_temp = -10.0f;


// ---------------------------------------------------------------------------
// ISR — DS3231 alarm fires, D2 goes LOW (falling edge)
// ---------------------------------------------------------------------------
volatile bool alarmFired = false;
void rtcAlarmISR() { alarmFired = true; }

// ---------------------------------------------------------------------------
// Bit-bang I2C — DS3231 only (SAMD21 Wire/SERCOM hangs permanently)
// Open-drain: release = INPUT_PULLUP, drive = OUTPUT LOW
// ---------------------------------------------------------------------------
static void sda_hi() { pinMode(RTC_SDA, INPUT_PULLUP); }
static void sda_lo() { pinMode(RTC_SDA, OUTPUT); digitalWrite(RTC_SDA, LOW); }
static void scl_hi() { pinMode(RTC_SCL, INPUT_PULLUP); }
static void scl_lo() { pinMode(RTC_SCL, OUTPUT); digitalWrite(RTC_SCL, LOW); }

static void bb_start() {
  sda_hi(); scl_hi(); delayMicroseconds(5);
  sda_lo();           delayMicroseconds(5);
  scl_lo();           delayMicroseconds(5);
}
static void bb_stop() {
  sda_lo();
  scl_hi(); delayMicroseconds(5);
  sda_hi(); delayMicroseconds(5);
}
static bool bb_write(uint8_t d) {
  for (int i = 7; i >= 0; i--) {
    if (d & (1 << i)) sda_hi(); else sda_lo();
    delayMicroseconds(3);
    scl_hi(); delayMicroseconds(5);
    scl_lo(); delayMicroseconds(3);
  }
  sda_hi(); delayMicroseconds(3);
  scl_hi(); delayMicroseconds(5);
  bool ack = (digitalRead(RTC_SDA) == LOW);
  scl_lo(); delayMicroseconds(3);
  return ack;
}
static uint8_t bb_read(bool sendAck) {
  uint8_t d = 0;
  sda_hi();
  for (int i = 7; i >= 0; i--) {
    delayMicroseconds(3);
    scl_hi(); delayMicroseconds(5);
    if (digitalRead(RTC_SDA)) d |= (1 << i);
    scl_lo(); delayMicroseconds(3);
  }
  if (sendAck) sda_lo(); else sda_hi();
  delayMicroseconds(3);
  scl_hi(); delayMicroseconds(5);
  scl_lo(); delayMicroseconds(3);
  sda_hi();
  return d;
}
static void rtc_wreg(uint8_t reg, uint8_t val) {
  bb_start();
  bb_write((DS3231_ADDR << 1) | 0);
  bb_write(reg); bb_write(val);
  bb_stop();
}
static uint8_t rtc_rreg(uint8_t reg) {
  bb_start(); bb_write((DS3231_ADDR << 1) | 0); bb_write(reg); bb_stop();
  bb_start(); bb_write((DS3231_ADDR << 1) | 1);
  uint8_t v = bb_read(false);
  bb_stop();
  return v;
}
static void rtc_rburst(uint8_t reg, uint8_t* buf, uint8_t len) {
  bb_start(); bb_write((DS3231_ADDR << 1) | 0); bb_write(reg); bb_stop();
  bb_start(); bb_write((DS3231_ADDR << 1) | 1);
  for (uint8_t i = 0; i < len - 1; i++) buf[i] = bb_read(true);
  buf[len - 1] = bb_read(false);
  bb_stop();
}
static uint8_t bcd2dec(uint8_t b) { return (b >> 4) * 10 + (b & 0x0F); }
static uint8_t dec2bcd(uint8_t d) { return ((d / 10) << 4) | (d % 10); }

// ---------------------------------------------------------------------------
// RTC read
// ---------------------------------------------------------------------------
static LogTime rtc_read() {
  uint8_t r[7];
  rtc_rburst(0x00, r, 7);
  LogTime t;
  t.second    = bcd2dec(r[0] & 0x7F);
  t.minute    = bcd2dec(r[1] & 0x7F);
  t.hour      = bcd2dec(r[2] & 0x3F);
  t.dayOfWeek = r[3] & 0x07;
  t.date      = bcd2dec(r[4] & 0x3F);
  t.month     = bcd2dec(r[5] & 0x1F);
  t.year      = bcd2dec(r[6]) + 2000;
  return t;
}

// ---------------------------------------------------------------------------
// DS3231 Alarm 1
//
// Registers 0x07–0x0A:
//   0x07  seconds  A1M1=0 → match seconds
//   0x08  minutes  A1M2=0 → match minutes
//   0x09  hours    A1M3=1 → ignore
//   0x0A  date     A1M4=1 → ignore
// → fires once per hour when min:sec matches; re-armed every 10 s.
//
// Control  0x0E  bit2 INTCN=1 → INT/SQW outputs interrupt
//                bit0 A1IE=1  → Alarm 1 interrupt enabled
// Status   0x0F  bit0 A1F     → cleared in software after servicing
// ---------------------------------------------------------------------------
static void rtc_init() {
  rtc_wreg(0x0E, 0x04); // INTCN=1, alarms disabled for now
  rtc_wreg(0x0F, 0x00); // clear OSF, A2F, A1F
  delay(10);
}
static void rtc_arm_alarm1() {
  LogTime now = rtc_read();
  uint8_t s   = now.second + 5;
  uint8_t m   = now.minute;
  if (s >= 60) { s -= 60; m = (m + 1) % 60; }
  rtc_wreg(0x07, dec2bcd(s));
  rtc_wreg(0x08, dec2bcd(m));
  rtc_wreg(0x09, 0x80);                    // A1M3=1 ignore hour
  rtc_wreg(0x0A, 0x80);                    // A1M4=1 ignore date
  rtc_wreg(0x0F, rtc_rreg(0x0F) & ~0x01); // clear A1F → INT de-asserts
  rtc_wreg(0x0E, 0x05);                    // INTCN=1, A1IE=1
}
static void rtc_clear_alarm1() {
  rtc_wreg(0x0F, rtc_rreg(0x0F) & ~0x01);
}

// ---------------------------------------------------------------------------
// Sensor power control
// sensors_on()  — D9 HIGH; only waits 100 ms if D9 was actually LOW
// sensors_off() — tri-states SPI + 1-Wire lines then D9 LOW
//                 (prevents back-feeding through I/O protection diodes)
// ---------------------------------------------------------------------------
static void sensors_on() {
  if (digitalRead(ENABLE_PIN_SENSORS) == LOW) {
    digitalWrite(ENABLE_PIN_SENSORS, HIGH);
    delay(100);   // VDD ramp + BMP388 2 ms boot + DS18B20 start-up
  }
}
static void sensors_off() {
  SPI.end();
  pinMode(11,          INPUT);   // MOSI
  pinMode(12,          INPUT);   // MISO
  pinMode(13,          INPUT);   // SCK
  pinMode(BMP_CS,      INPUT);   // CS
  pinMode(DS18B20_PIN, INPUT);   // 1-Wire DQ
  digitalWrite(ENABLE_PIN_SENSORS, LOW);
}

// ---------------------------------------------------------------------------
// Sensor communicate + store functions
//
// Each communicate function:
//   • writes its result into a file-scope staging variable
//   • returns true  if the reading is valid (within plausible range)
//   • returns false if the sensor did not respond or the value is out of range
//
// Each store function copies the staging variable into the global reading.
// Store is only ever called after communicate returned true.
// ---------------------------------------------------------------------------

// ── BMP388 (SPI) ────────────────────────────────────────────────────────────

static float _bmp_pres_stg = NAN;
static float p = NAN;

static bool bmp_communicate() {
  _bmp_pres_stg = NAN;
/////////////////////////////////
  bool bmpOK = bmp.begin_SPI(BMP_CS);
  float bmpPresHPa = NAN;
  if (bmpOK) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    // performReading() triggers forced mode and polls drdy (~14 ms).
    if (bmp.performReading()) {
      bmpPresHPa = bmp.pressure / 100.0f; // Pa → hPa
    } else {return false;}
  } else {return false;}

//DELAY to allow pressure sensor to initialize
//delay(2000);
  delay(1000);

  if (bmp.performReading()) {
    bmpPresHPa = bmp.pressure / 100.0f; // Pa → hPa
  }


  //CALIBRATION
//float pressure = p_uncalib*0.9269 + 66.58;
  //p = p*0.9269 + 66.58;
  float bmpPresHPa_calib = bmpPresHPa*0.9269 + 66.58;


///////////////////////
  //Range check: BMP388 operating range and upper altuitude limit change for implementation
  if (bmpPresHPa_calib < 1000.0f || bmpPresHPa_calib > 1250.0f)  return false;

  _bmp_pres_stg = bmpPresHPa_calib;
  return true;
}
static void bmp_store() {
  g_bmp_pres = _bmp_pres_stg;
}

// ── DS18B20 (1-Wire / Dallas) ────────────────────────────────────────────────
static float _ds18_temp_stg = NAN;  // staging: filled by ds18_communicate()

static bool ds18_communicate() {
  _ds18_temp_stg = NAN;

  ds18.begin();
  //discard first reading
  ds18.requestTemperatures();
  delay(800);

  if (ds18.getDeviceCount() == 0)  return false;  // no sensor found on bus
  // ds18.setWaitForConversion(true);        // blocking: returns after conversion
  
  ds18.requestTemperatures();             

  float t = ds18.getTempCByIndex(0);

  if (t == DEVICE_DISCONNECTED_C)   return false;

  //CALIBRATE
  //float temp = 1.0175*temp_uncalib + 0.02442;
  t = t*1.0175 + 0.02442;

  if (t < -2.0f || t > 30.0f)    return false;  // DS18B20 operating range

  _ds18_temp_stg = t;
  return true;
}
static void ds18_store() {
  g_ds18_temp = _ds18_temp_stg;
}

// ---------------------------------------------------------------------------
// Sensor polling state machine
//
// Runs synchronously to completion for one sensor per call.
// Uses a function pointer for communicate() so the same machine drives
// both BMP388 and DS18B20 without duplicating control logic.
//
// Parameters:
//   valid      — reference to the sensor's global validity flag
//   comm_fn()  — initialise & read; returns true if reading is in-range
//   store_fn() — copy staging value to global; called only on success
// ---------------------------------------------------------------------------
enum PollState {
  PS_SENSOR_ON,    // D9 HIGH (if not already)
  PS_COMMUNICATE,  // call comm_fn(), store result in reading_ok
  PS_CHECK_VALUE,  // branch on reading_ok / count_poll
  PS_POWER_CYCLE,  // D9 LOW → 50 ms → D9 HIGH
  PS_END           // terminal state — exit while loop
};

static void poll_sensor(bool& valid,
                        bool (*comm_fn)(),
                        void (*store_fn)()) {

  // CHECK_VALID gate — skip permanently invalid sensors
  if (!valid) return;

  int       count_poll = 0;
  PollState state      = PS_SENSOR_ON;
  bool      reading_ok = false;

  while (state != PS_END) {
    switch (state) {

      // ── PS_SENSOR_ON ────────────────────────────────────────────────────
      // Power D9 HIGH (sensors_on is a no-op if already HIGH).
      // Transitions unconditionally to PS_COMMUNICATE.
      case PS_SENSOR_ON:
        sensors_on();
        state = PS_COMMUNICATE;
        break;

      // ── PS_COMMUNICATE ──────────────────────────────────────────────────
      // Initialise the sensor over its interface and take a reading.
      // reading_ok = true  if the value is within the valid range.
      // reading_ok = false if the sensor did not respond or value out of range.
      // Transitions unconditionally to PS_CHECK_VALUE.
      case PS_COMMUNICATE:
        reading_ok = comm_fn();
        state = PS_CHECK_VALUE;
        break;

      // ── PS_CHECK_VALUE ───────────────────────────────────────────────────
      // Three outcomes:
      //   (A) reading_ok           → store value → END
      //   (B) !reading_ok, count_poll < 2   → soft retry (PS_COMMUNICATE)
      //   (C) !reading_ok, 2 ≤ count_poll < 4 → power-cycle retry (PS_POWER_CYCLE)
      //   (D) !reading_ok, count_poll ≥ 4  → mark invalid → END
      case PS_CHECK_VALUE:
        if (reading_ok) {
          // (A) — valid reading obtained
          store_fn();
          state = PS_END;

        } else {
          count_poll++;

          if (count_poll < 2) {
            // (B) — soft retry: re-communicate without touching power
            state = PS_COMMUNICATE;

          } else if (count_poll >= 4) {
            // (D) — retry limit exceeded: sensor permanently invalid
            valid = false;
            state = PS_END;

          } else {
            // (C) — count_poll is 2 or 3: hard retry with power cycle
            state = PS_POWER_CYCLE;
          }
        }
        break;

      // ── PS_POWER_CYCLE ───────────────────────────────────────────────────
      // Hard reset: cut sensor VDD, wait 50 ms for supply to discharge,
      // restore VDD and allow boot time, then re-communicate.
      // SPI lines are tri-stated in sensors_off() to prevent back-feed.
      case PS_POWER_CYCLE:
        sensors_off();          // D9 LOW + tri-state SPI/1-Wire
        delay(50);              // discharge VDD
        sensors_on();           // D9 HIGH + 100 ms stabilise
        state = PS_COMMUNICATE;
        break;

      case PS_END:
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// Print helpers
// ---------------------------------------------------------------------------
static void p2(uint8_t v) { if (v < 10) Serial.print('0'); Serial.print(v); }
static const char* dowName(uint8_t d) {
  static const char* n[] = { "", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
  return (d >= 1 && d <= 7) ? n[d] : "???";
}
static void printTimestamp(const LogTime& t) {
  Serial.print(dowName(t.dayOfWeek)); Serial.print(' ');
  p2(t.date);   Serial.print('/');
  p2(t.month);  Serial.print('/');
  Serial.print(t.year);   Serial.print("  ");
  p2(t.hour);   Serial.print(':');
  p2(t.minute); Serial.print(':');
  p2(t.second);
}

// ---------------------------------------------------------------------------
// Optional compile-time RTC set
// Uncomment SET_RTC_TIME once, upload, comment out again and re-upload.
// ---------------------------------------------------------------------------
#define SET_RTC_TIME
#ifdef SET_RTC_TIME
static uint8_t calcDow(uint16_t y, uint8_t m, uint8_t d) {
  static const int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
  if (m < 3) y--;
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7 + 1;
}
static void rtc_set_compile_time() {
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char md[] = __DATE__, mt[] = __TIME__;
  char ms[4] = { md[0], md[1], md[2], 0 };
  uint8_t  mo = (uint8_t)((strstr(months, ms) - months) / 3 + 1);
  uint8_t  dy = (md[4] == ' ' ? 0 : md[4] - '0') * 10 + (md[5] - '0');
  uint16_t yr = (md[7]-'0')*1000+(md[8]-'0')*100+(md[9]-'0')*10+(md[10]-'0');
  uint8_t  h  = (mt[0]-'0')*10+(mt[1]-'0');
  uint8_t  mn = (mt[3]-'0')*10+(mt[4]-'0');
  uint8_t  sc = (mt[6]-'0')*10+(mt[7]-'0');
  rtc_wreg(0x0E, 0x04); rtc_wreg(0x0F, 0x00);
  bb_start();
  bb_write((DS3231_ADDR << 1) | 0); bb_write(0x00);
  bb_write(dec2bcd(sc)); bb_write(dec2bcd(mn)); bb_write(dec2bcd(h));
  bb_write(calcDow(yr, mo, dy));
  bb_write(dec2bcd(dy)); bb_write(dec2bcd(mo)); bb_write(dec2bcd(yr - 2000));
  bb_stop();
  Serial.print("RTC set: ");
  Serial.print(yr); Serial.print('-');
  if (mo < 10) Serial.print('0'); Serial.print(mo); Serial.print('-');
  if (dy < 10) Serial.print('0'); Serial.println(dy);
}
#endif





//UART functions
void sendDataUART(float value1, float value2, float value3, 
                  int flag, int year, int month, int day, 
                  int hour, int minute, int second) {
    
    char buffer[64];  // Sufficient size for the formatted string
    
    // Format: "DATA:21.02,1000.01,5.03,1,2026,05,12,13,53,22"
    snprintf(buffer, sizeof(buffer), 
             "DATA:%.2f,%.2f,%.2f,%d,%d,%02d,%02d,%02d,%02d,%02d",
             value1, value2, value3, flag, year, month, day, 
             hour, minute, second);
    
    // Send via UART (using Serial1)
    Serial1.print(buffer);

}


// Waits for "ON" on Serial1. Returns true if received in time.
bool waitForON() {
  Serial.println("   Waiting for ON...");
  unsigned long start = millis();

  while (millis() - start < COMPLETE_TIMEOUT_MS) {
    if (Serial1.available()) {
      String response = Serial1.readStringUntil('ON');
      // response.trim();
      Serial.print("   Received: ");
      Serial.println(response);
      return true;

      // if (response == "ON") {
      //   return true;
      // }
    }
  }

  Serial.println("   ERROR: Timed out waiting for ON.");
  return false;
}

// Waits for "COMPLETE" on Serial1. Returns true if received in time.
bool waitForCOMPLETE() {
  Serial.println("   Waiting for COMPLETE...");
  unsigned long start = millis();

  while (millis() - start < COMPLETE_TIMEOUT_MS) {
    if (Serial1.available()) {
      String response = Serial1.readStringUntil('COMPLETE');
      // response.trim();
      Serial.print("   Received: ");
      Serial.println(response);

      return true;
      // if (response == "COMPLETE") {
      //   return true;
      // }
    }
  }

  Serial.println("   ERROR: Timed out waiting for COMPLETE.");
  return false;
}





// ===========================================================================
// setup
// ===========================================================================
void setup() {
  Serial.begin(9600);
  while (!Serial);


  pinMode(LED_BUILTIN, OUTPUT);

  
  // Sensors off — tri-state SPI/1-Wire, D9 LOW
  pinMode(ENABLE_PIN_SENSORS, OUTPUT);
  sensors_off();

  pinMode(ENABLE_PIN_MOTOR, OUTPUT);
  digitalWrite(ENABLE_PIN_MOTOR, LOW);

  pinMode(ENABLE_PIN_ESP, OUTPUT);
  digitalWrite(ENABLE_PIN_ESP, LOW);
  

  // D2 = DS3231 INT/SQW: open-drain active-LOW, falling edge wakes CPU
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_INT_PIN), rtcAlarmISR, FALLING);

  sda_hi(); scl_hi();
  delay(10);

  rtc_init();

#ifdef SET_RTC_TIME
  rtc_set_compile_time();
  Serial.println();
#endif

  rtc_arm_alarm1();   // first alarm 10 s from now

  Serial.println("Environmental Logger — State Machine Sensor Polling");
  Serial.println("DS3231 Alarm 1 fires every ~10 s; CPU idles between alarms.\n");
  Serial.println("Timestamp               | DS18B20 (C) | Pressure (hPa) | BMP? | DS18?");
  Serial.println("------------------------|-------------|----------------|------|------");
}

// ===========================================================================
// loop
// ===========================================================================
void loop() {

  // ── PSEUDO SLEEP ─────────────────────────────────────────────────────────
  // CPU enters SAMD21 idle mode — all clocks/USB stay active so Serial works.
  // Returns on any interrupt; alarmFired flag filters out USB SOF wakes.
  Serial.flush();
  while (!alarmFired) {
    __WFI();
  }
  alarmFired = false;

  g_ds18_temp = -10.0f;   // reset before each cycle
  g_bmp_pres  = -10.0f;

  // ── POLL DS18B20 ─────────────────────────────────────────────────────────
  // State machine: turns D9 on, attempts to read, retries up to 4×,
  // power-cycles after 2 failures, marks invalid after 4 failures.
  poll_sensor(g_ds18_valid, ds18_communicate, ds18_store);

  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(500);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(500);

  // ── POLL BMP388 ──────────────────────────────────────────────────────────
  // D9 is already HIGH from the DS18B20 poll above (sensors_on is idempotent).
  // If DS18B20 poll ended with a power-cycle, D9 was restored HIGH before exit.
  poll_sensor(g_bmp_valid, bmp_communicate, bmp_store);

  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(250);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(250);
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(250);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(250);

  // ── TIMESTAMP ────────────────────────────────────────────────────────────
  LogTime t = rtc_read();

  // ── LOG ──────────────────────────────────────────────────────────────────
  printTimestamp(t);
  Serial.print(" | ");

  if (!isnan(g_ds18_temp)) {
    if (g_ds18_temp >= 0 && g_ds18_temp < 10) Serial.print(' ');
    Serial.print(g_ds18_temp, 2);
  } else {
    Serial.print(g_ds18_valid ? "  ---  " : "  INV  ");
  }
  Serial.print("        | ");

  if (!isnan(g_bmp_pres)) {
    Serial.print(g_bmp_pres, 2);
  } else {
    Serial.print(g_bmp_valid ? "   ---   " : "   INV   ");
  }
  Serial.print("  | ");

  // Validity status column — shows live / FAIL / INV
  Serial.print(g_bmp_valid  ? "OK  " : "INV ");
  Serial.print(" | ");
  Serial.println(g_ds18_valid ? "OK" : "INV");


  // ── POWER OFF ────────────────────────────────────────────────────────────
  sensors_off();



  //VIBRATE CYCLE
  digitalWrite(ENABLE_PIN_MOTOR, HIGH);
  Serial.print("Vibrating!");

  bool vibrated = digitalRead(ENABLE_PIN_MOTOR);

  Serial.print(vibrated ? "Vibrated? TRUE" : "Vibrated? FALSE");
  Serial.println();

  
  digitalWrite(ENABLE_PIN_MOTOR, LOW);
  Serial.print("Stopped Vibrating.");
  Serial.println();

  //Wait 30 seconds SETTLING DELAY
  Serial.print("SETTLING DELAY.");
  Serial.println();

//30 seconds for reasons
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);

  Serial.print("SETTLING DELAY LAPSED.");
  Serial.println();
  
  //CAMERA subsystem interfacing
  Serial.print("Turning on CAM and serial");

  //Waiting to receive "ON" string from esp
  Serial1.begin(UART_BAUD);
  digitalWrite(ENABLE_PIN_ESP, HIGH);

  // confirms ESP is on
  bool ESP_ON = waitForON();


  if (ESP_ON) {
     //Transmitting string
    sendDataUART(g_ds18_temp, g_bmp_pres, -10.00, 
                  (int)vibrated, t.year, t.month, t.date, 
                  t.hour, t.minute, t.second);
  
    
  }
  else {
    Serial.print("CAM not responding");
    }

  bool ESP_DONE = waitForCOMPLETE();

  if (ESP_DONE) {
    Serial.print("CAM operations Complete");
  } else {
    Serial.print("CAM not responding");
  }

  // turn off ESP
  digitalWrite(ENABLE_PIN_ESP, LOW);

  // ── RE-ARM ───────────────────────────────────────────────────────────────
  rtc_clear_alarm1();
  rtc_arm_alarm1();

  
}
