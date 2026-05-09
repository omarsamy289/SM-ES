/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║           SMART MATTRESS — ESP32 Firmware v2.0          ║
 * ║                      Omar Samy                          ║
 * ╠══════════════════════════════════════════════════════════╣
 * ║  HARDWARE MAP                                           ║
 * ║  Motor 1 (IRFZ44N gate) ── GPIO 32                     ║
 * ║  Motor 2 (IRFZ44N gate) ── GPIO 21                     ║
 * ║  Motor 3 (IRFZ44N gate) ── GPIO 4                      ║
 * ║  Motor 4 (IRFZ44N gate) ── GPIO 13                     ║
 * ║  Fan     (IRFZ44N gate) ── GPIO 23                     ║
 * ║  Piezo Impact           ── GPIO 14  (ADC)              ║
 * ║  Piezo Vibration        ── GPIO 27  (ADC)              ║
 * ║  DHT22                  ── GPIO 2                      ║
 * ║  RTC DS3231 SDA/SCL     ── GPIO 17/5  (Wire)           ║
 * ║  RTC DS3231 SQW         ── GPIO 18                     ║
 * ║  OLED 0.96" SDA/SCL     ── GPIO 22/19 (Wire1)          ║
 * ║  MPU6050  SDA/SCL/INT   ── GPIO 25/26/33               ║
 * ║    (MPU uses Wire1 bus — shares with OLED, both        ║
 * ║     0x68 vs 0x3C so no address collision)              ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * REQUIRED LIBRARIES (install via Library Manager):
 *   - DHT sensor library (Adafruit)
 *   - RTClib (Adafruit)
 *   - Adafruit SSD1306
 *   - Adafruit GFX Library
 *   - MPU6050 (Electronic Cats or I2Cdevlib)
 *   - WiFi, WebServer (built-in ESP32)
 *
 * NOTE ON I2C:
 *   Wire  (bus 0) → RTC DS3231  (SDA=17, SCL=5)
 *   Wire1 (bus 1) → OLED 0x3C + MPU6050 0x68 (SDA=22, SCL=19)
 *   MPU SDA/SCL are physically on GPIO25/26 but we software-
 *   bridge them by calling Wire1.begin(22,19) — if your MPU
 *   is wired to 25/26 you need a separate TwoWire instance;
 *   see WIRING NOTE below.
 *
 * WIRING NOTE — THREE I2C BUSES:
 *   ESP32 has 2 HW I2C buses. Your MPU is on 25/26, OLED on
 *   22/19, RTC on 17/5.  Solution: use TwoWire for all three.
 *   Wire  → RTC   (SDA=17, SCL=5)
 *   Wire1 → OLED  (SDA=22, SCL=19)
 *   Wire2 → MPU   (SDA=25, SCL=26)  ← software I2C via TwoWire(2)
 */

// ──────────────────────────── INCLUDES ────────────────────────────
#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <MPU6050.h>           // Electronic Cats MPU6050 lib
#include <esp_sleep.h>
#include <driver/rtc_io.h>

// ──────────────────────────── PIN DEFINITIONS ─────────────────────
#define DHTPIN          2
#define DHTTYPE         DHT22

#define MOTOR_PIN_1     32
#define MOTOR_PIN_2     21
#define MOTOR_PIN_3     4
#define MOTOR_PIN_4     13
#define FAN_PIN         23

#define PIEZO_IMPACT    14     // ADC — woke from deep sleep via ext0
#define PIEZO_VIB       27     // ADC
#define RTC_SQW_PIN     18     // RTC alarm interrupt (ext0 wake)
#define MPU_INT_PIN     33     // MPU6050 motion interrupt (ext1 wake)

// I2C buses
#define RTC_SDA         17
#define RTC_SCL         5
#define OLED_SDA        22
#define OLED_SCL        19
#define MPU_SDA         25
#define MPU_SCL         26

// OLED
#define SCREEN_W        128
#define SCREEN_H        64
#define OLED_ADDR       0x3C

// ──────────────────────────── CONSTANTS ──────────────────────────
#define MAX_ALARMS      9
#define SNOOZE_SEC      10
#define ALARM_VIB_SEC   30
#define FAN_ON_HOUR     12     // Noon scheduled fan-on (RTC task)
#define WAKE_HOUR       23     // 11 PM — system wakes via RTC

// Sleep-score history (one point per 30 s while awake, up to 8 h)
#define SCORE_HISTORY   960    // 8 h × 2 pts/min

// Piezo ADC threshold (raw 12-bit, tweak per hardware)
#define PIEZO_THRESH    300

// ──────────────────────────── GLOBALS ────────────────────────────

/* I2C buses */
TwoWire I2C_RTC  = TwoWire(0);   // RTC
TwoWire I2C_OLED = TwoWire(1);   // OLED
// MPU uses its own TwoWire instance via constructor below

/* Peripherals */
DHT          dht(DHTPIN, DHTTYPE);
RTC_DS3231   rtc;
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &I2C_OLED, -1);
MPU6050      mpu;                 // will be given Wire instance in setup

/* Network */
const char* ssid     = "ESP32-omar";
const char* password = "om1414";
WebServer   server(80);

/* Sensor readings */
float temperature  = 0;
float humidity     = 0;
int16_t ax, ay, az, gx, gy, gz;   // raw MPU readings

/* Sleep score */
float sleepScore          = 100.0;
float scoreHistory[SCORE_HISTORY];
int   scoreHead           = 0;     // circular buffer head
int   scoreCount          = 0;     // how many valid entries
float dailyScore          = -1;    // saved at 9 AM
char  dailyScoreDate[12]  = "";

/* Alarms */
struct Alarm {
  int  hour;
  int  minute;
  bool enabled;
  bool fired;
};
Alarm alarms[MAX_ALARMS];

/* Motor / fan state */
int  motorPWM[4]    = {0,0,0,0};  // individual slider PWM
bool motorActive[4] = {false,false,false,false};
int  fanPWM         = 0;
bool fanOn          = false;

/* Pattern / sequencer */
bool   patternActive    = false;
bool   fanTaskActive    = false;   // scheduled fan-on task
unsigned long fanTaskEnd = 0;

/* OLED page sequencer */
int           oledPage        = 0;
unsigned long lastOledSwitch  = 0;
#define OLED_PAGE_INTERVAL 3000   // ms per page

/* Snooze */
bool          snoozePending   = false;
unsigned long snoozeEnd       = 0;

/* Timing */
unsigned long lastSensorRead  = 0;
unsigned long lastScoreUpdate = 0;
unsigned long motorTimers[4]  = {0,0,0,0};  // 5-s button timers
unsigned long fanTimer        = 0;

/* Wake cause (stored in RTC memory — survives deep sleep) */
RTC_DATA_ATTR int  wakeCount   = 0;
RTC_DATA_ATTR bool firstBoot   = true;

// ──────────────────────────── FORWARD DECLARATIONS ───────────────
void handleRoot();
void handleData();
void handleControl();
void runBuzzPattern();
void startupSequence();
void welcomeProgram();
void enterDeepSleep();
void updateOLED();
void drawScoreGraph(bool fullScreen);
void updateSleepScore();
void checkAlarms(DateTime& now);
void checkPiezo();
void setMotor(int idx, int pwm);
void setFan(int pwm);
String buildScoreJSON();
String buildAlarmJSON();

// ─────────────────────────── SETUP ───────────────────────────────
void setup() {
  Serial.begin(115200);

  // ── I2C buses ──
  I2C_RTC.begin(RTC_SDA, RTC_SCL);
  I2C_OLED.begin(OLED_SDA, OLED_SCL);

  // ── RTC ──
  rtc = RTC_DS3231();
  if (!rtc.begin(&I2C_RTC)) {
    Serial.println("[RTC] not found!");
  } else {
    Serial.println("[RTC] OK");
    // Clear any existing RTC alarms
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
    rtc.disableAlarm(1);
    rtc.disableAlarm(2);
  }

  // ── OLED ──
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] not found!");
  } else {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.display();
  }

  // ── MPU6050 (third I2C — software via Wire.begin on 25/26) ──
  // The MPU6050 library expects Wire; we temporarily override it:
  Wire.begin(MPU_SDA, MPU_SCL);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[MPU] not found!");
  } else {
    Serial.println("[MPU] OK");
    // Enable motion interrupt: threshold ~10 LSB, duration 1 sample
    mpu.setMotionDetectionThreshold(10);
    mpu.setMotionDetectionDuration(1);
    mpu.setIntMotionEnabled(true);
  }

  // ── DHT ──
  dht.begin();

  // ── Motor / Fan PWM ──
  ledcAttach(MOTOR_PIN_1, 5000, 8); ledcWrite(MOTOR_PIN_1, 0);
  ledcAttach(MOTOR_PIN_2, 5000, 8); ledcWrite(MOTOR_PIN_2, 0);
  ledcAttach(MOTOR_PIN_3, 5000, 8); ledcWrite(MOTOR_PIN_3, 0);
  ledcAttach(MOTOR_PIN_4, 5000, 8); ledcWrite(MOTOR_PIN_4, 0);
  ledcAttach(FAN_PIN,     5000, 8); ledcWrite(FAN_PIN, 0);

  // ── Alarms init ──
  for (int i = 0; i < MAX_ALARMS; i++) {
    alarms[i] = {0, 0, false, false};
  }

  // ── Score history init ──
  for (int i = 0; i < SCORE_HISTORY; i++) scoreHistory[i] = -1;

  // ── Determine wake cause ──
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.printf("[WAKE] cause=%d  firstBoot=%d\n", cause, firstBoot);

  bool doStartup = false;

  if (firstBoot) {
    firstBoot = false;
    doStartup = true;
  } else {
    switch (cause) {
      case ESP_SLEEP_WAKEUP_EXT0:   // Piezo impact or RTC SQW
        doStartup = true;
        break;
      case ESP_SLEEP_WAKEUP_EXT1:   // MPU6050 motion interrupt
        doStartup = true;
        break;
      case ESP_SLEEP_WAKEUP_TIMER:  // Internal timer (scheduled tasks)
      {
        // Scheduled wake: check if it is the fan-at-noon task
        DateTime now = rtc.now();
        if (now.hour() == FAN_ON_HOUR && now.minute() == 0) {
          // Fan-only task: run fan 1 min then go back to sleep
          setFan(200);
          delay(60000);
          setFan(0);
          enterDeepSleep();
          return;                    // never reached
        }
        doStartup = true;           // 11 PM scheduled full wake
        break;
      }
      default:
        doStartup = true;
        break;
    }
  }

  wakeCount++;

  // ── WiFi AP ──
  WiFi.softAP(ssid, password);
  Serial.print("[WiFi] AP IP: ");
  Serial.println(WiFi.softAPIP());

  // ── Web server routes ──
  server.on("/",            handleRoot);
  server.on("/control",     handleControl);
  server.on("/data",        handleData);

  server.on("/setPWM", []() {
    if (server.hasArg("ch") && server.hasArg("v")) {
      int ch  = constrain(server.arg("ch").toInt(), 0, 3);
      int val = constrain(server.arg("v").toInt(),  0, 255);
      motorPWM[ch] = val;
      setMotor(ch, val);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/motorPulse", []() {
    if (server.hasArg("ch")) {
      int ch = constrain(server.arg("ch").toInt(), 0, 3);
      setMotor(ch, 200);
      motorTimers[ch]  = millis();
      motorActive[ch]  = true;
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/setFan", []() {
    if (server.hasArg("v")) {
      fanPWM = constrain(server.arg("v").toInt(), 0, 255);
      setFan(fanPWM);
      fanOn = (fanPWM > 0);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/fanToggle", []() {
    fanOn = !fanOn;
    fanPWM = fanOn ? 200 : 0;
    setFan(fanPWM);
    server.send(200, "text/plain", fanOn ? "on" : "off");
  });

  server.on("/buzz", []() {
    patternActive = true;
    server.send(200, "text/plain", "buzz");
  });

  server.on("/addAlarm", []() {
    if (server.hasArg("h") && server.hasArg("m")) {
      for (int i = 0; i < MAX_ALARMS; i++) {
        if (!alarms[i].enabled) {
          alarms[i] = {server.arg("h").toInt(),
                       server.arg("m").toInt(), true, false};
          break;
        }
      }
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/deleteAlarm", []() {
    if (server.hasArg("i")) {
      int idx = server.arg("i").toInt();
      if (idx >= 0 && idx < MAX_ALARMS)
        alarms[idx] = {0, 0, false, false};
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/snooze", []() {
    snoozePending = true;
    snoozeEnd     = millis() + SNOOZE_SEC * 1000UL;
    server.send(200, "text/plain", "snoozed");
  });

  server.on("/setTime", []() {
    if (server.hasArg("h") && server.hasArg("m") && server.hasArg("s")) {
      DateTime now = rtc.now();
      rtc.adjust(DateTime(now.year(), now.month(), now.day(),
                          server.arg("h").toInt(),
                          server.arg("m").toInt(),
                          server.arg("s").toInt()));
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/sleep", []() {
    server.send(200, "text/plain", "sleeping");
    delay(500);
    enterDeepSleep();
  });

  server.on("/scoreHistory", []() {
    server.send(200, "application/json", buildScoreJSON());
  });

  server.on("/alarms", []() {
    server.send(200, "application/json", buildAlarmJSON());
  });

  server.begin();

  // ── Startup sequence ──
  if (doStartup) startupSequence();
}

// ─────────────────────────── LOOP ────────────────────────────────
void loop() {
  server.handleClient();

  unsigned long ms  = millis();
  DateTime      now = rtc.now();

  // ── Sensor reads (DHT every 2 s) ──
  if (ms - lastSensorRead >= 2000) {
    lastSensorRead = ms;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h)) humidity    = h;
    if (!isnan(t)) temperature = t;

    // MPU read
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  }

  // ── Sleep score update every 30 s ──
  if (ms - lastScoreUpdate >= 30000) {
    lastScoreUpdate = ms;
    updateSleepScore();

    // At 9 AM: save daily score
    if (now.hour() == 9 && now.minute() == 0) {
      dailyScore = sleepScore;
      snprintf(dailyScoreDate, sizeof(dailyScoreDate),
               "%02d/%02d/%04d", now.day(), now.month(), now.year());
    }
  }

  // ── Piezo check ──
  checkPiezo();

  // ── Alarm check ──
  checkAlarms(now);

  // ── Snooze expiry ──
  if (snoozePending && ms >= snoozeEnd) {
    snoozePending = false;
    patternActive = true;
  }

  // ── Motor 5-second pulse timers ──
  for (int i = 0; i < 4; i++) {
    if (motorActive[i] && ms - motorTimers[i] >= 5000) {
      motorActive[i] = false;
      setMotor(i, motorPWM[i]);   // restore slider value
    }
  }

  // ── Fan task timer ──
  if (fanTaskActive && ms >= fanTaskEnd) {
    fanTaskActive = false;
    setFan(0);
    fanOn = false;
  }

  // ── Run buzz pattern ──
  if (patternActive && !snoozePending) {
    patternActive = false;
    runBuzzPattern();
    // restore motors to slider values
    for (int i = 0; i < 4; i++) setMotor(i, motorPWM[i]);
  }

  // ── OLED update ──
  if (ms - lastOledSwitch >= OLED_PAGE_INTERVAL) {
    lastOledSwitch = ms;
    oledPage = (oledPage + 1) % 6;
  }
  updateOLED();

  // ── Scheduled deep-sleep at 11 PM (if no alarm active) ──
  // Only sleep if no alarm is set near this time
  if (now.hour() == 23 && now.minute() == 0 && !patternActive) {
    // Schedule wake at 11 PM next day via timer (~24 h)
    // In practice you'd set RTC alarm; here we use timer sleep
    enterDeepSleep();
  }
}

// ─────────────────────────── DEEP SLEEP ──────────────────────────
void enterDeepSleep() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(20, 28);
  oled.print("Entering sleep...");
  oled.display();
  delay(1000);
  oled.clearDisplay();
  oled.display();

  // Stop all outputs
  for (int i = 0; i < 4; i++) setMotor(i, 0);
  setFan(0);

  WiFi.softAPdisconnect(true);

  // Wake sources:
  //  EXT0  → GPIO 14 (piezo impact, active HIGH)
  //  EXT1  → GPIO 33 (MPU INT, active HIGH)  bitmask
  //  Timer → ~1 h (to handle fan-at-noon or 11PM wake)

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14, 1);  // Piezo impact HIGH
  esp_sleep_enable_ext1_wakeup((1ULL << MPU_INT_PIN), ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_sleep_enable_timer_wakeup(3600ULL * 1000000ULL); // 1 h fallback

  Serial.println("[SLEEP] entering deep sleep");
  esp_deep_sleep_start();
}

// ─────────────────────────── STARTUP SEQUENCE ────────────────────
void startupSequence() {
  // OLED welcome
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(10, 10);
  oled.print("Welcome to Everest");
  oled.setCursor(30, 30);
  oled.print("Initializing...");
  oled.display();
  delay(2000);

  welcomeProgram();
}

void welcomeProgram() {
  // Fan on for 30 s total; motors start after 5 s
  setFan(200);
  delay(5000);

  // 20-second PWM welcome sequence for 4 motors
  unsigned long start = millis();
  int phase = 0;
  while (millis() - start < 20000) {
    int t    = (millis() - start);
    int pwm  = (int)(127.5 + 127.5 * sin(t * 0.005));  // smooth sine wave
    for (int i = 0; i < 4; i++) ledcWrite(
      (int[]){MOTOR_PIN_1,MOTOR_PIN_2,MOTOR_PIN_3,MOTOR_PIN_4}[i], pwm);
    delay(20);
  }
  // All motors off; fan continues 5 more seconds to complete 30 s
  for (int i = 0; i < 4; i++) setMotor(i, 0);
  delay(5000);
  setFan(0);

  // Restore slider PWM
  for (int i = 0; i < 4; i++) setMotor(i, motorPWM[i]);
}

// ─────────────────────────── BUZZ PATTERN ────────────────────────
void runBuzzPattern() {
  // OLED: show motor status during buzz
  for (int pulse = 0; pulse < 20; pulse++) {
    ledcWrite(MOTOR_PIN_1, 200); ledcWrite(MOTOR_PIN_3, 200);
    ledcWrite(MOTOR_PIN_2, 0);   ledcWrite(MOTOR_PIN_4, 0);
    delay(200);
    ledcWrite(MOTOR_PIN_1, 0);   ledcWrite(MOTOR_PIN_3, 0);
    ledcWrite(MOTOR_PIN_2, 200); ledcWrite(MOTOR_PIN_4, 200);
    delay(100);
  }
  for (int i = 0; i < 4; i++) {
    ledcWrite((int[]){MOTOR_PIN_1,MOTOR_PIN_2,MOTOR_PIN_3,MOTOR_PIN_4}[i], 0);
  }
}

// ─────────────────────────── HELPERS ─────────────────────────────
void setMotor(int idx, int pwm) {
  int pins[] = {MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4};
  if (idx < 0 || idx > 3) return;
  ledcWrite(pins[idx], constrain(pwm, 0, 255));
}

void setFan(int pwm) {
  ledcWrite(FAN_PIN, constrain(pwm, 0, 255));
}

// ─────────────────────────── PIEZO CHECK ─────────────────────────
void checkPiezo() {
  int impact = analogRead(PIEZO_IMPACT);
  int vib    = analogRead(PIEZO_VIB);

  // Both piezos contribute to sleep score degradation
  if (impact > PIEZO_THRESH || vib > PIEZO_THRESH) {
    // Deduct from sleep score (each event = -0.5 points, clamped)
    sleepScore = constrain(sleepScore - 0.5, 0.0, 100.0);

    // Store in history
    scoreHistory[scoreHead] = sleepScore;
    scoreHead = (scoreHead + 1) % SCORE_HISTORY;
    if (scoreCount < SCORE_HISTORY) scoreCount++;
  }
}

// ─────────────────────────── SLEEP SCORE UPDATE ──────────────────
void updateSleepScore() {
  // MPU-based: count axis changes (X and Y)
  // Two half-turns on X = 1 full turn = -1 point
  static int16_t lastAx = 0, lastAy = 0;
  int16_t dAx = abs(ax - lastAx);
  int16_t dAy = abs(ay - lastAy);
  lastAx = ax;
  lastAy = ay;

  // Scale: 32768 = 1g. A significant roll > 0.3g
  float movX = dAx / 32768.0f;
  float movY = dAy / 32768.0f;
  float penalty = (movX + movY) * 5.0f;   // tune as needed

  sleepScore = constrain(sleepScore - penalty, 0.0, 100.0);

  // Push to circular history
  scoreHistory[scoreHead] = sleepScore;
  scoreHead = (scoreHead + 1) % SCORE_HISTORY;
  if (scoreCount < SCORE_HISTORY) scoreCount++;
}

// ─────────────────────────── ALARM CHECK ─────────────────────────
void checkAlarms(DateTime& now) {
  for (int i = 0; i < MAX_ALARMS; i++) {
    if (!alarms[i].enabled) continue;
    bool match = (now.hour()   == alarms[i].hour &&
                  now.minute() == alarms[i].minute);
    if (match && !alarms[i].fired) {
      alarms[i].fired = true;
      patternActive   = true;
    }
    if (!match) alarms[i].fired = false;
  }
}

// ─────────────────────────── OLED DISPLAY ────────────────────────
void updateOLED() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  DateTime now = rtc.now();

  // If any motor is active, show motor status overlay instead
  bool anyMotor = false;
  for (int i = 0; i < 4; i++) if (ledcRead(
      (int[]){MOTOR_PIN_1,MOTOR_PIN_2,MOTOR_PIN_3,MOTOR_PIN_4}[i]) > 0) anyMotor = true;
  bool fanRunning = (ledcRead(FAN_PIN) > 0);

  if (anyMotor || fanRunning) {
    // Motor/fan status page
    oled.setTextSize(1);
    oled.setCursor(0, 0); oled.print("== Motor Status ==");

    // 4 squares
    int cols[] = {0, 64, 0, 64};
    int rows[] = {12, 12, 38, 38};
    const char* labels[] = {"V1","V2","V3","V4"};
    int pins[] = {MOTOR_PIN_1,MOTOR_PIN_2,MOTOR_PIN_3,MOTOR_PIN_4};
    for (int i = 0; i < 4; i++) {
      bool on = ledcRead(pins[i]) > 0;
      oled.drawRect(cols[i], rows[i], 58, 22, SSD1306_WHITE);
      if (on) oled.fillRect(cols[i]+1, rows[i]+1, 56, 20, SSD1306_WHITE);
      oled.setTextColor(on ? SSD1306_BLACK : SSD1306_WHITE);
      oled.setCursor(cols[i]+18, rows[i]+7);
      oled.setTextSize(1);
      oled.print(labels[i]);
    }
    oled.setTextColor(SSD1306_WHITE);
    if (fanRunning) {
      oled.setCursor(0, 56);
      oled.printf("Fan ON  PWM:%d", ledcRead(FAN_PIN));
    }
    oled.display();
    return;
  }

  // Normal rotating pages
  switch (oledPage) {
    case 0: { // Next alarm
      oled.setTextSize(1);
      oled.setCursor(20, 0); oled.print("Next Alarm");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      bool found = false;
      for (int i = 0; i < MAX_ALARMS; i++) {
        if (alarms[i].enabled) {
          oled.setTextSize(2);
          oled.setCursor(20, 25);
          oled.printf("%02d:%02d", alarms[i].hour, alarms[i].minute);
          found = true;
          break;
        }
      }
      if (!found) {
        oled.setTextSize(1);
        oled.setCursor(5, 28); oled.print("No alarm is set");
      }
      break;
    }
    case 1: { // Sleep score graph (mini)
      oled.setTextSize(1);
      oled.setCursor(20, 0); oled.print("Sleep Score");
      drawScoreGraph(false);
      oled.setCursor(85, 56);
      oled.printf("%.0f", sleepScore);
      break;
    }
    case 2: { // Temp & Humidity
      oled.setTextSize(1);
      oled.setCursor(15, 0); oled.print("Temp & Humidity");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      oled.setTextSize(2);
      oled.setCursor(5, 18);
      oled.printf("%.1f C", temperature);
      oled.setCursor(5, 42);
      oled.printf("%.1f%%", humidity);
      break;
    }
    case 3: { // Current time
      oled.setTextSize(1);
      oled.setCursor(35, 0); oled.print("Time");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      oled.setTextSize(3);
      oled.setCursor(5, 20);
      oled.printf("%02d:%02d", now.hour(), now.minute());
      oled.setTextSize(1);
      oled.setCursor(30, 56);
      oled.printf("%02d/%02d/%04d", now.day(), now.month(), now.year());
      break;
    }
    case 4: { // System status
      oled.setTextSize(1);
      oled.setCursor(20, 0); oled.print("System Status");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      oled.setCursor(0, 15); oled.print("Status: Stable");
      oled.setCursor(0, 27);
      oled.printf("WiFi: %s", ssid);
      oled.setCursor(0, 39);
      oled.printf("Clients: %d", WiFi.softAPgetStationNum());
      oled.setCursor(0, 51);
      oled.printf("Wakes: %d", wakeCount);
      break;
    }
    case 5: { // Daily score (if saved)
      oled.setTextSize(1);
      oled.setCursor(15, 0); oled.print("Daily Score");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      if (dailyScore >= 0) {
        oled.setTextSize(2);
        oled.setCursor(30, 20);
        oled.printf("%.0f", dailyScore);
        oled.setTextSize(1);
        oled.setCursor(10, 50); oled.print(dailyScoreDate);
      } else {
        oled.setTextSize(1);
        oled.setCursor(5, 28); oled.print("No data yet");
      }
      break;
    }
  }
  oled.display();
}

void drawScoreGraph(bool fullScreen) {
  int gx0 = 0, gy0 = 12, gw = 128, gh = 50;
  if (!fullScreen) { gy0 = 12; gh = 50; }

  int pts = min(scoreCount, gw);
  if (pts < 2) return;

  for (int i = 0; i < pts - 1; i++) {
    int idx0 = (scoreHead - pts + i + SCORE_HISTORY) % SCORE_HISTORY;
    int idx1 = (scoreHead - pts + i + 1 + SCORE_HISTORY) % SCORE_HISTORY;
    float v0 = scoreHistory[idx0];
    float v1 = scoreHistory[idx1];
    if (v0 < 0 || v1 < 0) continue;
    int x0 = gx0 + i * gw / pts;
    int x1 = gx0 + (i+1) * gw / pts;
    int y0 = gy0 + gh - (int)(v0 / 100.0 * gh);
    int y1 = gy0 + gh - (int)(v1 / 100.0 * gh);
    oled.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
  }
}

// ─────────────────────────── JSON HELPERS ────────────────────────
String buildScoreJSON() {
  String j = "[";
  int pts = min(scoreCount, SCORE_HISTORY);
  for (int i = 0; i < pts; i++) {
    int idx = (scoreHead - pts + i + SCORE_HISTORY) % SCORE_HISTORY;
    if (scoreHistory[idx] >= 0) {
      if (j.length() > 1) j += ",";
      j += String(scoreHistory[idx], 1);
    }
  }
  j += "]";
  return j;
}

String buildAlarmJSON() {
  String j = "[";
  bool first = true;
  for (int i = 0; i < MAX_ALARMS; i++) {
    if (alarms[i].enabled) {
      if (!first) j += ",";
      j += "{\"i\":" + String(i) +
           ",\"h\":" + String(alarms[i].hour) +
           ",\"m\":" + String(alarms[i].minute) + "}";
      first = false;
    }
  }
  j += "]";
  return j;
}

// ─────────────────────────── DATA ENDPOINT ───────────────────────
void handleData() {
  DateTime now = rtc.now();
  char timeBuf[9], dateBuf[11];
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d",
           now.hour(), now.minute(), now.second());
  snprintf(dateBuf, sizeof(dateBuf), "%02d/%02d/%04d",
           now.day(), now.month(), now.year());

  char json[512];
  snprintf(json, sizeof(json),
    "{"
    "\"temp\":%.1f,"
    "\"hum\":%.1f,"
    "\"time\":\"%s\","
    "\"date\":\"%s\","
    "\"sleep\":%.1f,"
    "\"dailyScore\":%.1f,"
    "\"dailyDate\":\"%s\","
    "\"fanPWM\":%d,"
    "\"fanOn\":%s,"
    "\"motorPWM\":[%d,%d,%d,%d],"
    "\"ax\":%d,\"ay\":%d,\"az\":%d"
    "}",
    temperature, humidity, timeBuf, dateBuf,
    sleepScore,
    dailyScore >= 0 ? dailyScore : 0.0f,
    dailyScoreDate,
    fanPWM, fanOn ? "true" : "false",
    motorPWM[0], motorPWM[1], motorPWM[2], motorPWM[3],
    ax, ay, az);

  server.send(200, "application/json", json);
}

// ─────────────────────────── MAIN PAGE ───────────────────────────
void handleRoot() {
  // Captive-portal style: full dashboard
  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Everest Smart Mattress</title>
<style>
  :root{--blue:#3b82f6;--red:#ef4444;--green:#22c55e;--card:#1e293b;
        --bg:#0f172a;--text:#f1f5f9;--sub:#94a3b8;--accent:#6366f1}
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;
       padding:12px;max-width:500px;margin:0 auto}
  h1{text-align:center;font-size:1.4em;color:var(--blue);margin:8px 0 2px}
  h4{text-align:center;color:var(--sub);font-size:.85em;margin-bottom:14px}
  .card{background:var(--card);border-radius:14px;padding:14px;margin:10px 0;
        box-shadow:0 4px 16px rgba(0,0,0,.4)}
  .card h3{color:var(--accent);font-size:1em;margin-bottom:10px;
            border-bottom:1px solid #334155;padding-bottom:6px}
  .stats{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .stat-item{background:#0f172a;border-radius:10px;padding:10px;text-align:center}
  .stat-val{font-size:1.6em;font-weight:700;color:var(--blue)}
  .stat-lbl{font-size:.72em;color:var(--sub);margin-top:2px}
  .score-bar-bg{background:#1e3a5f;border-radius:99px;height:12px;margin:8px 0}
  .score-bar{background:linear-gradient(90deg,var(--red),var(--green));
             height:12px;border-radius:99px;transition:width .5s}
  canvas{width:100%;height:160px;display:block;border-radius:8px;
         background:#0f172a;margin-top:8px}
  label{font-size:.82em;color:var(--sub)}
  input[type=range]{width:100%;accent-color:var(--blue);margin:6px 0}
  .btn{display:inline-block;padding:9px 18px;border:none;border-radius:9px;
       font-size:.9em;cursor:pointer;font-weight:600;transition:.15s}
  .btn-blue{background:var(--blue);color:#fff}
  .btn-red{background:var(--red);color:#fff}
  .btn-green{background:var(--green);color:#000}
  .btn-gray{background:#334155;color:#fff}
  .btn:active{opacity:.75;transform:scale(.97)}
  .btn-row{display:flex;gap:8px;flex-wrap:wrap;margin-top:8px}
  .alarm-list{min-height:24px}
  .alarm-tag{display:inline-flex;align-items:center;gap:6px;
             background:#1e3a5f;border-radius:8px;padding:5px 10px;
             margin:3px;font-size:.88em}
  .del-btn{background:none;border:none;color:var(--red);cursor:pointer;
           font-size:1em;line-height:1}
  .input-row{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap;align-items:center}
  input[type=number],input[type=text]{background:#0f172a;border:1px solid #334155;
    color:var(--text);border-radius:7px;padding:7px;width:65px;text-align:center;
    font-size:.9em}
  .tab-row{display:flex;gap:6px;margin-bottom:12px}
  .tab{flex:1;padding:8px;border:1px solid #334155;border-radius:8px;
       background:#1e293b;color:var(--sub);cursor:pointer;
       text-align:center;font-size:.85em;transition:.15s}
  .tab.active{background:var(--accent);color:#fff;border-color:var(--accent)}
  .page{display:none}.page.active{display:block}
  .daily-box{background:#0f172a;border-radius:10px;padding:12px;text-align:center;margin-top:8px}
  .daily-score{font-size:2.5em;font-weight:700;color:var(--green)}
  .motor-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .motor-card{background:#0f172a;border-radius:10px;padding:10px}
  .motor-card h4{color:var(--sub);font-size:.8em;margin-bottom:6px}
</style>
</head>
<body>
<h1>🛏 Everest Smart Mattress</h1>
<h4>Omar Samy</h4>

<div class="tab-row">
  <div class="tab active" onclick="showPage('dashboard')">Dashboard</div>
  <div class="tab" onclick="showPage('control')">Control</div>
  <div class="tab" onclick="showPage('alarms')">Alarms</div>
  <div class="tab" onclick="showPage('settings')">Settings</div>
</div>

<!-- ═══════════ DASHBOARD PAGE ═══════════ -->
<div id="page-dashboard" class="page active">

  <div class="card">
    <h3>📊 Live Readings</h3>
    <div class="stats">
      <div class="stat-item">
        <div class="stat-val" id="temp">--</div>
        <div class="stat-lbl">Temperature °C</div>
      </div>
      <div class="stat-item">
        <div class="stat-val" id="hum">--</div>
        <div class="stat-lbl">Humidity %</div>
      </div>
      <div class="stat-item">
        <div class="stat-val" id="time">--</div>
        <div class="stat-lbl">Time</div>
      </div>
      <div class="stat-item">
        <div class="stat-val" id="date" style="font-size:1em">--</div>
        <div class="stat-lbl">Date</div>
      </div>
    </div>
  </div>

  <div class="card">
    <h3>😴 Sleep Score</h3>
    <div style="font-size:2em;font-weight:700;text-align:center;color:var(--green)" id="sleepScore">--</div>
    <div class="score-bar-bg">
      <div class="score-bar" id="scoreBar" style="width:0%"></div>
    </div>
    <canvas id="scoreChart"></canvas>
  </div>

  <div class="card">
    <h3>📅 Daily Sleep Report</h3>
    <div class="daily-box">
      <div class="daily-score" id="dailyScore">--</div>
      <div style="color:var(--sub);font-size:.82em;margin-top:4px" id="dailyDate">No data yet</div>
    </div>
  </div>

</div>

<!-- ═══════════ CONTROL PAGE ═══════════ -->
<div id="page-control" class="page">

  <div class="card">
    <h3>⚡ Vibration Motors</h3>
    <div class="motor-grid" id="motorGrid">
    </div>
    <div class="btn-row" style="margin-top:12px">
      <button class="btn btn-red" onclick="activateBuzz()">⚡ Buzz Pattern</button>
    </div>
  </div>

  <div class="card">
    <h3>💨 Fan Control</h3>
    <label>Fan Speed — PWM: <span id="fanVal">0</span></label>
    <input type="range" min="0" max="255" value="0" id="fanSlider"
           oninput="setFan(this.value)">
    <div class="btn-row">
      <button class="btn btn-green" id="fanToggleBtn" onclick="toggleFan()">Fan ON (200)</button>
    </div>
  </div>

</div>

<!-- ═══════════ ALARMS PAGE ═══════════ -->
<div id="page-alarms" class="page">

  <div class="card">
    <h3>⏰ Alarms</h3>
    <div class="alarm-list" id="alarmList">Loading...</div>
    <div class="input-row" style="margin-top:12px">
      <input type="number" id="ah" min="0" max="23" placeholder="HH">
      <span style="color:var(--sub)">:</span>
      <input type="number" id="am" min="0" max="59" placeholder="MM">
      <button class="btn btn-blue" onclick="addAlarm()">Add</button>
    </div>
    <div class="btn-row" style="margin-top:10px">
      <button class="btn btn-gray" onclick="snooze()">💤 Snooze (10s)</button>
    </div>
  </div>

</div>

<!-- ═══════════ SETTINGS PAGE ═══════════ -->
<div id="page-settings" class="page">

  <div class="card">
    <h3>🕐 Set RTC Time</h3>
    <div class="input-row">
      <input type="number" id="th" min="0" max="23" placeholder="HH">
      <input type="number" id="tm" min="0" max="59" placeholder="MM">
      <input type="number" id="ts" min="0" max="59" placeholder="SS">
      <button class="btn btn-blue" onclick="setRTC()">Set</button>
    </div>
  </div>

  <div class="card">
    <h3>🔧 System</h3>
    <div class="btn-row">
      <button class="btn btn-gray" onclick="sleepNow()">💤 Deep Sleep</button>
    </div>
    <div style="margin-top:10px;font-size:.8em;color:var(--sub)" id="sysInfo">
      MPU: ax=<span id="ax">-</span> ay=<span id="ay">-</span> az=<span id="az">-</span>
    </div>
  </div>

</div>

<script>
// ── Tab navigation ──
function showPage(id){
  document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.getElementById('page-'+id).classList.add('active');
  event.target.classList.add('active');
}

// ── Motor controls ──
var motorLabels = ['V1','V2','V3','V4'];
var motorPWM    = [0,0,0,0];
function buildMotorGrid(){
  var g = document.getElementById('motorGrid');
  g.innerHTML = '';
  motorLabels.forEach(function(lbl,i){
    g.innerHTML +=
      '<div class="motor-card">'+
        '<h4>Motor '+lbl+'</h4>'+
        '<label>PWM: <span id="mval'+i+'">0</span></label>'+
        '<input type="range" min="0" max="255" value="0" '+
               'oninput="setMotorPWM('+i+',this.value)" id="ms'+i+'">'+
        '<button class="btn btn-blue" style="width:100%;margin-top:6px" '+
                'onclick="motorPulse('+i+')">▶ 5s Pulse</button>'+
      '</div>';
  });
}
buildMotorGrid();

function setMotorPWM(ch,val){
  motorPWM[ch]=parseInt(val);
  document.getElementById('mval'+ch).textContent=val;
  fetch('/setPWM?ch='+ch+'&v='+val);
}
function motorPulse(ch){fetch('/motorPulse?ch='+ch);}
function activateBuzz(){
  fetch('/buzz').then(()=>{
    var btn=event.target;
    btn.textContent='Buzzing...';btn.disabled=true;
    setTimeout(()=>{btn.textContent='⚡ Buzz Pattern';btn.disabled=false;},6200);
  });
}

// ── Fan ──
var fanIsOn = false;
function setFan(v){
  document.getElementById('fanVal').textContent=v;
  fetch('/setFan?v='+v);
}
function toggleFan(){
  fanIsOn=!fanIsOn;
  var pwm=fanIsOn?200:0;
  fetch('/fanToggle').then(()=>{
    document.getElementById('fanToggleBtn').textContent=
      fanIsOn?'Fan OFF':'Fan ON (200)';
    document.getElementById('fanSlider').value=pwm;
    document.getElementById('fanVal').textContent=pwm;
  });
}

// ── Alarms ──
function loadAlarms(){
  fetch('/alarms').then(r=>r.json()).then(function(arr){
    var list=document.getElementById('alarmList');
    if(arr.length===0){list.innerHTML='<span style="color:var(--sub)">No alarms set</span>';return;}
    list.innerHTML='';
    arr.forEach(function(a){
      var tag=document.createElement('span');
      tag.className='alarm-tag';
      tag.innerHTML=(a.h<10?'0':'')+a.h+':'+(a.m<10?'0':'')+a.m+
        ' <button class="del-btn" onclick="deleteAlarm('+a.i+')">✕</button>';
      list.appendChild(tag);
    });
  });
}
loadAlarms();

function addAlarm(){
  var h=document.getElementById('ah').value;
  var m=document.getElementById('am').value;
  if(h===''||m===''){alert('Enter hour and minute');return;}
  fetch('/addAlarm?h='+h+'&m='+m).then(()=>{
    document.getElementById('ah').value='';
    document.getElementById('am').value='';
    loadAlarms();
  });
}
function deleteAlarm(i){fetch('/deleteAlarm?i='+i).then(loadAlarms);}
function snooze(){fetch('/snooze').then(()=>alert('Snoozed 10 seconds'));}

// ── RTC ──
function setRTC(){
  var h=document.getElementById('th').value;
  var m=document.getElementById('tm').value;
  var s=document.getElementById('ts').value||'0';
  fetch('/setTime?h='+h+'&m='+m+'&s='+s).then(()=>alert('Time updated!'));
}

// ── Deep Sleep ──
function sleepNow(){
  if(confirm('Send ESP32 to deep sleep?'))
    fetch('/sleep').then(()=>alert('System sleeping. Reconnect to wake.'));
}

// ── Score chart (canvas) ──
var scoreData = [];
function drawChart(){
  var canvas=document.getElementById('scoreChart');
  if(!canvas) return;
  var ctx=canvas.getContext('2d');
  var W=canvas.width,H=canvas.height;
  ctx.clearRect(0,0,W,H);
  if(scoreData.length<2) return;
  // grid
  ctx.strokeStyle='#1e3a5f';ctx.lineWidth=1;
  [25,50,75,100].forEach(function(v){
    var y=H-v/100*H;
    ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();
  });
  // red line (below 50) + blue line (above 50)
  var pts=scoreData;
  var step=W/(pts.length-1);
  // Draw as gradient line
  ctx.lineWidth=2;
  for(var i=0;i<pts.length-1;i++){
    var v0=pts[i],v1=pts[i+1];
    var x0=i*step,x1=(i+1)*step;
    var y0=H-v0/100*H,y1=H-v1/100*H;
    var grad=ctx.createLinearGradient(x0,0,x1,0);
    var c0=scoreColor(v0),c1=scoreColor(v1);
    grad.addColorStop(0,c0);grad.addColorStop(1,c1);
    ctx.strokeStyle=grad;
    ctx.beginPath();ctx.moveTo(x0,y0);ctx.lineTo(x1,y1);ctx.stroke();
  }
}
function scoreColor(v){
  // blue at 100, red at 0
  var r=Math.round(239-(239-59)*(v/100));
  var g=Math.round(68+(197-68)*(v/100));
  var b=Math.round(68+(246-68)*(v/100));
  return'rgb('+r+','+g+','+b+')';
}

// ── Live data polling ──
setInterval(function(){
  fetch('/data').then(r=>r.json()).then(function(d){
    document.getElementById('temp').textContent   = d.temp;
    document.getElementById('hum').textContent    = d.hum;
    document.getElementById('time').textContent   = d.time;
    document.getElementById('date').textContent   = d.date;
    document.getElementById('sleepScore').textContent = d.sleep.toFixed(1);
    document.getElementById('scoreBar').style.width  = d.sleep+'%';
    if(d.dailyScore>0){
      document.getElementById('dailyScore').textContent = d.dailyScore.toFixed(1);
      document.getElementById('dailyDate').textContent  = d.dailyDate;
    }
    // MPU
    document.getElementById('ax').textContent=d.ax;
    document.getElementById('ay').textContent=d.ay;
    document.getElementById('az').textContent=d.az;
    // sync fan slider
    document.getElementById('fanSlider').value=d.fanPWM;
    document.getElementById('fanVal').textContent=d.fanPWM;
    fanIsOn=d.fanOn;
    document.getElementById('fanToggleBtn').textContent=fanIsOn?'Fan OFF':'Fan ON (200)';
    // sync motor sliders
    d.motorPWM.forEach(function(v,i){
      var sl=document.getElementById('ms'+i);
      if(sl) sl.value=v;
      var vl=document.getElementById('mval'+i);
      if(vl) vl.textContent=v;
    });
  }).catch(()=>{});
  // score chart
  fetch('/scoreHistory').then(r=>r.json()).then(function(arr){
    scoreData=arr;
    drawChart();
  }).catch(()=>{});
},1000);

// Resize canvas to actual display size
window.addEventListener('load',function(){
  var c=document.getElementById('scoreChart');
  c.width=c.offsetWidth;c.height=160;
});
</script>
</body>
</html>
)rawhtml";

  server.send(200, "text/html", html);
}

// Legacy /control redirect
void handleControl() {
  server.sendHeader("Location", "/");
  server.send(303);
}
