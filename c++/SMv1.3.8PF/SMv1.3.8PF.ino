/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║         SMART MATTRESS — SMv1.3.8PF  (O Solutions Everest)      ║
 * ║                         Omar Samy                               ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  HARDWARE MAP                                                   ║
 * ║  Motor 1 (IRFZ44N gate) ── GPIO 32                             ║
 * ║  Motor 2 (IRFZ44N gate) ── GPIO 21                             ║
 * ║  Motor 3 (IRFZ44N gate) ── GPIO 4                              ║
 * ║  Motor 4 (IRFZ44N gate) ── GPIO 13                             ║
 * ║  Fan PWM (IRFZ44N gate) ── GPIO 23                             ║
 * ║  Relay Fan RIGHT        ── GPIO 15                             ║
 * ║  Relay Fan LEFT         ── GPIO 16                             ║
 * ║  Piezo Impact           ── GPIO 14  (EXT0 wake)               ║
 * ║  Piezo Vibration        ── GPIO 27  (ADC)                      ║
 * ║  DHT11                  ── GPIO 2                              ║
 * ║  RTC DS3231 SDA/SCL     ── GPIO 17 / 5   (Wire  bus 0)        ║
 * ║  RTC DS3231 SQW         ── GPIO 18                             ║
 * ║  OLED 0.96" SDA/SCL     ── GPIO 25 / 26  (Wire1 bus 1)        ║
 * ║  MPU6050  SDA/SCL/INT   ── GPIO 25 / 26 / 33  (Wire1 bus 1)  ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  I2C STRATEGY:                                                 ║
 * ║   Wire  (bus 0) → RTC only  SDA=17  SCL=5   (permanent)       ║
 * ║   Wire1 (bus 1) → OLED + MPU6050  SDA=25  SCL=26              ║
 * ║     OLED addr=0x3C  MPU addr=0x68  — no address conflict       ║
 * ║     No pin-swapping needed. Simple and reliable.               ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  REQUIRED LIBRARIES:                                           ║
 * ║   • Adafruit DHT sensor library                                ║
 * ║   • Adafruit RTClib                                            ║
 * ║   • Adafruit SSD1306 + Adafruit GFX                           ║
 * ║   • MPU6050_light  (rfetick)                                   ║
 * ║   • DNSServer (built-in ESP32 Arduino)                         ║
 * ║   • WiFi + WebServer (built-in ESP32 Arduino)                  ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

// ═══════════════════════════ INCLUDES ════════════════════════════
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>           // Captive portal
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <MPU6050_light.h>      // rfetick/MPU6050_light — simple & reliable
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <math.h>

// ═══════════════════════════ PIN DEFINITIONS ══════════════════════
#define DHTPIN          2
#define DHTTYPE         DHT11        // ← changed from DHT22

#define MOTOR_PIN_1     32
#define MOTOR_PIN_2     21
#define MOTOR_PIN_3     4
#define MOTOR_PIN_4     13
#define FAN_PWM_PIN     23           // PWM-controlled fan via MOSFET

#define RELAY_RIGHT     15           // GPIO15 — right ventilation fan (safe pin)
#define RELAY_LEFT      16           // GPIO16 — left  ventilation fan (safe pin)

#define PIEZO_IMPACT    14           // EXT0 wake source
#define PIEZO_VIB       27
#define RTC_SQW_PIN     18
#define MPU_INT_PIN     33

// I2C pins
#define RTC_SDA   17
#define RTC_SCL   5
#define OLED_SDA  25   // OLED rewired to share GPIO25/26 with MPU6050
#define OLED_SCL  26
#define MPU_SDA   25   // same physical bus as OLED — different I2C addresses
#define MPU_SCL   26

// OLED
#define SCREEN_W   128
#define SCREEN_H   64
#define OLED_ADDR  0x3C

// ═══════════════════════════ CONSTANTS ═══════════════════════════
#define MAX_ALARMS         9
#define SNOOZE_SEC         10
#define ALARM_VIB_SEC      30
#define FAN_ON_HOUR        12        // Noon scheduled fan task
#define SCORE_HISTORY      120       // 120 points (one per 30 s ≈ 1 h window)
#define PIEZO_THRESH       300       // ADC 12-bit threshold
#define AUTO_SLEEP_MIN     15        // Auto sleep after 15 min idle + no clients
#define OLED_PAGE_COUNT    10        // Total OLED screens available

// Relay polarity — most relay modules are active LOW
#define RELAY_ON   LOW
#define RELAY_OFF  HIGH

// ═══════════════════════════ I2C BUSES ════════════════════════════
// Wire  (bus 0) → RTC DS3231  only   SDA=17  SCL=5
// Wire1 (bus 1) → OLED 0x3C + MPU6050 0x68   SDA=25  SCL=26
// Different I2C addresses → zero conflict. No pin-swapping ever needed.
TwoWire I2C_RTC  = TwoWire(0);  // RTC  — permanent on GPIO17/5
TwoWire I2C_OLED = TwoWire(1);  // OLED + MPU shared — permanent on GPIO25/26

// ═══════════════════════════ PERIPHERALS ══════════════════════════
DHT              dht(DHTPIN, DHTTYPE);
RTC_DS3231       rtc;                                        // uses I2C_RTC
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &I2C_OLED, -1);  // Wire1 25/26
MPU6050          mpu(I2C_OLED);  // Wire1 25/26 — addr 0x68 vs OLED 0x3C

// ═══════════════════════════ NETWORK ══════════════════════════════
const char*  AP_SSID = "ESP32-omar";
const char*  AP_PASS = "om1414";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_MASK(255, 255, 255, 0);
WebServer    server(80);
DNSServer    dns;

// ═══════════════════════════ SENSOR STATE ═════════════════════════
float    temperature  = 0;
float    humidity     = 0;
float    mpuX = 0, mpuY = 0, mpuZ = 0;   // angles from MPU6050_light
float    mpuGX = 0, mpuGY = 0, mpuGZ = 0; // gyro rates
bool     mpuOK        = false;

// ═══════════════════════════ SLEEP SCORE ══════════════════════════
float    sleepScore        = 100.0;
float    scoreHistory[SCORE_HISTORY];
int      scoreHead         = 0;
int      scoreCount        = 0;
float    dailyScore        = -1;
char     dailyScoreDate[12] = "";
float    lastScoreForIdle  = 100.0;   // for auto-sleep idle detection
unsigned long lastScoreChange = 0;    // millis of last score change

// ═══════════════════════════ PERSON DETECTION ═════════════════════
bool     personDetected      = false; // true once first motion seen
bool     showPersonDetected  = false; // OLED flash flag
unsigned long personFlashEnd = 0;     // how long to show "person detected"
bool     scoreTrackingActive = false; // only score after person detected

// ═══════════════════════════ ALARMS ═══════════════════════════════
struct Alarm { int hour, minute; bool enabled, fired; };
Alarm    alarms[MAX_ALARMS];

// ═══════════════════════════ MOTORS / FAN ═════════════════════════
int      motorPWM[4]        = {0,0,0,0};  // slider set-point
int      currentMotorPWM[4] = {0,0,0,0};  // actual running PWM (for OLED)
bool     motorPulsing[4]    = {false,false,false,false};
unsigned long motorPulseEnd[4] = {0,0,0,0};
int      fanPWM         = 0;
bool     fanOn          = false;
bool     relayRight     = false;
bool     relayLeft      = false;

// ═══════════════════════════ BUZZ ════════════════════════════════
bool     patternActive    = false;
bool     buzzRunning      = false;   // true while buzz pattern is executing
bool     motorOledActive  = false;   // true when any motor is running (shows 4-square OLED)
unsigned long motorOledEnd = 0;      // millis when to clear motor overlay

// ═══════════════════════════ SNOOZE ══════════════════════════════
bool          snoozePending = false;
unsigned long snoozeEnd     = 0;

// ═══════════════════════════ OLED CONTROL ════════════════════════
/*
 * OLED screens (indices):
 *  0 = Next Alarm       5 = Fan PWM status
 *  1 = Sleep Score Graph 6 = Ventilation status
 *  2 = Temp & Humidity  7 = MPU Motion
 *  3 = Current Time     8 = System Status
 *  4 = Daily Score      9 = O Solutions Everest (brand)
 */
#define OLED_SCREEN_ALARM      0
#define OLED_SCREEN_SCORE      1
#define OLED_SCREEN_CLIMATE    2
#define OLED_SCREEN_TIME       3
#define OLED_SCREEN_DAILY      4
#define OLED_SCREEN_FAN        5
#define OLED_SCREEN_VENT       6
#define OLED_SCREEN_MPU        7
#define OLED_SCREEN_SYS        8
#define OLED_SCREEN_BRAND      9
#define OLED_SCREEN_MOTORS     10   // 4-square motor status overlay

// User-configurable sequence (up to 10 slots)
uint8_t  oledSequence[OLED_PAGE_COUNT] = {3,2,0,1,4,7,5,6,8,9};
uint16_t oledDuration[OLED_PAGE_COUNT] = {3,3,3,4,3,3,3,3,3,4}; // seconds per screen
uint8_t  oledSeqLen    = OLED_PAGE_COUNT;
uint8_t  oledSeqIdx    = 0;           // current position in sequence
uint8_t  oledDefault   = 3;           // default screen (time)
bool     oledForced    = false;       // true when web UI forced a screen
uint8_t  oledForcedScreen = 0;
unsigned long lastOledSwitch = 0;

// ═══════════════════════════ TIMING ══════════════════════════════
unsigned long lastSensorRead  = 0;
unsigned long lastScoreUpdate = 0;
unsigned long lastMpuUpdate   = 0;
unsigned long lastClientCheck = 0;

// ═══════════════════════════ RTC MEMORY (survives deep sleep) ═════
RTC_DATA_ATTR int   wakeCount  = 0;
RTC_DATA_ATTR bool  firstBoot  = true;

// ══════════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ══════════════════════════════════════════════════════════════════
void setupWiFi();
void setupRoutes();
void enterDeepSleep();
void startupSequence();
void welcomeProgram();
void runBuzzPattern();
void setMotor(int idx, int pwm);
void setFanPWM(int pwm);
void setRelayRight(bool on);
void setRelayLeft(bool on);
void checkPiezo();
void checkMPU();
void updateSleepScore(float delta);
void checkAlarms(const DateTime& now);
void updateOLED();
void oledShow();
DateTime rtcNow();
void drawOledPage(uint8_t screen);
void drawScoreGraph();
String buildScoreJSON();
String buildAlarmJSON();
String buildOledConfigJSON();
void handleRoot();
void handleData();
void handleControl();
void handleVentilation();
void handleOledConfig();

// ══════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════
void setup() {
  // Serial enabled — GPIO1/GPIO3 now free (relays moved to GPIO15/16)
  Serial.begin(115200);
  Serial.println("[BOOT] SMv1.3.8PF starting...");

  // ── Relay pins ──
  pinMode(RELAY_RIGHT, OUTPUT); digitalWrite(RELAY_RIGHT, RELAY_OFF);
  pinMode(RELAY_LEFT,  OUTPUT); digitalWrite(RELAY_LEFT,  RELAY_OFF);

  // ── I2C buses ──
  // Bus 0: RTC only on GPIO17/5
  I2C_RTC.begin(RTC_SDA, RTC_SCL, 100000);
  // Bus 1: OLED + MPU6050 on GPIO25/26
  I2C_OLED.begin(OLED_SDA, OLED_SCL, 400000);

  // ── RTC ──
  if (rtc.begin(&I2C_RTC)) {
    rtc.clearAlarm(1); rtc.clearAlarm(2);
    rtc.disableAlarm(1); rtc.disableAlarm(2);
    rtc.writeSqwPinMode(DS3231_OFF);
    // If RTC lost power (e.g. battery dead), it resets to 2000-01-01 00:00:00
    // We detect this and flag it — user must set time via web UI
    if (rtc.lostPower()) {
      Serial.println("[RTC] lost power — time not set. Use Settings to set date/time.");
      // Preset to a known placeholder so display shows something obvious
      rtc.adjust(DateTime(2024, 1, 1, 0, 0, 0));
    }
    Serial.println("[RTC] OK");
  } else {
    Serial.println("[RTC] not found");
  }

  // ── OLED ──
  if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.display();
    Serial.println("[OLED] OK");
  } else {
    Serial.println("[OLED] not found");
  }

  // ── MPU6050 — Wire1 (I2C_OLED) on GPIO25/26, addr 0x68 ──
  byte mpuErr = mpu.begin();
  if (mpuErr == 0) {
    mpuOK = true;
    mpu.calcOffsets(true, true);   // auto-calibrate gyro + accel (~2 s)
    // Motion-detection interrupt registers written via I2C_OLED (same bus)
    I2C_OLED.beginTransmission(0x68);
    I2C_OLED.write(0x1F); I2C_OLED.write(15);   // MOT_THR = 15 LSB
    I2C_OLED.endTransmission();
    I2C_OLED.beginTransmission(0x68);
    I2C_OLED.write(0x20); I2C_OLED.write(2);    // MOT_DUR = 2 ms
    I2C_OLED.endTransmission();
    I2C_OLED.beginTransmission(0x68);
    I2C_OLED.write(0x38); I2C_OLED.write(0x40); // INT_ENABLE: motion bit
    I2C_OLED.endTransmission();
    pinMode(MPU_INT_PIN, INPUT);
    Serial.println("[MPU] OK");
  } else {
    Serial.printf("[MPU] FAIL code=%d\n", mpuErr);
  }

  // ── DHT ──
  dht.begin();

  // ── Motor PWM ──
  ledcAttach(MOTOR_PIN_1, 5000, 8); ledcWrite(MOTOR_PIN_1, 0);
  ledcAttach(MOTOR_PIN_2, 5000, 8); ledcWrite(MOTOR_PIN_2, 0);
  ledcAttach(MOTOR_PIN_3, 5000, 8); ledcWrite(MOTOR_PIN_3, 0);
  ledcAttach(MOTOR_PIN_4, 5000, 8); ledcWrite(MOTOR_PIN_4, 0);
  ledcAttach(FAN_PWM_PIN, 5000, 8); ledcWrite(FAN_PWM_PIN, 0);

  // ── Score history init ──
  for (int i = 0; i < SCORE_HISTORY; i++) scoreHistory[i] = -1.0f;

  // ── Alarms init ──
  for (int i = 0; i < MAX_ALARMS; i++) alarms[i] = {0,0,false,false};

  // ── Wake-cause handling ──
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool doStartup = firstBoot;

  if (firstBoot) {
    firstBoot = false;
  } else {
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
      // Scheduled timer wake: fan-at-noon task only
      DateTime now = rtcNow();
      if (now.hour() == FAN_ON_HOUR && now.minute() < 2) {
        setFanPWM(200);
        delay(60000);
        setFanPWM(0);
        enterDeepSleep();
        return;
      }
    }
    doStartup = true;
  }
  wakeCount++;

  // ── WiFi + captive portal ──
  setupWiFi();

  // ── Web server routes ──
  setupRoutes();
  server.begin();

  // ── Startup sequence ──
  if (doStartup) startupSequence();

  lastScoreChange = millis();
}

// ══════════════════════════════════════════════════════════════════
//  RTC helper — swaps Wire1 to RTC pins, reads, restores OLED pins
// ══════════════════════════════════════════════════════════════════
// ══════════════════════════════════════════════════════════════════
//  RTC helper — RTC has its own dedicated bus (I2C_RTC / Wire0)
//  No pin-swapping needed. Simple direct read.
// ══════════════════════════════════════════════════════════════════
DateTime rtcNow() {
  return rtc.now();
}

// ══════════════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════════════
void loop() {
  dns.processNextRequest();
  server.handleClient();

  unsigned long ms  = millis();
  DateTime      now = rtcNow();

  // ── DHT11 every 3 s (DHT11 minimum sample rate is 1 s; 3 s is reliable) ──
  // EMA smoothing: new = 0.2*raw + 0.8*prev — eliminates fast spikes
  if (ms - lastSensorRead >= 3000) {
    lastSensorRead = ms;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && h >= 0 && h <= 100) {
      // First valid read — initialise directly; thereafter apply EMA
      humidity    = (humidity == 0) ? h : (0.2f * h + 0.8f * humidity);
    }
    if (!isnan(t) && t > -40 && t < 80) {
      temperature = (temperature == 0) ? t : (0.2f * t + 0.8f * temperature);
    }
  }

  // ── MPU6050 every 500 ms (rate-limited; pin-swap inside checkMPU) ──
  if (mpuOK && ms - lastMpuUpdate >= 500) {
    lastMpuUpdate = ms;
    checkMPU();
  }

  // ── Piezo ──
  checkPiezo();

  // ── Score push every 30 s (even without motion = no penalty) ──
  if (ms - lastScoreUpdate >= 30000) {
    lastScoreUpdate = ms;
    // Push current score to history
    scoreHistory[scoreHead] = sleepScore;
    scoreHead = (scoreHead + 1) % SCORE_HISTORY;
    if (scoreCount < SCORE_HISTORY) scoreCount++;

    // Daily save at 9 AM
    if (now.hour() == 9 && now.minute() == 0 && dailyScore < 0) {
      dailyScore = sleepScore;
      snprintf(dailyScoreDate, sizeof(dailyScoreDate),
               "%02d/%02d/%04d", now.day(), now.month(), now.year());
    }
  }

  // ── Alarms ──
  checkAlarms(now);

  // ── Snooze expiry ──
  if (snoozePending && ms >= snoozeEnd) {
    snoozePending = false;
    patternActive = true;
  }

  // ── Motor pulse timers ──
  for (int i = 0; i < 4; i++) {
    if (motorPulsing[i] && ms >= motorPulseEnd[i]) {
      motorPulsing[i]     = false;
      currentMotorPWM[i]  = motorPWM[i];  // restore to slider value
      setMotor(i, motorPWM[i]);
    }
  }

  // ── Buzz ──
  if (patternActive && !snoozePending) {
    patternActive = false;
    runBuzzPattern();
    for (int i = 0; i < 4; i++) setMotor(i, motorPWM[i]);
  }

  // ── Auto deep sleep: no score change + no WiFi clients for AUTO_SLEEP_MIN ──
  if (ms - lastClientCheck >= 30000) {
    lastClientCheck = ms;
    int clients = WiFi.softAPgetStationNum();
    bool scoreChanged = (fabsf(sleepScore - lastScoreForIdle) > 0.1f);
    if (scoreChanged) {
      lastScoreForIdle = sleepScore;
      lastScoreChange  = ms;
    }
    if (clients == 0 && (ms - lastScoreChange) > (unsigned long)AUTO_SLEEP_MIN * 60000UL) {
      enterDeepSleep();
    }
  }

  // ── Scheduled deep-sleep trigger at 23:00 ──
  if (now.hour() == 23 && now.minute() == 0 && !patternActive && !snoozePending) {
    enterDeepSleep();
  }

  // ── Clear person-detected flash ──
  if (showPersonDetected && ms >= personFlashEnd) {
    showPersonDetected = false;
  }

  // ── OLED page rotation ──
  if (!oledForced) {
    uint16_t dur = oledDuration[oledSeqIdx] * 1000UL;
    if (ms - lastOledSwitch >= dur) {
      lastOledSwitch = ms;
      oledSeqIdx = (oledSeqIdx + 1) % oledSeqLen;
    }
  }
  updateOLED();
}

// ══════════════════════════════════════════════════════════════════
//  WIFI + CAPTIVE PORTAL
// ══════════════════════════════════════════════════════════════════
void setupWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);
  // Always re-call softAP with credentials — fixes open-network bug
  WiFi.softAP(AP_SSID, AP_PASS);

  // DNS: redirect every domain → AP_IP so device opens the page
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", AP_IP);
}

// ══════════════════════════════════════════════════════════════════
//  DEEP SLEEP
// ══════════════════════════════════════════════════════════════════
void enterDeepSleep() {
  // Show sleep message on OLED
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(10, 20); oled.print("O Solutions Everest");
  oled.setCursor(25, 36); oled.print("Entering sleep...");
  oled.display();
  delay(1500);
  oled.clearDisplay(); oled.display();

  // All outputs off
  for (int i = 0; i < 4; i++) setMotor(i, 0);
  setFanPWM(0);
  setRelayRight(false);
  setRelayLeft(false);

  // Stop WiFi cleanly — re-init happens on wake via setup()
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  // Wake sources
  rtc_gpio_init(GPIO_NUM_14);
  rtc_gpio_set_direction(GPIO_NUM_14, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_en(GPIO_NUM_14);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14, 1);       // Piezo impact

  esp_sleep_enable_ext1_wakeup(
    (1ULL << MPU_INT_PIN),
    ESP_EXT1_WAKEUP_ANY_HIGH);                         // MPU motion

  esp_sleep_enable_timer_wakeup(3600ULL * 1000000ULL); // 1 h timer fallback

  esp_deep_sleep_start();
}

// ══════════════════════════════════════════════════════════════════
//  STARTUP SEQUENCE
// ══════════════════════════════════════════════════════════════════
void startupSequence() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(10,  8); oled.print("O Solutions Everest");
  oled.setCursor(18, 24); oled.print("Welcome to Everest");
  oled.setCursor(30, 40); oled.print("Initializing...");
  oled.display();
  delay(2500);
  welcomeProgram();
}

void welcomeProgram() {
  // Fan on; after 5 s, vibration motors do a 20-s sine sequence
  setFanPWM(200);
  delay(5000);

  unsigned long t0 = millis();
  while (millis() - t0 < 20000) {
    int elapsed = (int)(millis() - t0);
    int pwm = (int)(127.5f + 127.5f * sinf(elapsed * 0.005f));
    ledcWrite(MOTOR_PIN_1, pwm);
    ledcWrite(MOTOR_PIN_2, pwm);
    ledcWrite(MOTOR_PIN_3, pwm);
    ledcWrite(MOTOR_PIN_4, pwm);
    delay(20);
  }
  for (int i = 0; i < 4; i++) setMotor(i, 0);
  delay(5000);    // Fan runs total ~30 s
  setFanPWM(0);
  for (int i = 0; i < 4; i++) setMotor(i, motorPWM[i]);
}

// ══════════════════════════════════════════════════════════════════
//  BUZZ PATTERN
// ══════════════════════════════════════════════════════════════════
void runBuzzPattern() {
  buzzRunning = true;
  motorOledActive = true;
  motorOledEnd    = millis() + 7000;  // keep overlay for full buzz duration

  for (int p = 0; p < 20; p++) {
    // Phase A: motors 1 & 3 ON
    currentMotorPWM[0] = 200; currentMotorPWM[2] = 200;
    currentMotorPWM[1] = 0;   currentMotorPWM[3] = 0;
    ledcWrite(MOTOR_PIN_1, 200); ledcWrite(MOTOR_PIN_3, 200);
    ledcWrite(MOTOR_PIN_2, 0);   ledcWrite(MOTOR_PIN_4, 0);
    delay(200);
    // Phase B: motors 2 & 4 ON
    currentMotorPWM[0] = 0;   currentMotorPWM[2] = 0;
    currentMotorPWM[1] = 200; currentMotorPWM[3] = 200;
    ledcWrite(MOTOR_PIN_1, 0);   ledcWrite(MOTOR_PIN_3, 0);
    ledcWrite(MOTOR_PIN_2, 200); ledcWrite(MOTOR_PIN_4, 200);
    delay(100);
  }
  // All off
  for (int i = 0; i < 4; i++) currentMotorPWM[i] = 0;
  ledcWrite(MOTOR_PIN_1, 0); ledcWrite(MOTOR_PIN_2, 0);
  ledcWrite(MOTOR_PIN_3, 0); ledcWrite(MOTOR_PIN_4, 0);
  buzzRunning = false;
}

// ══════════════════════════════════════════════════════════════════
//  HELPERS — motors, fan, relays
// ══════════════════════════════════════════════════════════════════
void setMotor(int idx, int pwm) {
  const int pins[] = {MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4};
  if (idx < 0 || idx > 3) return;
  int val = constrain(pwm, 0, 255);
  currentMotorPWM[idx] = val;
  ledcWrite(pins[idx], val);
  // Show motor OLED overlay for 3 s whenever any motor is active
  bool anyOn = false;
  for (int i = 0; i < 4; i++) if (currentMotorPWM[i] > 0) anyOn = true;
  if (anyOn) {
    motorOledActive = true;
    motorOledEnd    = millis() + 3000;
  }
}
void setFanPWM(int pwm) {
  fanPWM = constrain(pwm, 0, 255);
  fanOn  = (fanPWM > 0);
  ledcWrite(FAN_PWM_PIN, fanPWM);
}
void setRelayRight(bool on) {
  relayRight = on;
  digitalWrite(RELAY_RIGHT, on ? RELAY_ON : RELAY_OFF);
}
void setRelayLeft(bool on) {
  relayLeft = on;
  digitalWrite(RELAY_LEFT, on ? RELAY_ON : RELAY_OFF);
}

// ══════════════════════════════════════════════════════════════════
//  PIEZO CHECK
// ══════════════════════════════════════════════════════════════════
void checkPiezo() {
  static unsigned long lastPiezoHit = 0;
  if (millis() - lastPiezoHit < 200) return;   // debounce 200 ms

  int impact = analogRead(PIEZO_IMPACT);
  int vib    = analogRead(PIEZO_VIB);

  if (impact > PIEZO_THRESH || vib > PIEZO_THRESH) {
    lastPiezoHit = millis();
    // Person detection on first event
    if (!personDetected) {
      personDetected     = true;
      scoreTrackingActive= true;
      showPersonDetected = true;
      personFlashEnd     = millis() + 4000;
    }
    if (scoreTrackingActive) updateSleepScore(-0.5f);  // deduct 0.5 pt per hit
  }
}

// ══════════════════════════════════════════════════════════════════
//  MPU6050 CHECK  (called every 500 ms — rate-limited for stability)
// ══════════════════════════════════════════════════════════════════
//  MPU6050 CHECK  (called every 500 ms)
//  I2C_OLED (Wire1/GPIO25/26) is shared with OLED — no pin swap.
// ══════════════════════════════════════════════════════════════════
void checkMPU() {
  if (!mpuOK) return;

  mpu.update();
  float newX = mpu.getAngleX();
  float newY = mpu.getAngleY();
  float newZ = mpu.getAngleZ();
  float gx_  = mpu.getGyroX();
  float gy_  = mpu.getGyroY();
  float gz_  = mpu.getGyroZ();

  float dX = fabsf(newX - mpuX);
  float dY = fabsf(newY - mpuY);
  float dZ = fabsf(newZ - mpuZ);
  mpuX  = newX;  mpuY  = newY;  mpuZ  = newZ;
  mpuGX = gx_;   mpuGY = gy_;   mpuGZ = gz_;

  float totalDelta = dX + dY + dZ;

  if (totalDelta > 5.0f) {
    if (!personDetected) {
      personDetected      = true;
      scoreTrackingActive = true;
      showPersonDetected  = true;
      personFlashEnd      = millis() + 4000;
      Serial.println("[MPU] Person detected");
    }
    if (scoreTrackingActive) {
      float penalty = constrain(totalDelta * 0.001f, 0.0f, 0.3f);
      updateSleepScore(-penalty);
      Serial.printf("[MPU] delta=%.1f penalty=%.3f score=%.1f\n",
                    totalDelta, penalty, sleepScore);
    }
  } else if (scoreTrackingActive && totalDelta < 1.0f) {
    updateSleepScore(0.05f);   // slow recovery during stillness
  }
}

// ══════════════════════════════════════════════════════════════════
//  OLED display() wrapper
// ══════════════════════════════════════════════════════════════════
void oledShow() {
  oled.display();
}

// ══════════════════════════════════════════════════════════════════
//  SLEEP SCORE UPDATE
//  delta > 0 → recover (gain points)
//  delta < 0 → penalise (lose points)
//  Callers pass the raw signed delta; piezo passes negative values.
// ══════════════════════════════════════════════════════════════════
void updateSleepScore(float delta) {
  float prev    = sleepScore;
  sleepScore    = constrain(sleepScore + delta, 0.0f, 100.0f);
  if (fabsf(sleepScore - prev) > 0.01f) {
    lastScoreChange = millis();
    scoreHistory[scoreHead] = sleepScore;
    scoreHead = (scoreHead + 1) % SCORE_HISTORY;
    if (scoreCount < SCORE_HISTORY) scoreCount++;
  }
}

// ══════════════════════════════════════════════════════════════════
//  ALARM CHECK
// ══════════════════════════════════════════════════════════════════
void checkAlarms(const DateTime& now) {
  for (int i = 0; i < MAX_ALARMS; i++) {
    if (!alarms[i].enabled) continue;
    bool match = (now.hour() == alarms[i].hour &&
                  now.minute() == alarms[i].minute);
    if (match && !alarms[i].fired) {
      alarms[i].fired = true;
      patternActive   = true;
    }
    if (!match) alarms[i].fired = false;
  }
}

// ══════════════════════════════════════════════════════════════════
//  OLED  — top-level router
// ══════════════════════════════════════════════════════════════════
void updateOLED() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // ── Clear motor overlay when timer expired ──
  if (motorOledActive && millis() >= motorOledEnd) {
    bool anyOn = false;
    for (int i = 0; i < 4; i++) if (currentMotorPWM[i] > 0) anyOn = true;
    if (!anyOn) motorOledActive = false;
    else motorOledEnd = millis() + 1000; // extend while motors still running
  }

  // ── Priority 1: Person detected flash (4 s) ──
  if (showPersonDetected) {
    oled.setTextSize(1);
    oled.setCursor(14, 10); oled.print("* PERSON DETECTED *");
    oled.drawLine(0, 20, 127, 20, SSD1306_WHITE);
    oled.setCursor(10, 28); oled.print("Sleep tracking started");
    oled.setCursor(20, 42); oled.print("Score: ");
    oled.setTextSize(2);
    oled.setCursor(65, 38);
    char buf[8]; snprintf(buf,8,"%.0f",sleepScore);
    oled.print(buf);
    oledShow(); return;
  }

  // ── Priority 2: Motor overlay (manual or buzz) ──
  if (motorOledActive) {
    drawOledPage(OLED_SCREEN_MOTORS);
    oledShow(); return;
  }

  // ── Priority 3: Fan/vent overlay when any fan is on ──
  if ((relayRight || relayLeft || (fanPWM > 0 && fanOn)) && !buzzRunning) {
    drawOledPage(OLED_SCREEN_FAN);
    oledShow(); return;
  }

  // ── Normal rotating sequence ──
  uint8_t screen = oledForced
                   ? oledForcedScreen
                   : oledSequence[oledSeqIdx % oledSeqLen];
  drawOledPage(screen);
  oledShow();
}

// ══════════════════════════════════════════════════════════════════
//  OLED  — individual screen renderers
// ══════════════════════════════════════════════════════════════════
void drawOledPage(uint8_t screen) {
  DateTime now = rtcNow();

  switch (screen) {

    // ─── 0: Next Alarm ───────────────────────────────────────────
    case OLED_SCREEN_ALARM: {
      oled.setTextSize(1);
      oled.setCursor(28, 0); oled.print("Next Alarm");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      bool found = false;
      for (int i = 0; i < MAX_ALARMS; i++) {
        if (alarms[i].enabled) {
          oled.setTextSize(2);
          oled.setCursor(22, 22);
          char buf[8]; snprintf(buf,8,"%02d:%02d",alarms[i].hour,alarms[i].minute);
          oled.print(buf);
          found = true; break;
        }
      }
      if (!found) {
        oled.setTextSize(1);
        oled.setCursor(12, 28); oled.print("No alarm is set");
      }
      break;
    }

    // ─── 1: Sleep Score Graph ─────────────────────────────────────
    case OLED_SCREEN_SCORE: {
      oled.setTextSize(1);
      oled.setCursor(22, 0); oled.print("Sleep Score");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      drawScoreGraph();
      // Score label bottom-right
      oled.setTextSize(1);
      oled.setCursor(90, 56);
      char buf[8]; snprintf(buf,8,"%.0f/100",sleepScore);
      oled.print(buf);
      break;
    }

    // ─── 2: Temp & Humidity ───────────────────────────────────────
    case OLED_SCREEN_CLIMATE: {
      oled.setTextSize(1);
      oled.setCursor(15, 0); oled.print("Temp & Humidity");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      oled.setTextSize(2);
      oled.setCursor(4, 16);
      char tb[12]; snprintf(tb,12,"%.1f C", temperature);
      oled.print(tb);
      oled.setCursor(4, 40);
      char hb[12]; snprintf(hb,12,"%.1f %%", humidity);
      oled.print(hb);
      break;
    }

    // ─── 3: Current Time ──────────────────────────────────────────
    case OLED_SCREEN_TIME: {
      oled.setTextSize(1);
      oled.setCursor(40, 0); oled.print("Time");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      oled.setTextSize(3);
      oled.setCursor(4, 18);
      char tb[8]; snprintf(tb,8,"%02d:%02d",now.hour(),now.minute());
      oled.print(tb);
      oled.setTextSize(1);
      oled.setCursor(22, 56);
      char db[12]; snprintf(db,12,"%02d/%02d/%04d",now.day(),now.month(),now.year());
      oled.print(db);
      break;
    }

    // ─── 4: Daily Score ───────────────────────────────────────────
    case OLED_SCREEN_DAILY: {
      oled.setTextSize(1);
      oled.setCursor(20, 0); oled.print("Daily Report");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      if (dailyScore >= 0) {
        oled.setTextSize(3);
        oled.setCursor(30, 18);
        char buf[8]; snprintf(buf,8,"%.0f",dailyScore);
        oled.print(buf);
        oled.setTextSize(1);
        oled.setCursor(22, 52); oled.print(dailyScoreDate);
      } else {
        oled.setTextSize(1);
        oled.setCursor(14, 28); oled.print("No data yet");
        oled.setCursor(8,  42); oled.print("Saves at 09:00");
      }
      break;
    }

    // ─── 5: Fan PWM + Ventilation status ─────────────────────────
    case OLED_SCREEN_FAN: {
      oled.setTextSize(1);
      oled.setCursor(22, 0); oled.print("Fan Status");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      // PWM fan
      oled.setCursor(0, 14);
      char pw[24]; snprintf(pw,24,"PWM Fan: %s (%d)",fanOn?"ON ":"OFF",fanPWM);
      oled.print(pw);
      // PWM bar
      int barW = map(fanPWM, 0, 255, 0, 120);
      oled.drawRect(0, 24, 122, 8, SSD1306_WHITE);
      if (barW > 0) oled.fillRect(1, 25, barW, 6, SSD1306_WHITE);
      // Relay fans
      oled.setCursor(0, 36);
      oled.print(relayRight ? "R-Vent: [ ON  ]" : "R-Vent: [ OFF ]");
      oled.setCursor(0, 48);
      oled.print(relayLeft  ? "L-Vent: [ ON  ]" : "L-Vent: [ OFF ]");
      break;
    }

    // ─── 6: Ventilation squares ───────────────────────────────────
    case OLED_SCREEN_VENT: {
      oled.setTextSize(1);
      oled.setCursor(20, 0); oled.print("Ventilation");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      // Right square
      if (relayRight) oled.fillRoundRect(2, 14, 58, 48, 4, SSD1306_WHITE);
      else            oled.drawRoundRect(2, 14, 58, 48, 4, SSD1306_WHITE);
      oled.setTextColor(relayRight ? SSD1306_BLACK : SSD1306_WHITE);
      oled.setTextSize(1);
      oled.setCursor(10, 24); oled.print("R-Side");
      oled.setCursor(10, 36); oled.print("  Fan ");
      oled.setCursor(10, 48); oled.print(relayRight ? "  ON  " : " OFF  ");
      // Left square
      if (relayLeft) oled.fillRoundRect(68, 14, 58, 48, 4, SSD1306_WHITE);
      else           oled.drawRoundRect(68, 14, 58, 48, 4, SSD1306_WHITE);
      oled.setTextColor(relayLeft ? SSD1306_BLACK : SSD1306_WHITE);
      oled.setCursor(76, 24); oled.print("L-Side");
      oled.setCursor(76, 36); oled.print("  Fan ");
      oled.setCursor(76, 48); oled.print(relayLeft ? "  ON  " : " OFF  ");
      oled.setTextColor(SSD1306_WHITE);
      break;
    }

    // ─── 7: MPU Motion ────────────────────────────────────────────
    case OLED_SCREEN_MPU: {
      oled.setTextSize(1);
      if (!mpuOK) {
        oled.setCursor(14, 26); oled.print("MPU6050 not found");
        break;
      }
      oled.setCursor(30, 0); oled.print("Motion");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

      // Three bars for X, Y, Z angles (−180° to +180°)
      const char* labels[] = {"X","Y","Z"};
      float vals[]   = {mpuX, mpuY, mpuZ};
      for (int i = 0; i < 3; i++) {
        int y0 = 14 + i * 16;
        oled.setCursor(0, y0);
        oled.print(labels[i]);
        // Map -180..180 → 0..110
        int barFull = 110;
        int barX    = 14;
        oled.drawRect(barX, y0, barFull, 8, SSD1306_WHITE);
        int fill = (int)((vals[i] + 180.0f) / 360.0f * (barFull - 2));
        fill = constrain(fill, 0, barFull - 2);
        if (fill > 0) oled.fillRect(barX + 1, y0 + 1, fill, 6, SSD1306_WHITE);
        // Value label
        char vb[8]; snprintf(vb,8,"%4.0f",vals[i]);
        oled.setCursor(126 - 24, y0);
        oled.print(vb);
      }
      // Gyro rates on last line
      oled.setCursor(0, 58);
      char gb[32];
      snprintf(gb,32,"G %4.0f %4.0f %4.0f",mpuGX,mpuGY,mpuGZ);
      oled.print(gb);
      break;
    }

    // ─── 8: System Status ─────────────────────────────────────────
    case OLED_SCREEN_SYS: {
      oled.setTextSize(1);
      oled.setCursor(18, 0); oled.print("System Status");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      oled.setCursor(0, 13); oled.print("Status : Stable");
      oled.setCursor(0, 23);
      char wb[24]; snprintf(wb,24,"WiFi   : %s",AP_SSID);
      oled.print(wb);
      oled.setCursor(0, 33);
      char cb[24]; snprintf(cb,24,"Clients: %d",WiFi.softAPgetStationNum());
      oled.print(cb);
      oled.setCursor(0, 43);
      char wk[20]; snprintf(wk,20,"Wakes  : %d",wakeCount);
      oled.print(wk);
      oled.setCursor(0, 53);
      char sc[24]; snprintf(sc,24,"Score  : %.0f/100",sleepScore);
      oled.print(sc);
      break;
    }

    // ─── 9: Brand screen ──────────────────────────────────────────
    case OLED_SCREEN_BRAND: {
      oled.setTextSize(1);
      oled.drawRoundRect(0, 0, 128, 64, 6, SSD1306_WHITE);
      oled.setCursor(10, 12); oled.print("O Solutions Everest");
      oled.drawLine(8, 24, 119, 24, SSD1306_WHITE);
      oled.setTextSize(1);
      oled.setCursor(22, 30); oled.print("Smart Mattress");
      oled.setCursor(34, 42); oled.print("SMv1.3.8PF");
      oled.setCursor(30, 54); oled.print("Omar Samy");
      break;
    }

    // ─── 10: Motor 4-square overlay ───────────────────────────────
    case OLED_SCREEN_MOTORS: {
      oled.setTextSize(1);
      oled.setCursor(22, 0);
      oled.print(buzzRunning ? "Buzz Pattern" : "Motor Status");
      oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

      // 4 squares: 2×2 grid, each 58×24 px
      // Layout: [V1][V2]
      //         [V3][V4]
      const char* labels[] = {"V1","V2","V3","V4"};
      int sx[] = {0, 65, 0, 65};
      int sy[] = {13, 13, 38, 38};

      for (int i = 0; i < 4; i++) {
        bool on = (currentMotorPWM[i] > 0);
        if (on) {
          oled.fillRoundRect(sx[i], sy[i], 60, 24, 3, SSD1306_WHITE);
          oled.setTextColor(SSD1306_BLACK);
        } else {
          oled.drawRoundRect(sx[i], sy[i], 60, 24, 3, SSD1306_WHITE);
          oled.setTextColor(SSD1306_WHITE);
        }
        // Label centred in square
        oled.setTextSize(1);
        oled.setCursor(sx[i] + 4, sy[i] + 4); oled.print(labels[i]);
        // PWM value or OFF
        char pb[8];
        if (on) snprintf(pb, 8, "P:%d", currentMotorPWM[i]);
        else    snprintf(pb, 8, "OFF");
        oled.setCursor(sx[i] + 4, sy[i] + 13); oled.print(pb);
        oled.setTextColor(SSD1306_WHITE);
      }
      break;
    }

    default: break;
  }  // end switch
}    // end drawOledPage

// ══════════════════════════════════════════════════════════════════
//  OLED  — score graph helper
// ══════════════════════════════════════════════════════════════════
void drawScoreGraph() {
  const int gx = 0, gy = 12, gw = 128, gh = 50;
  int pts = min(scoreCount, gw);
  if (pts < 2) {
    oled.setTextSize(1);
    oled.setCursor(20, 36); oled.print("No data yet");
    return;
  }
  for (int i = 0; i < pts - 1; i++) {
    int i0  = (scoreHead - pts + i     + SCORE_HISTORY) % SCORE_HISTORY;
    int i1  = (scoreHead - pts + i + 1 + SCORE_HISTORY) % SCORE_HISTORY;
    float v0 = scoreHistory[i0];
    float v1 = scoreHistory[i1];
    if (v0 < 0 || v1 < 0) continue;
    int x0 = gx + i     * gw / pts;
    int x1 = gx + (i+1) * gw / pts;
    int y0 = gy + gh - (int)(v0 / 100.0f * gh);
    int y1 = gy + gh - (int)(v1 / 100.0f * gh);
    oled.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
  }
}

// ══════════════════════════════════════════════════════════════════
//  JSON HELPERS
// ══════════════════════════════════════════════════════════════════
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
  return j + "]";
}

String buildAlarmJSON() {
  String j = "["; bool first = true;
  for (int i = 0; i < MAX_ALARMS; i++) {
    if (!alarms[i].enabled) continue;
    if (!first) j += ",";
    j += "{\"i\":" + String(i) +
         ",\"h\":" + String(alarms[i].hour) +
         ",\"m\":" + String(alarms[i].minute) + "}";
    first = false;
  }
  return j + "]";
}

String buildOledConfigJSON() {
  String j = "{\"len\":" + String(oledSeqLen) + ",\"default\":" + String(oledDefault);
  j += ",\"seq\":[";
  for (int i = 0; i < oledSeqLen; i++) {
    if (i) j += ",";
    j += "{\"s\":" + String(oledSequence[i]) + ",\"d\":" + String(oledDuration[i]) + "}";
  }
  j += "]}";
  return j;
}

// ══════════════════════════════════════════════════════════════════
//  DATA ENDPOINT
// ══════════════════════════════════════════════════════════════════
void handleData() {
  DateTime now = rtcNow();
  char timeBuf[9], dateBuf[11];
  snprintf(timeBuf,9,  "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  snprintf(dateBuf,11, "%02d/%02d/%04d", now.day(),  now.month(),  now.year());

  // Build JSON in segments to avoid one huge snprintf
  String j = "{";
  char tmp[64];
  snprintf(tmp,64,"\"temp\":%.1f,\"hum\":%.1f,",temperature,humidity); j+=tmp;
  j += "\"time\":\""; j += timeBuf; j += "\",";
  j += "\"date\":\""; j += dateBuf; j += "\",";
  snprintf(tmp,64,"\"sleep\":%.1f,",sleepScore); j+=tmp;
  snprintf(tmp,64,"\"dailyScore\":%.1f,",dailyScore>=0?dailyScore:0.0f); j+=tmp;
  j += "\"dailyDate\":\""; j += dailyScoreDate; j += "\",";
  snprintf(tmp,64,"\"fanPWM\":%d,\"fanOn\":%s,",fanPWM,fanOn?"true":"false"); j+=tmp;
  snprintf(tmp,64,"\"relayR\":%s,\"relayL\":%s,",
           relayRight?"true":"false", relayLeft?"true":"false"); j+=tmp;
  snprintf(tmp,64,"\"motorPWM\":[%d,%d,%d,%d],",
           motorPWM[0],motorPWM[1],motorPWM[2],motorPWM[3]); j+=tmp;
  snprintf(tmp,64,"\"mpuX\":%.1f,\"mpuY\":%.1f,\"mpuZ\":%.1f,",mpuX,mpuY,mpuZ); j+=tmp;
  snprintf(tmp,64,"\"mpuGX\":%.1f,\"mpuGY\":%.1f,\"mpuGZ\":%.1f,",mpuGX,mpuGY,mpuGZ); j+=tmp;
  snprintf(tmp,32,"\"mpuOK\":%s,",mpuOK?"true":"false"); j+=tmp;
  snprintf(tmp,32,"\"person\":%s",personDetected?"true":"false"); j+=tmp;
  j += "}";

  server.send(200, "application/json", j);
}

// ══════════════════════════════════════════════════════════════════
//  WEB SERVER ROUTES
// ══════════════════════════════════════════════════════════════════
void setupRoutes() {

  // Captive portal catch-all
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  });

  server.on("/",            HTTP_GET, handleRoot);
  server.on("/data",        HTTP_GET, handleData);
  server.on("/scoreHistory",HTTP_GET, [](){server.send(200,"application/json",buildScoreJSON());});
  server.on("/alarms",      HTTP_GET, [](){server.send(200,"application/json",buildAlarmJSON());});
  server.on("/oledConfig",  HTTP_GET, [](){server.send(200,"application/json",buildOledConfigJSON());});

  server.on("/setPWM", HTTP_GET, [](){
    if (server.hasArg("ch") && server.hasArg("v")) {
      int ch = constrain(server.arg("ch").toInt(), 0, 3);
      int v  = constrain(server.arg("v").toInt(),  0, 255);
      motorPWM[ch] = v;
      currentMotorPWM[ch] = v;
      setMotor(ch, v);   // setMotor also triggers OLED overlay if v>0
    }
    server.send(200,"text/plain","OK");
  });

  server.on("/motorPulse", HTTP_GET, [](){
    if (server.hasArg("ch")) {
      int ch = constrain(server.arg("ch").toInt(), 0, 3);
      motorPulsing[ch]  = true;
      motorPulseEnd[ch] = millis() + 5000;
      currentMotorPWM[ch] = 200;
      setMotor(ch, 200);   // setMotor also sets motorOledActive
    }
    server.send(200,"text/plain","OK");
  });

  server.on("/setFan", HTTP_GET, [](){
    if (server.hasArg("v")) setFanPWM(constrain(server.arg("v").toInt(),0,255));
    server.send(200,"text/plain","OK");
  });

  server.on("/fanToggle", HTTP_GET, [](){
    setFanPWM(fanOn ? 0 : 200);
    server.send(200,"text/plain", fanOn?"on":"off");
  });

  server.on("/buzz", HTTP_GET, [](){
    patternActive = true;
    server.send(200,"text/plain","buzz");
  });

  server.on("/addAlarm", HTTP_GET, [](){
    if (server.hasArg("h") && server.hasArg("m")) {
      for (int i = 0; i < MAX_ALARMS; i++) {
        if (!alarms[i].enabled) {
          alarms[i] = {server.arg("h").toInt(), server.arg("m").toInt(), true, false};
          break;
        }
      }
    }
    server.send(200,"text/plain","OK");
  });

  server.on("/deleteAlarm", HTTP_GET, [](){
    if (server.hasArg("i")) {
      int idx = server.arg("i").toInt();
      if (idx >= 0 && idx < MAX_ALARMS) alarms[idx] = {0,0,false,false};
    }
    server.send(200,"text/plain","OK");
  });

  server.on("/snooze", HTTP_GET, [](){
    snoozePending = true;
    snoozeEnd     = millis() + SNOOZE_SEC * 1000UL;
    server.send(200,"text/plain","snoozed");
  });

  server.on("/setTime", HTTP_GET, [](){
    if (server.hasArg("h") && server.hasArg("m") && server.hasArg("s")) {
      DateTime n = rtcNow();
      int yr = server.hasArg("yr") ? server.arg("yr").toInt() : n.year();
      int mo = server.hasArg("mo") ? server.arg("mo").toInt() : n.month();
      int dy = server.hasArg("dy") ? server.arg("dy").toInt() : n.day();
      int hh = server.arg("h").toInt();
      int mm = server.arg("m").toInt();
      int ss = server.arg("s").toInt();
      rtc.adjust(DateTime(yr, mo, dy, hh, mm, ss));
      Serial.printf("[RTC] Set to %04d-%02d-%02d %02d:%02d:%02d\n",
                    yr, mo, dy, hh, mm, ss);
    }
    server.send(200,"text/plain","OK");
  });

  server.on("/sleep", HTTP_GET, [](){
    server.send(200,"text/plain","sleeping");
    delay(400);
    enterDeepSleep();
  });

  // Relay ventilation
  server.on("/setRelay", HTTP_GET, [](){
    if (server.hasArg("side") && server.hasArg("v")) {
      bool on = server.arg("v") == "1";
      if (server.arg("side") == "R") setRelayRight(on);
      else if (server.arg("side") == "L") setRelayLeft(on);
      else if (server.arg("side") == "both") { setRelayRight(on); setRelayLeft(on); }
    }
    server.send(200,"text/plain","OK");
  });

  // OLED control
  server.on("/setOledScreen", HTTP_GET, [](){
    if (server.hasArg("s")) {
      oledForcedScreen = constrain(server.arg("s").toInt(), 0, OLED_PAGE_COUNT-1);
      oledForced = true;
    }
    server.send(200,"text/plain","OK");
  });
  server.on("/oledAuto", HTTP_GET, [](){
    oledForced = false;
    server.send(200,"text/plain","OK");
  });
  server.on("/setOledSeq", HTTP_GET, [](){
    // POST-style via GET args: seq=3,2,0,1&dur=4,3,3,4&def=3
    if (server.hasArg("seq")) {
      String s = server.arg("seq");
      String d = server.hasArg("dur") ? server.arg("dur") : "";
      uint8_t idx = 0;
      int pos = 0;
      while (pos <= (int)s.length() && idx < OLED_PAGE_COUNT) {
        int comma = s.indexOf(',', pos);
        if (comma < 0) comma = s.length();
        oledSequence[idx] = constrain(s.substring(pos,comma).toInt(), 0, OLED_PAGE_COUNT-1);
        pos = comma + 1; idx++;
      }
      oledSeqLen = idx ? idx : 1;
      // durations
      idx = 0; pos = 0;
      while (pos <= (int)d.length() && idx < oledSeqLen) {
        int comma = d.indexOf(',', pos);
        if (comma < 0) comma = d.length();
        oledDuration[idx] = max(1, (int)d.substring(pos,comma).toInt());
        pos = comma + 1; idx++;
      }
    }
    if (server.hasArg("def"))
      oledDefault = constrain(server.arg("def").toInt(), 0, OLED_PAGE_COUNT-1);
    oledSeqIdx = 0;
    server.send(200,"text/plain","OK");
  });
  server.on("/setOledDefault", HTTP_GET, [](){
    if (server.hasArg("s")) {
      oledDefault = constrain(server.arg("s").toInt(), 0, OLED_PAGE_COUNT-1);
      oledForcedScreen = oledDefault;
      oledForced = true;
    }
    server.send(200,"text/plain","OK");
  });
}

// ══════════════════════════════════════════════════════════════════
//  MAIN WEB PAGE
// ══════════════════════════════════════════════════════════════════
void handleRoot() {
  // Captive-portal redirect for non-root paths
  if (server.uri() != "/") {
    server.sendHeader("Location","http://192.168.4.1/",true);
    server.send(302,"text/plain","");
    return;
  }

  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Everest Smart Mattress</title>
<style>
:root{--blu:#3b82f6;--red:#ef4444;--grn:#22c55e;--pur:#8b5cf6;
      --org:#f59e0b;--crd:#1e293b;--bg:#0f172a;--txt:#f1f5f9;
      --sub:#94a3b8;--bdr:#334155}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:'Segoe UI',sans-serif;
     max-width:520px;margin:0 auto;padding:10px 10px 40px}
h1{text-align:center;font-size:1.35em;color:var(--blu);margin:8px 0 2px}
.sub{text-align:center;color:var(--sub);font-size:.8em;margin-bottom:10px}
/* Tabs */
.tabs{display:flex;gap:5px;margin-bottom:10px;overflow-x:auto;padding-bottom:2px}
.tab{flex:0 0 auto;padding:7px 13px;border:1px solid var(--bdr);border-radius:8px;
     background:var(--crd);color:var(--sub);cursor:pointer;font-size:.8em;
     white-space:nowrap;transition:.15s}
.tab.act{background:var(--pur);color:#fff;border-color:var(--pur)}
.pg{display:none}.pg.act{display:block}
/* Cards */
.card{background:var(--crd);border-radius:14px;padding:14px;margin:8px 0;
      box-shadow:0 4px 18px rgba(0,0,0,.45)}
.card h3{color:var(--pur);font-size:.95em;margin-bottom:10px;
          border-bottom:1px solid var(--bdr);padding-bottom:6px}
/* Stat grid */
.sg{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.si{background:#0f172a;border-radius:10px;padding:10px;text-align:center}
.sv{font-size:1.55em;font-weight:700;color:var(--blu)}
.sl{font-size:.7em;color:var(--sub);margin-top:2px}
/* Score bar */
.sbb{background:#1e3a5f;border-radius:99px;height:10px;margin:6px 0}
.sb{height:10px;border-radius:99px;transition:width .5s;
    background:linear-gradient(90deg,var(--red),var(--grn))}
/* Chart */
canvas{width:100%;display:block;border-radius:8px;background:#0a1628;margin-top:8px}
/* Inputs */
input[type=range]{width:100%;accent-color:var(--blu);margin:4px 0}
input[type=number]{background:#0f172a;border:1px solid var(--bdr);
  color:var(--txt);border-radius:7px;padding:7px;width:62px;
  text-align:center;font-size:.9em}
/* Buttons */
.btn{display:inline-block;padding:9px 16px;border:none;border-radius:9px;
     font-size:.85em;cursor:pointer;font-weight:600;transition:.15s;
     user-select:none}
.bb{background:var(--blu);color:#fff}
.br{background:var(--red);color:#fff}
.bg{background:var(--grn);color:#000}
.bp{background:var(--pur);color:#fff}
.bo{background:var(--org);color:#000}
.bk{background:var(--bdr);color:#fff}
.btn:active{opacity:.75;transform:scale(.97)}
.brow{display:flex;gap:8px;flex-wrap:wrap;margin-top:8px}
/* Alarm tags */
.al{min-height:22px;margin-bottom:6px}
.at{display:inline-flex;align-items:center;gap:5px;background:#1e3a5f;
    border-radius:8px;padding:5px 9px;margin:3px;font-size:.85em}
.xb{background:none;border:none;color:var(--red);cursor:pointer;font-size:1em}
.irow{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap;align-items:center}
/* Motor grid */
.mg{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.mc{background:#0f172a;border-radius:10px;padding:10px}
.mc h4{color:var(--sub);font-size:.78em;margin-bottom:5px}
/* Fan cards */
.fc{display:flex;gap:8px;margin-top:8px}
.fs{flex:1;background:#0f172a;border-radius:12px;padding:14px;
    text-align:center;cursor:pointer;border:2px solid var(--bdr);
    transition:.2s;position:relative}
.fs.on{border-color:var(--grn);background:#052e16}
.fs .fi{font-size:2.2em;margin-bottom:6px}
.fs .fn{font-size:.82em;font-weight:600;color:var(--txt)}
.fs .fst{font-size:.75em;margin-top:4px}
.fs.on .fst{color:var(--grn)}
.fs:not(.on) .fst{color:var(--sub)}
/* OLED screen list */
.olist{display:flex;flex-direction:column;gap:6px;margin-top:8px}
.osi{display:flex;align-items:center;gap:8px;background:#0f172a;
     border-radius:8px;padding:8px 10px}
.osi span{flex:1;font-size:.85em}
.osi input{width:48px}
.osact{border-left:3px solid var(--pur)}
/* MPU bars */
.mbar-wrap{margin:4px 0}
.mbar-lbl{font-size:.75em;color:var(--sub);margin-bottom:2px}
.mbar-bg{background:#1e3a5f;border-radius:4px;height:10px;position:relative}
.mbar-fill{height:10px;border-radius:4px;position:absolute;top:0;
           background:linear-gradient(90deg,var(--blu),var(--pur))}
.mbar-center{position:absolute;top:0;left:50%;width:2px;height:10px;background:var(--bdr)}
</style>
</head>
<body>
<h1>🛏 Everest Smart Mattress</h1>
<div class="sub">O Solutions &nbsp;|&nbsp; Omar Samy &nbsp;|&nbsp; SMv1.3.8PF</div>

<div class="tabs">
  <div class="tab act" onclick="tab('dashboard',this)">📊 Dashboard</div>
  <div class="tab" onclick="tab('control',this)">⚡ Control</div>
  <div class="tab" onclick="tab('ventilation',this)">💨 Ventilation</div>
  <div class="tab" onclick="tab('alarms',this)">⏰ Alarms</div>
  <div class="tab" onclick="tab('mpu',this)">📡 Motion</div>
  <div class="tab" onclick="tab('oled',this)">🖥 OLED</div>
  <div class="tab" onclick="tab('settings',this)">⚙ Settings</div>
</div>

<!-- ══════════ DASHBOARD ══════════ -->
<div id="pg-dashboard" class="pg act">
  <div class="card">
    <h3>📊 Live Readings</h3>
    <div class="sg">
      <div class="si"><div class="sv" id="temp">--</div><div class="sl">Temperature °C</div></div>
      <div class="si"><div class="sv" id="hum">--</div><div class="sl">Humidity %</div></div>
      <div class="si"><div class="sv" id="time">--</div><div class="sl">Time</div></div>
      <div class="si"><div class="sv" id="date" style="font-size:1em">--</div><div class="sl">Date</div></div>
    </div>
  </div>
  <div class="card">
    <h3>😴 Sleep Score</h3>
    <div id="personBadge" style="display:none;background:#052e16;border:1px solid var(--grn);
      border-radius:8px;padding:6px 10px;margin-bottom:8px;font-size:.82em;color:var(--grn)">
      ✅ Person detected — tracking active
    </div>
    <div style="font-size:2.2em;font-weight:700;text-align:center;color:var(--grn)" id="sleepScore">--</div>
    <div class="sbb"><div class="sb" id="scoreBar" style="width:0%"></div></div>
    <canvas id="scoreChart" height="150"></canvas>
  </div>
  <div class="card">
    <h3>📅 Daily Sleep Report</h3>
    <div style="background:#0f172a;border-radius:10px;padding:14px;text-align:center">
      <div style="font-size:2.6em;font-weight:700;color:var(--grn)" id="dailyScore">--</div>
      <div style="color:var(--sub);font-size:.8em;margin-top:4px" id="dailyDate">No data yet (saves at 09:00)</div>
    </div>
  </div>
</div>

<!-- ══════════ CONTROL ══════════ -->
<div id="pg-control" class="pg">
  <div class="card">
    <h3>⚡ Vibration Motors</h3>
    <div class="mg" id="motorGrid"></div>
    <div class="brow" style="margin-top:14px">
      <button class="btn br" onclick="activateBuzz(this)">⚡ Buzz Pattern</button>
    </div>
  </div>
  <div class="card">
    <h3>🌀 PWM Fan</h3>
    <label style="font-size:.82em;color:var(--sub)">Speed — PWM: <span id="fanVal">0</span></label>
    <input type="range" min="0" max="255" value="0" id="fanSlider" oninput="setFanSlider(this.value)">
    <div class="brow">
      <button class="btn bg" id="fanTgl" onclick="toggleFan()">Fan ON</button>
    </div>
  </div>
</div>

<!-- ══════════ VENTILATION ══════════ -->
<div id="pg-ventilation" class="pg">
  <div class="card">
    <h3>💨 Ventilation Control</h3>
    <div class="fc" id="ventGrid">
      <div class="fs" id="ventR" onclick="toggleRelay('R')">
        <div class="fi">🌀</div>
        <div class="fn">Right Side</div>
        <div class="fn" style="font-size:.72em;color:var(--sub)">Ventilation</div>
        <div class="fst" id="ventRst">OFF</div>
      </div>
      <div class="fs" id="ventL" onclick="toggleRelay('L')">
        <div class="fi">🌀</div>
        <div class="fn">Left Side</div>
        <div class="fn" style="font-size:.72em;color:var(--sub)">Ventilation</div>
        <div class="fst" id="ventLst">OFF</div>
      </div>
    </div>
    <div class="brow" style="margin-top:12px">
      <button class="btn bg" onclick="setBoth(true)">▶ Both ON</button>
      <button class="btn bk" onclick="setBoth(false)">■ Both OFF</button>
    </div>
  </div>
</div>

<!-- ══════════ ALARMS ══════════ -->
<div id="pg-alarms" class="pg">
  <div class="card">
    <h3>⏰ Alarms</h3>
    <div class="al" id="alarmList">Loading...</div>
    <div class="irow">
      <input type="number" id="ah" min="0" max="23" placeholder="HH">
      <span style="color:var(--sub)">:</span>
      <input type="number" id="am" min="0" max="59" placeholder="MM">
      <button class="btn bb" onclick="addAlarm()">Add</button>
    </div>
    <div class="brow" style="margin-top:10px">
      <button class="btn bk" onclick="snooze()">💤 Snooze 10s</button>
    </div>
  </div>
</div>

<!-- ══════════ MPU / MOTION ══════════ -->
<div id="pg-mpu" class="pg">
  <div class="card">
    <h3>📡 MPU6050 — Motion & Orientation</h3>
    <div id="mpuStatus" style="font-size:.8em;color:var(--sub);margin-bottom:8px"></div>
    <div class="mbar-wrap">
      <div class="mbar-lbl">X Angle: <span id="mpuXv">0</span>°</div>
      <div class="mbar-bg"><div class="mbar-center"></div><div class="mbar-fill" id="mpuXb" style="width:50%;left:0"></div></div>
    </div>
    <div class="mbar-wrap">
      <div class="mbar-lbl">Y Angle: <span id="mpuYv">0</span>°</div>
      <div class="mbar-bg"><div class="mbar-center"></div><div class="mbar-fill" id="mpuYb" style="width:50%;left:0"></div></div>
    </div>
    <div class="mbar-wrap">
      <div class="mbar-lbl">Z Angle: <span id="mpuZv">0</span>°</div>
      <div class="mbar-bg"><div class="mbar-center"></div><div class="mbar-fill" id="mpuZb" style="width:50%;left:0"></div></div>
    </div>
    <div style="margin-top:12px">
      <div style="font-size:.78em;color:var(--sub);margin-bottom:4px">Gyro (°/s)</div>
      <div class="sg">
        <div class="si"><div class="sv" id="mpuGXv" style="font-size:1.1em">0</div><div class="sl">GX</div></div>
        <div class="si"><div class="sv" id="mpuGYv" style="font-size:1.1em">0</div><div class="sl">GY</div></div>
      </div>
      <div class="sg" style="margin-top:8px">
        <div class="si"><div class="sv" id="mpuGZv" style="font-size:1.1em">0</div><div class="sl">GZ</div></div>
        <div class="si"><div class="sv" id="mpuTotal" style="font-size:1.1em;color:var(--org)">0</div><div class="sl">Total Δ</div></div>
      </div>
    </div>
    <div id="personMpuBadge" style="display:none;margin-top:10px;background:#052e16;
      border:1px solid var(--grn);border-radius:8px;padding:8px;text-align:center;
      font-size:.82em;color:var(--grn)">🧑 Person detected — sleep tracking active</div>
  </div>
</div>

<!-- ══════════ OLED CONTROL ══════════ -->
<div id="pg-oled" class="pg">
  <div class="card">
    <h3>🖥 OLED Screen Control</h3>
    <!-- Force single screen -->
    <div style="font-size:.82em;color:var(--sub);margin-bottom:8px">Force a screen now:</div>
    <div style="display:flex;gap:6px;flex-wrap:wrap" id="oledBtns"></div>
    <div class="brow" style="margin-top:8px">
      <button class="btn bk" onclick="oledAuto()">▶ Auto Rotate</button>
    </div>
  </div>
  <div class="card">
    <h3>🗂 Screen Sequence & Timing</h3>
    <div style="font-size:.78em;color:var(--sub);margin-bottom:8px">
      Drag to reorder · Set seconds per screen · Choose default screen
    </div>
    <div class="olist" id="oledSeqList"></div>
    <div class="brow" style="margin-top:10px">
      <button class="btn bb" onclick="saveOledSeq()">💾 Save Sequence</button>
    </div>
  </div>
</div>

<!-- ══════════ SETTINGS ══════════ -->
<div id="pg-settings" class="pg">
  <div class="card">
    <h3>🕐 Set RTC Date & Time</h3>
    <div style="font-size:.78em;color:var(--sub);margin-bottom:8px">
      Set both date and time — RTC keeps ticking even after power off (battery backed).
    </div>
    <div style="font-size:.8em;color:var(--sub);margin-bottom:4px">Date</div>
    <div class="irow">
      <input type="number" id="tyr" min="2024" max="2099" placeholder="YYYY" style="width:72px">
      <input type="number" id="tmo" min="1" max="12" placeholder="MM">
      <input type="number" id="tdy" min="1" max="31" placeholder="DD">
    </div>
    <div style="font-size:.8em;color:var(--sub);margin:8px 0 4px">Time</div>
    <div class="irow">
      <input type="number" id="th" min="0" max="23" placeholder="HH">
      <input type="number" id="tm" min="0" max="59" placeholder="MM">
      <input type="number" id="ts" min="0" max="59" placeholder="SS">
      <button class="btn bb" onclick="setRTC()">Set</button>
    </div>
    <div style="margin-top:8px">
      <button class="btn bg" onclick="setRTCNow()" style="font-size:.8em">
        📱 Use Device Time
      </button>
    </div>
  </div>
  <div class="card">
    <h3>🔧 System</h3>
    <div class="brow">
      <button class="btn bk" onclick="sleepNow()">💤 Deep Sleep</button>
    </div>
    <div style="margin-top:12px;font-size:.78em;color:var(--sub);line-height:1.8">
      Auto-sleep after <b>15 min</b> idle + no clients connected.<br>
      Wake sources: Piezo impact · MPU motion · 1h timer
    </div>
  </div>
</div>

<script>
// ── Tab ──────────────────────────────────────────────────────────
function tab(id,el){
  document.querySelectorAll('.pg').forEach(p=>p.classList.remove('act'));
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('act'));
  document.getElementById('pg-'+id).classList.add('act');
  el.classList.add('act');
}

// ── Motors ───────────────────────────────────────────────────────
var mPWM=[0,0,0,0];
(function buildMotors(){
  var g=document.getElementById('motorGrid'); g.innerHTML='';
  ['V1','V2','V3','V4'].forEach(function(lbl,i){
    g.innerHTML+='<div class="mc"><h4>Motor '+lbl+'</h4>'+
      '<div style="font-size:.75em;color:var(--sub)">PWM: <span id="mv'+i+'">0</span></div>'+
      '<input type="range" min="0" max="255" value="0" id="ms'+i+'"'+
      ' oninput="setMPWM('+i+',this.value)">'+
      '<button class="btn bb" style="width:100%;margin-top:6px;font-size:.8em"'+
      ' onclick="mPulse('+i+')">▶ 5s</button></div>';
  });
})();
function setMPWM(ch,v){ mPWM[ch]=+v;
  document.getElementById('mv'+ch).textContent=v; fetch('/setPWM?ch='+ch+'&v='+v); }
function mPulse(ch){ fetch('/motorPulse?ch='+ch); }
function activateBuzz(btn){
  fetch('/buzz').then(()=>{
    btn.textContent='Buzzing...'; btn.disabled=true;
    setTimeout(()=>{btn.textContent='⚡ Buzz Pattern';btn.disabled=false;},6400);
  });
}

// ── PWM Fan ──────────────────────────────────────────────────────
var fanIsOn=false;
function setFanSlider(v){ document.getElementById('fanVal').textContent=v; fetch('/setFan?v='+v); }
function toggleFan(){
  fanIsOn=!fanIsOn;
  fetch('/fanToggle').then(()=>{
    document.getElementById('fanTgl').textContent=fanIsOn?'Fan OFF':'Fan ON';
    document.getElementById('fanSlider').value=fanIsOn?200:0;
    document.getElementById('fanVal').textContent=fanIsOn?200:0;
  });
}

// ── Relay ventilation ────────────────────────────────────────────
var rR=false,rL=false;
function toggleRelay(side){
  var on=(side==='R')?!rR:!rL;
  fetch('/setRelay?side='+side+'&v='+(on?1:0)).then(()=>syncRelayUI());
}
function setBoth(on){
  fetch('/setRelay?side=both&v='+(on?1:0)).then(()=>syncRelayUI());
}
function syncRelayUI(){
  ['R','L'].forEach(function(s){
    var el=document.getElementById('vent'+s);
    var st=document.getElementById('vent'+s+'st');
    var on=(s==='R')?rR:rL;
    el.classList.toggle('on',on);
    st.textContent=on?'● ON':'○ OFF';
  });
}

// ── Alarms ───────────────────────────────────────────────────────
function loadAlarms(){
  fetch('/alarms').then(r=>r.json()).then(function(a){
    var el=document.getElementById('alarmList');
    if(!a.length){el.innerHTML='<span style="color:var(--sub)">No alarms set</span>';return;}
    el.innerHTML='';
    a.forEach(function(x){
      var t=document.createElement('span'); t.className='at';
      t.innerHTML=(x.h<10?'0':'')+x.h+':'+(x.m<10?'0':'')+x.m+
        ' <button class="xb" onclick="delAlarm('+x.i+')">✕</button>';
      el.appendChild(t);
    });
  });
}
loadAlarms();
function addAlarm(){
  var h=document.getElementById('ah').value, m=document.getElementById('am').value;
  if(h===''||m===''){alert('Enter hour and minute');return;}
  fetch('/addAlarm?h='+h+'&m='+m).then(()=>{
    document.getElementById('ah').value='';
    document.getElementById('am').value='';
    loadAlarms();
  });
}
function delAlarm(i){fetch('/deleteAlarm?i='+i).then(loadAlarms);}
function snooze(){fetch('/snooze').then(()=>alert('Snoozed for 10 seconds'));}

// ── RTC / Sleep ──────────────────────────────────────────────────
function setRTC(){
  var yr=document.getElementById('tyr').value;
  var mo=document.getElementById('tmo').value;
  var dy=document.getElementById('tdy').value;
  var h =document.getElementById('th').value;
  var m =document.getElementById('tm').value;
  var s =document.getElementById('ts').value||'0';
  if(!yr||!mo||!dy||!h||!m){alert('Fill in all date and time fields');return;}
  fetch('/setTime?yr='+yr+'&mo='+mo+'&dy='+dy+'&h='+h+'&m='+m+'&s='+s)
    .then(()=>alert('RTC date & time updated!'));
}
function setRTCNow(){
  // Use the browser/phone's current time to set the RTC
  var now=new Date();
  fetch('/setTime?yr='+now.getFullYear()
       +'&mo='+(now.getMonth()+1)
       +'&dy='+now.getDate()
       +'&h='+now.getHours()
       +'&m='+now.getMinutes()
       +'&s='+now.getSeconds())
    .then(()=>alert('RTC synced to your device time!'));
}
function sleepNow(){
  if(confirm('Put ESP32 into deep sleep?'))
    fetch('/sleep').then(()=>alert('System sleeping. Reconnect WiFi to wake via web.'));
}

// ── OLED control ─────────────────────────────────────────────────
var oledNames=['Next Alarm','Sleep Graph','Climate','Time','Daily Score',
               'Fan Status','Ventilation','MPU Motion','System','Brand'];
var oledSeq=[],oledDurations=[],oledDefault=3;

function buildOledBtns(){
  var d=document.getElementById('oledBtns'); d.innerHTML='';
  oledNames.forEach(function(n,i){
    var b=document.createElement('button');
    b.className='btn bp'; b.style.fontSize='.75em'; b.style.padding='6px 10px';
    b.textContent=i+' '+n;
    b.onclick=function(){fetch('/setOledScreen?s='+i)
      .then(()=>{ document.querySelectorAll('#oledBtns .btn')
        .forEach(x=>x.style.outline='none');
        b.style.outline='2px solid var(--grn)'; });};
    d.appendChild(b);
  });
}
function oledAuto(){
  fetch('/oledAuto').then(()=>{
    document.querySelectorAll('#oledBtns .btn').forEach(x=>x.style.outline='none');
    alert('Auto-rotate resumed');
  });
}
function buildOledSeqList(){
  var el=document.getElementById('oledSeqList'); el.innerHTML='';
  oledSeq.forEach(function(s,i){
    var row=document.createElement('div');
    row.className='osi'+(s===oledDefault?' osact':'');
    row.innerHTML='<span>'+i+'. <b>'+oledNames[s]+'</b></span>'+
      '<input type="number" min="1" max="60" value="'+(oledDurations[i]||3)+
      '" id="od'+i+'" style="width:46px"> s'+
      ' <button class="btn bk" style="padding:4px 8px;font-size:.75em" onclick="setDef('+s+')">Default</button>';
    el.appendChild(row);
  });
}
function setDef(s){
  fetch('/setOledDefault?s='+s).then(()=>{oledDefault=s;buildOledSeqList();});
}
function saveOledSeq(){
  var seq=oledSeq.join(',');
  var dur=oledSeq.map(function(_,i){
    return document.getElementById('od'+i)?document.getElementById('od'+i).value:3;
  }).join(',');
  fetch('/setOledSeq?seq='+seq+'&dur='+dur+'&def='+oledDefault)
    .then(()=>alert('OLED sequence saved!'));
}
fetch('/oledConfig').then(r=>r.json()).then(function(cfg){
  oledDefault=cfg.default;
  oledSeq=cfg.seq.map(function(x){return x.s;});
  oledDurations=cfg.seq.map(function(x){return x.d;});
  buildOledBtns();
  buildOledSeqList();
});

// ── Score chart ───────────────────────────────────────────────────
var scoreData=[];
function drawChart(){
  var c=document.getElementById('scoreChart');
  if(!c)return;
  var ctx=c.getContext('2d'),W=c.width,H=c.height;
  ctx.clearRect(0,0,W,H);
  // Grid lines
  ctx.strokeStyle='#1e3a5f'; ctx.lineWidth=1;
  [25,50,75,100].forEach(function(v){
    var y=H-v/100*H;
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();
    ctx.fillStyle='#475569'; ctx.font='9px sans-serif';
    ctx.fillText(v,2,y-2);
  });
  if(scoreData.length<2)return;
  var step=W/(scoreData.length-1);
  ctx.lineWidth=2;
  for(var i=0;i<scoreData.length-1;i++){
    var v0=scoreData[i],v1=scoreData[i+1];
    var x0=i*step,x1=(i+1)*step;
    var y0=H-v0/100*H,y1=H-v1/100*H;
    var g=ctx.createLinearGradient(x0,0,x1,0);
    g.addColorStop(0,scoreColor(v0)); g.addColorStop(1,scoreColor(v1));
    ctx.strokeStyle=g;
    ctx.beginPath(); ctx.moveTo(x0,y0); ctx.lineTo(x1,y1); ctx.stroke();
  }
}
function scoreColor(v){
  var r=Math.round(239-(239-59)*(v/100));
  var g=Math.round(68+(197-68)*(v/100));
  var b=Math.round(68+(246-68)*(v/100));
  return'rgb('+r+','+g+','+b+')';
}

// ── MPU bar helper ────────────────────────────────────────────────
function mpuBar(id,val){
  // -180..+180 → 0..100% of bar, center = 50%
  var pct=(val+180)/360*100;
  pct=Math.max(0,Math.min(100,pct));
  var el=document.getElementById(id);
  if(el){ el.style.width=Math.abs(pct-50)+'%';
          el.style.left=(pct<50?pct:50)+'%'; }
}

// ── Live poll ─────────────────────────────────────────────────────
var prevX=0,prevY=0,prevZ=0;
setInterval(function(){
  fetch('/data').then(r=>r.json()).then(function(d){
    // Dashboard
    document.getElementById('temp').textContent=d.temp;
    document.getElementById('hum').textContent=d.hum;
    document.getElementById('time').textContent=d.time;
    document.getElementById('date').textContent=d.date;
    var sc=parseFloat(d.sleep);
    document.getElementById('sleepScore').textContent=sc.toFixed(1);
    document.getElementById('scoreBar').style.width=sc+'%';
    if(d.dailyScore>0){
      document.getElementById('dailyScore').textContent=parseFloat(d.dailyScore).toFixed(1);
      document.getElementById('dailyDate').textContent=d.dailyDate;
    }
    // Person badge
    var pb=document.getElementById('personBadge');
    var pb2=document.getElementById('personMpuBadge');
    if(d.person){pb.style.display='block';pb2.style.display='block';}
    // Fan sync
    fanIsOn=d.fanOn;
    document.getElementById('fanTgl').textContent=fanIsOn?'Fan OFF':'Fan ON';
    document.getElementById('fanSlider').value=d.fanPWM;
    document.getElementById('fanVal').textContent=d.fanPWM;
    // Motor sync
    d.motorPWM.forEach(function(v,i){
      var s=document.getElementById('ms'+i); if(s)s.value=v;
      var vl=document.getElementById('mv'+i); if(vl)vl.textContent=v;
    });
    // Relay sync
    rR=d.relayR; rL=d.relayL; syncRelayUI();
    // MPU
    document.getElementById('mpuXv').textContent=parseFloat(d.mpuX).toFixed(1);
    document.getElementById('mpuYv').textContent=parseFloat(d.mpuY).toFixed(1);
    document.getElementById('mpuZv').textContent=parseFloat(d.mpuZ).toFixed(1);
    mpuBar('mpuXb',d.mpuX);
    mpuBar('mpuYb',d.mpuY);
    mpuBar('mpuZb',d.mpuZ);
    document.getElementById('mpuGXv').textContent=parseFloat(d.mpuGX).toFixed(1);
    document.getElementById('mpuGYv').textContent=parseFloat(d.mpuGY).toFixed(1);
    document.getElementById('mpuGZv').textContent=parseFloat(d.mpuGZ).toFixed(1);
    var total=Math.sqrt(d.mpuGX*d.mpuGX+d.mpuGY*d.mpuGY+d.mpuGZ*d.mpuGZ);
    document.getElementById('mpuTotal').textContent=total.toFixed(1);
    document.getElementById('mpuStatus').textContent=
      d.mpuOK?'✅ MPU6050 connected':'❌ MPU6050 not detected';
    prevX=d.mpuX; prevY=d.mpuY; prevZ=d.mpuZ;
  }).catch(function(){});

  fetch('/scoreHistory').then(r=>r.json()).then(function(a){
    scoreData=a; drawChart();
  }).catch(function(){});
},1000);

// Canvas size on load
window.addEventListener('load',function(){
  var c=document.getElementById('scoreChart');
  if(c){c.width=c.offsetWidth;}
});
</script>
</body>
</html>
)rawhtml";

  server.send(200, "text/html", html);
}

void handleControl() {
  server.sendHeader("Location","http://192.168.4.1/",true);
  server.send(302,"text/plain","");
}
void handleVentilation()  { handleRoot(); }
void handleOledConfig()   { handleRoot(); }
