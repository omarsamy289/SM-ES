#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>

#define DHTPIN 15
#define DHTTYPE DHT22
#define MOTOR_PIN_1 23
#define MOTOR_PIN_2 25
#define MOTOR_PIN_3 18
#define MOTOR_PIN_4 32
#define MAX_ALARMS 5

void handleRoot();
void handleData();
void runBuzzPattern();

DHT dht(DHTPIN, DHTTYPE);
RTC_DS3231 rtc;

const char* ssid     = "ESP32-omar";
const char* password = "om1414";

WebServer server(80);

float temperature = 0;
float humidity    = 0;

int  motorPWM     = 0;
bool patternActive = false;

/* alarms */
int  alarmHour[MAX_ALARMS];
int  alarmMinute[MAX_ALARMS];
bool alarmEnabled[MAX_ALARMS];
bool alarmFired[MAX_ALARMS];   // FIX: prevents re-firing every second within the same minute

/* sleep tracking */
int   movementCount = 0;
float sleepScore    = 100.0;

/* timing — avoids delay() blocking the server */
unsigned long lastSensorRead  = 0;
unsigned long lastScoreUpdate = 0;

/* ── buzz pattern (non-blocking would be ideal but kept simple) ── */
void runBuzzPattern() {
  // 6 pulses: pins 1&3 on, then 2&4 on
  for (int i = 0; i < 6; i++) {
    ledcWrite(MOTOR_PIN_1, 200); ledcWrite(MOTOR_PIN_3, 200);
    ledcWrite(MOTOR_PIN_2, 0);   ledcWrite(MOTOR_PIN_4, 0);
    delay(200);
    ledcWrite(MOTOR_PIN_1, 0);   ledcWrite(MOTOR_PIN_3, 0);
    ledcWrite(MOTOR_PIN_2, 200); ledcWrite(MOTOR_PIN_4, 200);
    delay(100);
  }
  // all off
  ledcWrite(MOTOR_PIN_1, 0); ledcWrite(MOTOR_PIN_2, 0);
  ledcWrite(MOTOR_PIN_3, 0); ledcWrite(MOTOR_PIN_4, 0);
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  Wire.begin(21, 22);

  if (!rtc.begin()) {
    Serial.println("RTC not found");
  }

  ledcAttach(MOTOR_PIN_1, 5000, 8); ledcWrite(MOTOR_PIN_1, 0);
  ledcAttach(MOTOR_PIN_2, 5000, 8); ledcWrite(MOTOR_PIN_2, 0);
  ledcAttach(MOTOR_PIN_3, 5000, 8); ledcWrite(MOTOR_PIN_3, 0);
  ledcAttach(MOTOR_PIN_4, 5000, 8); ledcWrite(MOTOR_PIN_4, 0);

  // FIX: initialise alarm arrays
  for (int i = 0; i < MAX_ALARMS; i++) {
    alarmHour[i]    = 0;
    alarmMinute[i]  = 0;
    alarmEnabled[i] = false;
    alarmFired[i]   = false;
  }

  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);

  /* slider */
  server.on("/setPWM", []() {
    if (server.hasArg("value")) {
      motorPWM = server.arg("value").toInt();
      motorPWM = constrain(motorPWM, 0, 255);   // FIX: clamp input
      ledcWrite(MOTOR_PIN_1, motorPWM);
      ledcWrite(MOTOR_PIN_2, motorPWM);
      ledcWrite(MOTOR_PIN_3, motorPWM);
      ledcWrite(MOTOR_PIN_4, motorPWM);
    }
    server.send(200, "text/plain", "OK");
  });

  /* manual buzz button */
  server.on("/buzz", []() {
    patternActive = true;
    server.send(200, "text/plain", "buzz");
  });

  /* add alarm */
  server.on("/addAlarm", []() {
    if (server.hasArg("h") && server.hasArg("m")) {
      for (int i = 0; i < MAX_ALARMS; i++) {
        if (!alarmEnabled[i]) {
          alarmHour[i]    = server.arg("h").toInt();
          alarmMinute[i]  = server.arg("m").toInt();
          alarmEnabled[i] = true;
          alarmFired[i]   = false;
          break;
        }
      }
    }
    server.sendHeader("Location", "/");
    server.send(303);
  });

  /* delete alarm */
  server.on("/deleteAlarm", []() {
    if (server.hasArg("i")) {
      int idx = server.arg("i").toInt();
      if (idx >= 0 && idx < MAX_ALARMS) {
        alarmEnabled[idx] = false;
        alarmFired[idx]   = false;
      }
    }
    server.sendHeader("Location", "/");
    server.send(303);
  });

  /* set rtc time */
  server.on("/setTime", []() {
    if (server.hasArg("h") && server.hasArg("m") && server.hasArg("s")) {
      DateTime now = rtc.now();
      rtc.adjust(DateTime(
        now.year(), now.month(), now.day(),
        server.arg("h").toInt(),
        server.arg("m").toInt(),
        server.arg("s").toInt()
      ));
    }
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.begin();
}

void loop() {
  server.handleClient();

  DateTime now = rtc.now();
  unsigned long ms = millis();

  /* read sensors every 2 s (DHT22 max rate) */
  if (ms - lastSensorRead >= 2000) {
    lastSensorRead = ms;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    // FIX: only update if read succeeded (avoids NaN in JSON)
    if (!isnan(h)) humidity    = h;
    if (!isnan(t)) temperature = t;
  }

  /* fake sleep score — update once per minute so it drains slowly */
  if (ms - lastScoreUpdate >= 60000) {
    lastScoreUpdate = ms;
    movementCount++;
    sleepScore = constrain(100.0 - movementCount * 2.0, 0.0, 100.0); // FIX: clamped
  }

  /* alarm check — FIX: alarmFired prevents re-triggering every second */
  for (int i = 0; i < MAX_ALARMS; i++) {
    if (alarmEnabled[i]) {
      bool match = (now.hour() == alarmHour[i] && now.minute() == alarmMinute[i]);
      if (match && !alarmFired[i]) {
        patternActive  = true;
        alarmFired[i]  = true;
      }
      // reset fired flag once the minute has passed
      if (!match) {
        alarmFired[i] = false;
      }
    }
  }

  /* run buzz pattern when requested */
  if (patternActive) {
    patternActive = false;
    runBuzzPattern();
    // restore slider PWM after pattern
    ledcWrite(MOTOR_PIN_1, motorPWM);
    ledcWrite(MOTOR_PIN_2, motorPWM);
    ledcWrite(MOTOR_PIN_3, motorPWM);
    ledcWrite(MOTOR_PIN_4, motorPWM);
  }
}

/* ── JSON endpoint ── */
void handleData() {
  DateTime now = rtc.now();

  // FIX: pad time/date components so "9:5:3" becomes "09:05:03"
  char timeBuf[9], dateBuf[11];
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d",
           now.hour(), now.minute(), now.second());
  snprintf(dateBuf, sizeof(dateBuf), "%02d/%02d/%04d",
           now.day(), now.month(), now.year());

  // FIX: safe float formatting avoids NaN in JSON
  char json[200];
  snprintf(json, sizeof(json),
    "{\"temp\":%.1f,\"hum\":%.1f,\"time\":\"%s\",\"date\":\"%s\",\"sleep\":%.1f}",
    temperature, humidity, timeBuf, dateBuf, sleepScore);

  server.send(200, "application/json", json);
}

/* ── Web page ── */
void handleRoot() {

  // build alarm list rows
  String alarmRows = "";
  for (int i = 0; i < MAX_ALARMS; i++) {
    if (alarmEnabled[i]) {
      alarmRows += "<tr><td>";
      alarmRows += (alarmHour[i]   < 10 ? "0" : ""); alarmRows += String(alarmHour[i]);
      alarmRows += ":";
      alarmRows += (alarmMinute[i] < 10 ? "0" : ""); alarmRows += String(alarmMinute[i]);
      alarmRows += "</td><td><a href='/deleteAlarm?i=";
      alarmRows += String(i);
      alarmRows += "'>Delete</a></td></tr>";
    }
  }

  String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>Smart Mattress</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background: #f0f4f8; margin: 0; padding: 20px; }
    h2   { color: #2c3e50; }
    h4   { color: #7f8c8d; margin-top: -10px; }
    .card { background: white; border-radius: 12px; padding: 16px; margin: 12px auto;
            max-width: 420px; box-shadow: 0 2px 8px rgba(0,0,0,.1); }
    .stat { font-size: 1.3em; margin: 6px 0; }
    input[type=range] { width: 80%; }
    input[type=number], input[type=text] { width: 60px; padding: 6px; margin: 4px; border-radius: 6px;
            border: 1px solid #ccc; text-align: center; }
    button, input[type=submit] {
            background: #3498db; color: white; border: none; padding: 10px 22px;
            border-radius: 8px; font-size: 1em; cursor: pointer; margin: 6px; }
    button:hover, input[type=submit]:hover { background: #2980b9; }
    #buzzBtn { background: #e74c3c; }
    #buzzBtn:hover { background: #c0392b; }
    table { margin: 0 auto; border-collapse: collapse; }
    td    { padding: 6px 12px; border-bottom: 1px solid #eee; }
    a     { color: #e74c3c; text-decoration: none; }
  </style>
</head>
<body>
  <h2>Smart Mattress</h2>
  <h4>Omar Samy</h4>

  <div class='card'>
    <div class='stat'>&#128336; <b><span id='time'>--</span></b> &nbsp; <span id='date'>--</span></div>
    <div class='stat'>&#127777; Temperature: <b><span id='temp'>--</span> &deg;C</b></div>
    <div class='stat'>&#128167; Humidity: <b><span id='hum'>--</span> %</b></div>
    <div class='stat'>&#128164; Sleep Score: <b><span id='sleep'>--</span></b></div>
  </div>

  <div class='card'>
    <h3>Motor Control</h3>
    <input type='range' min='0' max='255' value='0' id='slider'><br>
    PWM: <span id='val'>0</span>
    <br><br>
    <button id='buzzBtn' onclick='buzz()'>&#9889; Activate Buzz</button>
  </div>

  <div class='card'>
    <h3>Alarms</h3>
    <table id='alarmTable'>)rawhtml";

  html += alarmRows.length() ? alarmRows : "<tr><td>No alarms set</td></tr>";

  html += R"rawhtml(
    </table>
    <br>
    <b>Add Alarm</b><br>
    Hour   <input type='number' id='ah' min='0' max='23' placeholder='HH'>
    Minute <input type='number' id='am' min='0' max='59' placeholder='MM'>
    <br>
    <button onclick="addAlarm()">Add</button>
  </div>

  <div class='card'>
    <h3>Set RTC Time</h3>
    Hour   <input type='number' id='th' min='0' max='23' placeholder='HH'>
    Minute <input type='number' id='tm' min='0' max='59' placeholder='MM'>
    Second <input type='number' id='ts' min='0' max='59' placeholder='SS'>
    <br>
    <button onclick="setTime()">Set Time</button>
  </div>

  <script>
    // Slider
    var slider = document.getElementById('slider');
    var valSpan = document.getElementById('val');
    slider.oninput = function() {
      valSpan.innerHTML = this.value;
      fetch('/setPWM?value=' + this.value);
    };

    // Buzz button
    function buzz() {
      fetch('/buzz').then(r => r.text()).then(() => {
        var btn = document.getElementById('buzzBtn');
        btn.textContent = 'Buzzing...';
        btn.disabled = true;
        setTimeout(() => { btn.textContent = '\u26A1 Activate Buzz'; btn.disabled = false; }, 2200);
      });
    }

    // Add alarm via fetch (no page reload)
    function addAlarm() {
      var h = document.getElementById('ah').value;
      var m = document.getElementById('am').value;
      if (h === '' || m === '') { alert('Enter hour and minute'); return; }
      fetch('/addAlarm?h=' + h + '&m=' + m).then(() => location.reload());
    }

    // Set RTC time via fetch
    function setTime() {
      var h = document.getElementById('th').value;
      var m = document.getElementById('tm').value;
      var s = document.getElementById('ts').value;
      fetch('/setTime?h=' + h + '&m=' + m + '&s=' + (s||0)).then(() => alert('Time set!'));
    }

    // Live sensor update every second
    setInterval(function() {
      fetch('/data')
        .then(r => r.json())
        .then(d => {
          document.getElementById('time').innerHTML  = d.time;
          document.getElementById('date').innerHTML  = d.date;
          document.getElementById('temp').innerHTML  = d.temp;
          document.getElementById('hum').innerHTML   = d.hum;
          document.getElementById('sleep').innerHTML = d.sleep;
        })
        .catch(() => {}); // silently ignore if ESP is busy during buzz
    }, 1000);
  </script>
</body>
</html>
)rawhtml";

  server.send(200, "text/html", html);
}