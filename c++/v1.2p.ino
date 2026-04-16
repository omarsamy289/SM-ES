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

DHT dht(DHTPIN, DHTTYPE);
RTC_DS3231 rtc;

const char* ssid = "ESP32-omar";
const char* password = "om1414";

WebServer server(80);

float temperature = 0;
float humidity = 0;

int motorPWM = 0;
bool patternActive = false;

/* alarms */

int alarmHour[MAX_ALARMS];
int alarmMinute[MAX_ALARMS];
bool alarmEnabled[MAX_ALARMS];

/* sleep tracking */

int movementCount = 0;
float sleepScore = 100;

void setup(){

Serial.begin(115200);

dht.begin();
Wire.begin(21,22);

if(!rtc.begin()){
Serial.println("RTC not found");
}

ledcAttach(MOTOR_PIN_1,5000,8);
ledcWrite(MOTOR_PIN_1,0);
ledcAttach(MOTOR_PIN_2,5000,8);
ledcWrite(MOTOR_PIN_2,0);
ledcAttach(MOTOR_PIN_3,5000,8);
ledcWrite(MOTOR_PIN_3,0);
ledcAttach(MOTOR_PIN_4,5000,8);
ledcWrite(MOTOR_PIN_4,0);

WiFi.softAP(ssid,password);

Serial.println(WiFi.softAPIP());

server.on("/",handleRoot);
server.on("/data",handleData);

/* slider */

server.on("/setPWM",[](){

if(server.hasArg("value")){
motorPWM = server.arg("value").toInt();
ledcWrite(MOTOR_PIN_1,motorPWM);
ledcWrite(MOTOR_PIN_2,motorPWM);
ledcWrite(MOTOR_PIN_3,motorPWM);
ledcWrite(MOTOR_PIN_4,motorPWM);
}

server.send(200,"text/plain","OK");

});


server.on("/buzz",[](){

patternActive = true;
server.send(200,"text/plain","buzz");

});

/* add alarm */

server.on("/addAlarm",[](){

if(server.hasArg("h") && server.hasArg("m")){

for(int i=0;i<MAX_ALARMS;i++){

if(!alarmEnabled[i]){

alarmHour[i] = server.arg("h").toInt();
alarmMinute[i] = server.arg("m").toInt();
alarmEnabled[i] = true;

break;

}

}

}

server.sendHeader("Location","/");
server.send(303);

});

/* set rtc time */

server.on("/setTime",[](){

if(server.hasArg("h") && server.hasArg("m") && server.hasArg("s")){

DateTime now = rtc.now();

rtc.adjust(DateTime(
now.year(),
now.month(),
now.day(),
server.arg("h").toInt(),
server.arg("m").toInt(),
server.arg("s").toInt()
));

}

server.sendHeader("Location","/");
server.send(303);

});

server.begin();

}

void loop(){

server.handleClient();

DateTime now = rtc.now();

/* read sensors */

humidity = dht.readHumidity();
temperature = dht.readTemperature();

/* alarm check */

for(int i=0;i<MAX_ALARMS;i++){

if(alarmEnabled[i]){

if(now.hour()==alarmHour[i] && now.minute()==alarmMinute[i]){

patternActive = true;

}

}

}

/* vibration */

if(patternActive){

ledcWrite(MOTOR_PIN_1,200);
ledcWrite(MOTOR_PIN_3,200);
ledcWrite(MOTOR_PIN_2,0);
ledcWrite(MOTOR_PIN_4,0);
delay(200);
ledcWrite(MOTOR_PIN_1,0);
ledcWrite(MOTOR_PIN_3,0);
ledcWrite(MOTOR_PIN_2,200);
ledcWrite(MOTOR_PIN_4,200);
delay(100);
ledcWrite(MOTOR_PIN_1,200);
ledcWrite(MOTOR_PIN_3,200);
ledcWrite(MOTOR_PIN_2,0);
ledcWrite(MOTOR_PIN_4,0);
delay(200);
ledcWrite(MOTOR_PIN_1,0);
ledcWrite(MOTOR_PIN_3,0);
ledcWrite(MOTOR_PIN_2,200);
ledcWrite(MOTOR_PIN_4,200);
delay(100);
ledcWrite(MOTOR_PIN_1,200);
ledcWrite(MOTOR_PIN_3,200);
ledcWrite(MOTOR_PIN_2,0);
ledcWrite(MOTOR_PIN_4,0);
delay(200);
ledcWrite(MOTOR_PIN_1,0);
ledcWrite(MOTOR_PIN_3,0);
ledcWrite(MOTOR_PIN_2,200);
ledcWrite(MOTOR_PIN_4,200);
delay(100);
ledcWrite(MOTOR_PIN_1,200);
ledcWrite(MOTOR_PIN_3,200);
ledcWrite(MOTOR_PIN_2,0);
ledcWrite(MOTOR_PIN_4,0);
delay(200);
ledcWrite(MOTOR_PIN_1,0);
ledcWrite(MOTOR_PIN_3,0);
ledcWrite(MOTOR_PIN_2,200);
ledcWrite(MOTOR_PIN_4,200);
delay(100);
ledcWrite(MOTOR_PIN_1,200);
ledcWrite(MOTOR_PIN_3,200);
ledcWrite(MOTOR_PIN_2,0);
ledcWrite(MOTOR_PIN_4,0);
delay(200);
ledcWrite(MOTOR_PIN_1,0);
ledcWrite(MOTOR_PIN_3,0);
ledcWrite(MOTOR_PIN_2,200);
ledcWrite(MOTOR_PIN_4,200);
delay(100);
ledcWrite(MOTOR_PIN_1,200);
ledcWrite(MOTOR_PIN_3,200);
ledcWrite(MOTOR_PIN_2,0);
ledcWrite(MOTOR_PIN_4,0);
delay(200);
ledcWrite(MOTOR_PIN_1,0);
ledcWrite(MOTOR_PIN_3,0);
ledcWrite(MOTOR_PIN_2,200);
ledcWrite(MOTOR_PIN_4,200);
delay(100);




patternActive=false;

}

/* fake movement detection for demo */

movementCount++;
sleepScore = 100 - movementCount*0.1;

delay(1000);

}

/* JSON data for live page */

void handleData(){

DateTime now = rtc.now();

String json="{";

json+="\"temp\":"+String(temperature)+",";
json+="\"hum\":"+String(humidity)+",";

json+="\"time\":\"";
json+=String(now.hour())+":"+String(now.minute())+":"+String(now.second());
json+="\",";

json+="\"date\":\"";
json+=String(now.day())+"/"+String(now.month())+"/"+String(now.year());
json+="\",";

json+="\"sleep\":"+String(sleepScore);

json+="}";

server.send(200,"application/json",json);

}

/* web page */

void handleRoot(){

String html="<!DOCTYPE html><html>";

html+="<head>";
html+="<meta name='viewport' content='width=device-width, initial-scale=1'>";
html+="</head>";

html+="<body style='font-family:Arial;text-align:center'>";

html+="<h2>Smart Mattress</h2>";
html+="<h4>Omar Samy</h4>";

html+="<h3>Time: <span id='time'>--</span></h3>";
html+="<h3>Date: <span id='date'>--</span></h3>";

html+="<h3>Temperature: <span id='temp'>--</span> C</h3>";
html+="<h3>Humidity: <span id='hum'>--</span> %</h3>";

html+="<h3>Sleep Score: <span id='sleep'>--</span></h3>";

/* slider */

html+="<h3>Motor Control</h3>";
html+="<input type='range' min='0' max='255' value='0' id='slider'>";
html+="<p>PWM: <span id='val'>0</span></p>";

html+="<script>";
html+="var s=document.getElementById('slider');";
html+="var v=document.getElementById('val');";
html+="s.oninput=function(){";
html+="v.innerHTML=this.value;";
html+="fetch('/setPWM?value='+this.value);";
html+="}";
html+="</script>";

button{
padding:15px;
font-size:18px;
}

<button onclick="buzz()">Activate Buzz</button>
/* alarms */

html+="<h3>Add Alarm</h3>";
html+="<form action='/addAlarm'>";
html+="Hour <input type='number' name='h'>";
html+="Minute <input type='number' name='m'>";
html+="<input type='submit'>";
html+="</form>";


/* rtc */

html+="<h3>Set RTC Time</h3>";
html+="<form action='/setTime'>";
html+="Hour <input name='h'>";
html+="Minute <input name='m'>";
html+="Second <input name='s'>";
html+="<input type='submit'>";
html+="</form>";

/* live update */

html+="<script>";

html+="setInterval(function(){";

html+="fetch('/data')";
html+=".then(response=>response.json())";
html+=".then(data=>{";

html+="document.getElementById('time').innerHTML=data.time;";
html+="document.getElementById('date').innerHTML=data.date;";
html+="document.getElementById('temp').innerHTML=data.temp;";
html+="document.getElementById('hum').innerHTML=data.hum;";
html+="document.getElementById('sleep').innerHTML=data.sleep;";

html+="});";

html+="},1000);";

html+="</script>";

html+="</body></html>";

function buzz(){

fetch("/buzz");

}

server.send(200,"text/html",html);

}
