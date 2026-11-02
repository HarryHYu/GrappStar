// KEDGE dart firmware v3  (Seeed XIAO ESP32S3)
// Control logic lives in kedge_logic.h (unit-tested on a PC: fw/test/test_logic.cpp). This file is the hardware glue.
//
// Keyboard control, two ways:
//   1. USB:  Arduino Serial Monitor, 115200 baud, "No line ending". Type a key, press Enter (each press runs 1.5 s).
//   2. WiFi: join "KEDGE" (password grappstar), open http://192.168.4.1 . Hold a key or button to run, release to stop.
// Keys:  1 / !  deploy pods down / up          2 / @  kick leg out / back
//        3 / #  drill augers in / out          4 / $  foot auger in / out
//        space or s  STOP ALL   + / -  speed   i  current   b  attitude   z  I2C scan
//        a  auto-anchor now     A  arm/disarm auto-anchor on landing     h  help
// Rover / Jetson API (same WiFi):  GET /status (JSON)   GET /key?c=<key>&ms=<run ms>   GET /stop   GET /auto
// Board: "XIAO_ESP32S3", Tools > USB CDC On Boot: Enabled. Pull the power loop plug before plugging in USB.
// Safety (trip, deadman timers, auto sequence) and all I2C run in their own FreeRTOS task every 5 ms, so a slow web
// client or a stuck Serial port cannot stop the motors from being stopped.
// Wireless upload: Arduino IDE > Port > network port "kedge" while joined to KEDGE.
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BNO08x.h>
#include "kedge_logic.h"
#if !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot: Enabled. With it disabled, Serial uses GPIO43/44 = D6/D7, the foot motor pins."
#endif

// D6/D7 are GPIO43/44 = UART0 TX/RX: the boot ROM toggles TX at every reset, before setup() runs. They drive the FOOT
// (slow worm: a reset twitch is harmless); the fast drill pair is on D8/D9.
const int PIN_A[4] = {D0, D2, D8, D6};   // deploy, kick, drills (DRV#2 A+B tied), foot
const int PIN_B[4] = {D1, D3, D9, D7};
const int PIN_SDA = D4, PIN_SCL = D5;
const int PWM_HZ = 20000, PWM_BITS = 8;

Adafruit_INA219 ina;
Adafruit_BNO08x bno(-1);
sh2_SensorValue_t sv;
bool inaOk = false, bnoOk = false;
float roll = 0, pitch = 0, accG = 1.0;
WebServer web(80);
kedge::Kedge K;
kedge::Landing landing;
bool landed = false;
volatile float railCache = 0;
String reply;
SemaphoreHandle_t mtx;                                 // guards K and the log ring
char ring[8][64]; volatile int ringHead = 0, ringTail = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmAttach(int pin, int) { ledcAttach(pin, PWM_HZ, PWM_BITS); }
void pwmWrite(int pin, int ch, int duty) { (void)ch; ledcWrite(pin, duty); }
#else
void pwmAttach(int pin, int ch) { ledcSetup(ch, PWM_HZ, PWM_BITS); ledcAttachPin(pin, ch); }
void pwmWrite(int pin, int ch, int duty) { (void)pin; ledcWrite(ch, duty); }
#endif

void hwSetMotor(int m, int dir, int duty) {          // DRV8833: PWM one input, other low; both low = coast
  pwmWrite(PIN_A[m], 2 * m, dir > 0 ? duty : 0);
  pwmWrite(PIN_B[m], 2 * m + 1, dir < 0 ? duty : 0);
}
float hwCurrent() { return inaOk ? ina.getCurrent_mA() : 0; }
float hwRail() { return inaOk ? ina.getBusVoltage_V() : 0; }
void hwLog(const char* s) {                          // called with mtx held; drained to Serial / web by loop()
  strncpy(ring[ringHead], s, 63); ring[ringHead][63] = 0; ringHead = (ringHead + 1) % 8; if (ringHead == ringTail) ringTail = (ringTail + 1) % 8;
}
void drainLog() {
  while (true) {
    char line[64];
    xSemaphoreTake(mtx, portMAX_DELAY);
    bool have = ringTail != ringHead; if (have) { memcpy(line, ring[ringTail], 64); ringTail = (ringTail + 1) % 8; }
    xSemaphoreGive(mtx);
    if (!have) break;
    Serial.println(line); reply += line; reply += "\n";
  }
}
void say(const String& s) { Serial.println(s); reply += s; reply += "\n"; }

void readImu() {
  if (!bnoOk) return;
  if (bno.wasReset()) { bno.enableReport(SH2_GAME_ROTATION_VECTOR, 20000); bno.enableReport(SH2_ACCELEROMETER, 2500); }
  while (bno.getSensorEvent(&sv)) {
    if (sv.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float r = sv.un.gameRotationVector.real, i = sv.un.gameRotationVector.i, j = sv.un.gameRotationVector.j, k = sv.un.gameRotationVector.k;
      roll = atan2f(2 * (r * i + j * k), 1 - 2 * (i * i + j * j)) * 57.2958f;
      pitch = asinf(constrain(2 * (r * j - k * i), -1.0f, 1.0f)) * 57.2958f;
    } else if (sv.sensorId == SH2_ACCELEROMETER) {
      float ax = sv.un.accelerometer.x, ay = sv.un.accelerometer.y, az = sv.un.accelerometer.z;
      accG = sqrtf(ax * ax + ay * ay + az * az) / 9.81f;
    }
  }
}

void scanI2C() {
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    xSemaphoreTake(mtx, portMAX_DELAY); Wire.beginTransmission(a); bool ok = Wire.endTransmission() == 0; xSemaphoreGive(mtx);
    if (ok) { say("I2C device at 0x" + String(a, HEX)); n++; }
  }
  say(String(n) + " device(s). Expect 0x40 (INA219) and 0x4a (BNO085).");
}
void help() {
  say("KEDGE ready. 1/! deploy down/up  2/@ kick out/back  3/# drills in/out  4/$ foot in/out");
  say("space/s STOP  +/- speed  i current  b attitude  z I2C scan  a auto-anchor  A arm auto on landing");
}

void handleKey(char c, uint32_t runMs) {
  uint32_t now = millis();
  if (c == 'A') { xSemaphoreTake(mtx, portMAX_DELAY); landing.reset(); landed = false; xSemaphoreGive(mtx); }   // re-arm the landing detector
  switch (c) {
    case 'i': say("rail " + String(railCache, 2) + " V  " + String(K.lastMa, 0) + " mA"); return;
    case 'b': say(bnoOk ? "roll " + String(roll, 1) + "  pitch " + String(pitch, 1) + "  |a| " + String(accG, 2) + " g" : String("BNO085 not found")); return;
    case 'z': xSemaphoreTake(mtx, portMAX_DELAY); K.key(' ', 0, now); xSemaphoreGive(mtx); scanI2C(); return;   // motors stopped first; lock taken per address
    case 'h': case '?': help(); return;
    case '+': case '-': xSemaphoreTake(mtx, portMAX_DELAY); K.key(c, runMs, now); xSemaphoreGive(mtx); say("speed " + String(K.spd)); return;
    default: xSemaphoreTake(mtx, portMAX_DELAY); K.key(c, runMs, now); xSemaphoreGive(mtx);
  }
  drainLog();
}

void tickTask(void*) {                                 // 5 ms: IMU, landing, current, trip, timers, auto sequence
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    uint32_t now = millis();
    xSemaphoreTake(mtx, portMAX_DELAY);                // all I2C traffic happens inside this lock
    readImu();
    static uint32_t lastV = 0; if (now - lastV > 100) { lastV = now; railCache = hwRail(); }
    if (!landed && landing.update(accG, now)) { landed = true; hwLog("landed"); if (K.armedOnLanding) K.startAuto(now); }
    K.tick(now);
    xSemaphoreGive(mtx);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
  }
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>KEDGE</title><style>body{font:15px Arial;margin:16px;background:#111;color:#eee}button{font:15px Arial;padding:14px;margin:4px;min-width:120px;touch-action:none;user-select:none;-webkit-user-select:none}
#s{font:13px monospace;white-space:pre-wrap;background:#222;padding:8px}pre{background:#222;padding:8px;height:30vh;overflow:auto}</style></head><body><h3>KEDGE dart</h3>
<p>Hold a key or button to run, release to stop. 1/! deploy  2/@ kick  3/# drills  4/$ foot  space STOP  a auto  A arm-on-landing</p>
<div id=b></div><div id=s></div><pre id=o></pre><script>
const o=document.getElementById('o'),S=document.getElementById('s');const held=new Map();
function add(x){if(x){o.textContent+=x;o.scrollTop=1e9}}
let N=0;const busy={};
function send(c){const n=++N;if(c!==' '&&busy[c])return;busy[c]=1;const ac=new AbortController();const to=setTimeout(()=>ac.abort(),300);
fetch('/key?c='+encodeURIComponent(c)+'&ms=350&n='+n,{signal:ac.signal}).then(r=>r.text()).then(add).catch(()=>{}).finally(()=>{clearTimeout(to);busy[c]=0})}
setInterval(()=>{for(const c of held.values())send(c)},150);   // several keys can be held at once (e.g. 2 and 4)
function hold(id,c){if(held.has(id))return;held.set(id,c);send(c)}
function release(id){if(!held.has(id))return;held.delete(id);if(!held.size)send(' ')}
const RUN='1!2@3#4$';
addEventListener('keydown',e=>{if(e.key===' '||e.key==='Enter')e.preventDefault();const c=e.key===' '?' ':e.key;if(RUN.includes(c)){e.preventDefault();hold(e.code,c)}else if(e.key==='Shift'||e.key==='Enter'){}else if(!e.repeat){if(c===' ')held.clear();send(c)}});
addEventListener('keyup',e=>{if(e.key===' '||e.key==='Enter')e.preventDefault();if(held.has(e.code))release(e.code)});   // keyed by physical key: Shift order does not matter
function lost(){if(held.size){held.clear();send(' ')}}
addEventListener('blur',lost);addEventListener('pagehide',lost);document.addEventListener('visibilitychange',()=>{if(document.hidden)lost()});
addEventListener('contextmenu',e=>e.preventDefault());
const L=[['1','deploy down'],['!','deploy up'],['2','kick out'],['@','kick back'],['3','drills in'],['#','drills out'],['4','foot in'],['$','foot out']];
const B=document.getElementById('b');for(const [c,n] of L){const x=document.createElement('button');x.textContent=n;
x.tabIndex=-1;const id='btn'+c;x.onpointerdown=()=>hold(id,c);x.onpointerup=()=>release(id);x.onpointerleave=()=>release(id);x.onpointercancel=()=>release(id);B.appendChild(x)}
for(const [c,n] of [[' ','STOP'],['a','AUTO'],['A','arm on landing'],['i','current'],['b','attitude']]){const x=document.createElement('button');x.textContent=n;x.tabIndex=-1;x.onclick=()=>{send(c);x.blur()};B.appendChild(x)}
setInterval(()=>fetch('/status').then(r=>r.json()).then(j=>{S.textContent=`rail ${j.rail_V} V  ${j.current_mA} mA (peak ${j.peak_mA})  roll ${j.roll}  pitch ${j.pitch}\n${j.seq}  ${j.armed?'ARMED':''} ${j.landed?'LANDED':''}  last: ${j.event}`}).catch(()=>{}),400);
</script></body></html>)HTML";

void setup() {
  for (int m = 0; m < 4; m++) { pwmAttach(PIN_A[m], 2 * m); pwmAttach(PIN_B[m], 2 * m + 1); hwSetMotor(m, 0, 0); }   // motors safe before anything else
  mtx = xSemaphoreCreateMutex();
  K.io = {hwSetMotor, hwCurrent, hwRail, hwLog};
  K.stopAll(millis());
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);                            // never block on a full USB buffer
#endif
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 2500) delay(10);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  inaOk = ina.begin();
  if (!inaOk) Serial.println("INA219 not found");
  bnoOk = bno.begin_I2C(0x4A) || bno.begin_I2C(0x4B);
  if (bnoOk) { bno.enableReport(SH2_GAME_ROTATION_VECTOR, 20000); bno.enableReport(SH2_ACCELEROMETER, 2500); }   // 400 Hz so a short impact spike is seen
  else Serial.println("BNO085 not found");
  WiFi.mode(WIFI_AP);
  WiFi.softAP("KEDGE", "grappstar");
  web.on("/", []() { web.send_P(200, "text/html", PAGE); });
  web.on("/key", []() {
    reply = "";
    String c = web.arg("c"); uint32_t ms = web.hasArg("ms") ? web.arg("ms").toInt() : K.tu.wifiDeadmanMs;
    static long lastStopN = -1; long n = web.hasArg("n") ? web.arg("n").toInt() : -1;   // page numbers every request
    if (c == " " || c == "s") { if (n > lastStopN) lastStopN = n; }
    else if (n >= 0 && n < lastStopN) { web.send(200, "text/plain", ""); return; }      // a run key that left before the stop: drop it
    if (ms < 50) ms = 50; if (ms > 5000) ms = 5000;
    if (c.length()) handleKey(c[0], ms);
    web.send(200, "text/plain", reply);
  });
  web.on("/stop", []() { reply = ""; handleKey(' ', 0); web.send(200, "text/plain", reply); });
  web.on("/auto", []() { reply = ""; handleKey('a', 0); web.send(200, "text/plain", reply); });
  web.on("/status", []() { char b[400]; xSemaphoreTake(mtx, portMAX_DELAY); K.status(b, sizeof b, roll, pitch, landed, millis(), railCache); xSemaphoreGive(mtx); web.send(200, "application/json", b); });
  web.begin();
  ArduinoOTA.setHostname("kedge");
  ArduinoOTA.onStart([]() { xSemaphoreTake(mtx, portMAX_DELAY); K.key(' ', 0, millis()); K.armedOnLanding = false; xSemaphoreGive(mtx); });   // abort any sequence too
  ArduinoOTA.begin();
  xTaskCreatePinnedToCore(tickTask, "kedge_tick", 6144, nullptr, 3, nullptr, 1);
  help();
  Serial.print("WiFi: join KEDGE, open http://"); Serial.println(WiFi.softAPIP());
}

void loop() {
  if (Serial.available()) { char c = Serial.read(); if (c != '\n' && c != '\r') { reply = ""; handleKey(c, K.tu.serialRunMs); } }
  web.handleClient();
  ArduinoOTA.handle();
  reply = ""; drainLog();                              // print sequence events from the tick task
  delay(1);
}
