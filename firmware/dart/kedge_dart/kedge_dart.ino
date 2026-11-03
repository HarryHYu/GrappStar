// KEDGE dart firmware v4  (Seeed XIAO ESP32S3)
// Control logic lives in kedge_logic.h (unit-tested on a PC: ../test/test_logic.cpp). This file is the hardware glue.
//
// v4 (3 Nov 2026): DRV8833 #3 is no longer on the dart. DRV#1 A = deploy, DRV#1 B = kick, DRV#2 A = BOTH drill motors
// (right drill leads swapped so the augers counter-rotate), DRV#2 B = foot. New API: /release, /flightlog, /arm, and
// "phase" + "fault" in /status. Full API: ../API.md. Wiring: ../WIRING.md.
//
// Keyboard control, two ways:
//   1. USB:  Arduino Serial Monitor, 115200 baud, "No line ending". Type a key, press Enter (each press runs 1.5 s).
//   2. WiFi: join "KEDGE" (password grappstar), open http://192.168.4.1 . Hold a key or button to run, release to stop.
// Keys:  1 / !  deploy pods down / up          2 / @  kick leg out / back
//        3 / #  drill augers in / out          4 / $  foot auger in / out
//        space or s  STOP ALL   + / -  speed   i  current   b  attitude   z  I2C scan   p  status   f  flight log
//        a  auto-anchor now     r  release now     A  arm / disarm (auto-anchor on landing)     h  help
// Board: "XIAO_ESP32S3", Tools > USB CDC On Boot: Enabled. Pull the power loop plug before plugging in USB.
// Safety (trip, stall, deadman timers, sequences, flight detection) and all I2C run in their own FreeRTOS task every
// 5 ms under a task watchdog, so a slow web client or a stuck Serial port cannot stop the motors from being stopped.
// Wireless upload: Arduino IDE > Port > network port "kedge" while joined to KEDGE.
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Adafruit_BNO08x.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include "kedge_logic.h"
#if !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot: Enabled. With it disabled, Serial uses GPIO43/44 = D6/D7, the foot motor pins."
#endif

using kedge::PINS;
static_assert(PINS[kedge::DEPLOY].gpioA == D0 && PINS[kedge::DEPLOY].gpioB == D1, "deploy pins");
static_assert(PINS[kedge::KICK].gpioA == D2 && PINS[kedge::KICK].gpioB == D3, "kick pins");
static_assert(PINS[kedge::DRILLS].gpioA == D8 && PINS[kedge::DRILLS].gpioB == D9, "drill pins");
static_assert(PINS[kedge::FOOT].gpioA == D6 && PINS[kedge::FOOT].gpioB == D7, "foot pins");
static_assert(kedge::PIN_SDA_GPIO == D4 && kedge::PIN_SCL_GPIO == D5, "I2C pins");
const int PWM_HZ = 20000, PWM_BITS = 8;

Adafruit_BNO08x bno(-1);
bool inaPresent = false, bnoPresent = false;
float roll = 0, pitch = 0;
float quat[4] = {1, 0, 0, 0}, accV[3] = {0, 0, 9.81f};
uint32_t lastImuMs = 0, lastImuRetry = 0;
WebServer web(80);
kedge::Kedge K;
volatile float railCache = -1;
volatile bool linkLostFlag = false;
const char* resetName = "";
String reply;
SemaphoreHandle_t mtx;                                 // guards K, I2C and the log ring
char ring[8][64]; volatile int ringHead = 0, ringTail = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmAttach(int pin, int) { ledcAttach(pin, PWM_HZ, PWM_BITS); }
void pwmWrite(int pin, int ch, int duty) { (void)ch; ledcWrite(pin, duty); }
#else
void pwmAttach(int pin, int ch) { ledcSetup(ch, PWM_HZ, PWM_BITS); ledcAttachPin(pin, ch); }
void pwmWrite(int pin, int ch, int duty) { (void)pin; ledcWrite(ch, duty); }
#endif

void hwSetMotor(int m, int dir, int duty) {          // DRV8833: PWM one input, other low; both low = coast
  pwmWrite(PINS[m].gpioA, 2 * m, dir > 0 ? duty : 0);
  pwmWrite(PINS[m].gpioB, 2 * m + 1, dir < 0 ? duty : 0);
}

// ---- INA219, read directly so a failed read is seen (the library returns stale numbers silently).
// 0.1 ohm shunt, 32 V range, PGA /8 (320 mV = 3.2 A), current LSB 0.1 mA (calibration 4096), shunt ADC 4-sample average.
const uint8_t INA_ADDR = 0x40; const uint16_t INA_CAL = 4096, INA_CONFIG = 0x39D7;
bool inaWrite(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(INA_ADDR); Wire.write(reg); Wire.write(v >> 8); Wire.write(v & 0xFF); return Wire.endTransmission() == 0;
}
bool inaRead(uint8_t reg, uint16_t& v) {
  Wire.beginTransmission(INA_ADDR); Wire.write(reg); if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(INA_ADDR, (uint8_t)2) != 2) return false;
  v = (uint16_t)(Wire.read() << 8); v |= (uint16_t)Wire.read(); return true;
}
bool inaInit() { return inaWrite(0x00, INA_CONFIG) && inaWrite(0x05, INA_CAL); }
float hwCurrent() {                                  // tick task, mtx held: NAN = read failed, 9999 = overflow (trips)
  uint16_t bus, r;
  if (!inaRead(0x02, bus) || !inaRead(0x04, r)) { railCache = -1; return NAN; }
  railCache = (bus >> 3) * 0.004f;
  if (bus & 1) return 9999;                          // OVF: shunt over its 3.2 A range, the current register is meaningless
  return (int16_t)r / 10.0f;
}
float hwRail() { return railCache; }
void inaCheckCal() {                                 // 10 Hz: an INA219 that browned out comes back uncalibrated (reads 0 mA)
  uint16_t cal;
  if (inaRead(0x05, cal) && cal != INA_CAL) { inaInit(); hwLog("INA219 lost its setup: re-initialised"); }
}

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

void enableImuReports() {
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);  // 50 Hz attitude for the flight log
  bno.enableReport(SH2_ACCELEROMETER, 2500);          // 400 Hz so a short launch or impact spike is seen
}
// Every BNO085 report goes through this callback (the library's getSensorEvent keeps only the last report of each
// packet, which can drop 400 Hz accelerometer samples). Runs inside sh2_service(): tick task, mtx held.
void imuEvent(void*, sh2_SensorEvent_t* e) {
  sh2_SensorValue_t v;
  if (sh2_decodeSensorEvent(&v, e) != SH2_OK) return;
  uint32_t now = millis();
  if (v.sensorId == SH2_GAME_ROTATION_VECTOR) {
    float r = v.un.gameRotationVector.real, i = v.un.gameRotationVector.i, j = v.un.gameRotationVector.j, k = v.un.gameRotationVector.k;
    if (!(r == r && i == i && j == j && k == k)) return;
    lastImuMs = now;
    quat[0] = r; quat[1] = i; quat[2] = j; quat[3] = k;
    roll = atan2f(2 * (r * i + j * k), 1 - 2 * (i * i + j * j)) * 57.2958f;
    pitch = asinf(constrain(2 * (r * j - k * i), -1.0f, 1.0f)) * 57.2958f;
  } else if (v.sensorId == SH2_ACCELEROMETER) {
    float x = v.un.accelerometer.x, y = v.un.accelerometer.y, z = v.un.accelerometer.z;
    if (!(x == x && y == y && z == z)) return;
    lastImuMs = now;
    accV[0] = x; accV[1] = y; accV[2] = z;
    K.onAccel(sqrtf(x * x + y * y + z * z) / 9.80665f, now);
  }
}
void readImu(uint32_t now) {                          // tick task, mtx held
  if (!bnoPresent) return;
  if (bno.wasReset()) enableImuReports();
  for (int i = 0; i < 3; i++) sh2_service();          // calls imuEvent for every report waiting (one transfer per call)
  bool fresh = now - lastImuMs < 1000;
  if (fresh != K.imuOk) { K.imuOk = fresh; hwLog(fresh ? "BNO085 data back" : "BNO085 silent for 1 s"); }
  if (fresh) K.imuSample(now, quat, accV);
  else if (now - lastImuRetry > 2000) { lastImuRetry = now; enableImuReports(); }   // short I2C writes, never a re-begin here
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
  say("KEDGE v4 ready. 1/! deploy down/up  2/@ kick out/back  3/# drills in/out  4/$ foot in/out");
  say("space/s STOP  +/- speed  i current  b attitude  z I2C scan  p status  f flight log  a auto-anchor  r release  A arm");
}

// runs a key under the lock; returns the refusal reason or nullptr
const char* handleKey(char c, uint32_t runMs) {
  uint32_t now = millis(); const char* why = nullptr;
  switch (c) {
    case 'i': say("rail " + String(railCache, 2) + " V  " + String(K.lastMa, 0) + " mA  peak " + String(K.peakMa, 0) + " mA since the last i");
              xSemaphoreTake(mtx, portMAX_DELAY); K.peakMa = 0; xSemaphoreGive(mtx); return nullptr;
    case 'b': say(K.imuOk ? "roll " + String(roll, 1) + "  pitch " + String(pitch, 1) + "  |a| " + String(K.accG, 2) + " g" : String("BNO085 not found or silent")); return nullptr;
    case 'z': xSemaphoreTake(mtx, portMAX_DELAY); K.stop(now); xSemaphoreGive(mtx); scanI2C(); return nullptr;   // motors stopped first
    case 'p': { char b[1024]; xSemaphoreTake(mtx, portMAX_DELAY); K.status(b, sizeof b, roll, pitch, now, railCache, resetName); xSemaphoreGive(mtx); say(b); return nullptr; }
    case 'f': { xSemaphoreTake(mtx, portMAX_DELAY); int n = K.log.count; bool fr = K.log.frozen; uint32_t tl = K.flight.tLaunch, td = K.flight.tLand; bool la = K.flight.launched, ld = K.landed; xSemaphoreGive(mtx);
                say("flight log: " + String(n) + " samples, " + (fr ? "frozen" : "live") + (la ? ", launch " + String(tl) + " ms" : String("")) + (ld ? ", landed " + String(td) + " ms" : String("")) + ". Full log: GET /flightlog");
                return nullptr; }
    case 'h': case '?': help(); return nullptr;
    case '\n': case '\r': return nullptr;
    default: xSemaphoreTake(mtx, portMAX_DELAY); why = K.key(c, runMs, now); xSemaphoreGive(mtx);
  }
  if (c == '+' || c == '-') say("speed " + String(K.spd));
  if (why) say(String("refused: ") + why);
  drainLog();
  return why;
}

void tickTask(void*) {                                 // 5 ms: IMU, flight, current, trip, stall, timers, sequences
  esp_task_wdt_add(nullptr);
  TickType_t last = xTaskGetTickCount();
  uint32_t lastV = 0;
  for (;;) {
    uint32_t now = millis();
    xSemaphoreTake(mtx, portMAX_DELAY);                // all I2C traffic happens inside this lock
    readImu(now);
    if (now - lastV >= 100) { lastV = now; inaCheckCal(); }   // also sets up an INA219 that was missing at boot
    if (linkLostFlag) { linkLostFlag = false; K.linkLost(now); }
    K.tick(now);
    xSemaphoreGive(mtx);
    esp_task_wdt_reset();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
  }
}

void setupWatchdog() {                                 // 2 s without a tick = panic reset (motors coast: DRV8833 inputs pull down)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t c = {};
  c.timeout_ms = 2000; c.idle_core_mask = 0; c.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&c) != ESP_OK) esp_task_wdt_init(&c);
#else
  esp_task_wdt_init(2, true);
#endif
}

int resetFault() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: resetName = "poweron"; return kedge::F_NONE;
    case ESP_RST_SW: resetName = "software"; return kedge::F_NONE;   // OTA, ESP.restart()
    case ESP_RST_BROWNOUT: resetName = "brownout"; return kedge::F_BROWNOUT;
    case ESP_RST_PANIC: resetName = "panic"; return kedge::F_WATCHDOG;
    case ESP_RST_INT_WDT: resetName = "int_wdt"; return kedge::F_WATCHDOG;
    case ESP_RST_TASK_WDT: resetName = "task_wdt"; return kedge::F_WATCHDOG;
    case ESP_RST_WDT: resetName = "wdt"; return kedge::F_WATCHDOG;
    case ESP_RST_EXT: resetName = "reset_pin"; return kedge::F_NONE;
    case ESP_RST_DEEPSLEEP: resetName = "deepsleep"; return kedge::F_NONE;
    default: resetName = "other"; return kedge::F_NONE;   // USB upload resets land here
  }
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>KEDGE</title><style>body{font:15px Arial;margin:16px;background:#111;color:#eee}button{font:15px Arial;padding:14px;margin:4px;min-width:120px;touch-action:none;user-select:none;-webkit-user-select:none}
#s{font:13px monospace;white-space:pre-wrap;background:#222;padding:8px}pre{background:#222;padding:8px;height:30vh;overflow:auto}a{color:#8cf}</style></head><body><h3>KEDGE dart v4</h3>
<p>Hold a key or button to run, release to stop. 1/! deploy  2/@ kick  3/# drills  4/$ foot  space STOP  a auto  r release  A arm. <a href=/flightlog download=flightlog.json>flight log</a></p>
<div id=b></div><div id=s></div><pre id=o></pre><script>
const o=document.getElementById('o'),S=document.getElementById('s');const held=new Map();
function add(x){if(x){o.textContent+=x;o.scrollTop=1e9}}
let N=0;const busy={};const P=Math.random().toString(36).slice(2,8);
function send(c){const n=++N;if(c!==' '&&busy[c])return;busy[c]=1;const ac=new AbortController();const to=setTimeout(()=>ac.abort(),300);
fetch('/key?c='+encodeURIComponent(c)+'&ms=350&n='+n+'&p='+P,{signal:ac.signal}).then(r=>r.text()).then(add).catch(()=>{}).finally(()=>{clearTimeout(to);busy[c]=0})}
setInterval(()=>{for(const c of held.values())send(c)},150);   // several keys can be held at once (e.g. 2 and 4)
function hold(id,c){if(held.has(id))return;held.set(id,c);send(c)}
function release(id){if(!held.has(id))return;held.delete(id);if(!held.size)send(' ')}
const RUN='1!2@3#4$';
addEventListener('keydown',e=>{if(e.key===' '||e.key==='Enter')e.preventDefault();const c=e.key===' '?' ':e.key;if(RUN.includes(c)){e.preventDefault();hold(e.code,c)}else if(e.key==='Shift'||e.key==='Enter'){}else if(!e.repeat&&c.length===1){if(c===' ')held.clear();send(c)}});
addEventListener('keyup',e=>{if(e.key===' '||e.key==='Enter')e.preventDefault();if(held.has(e.code))release(e.code)});   // keyed by physical key: Shift order does not matter
function lost(){if(held.size){held.clear();send(' ')}}
addEventListener('blur',lost);addEventListener('pagehide',lost);document.addEventListener('visibilitychange',()=>{if(document.hidden)lost()});
addEventListener('contextmenu',e=>e.preventDefault());
const L=[['1','deploy down'],['!','deploy up'],['2','kick out'],['@','kick back'],['3','drills in'],['#','drills out'],['4','foot in'],['$','foot out']];
const B=document.getElementById('b');for(const [c,n] of L){const x=document.createElement('button');x.textContent=n;
x.tabIndex=-1;const id='btn'+c;x.onpointerdown=()=>hold(id,c);x.onpointerup=()=>release(id);x.onpointerleave=()=>release(id);x.onpointercancel=()=>release(id);B.appendChild(x)}
for(const [c,n] of [[' ','STOP'],['a','AUTO anchor'],['r','RELEASE'],['A','arm / disarm'],['i','current'],['b','attitude']]){const x=document.createElement('button');x.textContent=n;x.tabIndex=-1;x.onclick=()=>{send(c);x.blur()};B.appendChild(x)}
setInterval(()=>fetch('/status').then(r=>r.json()).then(j=>{S.textContent=`${j.phase}${j.fault?' '+j.fault:''}   ${j.seq}\nrail ${j.rail_V} V  ${j.current_mA} mA (peak ${j.peak_mA})  roll ${j.roll}  pitch ${j.pitch}  ${j.accel_g} g\nINA219 ${j.ina_ok?'ok':'MISSING'}  BNO085 ${j.imu_ok?'ok':'MISSING'}  ${j.armed?'ARMED':''}  last: ${j.event}`}).catch(()=>{}),400);
</script></body></html>)HTML";

void sendResult(const char* why) {
  if (why) web.send(409, "text/plain", String("refused: ") + why + "\n" + reply);
  else web.send(200, "text/plain", reply);
}

void handleFlightlog() {                               // streamed from a snapshot so the tick task is held for microseconds only
  // while sending (about 0.3 s, longer on a weak link) the server cannot take /stop, so refuse while anything moves
  // also while armed or in flight: the landing detector could start the motors in the middle of the stream
  xSemaphoreTake(mtx, portMAX_DELAY); bool busy = K.running() || K.seqRunning() || K.armed || K.flight.flying(); xSemaphoreGive(mtx);
  if (busy) { web.send(409, "text/plain", "refused: busy (motors running, armed or in flight)\n"); return; }
  kedge::Log* snap = (kedge::Log*)malloc(sizeof(kedge::Log));
  if (!snap) { web.send(503, "text/plain", "out of memory\n"); return; }
  kedge::LogJson w;
  xSemaphoreTake(mtx, portMAX_DELAY);
  memcpy(snap, &K.log, sizeof(kedge::Log));
  w.begin(snap, kedge::phaseName(K.phase()), K.flight.launched, K.flight.tLaunch, K.flight.st == kedge::Flight::LANDED, K.flight.tLand, K.flight.how);
  xSemaphoreGive(mtx);
  web.setContentLength(CONTENT_LENGTH_UNKNOWN);
  web.send(200, "application/json", "");
  static char buf[1400]; int n;
  while ((n = w.chunk(buf, sizeof buf)) > 0) web.sendContent(buf, n);
  web.sendContent("");
  free(snap);
}

void setup() {
  for (int m = 0; m < kedge::NMOT; m++) {             // motors safe before anything else
    pinMode(PINS[m].gpioA, OUTPUT); digitalWrite(PINS[m].gpioA, LOW);
    pinMode(PINS[m].gpioB, OUTPUT); digitalWrite(PINS[m].gpioB, LOW);
  }
  for (int m = 0; m < kedge::NMOT; m++) { pwmAttach(PINS[m].gpioA, 2 * m); pwmAttach(PINS[m].gpioB, 2 * m + 1); hwSetMotor(m, 0, 0); }
  mtx = xSemaphoreCreateMutex();
  K.io = {hwSetMotor, hwCurrent, hwRail, hwLog};
  int rf = resetFault();
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);                            // never block on a full USB buffer
#endif
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 2500) delay(10);
  Wire.begin(kedge::PIN_SDA_GPIO, kedge::PIN_SCL_GPIO);
  Wire.setClock(400000);
  Wire.setTimeOut(20);                                 // a stuck bus costs at most 20 ms per transfer
  inaPresent = inaInit();
  K.inaOk = inaPresent;
  if (!inaPresent) Serial.println("INA219 not found: no current sensing, auto and release refused, manual keys still work");
  bnoPresent = bno.begin_I2C(0x4A) || bno.begin_I2C(0x4B);
  if (bnoPresent) { sh2_setSensorCallback(imuEvent, nullptr); enableImuReports(); }
  else Serial.println("BNO085 not found: no attitude, no flight log, arming refused");
  K.imuOk = bnoPresent; lastImuMs = millis();
  K.boot(rf, millis());
  Serial.printf("reset reason: %s\n", resetName);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("KEDGE", "grappstar");
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) { linkLostFlag = true; }, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
  web.on("/", []() { web.send_P(200, "text/html", PAGE); });
  web.on("/key", []() {
    reply = "";
    String c = web.arg("c"); long ms = web.hasArg("ms") ? web.arg("ms").toInt() : K.tu.wifiDeadmanMs;
    static long lastStopN = -1; static String lastPage; long n = web.hasArg("n") ? web.arg("n").toInt() : -1;   // the page numbers every request
    String pg = web.arg("p"); if (pg != lastPage) { lastPage = pg; lastStopN = -1; }   // a reloaded page starts counting again
    if (c == " " || c == "s") { if (n > lastStopN) lastStopN = n; }
    else if (n >= 0 && n < lastStopN) { web.send(200, "text/plain", ""); return; }      // a run key that left before the stop: drop it
    if (ms < 50) ms = 50;
    if (ms > (long)K.tu.wifiMaxRunMs) ms = K.tu.wifiMaxRunMs;
    if (!c.length()) { web.send(400, "text/plain", "missing c\n"); return; }
    sendResult(handleKey(c[0], (uint32_t)ms));
  });
  web.on("/stop", []() {
    reply = "";
    if (web.hasArg("disarm") && web.arg("disarm") == "0") { xSemaphoreTake(mtx, portMAX_DELAY); K.stop(millis(), false); xSemaphoreGive(mtx); drainLog(); sendResult(nullptr); }
    else sendResult(handleKey(' ', 0));
  });
  web.on("/auto", []() { reply = ""; sendResult(handleKey('a', 0)); });
  web.on("/release", []() { reply = ""; sendResult(handleKey('r', 0)); });
  web.on("/arm", []() {
    reply = ""; bool on = !web.hasArg("on") || web.arg("on") != "0"; bool withAuto = !web.hasArg("auto") || web.arg("auto") != "0";
    xSemaphoreTake(mtx, portMAX_DELAY); const char* why = K.arm(on, withAuto, millis()); xSemaphoreGive(mtx);
    drainLog(); sendResult(why);
  });
  web.on("/status", []() { char b[1024]; xSemaphoreTake(mtx, portMAX_DELAY); K.status(b, sizeof b, roll, pitch, millis(), railCache, resetName); xSemaphoreGive(mtx); web.send(200, "application/json", b); });
  web.on("/flightlog", handleFlightlog);
  web.onNotFound([]() { web.send(404, "text/plain", "not found\n"); });
  web.begin();
  ArduinoOTA.setHostname("kedge");
  ArduinoOTA.onStart([]() { xSemaphoreTake(mtx, portMAX_DELAY); K.stop(millis()); xSemaphoreGive(mtx); });   // abort any sequence and disarm
  ArduinoOTA.begin();
  setupWatchdog();
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
