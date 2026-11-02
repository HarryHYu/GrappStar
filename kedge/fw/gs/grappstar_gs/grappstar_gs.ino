// GrappStar ground station firmware v1 (ESP32-S3 DevKitC-1): launcher + reel on ONE wiper motor through a BTS7960.
// Control logic lives in gs_logic.h (unit-tested on a PC: fw/test/test_gs.cpp). This file is the hardware glue.
// Mechanism and pins: gs/ARCHITECTURE.md. Motor + (RPWM) = F = reel wind. Motor - (LPWM) = B = launcher program gear.
//
// Keyboard control, two ways:
//   1. USB:  Arduino Serial Monitor, 115200 baud, "No line ending". Type a key, press Enter (f/b run 1.5 s per press).
//   2. WiFi: join "GRAPPSTAR-GS" (password grappstar), open http://192.168.5.1 . (The dart stays on 192.168.4.1.) Hold f or b to jog, release to stop.
// Keys:  f / b  jog reel wind / launcher (hold-to-run)     space or s  STOP     x  clear fault
//        c  cock     F F  FIRE (capital F twice in 3 s)     t  tow to the limit     r  retrieve     y y  full cycle (twice)
//        n  anchored flag     m  released flag     l<N>  tension limit in N (e.g. l20)     + / -  jog speed
//        z  zero (motor stopped: current offset + HX711 tare; jogging f on a slack line: learn free current)
//        E E  finish (twice in 3 s): run B to 338 from phi 235..338 (fires if cocked)   O  force: allow the next F with the carriage partly drawn
//        L L  lower (twice in 3 s): rail up and phi unknown only. Slow B until the rail is down, on to 338 (fires if cocked)
//        i  status line     p  status JSON     h  help
// Rover / Jetson API (same WiFi): GET /status (JSON)   GET /cmd?c=cock|fire|tow|retrieve|stop|cycle|limit|anchored|released|clear|line|zero|finish|lower|force&v=<number>
//                                 GET /key?c=f|b&ms=<run ms>   GET /stop
// Safety (current trip, deadman, stall, soft start, reversal gap, state machine) and all I2C run in their own FreeRTOS
// task every 5 ms, so a slow web client or a stuck Serial port cannot stop the motor from being stopped.
// Wireless upload: Arduino IDE > Port > network port "grappstar-gs" while joined to GRAPPSTAR-GS.
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <HX711.h>
#include <Preferences.h>
#include "gs_logic.h"

const int PIN_RPWM = 4, PIN_LPWM = 5, PIN_REN = 6, PIN_LEN = 7, PIN_IS = 1;
const int PIN_SDA = 8, PIN_SCL = 9;
const int PIN_SW_LOW = 10, PIN_SW_COCKED = 11, PIN_SW_SEATED = 12;
const int PIN_HX_DOUT = 13, PIN_HX_SCK = 14;
const int PWM_HZ = 20000, PWM_BITS = 8;
const uint8_t AS5600 = 0x36;

HX711 hx;
Preferences prefs;                                     // phi survives a power cycle (the worm holds the gear)
volatile bool saveDue = false; volatile float savePhi = 0; volatile bool saveKnown = false, saveProb = false;
bool hxOk = false, magnetOk = false;
WebServer web(80);
gs::GS G;
String reply;
SemaphoreHandle_t mtx;                                 // guards G and the log ring
char ring[8][72]; volatile int ringHead = 0, ringTail = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmAttach(int pin, int) { ledcAttach(pin, PWM_HZ, PWM_BITS); }
void pwmWrite(int pin, int ch, int duty) { (void)ch; ledcWrite(pin, duty); }
#else
void pwmAttach(int pin, int ch) { ledcSetup(ch, PWM_HZ, PWM_BITS); ledcAttachPin(pin, ch); }
void pwmWrite(int pin, int ch, int duty) { (void)pin; ledcWrite(ch, duty); }
#endif

void hwSetMotor(int dir, int duty) {                   // BTS7960: PWM one side, other side low; both low = brake
  pwmWrite(PIN_RPWM, 0, dir > 0 ? duty : 0);
  pwmWrite(PIN_LPWM, 1, dir < 0 ? duty : 0);
}
int as5600Read(uint8_t reg, int n) {                   // big-endian register read, -1 on any I2C error
  Wire.beginTransmission(AS5600); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(AS5600, (uint8_t)n) != n) return -1;
  int v = 0; for (int i = 0; i < n; i++) v = (v << 8) | Wire.read();
  return v;
}
int hwAngle() {                                        // called with mtx held (tick task)
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 50) {                    // STATUS bit 5 = magnet detected
    lastStatus = millis(); int s = as5600Read(0x0B, 1);
    magnetOk = s >= 0 && (s & 0x20);
  }
  if (!magnetOk) return -1;
  int r = as5600Read(0x0C, 2);
  return r < 0 ? -1 : (r & 0x0FFF);
}
float hwIsVolts() { uint32_t mv = 0; for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(PIN_IS); return mv / 4000.0f; }
float hwHx() { return hxOk && hx.is_ready() ? hx.read() : NAN; }
uint8_t hwSwitches() {
  return (digitalRead(PIN_SW_LOW) == LOW ? gs::SW_LOW : 0) | (digitalRead(PIN_SW_COCKED) == LOW ? gs::SW_COCKED : 0) |
         (digitalRead(PIN_SW_SEATED) == LOW ? gs::SW_SEATED : 0);
}
void hwSave(float phi, bool known, bool prob) { savePhi = phi; saveKnown = known; saveProb = prob; saveDue = true; }   // flash write happens in loop()
void hwLog(const char* s) {                            // called with mtx held; drained to Serial / web by loop()
  strncpy(ring[ringHead], s, 71); ring[ringHead][71] = 0; ringHead = (ringHead + 1) % 8; if (ringHead == ringTail) ringTail = (ringTail + 1) % 8;
}
void drainLog() {
  while (true) {
    char line[72];
    xSemaphoreTake(mtx, portMAX_DELAY);
    bool have = ringTail != ringHead; if (have) { memcpy(line, ring[ringTail], 72); ringTail = (ringTail + 1) % 8; }
    xSemaphoreGive(mtx);
    if (!have) break;
    Serial.println(line); reply += line; reply += "\n";
  }
}
void say(const String& s) { Serial.println(s); reply += s; reply += "\n"; }

void help() {
  say("GRAPPSTAR GS. f/b jog wind/launcher  space/s STOP  x clear  c cock  FF FIRE  t tow  r retrieve  yy cycle");
  say("n anchored  m released  l<N> limit  +/- jog speed  z zero  EE finish to 338  LL lower (rail up, phi unknown)  O force F  i status  p JSON  h help");
  say("FF / EE / yy / LL: the same capital key twice within 3 s (one stray byte never fires)");
}
void statusLine() {
  xSemaphoreTake(mtx, portMAX_DELAY);
  char b[200];
  snprintf(b, sizeof b, "%s  phi %.1f%s  theta %.1f  line %.2f m  %.2f A  %.1f N (%s, limit %.0f)  sw L%d C%d S%d  %s%s",
           gs::stateName(G.state), G.phi(), G.phiKnown ? "" : "?", G.theta(), G.lineOut, G.iF, G.tension, G.useHx(millis()) ? "hx711" : "current",
           G.tu.limitN, G.closed(gs::SW_LOW), G.closed(gs::SW_COCKED), G.closed(gs::SW_SEATED), G.encOk ? "" : "AS5600 MISSING ", G.fault);
  xSemaphoreGive(mtx);
  say(b);
}
void handleKey(char c, uint32_t runMs) {
  switch (c) {
    case 'i': statusLine(); return;
    case 'p': { char b[700]; xSemaphoreTake(mtx, portMAX_DELAY); G.status(b, sizeof b, millis()); xSemaphoreGive(mtx); say(b); return; }
    case 'h': case '?': help(); return;
    case 'l': { float v = Serial.parseFloat(); xSemaphoreTake(mtx, portMAX_DELAY); G.cmd("limit", v, millis()); xSemaphoreGive(mtx); break; }
    case '+': case '-': xSemaphoreTake(mtx, portMAX_DELAY); G.key(c, runMs, millis()); xSemaphoreGive(mtx); say("jog speed " + String(G.jogSpd)); return;
    default: xSemaphoreTake(mtx, portMAX_DELAY); G.key(c, runMs, millis()); xSemaphoreGive(mtx);
  }
  drainLog();
}

void savePhiNow() {                                   // flash write outside the mutex (it stalls both cores while it runs)
  xSemaphoreTake(mtx, portMAX_DELAY);
  bool due = saveDue; float p = savePhi; bool k = saveKnown, pr = saveProb; saveDue = false;
  xSemaphoreGive(mtx);
  if (due) { prefs.putFloat("phi", p); prefs.putBool("known", k); prefs.putBool("prob", pr); }
}

void tickTask(void*) {                                 // 5 ms: sensors, trip, stall, deadman, ramp, state machine
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    xSemaphoreTake(mtx, portMAX_DELAY);                // all I2C traffic happens inside this lock
    G.tick(millis(), 5);
    xSemaphoreGive(mtx);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
  }
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>GRAPPSTAR GS</title><style>body{font:15px Arial;margin:16px;background:#111;color:#eee}button{font:15px Arial;padding:14px;margin:4px;min-width:110px;touch-action:none;user-select:none;-webkit-user-select:none}
#s{font:13px monospace;white-space:pre-wrap;background:#222;padding:8px}pre{background:#222;padding:8px;height:30vh;overflow:auto}input{font:15px Arial;width:60px;padding:10px}</style></head><body><h3>GrappStar ground station</h3>
<p>Hold f or b (or the buttons) to jog, release to stop. Space = STOP. Shift+F = FIRE, Shift+E = finish, Shift+L = lower (all ask first).</p>
<div id=b></div><div>limit N <input id=lim value=20><button id=setlim>set limit</button></div><div id=s></div><pre id=o></pre><script>
const o=document.getElementById('o'),S=document.getElementById('s');const held=new Map();
function add(x){if(x){o.textContent+=x;o.scrollTop=1e9}}
let N=0;const busy={};const SID=Math.random().toString(36).slice(2,10);
function send(c){const n=++N;if(c!==' '&&busy[c])return;busy[c]=1;const ac=new AbortController();const to=setTimeout(()=>ac.abort(),300);
fetch('/key?c='+encodeURIComponent(c)+'&ms=350&n='+n+'&sid='+SID,{signal:ac.signal}).then(r=>r.text()).then(add).catch(()=>{}).finally(()=>{clearTimeout(to);busy[c]=0})}
function cmd(c,v){fetch('/cmd?c='+c+(v!==undefined?'&v='+v:'')).then(r=>r.text()).then(add).catch(()=>{})}
function fire(){if(confirm('FIRE the launcher?'))cmd('fire')}
function finish(){if(confirm('Run B to 338? It FIRES if cocked.'))cmd('finish')}
function lower(){if(confirm('Lower the rail (phi unknown)? Runs B slowly until the rail is down. It FIRES if cocked.'))cmd('lower')}
setInterval(()=>{for(const c of held.values())send(c)},150);
function hold(id,c){if(held.has(id))return;held.set(id,c);send(c)}
function release(id){if(!held.has(id))return;held.delete(id);if(!held.size)send(' ')}
const MAP={c:'cock',t:'tow',r:'retrieve',y:'cycle',x:'clear',n:'anchored',m:'released',z:'zero'};
addEventListener('keydown',e=>{if(e.key===' '){e.preventDefault();held.clear();send(' ');return}
if(e.metaKey||e.ctrlKey||e.altKey){lost();return}
if(e.key==='f'||e.key==='b'){e.preventDefault();hold(e.code,e.key);return}if(e.repeat)return;
if(e.key==='F'){fire();return}if(e.key==='E'){finish();return}if(e.key==='L'){lower();return}if(e.key==='O'){cmd('force');return}if(MAP[e.key])cmd(MAP[e.key]);else if(e.key==='+'||e.key==='-')send(e.key)});
addEventListener('keyup',e=>{if(e.key==='Meta'||e.key==='Control'||e.key==='Alt'){lost();return}if(held.has(e.code))release(e.code)});
function lost(){if(held.size){held.clear();send(' ')}}   // macOS sends no keyup for f while Cmd is held: drop every hold
addEventListener('blur',lost);addEventListener('pagehide',lost);document.addEventListener('visibilitychange',()=>{if(document.hidden)lost()});
addEventListener('contextmenu',e=>e.preventDefault());
const B=document.getElementById('b');
for(const [c,n] of [['f','reel wind (f)'],['b','launcher (b)']]){const x=document.createElement('button');x.textContent=n;x.tabIndex=-1;const id='btn'+c;
x.onpointerdown=()=>hold(id,c);x.onpointerup=()=>release(id);x.onpointerleave=()=>release(id);x.onpointercancel=()=>release(id);B.appendChild(x)}
for(const [f,n] of [[()=>send(' '),'STOP'],[()=>cmd('cock'),'cock'],[fire,'FIRE'],[()=>cmd('tow'),'tow'],[()=>cmd('retrieve'),'retrieve'],[()=>cmd('cycle'),'cycle'],
[finish,'finish'],[lower,'lower'],[()=>cmd('force'),'force F'],[()=>cmd('anchored'),'anchored'],[()=>cmd('released'),'released'],[()=>cmd('clear'),'clear fault'],[()=>cmd('zero'),'zero']]){const x=document.createElement('button');x.textContent=n;x.tabIndex=-1;x.onclick=()=>{f();x.blur()};B.appendChild(x)}
document.getElementById('setlim').onclick=()=>cmd('limit',document.getElementById('lim').value);
setInterval(()=>fetch('/status').then(r=>r.json()).then(j=>{const w=j.switches;S.textContent=`${j.state.toUpperCase()}  cycle ${j.cycle}  ${j.fault?'FAULT: '+j.fault:''}
phi ${j.phi}${j.phi_known?'':'?'}  theta ${j.theta_est}  turns ${j.turns}  line ${j.line_m} m
${j.current_A} A (peak ${j.peak_A})  tension ${j.tension_N} N (${j.tension_src}, limit ${j.limit_N})${j.holding?'  HOLDING':''}
switches low ${w.low} cocked ${w.cocked} seated ${w.seated}  AS5600 ${j.enc_ok?'ok':'MISSING'}  anchored ${j.anchored} released ${j.released}
last: ${j.event}`}).catch(()=>{}),400);
</script></body></html>)HTML";

void setup() {
  pwmAttach(PIN_RPWM, 0); pwmAttach(PIN_LPWM, 1); hwSetMotor(0, 0);   // motor safe before anything else
  pinMode(PIN_REN, OUTPUT); pinMode(PIN_LEN, OUTPUT); digitalWrite(PIN_REN, HIGH); digitalWrite(PIN_LEN, HIGH);
  pinMode(PIN_SW_LOW, INPUT_PULLUP); pinMode(PIN_SW_COCKED, INPUT_PULLUP); pinMode(PIN_SW_SEATED, INPUT_PULLUP);
  analogSetPinAttenuation(PIN_IS, ADC_11db);
  mtx = xSemaphoreCreateMutex();
  G.io = {hwSetMotor, hwAngle, hwIsVolts, hwHx, hwSwitches, hwLog, hwSave};
  prefs.begin("gs", false);
  G.restore(prefs.getFloat("phi", 0), prefs.getBool("known", false), prefs.getBool("prob", false));
  Serial.begin(115200);
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 1500) delay(10);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);   // 100 kHz: about 0.4 m of wire next to the motor leads
  Wire.setTimeOut(5);
  hx.begin(PIN_HX_DOUT, PIN_HX_SCK, true);
  hxOk = hx.wait_ready_timeout(600);                   // HX711 auto-detect: no data line toggling = not fitted
  Serial.println(hxOk ? "HX711 found: tension from the load cell" : "no HX711: tension from motor current");
  if (as5600Read(0x0B, 1) < 0) Serial.println("AS5600 not found (I2C 0x36). The motor will not run without it.");
  WiFi.mode(WIFI_AP);
  WiFi.softAP("GRAPPSTAR-GS", "grappstar");
  WiFi.softAPConfig(IPAddress(192, 168, 5, 1), IPAddress(192, 168, 5, 1), IPAddress(255, 255, 255, 0));   // own subnet: dart is 192.168.4.1
  web.on("/", []() { web.send_P(200, "text/html", PAGE); });
  web.on("/key", []() {
    reply = "";
    String c = web.arg("c"); uint32_t ms = web.hasArg("ms") ? web.arg("ms").toInt() : G.tu.wifiDeadmanMs;
    static long lastStopN = -1; long n = web.hasArg("n") ? web.arg("n").toInt() : -1;   // page numbers every request
    static String lastSid; String sid = web.arg("sid");
    if (sid.length() && sid != lastSid) { lastSid = sid; lastStopN = -1; }              // reloaded page counts from 1 again
    if (c == " " || c == "s") { if (n > lastStopN) lastStopN = n; }
    else if (n >= 0 && n < lastStopN) { web.send(200, "text/plain", ""); return; }      // a jog that left before the stop: drop it
    if (ms < 50) ms = 50;
    if (ms > G.tu.serialRunMs) ms = G.tu.serialRunMs;   // hold-to-run: a lost client may leave the motor running no longer than this
    if (c.length() && strchr("fb sS+-", c[0])) handleKey(c[0], ms);   // jog keys only; commands go through /cmd
    web.send(200, "text/plain", reply);
  });
  web.on("/cmd", []() {
    reply = "";
    String c = web.arg("c"); float v = web.hasArg("v") ? web.arg("v").toFloat() : 0;
    xSemaphoreTake(mtx, portMAX_DELAY); bool ok = G.cmd(c.c_str(), v, millis()); xSemaphoreGive(mtx);
    drainLog();
    web.send(ok ? 200 : 409, "text/plain", reply);
  });
  web.on("/stop", []() { reply = ""; handleKey(' ', 0); web.send(200, "text/plain", reply); });
  web.on("/status", []() { char b[700]; xSemaphoreTake(mtx, portMAX_DELAY); G.status(b, sizeof b, millis()); xSemaphoreGive(mtx); web.send(200, "application/json", b); });
  web.begin();
  ArduinoOTA.setHostname("grappstar-gs");
  ArduinoOTA.onStart([]() {                            // stop, then write phi now: loop() (the normal saver) is blocked until the reboot
    xSemaphoreTake(mtx, portMAX_DELAY); G.cmd("stop", 0, millis()); xSemaphoreGive(mtx);
    delay(150);                                        // let the worm stop and the last counts come in
    xSemaphoreTake(mtx, portMAX_DELAY); savePhi = G.phi(); saveKnown = G.phiKnown; saveProb = G.probCocked; saveDue = true; xSemaphoreGive(mtx);
    savePhiNow();
  });
  ArduinoOTA.begin();
  xTaskCreatePinnedToCore(tickTask, "gs_tick", 6144, nullptr, 3, nullptr, 1);
  help();
  Serial.print("WiFi: join GRAPPSTAR-GS, open http://"); Serial.println(WiFi.softAPIP());
}

void loop() {
  if (Serial.available()) { char c = Serial.read(); if (c != '\n' && c != '\r') { reply = ""; handleKey(c, G.tu.serialRunMs); } }
  web.handleClient();
  ArduinoOTA.handle();
  reply = ""; drainLog();                              // print events from the tick task
  if (saveDue) savePhiNow();
  delay(1);
}
