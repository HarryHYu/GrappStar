// GrappStar base station firmware (ESP32-S3 DevKitC-1-N8). One wiper motor (BTS7960 + AS5600) drives launch, aim and
// reel; an N20 selector drum (DRV8833) swaps the trains. The Jetson talks to it over the native USB port: JSON lines,
// contract C1, see PROTOCOL.md. All logic is in base_logic.h (plain C++, tested on a PC: test/run_tests.sh); this file
// is only the hardware glue.
//
// Arduino IDE: board "ESP32S3 Dev Module", Tools > USB CDC On Boot: Enabled, USB Mode: Hardware CDC and JTAG.
// Library: HX711 by Rob Tillaart (only used if a load cell is fitted). See README.md.
//
// Safety: the 5 ms tick (sensors, current trip, stall, ramps, drum moves, state machine) runs in its own FreeRTOS task
// on core 1, so a slow USB host can never stop the motors from being stopped. Flash writes happen in loop(), never in
// the tick.
#include <Wire.h>
#include <Preferences.h>
#include <HX711.h>
#include <esp_system.h>
#if __has_include(<esp_random.h>)
#include <esp_random.h>
#endif
#include "pins.h"
#include "base_logic.h"

#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot: Enabled (the Jetson talks to the native USB port)"
#endif

static const int SW_PINS[8] = {PIN_SW_A, PIN_SW_B, PIN_SW_C, PIN_SW_D, PIN_SW_E, PIN_SW_F, PIN_SW_G, PIN_SW_H};

gb::Base B;
HX711 hx;
Preferences prefs;
SemaphoreHandle_t mtx;
bool hxOk = false;

// ---- event ring: filled by the tick (under mtx), drained to USB by loop()
static const int RING = 24;
char ring[RING][260]; volatile int ringHead = 0, ringTail = 0; volatile uint32_t ringDrops = 0;
// ---- pending flash writes (copied under mtx, written by loop() outside it)
volatile bool saveDue = false, tunDue = false; gb::Saved saveBuf; gb::Tun tunBuf;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmAttach(int pin, int ch) { (void)ch; ledcAttach(pin, PWM_HZ, PWM_BITS); }
void pwmWrite(int pin, int ch, int duty) { (void)ch; ledcWrite(pin, duty); }
#else
void pwmAttach(int pin, int ch) { ledcSetup(ch, PWM_HZ, PWM_BITS); ledcAttachPin(pin, ch); }
void pwmWrite(int pin, int ch, int duty) { (void)pin; ledcWrite(ch, duty); }
#endif

// ---------------------------------------------------------------- hardware callbacks (called from the tick, mtx held)
void hwWiper(int dir, int duty) {                      // BTS7960: PWM one side, other side low; both low = brake
  pwmWrite(PIN_RPWM, 0, dir > 0 ? duty : 0);
  pwmWrite(PIN_LPWM, 1, dir < 0 ? duty : 0);
}
void hwDrum(int dir, int duty) {                       // DRV8833 slow decay: one input high, the other PWM inverted;
  if (dir > 0) { pwmWrite(PIN_DRUM_IN1, 2, 255); pwmWrite(PIN_DRUM_IN2, 3, 255 - duty); }   // stop = both high = brake
  else if (dir < 0) { pwmWrite(PIN_DRUM_IN1, 2, 255 - duty); pwmWrite(PIN_DRUM_IN2, 3, 255); }
  else { pwmWrite(PIN_DRUM_IN1, 2, 255); pwmWrite(PIN_DRUM_IN2, 3, 255); }
}
void hwDrumSleep(bool awake) { digitalWrite(PIN_DRUM_SLP, awake ? HIGH : LOW); }
int as5600Read(uint8_t reg, int n) {                   // big-endian register read, -1 on any I2C error
  Wire.beginTransmission(AS5600_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)n) != n) return -1;
  int v = 0; for (int i = 0; i < n; i++) v = (v << 8) | Wire.read();
  return v;
}
int hwAngle(uint8_t* st) {
  static uint32_t lastSt = 0; static uint8_t status = 0;
  if (millis() - lastSt >= 50) {                       // STATUS: MD 0x20 magnet detected, ML 0x10 too weak, MH 0x08 too strong
    lastSt = millis(); int s = as5600Read(0x0B, 1); status = s < 0 ? 0 : (uint8_t)s;
  }
  *st = status;
  int r = as5600Read(0x0E, 2);                         // ANGLE (filtered, 12 bit)
  return r < 0 ? -1 : (r & 0x0FFF);
}
float hwWiperIs() { uint32_t mv = 0; for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_WIPER_IS); return mv / 8000.0f; }
float hwVbat() {
  uint32_t mv = analogReadMilliVolts(PIN_VBAT);
  float v = mv / 1000.0f * B.c.vbatScale;
  return v < 0.5f ? NAN : v;                           // nothing on the pin (divider not fitted) reads about 0: no battery check
}
uint8_t hwSwitches() { uint8_t b = 0; for (int i = 0; i < 8; i++) if (digitalRead(SW_PINS[i]) == LOW) b |= 1 << i; return b; }
float hwHx() { return hxOk && hx.is_ready() ? (float)hx.read() : NAN; }
void hwSave(const gb::Saved& s) { saveBuf = s; saveDue = true; }
void hwSaveTun(const gb::Tun& t) { tunBuf = t; tunDue = true; }
void hwOut(const char* line) {
  int next = (ringHead + 1) % RING;
  if (next == ringTail) { ringDrops = ringDrops + 1; return; }       // host not reading: drop the newest, never block the tick
  strncpy(ring[ringHead], line, 259); ring[ringHead][259] = 0; ringHead = next;
}
uint32_t hwRand() { return esp_random(); }

// ---------------------------------------------------------------- tick task
void tickTask(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    xSemaphoreTake(mtx, portMAX_DELAY);
    B.tick(millis());
    xSemaphoreGive(mtx);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
  }
}

const char* resetName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "POWERON"; case ESP_RST_BROWNOUT: return "BROWNOUT"; case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "WATCHDOG"; case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_USB: return "USB"; default: return "OTHER";
  }
}

void setup() {
  // motors safe before anything else: PWM low, enables low (10 k pull-downs hold them low during reset), drum asleep
  pwmAttach(PIN_RPWM, 0); pwmAttach(PIN_LPWM, 1); hwWiper(0, 0);
  pinMode(PIN_REN, OUTPUT); pinMode(PIN_LEN, OUTPUT); digitalWrite(PIN_REN, LOW); digitalWrite(PIN_LEN, LOW);
  pinMode(PIN_DRUM_SLP, OUTPUT); digitalWrite(PIN_DRUM_SLP, LOW);
  pwmAttach(PIN_DRUM_IN1, 2); pwmAttach(PIN_DRUM_IN2, 3); hwDrum(0, 0);
  pinMode(PIN_DRUM_FLT, INPUT_PULLUP);
  for (int i = 0; i < 8; i++) pinMode(SW_PINS[i], INPUT_PULLUP);
  analogSetPinAttenuation(PIN_WIPER_IS, ADC_11db);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  Serial.begin(115200);                                // USB CDC: the baud rate is ignored
#if ARDUINO_USB_MODE
  Serial.setTxTimeoutMs(0);                            // never block on a host that is not reading
#endif
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 1500) delay(10);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);                               // 100 kHz: the AS5600 lead runs next to the wiper leads
  Wire.setTimeOut(5);
  hx.begin(PIN_HX_DT, PIN_HX_SCK, true);
  hxOk = hx.wait_ready_timeout(600);                   // no data line toggling = no HX711 fitted

  mtx = xSemaphoreCreateMutex();
  gb::Io io;
  io.wiper = hwWiper; io.drum = hwDrum; io.drumSleep = hwDrumSleep; io.angle = hwAngle; io.wiperIsVolts = hwWiperIs;
  io.vbat = hwVbat; io.switches = hwSwitches; io.hxRaw = hwHx; io.save = hwSave; io.saveTun = hwSaveTun; io.out = hwOut;
  io.rand32 = hwRand;
  B.io = io;
  prefs.begin("gsbase", false);
  gb::Tun tun; bool haveTun = prefs.getBytes("tun", &tun, sizeof tun) == sizeof tun;
  B.restoreTun(haveTun ? &tun : nullptr);
  B.begin(millis());
  gb::Saved rec; bool haveRec = prefs.getBytes("rec", &rec, sizeof rec) == sizeof rec;
  B.restore(haveRec ? &rec : nullptr);

  digitalWrite(PIN_REN, HIGH); digitalWrite(PIN_LEN, HIGH);   // bridge enabled; PWM is still 0
  char hello[160];
  snprintf(hello, sizeof hello, "{\"evt\":\"HELLO\",\"fw\":\"%s\",\"reset\":\"%s\",\"hx711\":%s,\"as5600\":%s}", GB_FW_VERSION, resetName(),
           hxOk ? "true" : "false", as5600Read(0x0B, 1) >= 0 ? "true" : "false");
  writeLine(hello, 100);
  xTaskCreatePinnedToCore(tickTask, "base_tick", 8192, nullptr, 3, nullptr, 1);
}

// ---------------------------------------------------------------- loop: USB lines in, replies / events / telemetry out
char line[gb::JSON_MAX_LINE + 2]; int lineLen = 0; bool lineOver = false;
char outBuf[300];

// write a whole line or nothing (a half line would corrupt the JSON stream); wait up to waitMs for USB buffer room
bool writeLine(const char* s, uint32_t waitMs) {
  int n = (int)strlen(s);
  uint32_t t0 = millis();
  while (Serial.availableForWrite() < n + 1) { if (millis() - t0 >= waitMs) return false; delay(1); }
  Serial.write((const uint8_t*)s, n); Serial.write('\n');
  return true;
}

void drainEvents() {
  for (;;) {
    char l[260]; bool have = false;
    xSemaphoreTake(mtx, portMAX_DELAY);
    if (ringTail != ringHead) { memcpy(l, ring[ringTail], 260); have = true; }
    xSemaphoreGive(mtx);
    if (!have) break;
    if (!writeLine(l, 20)) break;                      // no room: keep it queued for the next loop
    xSemaphoreTake(mtx, portMAX_DELAY); ringTail = (ringTail + 1) % RING; xSemaphoreGive(mtx);
  }
}

void handleLine() {
  char rep[300];
  xSemaphoreTake(mtx, portMAX_DELAY);
  if (lineOver) snprintf(rep, sizeof rep, "{\"id\":null,\"ok\":false,\"err\":\"TOO_LONG\"}");
  else B.handleLine(line, lineLen, rep, sizeof rep);
  xSemaphoreGive(mtx);
  writeLine(rep, 200);                                 // the reply first, then any event the command raised
  drainEvents();
}

void loop() {
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch < 0) break;
    if (ch == '\n' || ch == '\r') {
      if (lineLen > 0 || lineOver) { line[lineLen < (int)sizeof line ? lineLen : (int)sizeof line - 1] = 0; handleLine(); }
      lineLen = 0; lineOver = false;
      continue;
    }
    if (lineLen < gb::JSON_MAX_LINE) line[lineLen++] = (char)ch; else lineOver = true;
  }
  drainEvents();
  uint32_t now = millis();
  bool due;
  xSemaphoreTake(mtx, portMAX_DELAY);
  due = B.telDue(now);
  if (due) B.telemetry(outBuf, sizeof outBuf);
  xSemaphoreGive(mtx);
  if (due) writeLine(outBuf, 0);                       // telemetry is dropped when there is no room, never queued
  // flash writes outside the lock (a write stalls both cores for a few ms)
  if (saveDue) { gb::Saved s; xSemaphoreTake(mtx, portMAX_DELAY); s = saveBuf; saveDue = false; xSemaphoreGive(mtx); prefs.putBytes("rec", &s, sizeof s); }
  if (tunDue) { gb::Tun t; xSemaphoreTake(mtx, portMAX_DELAY); t = tunBuf; tunDue = false; xSemaphoreGive(mtx); prefs.putBytes("tun", &t, sizeof t); }
  delay(1);
}
