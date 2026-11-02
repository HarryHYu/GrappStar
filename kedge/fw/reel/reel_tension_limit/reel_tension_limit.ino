// GrappStar reel: winch with a tether-tension limit, so the reel never pulls harder than the dart can hold.
// Board: ESP32-S3 DevKitC-1 ("ESP32S3 Dev Module"), BTS7960 as on your bench: RPWM GPIO4, LPWM GPIO5, R_EN GPIO6, L_EN GPIO7.
// NEW wires: BTS7960 R_IS and L_IS tied together -> GPIO1 (ADC), and a 10k resistor from GPIO1 to GND if your module has none.
// Optional: HX711 load cell in the line path, DOUT GPIO15, SCK GPIO16 (#define USE_HX711).
// Serial 115200, "No line ending":  f reel in   r pay out   s stop   1-9 speed   l<N> set tension limit in newtons (e.g. l20)
//                                   c calibrate zero (motor stopped)   t print tension
// How the limit works: reel-in speed is cut back smoothly as the estimated tension approaches the limit and stops at it.
// The dart's holding estimate (KEDGE engineering report, section 5): dry sand ~7 N, moist loam ~45 N, firm clay ~110 N. Start at 20 N.
//#define USE_HX711
#ifdef USE_HX711
#include "HX711.h"
HX711 scale;
const int HX_DOUT = 15, HX_SCK = 16;
float HX_N_PER_COUNT = 1.0f / 21000.0f;   // calibrate: hang a known mass, N = counts * this
#endif
const int RPWM = 4, LPWM = 5, R_EN = 6, L_EN = 7, IS_PIN = 1;
// Motor-current tension estimate: tension = (I - I_free) * K_N_PER_A. Calibrate K by pulling the line against a luggage scale.
float K_N_PER_A = 9.0f;          // newtons of line pull per amp above free running (drum radius and gearing set this)
float IS_A_PER_V = 8.5f;         // BTS7960 IS: ~8500:1 current ratio into the sense resistor; with 1k: 8.5 A per volt
float limitN = 20.0f;
int duty = 128, dir = 0;         // dir +1 reel in, -1 pay out
float iFree = 0, tension = 0;

float readCurrentA() { uint32_t mv = analogReadMilliVolts(IS_PIN); return mv / 1000.0f * IS_A_PER_V; }
float readTensionN(float iA) {
#ifdef USE_HX711
  if (scale.is_ready()) return scale.get_value(1) * HX_N_PER_COUNT;
#endif
  return max(0.0f, (iA - iFree) * K_N_PER_A);
}
void apply(int d) {
  dir = d;
  ledcWrite(0, d > 0 ? duty : 0);
  ledcWrite(1, d < 0 ? duty : 0);
}
void setup() {
  Serial.begin(115200);
  pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT); digitalWrite(R_EN, HIGH); digitalWrite(L_EN, HIGH);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(RPWM, 20000, 8, 0); ledcAttachChannel(LPWM, 20000, 8, 1);
#else
  ledcSetup(0, 20000, 8); ledcAttachPin(RPWM, 0); ledcSetup(1, 20000, 8); ledcAttachPin(LPWM, 1);
#endif
  analogSetPinAttenuation(IS_PIN, ADC_11db);
#ifdef USE_HX711
  scale.begin(HX_DOUT, HX_SCK); scale.tare();
#endif
  apply(0);
  Serial.println("reel ready: f in, r out, s stop, 1-9 speed, l<N> limit, c zero, t tension");
}
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '9') duty = (c - '0') * 28;
    else if (c == 'f') apply(+1);
    else if (c == 'r') apply(-1);
    else if (c == 's') apply(0);
    else if (c == 'l') { limitN = Serial.parseFloat(); Serial.printf("limit %.1f N\n", limitN); }
    else if (c == 'c') { float s = 0; for (int i = 0; i < 50; i++) { s += readCurrentA(); delay(5); } iFree = s / 50; Serial.printf("free current %.2f A\n", iFree);
#ifdef USE_HX711
      scale.tare();
#endif
    }
    else if (c == 't') Serial.printf("tension %.1f N  current %.2f A\n", tension, readCurrentA());
  }
  static uint32_t last = 0;
  if (millis() - last >= 10) {
    last = millis();
    float iA = readCurrentA();
    tension = 0.8f * tension + 0.2f * readTensionN(iA);         // light smoothing
    if (dir > 0) {                                               // reeling in: taper the speed over the last 30% of the limit
      float frac = tension / limitN;
      int d = frac >= 1.0f ? 0 : frac > 0.7f ? (int)(duty * (1.0f - frac) / 0.3f) : duty;
      ledcWrite(0, d); ledcWrite(1, 0);
      if (frac >= 1.0f) { static uint32_t lastMsg = 0; if (millis() - lastMsg > 1000) { lastMsg = millis(); Serial.printf("holding at limit %.1f N\n", tension); } }
    }
  }
}
