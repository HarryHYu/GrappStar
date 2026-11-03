// Tiny JSON for the C1 protocol: parses ONE flat object per line ({"id": 3, "cmd": "aim", "deg": 32.5}) and builds
// reply lines. No heap, no library. Nested objects or arrays inside a command are skipped (allowed, ignored).
// Plain C++ (no Arduino calls): tested on a PC by test/test_base.cpp.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

namespace gb {

static const int JSON_MAX_LINE = 256;    // C1: at most 256 bytes per line
static const int JSON_MAX_KEYS = 12;

enum JType { J_NONE, J_STR, J_NUM, J_BOOL, J_NULL, J_SKIP };

struct JVal {
  char key[16];
  JType t = J_NONE;
  char s[64];          // string value (unescaped, truncated to 63 bytes)
  double n = 0;        // number value (also 1 / 0 for bools)
};

struct JObj {
  JVal v[JSON_MAX_KEYS];
  int count = 0;
  const JVal* find(const char* k) const {
    for (int i = 0; i < count; i++) if (!strcmp(v[i].key, k)) return &v[i];
    return nullptr;
  }
  bool has(const char* k) const { return find(k) != nullptr; }
  bool num(const char* k, double* out) const {
    const JVal* x = find(k);
    if (!x || x->t != J_NUM) return false;
    if (!isfinite(x->n)) return false;
    *out = x->n; return true;
  }
  bool str(const char* k, const char** out) const {
    const JVal* x = find(k);
    if (!x || x->t != J_STR) return false;
    *out = x->s; return true;
  }
  bool boolean(const char* k, bool* out) const {
    const JVal* x = find(k);
    if (!x || x->t != J_BOOL) return false;
    *out = x->n != 0; return true;
  }
};

struct JParser {
  const char* p; const char* e;
  static bool ws(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
  void skipWs() { while (p < e && ws(*p)) p++; }
  bool lit(const char* w) { size_t n = strlen(w); if ((size_t)(e - p) < n || strncmp(p, w, n)) return false; p += n; return true; }
  // string into out (cap bytes incl. NUL); out may be null to skip
  bool string(char* out, int cap) {
    if (p >= e || *p != '"') return false;
    p++; int k = 0;
    while (p < e && *p != '"') {
      char c = *p++;
      if ((unsigned char)c < 0x20) return false;            // raw control characters are not allowed in JSON strings
      if (c == '\\') {
        if (p >= e) return false;
        char x = *p++;
        switch (x) {
          case '"': c = '"'; break; case '\\': c = '\\'; break; case '/': c = '/'; break;
          case 'b': c = '\b'; break; case 'f': c = '\f'; break; case 'n': c = '\n'; break;
          case 'r': c = '\r'; break; case 't': c = '\t'; break;
          case 'u': { if (e - p < 4) return false;
            for (int i = 0; i < 4; i++) { char h = p[i]; if (!((h >= '0' && h <= '9') || (h >= 'a' && h <= 'f') || (h >= 'A' && h <= 'F'))) return false; }
            p += 4; c = '?'; break; }                        // non-ASCII is never meaningful in a command
          default: return false;
        }
      }
      if (out && k < cap - 1) out[k++] = c;
    }
    if (p >= e) return false;
    p++;
    if (out) out[k] = 0;
    return true;
  }
  bool number(double* out) {
    const char* s = p;
    if (p < e && *p == '-') p++;
    if (p >= e || !(*p >= '0' && *p <= '9')) return false;
    while (p < e && ((*p >= '0' && *p <= '9') || *p == '.' || *p == 'e' || *p == 'E' || *p == '+' || *p == '-')) p++;
    char buf[40]; int n = (int)(p - s); if (n >= (int)sizeof buf) return false;
    memcpy(buf, s, n); buf[n] = 0;
    char* end; *out = strtod(buf, &end);
    return end == buf + n && isfinite(*out);
  }
  bool skipValue(int depth) {                               // any value, nested containers included
    skipWs(); if (p >= e) return false;
    if (*p == '"') return string(nullptr, 0);
    if (*p == '{' || *p == '[') {
      if (depth > 8) return false;
      char close = *p == '{' ? '}' : ']'; bool obj = *p == '{'; p++; skipWs();
      if (p < e && *p == close) { p++; return true; }
      for (;;) {
        skipWs();
        if (obj) { if (!string(nullptr, 0)) return false; skipWs(); if (p >= e || *p != ':') return false; p++; }
        if (!skipValue(depth + 1)) return false;
        skipWs(); if (p >= e) return false;
        if (*p == ',') { p++; continue; }
        if (*p == close) { p++; return true; }
        return false;
      }
    }
    if (lit("true") || lit("false") || lit("null")) return true;
    double d; return number(&d);
  }
};

// Parse one line. Returns true for a valid object. Unknown keys beyond JSON_MAX_KEYS are ignored.
static bool jsonParse(const char* line, int len, JObj* o) {
  o->count = 0;
  JParser P{line, line + len};
  P.skipWs();
  if (P.p >= P.e || *P.p != '{') return false;
  P.p++; P.skipWs();
  if (P.p < P.e && *P.p == '}') { P.p++; P.skipWs(); return P.p == P.e; }
  for (;;) {
    P.skipWs();
    char key[16];
    if (!P.string(key, sizeof key)) return false;
    P.skipWs(); if (P.p >= P.e || *P.p != ':') return false; P.p++; P.skipWs();
    if (P.p >= P.e) return false;
    JVal tmp; strcpy(tmp.key, key); tmp.s[0] = 0;
    if (*P.p == '"') { tmp.t = J_STR; if (!P.string(tmp.s, sizeof tmp.s)) return false; }
    else if (P.lit("true")) { tmp.t = J_BOOL; tmp.n = 1; }
    else if (P.lit("false")) { tmp.t = J_BOOL; tmp.n = 0; }
    else if (P.lit("null")) { tmp.t = J_NULL; }
    else if (*P.p == '{' || *P.p == '[') { tmp.t = J_SKIP; if (!P.skipValue(0)) return false; }
    else { tmp.t = J_NUM; if (!P.number(&tmp.n)) return false; }
    bool dup = false;
    for (int i = 0; i < o->count; i++) if (!strcmp(o->v[i].key, key)) { o->v[i] = tmp; dup = true; }   // last one wins
    if (!dup && o->count < JSON_MAX_KEYS) o->v[o->count++] = tmp;
    else if (!dup && (!strcmp(key, "id") || !strcmp(key, "cmd"))) {   // full: id and cmd win a slot over an unknown key
      for (int i = o->count - 1; i >= 0; i--) if (strcmp(o->v[i].key, "id") && strcmp(o->v[i].key, "cmd")) { o->v[i] = tmp; break; }
    }
    P.skipWs(); if (P.p >= P.e) return false;
    if (*P.p == ',') { P.p++; continue; }
    if (*P.p == '}') { P.p++; break; }
    return false;
  }
  P.skipWs();
  return P.p == P.e;
}

// Reply / telemetry builder into a fixed buffer. Never overflows: on overflow it marks itself bad and the caller
// sends a short fallback line instead.
struct JOut {
  char* b; int cap; int n = 0; bool first = true; bool bad = false;
  JOut(char* buf, int c) : b(buf), cap(c) { b[0] = 0; }
  void raw(const char* s) {
    int l = (int)strlen(s);
    if (n + l >= cap) { bad = true; return; }
    memcpy(b + n, s, l); n += l; b[n] = 0;
  }
  void open() { raw("{"); first = true; }
  void close() { raw("}"); first = false; }
  void sep() { if (!first) raw(","); first = false; }
  void keyRaw(const char* k) { sep(); raw("\""); raw(k); raw("\":"); }
  void str(const char* k, const char* v) {
    keyRaw(k);
    if (!v) { raw("null"); return; }
    raw("\"");
    for (const char* q = v; *q; q++) {
      char c = *q; char t[8];
      if (c == '"' || c == '\\') { t[0] = '\\'; t[1] = c; t[2] = 0; }
      else if ((unsigned char)c < 0x20) snprintf(t, sizeof t, "\\u%04x", c);
      else { t[0] = c; t[1] = 0; }
      raw(t);
    }
    raw("\"");
  }
  void num(const char* k, double v, int dec = 2) {
    keyRaw(k);
    if (!isfinite(v)) { raw("null"); return; }
    char t[32]; snprintf(t, sizeof t, "%.*f", dec, v);
    if (dec > 0) {                                           // trim trailing zeros: 12.50 -> 12.5, 3.00 -> 3
      char* d = strchr(t, '.');
      if (d) { char* z = t + strlen(t) - 1; while (z > d && *z == '0') *z-- = 0; if (z == d) *z = 0; }
    }
    if (!strcmp(t, "-0")) strcpy(t, "0");
    raw(t);
  }
  void integer(const char* k, long v) { keyRaw(k); char t[24]; snprintf(t, sizeof t, "%ld", v); raw(t); }
  void boolean(const char* k, bool v) { keyRaw(k); raw(v ? "true" : "false"); }
  void null(const char* k) { keyRaw(k); raw("null"); }
  void obj(const char* k) { keyRaw(k); raw("{"); first = true; }
};

}  // namespace gb
