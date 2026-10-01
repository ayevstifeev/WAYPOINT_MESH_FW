#include "SlimHealth.h"

#include <math.h>
#include <string.h>

namespace slimhealth {

//: Python's round() is banker's rounding - nearest, ties to even - and C's
//: round() is ties-away-from-zero. Every value here is quantised with one of
//: them, so using the wrong one puts a different number in the log every time
//: a reading lands exactly on a half step. The same helper as SlimCodecs.cpp,
//: copied rather than shared because these two files are transcribed
//: independently and a shared private helper would tie them together for no
//: reason.
static long roundHalfEven(double x) {
  const double f = floor(x), d = x - f;
  if (d > 0.5) return (long)f + 1;
  if (d < 0.5) return (long)f;
  return (fmod(f, 2.0) == 0.0) ? (long)f : (long)f + 1;
}

struct Spec { uint8_t bits; double offset; double step; };

//: Must match FIELDS in slim_health.py, in order and to the value.
static const Spec SPEC[FIELD_COUNT] = {
  {14, 0,    2},                     // batt_mv
  { 8, -40,  0.5},                   // temp_c
  {18, 0,    1},                     // uptime_min
  { 3, 0,    1},                     // fix
  { 5, 0,    1},                     // sats
  { 7, -140, 1},                     // rssi
  { 8, -20,  0.25},                  // snr
  { 7, -140, 1},                     // noise
  {16, 0,    1},                     // rx
  {16, 0,    1},                     // tx
  {20, 0,    1},                     // air_s
  { 6, 0,    1},                     // queue
  { 8, 0,    1},                     // errors
  { 8, 0,    1},                     // flags
  {24, -90,  180.0 / 0xFFFFFF},      // lat
  {24, -180, 360.0 / 0xFFFFFF},      // lon
};

static uint32_t clampTo(long v, uint8_t bits) {
  const uint32_t top = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
  if (v < 0) return 0;
  return ((uint32_t)v > top) ? top : (uint32_t)v;
}

int encode(const Status& s, uint8_t* out, uint8_t cap) {
  int last = -1;
  for (int i = 0; i < FIELD_COUNT; i++) {
    if (s.known((Field)i)) last = i;
  }
  if (last < 0) return 0;                     // nothing to say, and that is ok

  const int groups = (last / PER_BYTE) + 1;
  if (cap < groups) return -1;
  int n = 0;
  for (int g = 0; g < groups; g++) {
    uint8_t byte = 0;
    for (int b = 0; b < PER_BYTE; b++) {
      const int i = g * PER_BYTE + b;
      if (i < FIELD_COUNT && s.known((Field)i)) byte |= (uint8_t)(1u << b);
    }
    if (g < groups - 1) byte |= MORE;
    out[n++] = byte;
  }

  uint64_t acc = 0;
  uint8_t held = 0;
  for (int i = 0; i <= last; i++) {
    if (!s.known((Field)i)) continue;
    const Spec& sp = SPEC[i];
    const uint32_t raw =
        clampTo(roundHalfEven((s.get((Field)i) - sp.offset) / sp.step), sp.bits);
    acc = (acc << sp.bits) | raw;
    held = (uint8_t)(held + sp.bits);
    while (held >= 8) {
      held = (uint8_t)(held - 8);
      if (n >= cap) return -1;
      out[n++] = (uint8_t)((acc >> held) & 0xFF);
    }
  }
  if (held) {
    const uint8_t pad = (uint8_t)(8 - held);
    if (n >= cap) return -1;
    out[n++] = (uint8_t)(((acc << pad) | ((1u << pad) - 1u)) & 0xFF);
  }
  return n;
}

bool decode(const uint8_t* body, uint8_t len, Status& out, bool* truncated) {
  out.has = 0;
  if (truncated) *truncated = false;
  if (body == 0 || len == 0) return true;      // an empty status is a status

  uint16_t present = 0;
  uint8_t at = 0;
  bool more = true;
  while (at < len && more) {
    const uint8_t byte = body[at];
    for (uint8_t b = 0; b < PER_BYTE; b++) {
      const int i = at * PER_BYTE + b;
      if (!(byte & (1u << b))) continue;
      if (i < FIELD_COUNT) present |= (uint16_t)(1u << i);
      else if (truncated) *truncated = true;   // a field from a later build
    }
    more = (byte & MORE) != 0;
    at++;
  }
  if (more) return false;                      // said more was coming, nothing was

  uint64_t acc = 0;
  uint8_t held = 0;
  for (int i = 0; i < FIELD_COUNT; i++) {
    if (!(present & (1u << i))) continue;
    const Spec& sp = SPEC[i];
    while (held < sp.bits) {
      if (at >= len) {
        if (truncated) *truncated = true;
        return true;                           // keep what is certain
      }
      acc = (acc << 8) | body[at++];
      held = (uint8_t)(held + 8);
    }
    held = (uint8_t)(held - sp.bits);
    const uint32_t raw = (uint32_t)((acc >> held) & ((1ull << sp.bits) - 1ull));
    out.set((Field)i, raw * sp.step + sp.offset);
  }
  return true;
}

uint16_t wanted(const uint8_t* body, uint8_t len) {
  if (body == 0 || len == 0) return 0xFFFF;    // everything
  uint16_t want = 0;
  uint8_t at = 0;
  bool more = true;
  while (at < len && more) {
    const uint8_t byte = body[at];
    for (uint8_t b = 0; b < PER_BYTE; b++) {
      const int i = at * PER_BYTE + b;
      if ((byte & (1u << b)) && i < FIELD_COUNT) want |= (uint16_t)(1u << i);
    }
    more = (byte & MORE) != 0;
    at++;
  }
  return want;
}

int request(uint16_t fields, uint8_t* out, uint8_t cap) {
  if (fields == 0 || fields == 0xFFFF) return 0;
  int last = -1;
  for (int i = 0; i < FIELD_COUNT; i++) {
    if (fields & (1u << i)) last = i;
  }
  if (last < 0) return 0;
  const int groups = (last / PER_BYTE) + 1;
  if (cap < groups) return -1;
  for (int g = 0; g < groups; g++) {
    uint8_t byte = 0;
    for (int b = 0; b < PER_BYTE; b++) {
      const int i = g * PER_BYTE + b;
      if (i < FIELD_COUNT && (fields & (1u << i))) byte |= (uint8_t)(1u << b);
    }
    if (g < groups - 1) byte |= MORE;
    out[g] = byte;
  }
  return groups;
}

double batteryPercent(double millivolts, uint8_t cells) {
  static const double V[] = {3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77,
                             3.79, 3.83, 3.87, 3.92, 4.07, 4.20};
  static const double P[] = {0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 95, 100};
  const int n = (int)(sizeof(V) / sizeof(V[0]));
  const double v = millivolts / 1000.0 / (cells < 1 ? 1 : cells);
  if (v <= V[0]) return 0.0;
  if (v >= V[n - 1]) return 100.0;
  for (int i = 0; i + 1 < n; i++) {
    if (v <= V[i + 1]) {
      return P[i] + (P[i + 1] - P[i]) * (v - V[i]) / (V[i + 1] - V[i]);
    }
  }
  return 100.0;
}

}  // namespace slimhealth
