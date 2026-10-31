#include "SlimTrack.h"
#include "SlimCodecs.h"

#include <math.h>
#include <string.h>

namespace slimtrack {

const uint16_t CADENCES[16] = {60, 30, 15, 20, 45, 90, 120, 150, 180, 240,
                               300, 600, 900, 1800, 10, 5};
const double COMPACT_STEPS_M[4] = {5.0, 20.0, 80.0, 320.0};

//: Metres per degree of latitude, as slim_position.py has it. Close enough
//: everywhere: the error against a geodesic is far under the five-metre step.
static const double M_PER_DEG = 111320.0;

//: Radians per degree, written out rather than taken from M_PI:
//: M_PI is not standard C++ and is absent on some embedded toolchains,
//: and this has to be the same double the Python uses in math.radians.
static const double RAD_PER_DEG = 0.017453292519943295;

//: Python's round() is banker's rounding - nearest, ties to even - and C's is
//: ties-away-from-zero. Every delta here is quantised with one of them, so the
//: wrong one puts a sample five metres out whenever a step lands exactly on a
//: half, and only then, which is the worst kind of disagreement to find.
static long roundHalfEven(double x) {
  const double f = floor(x), d = x - f;
  if (d > 0.5) return (long)f + 1;
  if (d < 0.5) return (long)f;
  return (fmod(f, 2.0) == 0.0) ? (long)f : (long)f + 1;
}

uint8_t cadenceIndex(uint16_t seconds) {
  if (seconds == 0) seconds = DEFAULT_CADENCE_S;
  uint8_t best = 0;
  long gap = 0x7FFFFFFF;
  for (uint8_t i = 0; i < 16; i++) {
    const long d = labs((long)CADENCES[i] - (long)seconds);
    if (d < gap) { gap = d; best = i; }
  }
  return best;
}

bool isTrack(const uint8_t* body, uint8_t len) {
  return body != 0 && (len == BODY_BYTES || len == COMPACT_BYTES);
}

uint8_t samplesFor(uint32_t interval_s, uint16_t cadence_s, uint8_t cap) {
  if (cadence_s == 0) cadence_s = 1;
  const uint32_t n = interval_s / cadence_s;
  if (n < 1) return 1;
  return (n > cap) ? cap : (uint8_t)n;
}

// A bit writer and reader over the fifteen bytes. Big-endian, most significant
// bit of byte zero first, which is how the Python packs it.
namespace {

struct Writer {
  // An explicit constructor rather than a brace initialiser: the firmware
  // toolchain is older than the host one, and a struct with default
  // member initialisers is not an aggregate there.
  explicit Writer(uint8_t* o) : out(o), bit(0) {}
  uint8_t* out;
  uint16_t bit;
  void put(uint32_t value, uint8_t bits) {
    for (int8_t i = (int8_t)bits - 1; i >= 0; i--) {
      const uint16_t b = bit++;
      const uint8_t v = (uint8_t)((value >> i) & 1);
      if (v) out[b >> 3] |= (uint8_t)(0x80 >> (b & 7));
    }
  }
};

struct Reader {
  explicit Reader(const uint8_t* i) : in(i), bit(0) {}
  const uint8_t* in;
  uint16_t bit;
  uint32_t take(uint8_t bits) {
    uint32_t v = 0;
    for (uint8_t i = 0; i < bits; i++) {
      const uint16_t b = bit++;
      v = (v << 1) | ((in[b >> 3] >> (7 - (b & 7))) & 1);
    }
    return v;
  }
};

int16_t signExtend(uint32_t v, uint8_t bits) {
  const uint32_t top = 1u << (bits - 1);
  return (int16_t)((v >= top) ? (int32_t)v - (int32_t)(1u << bits) : (int32_t)v);
}

void apply(double& lat, double& lon, int16_t east, int16_t north,
           double m_lon, double step_m = STEP_M) {
  lat += north * step_m / M_PER_DEG;
  lon += east * step_m / m_lon;
}

int decodeCompact(const uint8_t* body, Point* out, uint8_t cap,
                  uint16_t& cadence_s) {
  Reader r(body);
  if (r.take(2) != COMPACT_FORM) return -1;   // a layout this build does not know
  cadence_s = CADENCES[r.take(4)];
  const double step_m = COMPACT_STEPS_M[r.take(2)];

  uint8_t head[slimcodec::POS_BYTES];
  for (uint8_t i = 0; i < slimcodec::POS_BYTES; i++) head[i] = (uint8_t)r.take(8);
  double lat, lon;
  if (!slimcodec::decodePos(head, slimcodec::POS_BYTES, lat, lon)) return -1;

  out[0].lat = lat;
  out[0].lon = lon;
  out[0].fix = true;
  uint8_t n = 1;
  const double m_lon = M_PER_DEG * cos(lat * RAD_PER_DEG);
  double alat = lat, alon = lon;
  for (uint8_t i = 0; i + 1 < COMPACT_SAMPLES; i++) {
    const int16_t east = signExtend(r.take(COMPACT_BITS), COMPACT_BITS);
    const int16_t north = signExtend(r.take(COMPACT_BITS), COMPACT_BITS);
    if (east == COMPACT_NONE || north == COMPACT_NONE) break;
    apply(alat, alon, east, north, m_lon, step_m);
    if (n >= cap) break;
    out[n].lat = alat;
    out[n].lon = alon;
    out[n].fix = true;
    n++;
  }
  return n;
}

}  // namespace

int encodeCompact(const Point* points, uint8_t count, uint16_t cadence_s,
                  uint8_t* out, uint8_t cap) {
  if (points == 0 || count == 0 || cap < COMPACT_BYTES) return -1;
  if (!points[0].fix) return -1;        // that report is the zero-body form

  // Every scale, finest first; the one that carries the most samples wins,
  // and the finer of two that carry as many. As slim_track._encode_compact.
  const double m_lon = M_PER_DEG * cos(points[0].lat * RAD_PER_DEG);
  uint8_t best_scale = 0, best_n = 0;
  int16_t best_e[COMPACT_SAMPLES - 1] = {0}, best_no[COMPACT_SAMPLES - 1] = {0};
  for (uint8_t s = 0; s < 4; s++) {
    const double step_m = COMPACT_STEPS_M[s];
    int16_t e_[COMPACT_SAMPLES - 1] = {0}, n_[COMPACT_SAMPLES - 1] = {0};
    uint8_t got = 0;
    double alat = points[0].lat, alon = points[0].lon;
    for (uint8_t i = 1; i < COMPACT_SAMPLES; i++) {
      if (i >= count || !points[i].fix) break;   // a gap ends the trail
      const long e = roundHalfEven((points[i].lon - alon) * m_lon / step_m);
      const long n = roundHalfEven((points[i].lat - alat) * M_PER_DEG / step_m);
      if (e < COMPACT_MIN || e > COMPACT_MAX || n < COMPACT_MIN || n > COMPACT_MAX) {
        break;                                    // too far at this scale
      }
      e_[got] = (int16_t)e;
      n_[got] = (int16_t)n;
      got++;
      // From where it decodes to, as in the long form.
      apply(alat, alon, (int16_t)e, (int16_t)n, m_lon, step_m);
    }
    if (s == 0 || got > best_n) {
      best_scale = s;
      best_n = got;
      for (uint8_t k = 0; k < COMPACT_SAMPLES - 1; k++) {
        best_e[k] = e_[k];
        best_no[k] = n_[k];
      }
    }
  }

  memset(out, 0, COMPACT_BYTES);
  Writer w(out);
  w.put(COMPACT_FORM, 2);
  w.put(cadenceIndex(cadence_s), 4);
  w.put(best_scale, 2);
  uint8_t head[slimcodec::POS_BYTES];
  slimcodec::encodePos(points[0].lat, points[0].lon, head);
  for (uint8_t i = 0; i < slimcodec::POS_BYTES; i++) w.put(head[i], 8);
  const uint32_t mask = (1u << COMPACT_BITS) - 1;
  for (uint8_t k = 0; k < COMPACT_SAMPLES - 1; k++) {
    const int16_t east = (k < best_n) ? best_e[k] : COMPACT_NONE;
    const int16_t north = (k < best_n) ? best_no[k] : COMPACT_NONE;
    w.put((uint32_t)east & mask, COMPACT_BITS);
    w.put((uint32_t)north & mask, COMPACT_BITS);
  }
  return COMPACT_BYTES;
}

int encode(const Point* points, uint8_t count, uint16_t cadence_s,
           uint8_t* out, uint8_t cap) {
  if (points == 0 || count == 0 || cap < BODY_BYTES) return -1;
  if (!points[0].fix) return -1;        // that report is the zero-body form

  memset(out, 0, BODY_BYTES);
  Writer w(out);
  w.put(0, 2);                          // form
  w.put(cadenceIndex(cadence_s), 4);

  uint8_t head[slimcodec::POS_BYTES];
  slimcodec::encodePos(points[0].lat, points[0].lon, head);
  for (uint8_t i = 0; i < slimcodec::POS_BYTES; i++) w.put(head[i], 8);

  // The scale is taken at the report's own latitude, which the receiver
  // recovers exactly from the absolute - so neither end carries it.
  const double m_lon = M_PER_DEG * cos(points[0].lat * RAD_PER_DEG);
  double alat = points[0].lat, alon = points[0].lon;
  bool ended = false;
  for (uint8_t i = 1; i <= DELTAS; i++) {
    int16_t east = NONE, north = NONE;
    if (!ended && i < count && points[i].fix) {
      const long e = roundHalfEven((points[i].lon - alon) * m_lon / STEP_M);
      const long n = roundHalfEven((points[i].lat - alat) * M_PER_DEG / STEP_M);
      if (e >= DELTA_MIN && e <= DELTA_MAX && n >= DELTA_MIN && n <= DELTA_MAX) {
        east = (int16_t)e;
        north = (int16_t)n;
        // From where the sample actually decodes to, not from where it was:
        // the receiver only has the decoded value, and measuring from anywhere
        // else lets rounding walk down the chain.
        apply(alat, alon, east, north, m_lon);
      } else {
        ended = true;
      }
    } else {
      ended = true;
    }
    w.put((uint32_t)east & ((1u << DELTA_BITS) - 1), DELTA_BITS);
    w.put((uint32_t)north & ((1u << DELTA_BITS) - 1), DELTA_BITS);
  }
  return BODY_BYTES;
}

int decode(const uint8_t* body, uint8_t len, Point* out, uint8_t cap,
           uint16_t& cadence_s) {
  if (!isTrack(body, len) || out == 0 || cap == 0) return -1;
  if (len == COMPACT_BYTES) return decodeCompact(body, out, cap, cadence_s);
  Reader r(body);
  if (r.take(2) != 0) return -1;        // a layout this build does not know
  cadence_s = CADENCES[r.take(4)];

  uint8_t head[slimcodec::POS_BYTES];
  for (uint8_t i = 0; i < slimcodec::POS_BYTES; i++) head[i] = (uint8_t)r.take(8);
  double lat, lon;
  if (!slimcodec::decodePos(head, slimcodec::POS_BYTES, lat, lon)) return -1;

  out[0].lat = lat;
  out[0].lon = lon;
  out[0].fix = true;
  uint8_t n = 1;
  const double m_lon = M_PER_DEG * cos(lat * RAD_PER_DEG);
  double alat = lat, alon = lon;
  for (uint8_t i = 0; i < DELTAS; i++) {
    const int16_t east = signExtend(r.take(DELTA_BITS), DELTA_BITS);
    const int16_t north = signExtend(r.take(DELTA_BITS), DELTA_BITS);
    if (east == NONE || north == NONE) break;
    apply(alat, alon, east, north, m_lon);
    if (n >= cap) break;
    out[n].lat = alat;
    out[n].lon = alon;
    out[n].fix = true;
    n++;
  }
  return n;
}

}  // namespace slimtrack
