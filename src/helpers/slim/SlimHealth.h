// How a node is, in about eighteen bytes. Transcribed from slim_health.py.
//
// TYPE_STATUS was reserved in the type space from the start and never spent
// (DESIGN.md section 4), which is why this needs no version bump: a node that
// does not know the type ignores the frame, and one that does gets the whole
// picture.
//
// **What it is for.** A repeater on a ridge runs on a battery and a small
// panel and nobody visits it. The first sign of a battery going is not an
// outage - it is the overnight low creeping down a few millivolts a day for
// weeks, long before the night it does not make it to morning. That is only
// visible as a record, so the base keeps one, and this is the frame that fills
// it. The same frame carries a tracker's battery and fix quality, which is why
// the position frame has no quality byte.
//
//     byte 0    presence, 7 fields per byte, bit 7 = another byte follows
//     ...       the fields that are present, in field order, at fixed widths
//
// **A presence bitmap rather than a fixed record**, for three reasons that are
// all the same reason: a repeater with no temperature sensor should not send a
// fake temperature, a tracker should not send a packet queue length, and a
// field added next year must not break a node flashed this year. Fields are
// APPEND ONLY and packed in index order, so a receiver decodes every field it
// knows and stops at the first it does not - the unknown ones are all after
// the known ones, by construction.
//
// Cross-checked against the Python in test_codecs.cpp, over a sweep of values,
// because both sides hold the same key: a field packed one way and read the
// other authenticates perfectly and logs a different voltage.

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace slimhealth {

//: Bit 7 of a presence byte: another follows. Seven fields per byte, so a
//: tracker pays one and a repeater two.
static const uint8_t MORE = 0x80;
static const uint8_t PER_BYTE = 7;

//: The fields, in wire order. **Append only** - the index is the wire format.
enum Field : uint8_t {
  F_BATT_MV = 0,   // 14 bits, 2 mV steps: a cell, a vehicle or an aircraft
  F_TEMP_C,        //  8 bits, half degrees from -40
  F_UPTIME_MIN,    // 18 bits, minutes, to 182 days
  F_FIX,           //  3 bits: 0 none, 1 2D, 2 3D, 3 differential
  F_SATS,          //  5 bits
  F_RSSI,          //  7 bits, dBm from -140
  F_SNR,           //  8 bits, quarter dB from -20
  F_NOISE,         //  7 bits, dBm from -140
  F_RX,            // 16 bits, saturating
  F_TX,            // 16 bits, saturating
  F_AIR_S,         // 20 bits, seconds of transmitting, to 12 days
  F_QUEUE,         //  6 bits
  F_ERRORS,        //  8 bits
  F_FLAGS,         //  8 bits, see FLAG_*
  F_LAT,           // 24 bits, for a repeater that moves
  F_LON,           // 24 bits
  FIELD_COUNT
};

//: What a node is. MOBILE is the one that changes behaviour rather than the
//: display: a repeater in a helicopter is a hundred kilometres from where it
//: was an hour ago, so a route through it is worth far less than a route
//: through a mast.
static const uint8_t FLAG_MOBILE  = 0x01;
static const uint8_t FLAG_MAINS   = 0x02;
static const uint8_t FLAG_SOLAR   = 0x04;
static const uint8_t FLAG_REPEATS = 0x08;
static const uint8_t FLAG_LOW     = 0x10;

//: The largest a status can be: every field, plus three presence bytes.
static const uint8_t MAX_BODY = 32;

//: What a node has to say. A value is included only if its `has` bit is set,
//: so a reading that failed is the same as one never taken - which is the
//: shape these arrive in, and the alternative is inventing a temperature.
struct Status {
  double value[FIELD_COUNT];
  uint16_t has = 0;                    // bit per field, low 16 fields

  void set(Field f, double v) { value[f] = v; has |= (uint16_t)(1u << f); }
  void clear(Field f) { has &= (uint16_t)~(1u << f); }
  bool known(Field f) const { return (has & (1u << f)) != 0; }
  double get(Field f) const { return value[f]; }
};

//: Writes the body. Returns its length, or -1 if it will not fit `cap`.
int encode(const Status& s, uint8_t* out, uint8_t cap);

//: Reads one back. Fields this build does not know stop the walk rather than
//: being guessed at; `truncated` says so. Returns false only for a body that
//: is not a status at all.
bool decode(const uint8_t* body, uint8_t len, Status& out, bool* truncated = 0);

//: Which fields a request asks for. An empty request asks for all of them,
//: which is the normal case and therefore the one that costs no bytes.
uint16_t wanted(const uint8_t* body, uint8_t len);

//: The inverse, for a base asking for a subset.
int request(uint16_t fields, uint8_t* out, uint8_t cap);

//: Roughly what is left of a lithium cell, on the curve the base already
//: draws. Here as well as there so a node can set FLAG_LOW itself rather than
//: every receiver holding the curve.
double batteryPercent(double millivolts, uint8_t cells = 1);

}  // namespace slimhealth
