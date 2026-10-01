#include "SlimApp.h"
#include "SlimText.h"

#include <string.h>

namespace slimapp {

const char DIGITS[65] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

//: The headers this replaces, in type order. Duplicated from the gateway's own
//: modules (teammap.PREFIX and its siblings) because the firmware imports
//: nothing from them; test_slim_app.py on the gateway side asserts the two
//: agree, so the duplication is checked rather than trusted.
static const char* const PREFIXES[TYPE_COUNT] = {
  "!P", "!N", "!M", "!H", "!U", "!T", "!S", "!F", "!A", "!V",
};

//: Seven bits, so 0x7F is free to mean "the rest is padding" - the same trick
//: slimtext plays with its END symbol, and for the same reason: a body carries
//: no length, and zero padding would decode as a real character at exactly the
//: string lengths nobody tests.
static const uint8_t TAIL_END = 0x7F;
static const uint8_t TAIL_MIN = 0x20, TAIL_MAX = 0x7E;

static int digitValue(char c) {
  for (int i = 0; i < 64; i++) if (DIGITS[i] == c) return i;
  return -1;
}

bool isApp(const uint8_t* body, uint8_t len) {
  if (body == NULL || len == 0) return false;
  const uint8_t t = body[0] & (uint8_t)~ALL_DIGITS;
  return t >= FIRST_TYPE && t < (uint8_t)(FIRST_TYPE + TYPE_COUNT);
}

uint8_t typeOf(const char* text) {
  if (text == NULL) return 0;
  for (uint8_t i = 0; i < TYPE_COUNT; i++) {
    if (strncmp(text, PREFIXES[i], 2) == 0) return (uint8_t)(FIRST_TYPE + i);
  }
  return 0;
}

// Fixed-width values, big-endian, tail filled with 1 bits. One writer for both
// widths: the only difference between the digit block and the free text is the
// number six.
class Packer {
 public:
  Packer(uint8_t* out, uint8_t cap) : _out(out), _cap(cap) {}
  bool put(uint16_t value, uint8_t width) {
    _acc = (_acc << width) | value;
    _held = (uint8_t)(_held + width);
    while (_held >= 8) {
      _held = (uint8_t)(_held - 8);
      if (_n >= _cap) return false;
      _out[_n++] = (uint8_t)((_acc >> _held) & 0xFF);
    }
    return true;
  }
  bool flush() {
    if (_held == 0) return true;
    const uint8_t pad = (uint8_t)(8 - _held);
    if (_n >= _cap) return false;
    _out[_n++] = (uint8_t)(((_acc << pad) | ((1u << pad) - 1)) & 0xFF);
    _held = 0;
    return true;
  }
  uint8_t length() const { return _n; }

 private:
  uint8_t* _out;
  uint8_t _cap;
  uint8_t _n = 0;
  uint32_t _acc = 0;
  uint8_t _held = 0;
};

int encode(const char* text, uint8_t* out, uint8_t cap) {
  const uint8_t type = typeOf(text);
  if (type == 0) return -1;
  const char* rest = text + 2;
  const size_t len = strlen(rest);
  if (len > MAX_TEXT) return -1;
  for (size_t i = 0; i < len; i++) {
    const unsigned char c = (unsigned char)rest[i];
    if (c < TAIL_MIN || c > TAIL_MAX) return -1;   // RAW text can carry it
  }
  size_t n = 0;
  while (n < len && n < 0xFF && digitValue(rest[n]) >= 0) n++;

  // Every character is a digit, so the count is the message and saying it
  // again costs a byte on the largest message in the network.
  //
  // Except at one length. Six-bit symbols packed into bytes leave 0, 2, 4 or 6
  // bits of padding, and six bits is a whole extra symbol the receiver cannot
  // tell from a real one - base64url has all 64 values spoken for, so there is
  // no spare symbol to mean END. That is `6n mod 8 == 2`, and at those lengths
  // the count byte goes back on rather than the message decoding one character
  // too long. Refusing the shortcut is a byte; guessing would be a position on
  // the map.
  const bool all_digits = (n == len) && ((n * 6) % 8 != 2);
  if (cap < 1) return -1;
  uint8_t i = 0;
  out[i++] = all_digits ? (uint8_t)(type | ALL_DIGITS) : type;
  if (!all_digits) {
    if (cap < 2) return -1;
    out[i++] = (uint8_t)n;
  }
  Packer head(out + i, (uint8_t)(cap - i));
  for (size_t k = 0; k < n; k++) {
    if (!head.put((uint16_t)digitValue(rest[k]), 6)) return -1;
  }
  if (!head.flush()) return -1;
  i = (uint8_t)(i + head.length());
  if (all_digits) return i;
  Packer tail(out + i, (uint8_t)(cap - i));
  for (size_t k = n; k < len; k++) {
    if (!tail.put((uint8_t)rest[k], 7)) return -1;
  }
  if (!tail.flush()) return -1;
  return (int)(i + tail.length());
}

int decode(const uint8_t* body, uint8_t len, char* out, uint8_t cap) {
  if (!isApp(body, len) || cap == 0) return -1;
  const uint8_t type = body[0] & (uint8_t)~ALL_DIGITS;
  uint8_t at, head_bytes;
  size_t n;
  if (body[0] & ALL_DIGITS) {
    at = 1;
    head_bytes = (uint8_t)(len - 1);
    // Exact, because encode never takes the shortcut at a length where the
    // padding would read as another digit.
    n = ((size_t)head_bytes * 8) / 6;
  } else {
    if (len < 2) return -1;
    at = 2;
    n = body[1];
    head_bytes = (uint8_t)((n * 6 + 7) / 8);
  }
  if ((size_t)len < (size_t)at + head_bytes) return -1;
  if ((size_t)cap < n + 4) return -1;

  int w = 0;
  out[w++] = PREFIXES[type - FIRST_TYPE][0];
  out[w++] = PREFIXES[type - FIRST_TYPE][1];

  uint32_t acc = 0;
  uint8_t held = 0;
  size_t got = 0;
  for (uint8_t k = 0; k < head_bytes; k++) {
    acc = (acc << 8) | body[at + k];
    held = (uint8_t)(held + 8);
    while (held >= 6 && got < n) {
      held = (uint8_t)(held - 6);
      out[w++] = DIGITS[(acc >> held) & 0x3F];
      got++;
    }
  }
  if (got < n) return -1;                       // truncated in the digits

  acc = 0;
  held = 0;
  for (uint8_t k = (uint8_t)(at + head_bytes); k < len; k++) {
    acc = (acc << 8) | body[k];
    held = (uint8_t)(held + 8);
    while (held >= 7) {
      held = (uint8_t)(held - 7);
      const uint8_t v = (uint8_t)((acc >> held) & 0x7F);
      if (v == TAIL_END) { held = 0; k = len; break; }   // padding
      if (v < TAIL_MIN || v > TAIL_MAX) return -1;
      if ((uint8_t)w + 1 >= cap) return -1;
      out[w++] = (char)v;
    }
  }
  out[w] = 0;
  return w;
}

//: The two buffers the tier check needs, kept OFF the stack.
//:
//: They are 360 bytes together, and bodyFor sits underneath every
//: transmission on a task whose whole stack is 4096 - below a receive path
//: that already reaches 3.7 KB when a command arrives, on a build where
//: FreeRTOS only samples for overflow at a context switch. The comment
//: below always intended this to be cheap; the fast path was, and the
//: verification path quietly was not.
//:
//: Safe because bodyFor is not reentrant: nothing it calls calls back into
//: it, and both firmwares transmit only from the one Arduino loop task. If
//: a second task ever sends, these go back on the stack.
static uint8_t s_packed[MAX_TEXT];
static char    s_back[MAX_TEXT + 8];

int bodyFor(const char* text, uint8_t* out, uint8_t cap) {
  // Straight into the caller's buffer, so the common answer costs no stack at
  // all. This runs on a radio with the frame buffer already live beneath it.
  const int alen = encode(text, out, cap);
  if (alen < 0) return slimtext::encode(text, out, cap);

  // Is the text tier smaller AND faithful? Asked by decoding it back rather
  // than by reasoning about alphabets, so this cannot drift from the Python,
  // which asks the same question the same way.
  const int plen = slimtext::encode(text, s_packed,
                                    (uint8_t)sizeof(s_packed));
  if (plen > 0 && plen <= alen) {
    const int blen = slimtext::decode(s_packed, (uint8_t)plen, s_back,
                                      (uint8_t)sizeof(s_back));
    if (blen > 0 && strcmp(s_back, text) == 0) {
      memcpy(out, s_packed, (size_t)plen);
      return plen;
    }
  }
  return alen;
}

int decodeBody(const uint8_t* body, uint8_t len, char* out, uint8_t cap) {
  if (isApp(body, len)) return decode(body, len, out, cap);
  return slimtext::decode(body, len, out, cap);
}

}  // namespace slimapp
