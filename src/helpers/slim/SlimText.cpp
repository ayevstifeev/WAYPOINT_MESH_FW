#include "SlimText.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>

namespace slimtext {

const char* const CODES[] = {
  "OK",
  "RTB",
  "NEED ASSIST",
  "CASUALTY LOCATED",
  "TASK COMPLETE",
  "NO SIGNAL",
  "STANDING BY",
  "MOVING",
  "AT LOCATION",
  "NEGATIVE CONTACT",
  "REPEAT LAST",
  "WAIT ONE",
  "OUT OF DAYLIGHT",
  "WEATHER DETERIORATING",
  "REQUEST EXTRACTION",
  "SEARCH AREA CLEAR",
};
const uint8_t CODE_COUNT = (uint8_t)(sizeof(CODES) / sizeof(CODES[0]));

// 4 bits, so sixteen symbols and not one more. Fifteen go to characters and
// the sixteenth to END, because a body carries no length and the last byte
// would otherwise be half padding that decodes as a "0".
static const char NIBBLE_ALPHABET[] = "0123456789 .-/:";
static const uint8_t NIBBLE_END = 15;

// 6 bits: 63 characters and END. Uppercase only - case is not information in a
// SAR message and costs a bit per character to carry.
const char PACK6_ALPHABET[] =
  " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,-/:()'?!+=*#%&<>@_;$\"^~[";
static const uint8_t PACK6_END = 63;
const uint8_t PACK6_COUNT = (uint8_t)(sizeof(PACK6_ALPHABET) - 1);

static int indexIn(const char* alphabet, char c) {
  for (int i = 0; alphabet[i]; i++) if (alphabet[i] == c) return i;
  return -1;
}

// Pack fixed-width values big-endian, filling the tail with 1 BITS.
//

static int unpackBits(const uint8_t* data, uint8_t len, uint8_t width,
                      const char* alphabet, uint8_t end,
                      char* out, uint8_t cap) {
  uint32_t acc = 0;
  int held = 0, n = 0;
  for (uint8_t i = 0; i < len; i++) {
    acc = (acc << 8) | data[i];
    held += 8;
    while (held >= width) {
      held -= width;
      const uint8_t v = (uint8_t)((acc >> held) & ((1u << width) - 1));
      if (v == end) { if (n < cap) out[n] = 0; return n; }
      if (n + 1 >= cap) { out[n] = 0; return n; }
      out[n++] = alphabet[v];
    }
  }
  out[n] = 0;
  return n;
}

//: How many bytes packInto produces for `n` symbols of `width` bits.
//:
//: Exact, not an estimate. The last byte is padded with ones - which is
//: also the END symbol, which is why no terminator is packed - so there is
//: never a byte beyond this one.
static int packedLen(uint16_t n, uint8_t width) {
  return (int)((((uint32_t)n * width) + 7u) / 8u);
}

//: Can every character be said in this alphabet?
//:
//: Case is folded, exactly as the packer folds it. PACK6 has no lower case
//: and does not refuse it - it uppercases - which is why a message typed in
//: sentence case arrives in capitals. That is the trade the six-bit
//: alphabet buys and it is why anything whose CASE carries meaning, like a
//: base64url header, must not come through here. See slimapp::bodyFor.
static bool sayableIn(const char* text, const char* alphabet) {
  for (const char* p = text; *p; p++) {
    if (indexIn(alphabet, (char)toupper((unsigned char)*p)) < 0) return false;
  }
  return true;
}

//: Pack straight into `out`. Returns the length, or -1 if a character is
//: outside the alphabet or the bytes will not fit.
static int packInto(const char* text, const char* alphabet, uint8_t width,
                    uint8_t* out, uint8_t cap) {
  uint32_t acc = 0;
  int held = 0, len = 0;
  for (const char* p = text; *p; p++) {
    const int idx = indexIn(alphabet, (char)toupper((unsigned char)*p));
    if (idx < 0) return -1;
    acc = (acc << width) | (uint32_t)idx;
    held += width;
    while (held >= 8) {
      held -= 8;
      if (len >= cap) return -1;
      out[len++] = (uint8_t)((acc >> held) & 0xFF);
    }
  }
  if (held) {
    // Ones, not zeros, and this is the whole trick that lets a body carry
    // no length field. Leftover padding is 0..7 bits: fewer than `width`
    // and the decoder runs out, but 6 or 7 bits of a 6-bit alphabet is a
    // whole extra symbol, and zero padding would decode it as a real
    // character - a space silently appended to every message unlucky
    // enough to land on the wrong boundary. All-ones is END in both
    // alphabets, so the decoder recognises padding as padding.
    const int pad = 8 - held;
    if (len >= cap) return -1;
    out[len++] = (uint8_t)(((acc << pad) | ((1u << pad) - 1)) & 0xFF);
  }
  return len;
}

static bool equalsIgnoreCaseTrimmed(const char* text, const char* phrase) {
  while (*text == ' ') text++;
  const char* end = text + strlen(text);
  while (end > text && end[-1] == ' ') end--;
  const size_t len = (size_t)(end - text);
  if (len != strlen(phrase)) return false;
  for (size_t i = 0; i < len; i++) {
    if (toupper((unsigned char)text[i]) != phrase[i]) return false;
  }
  return true;
}

int encode(const char* text, uint8_t* out, uint8_t cap) {
  if (text == nullptr || cap < 2) return -1;

  // No compression scheme competes with not sending the words at all.
  for (uint8_t i = 0; i < CODE_COUNT; i++) {
    if (equalsIgnoreCaseTrimmed(text, CODES[i])) {
      out[0] = TIER_CODE;
      out[1] = i;
      return 2;
    }
  }

  const size_t chars = strlen(text);
  if (chars > MAX_TEXT) return -1;
  // Sized, not packed. Two MAX_TEXT scratch arrays used to hold the
  // candidates while this chose between them, which put half a kilobyte on
  // the stack and a ceiling on a message - and the ceiling was the one that
  // mattered: past it BOTH packed tiers were refused and the message went
  // RAW, a third more airtime than PACK6, with nothing said. The two sizes
  // are arithmetic, so the tier can be chosen before anything is packed and
  // then packed once, into the caller's buffer.
  const int n_raw = (int)chars;
  const int n_nib = sayableIn(text, NIBBLE_ALPHABET)
                  ? packedLen((uint16_t)chars, 4) : -1;
  const int n_p6  = sayableIn(text, PACK6_ALPHABET)
                  ? packedLen((uint16_t)chars, 6) : -1;

  // Ties go to the lower tier number: cheaper to decode on a small MCU, and
  // more likely to be understood by a unit on an older build.
  // Each test is <=, not <, because each candidate has a LOWER tier number
  // than the one it is being compared against. "A" is one byte either way, and
  // it must come out as PACK6 rather than RAW or the two stacks disagree on
  // every short word.
  int best = n_raw; uint8_t tier = TIER_RAW;
  if (n_p6 >= 0 && n_p6 <= best)   { best = n_p6;  tier = TIER_PACK6; }
  if (n_nib >= 0 && n_nib <= best) { best = n_nib; tier = TIER_NIBBLE; }

  if (1 + best > cap) return -1;
  out[0] = tier;
  if (tier == TIER_RAW) {
    memcpy(&out[1], text, chars);
    return 1 + best;
  }
  const uint8_t width = (tier == TIER_NIBBLE) ? 4 : 6;
  const char* alphabet = (tier == TIER_NIBBLE) ? NIBBLE_ALPHABET
                                               : PACK6_ALPHABET;
  const int len = packInto(text, alphabet, width, &out[1],
                           (uint8_t)(cap - 1));
  // The arithmetic above said `best`. If the packer disagrees, something
  // here is wrong and the wrong answer is worse than none.
  if (len != best) return -1;
  return 1 + best;
}

int decode(const uint8_t* body, uint8_t len, char* out, uint8_t cap) {
  if (out == nullptr || cap == 0) return 0;
  if (body == nullptr || len == 0) { out[0] = 0; return 0; }

  const uint8_t tier = body[0];
  const uint8_t* rest = body + 1;
  const uint8_t rlen = (uint8_t)(len - 1);

  switch (tier) {
    case TIER_CODE:
      if (rlen == 0) return snprintf(out, cap, "[truncated code]");
      if (rest[0] < CODE_COUNT) return snprintf(out, cap, "%s", CODES[rest[0]]);
      // An older table must show that something arrived it cannot name, rather
      // than naming it wrongly.
      return snprintf(out, cap, "[code %u]", (unsigned)rest[0]);
    case TIER_NIBBLE:
      return unpackBits(rest, rlen, 4, NIBBLE_ALPHABET, NIBBLE_END, out, cap);
    case TIER_PACK6:
      return unpackBits(rest, rlen, 6, PACK6_ALPHABET, PACK6_END, out, cap);
    case TIER_RAW: {
      uint8_t n = rlen;
      if (n + 1u > cap) n = (uint8_t)(cap - 1);
      memcpy(out, rest, n);
      out[n] = 0;
      return n;
    }
    default:
      return snprintf(out, cap, "[unknown text format %u]", (unsigned)tier);
  }
}

}  // namespace slimtext
