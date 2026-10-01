#include "SlimCodecs.h"

#include <math.h>
#include <string.h>

namespace slimcodec {

// Python's round() is banker's rounding - nearest, ties to even - and C's
// lround() is ties-away-from-zero. They differ only on exact .5, but the
// anchor and offset maths lands on .5 often enough that a mismatch would put a
// position one grid step from where the base put it. Matching the reference
// exactly costs six lines, so it is matched exactly.
static long roundHalfEven(double x) {
  const double f = floor(x);
  const double d = x - f;
  if (d > 0.5) return (long)f + 1;
  if (d < 0.5) return (long)f;
  const long fi = (long)f;
  return (fi % 2 == 0) ? fi : fi + 1;
}

static long clampl(long v, long lo, long hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ---- position -----------------------------------------------------------

void encodePos(double lat, double lon, uint8_t out[6]) {
  // Rounding, not truncating. Truncation biases every position in the same
  // direction, which over a trail reads as a steady drift rather than noise.
  const long ilat = clampl(roundHalfEven((lat + 90.0) / 180.0 * 0xFFFFFF),
                           0, 0xFFFFFF);
  const long ilon = clampl(roundHalfEven((lon + 180.0) / 360.0 * 0xFFFFFF),
                           0, 0xFFFFFF);
  out[0] = (uint8_t)(ilat >> 16); out[1] = (uint8_t)(ilat >> 8); out[2] = (uint8_t)ilat;
  out[3] = (uint8_t)(ilon >> 16); out[4] = (uint8_t)(ilon >> 8); out[5] = (uint8_t)ilon;
}

bool decodePos(const uint8_t* body, uint8_t len, double& lat, double& lon) {
  if (body == nullptr || len < POS_BYTES) return false;
  const uint32_t ilat = ((uint32_t)body[0] << 16) | ((uint32_t)body[1] << 8) | body[2];
  const uint32_t ilon = ((uint32_t)body[3] << 16) | ((uint32_t)body[4] << 8) | body[5];
  lat = (double)ilat / (double)0xFFFFFF * 180.0 - 90.0;
  lon = (double)ilon / (double)0xFFFFFF * 360.0 - 180.0;
  return true;
}

// ---- teammap ------------------------------------------------------------

static const char* ALPHABET =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

static const int ANCHOR_STEPS = 4096;
static const int POS_STEPS    = 16384;
static const double BOX_LAT   = 0.5;
static const double BOX_LON   = 0.5;

static int valueOf(char c) {
  for (int i = 0; i < 64; i++) if (ALPHABET[i] == c) return i;
  return -1;
}

//: An integer as `width` base64url digits, most significant first.
static void encDigits(uint64_t value, int width, char* out) {
  for (int i = 0; i < width; i++) {
    const int shift = 6 * (width - 1 - i);
    out[i] = ALPHABET[(value >> shift) & 0x3F];
  }
}

//: Digits back to an integer. False on anything not base64url - the format is
//: fixed width, so a stray character means the message is not one of ours.
static bool decDigits(const char* text, int width, uint64_t& out) {
  uint64_t v = 0;
  for (int i = 0; i < width; i++) {
    const int d = valueOf(text[i]);
    if (d < 0) return false;
    v = (v << 6) | (uint64_t)d;
  }
  out = v;
  return true;
}

//: The grid point a position is measured from. Quantised so that this and the
//: base arrive at exactly the same corner from the same code - the whole
//: format depends on that agreement.
static void anchorFor(double lat, double lon, double& alat, double& alon,
                      char code[4]) {
  const long ilat = clampl(roundHalfEven((lat + 90.0) / 180.0 * ANCHOR_STEPS),
                           0, ANCHOR_STEPS - 1);
  const long ilon = clampl(roundHalfEven((lon + 180.0) / 360.0 * ANCHOR_STEPS),
                           0, ANCHOR_STEPS - 1);
  alat = (double)ilat / ANCHOR_STEPS * 180.0 - 90.0;
  alon = (double)ilon / ANCHOR_STEPS * 360.0 - 180.0;
  encDigits(((uint64_t)ilat << 12) | (uint64_t)ilon, 4, code);
}

static bool anchorFromCode(const char* code, double& alat, double& alon) {
  uint64_t packed;
  if (!decDigits(code, 4, packed)) return false;
  const uint32_t ilat = (uint32_t)((packed >> 12) & 0xFFF);
  const uint32_t ilon = (uint32_t)(packed & 0xFFF);
  alat = (double)ilat / ANCHOR_STEPS * 180.0 - 90.0;
  alon = (double)ilon / ANCHOR_STEPS * 360.0 - 180.0;
  return true;
}

//: A position as 14-bit offsets inside the anchor's box - wire.offsets.
//: Clamped rather than refused: a position slightly outside the box lands on
//: its edge, a few kilometres wrong at worst and still on the map. Callers
//: that care check `fits` first.
static void offsets(double lat, double lon, double alat, double alon,
                    long& ilat, long& ilon) {
  const long span = POS_STEPS - 1;
  ilat = clampl(
    roundHalfEven((lat - alat + BOX_LAT / 2.0) / BOX_LAT * (double)span), 0, span);
  ilon = clampl(
    roundHalfEven((lon - alon + BOX_LON / 2.0) / BOX_LON * (double)span), 0, span);
}

//: Is this position inside the box around that anchor? - wire.fits.
static bool fits(double lat, double lon, double alat, double alon) {
  return fabs(lat - alat) <= BOX_LAT / 2.0 && fabs(lon - alon) <= BOX_LON / 2.0;
}

int encodeMarker(char* out, uint8_t cap, uint8_t role, uint8_t number,
                 double lat, double lon, uint8_t icon, uint8_t mnum,
                 const char* label) {
  if (number > 127 || role > ROLE_BASE) return -1;
  if (icon > ICON_REMOVED || mnum >= MARKER_PER_DEVICE) return -1;

  // markers.py does (label or "").strip()[:MAX_LABEL]. Trimmed here for
  // the same reason it is trimmed there, and in the same order: a label
  // typed on a joystick arrives padded with the spaces nobody deleted,
  // and a pin called "Gear drop " is not the same pin as "Gear drop".
  const char* p = label ? label : "";
  while (*p == ' ') p++;
  int n = 0;
  while (p[n]) n++;
  while (n > 0 && p[n - 1] == ' ') n--;
  if (n > MARKER_MAX_LABEL) n = MARKER_MAX_LABEL;
  if (MARKER_CHARS + n + 1 > (int)cap) return -1;

  double alat, alon;
  char code[4];
  anchorFor(lat, lon, alat, alon, code);
  long ilat, ilon;
  offsets(lat, lon, alat, alon, ilat, ilon);

  const uint64_t creator = ((uint64_t)role << 7) | number;
  const uint64_t packed = (creator << 39) | ((uint64_t)ilat << 25)
                        | ((uint64_t)ilon << 11)
                        | ((uint64_t)icon << 6) | mnum;

  out[0] = '!'; out[1] = 'M';
  memcpy(&out[2], code, 4);
  encDigits(packed, 8, &out[6]);
  memcpy(&out[MARKER_CHARS], p, (size_t)n);
  out[MARKER_CHARS + n] = 0;
  return MARKER_CHARS + n;
}

bool encodeSelfReport(char* out, uint8_t role, uint8_t number,
                      double lat, double lon) {
  if (number > 127 || role > ROLE_BASE) return false;

  double alat, alon;
  char code[4];
  anchorFor(lat, lon, alat, alon, code);

  // The anchor is chosen from this very position, so the clamp in offsets()
  // only ever bites at the poles.
  long ilat, ilon;
  offsets(lat, lon, alat, alon, ilat, ilon);

  // 42 bits: role 2, number 7, lat 14, lon 14, age 5. Age is zero because a
  // device reading its own GPS is always current.
  const uint64_t packed = ((uint64_t)role << 40) | ((uint64_t)number << 33)
                        | ((uint64_t)ilat << 19) | ((uint64_t)ilon << 5) | 0;

  out[0] = '!'; out[1] = 'P';
  encDigits(0, 1, &out[2]);              // seq
  encDigits(SELF_TOTAL, 1, &out[3]);     // total 0 == "this is me, not a roster"
  memcpy(&out[4], code, 4);
  encDigits(packed, 7, &out[8]);
  out[SELF_CHARS] = 0;
  return true;
}

bool isTeamMap(const char* text) {
  return text != nullptr && text[0] == '!' && text[1] == 'P';
}

bool decodeSelfReport(const char* text, uint8_t& role, uint8_t& number,
                      double& lat, double& lon) {
  if (!isTeamMap(text)) return false;
  if (strlen(text) != SELF_CHARS) return false;   // exactly one record

  uint64_t total;
  if (!decDigits(&text[3], 1, total)) return false;
  // A roster is the base's picture of everyone. Plotting the first device in
  // it as though it were the sender's own position would put the wrong unit in
  // the wrong place, so a roster is refused here rather than half-read.
  if (total != SELF_TOTAL) return false;

  double alat, alon;
  if (!anchorFromCode(&text[4], alat, alon)) return false;

  uint64_t packed;
  if (!decDigits(&text[8], 7, packed)) return false;

  role   = (uint8_t)((packed >> 40) & 0x03);
  number = (uint8_t)((packed >> 33) & 0x7F);
  const uint32_t ilat = (uint32_t)((packed >> 19) & 0x3FFF);
  const uint32_t ilon = (uint32_t)((packed >> 5) & 0x3FFF);

  const double span = (double)(POS_STEPS - 1);
  lat = alat + (double)ilat / span * BOX_LAT - BOX_LAT / 2.0;
  lon = alon + (double)ilon / span * BOX_LON - BOX_LON / 2.0;
  return true;
}

// ---- backlog ------------------------------------------------------------

uint8_t encodeHistory(char* out, const HistPoint* pts, uint8_t n) {
  if (out == nullptr || pts == nullptr || n == 0) return 0;

  // The first point chooses the anchor, as in history.encode: a message is one
  // box, and a point outside it starts the next message rather than being
  // clamped onto an edge it does not belong on.
  double alat, alon;
  char code[4];
  anchorFor(pts[0].lat, pts[0].lon, alat, alon, code);

  out[0] = '!'; out[1] = 'H';
  encDigits(0, 1, &out[2]);                  // seq
  encDigits(1, 1, &out[3]);                  // total
  memcpy(&out[4], code, 4);

  char* p = out + HIST_HEADER_CHARS;
  uint8_t k = 0;
  for (; k < n && k < HIST_MAX_POINTS; k++) {
    if (!fits(pts[k].lat, pts[k].lon, alat, alon)) break;
    long ilat, ilon;
    offsets(pts[k].lat, pts[k].lon, alat, alon, ilat, ilon);
    // history._age_steps: half-to-even like Python's round(), and clamped
    // rather than refused - a place somebody stood twenty hours ago is still a
    // place somebody stood.
    long age = roundHalfEven((double)pts[k].age_s / (double)HIST_AGE_STEP_S);
    if (age > (long)HIST_AGE_MAX) age = (long)HIST_AGE_MAX;
    const uint64_t packed = ((uint64_t)ilat << 28) | ((uint64_t)ilon << 14)
                          | (uint64_t)age;
    encDigits(packed, HIST_RECORD_CHARS, p);
    p += HIST_RECORD_CHARS;
  }
  *p = 0;
  return k;
}

}  // namespace slimcodec
