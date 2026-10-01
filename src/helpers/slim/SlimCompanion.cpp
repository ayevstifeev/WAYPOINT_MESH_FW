#include "SlimCompanion.h"

#ifdef WITH_MESH_SLIM

#include <ctype.h>
#include <stdio.h>
#include <string.h>

namespace slimcompanion {

// Tracker is the default because a search team is mostly trackers, and a wrong
// role only mislabels a dot rather than misplacing it.
static uint8_t _role[slim::ADDR_COUNT];

void setUnitRole(uint8_t unit, uint8_t role) {
  if (unit <= slim::MAX_ADDR) _role[unit] = role;
}
uint8_t unitRole(uint8_t unit) {
  if (unit > slim::MAX_ADDR) return (uint8_t)slimcodec::ROLE_TRACKER;
  return _role[unit];
}

const char* skipSenderPrefix(const char* text) {
  if (text == nullptr) return nullptr;
  const char* colon = strstr(text, ": ");
  return colon ? colon + 2 : text;
}

bool readSelfReport(const char* text, uint8_t& role, uint8_t& unit,
                    uint8_t out_pos[slimcodec::POS_BYTES]) {
  // Decoding is the test. "!P" also marks a base roster, which is a different
  // thing and must stay on the text path, so the prefix alone proves nothing.
  double lat, lon;
  if (!slimcodec::decodeSelfReport(skipSenderPrefix(text), role, unit,
                                   lat, lon)) {
    return false;
  }
  slimcodec::encodePos(lat, lon, out_pos);
  return true;
}

void callsignFor(uint8_t unit, uint8_t role, char* out, uint8_t cap) {
  // The naming scheme the operation already uses on the voice net. The unit
  // number is the briefing number, not a hash of a public key, which is why
  // the two agree without anything being sent to make them agree.
  if (role == slimcodec::ROLE_BASE) {
    snprintf(out, cap, "MSHCMD");   // the base has no number of its own
    return;
  }
  snprintf(out, cap, "%s%02u",
           role == slimcodec::ROLE_REPEATER ? "MRPT" : "MTRK", (unsigned)unit);
}

uint32_t ackId(uint8_t unit, uint8_t seq) {
  return 0x51000000UL | ((uint32_t)unit << 8) | seq;
}

bool unitFromCallsign(const char* name, uint8_t& role, uint8_t& number) {
  if (name == nullptr) return false;

  char up[16];
  uint8_t n = 0;
  while (name[n] && n < sizeof(up) - 1) {
    up[n] = (char)toupper((unsigned char)name[n]);
    n++;
  }
  up[n] = 0;

  // The base has fixed names with no number of its own. MESHGW is what the
  // gateway was called before the scheme, and it is still on devices in the
  // field. Number 0 matches what wire.callsign_from() round-trips through.
  if (strcmp(up, "MSHCMD") == 0 || strcmp(up, "MESHGW") == 0) {
    role = slimcodec::ROLE_BASE;
    number = 0;
    return true;
  }

  if (n != 6) return false;
  if (up[4] < '0' || up[4] > '9' || up[5] < '0' || up[5] > '9') return false;
  const int num = (up[4] - '0') * 10 + (up[5] - '0');

  if (memcmp(up, "MTRK", 4) == 0)      role = slimcodec::ROLE_TRACKER;
  else if (memcmp(up, "MRPT", 4) == 0) role = slimcodec::ROLE_REPEATER;
  else if (memcmp(up, "MCMD", 4) == 0) role = slimcodec::ROLE_BASE;
  else return false;

  // MTRK00 has no meaning under the scheme, and above 15 there is no unit
  // number to hold it.
  if (num < 1 || num > slim::MAX_ADDR) return false;
  number = (uint8_t)num;
  return true;
}

bool isNoFix(const slim::Frame& f) {
  if (f.type != slim::TYPE_POS) return false;
  if (f.body_len < slimcodec::POS_BYTES) return false;
  for (uint8_t i = 0; i < slimcodec::POS_BYTES; i++) {
    if (f.body[i] != 0) return false;
  }
  return true;
}

int formatNoFix(const slim::Frame& f, char* out, uint8_t cap) {
  if (!isNoFix(f)) return 0;
  if (cap < MAX_TEXT) return 0;
  char callsign[16];
  callsignFor(f.src, unitRole(f.src), callsign, sizeof(callsign));
  // The callsign twice, as a self-report carries it twice: the prefix is
  // MeshCore's and the body is the app's, and a reader of one must never
  // have to trust the other.
  return snprintf(out, cap, "%s: " NO_FIX_PREFIX "%s", callsign, callsign);
}

int formatPosition(const slim::Frame& f, char* out, uint8_t cap) {
  if (f.type != slim::TYPE_POS) return 0;
  if (f.body_len < slimcodec::POS_BYTES) return 0;
  if (cap < MAX_TEXT) return 0;

  double lat, lon;
  if (!slimcodec::decodePos(f.body, f.body_len, lat, lon)) return 0;

  const uint8_t role = unitRole(f.src);
  char callsign[16];
  callsignFor(f.src, role, callsign, sizeof(callsign));

  char report[16];
  // The number in the record is the unit number, so the app resolves the same
  // callsign from the record that we just wrote into the prefix. If those two
  // ever disagree the map shows one device twice under different names.
  if (!slimcodec::encodeSelfReport(report, role, f.src, lat, lon)) return 0;

  return snprintf(out, cap, "%s: %s", callsign, report);
}

}  // namespace slimcompanion

#endif  // WITH_MESH_SLIM
