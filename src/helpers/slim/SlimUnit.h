// The MESH_SLIM address a callsign carries, worked out at compile time.
//
// **One knob, because two disagreed.** A board's identity used to be set in
// two places - ADVERT_NAME, the name it adverts, and MESH_SLIM_UNIT, the
// address it answers on - and they were set independently. A repeater
// shipped as name MRPT01 (unit 14) with MESH_SLIM_UNIT=9, which is MTRK09's
// address: it answered as a tracker that may also exist, and nothing on the
// wire can tell two nodes on one address apart. Both transmit, both are
// heard, and traffic for one is delivered to the other while every screen
// looks healthy.
//
// So the address is derived here and the callsign is the only thing anyone
// sets - `callsign` under [slim] in platformio.ini, one line, per board.
// A board already carrying saved prefs ignores all of this: loadPrefs reads
// node_name back out of flash and the runtime derives the address again from
// that (simple_repeater's loadSlimUnit, slimcompanion::unitFromCallsign).
// This is the value used before that has happened.
//
// Note the name: slimcompanion::unitFromCallsign answers a different
// question - it splits a callsign into a ROLE and the number printed on the
// unit. This turns a callsign into the address it answers on, which for a
// repeater is not that number. slim_node.unit_from_callsign on the Pi is the
// one that matches this, and test_slim_unit.py checks the two against each
// other over every callsign in the scheme.

#pragma once

#include <stdint.h>

#include "SlimPacket.h"

namespace slimcfg {

//: Length, so nothing below reads past the end of a short name. Recursive
//: because a C++11 constexpr is a single return statement, and the names are
//: six characters, so the depth is seven.
constexpr uint8_t nameLength(const char* name) {
  return *name ? (uint8_t)(1 + nameLength(name + 1)) : (uint8_t)0;
}

constexpr bool hasPrefix(const char* name, char a, char b, char c, char d) {
  return name[0] == a && name[1] == b && name[2] == c && name[3] == d;
}

constexpr bool hasTwoDigits(const char* name) {
  return name[4] >= '0' && name[4] <= '9'
      && name[5] >= '0' && name[5] <= '9';
}

//: The number printed on the unit: the "07" of MTRK07.
constexpr uint8_t twoDigits(const char* name) {
  return (uint8_t)((name[4] - '0') * 10 + (name[5] - '0'));
}

//: The base's fixed name, which carries no number of its own.
constexpr bool isBaseName(const char* name) {
  return hasPrefix(name, 'M', 'S', 'H', 'C')
      && name[4] == 'M' && name[5] == 'D';
}
constexpr bool isRepeaterName(const char* name) {
  return hasPrefix(name, 'M', 'R', 'P', 'T') && hasTwoDigits(name);
}
constexpr bool isTrackerName(const char* name) {
  return hasPrefix(name, 'M', 'T', 'R', 'K') && hasTwoDigits(name);
}
//: MCMDnn - a base, numbered for a human rather than for the wire.
constexpr bool isTeamBaseName(const char* name) {
  return hasPrefix(name, 'M', 'C', 'M', 'D') && hasTwoDigits(name);
}

//: MTRK00 has no meaning under the scheme, and above MAX_ADDR there is no
//: address to hold it - refused rather than wrapped, because a wrapped
//: number puts two devices on one address.
//:
//: **A repeater stops one short**, because its number is subtracted: MRPT14
//: is unit 1 and MRPT15 would be unit 0, which is nobody. The Pi's
//: unit_from_callsign refuses it on the same boundary (1 <= n < MAX_ADDR),
//: and the two have to agree or a board adverts a name the base cannot
//: address.
constexpr bool hasUsableNumber(const char* name) {
  return isRepeaterName(name)
           ? (twoDigits(name) >= 1 && twoDigits(name) < slim::ADDR_BASE)
       : (isTrackerName(name) || isTeamBaseName(name))
           ? (twoDigits(name) >= 1 && twoDigits(name) <= slim::MAX_ADDR)
       : false;
}

//: Checked before anything reads a digit: `&&` short-circuits inside a
//: constant expression, so a name that is not six characters is never
//: subscripted.
constexpr bool isCallsign(const char* name) {
  return nameLength(name) == 6
      && (isBaseName(name) || hasUsableNumber(name));
}

//: **Repeaters count down from the base and trackers count up.** MRPT01 is
//: 14, MRPT02 is 13; MTRK01 is 1. The same rule as loadSlimUnit in the
//: repeater firmware and unit_from_callsign on the Pi - and the reason the
//: two ranges meet in the middle, so MRPT01 and MTRK14 are both unit 14. The
//: base reports that clash; nothing here can see it, because this is handed
//: one name at a time.
//:
//: **Every base-role name is the top address**, numbered or not: MSHCMD and
//: MCMD07 are both unit 15. The number in an MCMDnn name says which base it
//: is for a human, and there is one base on a channel - so it is not an
//: address, and treating it as one sent commands meant for the base to
//: whoever was carrying MTRK07.
constexpr uint8_t addressFromCallsign(const char* name) {
  return (isBaseName(name) || isTeamBaseName(name)) ? slim::ADDR_BASE
       : isRepeaterName(name) ? (uint8_t)(slim::ADDR_BASE - twoDigits(name))
       : twoDigits(name);
}

}  // namespace slimcfg

#if defined(WITH_MESH_SLIM) && defined(ADVERT_NAME)

  //: A name off the scheme has no address, and a build that cannot say who it
  //: is must not reach a board: there is nothing a repeater on a hill could
  //: do about it and nothing it could say.
  static_assert(slimcfg::isCallsign(ADVERT_NAME),
                "ADVERT_NAME must be a WayPoint callsign: MTRK01-MTRK15, "
                "MRPT01-MRPT14, MCMD01-MCMD15 or MSHCMD. Set it on one line, "
                "as `callsign` under [slim] in platformio.ini.");

  //: **And that it is the right KIND of callsign for this env.**
  //:
  //: One `callsign` line serves every env, which is the point - you set it to
  //: the board in your hand. What it cannot do by itself is notice that the
  //: board in your hand is a handheld while the line still says MRPT01 from
  //: the last repeater. That compiles: MRPT01 is a perfectly valid callsign,
  //: just not a tracker's, and the handheld comes up claiming to be a
  //: repeater and answering on its address. Caught here, where the answer is
  //: a build error naming both halves, rather than in the field.
  #ifdef MESH_SLIM_EXPECT_TRACKER
    static_assert(slimcfg::isCallsign(ADVERT_NAME)
                    && slimcfg::isTrackerName(ADVERT_NAME),
                  "This env builds a TRACKER, so `callsign` under [slim] in "
                  "platformio.ini has to be MTRK01-MTRK15. It is not - as it "
                  "stands you would flash a handheld with a repeater's or "
                  "the base's callsign.");
  #endif
  #ifdef MESH_SLIM_EXPECT_REPEATER
    static_assert(slimcfg::isCallsign(ADVERT_NAME)
                    && slimcfg::isRepeaterName(ADVERT_NAME),
                  "This env builds a REPEATER, so `callsign` under [slim] in "
                  "platformio.ini has to be MRPT01-MRPT14 (MRPT14 is unit 1; "
                  "there is no MRPT15). It is not.");
  #endif

  //: Setting it by hand still wins, for the Pi's own build and for one-offs.
  #ifndef MESH_SLIM_UNIT
    #define MESH_SLIM_UNIT (slimcfg::addressFromCallsign(ADVERT_NAME))
  #endif

#elif defined(WITH_MESH_SLIM) && !defined(MESH_SLIM_UNIT)
  #error "a MESH_SLIM build needs ADVERT_NAME (see [slim] in platformio.ini)"
#endif
