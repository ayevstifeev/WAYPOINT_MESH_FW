// The two codecs the companion bridge needs, transcribed from the Python.
//
//   SlimPos      the 6-byte position that goes on the air
//   TeamMapSelf  the "!P" self-report the phone app already speaks
//
// The second one is a NARROW port on purpose. teammap.py is a full roster
// codec - chunked multi-device messages, age buckets, anchor-per-chunk - and
// porting all of it would make a third implementation of a format that already
// exists twice (the phone app's decodeTeam() and the Python). Only the
// single-device self-report is here: one anchor, one record, no chunking. The
// base station keeps doing every roster job it does today.
//
// Cross-checked byte for byte against the Python in test_codecs.cpp.

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace slimcodec {

// ---- position, 6 bytes --------------------------------------------------
//
// 24 bits of latitude over 180 degrees and 24 of longitude over 360: 1.2 m
// north-south and 1.7 m east-west at Glenorchy, anywhere on Earth, with no
// grid origin to agree at the briefing and no box to walk out of. Measured
// worst case over 20,000 random points on the globe is 0.60 m and 1.19 m.
//
// No quality byte: fix type and battery ride on STATUS. Battery does not
// change between position reports, so a byte on every one of them was paying
// continuously for something read occasionally.
static const uint8_t POS_BYTES = 6;

void encodePos(double lat, double lon, uint8_t out[6]);

//: false if `len` is short. Never invents a place from a corrupt packet.
bool decodePos(const uint8_t* body, uint8_t len, double& lat, double& lon);

// ---- teammap self-report ------------------------------------------------
//
// The wire format, from teammap.py:
//
//     !P <seq:1> <total:1> <anchor:4> <record:7>
//
// all base64url digits, most significant first. `total` of 0 is the reserved
// value meaning "one device reporting itself" rather than a base roster.
//
// The record is 42 bits: role 2, number 7, lat 14, lon 14, age 5 - offsets
// inside a 0.5-degree box around the anchor.

static const uint8_t SELF_CHARS = 15;    // "!P" + 1 + 1 + 4 + 7
static const uint8_t SELF_TOTAL = 0;

enum Role : uint8_t { ROLE_TRACKER = 0, ROLE_REPEATER = 1, ROLE_BASE = 2 };

//: Writes SELF_CHARS characters plus a NUL. `out` needs 16 bytes.
//: Returns false if the number will not fit 7 bits.
bool encodeSelfReport(char* out, uint8_t role, uint8_t number,
                      double lat, double lon);

//: Reads one back. False for anything that is not a single-device "!P".
//: A roster from the base decodes as `total != 0` and is rejected here - the
//: bridge must not plot the base's whole picture as though it were the
//: sender's own position.
bool decodeSelfReport(const char* text, uint8_t& role, uint8_t& number,
                      double& lat, double& lon);

//: True if the text is a "!P" broadcast at all, roster or self-report.
bool isTeamMap(const char* text);

// ---- a dropped pin ("!M") ----------------------------------------------
//
// From markers.py, and a deliberate sibling of the self-report: same
// alphabet, same anchor grid, same 14-bit offsets, so a reader that
// understands one understands the other.
//
//     !M <anchor:4> <record:8> <label>
//
// The record is 48 bits: creator 9 (role 2 then number 7, exactly as
// callsign_code packs it), lat 14, lon 14, icon 5, marker number 6.
//
//: **Identity is the creator plus that number**, which is what makes a
//: marker editable rather than write-only: sending the same pair again
//: moves it, renames it, or - with ICON_REMOVED - takes it off the map.
//: So a device has to keep its own counter rather than picking at random,
//: or its second pin replaces its first.

static const uint8_t MARKER_CHARS     = 14;   // "!M" + 4 + 8
static const uint8_t MARKER_MAX_LABEL = 24;
static const uint8_t MARKER_PER_DEVICE = 64;
//: Not an icon: a marker sent with this is being taken off the map.
static const uint8_t ICON_REMOVED = 31;

//: Writes the whole broadcast, label included, and NUL-terminates it.
//: Returns its length, or -1 if it will not fit `cap` or an argument is
//: out of range. The label is trimmed exactly as markers.py trims it, so
//: the two stacks agree on a label typed with a trailing space.
int encodeMarker(char* out, uint8_t cap, uint8_t role, uint8_t number,
                 double lat, double lon, uint8_t icon, uint8_t mnum,
                 const char* label);

// ---- backlog ("!H") -----------------------------------------------------
//
// From history.py: the track a unit walked while it could not reach the base,
// handed over once it can. The same 8-character header, 7-character record,
// anchor and box as the self-report:
//
//     !H <seq:1> <total:1> <anchor:4> <point:7> ...
//
// A point is 42 bits: lat 14, lon 14, age 14 - how long before the message
// was sent, in four-second steps. Ages rather than times, so a unit whose
// clock was never set still dates its track correctly.

static const uint8_t  HIST_HEADER_CHARS = 8;
static const uint8_t  HIST_RECORD_CHARS = 7;
static const uint32_t HIST_AGE_STEP_S   = 4;
static const uint32_t HIST_AGE_MAX      = 0x3FFF;   // 18.2 hours
//: The firmware's own cut for a direct message; history.MAX_DM_TEXT.
static const uint8_t  HIST_MAX_TEXT     = 140;
//: Points one message can carry: (140 - 8) / 7.
static const uint8_t  HIST_MAX_POINTS   =
    (HIST_MAX_TEXT - HIST_HEADER_CHARS) / HIST_RECORD_CHARS;

struct HistPoint { double lat, lon; uint32_t age_s; };

//: One "!H" message from the oldest of `n` points, oldest first. `out` needs
//: HIST_MAX_TEXT + 1 bytes. Returns how many points it carries: fewer than `n`
//: when the message is full or the next point lies outside the first one's
//: box - the rest go in the next message, exactly where history.encode would
//: start a new one. 0 when there is nothing to send.
//:
//: Every message is whole in itself - sequence 0 of 1 - because a unit sends
//: them one at a time, each only after the last was acknowledged, and cannot
//: know how many there will be. The base uses the pair only to log progress.
uint8_t encodeHistory(char* out, const HistPoint* pts, uint8_t n);

}  // namespace slimcodec
