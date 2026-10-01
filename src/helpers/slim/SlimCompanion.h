// The companion bridge: MESH_SLIM frames in, the app's own formats out.
//
// This is where the "identical API" claim is actually cashed. The phone app and
// the Pi gateway send CMD_SEND_CHANNEL_TXT_MSG carrying a teammap "!P" string
// and expect the same thing back on the way in. Neither knows the LoRa side has
// changed. So the bridge takes the app's "!P" apart, sends the position as a
// 13-byte slim frame, and rebuilds the identical "!P" at the far end.
//
//   app  "MTRK07: !PAAQD98AMSm91g"    39 chars of channel text
//                 |
//                 v  decodeSelfReport -> lat/lon -> encodePos
//   air  13 bytes                     <- this is the whole point
//                 |
//                 v  decodePos -> encodeSelfReport
//   app  "MTRK07: !PAAQD98AMSm91g"    character for character the same string
//
// The roster path is NOT here. Only a single-device self-report is handled;
// a base roster stays on MeshCore. That keeps the port narrow - one anchor and
// one record - instead of duplicating a codec that already exists in the app
// and the gateway.

#pragma once

#include "SlimProto.h"

#ifdef WITH_MESH_SLIM

#include "SlimCodecs.h"

namespace slimcompanion {

//: Longest text this bridge produces: "MSHCMD: " plus a 15-character report.
static const uint8_t MAX_TEXT = 40;

//: Step past MeshCore's "NAME: " sender prefix.
//:
//: That prefix is not decoration - MeshCore writes it into the body because a
//: group packet carries no sender field - so anything matching a channel
//: message for meaning has to look past it first. "MTRK01: !P..." does not
//: start with "!P".
const char* skipSenderPrefix(const char* text);

//: Read a channel text as a position, in one pass.
//:
//: True only for a single-device "!P" self-report: human chat, "!U" urgent and
//: a base roster all return false and belong on the text path.
//:
//: Returns the values rather than a bool alone because the caller needs them
//: immediately - asking "is this a position?" and then "what position?" decodes
//: 12 base64url digits twice on the most frequent packet in the network.
bool readSelfReport(const char* text, uint8_t& role, uint8_t& unit,
                    uint8_t out_pos[slimcodec::POS_BYTES]);

//: Rebuild the channel text for a slim POS that just arrived. Writes
//: "CALLSIGN: !P..." and returns its length, or 0 if the frame is not a
//: position this bridge can render.
int formatPosition(const slim::Frame& f, char* out, uint8_t cap);

//: The app code for "heard, no fix". One letter, beside "!P" for a
//: position and "!H" for history - see CHANNEL_PROTOCOL in track_page.py
//: and NO_FIX_PREFIX in history.py, which have to agree with this.
#define NO_FIX_PREFIX "!X"

//: Is this a unit saying "you can hear me, I cannot see the sky"?
//:
//: A position body of all zeros, which slim_node.py has always read this
//: way. It has to be asked BEFORE decoding, because a position codec has
//: no opinion about which coordinates are real: those six zero bytes are
//: a perfectly good encoding of 90S 180W, and that is where a no-fix
//: report was appearing on the map - a team at the South Pole, drawn with
//: exactly the confidence of a team standing in front of you.
bool isNoFix(const slim::Frame& f);

//: Rebuild the channel text for a no-fix report: "CALLSIGN: !XCALLSIGN".
//:
//: Its own code rather than a position, because it is not one. Heard with
//: no fix and not heard at all are the two states an operator must never
//: have to tell apart by eye: the first is a radio working under a canopy
//: that is not, the second is a team to go and look for.
int formatNoFix(const slim::Frame& f, char* out, uint8_t cap);

//: The callsign a unit number maps to, for the "NAME: " prefix. Follows the
//: briefing sheet, so unit 7 is MTRK07 - the same string people say on the
//: voice net.
void callsignFor(uint8_t unit, uint8_t role, char* out, uint8_t cap);

//: The inverse: MTRK07 -> (tracker, 7). False if the name does not follow the
//: scheme, or if the number is outside what a 4-bit unit can address.
//:
//: **This is the whole roster.** The unit number is already in the callsign, so
//: walking MeshCore's own contact list is enough to build the pubkey<->unit map
//: - no new command, no QR code, and no data that does not already exist. See
//: MyMesh::loadSlimRoster().
//:
//: Names go to 99 and units stop at 15, so MTRK16 and up are unaddressable.
//: They are refused here rather than truncated, because a silently wrapped
//: number would map two devices onto one unit and they would eat each other's
//: packets - a fault that reads as a range problem.
bool unitFromCallsign(const char* name, uint8_t& role, uint8_t& number);

//: Roles are per unit because a repeater's position should not be labelled as
//: a tracker's on the map. Defaults to tracker, which is what a search team is
//: mostly made of.
void setUnitRole(uint8_t unit, uint8_t role);
uint8_t unitRole(uint8_t unit);

//: The 4-byte id the app uses to match a sent message to its acknowledgement.
//:
//: Both ends compute it from the same two facts, so nothing has to carry it:
//: the sender stores `ackId(recipient, seq)` when it transmits, and recomputes
//: `ackId(sender_of_the_ack, seq_in_the_ack_body)` when one comes back. A slim
//: ACK therefore costs **one byte of body** - the sequence number - rather than
//: the four MeshCore spends on a hash, which is most of why it fits in 9 bytes.
//:
//: The top byte is a constant marker. It has no protocol meaning; it only keeps
//: these ids clear of the MeshCore hashes that share the app's table when a
//: dual-protocol node switches over.
uint32_t ackId(uint8_t unit, uint8_t seq);

}  // namespace slimcompanion

#endif  // WITH_MESH_SLIM
