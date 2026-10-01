// One byte says what a message is. Transcribed from slim_app.py.
//
// Everything that makes tracking useful - the team map, who is carrying which
// callsign, dropped markers, the backlog a unit walked while it was out of
// range - was written for MeshCore, where it rides as channel text behind a
// two-character header: "!P", "!N", "!M", "!H", "!U", "!T", "!S", "!F".
//
// On this network that header is the wrong shape twice over. It spends two
// characters of the thing the protocol exists to save, and the bodies are
// base64url, which is mixed case - and PACK6 is uppercase only. PACK6 does not
// refuse lowercase, it UPPERCASES it, so a roster handed to the text tier
// arrives looking like a roster and decodes to different places. Positions in
// the wrong valley, with nothing anywhere reporting a fault.
//
// So the header becomes one byte and the body stops being text:
//
//     byte 0   TYPE      which format this is; 16 and up
//     byte 1   NDIGITS   how many characters were base64url - omitted when
//                        TYPE has bit 0x80 set, because then all of them are
//     ...      those characters, six bits each, most significant first
//     ...      the rest, seven bits each, padded with 1 bits
//
// The byte is free: slimtext already puts a tier byte (0..3) at the front of
// every body, and a TYPE of 16 or more sits in the same place saying "not
// text, this". No new frame type is needed - which matters, because the 3-bit
// type field is full and a ninth type would mean a new sync word and a
// whole-team reflash.
//
// **Six bits is not a compromise.** These formats were packed into base64url
// because MeshCore's companion frame carries text, and a base64url digit is
// exactly six bits of payload. Unpacking them back to six bits each does not
// approximate the binary form of a roster - it is the binary form.
//
// **Seven for the rest, because case is the whole point.** A marker label and
// a person's name are mixed case ("Rockfall", "Mike"). Seven bits is every
// printable ASCII character with no shift state and nothing to agree on.
// Anything outside printable ASCII is refused here and falls back to the RAW
// text tier, which is UTF-8 and refuses nothing.
//
// Nothing here parses any of these formats. The encoder takes the longest run
// of base64url characters at the front at six bits and the rest at seven, so a
// format can change without this file knowing, and what comes out the far end
// is what went in, character for character.
//
// Cross-checked against the Python in test_codecs.cpp.

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace slimapp {

//: Where app types start. 0..3 are slimtext's tiers and 4..15 are kept for
//: more of them, so a tier and a type can never be confused by an
//: implementation that knows only one of the two.
static const uint8_t FIRST_TYPE = 0x10;

//: Set when every character is a base64url digit - the whole of a roster, a
//: backlog, and a marker whose label happens to be letters. NDIGITS is then
//: not on the wire and the header really is one byte, which is the case that
//: matters: the roster is the biggest and most frequent message the base
//: sends.
static const uint8_t ALL_DIGITS = 0x80;

enum Type : uint8_t {
  TYPE_TEAM    = FIRST_TYPE,      // "!P"  the team map: roster and self-report
  TYPE_NAME    = FIRST_TYPE + 1,  // "!N"  who is carrying which callsign
  TYPE_MARKER  = FIRST_TYPE + 2,  // "!M"  a dropped pin, or its removal
  TYPE_HISTORY = FIRST_TYPE + 3,  // "!H"  the track walked while out of range
  TYPE_URGENT  = FIRST_TYPE + 4,  // "!U"  a broadcast wanting an acknowledgement
  TYPE_TRACKER = FIRST_TYPE + 5,  // "!T"  report every n seconds, or stop
  TYPE_PROTO   = FIRST_TYPE + 6,  // "!S"  move to the other wire format
  TYPE_FIND    = FIRST_TYPE + 7,  // "!F"  find mode, for the pet and kid finder
  //: A node's own command line, and its reply - see
  //: MyMesh::handleSlimAdmin. It carries no commands of its own; it
  //: carries the ones the USB console and MeshCore admin already have.
  //:
  //: **This is why it is a datagram and not text.** slimtext PACK6 folds
  //: case, so "set name Ridge Top" would arrive as "SET NAME RIDGE TOP" -
  //: not a command, and not the name anybody asked for. Seven-bit packing
  //: is the whole reason this needs a type byte.
  TYPE_ADMIN   = FIRST_TYPE + 8,  // "!A"  the command line, both ways
  //: Who I am: callsign and public key.
  //:
  //: MeshCore learns a node from its advert, which carries a signed 32-byte
  //: identity and cannot fit a slim frame. So on slim a unit was never
  //: learned at all: its positions drew on the map under a callsign, but it
  //: had no key, so it could not be a contact - could not be picked in
  //: chat, could not be sent to, and its own messages to the base were
  //: DROPPED for having no contact to thread to.
  //:
  //: This is the one thing slim could not say. It is rare traffic - once an
  //: hour and on demand - so it is spelled out in hex rather than packed:
  //: about 53 bytes, against the 12 a position costs.
  TYPE_ADVERT  = FIRST_TYPE + 9,  // "!V"  callsign and public key
};
static const uint8_t TYPE_COUNT = 10;

//: A roster fills wire.MAX_TEXT (150 characters) and the rebuilt "NAME: "
//: prefix takes eight more. Bigger than slimtext::MAX_TEXT on purpose: the
//: text tiers only ever carried chat, and these formats are why the buffer
//: has to grow.
static const uint8_t MAX_TEXT = 176;

//: base64url. The index is the six-bit value, so the order is the wire format.
extern const char DIGITS[65];

//: True if this body is a datagram rather than text - one byte, read from the
//: position both formats already use.
bool isApp(const uint8_t* body, uint8_t len);

//: The type for a message, or 0 if it is ordinary text.
uint8_t typeOf(const char* text);

//: A datagram body. Returns its length, or -1 if this message is not one of
//: these formats, holds a character outside printable ASCII, or will not fit
//: `cap`.
int encode(const char* text, uint8_t* out, uint8_t cap);

//: The message back, character for character. Returns the length written, or
//: -1 for anything that is not a whole datagram. Never half-decodes: a
//: truncated roster must read as absent, not as people somewhere else.
int decode(const uint8_t* body, uint8_t len, char* out, uint8_t cap);

//: What to actually put in a frame, for any message at all.
//:
//: Never worse than what the network already sends. "!T120" and "!S1" are
//: uppercase, so PACK6 carries them faithfully and in fewer bytes, and they
//: keep going exactly as they go today - a format that works in the field is
//: not changed for consistency. The datagram is used where it is smaller, and
//: where it is the only thing that works at all.
int bodyFor(const char* text, uint8_t* out, uint8_t cap);

//: The inverse, for a body that arrived: the first byte says which it is.
int decodeBody(const uint8_t* body, uint8_t len, char* out, uint8_t cap);

}  // namespace slimapp
