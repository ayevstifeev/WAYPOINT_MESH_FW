// The MESH_SLIM frame, in C++.
//
// A transcription of slim_packet.py, not a reimplementation of it. The Python
// is the reference; where the two could differ they are checked against each
// other byte for byte by test_slim.cpp against vectors.h, which is generated
// from the Python and never edited by hand.
//
// Constraints this file works under, because it runs on a radio:
//   * no dynamic allocation, no STL containers, no exceptions
//   * no assumption about alignment or endianness of the host
//   * every buffer bounded at compile time
//
// See DESIGN.md sections 3, 7 and 8.

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace slim {

// ---- operation-wide constants, set at briefing --------------------------
//
// These are the ones DESIGN.md calls "the same on every unit, and therefore
// needing no bit on the wire to announce it". They are variables rather than
// constants for exactly that reason: they are configured, not compiled.

//: The sync word carries the protocol version (DESIGN.md section 3). The
//: SX126x compares it before raising an RX interrupt, so units on different
//: versions are inaudible to each other rather than mis-parsing each other.
static const uint8_t SYNC_WORD     = 0x2B;
static const uint8_t PROTO_VERSION = 0;

//: MeshCore sends 16 symbols (32 below SF9). At 12 bytes the preamble is about
//: 40% of the whole transmission, so this is the single biggest airtime item
//: left after the payload. Eight is the floor: below it the receiver starts
//: missing packets it would otherwise have decoded, which trades a certainty
//: for a saving and is the wrong direction for this protocol.
static const uint16_t PREAMBLE_SYMBOLS = 8;

static const uint8_t TAG_BYTES = 4;

//: ttl counts decrements, not hops: a packet sent at ttl=T is repeated T times
//: and crosses T+1 radio hops. So 7 is the ceiling and eight hops is what the
//: 3-bit field reaches. Capping a network at seven hops frees no bits.
static const uint8_t MAX_TTL  = 7;
static const uint8_t MAX_HOPS = MAX_TTL + 1;

//: Receivers recover hop count as TTL_INIT - ttl + 1, so this must be the same
//: on every unit in the operation or they compute each other's distance wrongly
//: and prefer the wrong routes.
extern uint8_t  TTL_INIT;

//: Separates one operation's keystream from the next even if a key is reused by
//: mistake. Never transmitted.
extern uint32_t OPERATION_EPOCH;

// SRC and HOP share a byte, DST takes a nibble: fifteen units per operation,
// and a 12-byte position that stays under the SF12 airtime step at 13 bytes.
// Turning this off costs one byte and 328 ms per position at SF12.
#ifndef SLIM_ADDR_NIBBLE
#define SLIM_ADDR_NIBBLE 1
#endif

#if SLIM_ADDR_NIBBLE
static const uint8_t MAX_ADDR   = 0x0F;
static const uint8_t ADDR_COUNT = 16;
#else
static const uint8_t MAX_ADDR   = 0xFF;
static const uint16_t ADDR_COUNT = 256;
#endif

static const uint8_t ADDR_BROADCAST = 0x00;

//: The base station, by the convention set at the briefing: trackers
//: count up from 1 and repeaters down from 14, so the top of the range
//: is the one address every unit can assume without being told.
//:
//: Named rather than spelled MAX_ADDR, which it currently equals and
//: does not mean: MAX_ADDR is the width of the address field, and
//: becomes 255 the moment SLIM_ADDR_NIBBLE is turned off.
static const uint8_t ADDR_BASE = 0x0F;

//: Generous: a position is 13 bytes and a direct message rarely thirty.
//:
//: It is this big for one message - the base's team map. A roster fills
//: wire.MAX_TEXT (150 characters) and packs to about 113 bytes as an app
//: datagram, so at 64 the largest and most useful broadcast in the network
//: would simply not fit and would be refused with no explanation anywhere.
//: The cost is RAM in the two buffers sized from it, not airtime: nothing
//: here makes a message longer, and the base keeps its chunks small on slim
//: so that one fade costs one chunk rather than the whole picture.
#ifndef SLIM_MAX_FRAME
#define SLIM_MAX_FRAME 192
#endif
static const uint8_t MAX_BODY = SLIM_MAX_FRAME - 8;

enum RouteMode : uint8_t { ROUTE_FLOOD = 0, ROUTE_DIRECT = 1 };

//: TYPE_GRPTEXT was the reserved eighth slot, spent on team chat. There is no
//: way to broadcast text without it - TEXT is addressed, and ALERT carries
//: transmit priority, so routine chat riding on ALERT would jump the queue
//: ahead of distress traffic. Sending chat as N addressed copies was the other
//: option and reintroduces exactly the flooding this design exists to avoid.
//:
//: **The type space is now full.** A ninth type needs a wider field, which
//: means a version bump, a new sync word, and a whole-team reflash.
enum Type : uint8_t {
  TYPE_POS = 0, TYPE_TEXT, TYPE_ACK,   TYPE_PING,
  TYPE_STATUS, TYPE_ROSTER, TYPE_ALERT, TYPE_GRPTEXT
};

//: Types that go to everyone and therefore carry no DST byte at all. The type
//: already says so, so a destination field would be a constant zero on the most
//: common packet in the network.
inline bool isBroadcastType(uint8_t t) {
  return t == TYPE_POS || t == TYPE_ROSTER || t == TYPE_ALERT
      || t == TYPE_GRPTEXT;
}

//: Not sent at TTL_INIT, so its arriving ttl says nothing about distance and
//: feeding it to the route table invents short routes to distant nodes.
inline bool teachesRoutes(uint8_t t) { return t != TYPE_ACK; }

// ---- results ------------------------------------------------------------
//
// ERR_BAD merges "malformed" and "failed authentication", exactly as the Python
// merges them into one BadPacket: a sender must not be able to tell which
// happened, or the failure itself becomes an oracle. ERR_NEEDS_CIPHER is
// separate because it is a misconfigured receiver rather than a hostile packet,
// and reporting it as corruption sends someone hunting the wrong fault.
enum Result : int8_t {
  OK               =  0,
  ERR_BAD          = -1,
  ERR_NEEDS_CIPHER = -2,
  ERR_ARG          = -3
};

// ---- cipher -------------------------------------------------------------

// One method, deliberately: CTR is its own inverse, so there is no
// encrypt/decrypt pair to call the wrong way round.
//
// NOTHING IN THIS REPOSITORY IMPLEMENTS AES. A conformant implementation wraps
// the MCU's hardware AES block; hand-rolled crypto in a design repo gets picked
// up and shipped. Building with enc and no cipher fails closed rather than
// quietly transmitting plaintext.
class Cipher {
public:
  virtual ~Cipher() {}
  virtual void xorStream(const uint8_t nonce16[16], uint8_t* data, uint8_t len) = 0;
};

//: The 16 bytes AES-CTR starts from. Never carried on the wire — every input is
//: either a briefing constant or already in the frame, so confidentiality costs
//: zero bytes.
//:
//: `seq_ext` is the full send counter, not the byte in the frame. SEQ is eight
//: bits and wraps every 256 packets; CTR reuses its keystream the instant a
//: nonce repeats, and two bodies under one keystream leak both.
void makeNonce(uint8_t out[16], uint8_t src, uint8_t type, uint32_t seq_ext);

// ---- frame --------------------------------------------------------------

struct Build {
  uint8_t  type     = TYPE_POS;
  uint8_t  src      = 0;
  uint8_t  seq      = 0;
  uint8_t  dst      = ADDR_BROADCAST;
  uint8_t  hop      = 0;
  uint8_t  ttl      = 0;            // 0 means "use TTL_INIT"
  uint8_t  route    = ROUTE_FLOOD;
  bool     enc      = false;
  uint32_t seq_ext  = 0;
  const uint8_t* body = nullptr;
  uint8_t  body_len = 0;
  bool     ttl_set  = false;        // distinguishes an explicit ttl of 0
};

struct Frame {
  uint8_t type;
  bool    enc;
  uint8_t route;
  uint8_t ttl;
  uint8_t hops;
  uint8_t src, hop, dst;
  uint8_t seq;
  uint8_t body[MAX_BODY];
  uint8_t body_len;

  bool isDirect() const { return route == ROUTE_DIRECT; }
};

//: Called after the tag has been checked and before the body is touched, to
//: turn a wire SEQ into the sender's wide counter. The ordering is the whole
//: defence: a receiver that let unauthenticated packets advance its wrap state
//: could be walked forward by an attacker until it reused a keystream.
typedef uint32_t (*SeqExtFn)(void* ctx, uint8_t src, uint8_t seq);

//: Returns the frame length, or a negative Result.
int build(const uint8_t key[16], const Build& b, Cipher* cipher,
          uint8_t* out, size_t cap);

//: Returns OK or a negative Result. Never fills `f` with unauthenticated data.
int parse(const uint8_t key[16], const uint8_t* frame, uint8_t len,
          Frame& f, Cipher* cipher = nullptr,
          SeqExtFn seq_ext = nullptr, void* ctx = nullptr);

//: The same packet with ttl decremented and the HOP byte rewritten, in place.
//: Returns the new length, or 0 for "do not repeat" (the ttl is spent).
//: Re-tagging is not needed and not done: the tag never covered ttl or HOP.
//:
//: What goes in HOP depends on the route mode, and the two meanings are
//: opposite ends of the same byte:
//:   FLOOD  - HOP is where this came from, so a repeater writes its own id
//:            and every receiver learns a route back. Pass next_hop = hop = me.
//:   DIRECT - HOP is who may carry it next, so a repeater writes the node it is
//:            handing off to. Pass next_hop = routes.via(dst).
//: Writing your own id into a DIRECT packet makes it name its own last carrier
//: as its next carrier, and it bounces between two nodes until the ttl expires.
uint8_t forward(uint8_t* frame, uint8_t len, uint8_t next_hop);

// ---- SipHash-2-4 --------------------------------------------------------

uint64_t siphash24(const uint8_t key[16], const uint8_t* data, size_t len);
void     sipTag(const uint8_t key[16], const uint8_t* data, size_t len,
                uint8_t* out, uint8_t size);
bool     sipVerify(const uint8_t key[16], const uint8_t* data, size_t len,
                   const uint8_t* given, uint8_t size);

}  // namespace slim
