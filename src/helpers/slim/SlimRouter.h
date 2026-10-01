// The receive and forward decision, transcribed from slim_packet.py.
//
// MESH_SLIM exists twice: C++ here, Python in pymc_core on the base station.
// The frame is the safe half to duplicate — a disagreement fails a tag check
// and shows up in seconds. THIS is the dangerous half. A forwarding rule that
// differs between the two produces a network that mostly works and loses one
// unit in a gully.
//
// So this file is a transcription of the Python class, not an independent
// implementation of the same idea, and test_slim.cpp runs the same table of
// cases against it. If you change a rule here, change it there in the same
// commit or the divergence is silent.
//
// No allocation, no STL, no clock of its own: time arrives as a parameter so
// the whole thing can be run as a network on a host.

#pragma once

#include "SlimPacket.h"

namespace slim {

//: Outside the nibble address range, so it can never collide with a unit id.
static const uint8_t NO_ROUTE = 0xFF;

//: A route unheard from for this long is not trusted. Units move, and a stale
//: route is worse than no route: no route floods and gets through, a wrong
//: route is delivered to somebody who has walked out of range.
static const uint32_t ROUTE_TTL_MS = 900000UL;   // 15 minutes

static const uint32_t DEDUP_WINDOW_MS = 60000UL;

#ifndef SLIM_SEEN_SIZE
#define SLIM_SEEN_SIZE 48
#endif

// ---- route table --------------------------------------------------------

//: What one extra hop is worth, in dB of link margin. See Routes::cost.
//:
//: **A briefing constant, and four is set for sheer valleys.** It is the
//: exchange rate between the two things that are scarce: airtime, which an
//: extra hop spends, and fade margin, which an extra hop buys. In steep,
//: bush-covered country margin is by far the scarcer of the two - a path
//: with 5 dB of headroom is one person walking into scrub away from
//: nothing, while a clear line to a repeater on a ridge has room to spare.
//: So a hop is priced cheaply here and the relay usually wins.
//:
//: Raise it for a flat, busy network where airtime is what runs out; at 4,
//: a two-hop path only has to be 4 dB better than a one-hop one to take
//: over. Change it on every unit or not at all: two units briefed
//: differently will choose different ways home, which is confusing rather
//: than broken, but still confusing.
static const int16_t HOP_COST_DB = 4;

//: How much better a route has to be before it replaces the one in use.
//:
//: Without it a node sitting between two comparable neighbours changes its
//: mind on the noise of each individual packet, and every change is a
//: message sent down a path that has just been abandoned. Three dB is half
//: a hop: enough that a real difference still wins promptly.
static const int16_t ROUTE_HYST_DB = 3;

//: The weakest route worth naming a single carrier on, in dB.
//:
//: **A DIRECT frame names one node and nobody else may repeat it.** That is
//: what makes it cheap, and it is also what makes it all-or-nothing: if the
//: named carrier does not hear it, the frame is gone, and no repeater on any
//: ridge can help. A flood is the opposite - every eligible node carries it,
//: which is exactly what a marginal link needs.
//:
//: So below this, the route is still remembered and still used for
//: forwarding, but it is not used to *choose* a carrier. Seen in the field
//: on 2026-09-30: the base held a one-hop route to a tracker at -16 dB, sent
//: everything down it, and recovered only on the retry that floods.
//:
//: -10 leaves about 7 dB of margin at SF11 and 10 at SF12. The cost of
//: being wrong in this direction is airtime; the cost of being wrong in the
//: other is a message that had no way through at all.
static const int8_t ROUTE_MIN_SNR_DB = -10;

class Routes {
public:
  void begin(uint8_t me);

  //: Called for every authenticated packet worth learning from. True if it
  //: changed anything. `snr` is how that packet sounded, in whole dB.
  bool learn(uint8_t src, uint8_t hop, uint8_t hops, uint32_t now,
             int8_t snr = 0);

  //: The next hop toward dst, or NO_ROUTE to flood.
  //:
  //: Used for FORWARDING, where the sender has already named this node as
  //: the carrier and the only question is who is next. It applies no
  //: quality floor: refusing here would drop a frame that cannot be
  //: promoted back to a flood, because `route` is inside the MAC.
  uint8_t via(uint8_t dst, uint32_t now) const;

  //: The next hop toward dst, but only if the route is worth committing a
  //: frame to. NO_ROUTE means flood. Used when CHOOSING how to send.
  //: See ROUTE_MIN_SNR_DB.
  uint8_t viaTrusted(uint8_t dst, uint32_t now) const;

  //: After a direct attempt fails: fall back to flooding next time.
  void forget(uint8_t dst);

  uint8_t hopsTo(uint8_t dst) const;
  //: How the packet that taught this route sounded, in whole dB. 0 when
  //: there is no route.
  int8_t snrTo(uint8_t dst) const;

  //: What a route costs, lower being better: hops priced in dB of margin,
  //: less the margin it actually has.
  //:
  //: **Hop count alone was the metric, and it is why a mounted repeater
  //: went unused.** A path is preferred for being short even when it is
  //: barely decodable, so a unit in a valley would send to the base over
  //: a one-hop path at -5 dB rather than a two-hop path over a repeater
  //: on the ridge at +8 dB. With CRC off - which is how this network
  //: buys two bytes - that marginal path does not drop frames, it
  //: delivers corrupt ones, and the authentication tag throws them away
  //: silently. Positions survived it because they are broadcast and flood
  //: both ways; addressed traffic named the bad path and took it alone.
  //:
  //: HOP_COST_DB is the price, and it is four - see the constant for why
  //: that number and not a larger one. An extra hop doubles the airtime
  //: and adds a node that can be busy, so it has to be worth real margin,
  //: but not so much that a solid relay never wins, which is the fault
  //: being fixed. At four, a one-hop link must be within 4 dB of a
  //: two-hop one to keep it.
  //:
  //: Named rather than numbered on purpose: this prose said "six" for as
  //: long as the constant said four.
  static int16_t cost(uint8_t hops, int8_t snr) {
    return (int16_t)(hops * HOP_COST_DB) - snr;
  }

private:
  struct Entry {
    uint8_t via; uint8_t hops; uint32_t at; bool used; int8_t snr;
  };
  Entry   _t[ADDR_COUNT];
  uint8_t _me = 0;
};

// ---- duplicate filter ---------------------------------------------------

//: What replaces MeshCore's accumulated route path. A flood does not need to
//: remember where it has been if every node remembers what it has heard.
class Seen {
public:
  void begin();
  //: True if this is new. Records it either way.
  bool check(uint8_t src, uint8_t seq, uint32_t now);

private:
  struct Entry { uint8_t src, seq; uint32_t at; bool used; };
  Entry _e[SLIM_SEEN_SIZE];
  uint8_t _next = 0;
};

// ---- the decision -------------------------------------------------------

//: Three independent outcomes, not one of three: a broadcast position is
//: normally both delivered to the application and repeated onward, and
//: conflating them loses one or the other.
struct Reception {
  Frame   fields;
  bool    deliver   = false;
  bool    duplicate = false;
  uint8_t onward[SLIM_MAX_FRAME];
  uint8_t onward_len = 0;          // 0 means "do not repeat"
  int8_t  err = OK;                // ERR_BAD or ERR_NEEDS_CIPHER
};

class Router {
public:
  //: Identical copies of one packet, sent back to back. In a mesh that
  //: deduplicates on (SRC, SEQ) a repeat is free of consequence — every
  //: receiver that already has it drops it before doing anything — so the only
  //: cost is airtime, and at 12 bytes that is the cheapest reliability there
  //: is. Three copies at SF11/250 kHz cost 743 ms against 690 ms for one
  //: MeshCore position: triple redundancy for eight per cent.
  //:
  //: Note what this is NOT: MeshCore's attempt byte, which makes each retry a
  //: distinct packet precisely so dedup will not eat it. That is the opposite
  //: trade — every copy then has to be handled, acked and possibly forwarded
  //: separately. Here the copies are byte-identical and the mesh collapses
  //: them; only the first to arrive intact does any work.
  static const uint8_t TX_BURST_MIN = 1;
  static const uint8_t TX_BURST_MAX = 4;

  void begin(uint8_t me, const uint8_t key[16], bool forwards);

  //: The flood policy, and the answer to the only question packet size cannot
  //: help with. Ten units that all repeat is N-squared transmissions and two
  //: thirds of the channel at SF12; the same ten with three designated
  //: repeaters is a quarter of it. Handhelds false, repeaters and base true.
  //:
  //: Path learning does NOT fix this. It narrows addressed traffic to one
  //: chain, but a position report is broadcast by design, so it floods to
  //: everyone eligible no matter how well routes are known.
  void setForwards(bool on)         { _forwards = on; }
  void setCipher(Cipher* c)         { _cipher = c; }
  //: Bit per type. Positions should normally stay clear: they are relayed to
  //: SARTrack over APRS on VHF in the open, so encrypting the LoRa leg protects
  //: nothing and burns keystream on the highest-volume packet in the network.
  void setEncTypes(uint8_t mask)    { _enc_types = mask; }
  bool setTxBurst(uint8_t n);
  uint8_t txBurst() const           { return _tx_burst; }

  //: A frame ready for the radio, DIRECT if we know a route to dst.
  //: Returns the length, or a negative Result.
  int send(uint8_t type, const uint8_t* body, uint8_t body_len, uint8_t dst,
           uint32_t now, uint8_t* out, size_t cap,
           bool prefer_direct = true, int ttl = -1);

  //: Authenticate, learn, and decide.
  //: `snr` is how this packet sounded, in whole dB. It is what ranks the
  //: route it teaches - see Routes::cost - so a caller that has the
  //: measurement must pass it; 0 is a neutral default for one that
  //: cannot.
  void onRecv(const uint8_t* frame, uint8_t len, uint32_t now,
              Reception& r, int8_t snr = 0);

  //: Turns a wire SEQ into the sender's wide counter, tracking wraps. Public
  //: only so the parse trampoline can reach it.
  uint32_t extFor(uint8_t src, uint8_t seq);

  //: What the next `send()` will stamp on its frame, without consuming it.
  //:
  //: Needed because an acknowledgement is matched on the sequence number, so
  //: the caller has to know it *before* the frame goes out - otherwise it
  //: cannot record what it is waiting for until after the reply could already
  //: have arrived.
  uint8_t peekNextSeq() const { return (uint8_t)((_seq_ext + 1) & 0xFF); }

  Routes   routes;
  Seen     seen;

private:
  uint8_t  _me = 0;
  uint8_t  _key[16] = {0};
  bool     _forwards = false;
  Cipher*  _cipher = nullptr;
  uint8_t  _enc_types = 0;
  uint8_t  _tx_burst = 1;
  uint32_t _seq_ext = 0;

  struct Peer { uint8_t last; uint32_t hi; bool used; };
  Peer _peer[ADDR_COUNT];
};

}  // namespace slim
