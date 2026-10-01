#include "SlimRouter.h"

#include <string.h>

namespace slim {

// Unsigned subtraction, so this stays correct across the 49-day millis()
// rollover. `now - at` wraps to the right small number; `now > at + window`
// would not.
static inline bool olderThan(uint32_t now, uint32_t at, uint32_t window) {
  return (uint32_t)(now - at) > window;
}

// ---- Routes -------------------------------------------------------------

void Routes::begin(uint8_t me) {
  _me = me;
  memset(_t, 0, sizeof(_t));
}

bool Routes::learn(uint8_t src, uint8_t hop, uint8_t hops, uint32_t now,
                   int8_t snr) {
  if (src > MAX_ADDR) return false;
  if (src == _me || hop == _me) {
    // Our own packet, come back to us around a loop. Storing a route to
    // ourselves would send everything we transmit to a neighbour to be
    // relayed back.
    return false;
  }
  if (src == hop && hops > 1) {
    // HOP still names the source after several hops, so either a repeater
    // failed to rewrite it or someone is lying. A direct neighbour has
    // hop == src at exactly one hop, which is the normal case and not this one.
    return false;
  }
  Entry& e = _t[src];
  const bool alive = e.used && !olderThan(now, e.at, ROUTE_TTL_MS);
  if (alive && e.via == hop && e.hops == hops) {
    // The route we are already using, heard again. Keep the freshest
    // measurement of it: a path that is fading should be seen to fade,
    // or it is defended by the good reading it was learned on.
    e.at = now;
    e.snr = snr;
    return false;
  }
  if (alive && cost(hops, snr) >= cost(e.hops, e.snr) - ROUTE_HYST_DB) {
    // Not enough better to be worth changing to. Not touched at all - its
    // age is the age of the last packet that actually came that way.
    return false;
  }
  e.via = hop;
  e.hops = hops;
  e.at = now;
  e.snr = snr;
  e.used = true;
  return true;
}

int8_t Routes::snrTo(uint8_t dst) const {
  if (dst > MAX_ADDR || !_t[dst].used) return 0;
  return _t[dst].snr;
}

uint8_t Routes::via(uint8_t dst, uint32_t now) const {
  if (dst > MAX_ADDR) return NO_ROUTE;
  const Entry& e = _t[dst];
  if (!e.used || olderThan(now, e.at, ROUTE_TTL_MS)) return NO_ROUTE;
  return e.via;
}

uint8_t Routes::viaTrusted(uint8_t dst, uint32_t now) const {
  const uint8_t v = via(dst, now);
  if (v == NO_ROUTE) return NO_ROUTE;
  return _t[dst].snr < ROUTE_MIN_SNR_DB ? NO_ROUTE : v;
}

void Routes::forget(uint8_t dst) {
  if (dst <= MAX_ADDR) _t[dst].used = false;
}

uint8_t Routes::hopsTo(uint8_t dst) const {
  if (dst > MAX_ADDR || !_t[dst].used) return 0;
  return _t[dst].hops;
}

// ---- Seen ---------------------------------------------------------------

void Seen::begin() {
  memset(_e, 0, sizeof(_e));
  _next = 0;
}

bool Seen::check(uint8_t src, uint8_t seq, uint32_t now) {
  for (uint8_t i = 0; i < SLIM_SEEN_SIZE; i++) {
    Entry& e = _e[i];
    if (!e.used) continue;
    if (olderThan(now, e.at, DEDUP_WINDOW_MS)) { e.used = false; continue; }
    if (e.src == src && e.seq == seq) { e.at = now; return false; }
  }
  // A ring rather than a search for a free slot: when it is full the oldest
  // entry is overwritten, which is the right thing to lose.
  Entry& slot = _e[_next];
  _next = (uint8_t)((_next + 1) % SLIM_SEEN_SIZE);
  slot.src = src; slot.seq = seq; slot.at = now; slot.used = true;
  return true;
}

// ---- Router -------------------------------------------------------------

void Router::begin(uint8_t me, const uint8_t key[16], bool forwards) {
  _me = me;
  memcpy(_key, key, 16);
  _forwards = forwards;
  _seq_ext = 0;
  memset(_peer, 0, sizeof(_peer));
  routes.begin(me);
  seen.begin();
}

bool Router::setTxBurst(uint8_t n) {
  if (n < TX_BURST_MIN || n > TX_BURST_MAX) return false;
  _tx_burst = n;
  return true;
}

uint32_t Router::extFor(uint8_t src, uint8_t seq) {
  if (src > MAX_ADDR) return seq;
  Peer& p = _peer[src];
  // A jump backwards of more than half the space is a wrap rather than a
  // reorder — the same rule IPsec uses. Getting this wrong does not break
  // parsing; it silently repeats a keystream, which is why it lives here and
  // not in each caller.
  if (p.used && seq < p.last && (uint8_t)(p.last - seq) > 128) {
    p.hi = (p.hi + 1) & 0xFFFFFF;
  }
  p.used = true;
  p.last = seq;
  return (p.hi << 8) | seq;
}

static uint32_t seqExtTrampoline(void* ctx, uint8_t src, uint8_t seq) {
  return static_cast<Router*>(ctx)->extFor(src, seq);
}

int Router::send(uint8_t type, const uint8_t* body, uint8_t body_len,
                 uint8_t dst, uint32_t now, uint8_t* out, size_t cap,
                 bool prefer_direct, int ttl) {
  const bool broadcast = isBroadcastType(type);

  Build b;
  b.type = type;
  b.src = _me;
  b.hop = _me;
  b.dst = broadcast ? ADDR_BROADCAST : dst;
  b.route = ROUTE_FLOOD;
  b.body = body;
  b.body_len = body_len;

  if (!broadcast && prefer_direct) {
    // Trusted, not merely known: a route too weak to commit a frame to is
    // left to the flood, which every eligible node carries.
    const uint8_t v = routes.viaTrusted(dst, now);
    if (v != NO_ROUTE) {
      b.route = ROUTE_DIRECT;
      b.hop = v;
    }
  }

  const uint8_t use_ttl = (ttl < 0) ? TTL_INIT : (uint8_t)ttl;
  if (b.route == ROUTE_FLOOD && teachesRoutes(type) && use_ttl != TTL_INIT) {
    // Not pedantry. Receivers derive hop count from TTL_INIT - ttl + 1, so a
    // flood launched at any other ttl teaches every node that hears it a wrong
    // distance, and wrong distances pick wrong routes for the fifteen minutes
    // until they expire.
    return ERR_ARG;
  }
  b.ttl = use_ttl;
  b.ttl_set = true;

  _seq_ext = (_seq_ext + 1) & 0xFFFFFFFFUL;
  b.seq = (uint8_t)(_seq_ext & 0xFF);
  b.seq_ext = _seq_ext;
  b.enc = (_enc_types >> (type & 7)) & 1;

  const int n = build(_key, b, _cipher, out, cap);
  if (n < 0) return n;

  // Our own packet, in our own dedup table. Without this it comes back from a
  // neighbour's repeat, looks new, and we learn a route to ourselves through
  // whoever handed it back.
  seen.check(_me, b.seq, now);
  return n;
}

void Router::onRecv(const uint8_t* frame, uint8_t len, uint32_t now,
                    Reception& r, int8_t snr) {
  r.deliver = false;
  r.duplicate = false;
  r.onward_len = 0;

  r.err = (int8_t)parse(_key, frame, len, r.fields, _cipher,
                        seqExtTrampoline, this);
  if (r.err != OK) return;

  const Frame& f = r.fields;
  const bool dup = !seen.check(f.src, f.seq, now);

  if (!f.isDirect() && teachesRoutes(f.type)) {
    // Only floods carry a trustworthy distance, and only floods reach us by a
    // path worth learning. A DIRECT packet's HOP names the next carrier rather
    // than the last one, so reading a route out of it would point us at the
    // node we were meant to send to.
    //
    // **Before the duplicate check, and that is the whole point.** One flood
    // arrives twice - once the short way and once relayed - and the two
    // copies are the two routes home. The short way arrives first, because it
    // is short, so filing the second as a duplicate and returning meant the
    // only route ever learned was the first one to turn up. On a valley floor
    // with no line of sight to the base that first copy is a reflection off a
    // mountainside: it decodes, badly, and it won every time while the clear
    // path over the repeater was thrown away unread.
    //
    // A duplicate is authenticated - it passed the tag - so it is a genuine
    // measurement of a genuine path. Learning from it costs nothing and is
    // the only way the better path is ever seen.
    routes.learn(f.src, f.hop, f.hops, now, snr);
  }

  if (dup) {
    r.duplicate = true;
    return;
  }

  r.deliver = (f.dst == ADDR_BROADCAST) || (f.dst == _me);

  if (f.dst != _me && f.ttl > 0) {
    if (f.isDirect()) {
      if (f.hop == _me) {
        const uint8_t v = routes.via(f.dst, now);
        if (v != NO_ROUTE) {
          memcpy(r.onward, frame, len);
          r.onward_len = forward(r.onward, len, v);
        }
        // No route and we were the chosen carrier: drop it. We must NOT promote
        // it to a flood — `route` is inside the MAC, so the bit cannot be
        // flipped without breaking the tag, and that is deliberate. The
        // sender's ack times out, it forgets the route, and its next attempt
        // floods. Recovery belongs to the endpoint that can tell whether the
        // message arrived, not to a relay guessing.
      }
    } else if (_forwards) {
      memcpy(r.onward, frame, len);
      r.onward_len = forward(r.onward, len, _me);
    }
  }
}

}  // namespace slim
