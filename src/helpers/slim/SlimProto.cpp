#include "SlimProto.h"

#ifdef WITH_MESH_SLIM

#include <math.h>
#include <string.h>
#include <stdlib.h>

namespace slimproto {

#ifndef MESH_SLIM_ONLY
static bool _active =
  #ifdef MESH_SLIM_DEFAULT
    MESH_SLIM_DEFAULT != 0;
  #else
    false;      // a dual build boots on the protocol the rest of the net speaks
  #endif
bool active() { return _active; }
#endif

static RadioConfigFn _radio_cfg = nullptr;
static Counts _counts = {0, 0, 0, 0, 0, 0, 0};
static SwitchFn      _switch_fn = nullptr;
static Handler*      _handler   = nullptr;
static slim::Router  _router;
static uint8_t       _unit      = 0;
static bool          _forwards  = false;
static bool          _begun     = false;

//: The one pre-shared operation key, distributed at the briefing. Until it is
//: set nothing can be sent or authenticated, which is the correct failure: a
//: node with no key must be silent, not transmit under a key of zeros that
//: every other unfinished node would also accept.
static uint8_t _key[16];
static bool    _have_key = false;

bool setKey(const uint8_t key[16]) {
  if (key == nullptr) return false;
  memcpy(_key, key, 16);
  _have_key = true;
  if (_begun) _router.begin(_unit, _key, _forwards);
  return true;
}

bool haveKey() { return _have_key; }

// ---- boot ---------------------------------------------------------------

#ifdef MESH_SLIM_KEY
static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
#endif

void begin() {
  static bool done = false;
  if (done) return;
  done = true;

#ifdef MESH_SLIM_KEY
  {
    const char* h = MESH_SLIM_KEY;
    uint8_t k[16];
    bool good = (strlen(h) == 32);
    for (int i = 0; good && i < 16; i++) {
      const int hi = hexNibble(h[i * 2]), lo = hexNibble(h[i * 2 + 1]);
      if (hi < 0 || lo < 0) { good = false; break; }
      k[i] = (uint8_t)((hi << 4) | lo);
    }
    // A malformed key is left unset rather than half-applied. The node then
    // refuses to switch to slim and says why, which is a great deal easier to
    // diagnose in the field than a radio that transmits and is never heard.
    if (good) setKey(k);
  }
#endif

#ifdef MESH_SLIM_FORWARDS
  setForwards(MESH_SLIM_FORWARDS != 0);
#endif
#ifdef MESH_SLIM_BURST
  setTxBurst(MESH_SLIM_BURST);
#endif
#ifdef MESH_SLIM_UNIT
  setUnit(MESH_SLIM_UNIT);      // last: it needs the key to have landed first
#endif
}

void setRadioConfigurator(RadioConfigFn fn) { _radio_cfg = fn; }
const Counts& counts() { return _counts; }
void setSwitchHandler(SwitchFn fn) { _switch_fn = fn; }
void setHandler(Handler* h) { _handler = h; }
slim::Router& router() { return _router; }

bool setActive(bool on) {
#ifdef MESH_SLIM_ONLY
  (void)on;
  return false;
#else
  if (on == _active) return false;
  _active = on;
  // Sync word, preamble, CRC and bandwidth are one decision, not four, so they
  // move together or the radio ends up in a state that is neither protocol.
  if (_radio_cfg) _radio_cfg(on);
  return true;
#endif
}

void setUnit(uint8_t u) {
  if (u > slim::MAX_ADDR || !_have_key) return;
  _unit = u;
  // begin() clears the route and dedup tables, which is right: routes learned
  // under one identity are meaningless under another, and keeping them would
  // leave the node forwarding to neighbours that no longer think it exists.
  _router.begin(u, _key, _forwards);
  _begun = true;
}
uint8_t unit() { return _unit; }

void setForwards(bool on) { _forwards = on; _router.setForwards(on); }
bool forwards() { return _forwards; }

bool    setTxBurst(uint8_t n) { return _router.setTxBurst(n); }
uint8_t txBurst() { return _router.txBurst(); }

// ---- the unit map -------------------------------------------------------

//: Only the prefix is stored. Every read is 6 bytes - the companion protocol
//: addresses text on the first 6 of the public key, and the push frame back to
//: the app carries the same 6 - so holding all 32 would be 384 bytes of RAM
//: that nothing ever looks at. Eight leaves headroom without pretending to be
//: a key store.
static const uint8_t MAP_KEY_BYTES = 8;

struct MapEntry { uint8_t key[MAP_KEY_BYTES]; bool used; };
static MapEntry _map[slim::ADDR_COUNT];

bool mapUnit(uint8_t u, const uint8_t* pub_key) {
  if (u > slim::MAX_ADDR || pub_key == nullptr) return false;
  memcpy(_map[u].key, pub_key, MAP_KEY_BYTES);
  _map[u].used = true;
  return true;
}

uint8_t unitForPubKey(const uint8_t* pub_key, uint8_t len) {
  if (pub_key == nullptr || len == 0) return NO_UNIT;
  // Clamped, not refused. A caller with a full 32-byte key should get an
  // answer from the bytes we kept, rather than a silent "not on the roster"
  // that reads as a provisioning fault.
  if (len > MAP_KEY_BYTES) len = MAP_KEY_BYTES;
  for (uint8_t u = 0; u < slim::ADDR_COUNT; u++) {
    if (_map[u].used && memcmp(_map[u].key, pub_key, len) == 0) return u;
  }
  return NO_UNIT;
}

const uint8_t* pubKeyForUnit(uint8_t u) {
  if (u > slim::MAX_ADDR || !_map[u].used) return nullptr;
  return _map[u].key;
}

void clearMap() { memset(_map, 0, sizeof(_map)); }

uint8_t mappedCount() {
  uint8_t n = 0;
  for (uint8_t u = 0; u < slim::ADDR_COUNT; u++) if (_map[u].used) n++;
  return n;
}

// ---- Packet as a container ----------------------------------------------

void frameToPacket(mesh::Packet* pkt, const uint8_t* frame, uint8_t len) {
  // RAW_CUSTOM is MeshCore's own escape hatch: "custom packet as raw bytes, for
  // applications with custom encryption, payloads, etc". Using it rather than
  // inventing a type means every debug print and length check upstream keeps
  // working instead of hitting an unknown type.
  pkt->header = (PAYLOAD_TYPE_RAW_CUSTOM << PH_TYPE_SHIFT) | ROUTE_TYPE_FLOOD;
  pkt->path_len = 0;
  pkt->transport_codes[0] = pkt->transport_codes[1] = 0;
  pkt->payload_len = len;
  memcpy(pkt->payload, frame, len);
}

uint8_t packetToFrame(const mesh::Packet* pkt, uint8_t* out) {
  const uint8_t len = (uint8_t)pkt->payload_len;
  memcpy(out, pkt->payload, len);
  return len;
}

// ---- receive ------------------------------------------------------------

mesh::DispatcherAction onRecvPacket(mesh::Packet* pkt, uint32_t now_ms,
                                    uint32_t jitter_ms) {
  if (!_begun) return ACTION_RELEASE;

  uint8_t frame[SLIM_MAX_FRAME];
  const uint8_t len = packetToFrame(pkt, frame);

  slim::Reception r;
  // The measurement goes in with the packet: it is what ranks the route
  // this packet teaches. See slim::Routes::cost - a marginal one-hop path
  // used to beat a solid two-hop one through a repeater on a ridge, and
  // addressed traffic then named the bad path and took it alone.
  _router.onRecv(frame, len, now_ms, r, (int8_t)lroundf(pkt->getSNR()));
  _counts.seen++;

  // Fail closed and silently. A packet that does not authenticate gets no
  // reply and no log of its contents: what a receiver says about a bad packet
  // is a channel of its own.
  if (r.err != slim::OK) { _counts.bad++; return ACTION_RELEASE; }
  if (r.duplicate) { _counts.dup++; return ACTION_RELEASE; }

  if (r.deliver) _counts.delivered++;
  if (r.onward_len) _counts.forwarded++;
  if (r.deliver && _handler) {
    Link link;
    link.snr_q4 = (int8_t)(pkt->getSNR() * 4.0f);
#ifdef WITH_MESH_SLIM
    link.rssi = pkt->_rssi;
#else
    link.rssi = 0;
#endif
    _handler->onSlimRecv(r.fields, link);
  }

  if (r.onward_len) {
    // Rewrite in place and let MeshCore's Dispatcher do the queueing, the CAD
    // check and the airtime accounting. That machinery is the reason to keep
    // the Packet as the carrier rather than bypassing it.
    memcpy(pkt->payload, r.onward, r.onward_len);
    pkt->payload_len = r.onward_len;

    // Every node that heard this is about to repeat it, and they all heard it
    // at the same instant. Without a randomised wait they transmit on top of
    // each other; `jitter_ms` is MeshCore's own hold-off, passed in.
    //
    // Priority falls off with distance, so a packet from a near neighbour is
    // sent before one that has already crossed the valley. Matches what
    // MeshCore does with path length.
    uint8_t pri = r.fields.hops;
    if (pri > 7) pri = 7;
    return ACTION_RETRANSMIT_DELAYED(pri, jitter_ms);
  }
  return ACTION_RELEASE;
}

// ---- send ---------------------------------------------------------------

int buildOutbound(mesh::Packet* pkt, uint8_t type, const uint8_t* body,
                  uint8_t body_len, uint8_t dst, uint32_t now_ms, int ttl,
                  bool prefer_direct) {
  if (!_begun) return slim::ERR_ARG;
  uint8_t frame[SLIM_MAX_FRAME];
  const int n = _router.send(type, body, body_len, dst, now_ms, frame,
                             sizeof(frame), prefer_direct, ttl);
  // Counted here because this is the last point every sender passes
  // through, and the one place a frame disappears without a word.
  if (n < 0) { _counts.refused++; return n; }
  _counts.sent++;
  frameToPacket(pkt, frame, (uint8_t)n);
  return n;
}

// ---- CLI ----------------------------------------------------------------

#include <stdio.h>

static const char* modeName(bool on) { return on ? "slim" : "meshcore"; }

void cli(const char* args, char* reply, uint32_t now_ms) {
  while (*args == ' ') args++;

  if (*args == 0) {
    // tx=sent/refused rx=seen/bad/dup/delivered. The refusal count is
    // the one worth having: it separates "this node will not answer"
    // from "its answer did not arrive", which look identical from the
    // other end of a valley and need opposite fixes.
    sprintf(reply, "proto=%s unit=%u fwd=%s burst=%u roster=%u key=%s%s"
                   " tx=%lu/%lu rx=%lu/%lu/%lu/%lu",
            modeName(active()), (unsigned)_unit, _forwards ? "on" : "off",
            (unsigned)txBurst(), (unsigned)mappedCount(),
            _have_key ? "set" : "MISSING",
            canSwitch() ? "" : " (slim-only build)",
            (unsigned long)_counts.sent, (unsigned long)_counts.refused,
            (unsigned long)_counts.seen, (unsigned long)_counts.bad,
            (unsigned long)_counts.dup, (unsigned long)_counts.delivered);
    return;
  }

  if (strcmp(args, "slim") == 0 || strcmp(args, "meshcore") == 0) {
    const bool want = (args[0] == 's');
    if (!canSwitch()) {
      sprintf(reply, "this build is %s only", modeName(active()));
    } else if (want && !_have_key) {
      // Refused rather than allowed-and-broken. Switching to slim without a key
      // would leave the node on a sync word nobody else on the old network can
      // hear, unable to authenticate anything, and unable to be told so.
      sprintf(reply, "refused: no operation key set");
    } else if (want == active()) {
      sprintf(reply, "already %s", modeName(want));
    } else if (_switch_fn) {
      if (_switch_fn(want)) {
        sprintf(reply, "proto %s saved - restarting in a few seconds",
                modeName(want));
      } else {
        sprintf(reply, "refused: could not save the choice");
      }
    } else if (setActive(want)) {
      sprintf(reply, "proto now %s - radio reconfigured, reboot to be sure",
              modeName(want));
    } else {
      sprintf(reply, "already %s", modeName(want));
    }
    return;
  }

  if (memcmp(args, "unit ", 5) == 0) {
    const int u = atoi(&args[5]);
    if (u < 0 || u > slim::MAX_ADDR) {
      sprintf(reply, "unit must be 0..%u", (unsigned)slim::MAX_ADDR);
    } else if (!_have_key) {
      sprintf(reply, "set the operation key first");
    } else {
      setUnit((uint8_t)u);
      sprintf(reply, "unit now %d (routes and dedup cleared)", u);
    }
    return;
  }

  if (memcmp(args, "forwards ", 9) == 0) {
    const bool on = (memcmp(&args[9], "on", 2) == 0);
    setForwards(on);
    sprintf(reply, "forwards %s - %s", on ? "on" : "off",
            on ? "this node repeats floods (repeater/base)"
               : "this node stays quiet (handheld)");
    return;
  }

  if (memcmp(args, "burst ", 6) == 0) {
    const int n = atoi(&args[6]);
    if (setTxBurst((uint8_t)n)) {
      sprintf(reply, "burst now %d", n);
    } else {
      sprintf(reply, "burst must be %u..%u",
              (unsigned)slim::Router::TX_BURST_MIN,
              (unsigned)slim::Router::TX_BURST_MAX);
    }
    return;
  }

  if (strcmp(args, "roster") == 0) {
    int n = sprintf(reply, "roster:");
    for (uint8_t u = 0; u <= slim::MAX_ADDR && n < 140; u++) {
      const uint8_t* k = pubKeyForUnit(u);
      if (k == nullptr) continue;
      n += sprintf(&reply[n], " %u=%02X%02X", (unsigned)u, k[0], k[1]);
    }
    if (n == 7) {
      strcpy(reply, "roster: empty - contacts need scheme names (MTRK01)");
    }
    return;
  }

  if (strcmp(args, "routes") == 0) {
    // 160 bytes of reply, fifteen possible units: print only what is known and
    // stop before the buffer does.
    int n = sprintf(reply, "routes:");
    for (uint8_t d = 0; d <= slim::MAX_ADDR && n < 140; d++) {
      const uint8_t h = _router.routes.hopsTo(d);
      if (h == 0) continue;
      n += sprintf(&reply[n], " %u>%u/%uh", (unsigned)d,
                   (unsigned)_router.routes.via(d, now_ms), (unsigned)h);
    }
    if (n == 7) strcpy(reply, "routes: none learned yet");
    return;
  }

  strcpy(reply,
         "?: proto [slim|meshcore|unit N|forwards on/off|burst N|routes|roster]");
}

}  // namespace slimproto

#endif  // WITH_MESH_SLIM
