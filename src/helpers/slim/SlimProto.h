// MESH_SLIM inside MeshCore: the mode switch, the unit map, and the glue that
// lets a slim frame travel through MeshCore's Dispatcher.
//
// Build modes, set from platformio.ini:
//
//   (nothing)                        stock MeshCore, byte-identical to upstream.
//                                    Not one line of slim code is compiled in.
//   -D WITH_MESH_SLIM=1              both protocols, switched at runtime by the
//                                    `proto` CLI command and persisted.
//   -D WITH_MESH_SLIM=1              slim only. active() folds to a constant,
//   -D MESH_SLIM_ONLY=1              so the switch costs nothing.
//
// `both` is the interesting one, and it exists for one reason: the sync word
// makes a slim unit and a stock unit MUTUALLY DEAF. A node that can be told to
// change protocol over the air can be recovered over the air; a node that
// cannot has to be walked to. Measured cost of carrying both: about 3.6 KB of
// code and 1 KB of RAM, against ~800 KB free on an nRF52840.
//
// See MESH_SLIM/DESIGN.md section 11.

#pragma once

#include "SlimRouter.h"
// Defines MESH_SLIM_UNIT from ADVERT_NAME, so it must come before the
// #ifdef MESH_SLIM_UNIT that reads it.
#include "SlimUnit.h"

#ifdef WITH_MESH_SLIM

#include <Packet.h>
#include <Dispatcher.h>

namespace slimproto {

// ---- boot -----------------------------------------------------------------
//
// Identity comes from build flags rather than saved settings, on purpose for
// now: a field trial needs a radio that is correct the moment it powers on, and
// settings that live only in RAM would be lost by the first flat battery.
// Flashing each unit with its own number is how the briefing sheet already
// works - unit 7 is MTRK07 - so this matches the paper rather than fighting it.
//
//   -D MESH_SLIM_UNIT=7          this node's briefing number, 1..15.
//                                Normally NOT set by hand: SlimUnit.h
//                                derives it from ADVERT_NAME, so the
//                                callsign is the only knob. Setting it
//                                wins, for the Pi and for one-off builds.
//   -D MESH_SLIM_FORWARDS=1      repeat other people's floods (repeater/base)
//   -D MESH_SLIM_BURST=3         identical copies per transmission, 1..4
//   -D MESH_SLIM_KEY='"000102..."'  32 hex characters, the operation key
//   -D MESH_SLIM_DEFAULT=1       boot into slim rather than MeshCore
//
// The CLI can still change any of them at runtime for an experiment; the flags
// are what the node comes up as.
//
// Called from RadioLibWrapper::begin(), which every build runs at boot, so no
// example's main() has to know this exists.
void begin();

// ---- mode ---------------------------------------------------------------

#ifdef MESH_SLIM_ONLY
inline bool active() { return true; }
inline bool canSwitch() { return false; }
#else
bool active();
inline bool canSwitch() { return true; }
#endif

//: Changes the wire format AND the radio settings together, because they are
//: one decision: sync word, preamble, CRC and bandwidth all move at once. The
//: caller reboots the radio afterwards. Returns false if this build cannot
//: switch, or if it is already in the requested mode.
bool setActive(bool on);

//: The app registers this because sync word and preamble live on the concrete
//: RadioLib class, not on mesh::Radio, and reaching them from here would mean
//: widening a core abstraction for one feature. Called on every switch.
typedef void (*RadioConfigFn)(bool slim);
void setRadioConfigurator(RadioConfigFn fn);

//: Where a switch asked for on the command line (`proto slim|meshcore`) goes,
//: when the app registers one. The app saves the choice and restarts, so the
//: reply reaches whoever asked before the radio moves - an admin logged in over
//: the old protocol would otherwise never see it - and the choice survives a
//: power cut. Returns false if it could not be saved. With none registered the
//: switch is made live, as before.
typedef bool (*SwitchFn)(bool slim);
void setSwitchHandler(SwitchFn fn);

// ---- identity -----------------------------------------------------------

//: The one pre-shared 128-bit operation key, from the briefing QR code. Until
//: it is set the node is silent: setUnit() will not take effect and nothing
//: authenticates. That is the correct failure — a node with no key must not
//: transmit under a key of zeros, which every other unfinished node would also
//: accept.
bool setKey(const uint8_t key[16]);
bool haveKey();

//: This node's unit number, from the briefing sheet. Unit 7 is MTRK07 on the
//: voice net; it is not derived from a public key, so that the number on the
//: radio and the number people say out loud are the same number.
void    setUnit(uint8_t unit);
uint8_t unit();

//: Whether this node repeats other people's floods. Handhelds false, dedicated
//: repeaters and the base true. This is the flood policy, and it is the answer
//: to the one question packet size cannot help with — see SlimRouter.h.
void setForwards(bool on);
bool forwards();

//: 1..4 identical copies per transmission. At 12 bytes, three copies cost about
//: what one MeshCore position costs, so redundancy is nearly free.
bool    setTxBurst(uint8_t n);
uint8_t txBurst();

slim::Router& router();

// ---- the unit map -------------------------------------------------------
//
// The companion API addresses contacts by public key; the wire addresses them
// by a 4-bit unit number. Neither needs to change: the map is local, costs no
// airtime, and is filled at briefing rather than learned over the air.
//
// This is the one real behaviour change of slim mode. A MeshCore advert carries
// a signed 32-byte identity and cannot fit a 12-byte budget, so contacts are
// provisioned rather than discovered. For a fifteen-unit team that is arguably
// more correct than discovery, but it is a change and it is not an accident.

//: Returns false if the unit is out of range or the table is full.
bool mapUnit(uint8_t unit, const uint8_t* pub_key);

//: slim::NO_ROUTE-style sentinel: 0xFF means "not on the roster".
static const uint8_t NO_UNIT = 0xFF;

//: Matches on the first `len` bytes - the 6-byte prefix the companion protocol
//: uses for text. Only a short prefix is stored, so a longer `len` is clamped
//: to it rather than refused.
uint8_t unitForPubKey(const uint8_t* pub_key, uint8_t len);

//: The stored public-key prefix, or NULL if that unit was never provisioned.
//: At least 6 bytes, which is what the companion push frame needs.
const uint8_t* pubKeyForUnit(uint8_t unit);

void clearMap();
uint8_t mappedCount();

// ---- Packet as a container ----------------------------------------------
//
// In slim mode a mesh::Packet carries the whole slim frame in `payload`, with
// path_len 0 and a RAW_CUSTOM header. Nothing above the codec has to know: the
// Dispatcher's queueing, airtime budget, CAD and duty-cycle tracking all work
// unchanged, which is exactly the machinery that would be a mistake to rewrite.

void    frameToPacket(mesh::Packet* pkt, const uint8_t* frame, uint8_t len);
uint8_t packetToFrame(const mesh::Packet* pkt, uint8_t* out);

// ---- receive ------------------------------------------------------------

//: What the app implements to be told about traffic addressed to it.
//: What happened to the frames this node has heard.
//:
//: **Silence is the one thing a failing slim network cannot tell you.** A
//: frame that does not authenticate is dropped without a word - rightly, because
//: what a receiver says about a bad packet is a channel of its own - and a
//: frame that never arrives is dropped by physics. From the outside the two
//: are identical: nothing on the map, nothing in the log, nothing to do next.
//: A base on the wrong key and a base out of range look exactly alike.
//:
//: A COUNT is not an oracle. It says how many were rejected, never which
//: byte was wrong or how far the check got, and anyone who can hear the
//: channel can already count their own packets. What it buys is the only
//: question worth asking first: is anything getting here at all?
struct Counts {
  uint32_t seen;       //: frames off the air with the slim sync word
  uint32_t bad;        //: malformed, or failed the operation key
  uint32_t dup;        //: already heard, by (SRC, SEQ)
  uint32_t delivered;  //: handed up to the app
  uint32_t forwarded;  //: repeated onward
  uint32_t sent;       //: frames built and handed to the radio
  //: ...and frames the router REFUSED to build.
  //:
  //: Its own number because a refusal is a different fault from a
  //: loss and wants a different fix. A flood at the wrong ttl is
  //: rejected by Router::send and dropped without a word, which from
  //: outside looks exactly like a node that will not answer.
  uint32_t refused;
};
const Counts& counts();

//: What the radio measured about one reception.
//:
//: Beside the frame rather than inside it: a frame is what was said, this
//: is how well it arrived. Nothing here is on the wire, and a repeater that
//: forwards the frame does not forward these - they are this receiver's
//: view of this hop, which is the only thing they could honestly be.
struct Link {
  int8_t  snr_q4;   //: SNR in quarter-dB, as mesh::Packet carries it
  int16_t rssi;     //: dBm, or 0 when the build cannot measure it
};

class Handler {
public:
  virtual ~Handler() {}
  virtual void onSlimRecv(const slim::Frame& f, const Link& link) = 0;
};

void setHandler(Handler* h);

//: The hook Mesh::onRecvPacket calls when slim is active. Runs the routing
//: rules, delivers upward if the packet is ours, and rewrites the packet in
//: place for retransmission if we are the node that should carry it onward.
//:
//: `jitter_ms` is the hold-off before any retransmission - Mesh's own
//: getRetransmitDelay(), passed in rather than rolled again here.
mesh::DispatcherAction onRecvPacket(mesh::Packet* pkt, uint32_t now_ms,
                                    uint32_t jitter_ms);

// ---- send ---------------------------------------------------------------

//: Builds a frame into `pkt`. Returns the frame length or a negative
//: slim::Result. DIRECT if a route to `dst` is known, FLOOD otherwise.
//: **A range test and a health check are reachability probes, and a probe
//: must not take a stored route.** DIRECT names one carrier: if that node
//: cannot relay it the frame is dropped where it stands, silently, and the
//: screen reports "no reply" — which reads as "the base cannot hear me from
//: here" when what happened is that one chosen neighbour did not pass it
//: on. That is the worst possible answer from an instrument somebody is
//: using to decide where to put a repeater.
//:
//: Flooding costs one relay per eligible node on a 9-byte frame that a
//: person presses a button for. The Pi's own request_status has done this
//: from the beginning, and says the same thing in the same words.
int buildOutbound(mesh::Packet* pkt, uint8_t type, const uint8_t* body,
                  uint8_t body_len, uint8_t dst, uint32_t now_ms,
                  int ttl = -1, bool prefer_direct = true);

// ---- CLI ----------------------------------------------------------------

//: Handles everything after the `proto` keyword; `reply` is the 160-byte CLI
//: buffer. Kept here rather than in CommonCLI so that the core file gains four
//: lines instead of eighty, and so a rebase onto upstream has almost nothing
//: to reconcile.
//:
//:   proto                     report mode, unit, roster size, burst
//:   proto slim | meshcore     switch wire format and radio settings
//:   proto unit <0-15>         this node's briefing number
//:   proto forwards on|off     whether this node repeats other people's floods
//:   proto burst <1-4>         identical copies per transmission
//:   proto routes              the learned route table
//:
//: `now_ms` is needed because routes expire: printing the table without it
//: would either show stale entries as live or hide live ones as stale.
void cli(const char* args, char* reply, uint32_t now_ms);

}  // namespace slimproto

#endif  // WITH_MESH_SLIM
