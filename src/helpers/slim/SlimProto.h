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
  //: ...and MeshCore packets kept off the air because slim is active: an
  //: advert, an ack, a path return, anything MeshCore built. Each one is a
  //: frame no slim receiver can read, as much airtime as a real one, and a
  //: collision with whatever a unit was saying - so the dispatcher refuses
  //: them all, at the one place every transmission passes, and counts them
  //: here. Anything above zero is a caller that should have spoken slim.
  uint32_t blocked;
};
const Counts& counts();
//: Counted by the dispatcher when it refuses a MeshCore packet. See blocked.
void noteBlocked();

//: What the radio measured about one reception.
//:
//: Beside the frame rather than inside it: a frame is what was said, this
//: is how well it arrived. Nothing here is on the wire, and a repeater that
//: forwards the frame does not forward these - they are this receiver's
//: view of this hop, which is the only thing they could honestly be.
struct Link {
  int8_t  snr_q4;   //: SNR in quarter-dB, as mesh::Packet carries it
  int16_t rssi;     //: dBm, or 0 when the build cannot measure it
  uint8_t len;      //: bytes on the air - what an answer has to wait out
};

// ---- when to answer -----------------------------------------------------
//
// **An answer sent while the asker is still transmitting is never heard.**
// Every sender puts txBurst() identical copies on the air back to back, and
// what is decoded is normally copy one - so the rest of the burst is still
// going out, and a half-duplex radio hears nothing while its own
// transmitter is on.
//
// It used to be a flat 2.5 s, chosen at SF11/250k where a whole burst took
// about that long. At SF12/125k a 13-byte position is 1.5 s a copy, so the
// base answered into copy three of every report - its own log said "channel
// still busy, transmitting anyway" each time - and MRPT01 answered the Pi's
// ping while the Pi was still sending it, so not one reading arrived
// (2026-10-23). The time is the asker's airtime, so it is worked out from it.
//
// The same numbers are slim_node.py's REPLY_QUIET_S, REPLY_COPY_GAP_S and
// REPLY_MARGIN_S: one fact about the radio, three implementations, and
// test_reply_timing.py pins them equal.
//
// **Since every frame goes once** (2026-10-23) a decoded frame's sender has
// stopped and is listening, so an answer waits only the turnaround: half a
// second. The 2.5 s floor kept from the three-copy days was most of every
// acknowledgement a person was watching for (lowered 2026-10-29).
static const uint32_t REPLY_QUIET_MIN_MS = 500;   //: never less than this
static const uint32_t REPLY_COPY_GAP_MS  = 250;   //: between two copies
static const uint32_t REPLY_MARGIN_MS    = 500;   //: turnaround, clock skew
//: ...but what an ASKER allows the far end is the slowest answer still in
//: the field. Units flashed before 2026-10-29 - the repeaters on the hills,
//: until their next flash - hold theirs 2.5 s, and a wait cut to this
//: unit's own half second retries an answer that is on its way: the same
//: words twice. Lower it once every unit has been flashed.
static const uint32_t PEER_QUIET_MAX_MS  = 2500;
static const uint32_t ACK_WAIT_MIN_MS    = 8000;
//: The longest a repeater holds a frame before relaying it, in half-airtimes
//: of that frame: MeshCore's getRetransmitDelay, rand(0..5) x airtime x
//: tx_delay_factor, which is 0.5 on a repeater. Two and a half airtimes.
static const uint32_t RELAY_HOLD_AIRTIMES_X2 = 5;

//: How long to hold an answer to a frame that took `airtime_ms` on the air:
//: the rest of its burst, then a margin.
uint32_t replyQuietMs(uint32_t airtime_ms);
//: How long the slowest unit in the field holds ITS answer to such a frame:
//: what a wait for somebody else's answer allows. See PEER_QUIET_MAX_MS.
uint32_t peerQuietMs(uint32_t airtime_ms);
//: How long a sender waits for the acknowledgement of a frame that takes
//: `airtime_ms` before trying again. A retry goes under a new sequence
//: number, so one sent before the first attempt's ack could arrive is the
//: same message shown twice: the far end's whole exchange, plus a repeater
//: holding and relaying it each way, and never less than ACK_WAIT_MIN_MS.
//: "Twice the exchange" was too short through MRPT01 at SF12: a canned
//: reply's ack took 13-16 s to come back, and every one was sent again
//: (2026-10-23).
uint32_t ackWaitMs(uint32_t airtime_ms);
//: How long until a frame this long has been answered directly OR carried on
//: by a repeater, whichever is later - so a sender that has heard NEITHER by
//: then knows it did not get out, and can try again rather than wait out
//: ackWaitMs, which allows for the answer coming back through a repeater.
uint32_t fastWaitMs(uint32_t airtime_ms);
//: How long after a frame has been heard its relays can still be on the air:
//: every repeater a flood can cross (TTL_INIT - 1) in turn, each after its
//: longest hold. Repeaters in a chain relay each other's copy, so the holds
//: add up: MRPT02's copy of a report came a median 8 s and at worst 15 s
//: after the tracker's own (2026-11-05). slim_node._relay_span_s.
uint32_t relaySpanMs(uint32_t airtime_ms);
//: A frame and every relay of it: what one transmission costs the channel.
uint32_t floodMs(uint32_t airtime_ms);

class Handler {
public:
  virtual ~Handler() {}
  virtual void onSlimRecv(const slim::Frame& f, const Link& link) = 0;
  //: One of this node's own frames, carried back by somebody else: it got
  //: out, and whatever answers it has further to come. Nothing by default -
  //: a node that never waits on an answer has no use for it.
  virtual void onSlimEcho(const slim::Frame& f) { (void)f; }
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
