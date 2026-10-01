// Tracker mode: a companion that reports its own position and sleeps between.
//
// For a unit with a GPS and nobody's phone attached - carried in a pack, on a
// dog's collar, in a child's pocket. It wakes on a timer, powers the GPS just
// long enough for a fix, sends one position, listens for a few seconds, and
// puts the radio and the GPS back to sleep:
//
//            fresh fix                 quiet for LISTEN
//     FIX  ------------->  LISTEN  ---------------------->  SLEEP
//      ^ \                                                    |
//      |  \-- no fix by the timeout: nothing sent ----------->|
//      +---------- `interval` after this cycle began ---------+
//
// The LISTEN window is the whole point of listening at all. For most of every
// cycle the unit is deaf, so anything sent to it then is lost. The base knows
// that: it sends a waiting command (see tracker_mode.py on the Pi) the moment
// it hears a report, which is exactly when the unit is listening.
//
// Commands, from the base over the air or from a phone as a custom variable:
//
//     !T300   report every 300 s, sleeping in between
//     !T0     leave tracker mode - back to an ordinary node
//
// ---- What it does when the base cannot hear it -------------------------
//
// On MeshCore, with the base in its contacts, each report is a direct message
// to the base, so MeshCore's own acknowledgement says whether it arrived. One
// that is never acknowledged goes into a backlog; once a report gets through
// again, the backlog follows in "!H" messages - history.py's format, which the
// base already records - one at a time, each dropped only when its own ack
// comes back. The walk behind the ridge ends up on the map after all.
//
// In slim mode a report is a broadcast, and the base - and only the base, so
// there is no ack storm - acknowledges it. So the same backlog works there,
// and the gap goes back as "!H" app datagrams (SlimApp.h), because "!H" is
// base64url and the uppercase-only text tier would have corrupted it.
//
// With no base contact at all a report is an unacknowledged broadcast, and
// nothing can know it was missed.
//
// ---- looking for a way through -----------------------------------------
//
// Two reports in a row with no answer and the unit decides it has lost the
// base (Reports::searching). It then stops sleeping the radio between cycles
// and has the chip sniff for a preamble instead, often enough that none can
// pass between two looks - about an eighth of the current of listening
// properly, with the processor asleep throughout.
//
// It is not waiting for a message. It is waiting for evidence that anything
// is in range: the base's team map, another unit's position being repeated, a
// helicopter overhead. Any of them means there is a way through, and the unit
// reports at once rather than waiting out the rest of its interval - because
// the way through may be a helicopter, and it may not be here in five
// minutes.
//
// This file is only the logic: the schedule, and what the base has and has
// not been given. No radio, no GPS and no clock of its own - the caller passes
// the time in - so it is tested on a PC (MESH_SLIM/cpp/test_tracker.cpp, which
// also walks a unit out of range and back) and the glue in MyMesh stays a few
// lines that match the hardware to it.

#pragma once

#ifdef WITH_TRACKER_MODE

#include <math.h>
#include <stdint.h>
#include <helpers/slim/SlimRound.h>

//: How long the radio stays listening after the last transmission of a
//: report. Long enough for the report to reach the base through a repeater and
//: a command to come back the same way; each second of it costs about 5 mA.
#ifndef TRACKER_LISTEN_S
  #define TRACKER_LISTEN_S 10
#endif
//: Give up on a fix after this long and send nothing. A hot start takes a few
//: seconds; a cold one, after the GPS has lost its almanac, can take a minute.
#ifndef TRACKER_FIX_TIMEOUT_S
  #define TRACKER_FIX_TIMEOUT_S 90
#endif
//: Reports in a row with no acknowledgement before a unit decides it has
//: lost the base and starts listening for one.
//:
//: Two, not one. A single lost packet is ordinary and recovers by itself;
//: two in a row is the link. The same judgement the phone app makes before
//: it says anything out loud, for the same reason.
#ifndef TRACKER_SEARCH_AFTER
  #define TRACKER_SEARCH_AFTER 2
#endif

//: How long before its slot a unit starts the GPS, so the moment it owns
//: is spent transmitting rather than waiting for a fix. A warm start is a
//: few seconds and a cold one can be a minute; this is capped at a third
//: of the round by the caller, so a short round does not leave the GPS on
//: continuously.
#ifndef TRACKER_FIX_LEAD_S
  #define TRACKER_FIX_LEAD_S 45
#endif

//: **A cold start cannot be done in instalments.**
//:
//: The GPS is normally powered only for the FIX phase, which is right
//: once a unit has a position: waking from standby it re-uses the
//: ephemeris it already holds and is ready in seconds. A unit that has
//: NEVER fixed holds nothing, and a first fix needs an uninterrupted
//: run - each satellite broadcasts its ephemeris over 30 seconds and the
//: almanac takes 12.5 minutes, and a receiver switched off partway
//: through keeps none of it. Switching the GPS off at the end of every
//: cycle therefore does not shorten the search, it restarts it, and a
//: unit left out overnight under a clear sky can never fix at all.
//:
//: So while the last report had no place in it, the GPS stays on through
//: LISTEN and SLEEP as well. It costs about 25 mA - the whole of this is
//: a hunt for a fix, not a saving - and it is the right trade, because a
//: tracker that cannot say where it is is not doing the job it is
//: carried for.
//:
//: Bounded, so a unit in a drawer at base camp does not hunt for ever:
//: one hunt runs this long, and is armed again every
//: TRACKER_HUNT_RETRY_S. Fifteen minutes is longer than any cold start
//: needs, so a unit that has not fixed by then will not from where it is
//: lying - and an hour later it may have been picked up and carried
//: outside, which is when the next hunt finds the sky.
#ifndef TRACKER_HUNT_MAX_S
  #define TRACKER_HUNT_MAX_S 900
#endif
#ifndef TRACKER_HUNT_RETRY_S
  #define TRACKER_HUNT_RETRY_S 3600
#endif

//: Seconds between the fixes a report carries. A position every four
//: minutes is a dotted line and a team walks 300 m between two of those
//: dots; a fix a minute makes the same report four places instead of one,
//: for one airtime step. See SlimTrack.h.
//:
//: The cost is the GPS waking four times a cycle instead of once, which
//: is the whole price of the feature - set this to 0 to pay none of it.
#ifndef TRACKER_SAMPLE_S
  #define TRACKER_SAMPLE_S 60
#endif

//: How long a slot is, until a base says otherwise in a round-ack.
//:
//: **A default rather than nothing.** Slotting only needs the UNITS to
//: agree, and a value every build compiles in gives that from the first
//: fix - without it a tracker waits for a base that may not be running
//: rounds at all, free-runs, and quietly sends single positions instead
//: of trails. Eight seconds holds a report and one repeat of it at SF12
//: and 62.5 kHz, which is the slowest setting this network uses.
#ifndef TRACKER_SLOT_S
  #define TRACKER_SLOT_S 8
#endif

//: Fixes kept while the base cannot be reached. At a report a minute that is
//: two hours out of range; at five minutes, ten. 12 bytes each.
#ifndef TRACKER_BACKLOG_POINTS
  #define TRACKER_BACKLOG_POINTS 120
#endif
//: Longest nap between checks while asleep. A board with buttons polls them
//: between naps, so it needs short ones or a press goes unseen; without
//: buttons a second is plenty. The CPU sleeps through either.
//:
//: 200 ms on a board with a display: five wakes a second rather than twenty.
//: The old 50 ms meant a unit spent every night of a deployment waking
//: twenty times a second to poll buttons nobody was pressing, and the
//: difference between the two is imperceptible on a press - a joystick is
//: held for far longer than a fifth of a second. This is the one number here
//: that trades feel against battery, so it is stated rather than inherited.
#ifndef TRACKER_SLEEP_SLICE_MS
  #ifdef DISPLAY_CLASS
    #define TRACKER_SLEEP_SLICE_MS 200
  #else
    #define TRACKER_SLEEP_SLICE_MS 1000
  #endif
#endif

namespace tracker {

//: The same bounds tracker_mode.py enforces, so a command the base accepts is
//: one the unit will follow.
static const uint32_t MIN_INTERVAL_S = 60;
static const uint32_t MAX_INTERVAL_S = 3600;

static_assert(TRACKER_LISTEN_S < MIN_INTERVAL_S,
              "the listen window must fit inside the shortest interval");
static_assert(TRACKER_BACKLOG_POINTS > 0 && TRACKER_BACKLOG_POINTS < 65536,
              "the backlog is counted in 16 bits");

//: Fresh fixes to see before sending. The first solution after a hot start is
//: the roughest; the next is a second later and usually much better.
static const uint8_t FIX_SETTLE = 2;

enum Phase : uint8_t { OFF, FIX, LISTEN, SLEEP };

//: 0 (off) or MIN_INTERVAL_S..MAX_INTERVAL_S.
bool validInterval(uint32_t seconds);

//: "!T300" -> 300, "!T0" -> 0. False for anything that is not exactly a
//: tracker command - which is most messages, so this is also the test for
//: "is this one". The value is not range-checked here: a command the unit
//: cannot follow is still a command, not chat.
bool parseCommand(const char* text, uint32_t& seconds);

//: "!F30" -> 30 minutes of find mode, "!F0" -> end it. Minutes rather than
//: seconds: find mode is always the fastest the unit will go, and what a
//: parent chooses is how long to pay for it in battery.
bool parseFindCommand(const char* text, uint32_t& minutes);

// ---- the family finder ------------------------------------------------------
//: The finder has no base. A parent's phone holds an ordinary MeshCore node
//: named "MCMD", every tracker in the family is on one channel named after
//: the family, and positions are broadcast on it - so both parents see every
//: check-in, rather than one device collecting them.
//:
//: That is also why commands ride the channel rather than being direct
//: messages: a direct message needs each side to hold the other's public key,
//: and a tracker asleep for all but ten seconds of every cycle may never hear
//: the advert that would give it one.

//: A parent's node: named exactly "MCMD". Trackers take commands from one,
//: as they do from a base - see MyMesh::isBaseName.
bool isParentName(const char* name);

//: A command as it arrives on the channel.
struct ChannelCmd {
  char sender[24];      //: who MeshCore says sent it - checked by the caller
  bool find;            //: "!F" (find mode) rather than "!T" (usual interval)
  uint32_t value;       //: minutes when `find`, seconds otherwise
  char target[32];      //: the unit it names, or "*"
};

//: "MCMD: !F30 BELLA" -> {sender "MCMD", find, 30, "BELLA"}. False for
//: anything that is not exactly a command, which is most messages - so this
//: is also the test for "is this one". Who is allowed to send it, and whether
//: `target` is us, are the caller's to decide.
bool parseChannelCommand(const char* text, ChannelCmd& out);

//: Whether a command's target names this unit: "*", or its name, ignoring the
//: case of the letters.
bool targetMatches(const char* target, const char* self_name);

class Schedule {
public:
  Schedule(uint32_t fix_timeout_ms = TRACKER_FIX_TIMEOUT_S * 1000UL,
           uint32_t listen_ms = TRACKER_LISTEN_S * 1000UL);

  //: Enter tracker mode, or restart it with a new interval. The first report
  //: is looked for straight away. False for an interval out of range.
  bool start(uint32_t interval_s, uint32_t now);
  void stop() { _phase = OFF; }

  bool on() const { return _phase != OFF; }
  Phase phase() const { return _phase; }
  //: The reporting interval, or 0 when off - which is the command that would
  //: put the unit in its current state.
  uint32_t interval() const { return on() ? _interval_s : 0; }

  //: One turn of the loop. `fix` is a fresh fix ready to send; `quiet` is
  //: nothing waiting to go out or come back. Returns true exactly once per
  //: cycle: the moment to send the position.
  //:
  //: **It returns true whether or not there is a fix.** A unit under
  //: canopy that cannot see the sky still has to be on the map as heard,
  //: because "MTRK03, no fix" and nothing at all are completely different
  //: things to an operator: the first is a radio working and a sky that
  //: is not, the second is a missing team. The caller sends what it has,
  //: and `fix` is what tells it which of the two this is.
  bool step(uint32_t now, bool fix, bool quiet);

  // ---- samples between reports --------------------------------------
  //
  // A position every four minutes is a dotted line, and a team walks 300 m
  // between two of those dots. Which side of a bluff they walked is what a
  // coverage map is for, so the unit takes a fix every minute and the
  // report carries the last four. On the air that is one frame one airtime
  // step larger - 12% - rather than four transmissions. See SlimTrack.h.
  //
  // The cost is the GPS, which now wakes four times a cycle instead of
  // once. That is the whole price, and it is why this is a setting rather
  // than the only behaviour.

  enum Action : uint8_t {
    NOTHING,   //: keep waiting
    SAMPLE,    //: take a fix and keep it; nothing goes on the air
    REPORT,    //: the slot is open - send what has been collected
  };

  //: One turn of the loop, saying which of the three this moment is.
  //: `step` is this with SAMPLE folded into NOTHING, for callers that do
  //: not collect a trail.
  Action tick(uint32_t now, bool fix, bool quiet);

  //: How many samples a report carries, and how far apart they are. One
  //: sample is the plain behaviour: a report of where it is now.
  void setSamples(uint8_t count, uint16_t cadence_s);
  uint8_t sampleCount() const { return _sample_n; }
  uint16_t sampleCadence() const { return _sample_s; }

  //: The next instant a fix is wanted, in epoch seconds. The newest
  //: sample is the slot itself, so this is the slot when nothing is owed
  //: before it.
  uint32_t nextSampleAt(uint32_t now) const;

  //: Milliseconds until the next cycle begins, while asleep. 0 otherwise.
  uint32_t sleepLeft(uint32_t now) const;

  // ---- slots ---------------------------------------------------------
  //
  // At SF11 and 250 kHz a position is 313 ms and units can take their
  // chances. At SF12 and 62.5 kHz - the setting that buys 8.5 dB and the
  // bush penetration this protocol exists for - it is 2.8 SECONDS, and
  // sixteen units taking their chances offer the channel three quarters
  // of its time and land about a quarter of their reports. Random access
  // peaks at 18% offered load and collapses past it.
  //
  // So each unit owns a moment instead. It costs nothing on the wire: the
  // number is already in the callsign, the interval is already commanded,
  // and the time comes from the GPS every tracker already carries. The
  // arithmetic is slim_round.py, transcribed in SlimRound.cpp, because the
  // two sides disagreeing by one second is two units transmitting over
  // each other for ever with nothing anywhere reporting it.

  //: What the time is. `epoch_s` is real seconds - from the GPS, or from
  //: the base's round-ack for a unit that has never had a fix. Without one
  //: the schedule free-runs, which is what a unit in a canyon must keep
  //: doing rather than going silent.
  void setClock(uint32_t epoch_s, uint32_t now);
  bool hasClock() const { return _clock_set; }
  uint32_t epoch(uint32_t now) const;

  //: Which slot this unit owns. `unit` 0 turns slotting off, which is the
  //: state every unit is in until the base programs it.
  void setSlot(uint8_t unit, uint16_t slot_s, uint16_t lead_s);
  uint8_t slotUnit() const { return _slot_unit; }
  uint16_t slotSeconds() const { return _slot_s; }
  //: Slotted AND able to say when: both are needed, and a unit can lose
  //: the second one halfway up a hill.
  bool slotted() const { return _slot_unit != 0 && _clock_set; }

  //: When this unit next transmits, in epoch seconds. 0 if unslotted.
  uint32_t nextSlot(uint32_t now) const;

private:
  void enter(Phase p, uint32_t now) { _phase = p; _phase_at = now; }

  Phase _phase;
  uint32_t _interval_s;
  uint32_t _cycle_at;       // when the current cycle's FIX began
  uint32_t _phase_at;       // when the current phase began
  uint32_t _fix_timeout_ms;
  uint32_t _listen_ms;
  uint32_t _epoch_s = 0;    // real time, as last told
  uint32_t _clock_at = 0;   // ...and the millis() when we were told
  bool _clock_set = false;
  uint8_t _slot_unit = 0;
  uint16_t _slot_s = 0;
  uint16_t _lead_s = 0;     // how long before the slot to start the GPS
  uint8_t _sample_n = 1;    // samples per report, including the newest
  uint16_t _sample_s = 0;   // seconds between them, 0 for none
  uint32_t _sample_at = 0;  // the sample instant being served now
};

//: A fix, stamped with the unit's uptime in seconds. Uptime rather than wall
//: time: the base is sent ages, so a unit whose clock was never set still
//: dates its backlog correctly.
//:
//: It may also hold NO fix, which is a different thing from a fix of zero.
//: A unit under canopy still reports - being heard with no position and
//: not being heard at all are not the same news - and that report goes on
//: the same list as every other, to be replayed as history if the base
//: missed it. Carried as a coordinate it would be replayed as one: 0N 0E,
//: a thousand kilometres off West Africa, plotted with the same confidence
//: as a real position and an hour after the unit was actually there.
struct Fix { float lat, lon; uint32_t at_s; };

//: A report from a unit that could not see the sky.
inline Fix noFix(uint32_t at_s) {
  Fix f = {NAN, NAN, at_s};
  return f;
}

//: Is there a place in here? Asked wherever one is about to be used as
//: one - NaN rather than a reserved coordinate, because every coordinate
//: is somewhere and this must not be anywhere.
inline bool hasFix(const Fix& f) { return !isnan(f.lat) && !isnan(f.lon); }

//: The fixes the base does not have, oldest first. When it fills the oldest
//: go: where somebody is now heading matters more than where they set off.
class Backlog {
public:
  void push(const Fix& f);
  uint16_t size() const { return _n; }
  const Fix& operator[](uint16_t i) const;     // 0 is the oldest
  void drop(uint16_t n);                       // the n oldest: the base has them
  //: Fixes pushed out for lack of room since the unit started.
  uint32_t lost() const { return _lost; }

private:
  Fix _buf[TRACKER_BACKLOG_POINTS];
  uint16_t _head = 0, _n = 0;
  uint32_t _lost = 0;
};

//: What the base has been given, going by its acknowledgements.
class Reports {
public:
  enum Ack : uint8_t { NOT_OURS, REPORT, HISTORY };

  //: A report went out, and this is the ack it will come back with. The one
  //: before it, if never acknowledged, joins the backlog now: the base does
  //: not have it.
  void sent(const Fix& f, uint32_t ack);
  //: A report that could not go out at all. It counts as missed either
  //: way; only a report with a place in it has anything to hand over.
  void unsent(const Fix& f) {
    if (hasFix(f)) _backlog.push(f);
    if (_missed < 255) _missed++;
  }
  //: An acknowledgement arrived. Which of ours it confirms, if either.
  Ack acked(uint32_t ack);

  //: The base's round-ack named this unit. Exactly what an
  //: acknowledgement of the outstanding report means - the base has it -
  //: but carried by one frame for the whole team instead of sixteen. At
  //: SF12 those sixteen are 40 seconds of airtime and sixteen more
  //: chances to collide, which is most of what slotting was for.
  Ack heardInRound();
  //: The backlog may go now: the latest report was confirmed, so the base can
  //: hear us, and no history message is still waiting for its ack.
  bool historyDue() const;
  //: The next "!H" message, from the oldest points, aged against `now_s`.
  //: `out` needs slimcodec::HIST_MAX_TEXT + 1 bytes. Returns how many points
  //: it carries; 0 means nothing to send.
  uint8_t nextHistory(char* out, uint32_t now_s) const;
  //: That message went out with this ack, carrying `n` points.
  void historySent(uint32_t ack, uint8_t n) { _hist_ack = ack; _hist_n = n; }
  //: A report is out and not yet acknowledged.
  bool waiting() const { return _report_ack != 0 || _hist_ack != 0; }

  //: Reports in a row that were never acknowledged.
  uint8_t missed() const { return _missed; }

  //: Has this unit lost the base?
  //:
  //: **What this decides is whether to listen, not whether to complain.**
  //: A unit in contact is deaf for all but ten seconds of each cycle,
  //: which is cheap and perfectly sufficient - the base only ever needs to
  //: answer a report, and it does that inside the window. A unit that has
  //: lost the base needs the opposite: it has nothing to wait for and no
  //: idea when anything will come, so it sniffs for a preamble instead,
  //: which the radio does by itself at about an eighth of the current of
  //: listening properly. It is looking for any traffic at all - the base's
  //: team map, another unit's position being repeated, a helicopter
  //: overhead - because any of them means there is a way through and the
  //: path it has been keeping can go now.
  bool searching() const { return _missed >= TRACKER_SEARCH_AFTER; }
  const Backlog& backlog() const { return _backlog; }

private:
  Backlog _backlog;
  Fix _last = {NAN, NAN, 0};   // nothing reported yet is not a place either
  uint32_t _report_ack = 0;     // the latest report's, until it comes back
  bool _linked = false;         // the latest report was confirmed
  uint32_t _hist_ack = 0;       // the history message in flight
  uint8_t _hist_n = 0;          // ...and how many points it carries
  uint8_t _missed = 0;          // reports in a row with no answer
};

}  // namespace tracker

#endif  // WITH_TRACKER_MODE
