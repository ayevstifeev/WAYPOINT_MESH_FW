// Everyone transmits at a different moment, and the base answers once.
// Transcribed from slim_round.py.
//
// At SF11 and 250 kHz a position is 313 ms and units can take their chances.
// At SF12 and 62.5 kHz - the setting that buys 8.5 dB and the bush
// penetration this protocol exists for - it is 2.8 SECONDS:
//
//     16 trackers, burst of 3, random      74% offered   ~23% get through
//     16 trackers, burst of 1, random      44% offered   ~42% get through
//     16 trackers, slotted                 35% offered   100%
//
// Random access peaks at 18% offered load and collapses past it, so this is
// not a tuning knob: without slotting the long-range setting cannot carry a
// team, and every dB bought by SF12 is handed straight back in collisions.
//
// **It costs nothing on the wire.** The number is already in the callsign, the
// interval is already commanded, and the time comes from the GPS every tracker
// already carries:
//
//     slot_start = interval * floor(epoch / interval) + unit * slot_s
//
// **A slot holds a flood, not a transmission** - the report and every repeat of
// it. So at SF12 a hop costs 2.8 s of every slot in the round, which is why
// height beats hops: one repeater high up beats three chained.
//
// **No clock, no slot, and that must not be fatal.** A unit that has never had
// a fix, or lost it in a canyon, free-runs and takes its chances as it does
// today. Degraded, not silent - and the round-ack carries the time, so it
// picks a slot up again the moment it hears one.

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace slimround {

//: Bit `n - 1` of the map is unit `n`.
static const uint8_t MAX_UNIT = 16;
static const uint8_t MIN_SLOT_S = 1, MAX_SLOT_S = 255;
static const uint8_t ACK_BYTES = 7;

//: When this unit's slot begins, in the round `epoch` falls in - not the next
//: one, so a caller can ask whether it is inside its slot right now.
uint32_t slotStart(uint32_t epoch, uint32_t interval, uint8_t unit,
                   uint8_t slot_s);

//: When it next transmits, strictly after `epoch`.
uint32_t nextSlot(uint32_t epoch, uint32_t interval, uint8_t unit,
                  uint8_t slot_s);

//: Is this instant inside this unit's slot?
bool inSlot(uint32_t epoch, uint32_t interval, uint8_t unit, uint8_t slot_s);

//: Seconds into the round at which the base sends its round-ack: after every
//: unit slot, so the map it carries is this round's rather than the last's.
uint32_t baseSlot(uint32_t interval, uint8_t slot_s, uint8_t units = MAX_UNIT);

//: Do this many slots and a round-ack fit inside the interval? The check that
//: stops a one-minute round being asked to hold sixteen nine-second slots.
bool slotsFit(uint32_t interval, uint8_t slot_s, uint8_t units = MAX_UNIT);

// ---- the round-ack ------------------------------------------------------
//
//     epoch:4   slot_s:1   heard:2       seven bytes
//
// Three jobs, which is why it is worth its 2.5 seconds: it acknowledges
// everyone at once (against sixteen separate acknowledgements costing 40 s of
// airtime and sixteen more chances to collide); it is the time beacon, so a
// unit with no fix can still hold a slot; and it is proof the base is alive,
// which is what a tracker is really asking when it decides it has lost
// contact.

struct Ack {
  uint32_t epoch;
  uint8_t slot_s;
  uint16_t heard;
};

//: Writes ACK_BYTES. Returns the length, or -1 if `cap` is too small.
int encodeAck(const Ack& a, uint8_t* out, uint8_t cap);

//: Reads one back. False for a short body - never a guess, because a wrong
//: epoch does not degrade anything, it puts every unit in the wrong slot at
//: once.
bool decodeAck(const uint8_t* body, uint8_t len, Ack& out);

//: The only question a tracker asks of a round-ack.
bool wasHeard(uint16_t heard, uint8_t unit);

//: ...and the only thing a base does to build one.
uint16_t withUnit(uint16_t heard, uint8_t unit);

}  // namespace slimround
