// Four places in one report: where it is, and where it has been.
// Transcribed from slim_track.py.
//
// A position every four minutes is a dotted line. A team walks 300 m between
// two of those dots, and which side of a bluff they walked is exactly what a
// coverage map is for. Sampling every minute and reporting every four gives
// the map four times the detail for ONE transmission, because the cost of a
// report is almost all in the transmission and hardly any in its length.
//
// LoRa encodes payload in whole blocks, so at SF12 and 62.5 kHz airtime steps
// every five bytes: 13-17 bytes cost 2834 ms, 18-22 cost 3162. A single
// position is 13 bytes, so three more samples do not quadruple anything - they
// move the frame one step, 12%. Sent as four separate reports the same four
// places would be four transmissions and 11.3 seconds.
//
// **The absolute stays big on purpose.** Offsets inside a 20 km box would fit
// the whole report one step lower, but a box has to be hung off something, and
// a unit whose first report comes from outside it is not degraded - it is
// plotted twenty kilometres from where it is, with nothing reporting a fault.
// Three bytes is not worth a class of bug that puts a team in the wrong
// valley. The deltas can be small because they are anchored to something
// exact; the anchor cannot be.
//
//     bits
//       0-1    form       0 for this layout
//       2-5    cadence    seconds between samples, an index into CADENCES
//       6-29   latitude   24 bits, exactly slimcodec::encodePos
//      30-53   longitude  24 bits
//      54-75   sample -1  east 11 bits, north 11 bits, from the one after it
//      76-97   sample -2
//      98-119  sample -3
//
// Fifteen bytes, nothing left over.
//
// **The cadence is carried, not agreed.** A base assuming thirty seconds for a
// tracker sampling sixty would draw every trail at half length, in the right
// direction, with nothing looking wrong.
//
// **The deltas chain backwards**, so eleven bits at a five-metre step covers
// +/-5,115 m between samples: 307 km/h at a one-minute cadence. NONE, the one
// value the range gives up, means "no sample here" - and because the chain
// runs backwards from the newest, a gap ends the older half of the trail
// rather than corrupting it.

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace slimtrack {

static const double STEP_M = 5.0;
static const uint8_t DELTA_BITS = 11;
static const int16_t DELTA_MIN = -1023, DELTA_MAX = 1023;
static const int16_t NONE = -1024;

//: Samples in one report, newest first, including the absolute.
static const uint8_t SAMPLES = 4;
static const uint8_t DELTAS = SAMPLES - 1;
static const uint8_t BODY_BYTES = 15;

//: Seconds between samples, by the four-bit field. **Append only** - the index
//: is the wire format. Index 0 is a minute because that is what a four-minute
//: report wants, and the common case should be the one a build with a wrong
//: table still gets right.
extern const uint16_t CADENCES[16];
static const uint16_t DEFAULT_CADENCE_S = 60;

struct Point { double lat, lon; bool fix; };

//: The nearest cadence this format can express. Never fails: a tracker whose
//: interval does not divide evenly still reports, and the error is in where
//: the older samples are drawn rather than whether they are drawn at all.
uint8_t cadenceIndex(uint16_t seconds);

//: Up to SAMPLES points, newest first, as one BODY_BYTES body. Returns the
//: length, or -1 if the newest has no fix - a report with nothing current is
//: the six zero bytes that mean "heard, no fix", which already exists.
//:
//: A sample too far from the one after it to express ends the trail rather
//: than being clamped: clamping would draw a line to a place nobody went.
int encode(const Point* points, uint8_t count, uint16_t cadence_s,
           uint8_t* out, uint8_t cap);

//: The samples back, newest first. Returns how many, or -1 for a body that is
//: not a track. `cadence_s` is what the sender said, not what anyone assumed.
int decode(const uint8_t* body, uint8_t len, Point* out, uint8_t cap,
           uint16_t& cadence_s);

//: Told apart from a plain position by length: six bytes against fifteen.
bool isTrack(const uint8_t* body, uint8_t len);

//: How many samples a report at this interval can actually carry. Padding it
//: with samples never taken is how a trail grows places a team never stood.
uint8_t samplesFor(uint32_t interval_s, uint16_t cadence_s);

}  // namespace slimtrack
