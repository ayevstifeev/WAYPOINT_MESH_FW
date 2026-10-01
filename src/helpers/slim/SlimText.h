// Text, in as few bytes as it can be said. Transcribed from slim_text.py.
//
// Three tiers and a chooser. The tier is the first byte of the body, so a
// receiver never guesses:
//
//   CODE    one byte for one of 256 agreed phrases
//   NIBBLE  two characters per byte, 16 symbols: numbers, not words
//   PACK6   1.33 characters per byte, 64 symbols: sentences
//   RAW     UTF-8, so nothing is ever refused, only expensive
//
// The ordering matters more than the packing does. Most SAR traffic is not free
// text - it is one of a few dozen things - and no compression scheme competes
// with not sending the words at all. If RAW turns up often in the field, the
// answer is more entries in CODES, not a wider alphabet.
//
// **Why 6 bits and not 5.** A 5-bit alphabet with a letters/digits shift packs
// letter-heavy text tighter, and it was written that way first. It was replaced
// because the shift is STATE: one flipped bit in a shift symbol changes the
// mode for everything after it, so a single bit error turns the rest of the
// message into digits. Six bits is stateless - one flipped bit costs exactly
// one wrong character. On a link chosen for its margin through wet beech
// forest, a codec whose failure mode is "the whole tail is garbage" is the
// wrong trade for two bytes.

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace slimtext {

enum Tier : uint8_t { TIER_CODE = 0, TIER_NIBBLE = 1, TIER_PACK6 = 2, TIER_RAW = 3 };

//: Longest message this codec will encode.
//:
//: It was 80, justified by a PACK6 body of 56 bytes being "the most a frame
//: can carry" - true of the 64-byte frame of the time, and SLIM_MAX_FRAME is
//: now 192. Past 80 characters both packed tiers were REFUSED and the
//: message went out RAW: a third more airtime than PACK6, silently, on
//: exactly the long messages where airtime is worth most. The Python
//: reference has no ceiling at all, so the two stacks also disagreed about
//: what the same message weighs.
//:
//: 176 is slimapp::MAX_TEXT - as much as the layer above will ever hand
//: down. It costs nothing now: choosing a tier is arithmetic, so nothing
//: here is sized by it.
static const uint8_t MAX_TEXT = 176;

//: Agreed at the briefing, the same table on every unit. **Order is the wire
//: format**: entries are only ever appended, never inserted or reordered. A
//: unit running an older table shows "[code 47]" rather than the wrong phrase.
extern const char* const CODES[];
extern const uint8_t CODE_COUNT;

//: The 6-bit alphabet, 63 characters and an END symbol.
//:
//: Exported for anything that has to OFFER characters rather than merely
//: carry them - a joystick composer on a handheld, where the letters
//: somebody can reach ARE the alphabet. Staying inside it keeps a typed
//: message on PACK6 instead of dropping it to RAW, which is a third more
//: airtime for the same words, decided by a character nobody meant to
//: choose.
extern const char PACK6_ALPHABET[];

//: How many characters it holds. Beside the array because the array is
//: extern, so sizeof is not available to a caller - and a composer that
//: has to strlen a 63-character literal on every keypress to find out is
//: asking a question the compiler already answered.
extern const uint8_t PACK6_COUNT;

//: The cheapest tier that can carry this string, tier byte included.
//: Returns the body length, or -1 if it will not fit `cap`.
int encode(const char* text, uint8_t* out, uint8_t cap);

//: Text from a body. Writes at most `cap` bytes including the NUL, and returns
//: the length written.
//:
//: Never fails: an unknown code or a tier from a future version produces a
//: readable placeholder. A unit running an older table must show that it
//: received something it cannot name - not the wrong phrase, and not nothing.
int decode(const uint8_t* body, uint8_t len, char* out, uint8_t cap);

}  // namespace slimtext
