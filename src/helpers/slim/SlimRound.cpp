#include "SlimRound.h"

namespace slimround {

uint32_t slotStart(uint32_t epoch, uint32_t interval, uint8_t unit,
                   uint8_t slot_s) {
  if (interval == 0) return epoch;
  // unit 1 takes the first slot, so sixteen numbers fill sixteen moments
  // exactly - and unit 16 does not land back at the start of the round.
  const uint32_t nth = (uint32_t)((unit + MAX_UNIT - 1) % MAX_UNIT);
  return epoch - (epoch % interval) + nth * slot_s;
}

uint32_t nextSlot(uint32_t epoch, uint32_t interval, uint8_t unit,
                  uint8_t slot_s) {
  const uint32_t at = slotStart(epoch, interval, unit, slot_s);
  return at > epoch ? at : at + interval;
}

bool inSlot(uint32_t epoch, uint32_t interval, uint8_t unit, uint8_t slot_s) {
  const uint32_t at = slotStart(epoch, interval, unit, slot_s);
  return epoch >= at && epoch < at + slot_s;
}

uint32_t baseSlot(uint32_t interval, uint8_t slot_s, uint8_t units) {
  const uint32_t after = (uint32_t)units * slot_s;
  const uint32_t last = (interval > slot_s) ? interval - slot_s : 0;
  return after < last ? after : last;
}

bool slotsFit(uint32_t interval, uint8_t slot_s, uint8_t units) {
  return (uint32_t)(units + 1) * slot_s <= interval;
}

int encodeAck(const Ack& a, uint8_t* out, uint8_t cap) {
  if (cap < ACK_BYTES) return -1;
  uint8_t slot = a.slot_s;
  if (slot < MIN_SLOT_S) slot = MIN_SLOT_S;
  out[0] = (uint8_t)(a.epoch >> 24);
  out[1] = (uint8_t)(a.epoch >> 16);
  out[2] = (uint8_t)(a.epoch >> 8);
  out[3] = (uint8_t)(a.epoch);
  out[4] = slot;
  out[5] = (uint8_t)(a.heard >> 8);
  out[6] = (uint8_t)(a.heard);
  return ACK_BYTES;
}

bool decodeAck(const uint8_t* body, uint8_t len, Ack& out) {
  if (body == 0 || len < ACK_BYTES) return false;
  if (body[4] < MIN_SLOT_S) return false;
  out.epoch = ((uint32_t)body[0] << 24) | ((uint32_t)body[1] << 16)
            | ((uint32_t)body[2] << 8) | (uint32_t)body[3];
  out.slot_s = body[4];
  out.heard = (uint16_t)(((uint16_t)body[5] << 8) | body[6]);
  return true;
}

bool wasHeard(uint16_t heard, uint8_t unit) {
  if (unit < 1 || unit > MAX_UNIT) return false;
  return (heard >> (unit - 1)) & 1;
}

uint16_t withUnit(uint16_t heard, uint8_t unit) {
  if (unit < 1 || unit > MAX_UNIT) return heard;
  return (uint16_t)(heard | (uint16_t)(1u << (unit - 1)));
}

}  // namespace slimround
