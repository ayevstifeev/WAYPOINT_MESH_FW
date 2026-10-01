#include "SlimPacket.h"

#include <string.h>

namespace slim {

uint8_t  TTL_INIT        = 3;
uint32_t OPERATION_EPOCH = 0;

// ---- SipHash-2-4 --------------------------------------------------------
//
// Transcribed from siphash.py. Chosen over a truncated HMAC-SHA256 because it
// was designed for exactly this: short messages, short tags, small processors.
// A 12-byte packet is one compression round.

static inline uint64_t rotl(uint64_t x, int b) {
  return (x << b) | (x >> (64 - b));
}

#define SIPROUND                     \
  do {                               \
    v0 += v1; v1 = rotl(v1, 13);     \
    v1 ^= v0; v0 = rotl(v0, 32);     \
    v2 += v3; v3 = rotl(v3, 16);     \
    v3 ^= v2;                        \
    v0 += v3; v3 = rotl(v3, 21);     \
    v3 ^= v0;                        \
    v2 += v1; v1 = rotl(v1, 17);     \
    v1 ^= v2; v2 = rotl(v2, 32);     \
  } while (0)

// Byte-at-a-time rather than a cast, so this behaves the same on a big-endian
// MCU as on the little-endian host the vectors were generated on.
static inline uint64_t le64(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 7; i >= 0; i--) v = (v << 8) | p[i];
  return v;
}

uint64_t siphash24(const uint8_t key[16], const uint8_t* data, size_t len) {
  const uint64_t k0 = le64(key), k1 = le64(key + 8);
  uint64_t v0 = k0 ^ 0x736F6D6570736575ULL;
  uint64_t v1 = k1 ^ 0x646F72616E646F6DULL;
  uint64_t v2 = k0 ^ 0x6C7967656E657261ULL;
  uint64_t v3 = k1 ^ 0x7465646279746573ULL;

  const size_t whole = len - (len % 8);
  for (size_t i = 0; i < whole; i += 8) {
    const uint64_t m = le64(data + i);
    v3 ^= m;
    SIPROUND; SIPROUND;
    v0 ^= m;
  }

  // The last block carries the length in its top byte. That is what stops two
  // messages differing only in trailing zeros from sharing a tag.
  uint64_t last = ((uint64_t)(len & 0xFF)) << 56;
  for (size_t i = len; i > whole; i--) {
    last |= ((uint64_t)data[i - 1]) << (8 * (i - 1 - whole));
  }
  v3 ^= last;
  SIPROUND; SIPROUND;
  v0 ^= last;

  v2 ^= 0xFF;
  SIPROUND; SIPROUND; SIPROUND; SIPROUND;
  return v0 ^ v1 ^ v2 ^ v3;
}

void sipTag(const uint8_t key[16], const uint8_t* data, size_t len,
            uint8_t* out, uint8_t size) {
  const uint64_t h = siphash24(key, data, len);
  for (uint8_t i = 0; i < size && i < 8; i++) out[i] = (uint8_t)(h >> (8 * i));
}

bool sipVerify(const uint8_t key[16], const uint8_t* data, size_t len,
               const uint8_t* given, uint8_t size) {
  uint8_t want[8];
  sipTag(key, data, len, want, size);
  // No early exit: the time taken must not depend on how much of the tag was
  // right, or it leaks the tag one byte at a time.
  uint8_t diff = 0;
  for (uint8_t i = 0; i < size; i++) diff |= (uint8_t)(want[i] ^ given[i]);
  return diff == 0;
}

// ---- nonce --------------------------------------------------------------

void makeNonce(uint8_t out[16], uint8_t src, uint8_t type, uint32_t seq_ext) {
  memset(out, 0, 16);
  out[0] = 'M'; out[1] = 'S';
  out[2] = (uint8_t)(OPERATION_EPOCH >> 24);
  out[3] = (uint8_t)(OPERATION_EPOCH >> 16);
  out[4] = (uint8_t)(OPERATION_EPOCH >> 8);
  out[5] = (uint8_t)(OPERATION_EPOCH);
  out[6] = src;
  out[7] = (uint8_t)(type & 7);
  out[8]  = (uint8_t)(seq_ext >> 24);
  out[9]  = (uint8_t)(seq_ext >> 16);
  out[10] = (uint8_t)(seq_ext >> 8);
  out[11] = (uint8_t)(seq_ext);
  // out[12..15] stay zero: the CTR block counter, which the cipher advances.
}

// ---- frame --------------------------------------------------------------

static inline uint8_t flagsByte(bool enc, uint8_t type, uint8_t route,
                                uint8_t ttl) {
  return (uint8_t)(((enc ? 1 : 0) << 7) | ((type & 7) << 4) |
                   ((route & 1) << 3) | (ttl & 7));
}

// What the tag covers: everything except the two fields that change in flight.
//
// ttl is decremented and HOP is rewritten at every hop, so neither can be
// authenticated end to end. They are masked to zero rather than skipped so that
// the tag still covers their positions — a packet cannot be made shorter or
// longer without changing the input.
//
// Note what is NOT masked: bit 7, the enc flag, and bit 3, the route mode. Both
// are inside the tag on purpose. Encryption cannot be stripped by clearing a
// bit, and a repeater cannot promote a DIRECT packet to a flood even helpfully.
static void macInput(const uint8_t* frame, uint8_t len, uint8_t* out) {
  memcpy(out, frame, len - TAG_BYTES);
  out[0] &= 0xF8;
#if SLIM_ADDR_NIBBLE
  out[1] &= 0xF0;
#else
  out[2] = 0;
#endif
}

static inline bool addrOk(uint16_t a) { return a <= MAX_ADDR; }

int build(const uint8_t key[16], const Build& b, Cipher* cipher,
          uint8_t* out, size_t cap) {
  const bool broadcast = isBroadcastType(b.type);
  const uint8_t ttl = b.ttl_set ? b.ttl : TTL_INIT;

  if (ttl > MAX_TTL) return ERR_ARG;
  if (broadcast && b.dst != ADDR_BROADCAST) return ERR_ARG;
  // DIRECT names one next hop in HOP and the forwarder resolves the hop after
  // that from route[DST]. A broadcast type carries no DST, so the chain has
  // nowhere to continue past the first repeater.
  if (b.route == ROUTE_DIRECT && broadcast) return ERR_ARG;
  if (!addrOk(b.src) || !addrOk(b.hop) || !addrOk(b.dst)) return ERR_ARG;
  if (b.body_len > MAX_BODY) return ERR_ARG;
  if (b.enc && cipher == nullptr) return ERR_ARG;

#if SLIM_ADDR_NIBBLE
  const uint8_t addr_bytes = broadcast ? 1 : 2;
#else
  const uint8_t addr_bytes = broadcast ? 2 : 3;
#endif
  const size_t total = 1 + addr_bytes + 1 + b.body_len + TAG_BYTES;
  if (total > cap || total > SLIM_MAX_FRAME) return ERR_ARG;

  size_t i = 0;
  out[i++] = flagsByte(b.enc, b.type, b.route, ttl);
#if SLIM_ADDR_NIBBLE
  out[i++] = (uint8_t)(((b.src & 0x0F) << 4) | (b.hop & 0x0F));
  if (!broadcast) out[i++] = (uint8_t)((b.dst & 0x0F) << 4);  // low nibble spare
#else
  out[i++] = b.src;
  out[i++] = b.hop;
  if (!broadcast) out[i++] = b.dst;
#endif
  out[i++] = b.seq;
  if (b.body_len) memcpy(out + i, b.body, b.body_len);

  if (b.enc) {
    uint8_t nonce[16];
    makeNonce(nonce, b.src, b.type, b.seq_ext);
    cipher->xorStream(nonce, out + i, b.body_len);
  }
  i += b.body_len;

  // Encrypt-then-MAC: the tag covers the ciphertext, so a forged body is
  // rejected before anything tries to decrypt it.
  uint8_t scratch[SLIM_MAX_FRAME];
  memcpy(scratch, out, i);
  scratch[0] &= 0xF8;
#if SLIM_ADDR_NIBBLE
  scratch[1] &= 0xF0;
#else
  scratch[2] = 0;
#endif
  sipTag(key, scratch, i, out + i, TAG_BYTES);
  return (int)(i + TAG_BYTES);
}

int parse(const uint8_t key[16], const uint8_t* frame, uint8_t len,
          Frame& f, Cipher* cipher, SeqExtFn seq_ext, void* ctx) {
#if SLIM_ADDR_NIBBLE
  const uint8_t least = 3 + TAG_BYTES;
#else
  const uint8_t least = 4 + TAG_BYTES;
#endif
  if (len < least || len > SLIM_MAX_FRAME) return ERR_BAD;

  const bool enc  = (frame[0] >> 7) != 0;
  const uint8_t type = (uint8_t)((frame[0] >> 4) & 7);
  const bool broadcast = isBroadcastType(type);
  if (!broadcast && len < least + 1) return ERR_BAD;

  uint8_t scratch[SLIM_MAX_FRAME];
  macInput(frame, len, scratch);
  if (!sipVerify(key, scratch, len - TAG_BYTES, frame + len - TAG_BYTES,
                 TAG_BYTES)) {
    return ERR_BAD;
  }

  // Past this line everything is authenticated.
  const uint8_t ttl = (uint8_t)(frame[0] & 7);
  uint8_t src, hop, dst = ADDR_BROADCAST;
  size_t i;
#if SLIM_ADDR_NIBBLE
  src = (uint8_t)(frame[1] >> 4);
  hop = (uint8_t)(frame[1] & 0x0F);
  i = 2;
  if (!broadcast) dst = (uint8_t)(frame[i++] >> 4);
#else
  src = frame[1];
  hop = frame[2];
  i = 3;
  if (!broadcast) dst = frame[i++];
#endif
  const uint8_t seq = frame[i++];
  const uint8_t body_len = (uint8_t)(len - TAG_BYTES - i);
  if (body_len > MAX_BODY) return ERR_BAD;

  if (enc && cipher == nullptr) return ERR_NEEDS_CIPHER;

  f.type = type;
  f.enc  = enc;
  f.route = (uint8_t)((frame[0] >> 3) & 1);
  f.ttl = ttl;
  // A sender transmits at TTL_INIT and each repeater decrements before
  // retransmitting, so a packet arriving at ttl T has been retransmitted
  // (TTL_INIT - T) times and has crossed one more radio hop than that to reach
  // us. The +1 is the leg from the last transmitter, which no counter records;
  // omitting it makes every neighbour look like a stranger.
  {
    const int h = (int)TTL_INIT - (int)ttl + 1;
    f.hops = (uint8_t)(h < 1 ? 1 : h);
  }
  f.src = src;
  f.hop = hop;
  f.dst = dst;
  f.seq = seq;
  f.body_len = body_len;
  if (body_len) memcpy(f.body, frame + i, body_len);

  if (enc) {
    const uint32_t ext = seq_ext ? seq_ext(ctx, src, seq) : seq;
    uint8_t nonce[16];
    makeNonce(nonce, src, type, ext);
    cipher->xorStream(nonce, f.body, body_len);
  }
  return OK;
}

uint8_t forward(uint8_t* frame, uint8_t len, uint8_t next_hop) {
  const uint8_t ttl = (uint8_t)(frame[0] & 7);
  if (ttl == 0) return 0;
  frame[0] = (uint8_t)((frame[0] & 0xF8) | (ttl - 1));
#if SLIM_ADDR_NIBBLE
  frame[1] = (uint8_t)((frame[1] & 0xF0) | (next_hop & 0x0F));
#else
  frame[2] = next_hop;
#endif
  return len;
}

}  // namespace slim
