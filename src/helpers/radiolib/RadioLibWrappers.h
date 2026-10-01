#pragma once

#include <Mesh.h>
#include <RadioLib.h>

class RadioLibWrapper : public mesh::Radio {
protected:
  PhysicalLayer* _radio;
  mesh::MainBoard* _board;
  uint32_t n_recv, n_sent, n_recv_errors;
  int16_t _noise_floor, _threshold;
  uint16_t _num_floor_samples;
  int32_t _floor_sample_sum;
  uint8_t _preamble_sf;

  void idle();
  void startRecv();
  float packetScoreInt(float snr, int sf, int packet_len);
  virtual bool isReceivingPacket() =0;
  virtual void doResetAGC();

public:
  RadioLibWrapper(PhysicalLayer& radio, mesh::MainBoard& board) : _radio(&radio), _board(&board), _preamble_sf(0) { n_recv = n_sent = 0; }

  void begin() override;
  virtual void powerOff() { _radio->sleep(); }
  int recvRaw(uint8_t* bytes, int sz) override;
  uint32_t getEstAirtimeFor(int len_bytes) override;
  bool startSendRaw(const uint8_t* bytes, int len) override;
  bool isSendComplete() override;
  void onSendFinished() override;
  bool isInRecvMode() const override;
  bool isChannelActive();

  bool isReceiving() override { 
    if (isReceivingPacket()) return true;

    return isChannelActive();
  }

  virtual void setParams(float freq, float bw, uint8_t sf, uint8_t cr) = 0;

#ifdef WITH_MESH_SLIM
  //: Move the whole physical layer between MeshCore and MESH_SLIM.
  //:
  //: These settings are ONE decision, not four, and changing some without the
  //: others leaves the radio in a state that is neither protocol:
  //:
  //:   sync word  0x12 -> 0x2B  the hardware network identifier. The SX126x
  //:                            compares it before raising an RX interrupt, so
  //:                            the two networks become mutually deaf at zero
  //:                            CPU cost. This is also the safety interlock: a
  //:                            half-flashed unit hears everything or nothing,
  //:                            never a mis-parsed mixture.
  //:   preamble     16 -> 8     40% of a 12-byte packet's airtime is preamble.
  //:                            Below 8 the receiver starts missing packets.
  //:   CRC          on -> off   the 4-byte MAC rejects corruption already, and
  //:                            unlike a CRC it also rejects deliberate
  //:                            corruption. Paying for both is paying twice.
  //:
  //: Returns false on radios that have no slim support, leaving them stock.
  virtual bool setProtocolMode(bool slim) { (void)slim; return false; }
#endif
#ifdef WITH_TRACKER_MODE
  //: Between tracker reports. A warm sleep keeps the chip's configuration -
  //: sync word, preamble, CRC - so nothing has to be set up again; powerOff()
  //: is a cold sleep on the SX1262 and would lose all of it. The next
  //: recvRaw() starts receiving again, which is what wakes the chip.
  void sleepWarm();

  //: Sniff for a preamble instead of sleeping. Returns false on a radio
  //: that cannot, and the caller sleeps as before.
  //:
  //: **For a unit that has lost the base.** One in contact is deaf for
  //: all but ten seconds of each cycle, which costs nothing and is
  //: perfectly sufficient: the base only has to answer a report, and it
  //: does that inside the window. A unit that has lost the base has
  //: nothing to wait for and no idea when anything will come, so instead
  //: of sleeping it has the radio listen in short bursts, often enough
  //: that no preamble can pass between two of them.
  //:
  //: The chip does this by itself and the processor sleeps through all of
  //: it, waking only when a packet actually arrives. The cost is set by
  //: the ratio of symbols, not by wall-clock time: a few symbols of
  //: listening per preamble is about an eighth of the current of
  //: listening properly, whatever the spreading factor. At SF12 and
  //: 62.5 kHz that works out at a look roughly once a second; at SF11 and
  //: 250 kHz, eight times as often for the same power.
  virtual bool startSniffing() { return false; }
#endif
  uint32_t getRngSeed();
  void setTxPower(int8_t dbm);

  virtual float getCurrentRSSI() =0;
  virtual uint8_t getSpreadingFactor() const { return LORA_SF; }
  static uint16_t preambleLengthForSF(uint8_t sf) { return sf <= 8 ? 32 : 16; }
  void updatePreamble(uint8_t sf) { _preamble_sf = sf; _radio->setPreambleLength(preambleLengthForSF(sf)); }

  int getNoiseFloor() const override { return _noise_floor; }
  void triggerNoiseFloorCalibrate(int threshold) override;
  void resetAGC() override;

  void loop() override;

  uint32_t getPacketsRecv() const { return n_recv; }
  uint32_t getPacketsRecvErrors() const { return n_recv_errors; }
  uint32_t getPacketsSent() const { return n_sent; }
  void resetStats() { n_recv = n_sent = n_recv_errors = 0; }

  virtual float getLastRSSI() const override;
  virtual float getLastSNR() const override;

  float packetScore(float snr, int packet_len) override { return packetScoreInt(snr, 10, packet_len); }  // assume sf=10

  virtual void setRxBoostedGainMode(bool) { }
  virtual bool getRxBoostedGainMode() const { return false; }
};

/**
 * \brief  an RNG impl using the noise from the LoRa radio as entropy.
 *         NOTE: this is VERY SLOW!  Use only for things like creating new LocalIdentity
*/
class RadioNoiseListener : public mesh::RNG {
  PhysicalLayer* _radio;
public:
  RadioNoiseListener(PhysicalLayer& radio): _radio(&radio) { }

  void random(uint8_t* dest, size_t sz) override {
    for (int i = 0; i < sz; i++) {
      dest[i] = _radio->randomByte() ^ (::random(0, 256) & 0xFF);
    }
  }
};
