#pragma once

#include "CustomSX1262.h"
#include "RadioLibWrappers.h"
#include "SX126xReset.h"

#ifdef WITH_MESH_SLIM
  #include <helpers/slim/SlimPacket.h>
  #ifdef MESH_SLIM_SHORT_PREAMBLE
    #include <helpers/slim/SlimProto.h>
  #endif
#endif

#ifndef USE_SX1262
#define USE_SX1262
#endif

class CustomSX1262Wrapper : public RadioLibWrapper {
public:
  CustomSX1262Wrapper(CustomSX1262& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    ((CustomSX1262 *)_radio)->setFrequency(freq);
    ((CustomSX1262 *)_radio)->setSpreadingFactor(sf);
    ((CustomSX1262 *)_radio)->setBandwidth(bw);
    ((CustomSX1262 *)_radio)->setCodingRate(cr);
    updatePreamble(sf);
#if defined(WITH_MESH_SLIM) && defined(MESH_SLIM_SHORT_PREAMBLE)
    if (slimproto::active()) _radio->setPreambleLength(slim::PREAMBLE_SYMBOLS);
#endif
  }

  bool isReceivingPacket() override { 
    return ((CustomSX1262 *)_radio)->isReceiving();
  }
  float getCurrentRSSI() override {
    return ((CustomSX1262 *)_radio)->getRSSI(false);
  }
  float getLastRSSI() const override { return ((CustomSX1262 *)_radio)->getRSSI(); }
  float getLastSNR() const override { return ((CustomSX1262 *)_radio)->getSNR(); }

  float packetScore(float snr, int packet_len) override {
    int sf = ((CustomSX1262 *)_radio)->spreadingFactor;
    return packetScoreInt(snr, sf, packet_len);
  }
  uint8_t getSpreadingFactor() const override { return ((CustomSX1262 *)_radio)->spreadingFactor; }
  virtual void powerOff() override {
    ((CustomSX1262 *)_radio)->sleep(false);
  }

  void doResetAGC() override { sx126xResetAGC((SX126x *)_radio); }

#ifdef WITH_MESH_SLIM
  // Named `on` rather than `slim` because `slim` is the namespace holding the
  // constants two lines down, and a parameter of that name silently shadows it.
  bool setProtocolMode(bool on) override {
    CustomSX1262* r = (CustomSX1262 *)_radio;
    // Sync word first. Everything after it only matters to packets this radio
    // is now willing to hear, so changing it first means there is no window in
    // which the radio listens to the old network with the new framing.
    r->setSyncWord(on ? slim::SYNC_WORD : RADIOLIB_SX126X_SYNC_WORD_PRIVATE);
    // One preamble on both protocols unless MESH_SLIM_SHORT_PREAMBLE. The
    // short slim one did not hold: setParams() resets the preamble on every
    // radio change, so a unit that booted on slim ran 16 at SF11 while one
    // switched after boot ran 8. One value everywhere, and the one the working
    // MeshCore net already uses against the base's 32.
#ifdef MESH_SLIM_SHORT_PREAMBLE
    r->setPreambleLength(on ? slim::PREAMBLE_SYMBOLS
                            : preambleLengthForSF(getSpreadingFactor()));
#else
    r->setPreambleLength(preambleLengthForSF(getSpreadingFactor()));
#endif
    r->setCRC(on ? 0 : 1);
    return true;
  }
#endif

#ifdef WITH_TRACKER_MODE
  bool startSniffing() override {
    CustomSX1262* r = (CustomSX1262 *)_radio;
    // RadioLib works the two periods out from the preamble every sender
    // here uses, with its own margin for the chip's timing drift, so
    // there is no window a preamble can slip through. Four symbols is
    // enough to lock on to one and keeps the duty cycle near an eighth;
    // fewer starts trading certainty for current, which is the wrong way
    // round for a unit that is already lost.
    const uint16_t preamble = preambleLengthForSF(getSpreadingFactor());
    return r->startReceiveDutyCycleAuto(preamble, 4) == RADIOLIB_ERR_NONE;
  }
#endif

  void setRxBoostedGainMode(bool en) override {
    ((CustomSX1262 *)_radio)->setRxBoostedGainMode(en);
  }
  bool getRxBoostedGainMode() const override {
    return ((CustomSX1262 *)_radio)->getRxBoostedGainMode();
  }
};
