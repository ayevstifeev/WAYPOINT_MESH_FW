#pragma once

#include <Arduino.h>
#include <Mesh.h>
#include "AbstractUITask.h"

/*------------ Frame Protocol --------------*/
#define FIRMWARE_VER_CODE 13

//: **When this image was compiled, not a number someone typed.**
//:
//: It was the fixed string "6 Jun 2026" and never moved, so `ver` answered
//: the same thing on every build ever made. On 2026-10-01 a repeater was
//: reflashed three times against that reply and it said "6 Jun 2026" each
//: time, which is indistinguishable from an upload that silently did
//: nothing - and that is exactly what had happened.
//:
//: One caveat worth knowing: __DATE__/__TIME__ are fixed when this
//: translation unit is compiled, so an incremental build that does not
//: recompile it leaves the old stamp. A changed build flag or `-t clean`
//: recompiles everything, which is when it matters.
#ifndef FIRMWARE_BUILD_DATE
#define FIRMWARE_BUILD_DATE __DATE__ " " __TIME__
#endif

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "v1.16.0"
#endif

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
#include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
#include <LittleFS.h>
#elif defined(ESP32)
#include <SPIFFS.h>
#endif

#include "DataStore.h"
#include "NodePrefs.h"

#include <RTClib.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/BaseSerialInterface.h>
#include <helpers/IdentityStore.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/StaticPoolPacketManager.h>
#include <target.h>

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#ifndef LORA_FREQ
#define LORA_FREQ 915.0
#endif
#ifndef LORA_BW
#define LORA_BW 250
#endif
#ifndef LORA_SF
#define LORA_SF 10
#endif
#ifndef LORA_CR
#define LORA_CR 5
#endif
#ifndef LORA_TX_POWER
#define LORA_TX_POWER 20
#endif
#ifndef MAX_LORA_TX_POWER
#define MAX_LORA_TX_POWER LORA_TX_POWER
#endif

#ifndef MAX_CONTACTS
#define MAX_CONTACTS 100
#endif

#ifndef OFFLINE_QUEUE_SIZE
#define OFFLINE_QUEUE_SIZE 16
#endif

#ifndef BLE_NAME_PREFIX
#define BLE_NAME_PREFIX "MeshCore-"
#endif

#include <helpers/BaseChatMesh.h>
#include <helpers/TransportKeyStore.h>

/* -------------------------------------------------------------------------------------- */

#define REQ_TYPE_GET_STATUS             0x01 // same as _GET_STATS
#define REQ_TYPE_KEEP_ALIVE             0x02
#define REQ_TYPE_GET_TELEMETRY_DATA     0x03

struct AdvertPath {
  uint8_t pubkey_prefix[7];
  uint8_t path_len;
  char    name[32];
  uint32_t recv_timestamp;
  uint8_t path[MAX_PATH_SIZE];
};

#ifdef WITH_TRACKER_MODE
  // It sends positions with the slim codecs - the 6-byte frame, and the "!P"
  // self-report on MeshCore - and a dual build is how it runs on either
  // protocol, so it is only offered where both are compiled.
  #ifndef WITH_MESH_SLIM
    #error "WITH_TRACKER_MODE needs WITH_MESH_SLIM"
  #endif
  #if ENV_INCLUDE_GPS != 1
    #error "WITH_TRACKER_MODE needs a GPS build (ENV_INCLUDE_GPS=1)"
  #endif
  #include "TrackerMode.h"
#endif

#ifdef WITH_MESH_SLIM
#include <helpers/slim/SlimCompanion.h>
#include <helpers/slim/SlimText.h>
#include <helpers/slim/SlimApp.h>
#include <helpers/slim/SlimHealth.h>
#include <helpers/slim/SlimRound.h>
#include <helpers/slim/SlimTrack.h>

class MyMesh : public BaseChatMesh, public DataStoreHost, public slimproto::Handler {
#else
class MyMesh : public BaseChatMesh, public DataStoreHost {
#endif
public:
  MyMesh(mesh::Radio &radio, mesh::RNG &rng, mesh::RTCClock &rtc, SimpleMeshTables &tables, DataStore& store, AbstractUITask* ui=NULL);

#ifdef WITH_MESH_SLIM
  //: What the radio measured about one reception, on either protocol.
  struct Heard {
    uint8_t hops;     //: repeaters in between, as the app reads path_len
    uint8_t via;      //: the unit that handed it over; 0 when unknown
    int8_t  snr_q4;
    int16_t rssi;
  };
  void emitChannelText(const char* text, const Heard& heard);
  //: A slim frame addressed to us, or broadcast. Rendered into the same push
  //: frame the app would have received from MeshCore, so nothing above the
  //: radio can tell which protocol carried it.
  void onSlimRecv(const slim::Frame& f,
                  const slimproto::Link& link) override;
  //: Answer a health check: what this node can measure of itself.
  void sendSlimStatus(uint8_t to, uint16_t want, int ttl = -1,
                      bool prefer_direct = true);

  //: Say who this unit is: callsign and public key, to everyone.
  //:
  //: The one thing a slim network could not express. Without it a unit
  //: has a callsign on the map and no identity behind it, so it cannot
  //: be a contact - and everything that addresses a contact fails,
  //: including its own messages arriving at the base.
  bool sendSlimAdvert();
  //: "!V" from somebody else: learn them. True if it was one.
  bool slimAdvert(const char* msg);
  //: When the next advert goes out, in millis().
  //:
  //: One shortly after boot, because the case this exists for is a unit
  //: just flashed and switched on beside a base that has never heard of
  //: it - nobody should have to know to press a button for that. Then
  //: hourly, because a base started later would otherwise never learn a
  //: unit that adverted once into an empty room.
  unsigned long _slim_advert_at = 0;
  //: The volunteered health reading, on its own clock. Sent back to back
  //: with the advert it contends with this unit's own airtime and is
  //: usually lost; ten seconds apart, both are heard.
  unsigned long _slim_status_at = 0;
  //: One link test: what was asked, and what came back.
  //:
  //: Both directions, because they are not the same link. The base has
  //: the better antenna and more power, so it is routinely heard from
  //: places it cannot hear - and it is the UP figure that decides
  //: whether a searcher appears on the map.
  struct LinkTest {
    uint32_t sent_at = 0;       // 0 = never run
    uint32_t got_at = 0;        // 0 = still waiting, or timed out
    int16_t  up_rssi = 0;       // the base, hearing this unit
    float    up_snr = 0;
    int16_t  dn_rssi = 0;       // this unit, hearing the reply
    float    dn_snr = 0;
    //: How many radio hops the answer crossed.
    //:
    //: The number that decides what a reading means. -97 dBm over one hop
    //: is a link; -97 dBm over two is a repeater doing the work, and the
    //: place being tested has no direct path at all. Siting a second
    //: repeater on the first reading would be siting it for a link that
    //: does not exist.
    uint8_t  hops = 0;
  };
  LinkTest _link_test;

  //: A reply that is owed, and the moment it may go out.
  //:
  //: **A reply built inside the receive callback is transmitted into the
  //: tail of the burst that asked for it, and a half-duplex radio hears
  //: none of it.** Every sender here sends its frame three times back to
  //: back, so by the time copy one has been decoded the asker is still
  //: transmitting copies two and three - deaf, because its own
  //: transmitter is on.
  //:
  //: The repeater found this the hard way: it answered nothing at all,
  //: no health check and no admin reply, until its replies were deferred
  //: by SLIM_REPLY_QUIET_MS. The same afternoon a tracker in clear line
  //: of sight of the base antenna decided it had lost contact and began
  //: keeping a backlog, because every acknowledgement it was owed had
  //: landed inside its own burst. One fault, three symptoms.
  struct Owed {
    uint8_t  to = 0;             // 0: this slot is free
    uint8_t  type = 0;           // TYPE_ACK or TYPE_STATUS
    uint8_t  seq = 0;            // the acknowledgement's one body byte
    uint16_t want = 0;           // which readings a status should carry
    unsigned long at = 0;        // not before this
  };
  //: Four. A base hears four units report inside one burst on a busy
  //: callout; a fifth is rarer than the 32 bytes of carrying it.
  static const uint8_t SLIM_OWED_MAX = 4;
  Owed _owed[SLIM_OWED_MAX];
  //: The earliest the next owed reply may go. Deferring two onto the same
  //: instant is the same collision wearing a different hat.
  unsigned long _owed_next = 0;

  //: Owe a reply instead of sending one. Silently dropped if nothing is
  //: free - the asker retries, which is what it is already built to do.
  void oweSlimReply(uint8_t to, uint8_t type, uint8_t seq, uint16_t want);
  //: Whatever has come due, one per pass. Called from loop().
  void sendOwedSlimReplies();
  //: True while a reply has been promised and not yet sent.
  //:
  //: **A tracker must not go quiet owing somebody an answer.** The reply is
  //: deliberately held back past the asker's burst, and for those seconds it
  //: is in no queue at all - so the sleep test, which asks whether anything
  //: is queued, said yes-sleep and the promise was broken in silence. The
  //: base then retried a command the unit had already carried out, five
  //: times, and recorded it as undelivered. Seen 2026-10-02: "!T180" put
  //: MTRK01 into tracker mode and the base gave up on it.
  bool owedSlimReply() const;

  //: Queue one slim frame `txBurst()` times. Identical copies: the mesh
  //: deduplicates on (SRC, SEQ), so every receiver after the first drops them
  //: for free, and at 13 bytes three copies cost about what one MeshCore
  //: position costs.
  //: `copies` of 0 means "use the configured tx burst". Acknowledgements pass
  //: the multi-ack setting instead, so `set multi.acks` governs them exactly as
  //: it does on MeshCore.
  //: `prefer_direct` false floods even when a route is known — for a
  //: reachability probe, whose whole job is to find out what can be
  //: reached. See MyMesh::sendLinkTest.
  bool sendSlimBurst(uint8_t type, const uint8_t* body, uint8_t body_len,
                     uint8_t dst, uint8_t copies = 0, int ttl = -1,
                     bool prefer_direct = true);

  //: Ask the base how this unit sounds from here. False if it could
  //: not be sent at all.
  bool sendLinkTest();
  const LinkTest& linkTest() const { return _link_test; }
  //: Milliseconds waiting, or 0 when idle or answered.
  uint32_t linkTestWaiting() const;

  //: How long a range test is worth waiting for, on the screen and in the
  //: radio alike. One number, because they are one question: the screen
  //: must not give up before the receiver does, and the receiver must not
  //: stay awake after the screen has.
  //:
  //: Twelve seconds, which is longer than it used to be. The answer is a
  //: burst out, the base's turnaround, and a burst back at SF11 - but the
  //: base no longer answers the instant it decodes the ping. It waits out
  //: the asker's own burst first (SLIM_REPLY_QUIET_MS), and if it owes
  //: somebody else an answer as well, one gap behind that. Eight seconds
  //: was drawn before any of that existed and would now report "no reply"
  //: to a unit whose reply was on its way.
  static const uint32_t LINK_TEST_WAIT_MS = 12000;

  //: A message typed or chosen on this unit's own screen, held until the
  //: base acknowledges it.
  //:
  //: **A message used to be sent once and forgotten.** "Sent" on the
  //: screen meant the frame had been queued, nothing more, and on
  //: 2026-09-29 a unit in a valley sent canned replies all afternoon that
  //: nobody ever saw. Roughly half the frames arriving at the base over
  //: the direct path failed their authentication tag - the network runs
  //: CRC off, so a marginal path delivers corrupt frames rather than
  //: dropping them, and the tag is what catches them. A position survives
  //: that because it is broadcast and another one follows in two minutes.
  //: A message is addressed, goes once, and is gone.
  //:
  //: Worse, it goes over the *worst* path available. `Routes::learn`
  //: ranks by hop count alone, so a barely-decodable one-hop path always
  //: beats a solid two-hop one through the repeater - and an addressed
  //: DIRECT frame names a single carrier, so the repeater never gets a
  //: copy to relay. Hence the second attempt forgets the route: flooding
  //: is how the message finds the path that actually works.
  //:
  //: **A marker is the same story with more at stake.** It is broadcast
  //: rather than addressed - a hazard matters most to the team walking
  //: towards it - so it took the best path available rather than the
  //: worst, but it still went once and was still never confirmed. A pin
  //: that half the team never received is worse than no pin, because
  //: everyone who did receive it assumes everyone else did too.
  struct Outgoing {
    char text[slimtext::MAX_TEXT + 1] = {0};
    uint8_t to = 0;               //: 0 = nothing outstanding
    uint8_t type = 0;             //: TYPE_TEXT, or TYPE_GRPTEXT for a marker
    uint8_t seq = 0;              //: what the acknowledgement will name
    uint8_t tries = 0;
    unsigned long due = 0;        //: when to try again
    bool acked = false;           //: cleared by the next send
    bool lost = false;
    bool marker = false;          //: for the screen to name what was lost
  };
  Outgoing _out;

  //: Send it, and hold it for the ladder. The one path both a message and
  //: a marker take, so neither can quietly stop being retried.
  bool sendSlimOutgoing(const char* text, uint8_t type, uint8_t to,
                        bool is_marker);
  //: Retry whatever is owed if its acknowledgement has not come. Called
  //: from loop().
  void retrySlimOutgoing();

  //: True when what is outstanding is a marker rather than a message.
  bool textIsMarker() const { return _out.marker; }
  //: What became of the last thing sent from this screen, for the
  //: screen to say so: 0 nothing, 1 waiting, 2 delivered, 3 lost.
  uint8_t textState() const;
  //: Which attempt is in flight, for "SENDING 2/3".
  uint8_t textTries() const { return _out.tries; }

  //: True while a message is outstanding and its acknowledgement could
  //: still arrive - the radio must stay listening for it, exactly as it
  //: must for a range test. See linkTestOpen.
  bool textAckOpen() const { return _out.type != 0; }

  //: True while a range test is outstanding and its answer could still
  //: arrive.
  //:
  //: Distinct from linkTestWaiting(), which keeps counting up after the
  //: answer has stopped being possible - the screen needs that to tell
  //: "no reply" from "never run", and the radio must not be held awake
  //: for it.
  bool linkTestOpen() const;

  //: Build the pubkey<->unit map by walking the contacts MeshCore already
  //: holds. The unit number is in the callsign - MTRK07 is tracker 7 - so
  //: there is nothing to import and no new data anywhere. Returns how many
  //: contacts mapped. Cheap enough to re-run whenever contacts change.
  int loadSlimRoster();
#endif
#ifdef WITH_TRACKER_MODE
  //: Every packet off the air passes through here, on either protocol,
  //: before anything asks who it was for. That is the point: a unit that
  //: has lost the base is not waiting for a message, it is waiting for
  //: evidence that anything at all is in range.
  mesh::DispatcherAction onRecvPacket(mesh::Packet* pkt) override;

  //: Something was heard. If this unit had given up, report now rather
  //: than waiting out the rest of the interval.
  //:
  //: `a_way_through` is the caller's judgement of whether what
  //: arrived is evidence that the BASE can be reached, as opposed to
  //: evidence that another handheld is nearby. A position broadcast that
  //: came straight from the unit that made it is the second thing: a unit
  //: broadcasting a position is very likely as lost as this one, and two
  //: of them within earshot would otherwise wake each other for ever,
  //: each reading the other's report as a way home. Anything that
  //: was REPEATED is the first thing - something relayed it, so there is
  //: infrastructure alive out there.
  void trackerHeardSomething(bool a_way_through);

  //: Line the schedule up on this unit's slot: its number from its
  //: callsign, the round from the interval it was given, and the length
  //: of a slot from the base. Called whenever any of the three changes.
  void applyTrackerSlot();

  //: The base's round-ack: the time, the slot length, and whether this
  //: unit was among those heard. One frame doing the work of sixteen.
  void onRoundAck(const slim::Frame& f);

  //: How long a slot is. The compiled-in default until a base says
  //: otherwise, so a unit slots itself from its own GPS clock rather
  //: than waiting for a base that may not be running rounds.
  uint8_t _slot_s = TRACKER_SLOT_S;

  // ---- the base's side of a round ----------------------------------
  //
  // Set by the app with the custom variable `round` as "300,7" - the
  // round in seconds and a slot in seconds. Zero, the default, means this
  // node is not running rounds and acknowledges each report as it arrives,
  // which is right for a handful of units and wrong for sixteen.

  //: One frame at the end of every round: the time, the slot length, and
  //: who was heard. Against sixteen separate acknowledgements, which at
  //: SF12 are 40 seconds of airtime and sixteen more chances to collide.
  void roundLoop();
  bool setRound(const char* spec);

  uint32_t _round_s = 0;        // the round, or 0 for none
  uint8_t _round_slot_s = 0;
  uint16_t _round_heard = 0;    // who has reported since the last ack
  uint32_t _round_ack_at = 0;   // the round whose ack has already gone

#endif

  void begin(bool has_display);
  void startInterface(BaseSerialInterface &serial);
#ifdef MESH_SLIM_BLE_OFF
  //: Remember whether the operator wants Bluetooth. See bleWanted.
  void saveBleWanted(bool on);
  bool bleWanted();
#endif

  const char *getNodeName();
  NodePrefs *getNodePrefs();
  uint32_t getBLEPin();

  void loop();
  void handleCmdFrame(size_t len);
  bool advert();
  void enterCLIRescue();

  int  getRecentlyHeard(AdvertPath dest[], int max_num);

protected:
  float getAirtimeBudgetFactor() const override;
  int getInterferenceThreshold() const override;
  int calcRxDelay(float score, uint32_t air_time) const override;
  uint32_t getRetransmitDelay(const mesh::Packet *packet) override;
  uint32_t getDirectRetransmitDelay(const mesh::Packet *packet) override;
  uint8_t getExtraAckTransmitCount() const override;
  bool filterRecvFloodPacket(mesh::Packet* packet) override;
  bool allowPacketForward(const mesh::Packet* packet) override;

  void sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis);
  void sendFloodScoped(const ContactInfo& recipient, mesh::Packet* pkt, uint32_t delay_millis=0) override;
  void sendFloodScoped(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t delay_millis=0) override;

  void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override;
  bool isAutoAddEnabled() const override;
  bool shouldAutoAddContactType(uint8_t type) const override;
  bool shouldOverwriteWhenFull() const override;
  uint8_t getAutoAddMaxHops() const override;
  void onContactsFull() override;
  void onContactOverwrite(const uint8_t* pub_key) override;
  bool onContactPathRecv(ContactInfo& from, uint8_t* in_path, uint8_t in_path_len, uint8_t* out_path, uint8_t out_path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override;
  void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t* path) override;
  void onContactPathUpdated(const ContactInfo &contact) override;
  ContactInfo* processAck(const uint8_t *data) override;
  void queueMessage(const ContactInfo &from, uint8_t txt_type, mesh::Packet *pkt, uint32_t sender_timestamp,
                    const uint8_t *extra, int extra_len, const char *text);

  void onMessageRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                     const char *text) override;
  void onCommandDataRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                         const char *text) override;
  void onSignedMessageRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                           const uint8_t *sender_prefix, const char *text) override;
  void onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t timestamp,
                            const char *text) override;
  void onChannelDataRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint16_t data_type,
                         const uint8_t *data, size_t data_len) override;

  uint8_t onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp, const uint8_t *data,
                           uint8_t len, uint8_t *reply) override;
  void onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len) override;
  void onControlDataRecv(mesh::Packet *packet) override;
  void onRawDataRecv(mesh::Packet *packet) override;
  void onTraceRecv(mesh::Packet *packet, uint32_t tag, uint32_t auth_code, uint8_t flags,
                   const uint8_t *path_snrs, const uint8_t *path_hashes, uint8_t path_len) override;

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override;
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override;
  void onSendTimeout() override;

  // DataStoreHost methods
  bool onContactLoaded(const ContactInfo& contact) override { return addContact(contact); }
  bool getContactForSave(uint32_t idx, ContactInfo& contact) override { return getContactByIdx(idx, contact); }
  bool onChannelLoaded(uint8_t channel_idx, const ChannelDetails& ch) override { return setChannel(channel_idx, ch); }
  bool getChannelForSave(uint8_t channel_idx, ChannelDetails& ch) override { return getChannel(channel_idx, ch); }

  void clearPendingReqs() {
    pending_login = pending_status = pending_telemetry = pending_discovery = pending_req = 0;
  }

public:
  void savePrefs() { _store->savePrefs(_prefs, sensors.node_lat, sensors.node_lon); }

#if ENV_INCLUDE_GPS == 1
  void applyGpsPrefs() {
    sensors.setSettingValue("gps", _prefs.gps_enabled ? "1" : "0");
    if (_prefs.gps_interval > 0) {
      char interval_str[12];  // Max: 24 hours = 86400 seconds (5 digits + null)
      sprintf(interval_str, "%u", _prefs.gps_interval);
      sensors.setSettingValue("gps_interval", interval_str);
    }
  }
#endif

#ifdef WITH_MESH_SLIM
  //: Which wire format the radio is on, for the menu.
  bool slimActive() const { return slimproto::active(); }
  //: Save the protocol to start on next time - the menu then restarts.
  //: False if this build cannot switch, or slim has no operation key.
  bool saveSlimProto(bool on);

  //: Send a line of text to whoever last sent US one, falling back to
  //: the base. For the unit's own screen: somebody reading a message
  //: on a hill answers the person who sent it, and has no way to pick
  //: a recipient from a joystick.
  //:
  //: A canned phrase from slimtext::CODES costs ONE byte on the air,
  //: which is why the menu is worth having at all - it is not a
  //: shortcut for typing, it is a different order of airtime.
  bool sendSlimText(const char* text);
  //: Who a reply would go to, for the screen to name. 0 if nobody has
  //: sent this unit anything yet.
  uint8_t slimReplyTo() const { return _slim_reply_to; }
#endif

#ifdef WITH_TRACKER_MODE
  //: Enter tracker mode (MIN..MAX seconds between reports), change its
  //: interval, or leave it (0). Saved, so a unit that browns out comes back
  //: reporting. False for an interval out of range, or no GPS to report from.
  bool setTrackerMode(uint32_t interval_s);
  //: Find mode: report as fast as tracker mode goes for `minutes`, then back
  //: to the usual interval by itself. 0 ends it now. Deliberately NOT saved:
  //: a unit that is reset, or one whose parent walked out of range with the
  //: find still running, must come back on the interval its battery was
  //: chosen for.
  bool startFind(uint32_t minutes);
  //: The saved mode. From main.cpp beside applyGpsPrefs(), and for the same
  //: reason: before sensors.begin() there is no GPS to find.
  void loadTrackerMode();
  //: For the menu: the interval (0 is off), what it is doing, and how many
  //: fixes the base is still owed.
  uint32_t trackerInterval() const { return _tracker.interval(); }
  tracker::Phase trackerPhase() const { return _tracker.phase(); }
  uint16_t trackerOwed() const { return _reports.backlog().size(); }
  //: Reports in a row the base did not answer. 0 means in contact.
  uint8_t trackerMissed() const { return _reports.missed(); }
  //: Given up on the base and sniffing for any traffic at all.
  bool trackerSearching() const { return _reports.searching(); }
  //: Minutes of find mode left, or 0. For the menu, and for the tests.
  uint32_t findLeft() const;

  //: Drop a pin where this unit is standing, for the whole team.
  //:
  //: Broadcast rather than sent to the base, because a marker is for
  //: everyone: the hazard matters most to the team walking towards it,
  //: and routing it through a base that may be out of range would mean
  //: the people closest to it are the last to hear.
  //:
  //: False with no position. **A marker with no fix is worse than no
  //: marker** - it lands at the anchor corner, which is a confident
  //: pin on ground nobody has been anywhere near.
  bool placeMarker(uint8_t icon, const char* label);
  //: Where this unit last knew it was. False if it never has.
  bool hereNow(double& lat, double& lon);
#endif

  // To check if there is pending work
  bool hasPendingWork() const;

private:
  void writeOKFrame();
  void writeErrFrame(uint8_t err_code);
  void writeDisabledFrame();
  void writeContactRespFrame(uint8_t code, const ContactInfo &contact);
  void updateContactFromFrame(ContactInfo &contact, uint32_t& last_mod, const uint8_t *frame, int len);
  void addToOfflineQueue(const uint8_t frame[], int len);
  int getFromOfflineQueue(uint8_t frame[]);
  int getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[]) override { 
    return _store->getBlobByKey(key, key_len, dest_buf);
  }
  bool putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], int len) override {
    return _store->putBlobByKey(key, key_len, src_buf, len);
  }

  void checkCLIRescueCmd();
  void checkSerialInterface();
  bool isValidClientRepeatFreq(uint32_t f) const;
#ifdef WITH_MESH_SLIM
  void loadSlimProto();
  //: MTRKnn is unit nn on slim, when the name has a number. See MyMesh.cpp.
  void loadSlimUnit();
  //: "!S1"/"!S0" from the base: save the protocol, restart once acked.
  bool protoCommand(const char* text, bool from_base);
  //: The unit that last sent this one a direct message, so a reply
  //: typed on the screen has somewhere to go. 0 means nobody yet.
  uint8_t _slim_reply_to = 0;
  unsigned long _proto_restart_at = 0;   // restart when the queue is quiet...
  unsigned long _proto_restart_by = 0;   // ...or by then, whatever it is doing
#endif

#ifdef WITH_TRACKER_MODE
  //: One turn of tracker mode. True while the radio is parked, when the rest
  //: of the loop has nothing to do.
  bool trackerLoop();
  //: The half of setTrackerMode that moves the hardware, without saving. Find
  //: mode changes the rate for half an hour; what is saved stays the interval
  //: the owner chose.
  bool applyTrackerInterval(uint32_t interval_s);
  void saveTrackerInterval(uint32_t interval_s);
  //: A command on the family channel: "MCMD: !F30 BELLA". True when it was
  //: for this unit and has been acted on. False for one aimed at somebody
  //: else, which still goes up to the phone - that is how a second parent's
  //: screen knows the first has started looking.
  bool trackerChannelCommand(const char* text);
  //: When find mode ends, in millis(), or 0.
  uint32_t _find_until = 0;
  //: What to go back to then.
  uint32_t _find_usual = 0;
  //: A "!T" command in a direct message. True if it was one, and handled.
  bool trackerCommand(const char* text, bool from_base);
  bool sendTrackerFix(double lat, double lon);

  //: The last few places this unit has been, newest first, taken every
  //: TRACKER_SAMPLE_S and sent together. One transmission, four places.
  //: Zeroed explicitly: _trail[0].fix is what hereNow() reads as "this
  //: unit has had a fix at some point", so it must start false rather
  //: than rely on where the object happens to be allocated.
  slimtrack::Point _trail[slimtrack::SAMPLES] = {};
  uint8_t _trail_n = 0;
  void pushSample(double lat, double lon);

  //: The trail as one frame. False when there is nothing to send or the
  //: node is not on MESH_SLIM.
  bool sendTrackerTrail();

  //: Hand one channel message up to the app, as MeshCore would have.
  //: Split out because a trail arrives as one frame and becomes two
  //: messages - where it is now, and where it has been.
  //: One reception, as the app is told about it: how far it came, who
  //: handed it over, and how well it arrived.
  //:
  //: `via` is the whole reason a path switch is visible at all. MESH_SLIM
  //: carries no accumulating path - every repeater OVERWRITES the HOP byte
  //: with its own id - so one byte says which node this copy came through,
  //: and `via == src` means it came straight from the unit.

  //: A report from a unit that could not get a fix. Six zero bytes, which
  //: is not a place and is not silence either: an operator has to be able
  //: to tell a radio working under a canopy from a team nobody can hear.
  //:
  //: MESH_SLIM only. MeshCore's position formats all carry a coordinate,
  //: and inventing one to mean "none" is how a unit ends up plotted in the
  //: Gulf of Guinea.
  bool sendTrackerNoFix();
  bool hasGps();
  //: The base, if it is in the contacts: the one reports go to, by DM.
  ContactInfo* trackerBase();
  //: The next piece of the backlog, if one is due. See tracker::Reports.
  void trackerHistory();

  //: Tell whoever is carrying this unit whether it is still in touch.
  //: Nothing if there is no buzzer, and nothing while it is muted -
  //: the same triple-click that silences message tones.
  void trackerSound(UIEventType e);
  //: What `Reports::missed()` was when we last looked, so the three
  //: beeps go once per failed report and not once per loop.
  uint8_t _tracker_missed = 0;

  tracker::Schedule _tracker;
  tracker::Reports _reports;
  unsigned long _ack_due = 0;   // listen for an ack until then, at most
  bool _tracker_gps = false;    // the GPS is on because tracker mode wants a fix
  //: The last report had no place in it, so the GPS stays on between
  //: cycles rather than restarting its search every time. See
  //: TRACKER_HUNT_MAX_S.
  bool _gps_hunting = true;
  uint32_t _hunt_from = 0;      // millis() the current hunt window opened
  bool _radio_parked = false;   // the radio is in warm sleep between reports
  long _fix_ts = 0;             // GPS time of the last fix counted
  uint8_t _fixes = 0;           // fresh fixes seen since the GPS came on
  //: This device's own marker counter. Creator plus this number is a
  //: marker's identity, so re-sending the pair moves that pin rather
  //: than adding one - see slimcodec::encodeMarker.
  uint8_t _marker_num = 0;
#endif

  // helpers, short-cuts
  void saveChannels() { _store->saveChannels(this); }
  void saveContacts();

  DataStore* _store;
  NodePrefs _prefs;
  uint32_t pending_login;
  uint32_t pending_status;
  uint32_t pending_telemetry, pending_discovery;   // pending _TELEMETRY_REQ
  uint32_t pending_req;   // pending _BINARY_REQ
  BaseSerialInterface *_serial;
  AbstractUITask* _ui;

  ContactsIterator _iter;
  uint32_t _iter_filter_since;
  uint32_t _most_recent_lastmod;
  uint32_t _active_ble_pin;
  bool _iter_started;
  bool _cli_rescue;
  bool send_unscoped;   // force un-scoped flood (instead of using send_scope)
  char cli_command[80];
  uint8_t app_target_ver;
  uint8_t *sign_data;
  uint32_t sign_data_len;
  unsigned long dirty_contacts_expiry;

  TransportKey send_scope;

  uint8_t cmd_frame[MAX_FRAME_SIZE + 1];
  uint8_t out_frame[MAX_FRAME_SIZE + 1];
  CayenneLPP telemetry;

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];

    bool isChannelMsg() const;
  };
  int offline_queue_len;
  Frame offline_queue[OFFLINE_QUEUE_SIZE];

  struct AckTableEntry {
    unsigned long msg_sent;
    uint32_t ack;
    ContactInfo* contact;
  };
  #define EXPECTED_ACK_TABLE_SIZE 8
  AckTableEntry expected_ack_table[EXPECTED_ACK_TABLE_SIZE]; // circular table
  int next_ack_idx;

  #define ADVERT_PATH_TABLE_SIZE   16
  AdvertPath advert_paths[ADVERT_PATH_TABLE_SIZE]; // circular table
};

extern MyMesh the_mesh;
