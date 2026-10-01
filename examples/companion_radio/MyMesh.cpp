#include "MyMesh.h"

#include <Arduino.h> // needed for PlatformIO
#include <Mesh.h>

#define CMD_APP_START                 1
#define CMD_SEND_TXT_MSG              2
#define CMD_SEND_CHANNEL_TXT_MSG      3
#define CMD_GET_CONTACTS              4 // with optional 'since' (for efficient sync)
#define CMD_GET_DEVICE_TIME           5
#define CMD_SET_DEVICE_TIME           6
#define CMD_SEND_SELF_ADVERT          7
#define CMD_SET_ADVERT_NAME           8
#define CMD_ADD_UPDATE_CONTACT        9
#define CMD_SYNC_NEXT_MESSAGE         10
#define CMD_SET_RADIO_PARAMS          11
#define CMD_SET_RADIO_TX_POWER        12
#define CMD_RESET_PATH                13
#define CMD_SET_ADVERT_LATLON         14
#define CMD_REMOVE_CONTACT            15
#define CMD_SHARE_CONTACT             16
#define CMD_EXPORT_CONTACT            17
#define CMD_IMPORT_CONTACT            18
#define CMD_REBOOT                    19
#define CMD_GET_BATT_AND_STORAGE      20   // was CMD_GET_BATTERY_VOLTAGE
#define CMD_SET_TUNING_PARAMS         21
#define CMD_DEVICE_QUERY              22
#define CMD_EXPORT_PRIVATE_KEY        23
#define CMD_IMPORT_PRIVATE_KEY        24
#define CMD_SEND_RAW_DATA             25
#define CMD_SEND_LOGIN                26
#define CMD_SEND_STATUS_REQ           27
#define CMD_HAS_CONNECTION            28
#define CMD_LOGOUT                    29 // 'Disconnect'
#define CMD_GET_CONTACT_BY_KEY        30
#define CMD_GET_CHANNEL               31
#define CMD_SET_CHANNEL               32
#define CMD_SIGN_START                33
#define CMD_SIGN_DATA                 34
#define CMD_SIGN_FINISH               35
#define CMD_SEND_TRACE_PATH           36
#define CMD_SET_DEVICE_PIN            37
#define CMD_SET_OTHER_PARAMS          38
#define CMD_SEND_TELEMETRY_REQ        39  // can deprecate this
#define CMD_GET_CUSTOM_VARS           40
#define CMD_SET_CUSTOM_VAR            41
#define CMD_GET_ADVERT_PATH           42
#define CMD_GET_TUNING_PARAMS         43
// NOTE: CMD range 44..49 parked, potentially for WiFi operations
#define CMD_SEND_BINARY_REQ           50
#define CMD_FACTORY_RESET             51
#define CMD_SEND_PATH_DISCOVERY_REQ   52
#define CMD_SET_FLOOD_SCOPE_KEY       54   // v8+
#define CMD_SEND_CONTROL_DATA         55   // v8+
#define CMD_GET_STATS                 56   // v8+, second byte is stats type
#define CMD_SEND_ANON_REQ             57
#define CMD_SET_AUTOADD_CONFIG        58
#define CMD_GET_AUTOADD_CONFIG        59
#define CMD_GET_ALLOWED_REPEAT_FREQ   60
#define CMD_SET_PATH_HASH_MODE        61
#define CMD_SEND_CHANNEL_DATA         62
#define CMD_SET_DEFAULT_FLOOD_SCOPE   63
#define CMD_GET_DEFAULT_FLOOD_SCOPE   64
#define CMD_SEND_RAW_PACKET           65

// Stats sub-types for CMD_GET_STATS
#define STATS_TYPE_CORE               0
#define STATS_TYPE_RADIO              1
#define STATS_TYPE_PACKETS             2

#define RESP_CODE_OK                  0
#define RESP_CODE_ERR                 1
#define RESP_CODE_CONTACTS_START      2  // first reply to CMD_GET_CONTACTS
#define RESP_CODE_CONTACT             3  // multiple of these (after CMD_GET_CONTACTS)
#define RESP_CODE_END_OF_CONTACTS     4  // last reply to CMD_GET_CONTACTS
#define RESP_CODE_SELF_INFO           5  // reply to CMD_APP_START
#define RESP_CODE_SENT                6  // reply to CMD_SEND_TXT_MSG
#define RESP_CODE_CONTACT_MSG_RECV    7  // a reply to CMD_SYNC_NEXT_MESSAGE (ver < 3)
#define RESP_CODE_CHANNEL_MSG_RECV    8  // a reply to CMD_SYNC_NEXT_MESSAGE (ver < 3)
#define RESP_CODE_CURR_TIME           9  // a reply to CMD_GET_DEVICE_TIME
#define RESP_CODE_NO_MORE_MESSAGES    10 // a reply to CMD_SYNC_NEXT_MESSAGE
#define RESP_CODE_EXPORT_CONTACT      11
#define RESP_CODE_BATT_AND_STORAGE    12 // a reply to a CMD_GET_BATT_AND_STORAGE
#define RESP_CODE_DEVICE_INFO         13 // a reply to CMD_DEVICE_QUERY
#define RESP_CODE_PRIVATE_KEY         14 // a reply to CMD_EXPORT_PRIVATE_KEY
#define RESP_CODE_DISABLED            15
#define RESP_CODE_CONTACT_MSG_RECV_V3 16 // a reply to CMD_SYNC_NEXT_MESSAGE (ver >= 3)
#define RESP_CODE_CHANNEL_MSG_RECV_V3 17 // a reply to CMD_SYNC_NEXT_MESSAGE (ver >= 3)
#define RESP_CODE_CHANNEL_INFO        18 // a reply to CMD_GET_CHANNEL
#define RESP_CODE_SIGN_START          19
#define RESP_CODE_SIGNATURE           20
#define RESP_CODE_CUSTOM_VARS         21
#define RESP_CODE_ADVERT_PATH         22
#define RESP_CODE_TUNING_PARAMS       23
#define RESP_CODE_STATS               24   // v8+, second byte is stats type
#define RESP_CODE_AUTOADD_CONFIG      25
#define RESP_ALLOWED_REPEAT_FREQ      26
#define RESP_CODE_CHANNEL_DATA_RECV   27
#define RESP_CODE_DEFAULT_FLOOD_SCOPE 28

#define MAX_CHANNEL_DATA_LENGTH       (MAX_FRAME_SIZE - 9)

#define SEND_TIMEOUT_BASE_MILLIS        500
#define FLOOD_SEND_TIMEOUT_FACTOR       16.0f
#define DIRECT_SEND_PERHOP_FACTOR       6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS 250
#define LAZY_CONTACTS_WRITE_DELAY       5000

#define PUBLIC_GROUP_PSK                "izOH6cXN6mrJ5e26oRXNcg=="

// these are _pushed_ to client app at any time
#define PUSH_CODE_ADVERT                0x80
#define PUSH_CODE_PATH_UPDATED          0x81
#define PUSH_CODE_SEND_CONFIRMED        0x82
#define PUSH_CODE_MSG_WAITING           0x83
#define PUSH_CODE_RAW_DATA              0x84
#define PUSH_CODE_LOGIN_SUCCESS         0x85
#define PUSH_CODE_LOGIN_FAIL            0x86
#define PUSH_CODE_STATUS_RESPONSE       0x87
#define PUSH_CODE_LOG_RX_DATA           0x88
#define PUSH_CODE_TRACE_DATA            0x89
#define PUSH_CODE_NEW_ADVERT            0x8A
#define PUSH_CODE_TELEMETRY_RESPONSE    0x8B
#define PUSH_CODE_BINARY_RESPONSE       0x8C
#define PUSH_CODE_PATH_DISCOVERY_RESPONSE 0x8D
#define PUSH_CODE_CONTROL_DATA          0x8E   // v8+
#define PUSH_CODE_CONTACT_DELETED       0x8F // used to notify client app of deleted contact when overwriting oldest
#define PUSH_CODE_CONTACTS_FULL         0x90 // used to notify client app that contacts storage is full

#define ERR_CODE_UNSUPPORTED_CMD        1
#define ERR_CODE_NOT_FOUND              2
#define ERR_CODE_TABLE_FULL             3
#define ERR_CODE_BAD_STATE              4
#define ERR_CODE_FILE_IO_ERROR          5
#define ERR_CODE_ILLEGAL_ARG            6

#define MAX_SIGN_DATA_LEN               (8 * 1024) // 8K

// Auto-add config bitmask
// Bit 0: If set, overwrite oldest non-favourite contact when contacts file is full
// Bits 1-4: these indicate which contact types to auto-add when manual_contact_mode = 0x01
#define AUTO_ADD_OVERWRITE_OLDEST (1 << 0)  // 0x01 - overwrite oldest non-favourite when full
#define AUTO_ADD_CHAT             (1 << 1)  // 0x02 - auto-add Chat (Companion) (ADV_TYPE_CHAT)
#define AUTO_ADD_REPEATER         (1 << 2)  // 0x04 - auto-add Repeater (ADV_TYPE_REPEATER)
#define AUTO_ADD_ROOM_SERVER      (1 << 3)  // 0x08 - auto-add Room Server (ADV_TYPE_ROOM)
#define AUTO_ADD_SENSOR           (1 << 4)  // 0x10 - auto-add Sensor (ADV_TYPE_SENSOR)

void MyMesh::writeOKFrame() {
  uint8_t buf[1];
  buf[0] = RESP_CODE_OK;
  _serial->writeFrame(buf, 1);
}
void MyMesh::writeErrFrame(uint8_t err_code) {
  uint8_t buf[2];
  buf[0] = RESP_CODE_ERR;
  buf[1] = err_code;
  _serial->writeFrame(buf, 2);
}

void MyMesh::writeDisabledFrame() {
  uint8_t buf[1];
  buf[0] = RESP_CODE_DISABLED;
  _serial->writeFrame(buf, 1);
}

void MyMesh::writeContactRespFrame(uint8_t code, const ContactInfo &contact) {
  int i = 0;
  out_frame[i++] = code;
  memcpy(&out_frame[i], contact.id.pub_key, PUB_KEY_SIZE);
  i += PUB_KEY_SIZE;
  out_frame[i++] = contact.type;
  out_frame[i++] = contact.flags;
  out_frame[i++] = contact.out_path_len;
  memcpy(&out_frame[i], contact.out_path, MAX_PATH_SIZE);
  i += MAX_PATH_SIZE;
  StrHelper::strzcpy((char *)&out_frame[i], contact.name, 32);
  i += 32;
  memcpy(&out_frame[i], &contact.last_advert_timestamp, 4);
  i += 4;
  memcpy(&out_frame[i], &contact.gps_lat, 4);
  i += 4;
  memcpy(&out_frame[i], &contact.gps_lon, 4);
  i += 4;
  memcpy(&out_frame[i], &contact.lastmod, 4);
  i += 4;
  _serial->writeFrame(out_frame, i);
}

void MyMesh::updateContactFromFrame(ContactInfo &contact, uint32_t& last_mod, const uint8_t *frame, int len) {
  int i = 0;
  uint8_t code = frame[i++]; // eg. CMD_ADD_UPDATE_CONTACT
  memcpy(contact.id.pub_key, &frame[i], PUB_KEY_SIZE);
  i += PUB_KEY_SIZE;
  contact.type = frame[i++];
  contact.flags = frame[i++];
  contact.out_path_len = frame[i++];
  memcpy(contact.out_path, &frame[i], MAX_PATH_SIZE);
  i += MAX_PATH_SIZE;
  memcpy(contact.name, &frame[i], 32);
  i += 32;
  memcpy(&contact.last_advert_timestamp, &frame[i], 4);
  i += 4;
  if (len >= i + 8) { // optional fields
    memcpy(&contact.gps_lat, &frame[i], 4);
    i += 4;
    memcpy(&contact.gps_lon, &frame[i], 4);
    i += 4;
    if (len >= i + 4) {
      memcpy(&last_mod, &frame[i], 4);
    }
  }
}

bool MyMesh::Frame::isChannelMsg() const {
  return buf[0] == RESP_CODE_CHANNEL_MSG_RECV || buf[0] == RESP_CODE_CHANNEL_MSG_RECV_V3 ||
         buf[0] == RESP_CODE_CHANNEL_DATA_RECV;
}

void MyMesh::addToOfflineQueue(const uint8_t frame[], int len) {
  if (offline_queue_len >= OFFLINE_QUEUE_SIZE) {
    MESH_DEBUG_PRINTLN("WARN: offline_queue is full!");
    int pos = 0;
    while (pos < offline_queue_len) {
      if (offline_queue[pos].isChannelMsg()) {
        for (int i = pos; i < offline_queue_len - 1; i++) { // delete oldest channel msg from queue
          offline_queue[i] = offline_queue[i + 1];
        }
        MESH_DEBUG_PRINTLN("INFO: removed oldest channel message from queue.");
        offline_queue[offline_queue_len - 1].len = len;
        memcpy(offline_queue[offline_queue_len - 1].buf, frame, len);
        return;
      }
      pos++;
    }
    MESH_DEBUG_PRINTLN("INFO: no channel messages to remove from queue.");
  } else {
    offline_queue[offline_queue_len].len = len;
    memcpy(offline_queue[offline_queue_len].buf, frame, len);
    offline_queue_len++;
  }
}

int MyMesh::getFromOfflineQueue(uint8_t frame[]) {
  if (offline_queue_len > 0) {         // check offline queue
    size_t len = offline_queue[0].len; // take from top of queue
    memcpy(frame, offline_queue[0].buf, len);

    offline_queue_len--;
    for (int i = 0; i < offline_queue_len; i++) { // delete top item from queue
      offline_queue[i] = offline_queue[i + 1];
    }
    return len;
  }
  return 0; // queue is empty
}

float MyMesh::getAirtimeBudgetFactor() const {
  return _prefs.airtime_factor;
}

int MyMesh::getInterferenceThreshold() const {
  return 0; // disabled for now, until currentRSSI() problem is resolved
}

int MyMesh::calcRxDelay(float score, uint32_t air_time) const {
  if (_prefs.rx_delay_base <= 0.0f) return 0;
  return (int)((pow(_prefs.rx_delay_base, 0.85f - score) - 1.0) * air_time);
}

uint32_t MyMesh::getRetransmitDelay(const mesh::Packet *packet) {
  uint32_t t = (_radio->getEstAirtimeFor(packet->getPathByteLen() + packet->payload_len + 2) * 0.5f);
  return getRNG()->nextInt(0, 5*t + 1);
}
uint32_t MyMesh::getDirectRetransmitDelay(const mesh::Packet *packet) {
  uint32_t t = (_radio->getEstAirtimeFor(packet->getPathByteLen() + packet->payload_len + 2) * 0.2f);
  return getRNG()->nextInt(0, 5*t + 1);
}

uint8_t MyMesh::getExtraAckTransmitCount() const {
  return _prefs.multi_acks;
}

void MyMesh::logRxRaw(float snr, float rssi, const uint8_t raw[], int len) {
  if (_serial->isConnected() && len + 3 <= MAX_FRAME_SIZE) {
    int i = 0;
    out_frame[i++] = PUSH_CODE_LOG_RX_DATA;
    out_frame[i++] = (int8_t)(snr * 4);
    out_frame[i++] = (int8_t)(rssi);
    memcpy(&out_frame[i], raw, len);
    i += len;

    _serial->writeFrame(out_frame, i);
  }
}

bool MyMesh::isAutoAddEnabled() const {
  return (_prefs.manual_add_contacts & 1) == 0;
}

bool MyMesh::shouldAutoAddContactType(uint8_t contact_type) const {
  if ((_prefs.manual_add_contacts & 1) == 0) {
    return true;
  }

  uint8_t type_bit = 0;
  switch (contact_type) {
    case ADV_TYPE_CHAT:
      type_bit = AUTO_ADD_CHAT;
      break;
    case ADV_TYPE_REPEATER:
      type_bit = AUTO_ADD_REPEATER;
      break;
    case ADV_TYPE_ROOM:
      type_bit = AUTO_ADD_ROOM_SERVER;
      break;
    case ADV_TYPE_SENSOR:
      type_bit = AUTO_ADD_SENSOR;
      break;
    default:
      return false;  // Unknown type, don't auto-add
  }

  return (_prefs.autoadd_config & type_bit) != 0;
}

bool MyMesh::shouldOverwriteWhenFull() const {
  return (_prefs.autoadd_config & AUTO_ADD_OVERWRITE_OLDEST) != 0;
}

uint8_t MyMesh::getAutoAddMaxHops() const {
  return _prefs.autoadd_max_hops;
}

void MyMesh::onContactOverwrite(const uint8_t* pub_key) {
    _store->deleteBlobByKey(pub_key, PUB_KEY_SIZE); // delete from storage
  if (_serial->isConnected()) {
    out_frame[0] = PUSH_CODE_CONTACT_DELETED;
    memcpy(&out_frame[1], pub_key, PUB_KEY_SIZE);
    _serial->writeFrame(out_frame, 1 + PUB_KEY_SIZE);
  }
}

void MyMesh::onContactsFull() {
  if (_serial->isConnected()) {
    out_frame[0] = PUSH_CODE_CONTACTS_FULL;
    _serial->writeFrame(out_frame, 1);
  }
}

void MyMesh::onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t* path) {
#ifdef WITH_MESH_SLIM
  // A contact learned or renamed changes the unit map, and a stale map is how a
  // direct message ends up addressed to whoever held that number before.
  {
    uint8_t role, unit;
    if (slimcompanion::unitFromCallsign(contact.name, role, unit)) {
      slimproto::mapUnit(unit, contact.id.pub_key);
      slimcompanion::setUnitRole(unit, role);
    }
  }
#endif
  if (_serial->isConnected()) {
    if (is_new) {
      writeContactRespFrame(PUSH_CODE_NEW_ADVERT, contact);
    } else {
      out_frame[0] = PUSH_CODE_ADVERT;
      memcpy(&out_frame[1], contact.id.pub_key, PUB_KEY_SIZE);
      _serial->writeFrame(out_frame, 1 + PUB_KEY_SIZE);
    }
  } else {
#ifdef DISPLAY_CLASS
    if (_ui) _ui->notify(UIEventType::newContactMessage);
#endif
  }

  // add inbound-path to mem cache
  if (path && mesh::Packet::isValidPathLen(path_len)) {  // check path is valid
    AdvertPath* p = advert_paths;
    uint32_t oldest = 0xFFFFFFFF;
    for (int i = 0; i < ADVERT_PATH_TABLE_SIZE; i++) {   // check if already in table, otherwise evict oldest
      if (memcmp(advert_paths[i].pubkey_prefix, contact.id.pub_key, sizeof(AdvertPath::pubkey_prefix)) == 0) {
        p = &advert_paths[i];   // found
        break;
      }
      if (advert_paths[i].recv_timestamp < oldest) {
        oldest = advert_paths[i].recv_timestamp;
        p = &advert_paths[i];
      }
    }

    memcpy(p->pubkey_prefix, contact.id.pub_key, sizeof(p->pubkey_prefix));
    strcpy(p->name, contact.name);
    p->recv_timestamp = getRTCClock()->getCurrentTime();
    p->path_len = mesh::Packet::copyPath(p->path, path, path_len);
  }

  if (!is_new) dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY); // only schedule lazy write for contacts that are in contacts[]
}

static int sort_by_recent(const void *a, const void *b) {
  return ((AdvertPath *) b)->recv_timestamp - ((AdvertPath *) a)->recv_timestamp;
}

int MyMesh::getRecentlyHeard(AdvertPath dest[], int max_num) {
  if (max_num > ADVERT_PATH_TABLE_SIZE) max_num = ADVERT_PATH_TABLE_SIZE;
  qsort(advert_paths, ADVERT_PATH_TABLE_SIZE, sizeof(advert_paths[0]), sort_by_recent);

  for (int i = 0; i < max_num; i++) {
    dest[i] = advert_paths[i];
  }
  return max_num;
}

void MyMesh::onContactPathUpdated(const ContactInfo &contact) {
  out_frame[0] = PUSH_CODE_PATH_UPDATED;
  memcpy(&out_frame[1], contact.id.pub_key, PUB_KEY_SIZE);
  _serial->writeFrame(out_frame, 1 + PUB_KEY_SIZE); // NOTE: app may not be connected

  dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
}

ContactInfo*  MyMesh::processAck(const uint8_t *data) {
#ifdef WITH_TRACKER_MODE
  {
    // A tracker report or a piece of its backlog, acknowledged by the base.
    // Either way the base can hear us, so whatever of the backlog is due
    // goes now, while the radio is still listening for the reply.
    uint32_t ack;
    memcpy(&ack, data, 4);
    const tracker::Reports::Ack got = _reports.acked(ack);
    if (got != tracker::Reports::NOT_OURS) {
      _ack_due = 0;
      // The report, not each piece of the backlog behind it: a gap
      // being handed over is a dozen acks in a row, and would be a
      // dozen beeps for one piece of news.
      if (got == tracker::Reports::REPORT)
        trackerSound(UIEventType::trackerHeard);
      trackerHistory();
      return trackerBase();   // who it came from: MeshCore stops resending
    }
  }
#endif
  // see if matches any in a table
  for (int i = 0; i < EXPECTED_ACK_TABLE_SIZE; i++) {
    if (memcmp(data, &expected_ack_table[i].ack, 4) == 0) { // got an ACK from recipient
      out_frame[0] = PUSH_CODE_SEND_CONFIRMED;
      memcpy(&out_frame[1], data, 4);
      uint32_t trip_time = _ms->getMillis() - expected_ack_table[i].msg_sent;
      memcpy(&out_frame[5], &trip_time, 4);
      _serial->writeFrame(out_frame, 9);

      // NOTE: the same ACK can be received multiple times!
      expected_ack_table[i].ack = 0; // clear expected hash, now that we have received ACK
      return expected_ack_table[i].contact;
    }
  }
  return checkConnectionsAck(data);
}

void MyMesh::queueMessage(const ContactInfo &from, uint8_t txt_type, mesh::Packet *pkt,
                          uint32_t sender_timestamp, const uint8_t *extra, int extra_len, const char *text) {
  int i = 0;
  if (app_target_ver >= 3) {
    out_frame[i++] = RESP_CODE_CONTACT_MSG_RECV_V3;
    out_frame[i++] = (int8_t)(pkt->getSNR() * 4);
    out_frame[i++] = 0; // reserved1
    out_frame[i++] = 0; // reserved2
  } else {
    out_frame[i++] = RESP_CODE_CONTACT_MSG_RECV;
  }
  memcpy(&out_frame[i], from.id.pub_key, 6);
  i += 6; // just 6-byte prefix
  uint8_t path_len = out_frame[i++] = pkt->isRouteFlood() ? pkt->path_len : 0xFF;
  out_frame[i++] = txt_type;
  memcpy(&out_frame[i], &sender_timestamp, 4);
  i += 4;
  if (extra_len > 0) {
    memcpy(&out_frame[i], extra, extra_len);
    i += extra_len;
  }
  int tlen = strlen(text); // TODO: UTF-8 ??
  if (i + tlen > MAX_FRAME_SIZE) {
    tlen = MAX_FRAME_SIZE - i;
  }
  memcpy(&out_frame[i], text, tlen);
  i += tlen;
  addToOfflineQueue(out_frame, i);

  if (_serial->isConnected()) {
    uint8_t frame[1];
    frame[0] = PUSH_CODE_MSG_WAITING; // send push 'tickle'
    _serial->writeFrame(frame, 1);
  }

#ifdef DISPLAY_CLASS
  // we only want to show text messages on display, not cli data
  bool should_display = txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_SIGNED_PLAIN;
  if (should_display && _ui) {
    _ui->newMsg(path_len, from.name, text, offline_queue_len);
    if (!_serial->isConnected()) {
      _ui->notify(UIEventType::contactMessage);
    }
  }
#endif
}

bool MyMesh::filterRecvFloodPacket(mesh::Packet* packet) {
  // REVISIT: try to determine which Region (from transport_codes[1]) that Sender is indicating for replies/responses
  //    if unknown, fallback to finding Region from transport_codes[0], the 'scope' used by Sender
  return false;
}

bool MyMesh::allowPacketForward(const mesh::Packet* packet) {
  return _prefs.client_repeat != 0;
}

void MyMesh::sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis) {
  if (scope.isNull()) {
    sendFlood(pkt, delay_millis, _prefs.path_hash_mode + 1);
  } else {
    uint16_t codes[2];
    codes[0] = scope.calcTransportCode(pkt);
    codes[1] = 0;  // REVISIT: set to 'home' Region, for sender/return region?
    sendFlood(pkt, codes, delay_millis, _prefs.path_hash_mode + 1);
  }
}

void MyMesh::sendFloodScoped(const ContactInfo& recipient, mesh::Packet* pkt, uint32_t delay_millis) {
  // TODO: dynamic send_scope, depending on recipient and current 'home' Region
  if (send_unscoped) {
    sendFlood(pkt, delay_millis, _prefs.path_hash_mode + 1);  // app has explicitly requested un-scoped
  } else {
    TransportKey default_scope;
    memcpy(&default_scope.key, _prefs.default_scope_key, sizeof(default_scope.key));

    auto scope = send_scope.isNull() ? &default_scope : &send_scope;
    sendFloodScoped(*scope, pkt, delay_millis);
  }
}
void MyMesh::sendFloodScoped(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t delay_millis) {
  // TODO: have per-channel send_scope
  if (send_unscoped) {
    sendFlood(pkt, delay_millis, _prefs.path_hash_mode + 1);  // app has explicitly requested un-scoped
  } else {
    TransportKey default_scope;
    memcpy(&default_scope.key, _prefs.default_scope_key, sizeof(default_scope.key));

    auto scope = send_scope.isNull() ? &default_scope : &send_scope;
    sendFloodScoped(*scope, pkt, delay_millis);
  }
}

#ifdef WITH_MESH_SLIM
//: Only the base may put a unit to sleep, wake it, or move it to the other
//: protocol. "!T0" typed by a teammate should arrive as a message, not switch
//: somebody's tracker off.
static bool isBaseName(const char* name) {
  uint8_t role, number;
  return slimcompanion::unitFromCallsign(name, role, number)
         && role == slimcodec::ROLE_BASE;
}
#endif

void MyMesh::onMessageRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                           const char *text) {
  markConnectionActive(from); // in case this is from a server, and we have a connection
#ifdef WITH_MESH_SLIM
  // The ack is queued as this returns, and it is the base's confirmation.
  // Not queued as a message: there is no phone to show it to.
  const bool from_base = isBaseName(from.name);
  if (protoCommand(text, from_base)) return;
#ifdef WITH_TRACKER_MODE
  //: ...or a parent's phone node in the family finder, which has no base at
  //: all. Only tracker mode: moving a unit to the other protocol stays the
  //: base's, because a family finder never changes protocol.
  if (trackerCommand(text, from_base || tracker::isParentName(from.name))) return;
#endif
#endif
  queueMessage(from, TXT_TYPE_PLAIN, pkt, sender_timestamp, NULL, 0, text);
}

void MyMesh::onCommandDataRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                               const char *text) {
  markConnectionActive(from); // in case this is from a server, and we have a connection
  queueMessage(from, TXT_TYPE_CLI_DATA, pkt, sender_timestamp, NULL, 0, text);
}

void MyMesh::onSignedMessageRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                                 const uint8_t *sender_prefix, const char *text) {
  markConnectionActive(from);
  // from.sync_since change needs to be persisted
  dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
  queueMessage(from, TXT_TYPE_SIGNED_PLAIN, pkt, sender_timestamp, sender_prefix, 4, text);
}

void MyMesh::onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t timestamp,
                                  const char *text) {
#ifdef WITH_TRACKER_MODE
  // A command for this unit is acted on and goes no further: there may be no
  // phone attached at all, and it is not conversation.
  if (trackerChannelCommand(text)) return;
#endif
  int i = 0;
  if (app_target_ver >= 3) {
    out_frame[i++] = RESP_CODE_CHANNEL_MSG_RECV_V3;
    out_frame[i++] = (int8_t)(pkt->getSNR() * 4);
    out_frame[i++] = 0; // reserved1
    out_frame[i++] = 0; // reserved2
  } else {
    out_frame[i++] = RESP_CODE_CHANNEL_MSG_RECV;
  }

  uint8_t channel_idx = findChannelIdx(channel);
  out_frame[i++] = channel_idx;
  uint8_t path_len = out_frame[i++] = pkt->isRouteFlood() ? pkt->path_len : 0xFF;

  out_frame[i++] = TXT_TYPE_PLAIN;
  memcpy(&out_frame[i], &timestamp, 4);
  i += 4;
  int tlen = strlen(text); // TODO: UTF-8 ??
  if (i + tlen > MAX_FRAME_SIZE) {
    tlen = MAX_FRAME_SIZE - i;
  }
  memcpy(&out_frame[i], text, tlen);
  i += tlen;
  addToOfflineQueue(out_frame, i);

  if (_serial->isConnected()) {
    uint8_t frame[1];
    frame[0] = PUSH_CODE_MSG_WAITING; // send push 'tickle'
    _serial->writeFrame(frame, 1);
  } else {
#ifdef DISPLAY_CLASS
    if (_ui) _ui->notify(UIEventType::channelMessage);
#endif
  }
#ifdef DISPLAY_CLASS
  // Get the channel name from the channel index
  const char *channel_name = "Unknown";
  ChannelDetails channel_details;
  if (getChannel(channel_idx, channel_details)) {
    channel_name = channel_details.name;
  }
  if (_ui) _ui->newMsg(path_len, channel_name, text, offline_queue_len);
#endif
}

#ifdef WITH_MESH_SLIM

int MyMesh::loadSlimRoster() {
  slimproto::clearMap();
  int mapped = 0;
  const int n = getNumContacts();
  for (int i = 0; i < n; i++) {
    ContactInfo c;
    if (!getContactByIdx((uint32_t)i, c)) continue;
    uint8_t role, unit;
    // A contact whose name does not follow the scheme is simply not mappable,
    // which is the same rule teammap applies when deciding who can be encoded.
    // By that point it has no callsign on the map either.
    if (!slimcompanion::unitFromCallsign(c.name, role, unit)) continue;
    // The base's fixed names (MSHCMD, MESHGW) carry no number, which reads as
    // 0 - the broadcast address. Mapped there, a message to the base went to
    // every unit on the net and every one of them acked it. The base is unit
    // 15 by convention (MESH_SLIM_UNIT on the Pi), as onSlimRecv assumes.
    if (role == slimcodec::ROLE_BASE && unit == slim::ADDR_BROADCAST) {
      unit = slim::ADDR_BASE;
    }
    if (slimproto::mapUnit(unit, c.id.pub_key)) {
      slimcompanion::setUnitRole(unit, role);
      mapped++;
    }
  }
  return mapped;
}

//: How long a reply waits before it goes on the air. See MyMesh::Owed:
//: a reply sent from inside the receive callback lands in the tail of
//: the burst that asked for it and is never heard. The repeater's
//: constant of the same name is this same number - one fact about a
//: half-duplex radio, not two settings - and both are comfortably
//: inside a tracker's ten-second listening window (TRACKER_LISTEN_S),
//: which is what a deferred reply has to arrive within.
static const unsigned long SLIM_REPLY_QUIET_MS = 2500;
//: How long to wait for a message's acknowledgement, and how many times
//: to try. Eight seconds is a burst out, the base's own reply delay
//: (SLIM_REPLY_QUIET_MS) and a burst back with a hop each way; three
//: attempts across half a minute is as long as anyone will stand holding
//: a handheld waiting to see whether what they said arrived.
static const unsigned long SLIM_TEXT_ACK_MS = 8000;
static const uint8_t SLIM_TEXT_TRIES = 3;
//: And how far apart two owed replies go out.
static const unsigned long SLIM_REPLY_GAP_MS = 2000;

bool MyMesh::sendSlimBurst(uint8_t type, const uint8_t* body, uint8_t body_len,
                           uint8_t dst, uint8_t copies, int ttl,
                           bool prefer_direct) {
  // Build once, then copy. The frame must be byte-identical across the burst -
  // same SEQ, same tag - or receivers treat each copy as a new packet and the
  // whole point is lost.
  mesh::Packet* first = obtainNewPacket();
  if (first == NULL) return false;
  const int n = slimproto::buildOutbound(first, type, body, body_len, dst,
                                         _ms->getMillis(), ttl,
                                         prefer_direct);
  if (n < 0) { releasePacket(first); return false; }

  if (copies == 0) copies = slimproto::txBurst();
  for (uint8_t c = 1; c < copies; c++) {
    mesh::Packet* dup = obtainNewPacket();
    if (dup == NULL) break;         // pool exhausted: send what we have
    memcpy(dup->payload, first->payload, first->payload_len);
    dup->payload_len = first->payload_len;
    dup->header = first->header;
    dup->path_len = 0;
    // Back to back rather than spaced: the failure being defended against is a
    // momentary fade or a collision, and both are over long before a retry
    // timer would fire.
    sendPacket(dup, 1, 0);
  }
  sendPacket(first, 1, 0);
  return true;
}

void MyMesh::oweSlimReply(uint8_t to, uint8_t type, uint8_t seq,
                          uint16_t want) {
  if (!to) return;
  for (uint8_t i = 0; i < SLIM_OWED_MAX; i++) {
    if (_owed[i].to) continue;
    const unsigned long now = _ms->getMillis();
    unsigned long at = now + SLIM_REPLY_QUIET_MS;
    // Behind whatever is already owed, so two replies do not go out on
    // the same instant and collide with each other instead.
    if (_owed_next > now && _owed_next + SLIM_REPLY_GAP_MS > at) {
      at = _owed_next + SLIM_REPLY_GAP_MS;
    }
    _owed_next = at;
    _owed[i].to = to;
    _owed[i].type = type;
    _owed[i].seq = seq;
    _owed[i].want = want;
    _owed[i].at = at;
    return;
  }
  // Nothing free: dropped rather than sent now. Sending now is the fault
  // this exists to avoid, and the asker retries.
}

void MyMesh::sendOwedSlimReplies() {
  if (!slimproto::active()) return;
  for (uint8_t i = 0; i < SLIM_OWED_MAX; i++) {
    if (!_owed[i].to || !millisHasNowPassed(_owed[i].at)) continue;
    const Owed owed = _owed[i];
    _owed[i].to = 0;                        // owed once, whatever happens
    if (owed.type == slim::TYPE_ACK) {
      sendSlimBurst(slim::TYPE_ACK, &owed.seq, 1, owed.to,
                    (uint8_t)(1 + getExtraAckTransmitCount()), 2);
    } else {
      // -1, not a cap. A STATUS teaches routes, so the router refuses to
      // FLOOD one at anything else and throws it away without a word -
      // and a flood is exactly what this falls back to when the asker's
      // own frame arrived DIRECT, which teaches us no route to answer
      // along. That is the unit that has just walked back into range.
      //
      // Flooded deliberately: the answer to "can you hear me" has to come
      // back by whatever path works, not by the one this node happens to
      // have stored for the asker.
      sendSlimStatus(owed.to, owed.want, -1, /*prefer_direct=*/false);
    }
    return;                                 // one per pass; they are spaced
  }
}

//: A health check, answered from whatever this node can measure. A
//: tracker's battery is the one that matters: a position frame carries no
//: quality byte precisely because battery and fix type ride here instead,
//: where they are read occasionally rather than paid for continuously.
void MyMesh::sendSlimStatus(uint8_t to, uint16_t want, int ttl,
                            bool prefer_direct) {
  slimhealth::Status st;
  if (want & (1u << slimhealth::F_BATT_MV)) {
    st.set(slimhealth::F_BATT_MV, board.getBattMilliVolts());
  }
  if (want & (1u << slimhealth::F_UPTIME_MIN)) {
    st.set(slimhealth::F_UPTIME_MIN, (_ms->getMillis() / 1000) / 60);
  }
  if (want & (1u << slimhealth::F_RSSI)) {
    st.set(slimhealth::F_RSSI, radio_driver.getLastRSSI());
  }
  if (want & (1u << slimhealth::F_SNR)) {
    st.set(slimhealth::F_SNR, radio_driver.getLastSNR());
  }
  // Where this unit is in its relationship with the sky. A dot that has
  // stopped is a team resting or a team under canopy whose position is
  // already stale, and those need different decisions.
  if (want & (1u << slimhealth::F_FIX)) {
    LocationProvider* loc = sensors.getLocationProvider();
    st.set(slimhealth::F_FIX, (loc != NULL && loc->isValid()) ? 3 : 0);
  }
  if (want & (1u << slimhealth::F_SATS)) {
    LocationProvider* loc = sensors.getLocationProvider();
    const long n_sats = (loc != NULL) ? loc->satellitesCount() : 0;
    st.set(slimhealth::F_SATS, n_sats < 0 ? 0 : (n_sats > 31 ? 31 : n_sats));
  }
  if (want & (1u << slimhealth::F_FLAGS)) {
    uint8_t flags = 0;
    if (slimhealth::batteryPercent(board.getBattMilliVolts()) < 15.0) {
      flags |= slimhealth::FLAG_LOW;
    }
    st.set(slimhealth::F_FLAGS, flags);
  }
  uint8_t body[slimhealth::MAX_BODY];
  const int n = slimhealth::encode(st, body, sizeof(body));
  if (n < 0) return;
  sendSlimBurst(slim::TYPE_STATUS, body, (uint8_t)n, to, 1, ttl,
                prefer_direct);
}

//: Everything a handheld can say about itself without inventing any of
//: it. Used when nobody asked, so there is no request to read the wanted
//: fields out of.
static const uint16_t SLIM_STATUS_ALL =
  (1u << slimhealth::F_BATT_MV) | (1u << slimhealth::F_UPTIME_MIN)
  | (1u << slimhealth::F_RSSI) | (1u << slimhealth::F_SNR)
  | (1u << slimhealth::F_FIX) | (1u << slimhealth::F_SATS)
  | (1u << slimhealth::F_FLAGS);

//: Ask the base how this unit sounds from where it is standing.
//:
//: A TYPE_PING for RSSI and SNR - the same exchange the base uses to
//: check a repeater, pointed the other way. Addressed to the base and
//: not broadcast: the question is about one link, and a broadcast would
//: be answered by every unit in earshot with its own unrelated numbers.
bool MyMesh::sendLinkTest() {
  if (!slimproto::active()) return false;
  uint8_t body[8];
  const int n = slimhealth::request(
      (1u << slimhealth::F_RSSI) | (1u << slimhealth::F_SNR),
      body, sizeof(body));
  if (n < 0) return false;
  _link_test = LinkTest();
  _link_test.sent_at = _ms->getMillis();
  // Flooded, not routed. A stored route names one carrier, and a carrier
  // that cannot relay it drops it where it stands - so the screen reports
  // "no reply", which reads as "the base cannot hear me from here" when
  // what happened is that one neighbour did not pass it on. That is the
  // worst answer an instrument can give somebody deciding where to put a
  // repeater. The base's request_status floods for the same reason.
  if (!sendSlimBurst(slim::TYPE_PING, body, (uint8_t)n, slim::ADDR_BASE,
                     0, -1, /*prefer_direct=*/false)) {
    _link_test.sent_at = 0;
    return false;
  }
  return true;
}

//: How long the current test has been unanswered. Zero once it is.
uint32_t MyMesh::linkTestWaiting() const {
  if (!_link_test.sent_at || _link_test.got_at) return 0;
  return _ms->getMillis() - _link_test.sent_at;
}

bool MyMesh::linkTestOpen() const {
  const uint32_t waiting = linkTestWaiting();
  return waiting != 0 && waiting < LINK_TEST_WAIT_MS;
}

//: A line of text from this unit's own screen.
//:
//: Addressed to whoever last sent US a message, or to the base if
//: nobody has. A handheld driven by a joystick cannot offer a contact
//: list worth scrolling, and those are the only two recipients anybody
//: actually wants: the person being answered, or the people running it.
bool MyMesh::sendSlimText(const char* text) {
  if (!slimproto::active() || text == NULL || *text == 0) return false;
  uint8_t body[slim::MAX_BODY];
  const int n = slimapp::bodyFor(text, body, sizeof(body));
  if (n < 0) return false;
  const uint8_t to = _slim_reply_to ? _slim_reply_to : slim::ADDR_BASE;
  return sendSlimOutgoing(text, slim::TYPE_TEXT, to, false);
}

bool MyMesh::sendSlimOutgoing(const char* text, uint8_t type, uint8_t to,
                              bool is_marker) {
  uint8_t body[slim::MAX_BODY];
  const int n = slimapp::bodyFor(text, body, sizeof(body));
  if (n < 0) return false;
  // Taken before the send, because the acknowledgement names it. peek, not
  // a read-back: the frame is built inside sendSlimBurst.
  const uint8_t seq = (uint8_t)(slimproto::router().peekNextSeq() & 0xFF);
  if (!sendSlimBurst(type, body, (uint8_t)n, to)) return false;
  // Held for the retry ladder. See MyMesh::Outgoing: one attempt is a coin
  // toss over a marginal path, and the screen used to call it "Sent".
  _out = Outgoing();
  strncpy(_out.text, text, sizeof(_out.text) - 1);
  _out.to = to;
  _out.type = type;
  _out.seq = seq;
  _out.tries = 1;
  _out.marker = is_marker;
  _out.due = futureMillis(SLIM_TEXT_ACK_MS);
  return true;
}

uint8_t MyMesh::textState() const {
  if (_out.type) return 1;                  // waiting
  if (_out.acked) return 2;
  if (_out.lost) return 3;
  return 0;
}

void MyMesh::retrySlimOutgoing() {
  if (!_out.type || !millisHasNowPassed(_out.due)) return;
  if (_out.tries >= SLIM_TEXT_TRIES) {
    _out.type = 0;
    _out.lost = true;
    if (_ui) _ui->notify(UIEventType::trackerLost);
    return;
  }
  // Throw the route away first. The one the router prefers is the one with
  // fewest hops, which on this network is routinely a barely-decodable
  // direct path - and an addressed DIRECT frame names a single carrier, so
  // the repeater never gets a copy. Flooding is how it finds the way that
  // works. See MyMesh::Outgoing.
  // A broadcast already floods, so there is no route to throw away and
  // the repeat is the whole of the retry.
  if (_out.to) slimproto::router().routes.forget(_out.to);
  uint8_t body[slim::MAX_BODY];
  const int n = slimapp::bodyFor(_out.text, body, sizeof(body));
  if (n < 0) { _out.type = 0; _out.lost = true; return; }
  _out.seq = (uint8_t)(slimproto::router().peekNextSeq() & 0xFF);
  _out.tries++;
  _out.due = futureMillis(SLIM_TEXT_ACK_MS);
  sendSlimBurst(_out.type, body, (uint8_t)n, _out.to);
}

#ifdef WITH_TRACKER_MODE
//: Where this unit last knew it was: the live GPS if it has settled,
//: otherwise the last place it recorded. Never a guess.
bool MyMesh::hereNow(double& lat, double& lon) {
  LocationProvider* loc = sensors.getLocationProvider();
  if (loc != NULL && _fixes >= tracker::FIX_SETTLE) {
    lat = loc->getLatitude() / 1000000.0;
    lon = loc->getLongitude() / 1000000.0;
    return true;
  }
  // The newest place this unit recorded. _trail_n is reset after every
  // report but _trail is not, so _trail[0] outlives the report it went
  // out in - which is exactly what a marker dropped a minute later
  // needs. `fix` is only ever set, by pushSample, so it reads as "this
  // unit has known where it was at least once".
  if (!_trail[0].fix) return false;
  lat = _trail[0].lat;
  lon = _trail[0].lon;
  return true;
}

//: A pin where this unit is standing, for the whole team.
bool MyMesh::placeMarker(uint8_t icon, const char* label) {
  double lat, lon;
  if (!slimproto::active() || !hereNow(lat, lon)) return false;
  uint8_t role = slimcodec::ROLE_TRACKER, number = 0;
  // The creator is nine bits of callsign. A unit whose name is not a
  // briefing callsign has no identity to sign a marker with - and an
  // unsigned pin could never be moved or removed by the person who put
  // it there, because identity is exactly what makes that possible.
  if (!slimcompanion::unitFromCallsign(_prefs.node_name, role, number)) {
    return false;
  }
  char text[slimcodec::MARKER_CHARS + slimcodec::MARKER_MAX_LABEL + 1];
  if (slimcodec::encodeMarker(text, sizeof(text), role, number, lat, lon,
                              icon, _marker_num, label) < 0) {
    return false;
  }
  // Through the same ladder a message takes. A pin that half the team
  // never received is worse than no pin: everyone who did receive it
  // assumes everyone else did too, and nobody is looking for the gap.
  if (!sendSlimOutgoing(text, slim::TYPE_GRPTEXT, slim::ADDR_BROADCAST,
                        true)) {
    return false;
  }
  // Only once it is actually away. Stepping the counter on a refused
  // send would leave a gap, and the gap is a pin somebody thinks they
  // dropped that nothing anywhere has.
  _marker_num = (uint8_t)((_marker_num + 1) % slimcodec::MARKER_PER_DEVICE);
  return true;
}
#endif

//: Long enough after boot that the radio has settled and a base nearby is
//: listening, short enough that somebody switching a unit on at the
//: vehicle sees it appear before they have walked away.
static const unsigned long SLIM_ADVERT_FIRST_MS = 15000;
//: ...and then hourly, which is MeshCore's own advert interval. At about
//: 53 bytes that is nothing, and it is what lets a base switched on after
//: the teams still learn every unit without anybody touching them.
static const unsigned long SLIM_ADVERT_EVERY_MS = 3600000UL;
//: ...but every ten minutes while this unit has lost the base.
//:
//: The hourly rate assumes the base already knows this unit. The case
//: that loses people is the one where it does not: a unit switched on
//: out of range, whose boot advert reached nobody, walking back in. Its
//: positions then arrive at a base with no contact to thread them to and
//: are discarded, and NEITHER END LOOKS WRONG - the base's router
//: acknowledges the frame regardless, so the unit believes it is in
//: contact while nothing it sends is being recorded.
//:
//: Only while searching, so the airtime is spent by exactly the units
//: that nothing has answered.
static const unsigned long SLIM_ADVERT_LOST_MS = 600000UL;


//: What this unit is running, for the advert to carry.
//:
//: FIRMWARE_VERSION alone does not answer the question people ask. It is a
//: hardcoded string in a header, so five different builds flashed in one
//: evening all report "v1.16.0" and the operator cannot tell which is on
//: the unit in their hand. __DATE__ and __TIME__ are the compiler's own
//: record of this build and change whenever this file is recompiled, which
//: is what makes the two together worth putting on the air.
static const char* SLIM_FW = FIRMWARE_VERSION " " __DATE__ " " __TIME__;
//: "!V<callsign><64 hex>" - who this unit is, to anyone listening.
//:
//: Broadcast, because the point is to reach a base that has never
//: heard of this unit and therefore cannot be addressed by name yet.
//: Hex rather than packed: it runs at most once an hour and on the
//: operator pressing Advert, and being able to read one in a log is
//: worth more than the fifteen bytes packing would save.
bool MyMesh::sendSlimAdvert() {
  if (!slimproto::active()) return false;
  // 64 for the key, the rest for "<name> <version>": a 32-char name and
  // a compile stamp both fit, and the length guard below refuses rather
  // than truncating if they ever do not.
  char text[PUB_KEY_SIZE * 2 + 96];
  // "<name> <version>": the reader takes the name up to the first space
  // and the key from the last 64 characters, so the version sits between
  // them and an advert from a unit too old to carry one still reads.
  //
  // MESH_SLIM_ADVERT_NO_VERSION builds the older, version-less form. A
  // base that predates the space-separated name reads the version as
  // part of the callsign and writes rubbish into its pairing file, so a
  // unit flashed ahead of its base needs this until the base catches up.
#ifdef MESH_SLIM_ADVERT_NO_VERSION
  const int n = snprintf(text, sizeof(text), "!V%s", _prefs.node_name);
#else
  const int n = snprintf(text, sizeof(text), "!V%s %s",
                         _prefs.node_name, SLIM_FW);
#endif
  if (n < 3 || n + PUB_KEY_SIZE * 2 >= (int)sizeof(text)) return false;
  mesh::Utils::toHex(&text[n], self_id.pub_key, PUB_KEY_SIZE);
  text[n + PUB_KEY_SIZE * 2] = 0;

  uint8_t body[slim::MAX_BODY];
  const int blen = slimapp::bodyFor(text, body, sizeof(body));
  if (blen < 0) return false;
  return sendSlimBurst(slim::TYPE_GRPTEXT, body, (uint8_t)blen,
                       slim::ADDR_BROADCAST);
}

//: Somebody said who they are. Learn them, so they can be addressed.
//:
//: noinline because it puts a ContactInfo on the stack and it is
//: reached from the packet receive path, which is the deepest call in
//: this firmware. Inlined, every slim frame would pay for it.
__attribute__((noinline))
bool MyMesh::slimAdvert(const char* msg) {
  if (msg[0] != '!' || msg[1] != 'V') return false;
  const int len = (int)strlen(msg);
  const int hex = PUB_KEY_SIZE * 2;
  // The key is fixed width and last, so whatever precedes it is the
  // name - which may be any length and needs no separator.
  if (len < 2 + 1 + hex) return true;      // malformed: ours, and dropped
  char name[32];
  const int nlen = len - 2 - hex;
  if (nlen <= 0 || nlen >= (int)sizeof(name)) return true;
  memcpy(name, &msg[2], (size_t)nlen);
  name[nlen] = 0;

  uint8_t key[PUB_KEY_SIZE];
  if (!mesh::Utils::fromHex(key, PUB_KEY_SIZE, &msg[2 + nlen])) return true;

  // A unit whose name is not a briefing callsign has no unit number,
  // so there is nothing to map it to and nothing that could address
  // it. Learned as a contact all the same: it is real, and the
  // operator can see it and rename it.
  ContactInfo* known = lookupContactByPubKey(key, PUB_KEY_SIZE);
  bool moved = false;      // the roster changed, not just a hello
  if (known == NULL) {
    ContactInfo c;
    memset(&c, 0, sizeof(c));
    c.id = mesh::Identity(key);
    StrHelper::strncpy(c.name, name, sizeof(c.name));
    c.type = ADV_TYPE_CHAT;
    c.out_path_len = -1;                   // flood until a route is known
    c.lastmod = getRTCClock()->getCurrentTime();
    c.last_advert_timestamp = c.lastmod;
    if (!addContact(c)) {
      // No room. Still announced: MeshCore tells the app about an
      // advert it could not file, so the operator sees a unit that
      // is on the air and can make room for it, rather than a unit
      // that appears not to exist.
      onDiscoveredContact(c, true, 0, NULL);
      onContactsFull();
      return true;
    }
    known = lookupContactByPubKey(key, PUB_KEY_SIZE);
    if (known == NULL) return true;        // cannot happen; not worth a crash
    moved = true;
  } else if (strcmp(known->name, name) != 0) {
    // Renamed on the unit itself. The name is the identity here, so
    // the contact follows it rather than keeping a stale one.
    StrHelper::strncpy(known->name, name, sizeof(known->name));
    moved = true;
  }
  known->last_advert_timestamp = getRTCClock()->getCurrentTime();
  known->lastmod = known->last_advert_timestamp;

  // MeshCore's own "I have learned a contact" hook, and a slim advert
  // is one. It maps the unit, pushes to the phone so the contact list
  // and Chat's recipient picker refresh, chimes the UI when no phone
  // is attached, and schedules the lazy write - all of which adding
  // the contact alone did not do. That was the whole bug: the unit
  // reached flash and nothing above the radio was ever told, so it
  // could not be picked to send to and its own messages were dropped
  // for having no contact to thread them to.
  //
  // No path: the advert-path cache is a MeshCore route table, and a
  // slim hop count says nothing it could store.
  onDiscoveredContact(*known, false, 0, NULL);

  // A unit number that has changed hands. Only on a change - the
  // common case is the same unit saying hello on its hour, and the
  // hook above has already mapped it.
  if (moved) loadSlimRoster();
  return true;
}

void MyMesh::onSlimRecv(const slim::Frame& f,
                        const slimproto::Link& link) {
  // Built once, from the frame and the radio, and handed to every emit
  // below. A trail arrives as two messages and both came off the same
  // transmission, so both must report the same link.
  const Heard heard = { (uint8_t)(f.hops - 1), f.hop,
                        link.snr_q4, link.rssi };
#ifdef WITH_TRACKER_MODE
  // Something authenticated, while this unit has been looking for a way
  // through - so report at once rather than waiting out the rest of the
  // interval, because the way through may be a helicopter and it may not
  // be here in five minutes.
  //
  // With one exception. A POSITION that arrived straight from the unit
  // that made it (the hop byte still its own) is not a way through: it is
  // another handheld nearby, and a handheld broadcasting a position is
  // very likely as lost as this one. Two of them in earshot would keep
  // each other awake for ever, each reading the other as evidence of a
  // base neither can reach - which is exactly the pair this feature
  // exists for, both behind the same ridge. A repeated frame is the
  // opposite: something relayed it, so infrastructure is alive.
  trackerHeardSomething(f.type != slim::TYPE_POS || f.hop != f.src);
#endif
  // A status request, from the base or from anyone holding the operation
  // key. Answered even with little to report: an empty status still says
  // the node is alive, and silence would be indistinguishable from out of
  // range.
  if (f.type == slim::TYPE_PING) {
    oweSlimReply(f.src, slim::TYPE_STATUS, 0,
                 slimhealth::wanted(f.body, f.body_len));
    return;
  }
  // The answer to a link test. Only while one is outstanding, and only
  // from the unit it was addressed to - a status volunteered by a
  // passing repeater is somebody else's numbers and would read as this
  // unit's link to the base.
  if (f.type == slim::TYPE_STATUS) {
    if (_link_test.sent_at && !_link_test.got_at
        && f.src == slim::ADDR_BASE) {
      slimhealth::Status st;
      if (slimhealth::decode(f.body, f.body_len, st)) {
        if (st.has & (1u << slimhealth::F_RSSI)) {
          _link_test.up_rssi = (int16_t)st.value[slimhealth::F_RSSI];
        }
        if (st.has & (1u << slimhealth::F_SNR)) {
          _link_test.up_snr = (float)st.value[slimhealth::F_SNR];
        }
        // ...and what this unit made of the reply, which it measured
        // itself on the way in. Free, and the other half of the answer.
        _link_test.hops = f.hops;
        _link_test.dn_rssi = link.rssi;
        _link_test.dn_snr = link.snr_q4 / 4.0f;
        _link_test.got_at = _ms->getMillis();
      }
    }
    return;
  }
#ifdef WITH_TRACKER_MODE
  if (f.type == slim::TYPE_ROSTER) {
    onRoundAck(f);
    return;
  }
#endif

  // An acknowledgement coming back for something we sent. The body is the one
  // byte of sequence number; both ends derive the app's 4-byte id from that
  // plus who sent it, so nothing has to carry the id itself.
  if (f.type == slim::TYPE_ACK) {
    if (f.body_len < 1) return;
    // The message this unit's own screen sent, which is waiting on exactly
    // this. Before anything else: it is the one somebody is standing there
    // watching for.
    // The base acknowledges a broadcast too - a marker and a position are
    // the same bargain - so the sender matters only for an addressed one.
    if (_out.type && f.body[0] == _out.seq
        && (f.src == _out.to || _out.to == slim::ADDR_BROADCAST)) {
      _out.type = 0;
      _out.acked = true;
      if (_ui) _ui->notify(UIEventType::ack);
      return;
    }
    const uint32_t id = slimcompanion::ackId(f.src, f.body[0]);
#ifdef WITH_TRACKER_MODE
    // A tracker report, or a piece of the path it kept while it was out
    // of contact. Either way the base can hear us again, so whatever is
    // owed goes now, while the radio is still listening for the reply -
    // one message per acknowledgement, chaining until the gap is filled.
    const tracker::Reports::Ack got = _reports.acked(id);
    if (got != tracker::Reports::NOT_OURS) {
      _ack_due = 0;
      if (got == tracker::Reports::REPORT)
        trackerSound(UIEventType::trackerHeard);
      trackerHistory();
      return;
    }
#endif
    for (int i = 0; i < EXPECTED_ACK_TABLE_SIZE; i++) {
      if (expected_ack_table[i].ack != id) continue;
      out_frame[0] = PUSH_CODE_SEND_CONFIRMED;
      memcpy(&out_frame[1], &id, 4);
      uint32_t trip = _ms->getMillis() - expected_ack_table[i].msg_sent;
      memcpy(&out_frame[5], &trip, 4);
      _serial->writeFrame(out_frame, 9);
      // Cleared so a duplicate ack - which multi-ack makes likely, and the
      // burst makes likelier still - is not reported to the app twice.
      expected_ack_table[i].ack = 0;
      return;
    }
    return;
  }

  // A direct message. Rendered as a contact message from whoever the unit map
  // says that unit number is, so the app threads it against the right contact
  // without ever seeing a unit number.
  if (f.type == slim::TYPE_TEXT) {
    // Both checks first, and both are pure computation - no I/O, nothing that
    // delays the reply meaningfully. Acknowledging before them would tell the
    // sender a message landed that we are about to drop, and it would stop
    // retrying: 1 + multi_acks transmissions spent to lose the message twice.
    const uint8_t* pub = slimproto::pubKeyForUnit(f.src);

    char msg[slimapp::MAX_TEXT + 24];
    const int mlen = slimapp::decodeBody(f.body, f.body_len, msg, sizeof(msg));
    if (mlen <= 0) return;

    // A command from the base needs no contact to thread to - and the base is
    // often not on the roster by unit number at all: it is MSHCMD, and unit 15
    // by convention (MESH_SLIM_UNIT on the Pi and in the SLIM base build).
    const bool from_base = slimcompanion::unitRole(f.src) == slimcodec::ROLE_BASE
                           || f.src == slim::ADDR_BASE;
    bool command = protoCommand(msg, from_base);
#ifdef WITH_TRACKER_MODE
    if (!command) command = trackerCommand(msg, from_base);
#endif
    // **A message from the base is never dropped for want of a contact.**
    //
    // `pub` is only needed to build the phone app's frame, which threads a
    // message against a contact by its key. A unit that has not yet heard
    // the base's advert has no key for it - which is every unit for the
    // first minutes after a reflash, and any unit that missed one advert.
    //
    // Dropping the base's words there is the worst failure this radio has:
    // the operator's screen says sent, the searcher sees nothing, and
    // nothing anywhere reports it. The base is unit 15 by the briefing
    // scheme, the same convention that already decides whose commands this
    // unit obeys, so it is known well enough to acknowledge and to show on
    // this unit's own screen. Only the phone's copy needs the key.
    if (pub == NULL && !command && !from_base) return;

    // Owed, not sent: see MyMesh::Owed. An acknowledgement transmitted from
    // here goes into the tail of the sender's own burst and is never heard,
    // whereupon the sender retries a message that did in fact arrive.
    //
    // ttl is capped at 2 rather than TTL_INIT so a reply cannot flood the
    // whole net behind every message. An ACK is the one type the protocol
    // lets travel short, because it teaches nothing about distance.
    oweSlimReply(f.src, slim::TYPE_ACK, f.seq, 0);
    // Acknowledged, which is the base's confirmation. Not queued: there is no
    // phone to show it to.
    if (command) return;

    // The phone's copy, which is the only part that needs the key. A
    // message from a sender this unit has no contact for still reaches the
    // screen below; it simply cannot be threaded in an app that files
    // everything by key.
    if (pub != NULL) {
      int i = 0;
      if (app_target_ver >= 3) {
        out_frame[i++] = RESP_CODE_CONTACT_MSG_RECV_V3;
        out_frame[i++] = 0;         // SNR
        out_frame[i++] = 0;
        out_frame[i++] = 0;
      } else {
        out_frame[i++] = RESP_CODE_CONTACT_MSG_RECV;
      }
      memcpy(&out_frame[i], pub, 6); i += 6;  // the 6-byte prefix, as always
      out_frame[i++] = (uint8_t)(f.hops - 1);
      out_frame[i++] = TXT_TYPE_PLAIN;
      uint32_t ts = getRTCClock()->getCurrentTime();
      memcpy(&out_frame[i], &ts, 4); i += 4;
      int n = mlen;
      if (i + n > MAX_FRAME_SIZE) n = MAX_FRAME_SIZE - i;
      memcpy(&out_frame[i], msg, n); i += n;
      addToOfflineQueue(out_frame, i);
      if (_serial->isConnected()) {
        uint8_t fr[1]; fr[0] = PUSH_CODE_MSG_WAITING;
        _serial->writeFrame(fr, 1);
      }
    }
    // Who to answer. Recorded even with no screen attached, because
    // the phone may not be the thing that replies.
    _slim_reply_to = f.src;
#ifdef DISPLAY_CLASS
    // ...and onto this unit's own screen. Without this a message
    // arrived, was acknowledged, and was visible only to a phone that
    // on a hill is in somebody's pack - so from the outside the radio
    // had simply ignored it.
    if (_ui) {
      char from[16];
      slimcompanion::callsignFor(f.src, slimcompanion::unitRole(f.src),
                                 from, sizeof(from));
      _ui->newMsg((uint8_t)(f.hops - 1), from, msg, offline_queue_len);
      if (!_serial->isConnected()) {
        _ui->notify(UIEventType::contactMessage);
      }
    }
#endif
    return;
  }

  char text[slimcompanion::MAX_TEXT + slimapp::MAX_TEXT + 8];
  int tlen = 0;

  if (f.type == slim::TYPE_POS) {
    // **Only the base acknowledges a position.** A position is broadcast, so
    // every unit hears it and fifteen units all replying would be the ack
    // storm the broadcast exists to avoid - but exactly one reply is nine
    // bytes, and it is the difference between a tracker that knows it is
    // still being heard and one that walks a ridge for an hour believing it.
    //
    // Which node that is needs no field on the wire: the base is unit 15 by
    // the briefing scheme (loadSlimUnit), the same convention that decides
    // whose commands a tracker obeys. This is what lets a TABLET stand in for
    // the Pi - without it the tablet plots everyone perfectly while every
    // tracker in the field quietly concludes it has lost the base.
    if (slimproto::unit() == slim::ADDR_BASE) {
#ifdef WITH_TRACKER_MODE
      // Noted for the round-ack, which answers everybody at once. Only a
      // base that is not running rounds replies to each report - right for
      // a handful of units, wrong for sixteen.
      _round_heard = slimround::withUnit(_round_heard, f.src);
      if (_round_s == 0)
#endif
      {
        // Owed, not sent: see MyMesh::Owed. This is the one that matters
        // most - a tracker that never hears its acknowledgement concludes
        // it has lost the base and starts keeping a backlog, in clear
        // line of sight of the antenna.
        oweSlimReply(f.src, slim::TYPE_ACK, f.seq, 0);
      }
    }
#ifdef WITH_TRACKER_MODE
    if (slimtrack::isTrack(f.body, f.body_len)) {
      // A trail: where it is, and where it has been. It goes up as the two
      // messages every base and every phone on this network already reads
      // - a "!P" for the current place and an "!H" for the older ones,
      // which carries their ages and is exactly how an out-of-range gap is
      // handed over. Nothing above this had to learn a new format.
      slimtrack::Point pts[slimtrack::SAMPLES];
      uint16_t cadence = 0;
      const int n = slimtrack::decode(f.body, f.body_len, pts,
                                      slimtrack::SAMPLES, cadence);
      if (n <= 0) return;
      uint8_t role = slimcompanion::unitRole(f.src), unit = f.src;
      char now_text[slimcompanion::MAX_TEXT + 24];
      char who[16];
      slimcompanion::callsignFor(f.src, role, who, sizeof(who));
      char self[slimcodec::SELF_CHARS + 1];
      if (slimcodec::encodeSelfReport(self, role, unit, pts[0].lat,
                                      pts[0].lon)) {
        snprintf(now_text, sizeof(now_text), "%s: %s", who, self);
        emitChannelText(now_text, heard);
      }
      if (n > 1) {
        slimcodec::HistPoint hist[slimtrack::SAMPLES];
        uint8_t hn = 0;
        for (int k = 1; k < n; k++) {
          hist[hn].lat = pts[k].lat;
          hist[hn].lon = pts[k].lon;
          hist[hn].age_s = (uint32_t)k * cadence;
          hn++;
        }
        char h[slimcodec::HIST_MAX_TEXT + 1];
        if (slimcodec::encodeHistory(h, hist, hn) > 0) {
          char with_who[slimcompanion::MAX_TEXT + slimcodec::HIST_MAX_TEXT + 8];
          snprintf(with_who, sizeof(with_who), "%s: %s", who, h);
          emitChannelText(with_who, heard);
        }
      }
      return;
    }
#endif
    // Asked before the position is decoded, not after: by then a no-fix
    // has already become a coordinate, and a coordinate is somewhere.
    if (slimcompanion::isNoFix(f)) {
      tlen = slimcompanion::formatNoFix(f, text, sizeof(text));
    } else {
      // Rebuild the exact "!P" channel text the app already knows how to
      // plot. The app is not told the protocol changed, which is the whole
      // design.
      tlen = slimcompanion::formatPosition(f, text, sizeof(text));
    }
  } else if (f.type == slim::TYPE_GRPTEXT) {
    // Broadcast, and **exactly one node answers it** - the base, by the
    // same rule and for the same reason it alone answers a position. A
    // marker that half the team never received is worse than none,
    // because everyone who did receive it assumes everyone else did too,
    // and one nine-byte reply is what lets the unit that dropped it try
    // again. Owed rather than sent: see MyMesh::Owed.
    if (slimproto::unit() == slim::ADDR_BASE) {
      oweSlimReply(f.src, slim::TYPE_ACK, f.seq, 0);
    }
    // Team chat. MeshCore writes "NAME: " into a group body because the packet
    // carries no sender, so the same prefix is rebuilt here from the unit
    // number - otherwise the app shows the message with nobody attached.
    char who[16];
    slimcompanion::callsignFor(f.src, slimcompanion::unitRole(f.src),
                               who, sizeof(who));
    char msg[slimapp::MAX_TEXT + 24];
    if (slimapp::decodeBody(f.body, f.body_len, msg, sizeof(msg)) <= 0) return;
    // Who somebody is, rather than something they said. Handled before
    // the text is dressed as chat, or an advert would appear in the
    // message log as a callsign followed by sixty-four hex digits.
    if (slimAdvert(msg)) return;
    tlen = snprintf(text, sizeof(text), "%s: %s", who, msg);
  }
  if (tlen <= 0) return;
  emitChannelText(text, heard);
}

//: The two reserved bytes of the v3 channel frame carry what MeshCore has
//: nowhere else to put: which node handed this copy over, and how strong it
//: was. Both were written as zero here, and the SNR byte was too - so every
//: slim reception reached the app reading 0 dB, from nobody, and a base
//: could not tell a unit on the edge of range from one in the car park.
//:
//: A stock app ignores reserved bytes, so this costs nothing outside the
//: fork. Zero means "not known" in both: unit 0 is the broadcast address
//: and invalid as a source, and an RSSI of 0 dBm does not happen on LoRa.
void MyMesh::emitChannelText(const char* text, const Heard& heard) {
  int tlen = (int)strlen(text);
  if (tlen <= 0) return;
  int i = 0;
  if (app_target_ver >= 3) {
    out_frame[i++] = RESP_CODE_CHANNEL_MSG_RECV_V3;
    out_frame[i++] = (uint8_t)heard.snr_q4;      // SNR in quarter-dB
    out_frame[i++] = heard.via;                  // reserved1: the last hop
    // reserved2: -dBm, so it fits a byte. Clamped rather than wrapped: a
    // reading off the end of the scale should read as the end of the scale.
    const int down = heard.rssi < 0 ? -heard.rssi : 0;
    out_frame[i++] = (uint8_t)(down > 255 ? 255 : down);
  } else {
    out_frame[i++] = RESP_CODE_CHANNEL_MSG_RECV;
  }
  out_frame[i++] = 0;                          // channel 0, the team channel
  out_frame[i++] = heard.hops;                 // path length, as the app reads it
  out_frame[i++] = TXT_TYPE_PLAIN;

  uint32_t timestamp = getRTCClock()->getCurrentTime();
  memcpy(&out_frame[i], &timestamp, 4);
  i += 4;

  if (i + tlen > MAX_FRAME_SIZE) tlen = MAX_FRAME_SIZE - i;
  memcpy(&out_frame[i], text, tlen);
  i += tlen;
  addToOfflineQueue(out_frame, i);

  if (_serial->isConnected()) {
    uint8_t frame[1];
    frame[0] = PUSH_CODE_MSG_WAITING;
    _serial->writeFrame(frame, 1);
  }

#ifdef DISPLAY_CLASS
  // Team chat, on this unit's own screen.
  //
  // App datagrams share this path and are NOT messages: a position, a
  // marker, a roster. Showing somebody on a hill "MTRK02: !MQD98AUSm91gA"
  // is worse than showing them nothing, because it reads as a fault in
  // the radio rather than as traffic that was never meant for them.
  if (_ui) {
    const char* body = slimcompanion::skipSenderPrefix(text);
    if (body[0] != 0 && body[0] != '!') {
      char from[20];
      size_t nlen = (size_t)(body - text);
      if (nlen >= 2) nlen -= 2;              // back off the ": "
      if (nlen == 0 || nlen >= sizeof(from)) {
        strcpy(from, "Team");
      } else {
        memcpy(from, text, nlen);
        from[nlen] = 0;
      }
      _ui->newMsg(heard.hops, from, body, offline_queue_len);
      if (!_serial->isConnected()) {
        _ui->notify(UIEventType::channelMessage);
      }
    }
  }
#endif
}

// ---- the protocol switch -----------------------------------------------
// Saved by the menu (the L1's Protocol page), applied at the next boot. A
// restart rather than a live change, as `proto slim` on a repeater advises:
// sync word, preamble and CRC move together, and a restart is the one way
// to be sure nothing is half-way through a packet when they do.

static const char* PROTO_FILE = "/proto";

bool MyMesh::saveSlimProto(bool on) {
  // Refused rather than allowed-and-broken: slim with no key is a radio on
  // a sync word nobody else uses, unable to authenticate anything.
  if (!slimproto::canSwitch() || (on && !slimproto::haveKey())) return false;
  File f = _store->openWrite(PROTO_FILE);
  if (!f) return false;
  const uint8_t v = on ? 1 : 0;
  f.write(&v, 1);
  f.close();
  return true;
}

void MyMesh::loadSlimProto() {
  File f = _store->openRead(PROTO_FILE);
  if (!f) return;
  uint8_t v = 0;
  const bool got = f.read(&v, 1) == 1;
  f.close();
  if (got && slimproto::canSwitch() && (v == 0 || slimproto::haveKey())) {
    slimproto::setActive(v != 0);   // reconfigures the radio if it changed
  }
}

// ---- moved by the base --------------------------------------------------
// "!S1" moves this unit to slim and "!S0" to MeshCore: the Protocol column
// on the base's web page (proto_mode.py). Saved like the menu's choice, then
// a restart - but only once the ack is on the air, because the ack is the
// last thing the base will hear from this unit until it moves too.

//: Quiet on the outbound queue for this long before restarting. The queue is
//: empty as soon as its last packet starts transmitting, and a slow radio
//: setting takes a couple of seconds to finish one.
static const unsigned long PROTO_RESTART_QUIET_MS = 4000;
//: Restart by then whatever the queue is doing: a busy channel must not keep
//: a unit on a protocol it has already told the base it is leaving.
static const unsigned long PROTO_RESTART_MAX_MS = 30000;

bool MyMesh::protoCommand(const char* text, bool from_base) {
  if (!from_base || text == NULL || text[0] != '!' || text[1] != 'S'
      || (text[2] != '0' && text[2] != '1') || text[3] != 0) {
    return false;
  }
  const bool on = text[2] == '1';
  // Handled either way. Already there, or refused - no key, or a build that
  // cannot switch - the ack still goes, and the base learns which from the
  // protocol it next hears this unit on.
  if (on != slimproto::active() && saveSlimProto(on)) {
    _proto_restart_at = futureMillis(PROTO_RESTART_QUIET_MS);
    _proto_restart_by = futureMillis(PROTO_RESTART_MAX_MS);
  }
  return true;
}

// ---- the unit number ----------------------------------------------------
// A unit named on the scheme is that number on slim - MTRK07 is unit 7 - which
// is how the base addresses it (slim_node.unit_from_callsign), so a unit
// moved between protocols keeps one callsign. MESH_SLIM_UNIT is the fallback
// for a name with no number. 15 is the base's by convention: a unit named
// MTRK15 keeps the build's number rather than answering as the base.
void MyMesh::loadSlimUnit() {
  uint8_t role, number;
  if (!slimcompanion::unitFromCallsign(_prefs.node_name, role, number)) return;
  // The base's names carry no number: MSHCMD is unit 15, the base's by
  // convention, and a base repeats what it hears - which is how a tablet
  // driving an ordinary tracker build stands in for the Pi on slim.
  if (role == slimcodec::ROLE_BASE && number == 0) {
    slimproto::setUnit(slim::ADDR_BASE);
    slimproto::setForwards(true);
    return;
  }
  if (number >= 1 && number < slim::ADDR_BASE) slimproto::setUnit(number);
}

#endif  // WITH_MESH_SLIM

void MyMesh::onChannelDataRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint16_t data_type,
                               const uint8_t *data, size_t data_len) {
  if (data_len > MAX_CHANNEL_DATA_LENGTH) {
    MESH_DEBUG_PRINTLN("onChannelDataRecv: dropping payload_len=%d exceeds frame limit=%d",
                       (uint32_t)data_len, (uint32_t)MAX_CHANNEL_DATA_LENGTH);
    return;
  }

  int i = 0;
  out_frame[i++] = RESP_CODE_CHANNEL_DATA_RECV;
  out_frame[i++] = (int8_t)(pkt->getSNR() * 4);
  out_frame[i++] = 0; // reserved1
  out_frame[i++] = 0; // reserved2

  uint8_t channel_idx = findChannelIdx(channel);
  out_frame[i++] = channel_idx;
  out_frame[i++] = pkt->isRouteFlood() ? pkt->path_len : 0xFF;
  out_frame[i++] = (uint8_t)(data_type & 0xFF);
  out_frame[i++] = (uint8_t)(data_type >> 8);
  out_frame[i++] = (uint8_t)data_len;

  int copy_len = (int)data_len;
  if (copy_len > 0) {
    memcpy(&out_frame[i], data, copy_len);
    i += copy_len;
  }
  addToOfflineQueue(out_frame, i);

  if (_serial->isConnected()) {
    uint8_t frame[1];
    frame[0] = PUSH_CODE_MSG_WAITING; // send push 'tickle'
    _serial->writeFrame(frame, 1);
  }
}

uint8_t MyMesh::onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp, const uint8_t *data,
                                 uint8_t len, uint8_t *reply) {
  if (data[0] == REQ_TYPE_GET_TELEMETRY_DATA) {
    uint8_t permissions = 0;
    uint8_t cp = contact.flags >> 1; // LSB used as 'favourite' bit (so only use upper bits)

    if (_prefs.telemetry_mode_base == TELEM_MODE_ALLOW_ALL) {
      permissions = TELEM_PERM_BASE;
    } else if (_prefs.telemetry_mode_base == TELEM_MODE_ALLOW_FLAGS) {
      permissions = cp & TELEM_PERM_BASE;
    }

    if (_prefs.telemetry_mode_loc == TELEM_MODE_ALLOW_ALL) {
      permissions |= TELEM_PERM_LOCATION;
    } else if (_prefs.telemetry_mode_loc == TELEM_MODE_ALLOW_FLAGS) {
      permissions |= cp & TELEM_PERM_LOCATION;
    }

    if (_prefs.telemetry_mode_env == TELEM_MODE_ALLOW_ALL) {
      permissions |= TELEM_PERM_ENVIRONMENT;
    } else if (_prefs.telemetry_mode_env == TELEM_MODE_ALLOW_FLAGS) {
      permissions |= cp & TELEM_PERM_ENVIRONMENT;
    }

    uint8_t perm_mask = ~(data[1]);    // NEW: first reserved byte (of 4), is now inverse mask to apply to permissions
    permissions &= perm_mask;

    if (permissions & TELEM_PERM_BASE) { // only respond if base permission bit is set
      telemetry.reset();
      telemetry.addVoltage(TELEM_CHANNEL_SELF, (float)board.getBattMilliVolts() / 1000.0f);
      // query other sensors -- target specific
      sensors.querySensors(permissions, telemetry);

      memcpy(reply, &sender_timestamp,
             4); // reflect sender_timestamp back in response packet (kind of like a 'tag')

      uint8_t tlen = telemetry.getSize();
      memcpy(&reply[4], telemetry.getBuffer(), tlen);
      return 4 + tlen;
    }
  }
  return 0; // unknown
}

void MyMesh::onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len) {
  uint32_t tag;
  memcpy(&tag, data, 4);

  if (pending_login && memcmp(&pending_login, contact.id.pub_key, 4) == 0) { // check for login response
    // yes, is response to pending sendLogin()
    pending_login = 0;

    int i = 0;
    if (memcmp(&data[4], "OK", 2) == 0) { // legacy Repeater login OK response
      out_frame[i++] = PUSH_CODE_LOGIN_SUCCESS;
      out_frame[i++] = 0; // legacy: is_admin = false
      memcpy(&out_frame[i], contact.id.pub_key, 6);
      i += 6;                                     // pub_key_prefix
    } else if (data[4] == RESP_SERVER_LOGIN_OK) { // new login response
      uint16_t keep_alive_secs = ((uint16_t)data[5]) * 16;
      if (keep_alive_secs > 0) {
        startConnection(contact, keep_alive_secs);
      }
      out_frame[i++] = PUSH_CODE_LOGIN_SUCCESS;
      out_frame[i++] = data[6]; // permissions (eg. is_admin)
      memcpy(&out_frame[i], contact.id.pub_key, 6);
      i += 6; // pub_key_prefix
      memcpy(&out_frame[i], &tag, 4);
      i += 4; // NEW: include server timestamp
      out_frame[i++] = data[7]; // NEW (v7): ACL permissions
      out_frame[i++] = data[12]; // FIRMWARE_VER_LEVEL
    } else {
      out_frame[i++] = PUSH_CODE_LOGIN_FAIL;
      out_frame[i++] = 0; // reserved
      memcpy(&out_frame[i], contact.id.pub_key, 6);
      i += 6; // pub_key_prefix
    }
    _serial->writeFrame(out_frame, i);
  } else if (len > 4 && // check for status response
             pending_status &&
             memcmp(&pending_status, contact.id.pub_key, 4) == 0 // legacy matching scheme
                                                                 // FUTURE: tag == pending_status
  ) {
    pending_status = 0;

    int i = 0;
    out_frame[i++] = PUSH_CODE_STATUS_RESPONSE;
    out_frame[i++] = 0; // reserved
    memcpy(&out_frame[i], contact.id.pub_key, 6);
    i += 6; // pub_key_prefix
    memcpy(&out_frame[i], &data[4], len - 4);
    i += (len - 4);
    _serial->writeFrame(out_frame, i);
  } else if (len > 4 && tag == pending_telemetry) {  // check for matching response tag
    pending_telemetry = 0;

    int i = 0;
    out_frame[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
    out_frame[i++] = 0; // reserved
    memcpy(&out_frame[i], contact.id.pub_key, 6);
    i += 6; // pub_key_prefix
    memcpy(&out_frame[i], &data[4], len - 4);
    i += (len - 4);
    _serial->writeFrame(out_frame, i);
  } else if (len > 4 && tag == pending_req) {  // check for matching response tag
    pending_req = 0;

    int i = 0;
    out_frame[i++] = PUSH_CODE_BINARY_RESPONSE;
    out_frame[i++] = 0; // reserved
    memcpy(&out_frame[i], &tag, 4);   // app needs to match this to RESP_CODE_SENT.tag
    i += 4;
    memcpy(&out_frame[i], &data[4], len - 4);
    i += (len - 4);
    _serial->writeFrame(out_frame, i);
  }
}

bool MyMesh::onContactPathRecv(ContactInfo& contact, uint8_t* in_path, uint8_t in_path_len, uint8_t* out_path, uint8_t out_path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) {
  if (extra_type == PAYLOAD_TYPE_RESPONSE && extra_len > 4) {
    uint32_t tag;
    memcpy(&tag, extra, 4);

    if (tag == pending_discovery) {  // check for matching response tag)
      pending_discovery = 0;

      if (!mesh::Packet::isValidPathLen(in_path_len) || !mesh::Packet::isValidPathLen(out_path_len)) {
        MESH_DEBUG_PRINTLN("onContactPathRecv, invalid path sizes: %d, %d", in_path_len, out_path_len);
      } else {
        int i = 0;
        out_frame[i++] = PUSH_CODE_PATH_DISCOVERY_RESPONSE;
        out_frame[i++] = 0; // reserved
        memcpy(&out_frame[i], contact.id.pub_key, 6);
        i += 6; // pub_key_prefix
        out_frame[i++] = out_path_len;
        i += mesh::Packet::writePath(&out_frame[i], out_path, out_path_len);
        out_frame[i++] = in_path_len;
        i += mesh::Packet::writePath(&out_frame[i], in_path, in_path_len);
        // NOTE: telemetry data in 'extra' is discarded at present

        _serial->writeFrame(out_frame, i);
      }
      return false;  // DON'T send reciprocal path!
    }
  }
  // let base class handle received path and data
  return BaseChatMesh::onContactPathRecv(contact, in_path, in_path_len, out_path, out_path_len, extra_type, extra, extra_len);
}

void MyMesh::onControlDataRecv(mesh::Packet *packet) {
  if (packet->payload_len + 4 > sizeof(out_frame)) {
    MESH_DEBUG_PRINTLN("onControlDataRecv(), payload_len too long: %d", packet->payload_len);
    return;
  }
  int i = 0;
  out_frame[i++] = PUSH_CODE_CONTROL_DATA;
  out_frame[i++] = (int8_t)(_radio->getLastSNR() * 4);
  out_frame[i++] = (int8_t)(_radio->getLastRSSI());
  out_frame[i++] = packet->path_len;
  memcpy(&out_frame[i], packet->payload, packet->payload_len);
  i += packet->payload_len;

  if (_serial->isConnected()) {
    _serial->writeFrame(out_frame, i);
  } else {
    MESH_DEBUG_PRINTLN("onControlDataRecv(), data received while app offline");
  }
}

void MyMesh::onRawDataRecv(mesh::Packet *packet) {
  if (packet->payload_len + 4 > sizeof(out_frame)) {
    MESH_DEBUG_PRINTLN("onRawDataRecv(), payload_len too long: %d", packet->payload_len);
    return;
  }
  int i = 0;
  out_frame[i++] = PUSH_CODE_RAW_DATA;
  out_frame[i++] = (int8_t)(_radio->getLastSNR() * 4);
  out_frame[i++] = (int8_t)(_radio->getLastRSSI());
  out_frame[i++] = 0xFF; // reserved (possibly path_len in future)
  memcpy(&out_frame[i], packet->payload, packet->payload_len);
  i += packet->payload_len;

  if (_serial->isConnected()) {
    _serial->writeFrame(out_frame, i);
  } else {
    MESH_DEBUG_PRINTLN("onRawDataRecv(), data received while app offline");
  }
}

void MyMesh::onTraceRecv(mesh::Packet *packet, uint32_t tag, uint32_t auth_code, uint8_t flags,
                         const uint8_t *path_snrs, const uint8_t *path_hashes, uint8_t path_len) {
  uint8_t path_sz = flags & 0x03;  // NEW v1.11+
  if (12 + path_len + (path_len >> path_sz) + 1 > sizeof(out_frame)) {
    MESH_DEBUG_PRINTLN("onTraceRecv(), path_len is too long: %d", (uint32_t)path_len);
    return;
  }
  int i = 0;
  out_frame[i++] = PUSH_CODE_TRACE_DATA;
  out_frame[i++] = 0; // reserved
  out_frame[i++] = path_len;
  out_frame[i++] = flags;
  memcpy(&out_frame[i], &tag, 4);
  i += 4;
  memcpy(&out_frame[i], &auth_code, 4);
  i += 4;
  memcpy(&out_frame[i], path_hashes, path_len);
  i += path_len;

  memcpy(&out_frame[i], path_snrs, path_len >> path_sz);
  i += path_len >> path_sz;
  out_frame[i++] = (int8_t)(packet->getSNR() * 4); // extra/final SNR (to this node)

  if (_serial->isConnected()) {
    _serial->writeFrame(out_frame, i);
  } else {
    MESH_DEBUG_PRINTLN("onTraceRecv(), data received while app offline");
  }
}

uint32_t MyMesh::calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const {
  return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
}
uint32_t MyMesh::calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const {
  uint8_t path_hash_count = path_len & 63;
  return SEND_TIMEOUT_BASE_MILLIS +
         ((pkt_airtime_millis * DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) *
          (path_hash_count + 1));
}

void MyMesh::onSendTimeout() {}

MyMesh::MyMesh(mesh::Radio &radio, mesh::RNG &rng, mesh::RTCClock &rtc, SimpleMeshTables &tables, DataStore& store, AbstractUITask* ui)
    : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables),
      _serial(NULL), telemetry(MAX_PACKET_PAYLOAD - 4), _store(&store), _ui(ui) {
  _iter_started = false;
  _cli_rescue = false;
  offline_queue_len = 0;
  app_target_ver = 0;
  clearPendingReqs();
  next_ack_idx = 0;
  sign_data = NULL;
  dirty_contacts_expiry = 0;
  memset(advert_paths, 0, sizeof(advert_paths));
  memset(send_scope.key, 0, sizeof(send_scope.key));
  send_unscoped = false;

  // defaults
  memset(&_prefs, 0, sizeof(_prefs));
  _prefs.airtime_factor = 1.0;
  strcpy(_prefs.node_name, "NONAME");
  _prefs.freq = LORA_FREQ;
  _prefs.sf = LORA_SF;
  _prefs.bw = LORA_BW;
  _prefs.cr = LORA_CR;
  _prefs.tx_power_dbm = LORA_TX_POWER;
  _prefs.gps_enabled = 0;       // GPS disabled by default
  _prefs.gps_interval = 0;      // No automatic GPS updates by default
  //_prefs.rx_delay_base = 10.0f;  enable once new algo fixed
#if defined(USE_SX1262) || defined(USE_SX1268)
#ifdef SX126X_RX_BOOSTED_GAIN
  _prefs.rx_boosted_gain = SX126X_RX_BOOSTED_GAIN;
#else
  _prefs.rx_boosted_gain = 1; // enabled by default
#endif
#endif
}

void MyMesh::begin(bool has_display) {
  BaseChatMesh::begin();

#ifdef WITH_MESH_SLIM
  // Register before anything can arrive. A slim frame received with no handler
  // is routed and repeated correctly but never reaches the operator, which
  // looks exactly like a radio fault and is not one.
  slimproto::setHandler(this);
#endif

  if (!_store->loadMainIdentity(self_id)) {
    self_id = radio_new_identity(); // create new random identity
    int count = 0;
    while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) { // reserved id hashes
      self_id = radio_new_identity();
      count++;
    }
    _store->saveMainIdentity(self_id);
  }

// if name is provided as a build flag, use that as default node name instead
#ifdef ADVERT_NAME
  strcpy(_prefs.node_name, ADVERT_NAME);
#else
  // use hex of first 4 bytes of identity public key as default node name
  char pub_key_hex[10];
  mesh::Utils::toHex(pub_key_hex, self_id.pub_key, 4);
  strcpy(_prefs.node_name, pub_key_hex);
#endif

  // if build provides default-scope, init with that
#ifdef DEFAULT_FLOOD_SCOPE_NAME
  strcpy(_prefs.default_scope_name, DEFAULT_FLOOD_SCOPE_NAME);
  {
    TransportKeyStore temp;
    TransportKey key;
    temp.getAutoKeyFor(0, "#" DEFAULT_FLOOD_SCOPE_NAME, key);
    memcpy(_prefs.default_scope_key, key.key, sizeof(key.key));
  }
#endif

  // load persisted prefs
  _store->loadPrefs(_prefs, sensors.node_lat, sensors.node_lon);

  // sanitise bad pref values
  _prefs.rx_delay_base = constrain(_prefs.rx_delay_base, 0, 20.0f);
  _prefs.airtime_factor = constrain(_prefs.airtime_factor, 0, 9.0f);
  _prefs.freq = constrain(_prefs.freq, 150.0f, 2500.0f);
  _prefs.bw = constrain(_prefs.bw, 7.8f, 500.0f);
  _prefs.sf = constrain(_prefs.sf, 5, 12);
  _prefs.cr = constrain(_prefs.cr, 5, 8);
  _prefs.tx_power_dbm = constrain(_prefs.tx_power_dbm, -9, MAX_LORA_TX_POWER);
  _prefs.gps_enabled = constrain(_prefs.gps_enabled, 0, 1);  // Ensure boolean 0 or 1
  _prefs.gps_interval = constrain(_prefs.gps_interval, 0, 86400);  // Max 24 hours

#ifdef BLE_PIN_CODE // 123456 by default
  if (_prefs.ble_pin == 0) {
#ifdef DISPLAY_CLASS
    if (has_display && BLE_PIN_CODE == 123456) {
      StdRNG rng;
      _active_ble_pin = rng.nextInt(100000, 999999); // random pin each session
    } else {
      _active_ble_pin = BLE_PIN_CODE; // otherwise static pin
    }
#else
    _active_ble_pin = BLE_PIN_CODE; // otherwise static pin
#endif
  } else {
    _active_ble_pin = _prefs.ble_pin;
  }
#else
  _active_ble_pin = 0;
#endif

  resetContacts();
  _store->loadContacts(this);
  bootstrapRTCfromContacts();
  addChannel("Public", PUBLIC_GROUP_PSK); // pre-configure Andy's public channel
  _store->loadChannels(this);

#ifdef WITH_MESH_SLIM
  // Contacts are loaded, so the callsigns that carry the unit numbers are
  // available. Positions and team chat never needed this - they are broadcast
  // and the unit number is the identity - but a direct message cannot be
  // addressed without it.
  loadSlimRoster();
  // The prefs are loaded by now, and the name is where the number comes from.
  loadSlimUnit();
  _slim_advert_at = futureMillis(SLIM_ADVERT_FIRST_MS);
#endif

  radio_driver.setParams(_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
#ifdef WITH_MESH_SLIM
  // The protocol the menu last saved, after setParams so it has the last
  // word on the radio. No file - every unit that has never used the menu -
  // changes nothing.
  loadSlimProto();
#endif
  radio_driver.setTxPower(_prefs.tx_power_dbm);
  radio_driver.setRxBoostedGainMode(_prefs.rx_boosted_gain);
  MESH_DEBUG_PRINTLN("RX Boosted Gain Mode: %s",
                     radio_driver.getRxBoostedGainMode() ? "Enabled" : "Disabled");
}

const char *MyMesh::getNodeName() {
  return _prefs.node_name;
}
NodePrefs *MyMesh::getNodePrefs() {
  return &_prefs;
}
uint32_t MyMesh::getBLEPin() {
  return _active_ble_pin;
}

struct FreqRange {
  uint32_t lower_freq, upper_freq;
};

static FreqRange repeat_freq_ranges[] = {
  #ifdef ALLOWED_REPEAT_FREQ_RANGE
  ALLOWED_REPEAT_FREQ_RANGE
  #else
  { 433000, 433000 },
  { 869495, 869495 },
  { 918000, 918000 }
  #endif
};

bool MyMesh::isValidClientRepeatFreq(uint32_t f) const {
  for (int i = 0; i < sizeof(repeat_freq_ranges)/sizeof(repeat_freq_ranges[0]); i++) {
    auto r = &repeat_freq_ranges[i];
    if (f >= r->lower_freq && f <= r->upper_freq) return true;
  }
  return false;
}

#ifdef MESH_SLIM_BLE_OFF
//: Whether Bluetooth is wanted, remembered across reboots.
//:
//: Its own file rather than a NodePrefs field: prefs are a flat struct
//: written at fixed offsets, so one new byte there would invalidate the
//: saved settings of every unit already carrying this firmware.
static const char* BLE_FILE = "/blewant";

void MyMesh::saveBleWanted(bool on) {
  File f = _store->openWrite(BLE_FILE);
  if (!f) return;
  const uint8_t v = on ? 1 : 0;
  f.write(&v, 1);
  f.close();
}

//: Off until asked on a tracker: a unit in a pack with no phone near it
//: spends the day advertising to nobody. Nothing here makes Bluetooth
//: unreachable - the BLUETOOTH page turns it on in one press, and that
//: choice is then what this reads back.
bool MyMesh::bleWanted() {
  File f = _store->openRead(BLE_FILE);
  if (!f) return false;                 // never asked: stay off
  uint8_t v = 0;
  const bool got = f.read(&v, 1) == 1;
  f.close();
  return got && v != 0;
}
#endif

void MyMesh::startInterface(BaseSerialInterface &serial) {
  _serial = &serial;
#ifdef MESH_SLIM_BLE_OFF
  if (bleWanted()) serial.enable(); else serial.disable();
#else
  serial.enable();
#endif
}

void MyMesh::handleCmdFrame(size_t len) {
#ifdef WITH_MESH_SLIM
  // Commands that build a MeshCore packet and have nowhere to go in slim mode.
  //
  // Left unguarded they are worse than useless: the slim codec strips the
  // MeshCore header on the way out, so each one goes on the air as a frame
  // that fails every receiver's tag check — spending airtime, and reporting
  // success to an operator who then waits for a reply that cannot come.
  //
  // Refused here rather than in each handler, because the list is the point:
  // these are the parts of MeshCore that MESH_SLIM does not carry.
  //
  // Two kinds of entry, and it matters which one is being read. Most have
  // nothing to say on this wire at all - a route is learned rather than
  // traced, a raw MeshCore packet is a raw MeshCore packet. But a command
  // can also leave this list by being ANSWERED rather than removed:
  // CMD_SEND_SELF_ADVERT now routes to sendSlimAdvert(), the same
  // question the button always asked, answered in the form slim carries.
  //
  // CMD_SEND_STATUS_REQ is the one that costs a feature. A slim status
  // exchange exists and the Pi already uses it (slim_health, and
  // repeater_status_slim on the gateway), so a repeater IS checkable on
  // slim - just not by a TABLET base, which reaches its node only through
  // this protocol. Routing it here is what would close that, and it needs
  // the reply matched back and translated into the status response shape
  // the app already parses. Named so the gap is a decision rather than a
  // symptom somebody rediscovers on a hill.
  if (slimproto::active()) {
    switch (cmd_frame[0]) {
      case CMD_SEND_LOGIN:
      case CMD_SEND_STATUS_REQ:
      case CMD_SEND_TELEMETRY_REQ:
      case CMD_SEND_BINARY_REQ:
      case CMD_SEND_ANON_REQ:
      case CMD_SEND_PATH_DISCOVERY_REQ:   // routes are learned, not discovered
      case CMD_SEND_TRACE_PATH:
      case CMD_SEND_RAW_DATA:
      case CMD_SEND_RAW_PACKET:
      case CMD_SEND_CONTROL_DATA:
      case CMD_SEND_CHANNEL_DATA:
        writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
        return;
      default:
        break;
    }
  }
#endif
  if (cmd_frame[0] == CMD_DEVICE_QUERY && len >= 2) { // sent when app establishes connection
    app_target_ver = cmd_frame[1];                    // which version of protocol does app understand

    int i = 0;
    out_frame[i++] = RESP_CODE_DEVICE_INFO;
    out_frame[i++] = FIRMWARE_VER_CODE;
    out_frame[i++] = MAX_CONTACTS / 2;   // v3+
    out_frame[i++] = MAX_GROUP_CHANNELS; // v3+
    memcpy(&out_frame[i], &_prefs.ble_pin, 4);
    i += 4;
    memset(&out_frame[i], 0, 12);
    strcpy((char *)&out_frame[i], FIRMWARE_BUILD_DATE);
    i += 12;
    StrHelper::strzcpy((char *)&out_frame[i], board.getManufacturerName(), 40);
    i += 40;
    StrHelper::strzcpy((char *)&out_frame[i], FIRMWARE_VERSION, 20);
    i += 20;
    out_frame[i++] = _prefs.client_repeat;   // v9+
    out_frame[i++] = _prefs.path_hash_mode;  // v10+
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_APP_START &&
             len >= 8) { // sent when app establishes connection, respond with node ID
    //  cmd_frame[1..7]  reserved future
    char *app_name = (char *)&cmd_frame[8];
    cmd_frame[len] = 0; // make app_name null terminated
    MESH_DEBUG_PRINTLN("App %s connected", app_name);

    _iter_started = false; // stop any left-over ContactsIterator
    int i = 0;
    out_frame[i++] = RESP_CODE_SELF_INFO;
    out_frame[i++] = ADV_TYPE_CHAT; // what this node Advert identifies as (maybe node's pronouns too?? :-)
    out_frame[i++] = _prefs.tx_power_dbm;
    out_frame[i++] = MAX_LORA_TX_POWER;
    memcpy(&out_frame[i], self_id.pub_key, PUB_KEY_SIZE);
    i += PUB_KEY_SIZE;

    int32_t lat, lon;
    lat = (sensors.node_lat * 1000000.0);
    lon = (sensors.node_lon * 1000000.0);
    memcpy(&out_frame[i], &lat, 4);
    i += 4;
    memcpy(&out_frame[i], &lon, 4);
    i += 4;
    out_frame[i++] = _prefs.multi_acks; // new v7+
    out_frame[i++] = _prefs.advert_loc_policy;
    out_frame[i++] = (_prefs.telemetry_mode_env << 4) | (_prefs.telemetry_mode_loc << 2) |
                     (_prefs.telemetry_mode_base); // v5+
    out_frame[i++] = _prefs.manual_add_contacts;

    uint32_t freq = _prefs.freq * 1000;
    memcpy(&out_frame[i], &freq, 4);
    i += 4;
    uint32_t bw = _prefs.bw * 1000;
    memcpy(&out_frame[i], &bw, 4);
    i += 4;
    out_frame[i++] = _prefs.sf;
    out_frame[i++] = _prefs.cr;

    int tlen = strlen(_prefs.node_name); // revisit: UTF_8 ??
    memcpy(&out_frame[i], _prefs.node_name, tlen);
    i += tlen;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_SEND_TXT_MSG && len >= 14) {
#ifdef WITH_MESH_SLIM
    if (slimproto::active()) {
      // The app addresses a contact by the first 6 bytes of its public key.
      // The wire addresses it by a 4-bit unit number. The map between them is
      // local and costs nothing on air, so neither side has to change.
      const uint8_t* prefix = &cmd_frame[8];       // after type, attempt, ts
      const uint8_t unit = slimproto::unitForPubKey(prefix, 6);
      if (unit == slimproto::NO_UNIT) {
        writeErrFrame(ERR_CODE_NOT_FOUND);         // not on the briefing roster
        return;
      }
      char* text = (char*)&cmd_frame[14];
      cmd_frame[len] = 0;                          // ensure null
      uint8_t body[slim::MAX_BODY];
      const int blen = slimapp::bodyFor(text, body, sizeof(body));
      if (blen < 0) {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);       // too long for one frame
        return;
      }
      // The sequence number the frame will carry, known before it is built
      // because the router hands them out in order.
      const uint8_t seq = (uint8_t)((slimproto::router().peekNextSeq()) & 0xFF);
      if (!sendSlimBurst(slim::TYPE_TEXT, body, (uint8_t)blen, unit)) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
        return;
      }
      // Both ends derive this from (unit, seq), so the ack itself carries only
      // the sequence byte. See slimcompanion::ackId.
      uint32_t expected_ack = slimcompanion::ackId(unit, seq);
      expected_ack_table[next_ack_idx].msg_sent = _ms->getMillis();
      expected_ack_table[next_ack_idx].ack = expected_ack;
      expected_ack_table[next_ack_idx].contact = lookupContactByPubKey(prefix, 6);
      next_ack_idx = (next_ack_idx + 1) % EXPECTED_ACK_TABLE_SIZE;

      uint32_t est_timeout = 8000;
      out_frame[0] = RESP_CODE_SENT;
      out_frame[1] = 1;                            // went out as a flood
      memcpy(&out_frame[2], &expected_ack, 4);
      memcpy(&out_frame[6], &est_timeout, 4);
      _serial->writeFrame(out_frame, 10);
      return;
    }
#endif
    int i = 1;
    uint8_t txt_type = cmd_frame[i++];
    uint8_t attempt = cmd_frame[i++];
    uint32_t msg_timestamp;
    memcpy(&msg_timestamp, &cmd_frame[i], 4);
    i += 4;
    uint8_t *pub_key_prefix = &cmd_frame[i];
    i += 6;
    ContactInfo *recipient = lookupContactByPubKey(pub_key_prefix, 6);
    if (recipient && (txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_CLI_DATA)) {
      char *text = (char *)&cmd_frame[i];
      int tlen = len - i;
      uint32_t est_timeout;
      text[tlen] = 0; // ensure null
      int result;
      uint32_t expected_ack;
      if (txt_type == TXT_TYPE_CLI_DATA) {
        msg_timestamp = getRTCClock()->getCurrentTimeUnique(); // Use node's RTC instead of app timestamp to avoid tripping replay protection
        result = sendCommandData(*recipient, msg_timestamp, attempt, text, est_timeout);
        expected_ack = 0; // no Ack expected
      } else {
        result = sendMessage(*recipient, msg_timestamp, attempt, text, expected_ack, est_timeout);
      }
      // TODO: add expected ACK to table
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        if (expected_ack) {
          expected_ack_table[next_ack_idx].msg_sent = _ms->getMillis(); // add to circular table
          expected_ack_table[next_ack_idx].ack = expected_ack;
          expected_ack_table[next_ack_idx].contact = recipient;
          next_ack_idx = (next_ack_idx + 1) % EXPECTED_ACK_TABLE_SIZE;
        }

        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &expected_ack, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(recipient == NULL
                        ? ERR_CODE_NOT_FOUND
                        : ERR_CODE_UNSUPPORTED_CMD); // unknown recipient, or unsupported TXT_TYPE_*
    }
  } else if (cmd_frame[0] == CMD_SEND_CHANNEL_TXT_MSG) { // send GroupChannel text msg
    int i = 1;
    uint8_t txt_type = cmd_frame[i++]; // should be TXT_TYPE_PLAIN
    uint8_t channel_idx = cmd_frame[i++];
    uint32_t msg_timestamp;
    memcpy(&msg_timestamp, &cmd_frame[i], 4);
    i += 4;
    const char *text = (char *)&cmd_frame[i];
#ifdef WITH_MESH_SLIM
    // The stock path below passes an explicit length and never needs this, but
    // the slim path hands `text` to strstr/strlen. Without a terminator those
    // read on into whatever the previous command left in cmd_frame.
    cmd_frame[len] = 0;
#endif

    if (txt_type != TXT_TYPE_PLAIN) {
      writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
#ifdef WITH_MESH_SLIM
    } else if (slimproto::active()) {
      // One decode decides which of the two paths this takes. A position goes
      // on the air as a 13-byte frame instead of ~40 bytes of encrypted channel
      // text; everything else - human chat, "!U" urgent, a base roster, which
      // stays text because only the base holds the whole picture - goes as
      // broadcast text.
      uint8_t role, unit, body[slim::MAX_BODY];
      int blen;
      uint8_t type;
      if (slimcompanion::readSelfReport(text, role, unit, body)) {
        type = slim::TYPE_POS;
        blen = slimcodec::POS_BYTES;
        // The role travels with the sender, so record it for when this unit's
        // position comes back the other way and has to be labelled.
        slimcompanion::setUnitRole(slimproto::unit(), role);
      } else {
        type = slim::TYPE_GRPTEXT;
        blen = slimapp::bodyFor(slimcompanion::skipSenderPrefix(text),
                               body, sizeof(body));
      }
      if (blen < 0) {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);        // too long for one frame
      } else if (sendSlimBurst(type, body, (uint8_t)blen,
                               slim::ADDR_BROADCAST)) {
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      }
#endif
    } else {
      ChannelDetails channel;
      bool success = getChannel(channel_idx, channel);
      if (success && sendGroupMessage(msg_timestamp, channel.channel, _prefs.node_name, text, len - i)) {
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_NOT_FOUND); // bad channel_idx
      }
    }
  } else if (cmd_frame[0] == CMD_SEND_CHANNEL_DATA) { // send GroupChannel datagram
    if (len < 4) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      return;
    }
    int i = 1;
    uint8_t channel_idx = cmd_frame[i++];
    uint8_t path_len = cmd_frame[i++];

    // validate path len, allowing 0xFF for flood
    if (!mesh::Packet::isValidPathLen(path_len) && path_len != OUT_PATH_UNKNOWN) {
      MESH_DEBUG_PRINTLN("CMD_SEND_CHANNEL_DATA invalid path size: %d", path_len);
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      return;
    }

    // parse provided path if not flood
    uint8_t path[MAX_PATH_SIZE];
    if (path_len != OUT_PATH_UNKNOWN) {
      i += mesh::Packet::writePath(path, &cmd_frame[i], path_len);
    }

    uint16_t data_type = ((uint16_t)cmd_frame[i]) | (((uint16_t)cmd_frame[i + 1]) << 8);
    i += 2;
    const uint8_t *payload = &cmd_frame[i];
    int payload_len = (len > (size_t)i) ? (int)(len - i) : 0;

    ChannelDetails channel;
    if (!getChannel(channel_idx, channel)) {
      writeErrFrame(ERR_CODE_NOT_FOUND); // bad channel_idx
    } else if (data_type == DATA_TYPE_RESERVED) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (payload_len > MAX_CHANNEL_DATA_LENGTH) {
      MESH_DEBUG_PRINTLN("CMD_SEND_CHANNEL_DATA payload too long: %d > %d", payload_len, MAX_CHANNEL_DATA_LENGTH);
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (sendGroupData(channel.channel, path, path_len, data_type, payload, payload_len)) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else if (cmd_frame[0] == CMD_GET_CONTACTS) { // get Contact list
    if (_iter_started) {
      writeErrFrame(ERR_CODE_BAD_STATE); // iterator is currently busy
    } else {
      if (len >= 5) { // has optional 'since' param
        memcpy(&_iter_filter_since, &cmd_frame[1], 4);
      } else {
        _iter_filter_since = 0;
      }

      uint8_t reply[5];
      reply[0] = RESP_CODE_CONTACTS_START;
      uint32_t count = getNumContacts(); // total, NOT filtered count
      memcpy(&reply[1], &count, 4);
      _serial->writeFrame(reply, 5);

      // start iterator
      _iter = startContactsIterator();
      _iter_started = true;
      _most_recent_lastmod = 0;
    }
  } else if (cmd_frame[0] == CMD_SET_ADVERT_NAME && len >= 2) {
    int nlen = len - 1;
    if (nlen > sizeof(_prefs.node_name) - 1) nlen = sizeof(_prefs.node_name) - 1; // max len
    memcpy(_prefs.node_name, &cmd_frame[1], nlen);
    _prefs.node_name[nlen] = 0; // null terminator
    savePrefs();
#ifdef WITH_MESH_SLIM
    loadSlimUnit();   // MTRK07 is unit 7 on slim
#endif
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_ADVERT_LATLON && len >= 9) {
    int32_t lat, lon, alt = 0;
    memcpy(&lat, &cmd_frame[1], 4);
    memcpy(&lon, &cmd_frame[5], 4);
    if (len >= 13) {
      memcpy(&alt, &cmd_frame[9], 4); // for FUTURE support
    }
    if (lat <= 90 * 1E6 && lat >= -90 * 1E6 && lon <= 180 * 1E6 && lon >= -180 * 1E6) {
      sensors.node_lat = ((double)lat) / 1000000.0;
      sensors.node_lon = ((double)lon) / 1000000.0;
      savePrefs();
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG); // invalid geo coordinate
    }
  } else if (cmd_frame[0] == CMD_GET_DEVICE_TIME) {
    uint8_t reply[5];
    reply[0] = RESP_CODE_CURR_TIME;
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(&reply[1], &now, 4);
    _serial->writeFrame(reply, 5);
  } else if (cmd_frame[0] == CMD_SET_DEVICE_TIME && len >= 5) {
    uint32_t secs;
    memcpy(&secs, &cmd_frame[1], 4);
    uint32_t curr = getRTCClock()->getCurrentTime();
    if (secs >= curr) {
      getRTCClock()->setCurrentTime(secs);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
#ifdef WITH_MESH_SLIM
  } else if (cmd_frame[0] == CMD_SEND_SELF_ADVERT && slimproto::active()) {
    // On slim an advert is "!V": the same question the button has always
    // asked - who is out there - answered in the one form this wire can
    // carry. Guarded, because a stock build has no slimproto at all and
    // must come out byte-identical to upstream.
    if (sendSlimAdvert()) writeOKFrame(); else writeErrFrame(ERR_CODE_TABLE_FULL);
#endif
  } else if (cmd_frame[0] == CMD_SEND_SELF_ADVERT) {
    mesh::Packet* pkt;
    if (_prefs.advert_loc_policy == ADVERT_LOC_NONE) {
      pkt = createSelfAdvert(_prefs.node_name);
    } else {
      pkt = createSelfAdvert(_prefs.node_name, sensors.node_lat, sensors.node_lon);
    }
    if (pkt) {
      if (len >= 2 && cmd_frame[1] == 1) { // optional param (1 = flood, 0 = zero hop)
        unsigned long delay_millis = 0;
        TransportKey default_scope;
        memcpy(&default_scope.key, _prefs.default_scope_key, sizeof(default_scope.key));
        sendFloodScoped(default_scope, pkt, delay_millis);
      } else {
        sendZeroHop(pkt);
      }
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else if (cmd_frame[0] == CMD_RESET_PATH && len >= 1 + 32) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      recipient->out_path_len = OUT_PATH_UNKNOWN;
      // recipient->lastmod = ??   shouldn't be needed, app already has this version of contact
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // unknown contact
    }
  } else if (cmd_frame[0] == CMD_ADD_UPDATE_CONTACT && len >= 1 + 32 + 2 + 1) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    uint32_t last_mod = getRTCClock()->getCurrentTime();  // fallback value if not present in cmd_frame
    if (recipient) {
      updateContactFromFrame(*recipient, last_mod, cmd_frame, len);
      recipient->lastmod = last_mod;
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
      writeOKFrame();
    } else {
      ContactInfo contact;
      updateContactFromFrame(contact, last_mod, cmd_frame, len);
      contact.lastmod = last_mod;
      contact.sync_since = 0;
      if (addContact(contact)) {
        dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      }
    }
  } else if (cmd_frame[0] == CMD_REMOVE_CONTACT) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient && removeContact(*recipient)) {
      _store->deleteBlobByKey(pub_key, PUB_KEY_SIZE);
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // not found, or unable to remove
    }
  } else if (cmd_frame[0] == CMD_SHARE_CONTACT) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      if (shareContactZeroHop(*recipient)) {
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL); // unable to send
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_GET_CONTACT_BY_KEY) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *contact = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (contact) {
      writeContactRespFrame(RESP_CODE_CONTACT, *contact);
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // not found
    }
  } else if (cmd_frame[0] == CMD_EXPORT_CONTACT) {
    if (len < 1 + PUB_KEY_SIZE) {
      // export SELF
      mesh::Packet* pkt;
      if (_prefs.advert_loc_policy == ADVERT_LOC_NONE) {
        pkt = createSelfAdvert(_prefs.node_name);
      } else {
        pkt = createSelfAdvert(_prefs.node_name, sensors.node_lat, sensors.node_lon);
      }
      if (pkt) {
        pkt->header |= ROUTE_TYPE_FLOOD; // would normally be sent in this mode

        out_frame[0] = RESP_CODE_EXPORT_CONTACT;
        uint8_t out_len = pkt->writeTo(&out_frame[1]);
        releasePacket(pkt); // undo the obtainNewPacket()
        _serial->writeFrame(out_frame, out_len + 1);
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL); // Error
      }
    } else {
      uint8_t *pub_key = &cmd_frame[1];
      ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
      uint8_t out_len;
      if (recipient && (out_len = exportContact(*recipient, &out_frame[1])) > 0) {
        out_frame[0] = RESP_CODE_EXPORT_CONTACT;
        _serial->writeFrame(out_frame, out_len + 1);
      } else {
        writeErrFrame(ERR_CODE_NOT_FOUND); // not found
      }
    }
  } else if (cmd_frame[0] == CMD_IMPORT_CONTACT && len > 2 + 32 + 64) {
    if (importContact(&cmd_frame[1], len - 1)) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_SYNC_NEXT_MESSAGE) {
    int out_len;
    if ((out_len = getFromOfflineQueue(out_frame)) > 0) {
      _serial->writeFrame(out_frame, out_len);
#ifdef DISPLAY_CLASS
      if (_ui) _ui->msgRead(offline_queue_len);
#endif
    } else {
      out_frame[0] = RESP_CODE_NO_MORE_MESSAGES;
      _serial->writeFrame(out_frame, 1);
    }
  } else if (cmd_frame[0] == CMD_SET_RADIO_PARAMS) {
    int i = 1;
    uint32_t freq;
    memcpy(&freq, &cmd_frame[i], 4);
    i += 4;
    uint32_t bw;
    memcpy(&bw, &cmd_frame[i], 4);
    i += 4;
    uint8_t sf = cmd_frame[i++];
    uint8_t cr = cmd_frame[i++];
    uint8_t repeat = 0;  // default - false
    if (len > i) {
      repeat = cmd_frame[i++];   // FIRMWARE_VER_CODE  9+
    }

    if (repeat && !isValidClientRepeatFreq(freq)) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (freq >= 150000 && freq <= 2500000 && sf >= 5 && sf <= 12 && cr >= 5 && cr <= 8 && bw >= 7000 &&
        bw <= 500000) {
      _prefs.sf = sf;
      _prefs.cr = cr;
      _prefs.freq = (float)freq / 1000.0;
      _prefs.bw = (float)bw / 1000.0;
      _prefs.client_repeat = repeat;
      savePrefs();

      radio_driver.setParams(_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
      MESH_DEBUG_PRINTLN("OK: CMD_SET_RADIO_PARAMS: f=%d, bw=%d, sf=%d, cr=%d", freq, bw, (uint32_t)sf,
                         (uint32_t)cr);

      writeOKFrame();
    } else {
      MESH_DEBUG_PRINTLN("Error: CMD_SET_RADIO_PARAMS: f=%d, bw=%d, sf=%d, cr=%d", freq, bw, (uint32_t)sf,
                         (uint32_t)cr);
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_SET_RADIO_TX_POWER) {
    int8_t power = (int8_t)cmd_frame[1];
    if (power < -9 || power > MAX_LORA_TX_POWER) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      _prefs.tx_power_dbm = power;
      savePrefs();
      radio_driver.setTxPower(_prefs.tx_power_dbm);
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_SET_TUNING_PARAMS) {
    int i = 1;
    uint32_t rx, af;
    memcpy(&rx, &cmd_frame[i], 4);
    i += 4;
    memcpy(&af, &cmd_frame[i], 4);
    i += 4;
    _prefs.rx_delay_base = ((float)rx) / 1000.0f;
    _prefs.airtime_factor = ((float)af) / 1000.0f;
    savePrefs();
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_GET_TUNING_PARAMS) {
    uint32_t rx = _prefs.rx_delay_base * 1000, af = _prefs.airtime_factor * 1000;
    int i = 0;
    out_frame[i++] = RESP_CODE_TUNING_PARAMS;
    memcpy(&out_frame[i], &rx, 4); i += 4;
    memcpy(&out_frame[i], &af, 4); i += 4;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_SET_OTHER_PARAMS) {
    _prefs.manual_add_contacts = cmd_frame[1];
    if (len >= 3) {
      _prefs.telemetry_mode_base = cmd_frame[2] & 0x03; // v5+
      _prefs.telemetry_mode_loc = (cmd_frame[2] >> 2) & 0x03;
      _prefs.telemetry_mode_env = (cmd_frame[2] >> 4) & 0x03;

      if (len >= 4) {
        _prefs.advert_loc_policy = cmd_frame[3];
        if (len >= 5) {
          _prefs.multi_acks = cmd_frame[4];
        }
      }
    }
    savePrefs();
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_PATH_HASH_MODE && cmd_frame[1] == 0 && len >= 3) {
    if (cmd_frame[2] >= 3) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      _prefs.path_hash_mode = cmd_frame[2];
      savePrefs();
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_REBOOT && memcmp(&cmd_frame[1], "reboot", 6) == 0) {
    if (dirty_contacts_expiry) { // is there are pending dirty contacts write needed?
      saveContacts();
    }
    board.reboot();
  } else if (cmd_frame[0] == CMD_GET_BATT_AND_STORAGE) {
    uint8_t reply[11];
    int i = 0;
    reply[i++] = RESP_CODE_BATT_AND_STORAGE;
    uint16_t battery_millivolts = board.getBattMilliVolts();
    uint32_t used = _store->getStorageUsedKb();
    uint32_t total = _store->getStorageTotalKb();
    memcpy(&reply[i], &battery_millivolts, 2); i += 2;
    memcpy(&reply[i], &used, 4); i += 4;
    memcpy(&reply[i], &total, 4); i += 4;
    _serial->writeFrame(reply, i);
  } else if (cmd_frame[0] == CMD_EXPORT_PRIVATE_KEY) {
#if ENABLE_PRIVATE_KEY_EXPORT
    uint8_t reply[65];
    reply[0] = RESP_CODE_PRIVATE_KEY;
    self_id.writeTo(&reply[1], 64);
    _serial->writeFrame(reply, 65);
#else
    writeDisabledFrame();
#endif
  } else if (cmd_frame[0] == CMD_IMPORT_PRIVATE_KEY && len >= 65) {
#if ENABLE_PRIVATE_KEY_IMPORT
    if (!mesh::LocalIdentity::validatePrivateKey(&cmd_frame[1])) {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG); // invalid key
    } else {
        mesh::LocalIdentity identity;
        identity.readFrom(&cmd_frame[1], 64);
        if (_store->saveMainIdentity(identity)) {
          self_id = identity;
          writeOKFrame();
          // re-load contacts, to invalidate ecdh shared_secrets
          resetContacts();
          _store->loadContacts(this);
        } else {
          writeErrFrame(ERR_CODE_FILE_IO_ERROR);
        }
    }
#else
    writeDisabledFrame();
#endif
  } else if (cmd_frame[0] == CMD_SEND_RAW_DATA && len >= 6) {
    int i = 1;
    int8_t path_len = cmd_frame[i++];
    if (path_len >= 0 && i + path_len + 4 <= len) { // minimum 4 byte payload
      uint8_t *path = &cmd_frame[i];
      i += path_len;
      auto pkt = createRawData(&cmd_frame[i], len - i);
      if (pkt) {
        sendDirect(pkt, path, path_len);
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      }
    } else {
      writeErrFrame(ERR_CODE_UNSUPPORTED_CMD); // flood, not supported (yet)
    }
  } else if (cmd_frame[0] == CMD_SEND_LOGIN && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    char *password = (char *)&cmd_frame[1 + PUB_KEY_SIZE];
    cmd_frame[len] = 0; // ensure null terminator in password
    if (recipient) {
      uint32_t est_timeout;
      int result = sendLogin(*recipient, password, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        memcpy(&pending_login, recipient->id.pub_key, 4); // match this to onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &pending_login, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_ANON_REQ && len > 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    ContactInfo anon;
    if (recipient == NULL) { // FIRMWARE_VER_CODE 13+,  allow non-contact requests
      memset(&anon, 0, sizeof(anon));
      memcpy(anon.id.pub_key, pub_key, PUB_KEY_SIZE);
      anon.out_path_len = 0;   // default to zero-hop direct
      anon.type = ADV_TYPE_NONE;  // unknown

      if (addContact(anon)) recipient = &anon;
    }
    uint8_t *data = &cmd_frame[1 + PUB_KEY_SIZE];
    if (recipient) {
      uint32_t tag, est_timeout;
      int result = sendAnonReq(*recipient, data, len - (1 + PUB_KEY_SIZE), tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_req = tag; // match this to onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL); // contacts full
    }
  } else if (cmd_frame[0] == CMD_SEND_STATUS_REQ && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint32_t tag, est_timeout;
      int result = sendRequest(*recipient, REQ_TYPE_GET_STATUS, tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        // FUTURE:  pending_status = tag;  // match this in onContactResponse()
        memcpy(&pending_status, recipient->id.pub_key, 4); // legacy matching scheme
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_PATH_DISCOVERY_REQ && cmd_frame[1] == 0 && len >= 2 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[2];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint32_t tag, est_timeout;
      // 'Path Discovery' is just a special case of flood + Telemetry req
      uint8_t req_data[9];
      req_data[0] = REQ_TYPE_GET_TELEMETRY_DATA;
      req_data[1] = ~(TELEM_PERM_BASE);  // NEW: inverse permissions mask (ie. we only want BASE telemetry)
      memset(&req_data[2], 0, 3);  // reserved
      getRNG()->random(&req_data[5], 4);   // random blob to help make packet-hash unique
      auto save = recipient->out_path_len;    // temporarily force sendRequest() to flood
      recipient->out_path_len = OUT_PATH_UNKNOWN;
      int result = sendRequest(*recipient, req_data, sizeof(req_data), tag, est_timeout);
      recipient->out_path_len = save;
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_discovery = tag; // match this in onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_TELEMETRY_REQ && len >= 4 + PUB_KEY_SIZE) {  // can deprecate, in favour of CMD_SEND_BINARY_REQ
    uint8_t *pub_key = &cmd_frame[4];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint32_t tag, est_timeout;
      int result = sendRequest(*recipient, REQ_TYPE_GET_TELEMETRY_DATA, tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_telemetry = tag; // match this in onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_TELEMETRY_REQ && len == 4) {  // 'self' telemetry request
    telemetry.reset();
    telemetry.addVoltage(TELEM_CHANNEL_SELF, (float)board.getBattMilliVolts() / 1000.0f);
    // query other sensors -- target specific
    sensors.querySensors(0xFF, telemetry);

    int i = 0;
    out_frame[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
    out_frame[i++] = 0; // reserved
    memcpy(&out_frame[i], self_id.pub_key, 6);
    i += 6; // pub_key_prefix
    uint8_t tlen = telemetry.getSize();
    memcpy(&out_frame[i], telemetry.getBuffer(), tlen);
    i += tlen;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_SEND_BINARY_REQ && len >= 2 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint8_t *req_data = &cmd_frame[1 + PUB_KEY_SIZE];
      uint32_t tag, est_timeout;
      int result = sendRequest(*recipient, req_data, len - (1 + PUB_KEY_SIZE), tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_req = tag; // match this in onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_HAS_CONNECTION && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    if (hasConnectionTo(pub_key)) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_LOGOUT && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    stopConnection(pub_key);
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_GET_CHANNEL && len >= 2) {
    uint8_t channel_idx = cmd_frame[1];
    ChannelDetails channel;
    if (getChannel(channel_idx, channel)) {
      int i = 0;
      out_frame[i++] = RESP_CODE_CHANNEL_INFO;
      out_frame[i++] = channel_idx;
      strcpy((char *)&out_frame[i], channel.name);
      i += 32;
      memcpy(&out_frame[i], channel.channel.secret, 16);
      i += 16; // NOTE: only 128-bit supported
      _serial->writeFrame(out_frame, i);
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_SET_CHANNEL && len >= 2 + 32 + 32) {
    writeErrFrame(ERR_CODE_UNSUPPORTED_CMD); // not supported (yet)
  } else if (cmd_frame[0] == CMD_SET_CHANNEL && len >= 2 + 32 + 16) {
    uint8_t channel_idx = cmd_frame[1];
    ChannelDetails channel;
    StrHelper::strncpy(channel.name, (char *)&cmd_frame[2], 32);
    memset(channel.channel.secret, 0, sizeof(channel.channel.secret));
    memcpy(channel.channel.secret, &cmd_frame[2 + 32], 16); // NOTE: only 128-bit supported
    if (setChannel(channel_idx, channel)) {
      saveChannels();
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // bad channel_idx
    }
  } else if (cmd_frame[0] == CMD_SIGN_START) {
    out_frame[0] = RESP_CODE_SIGN_START;
    out_frame[1] = 0; // reserved
    uint32_t len = MAX_SIGN_DATA_LEN;
    memcpy(&out_frame[2], &len, 4);
    _serial->writeFrame(out_frame, 6);

    if (sign_data) {
      free(sign_data);
    }
    sign_data = (uint8_t *)malloc(MAX_SIGN_DATA_LEN);
    sign_data_len = 0;
  } else if (cmd_frame[0] == CMD_SIGN_DATA && len > 1) {
    if (sign_data == NULL || sign_data_len + (len - 1) > MAX_SIGN_DATA_LEN) {
      writeErrFrame(sign_data == NULL ? ERR_CODE_BAD_STATE : ERR_CODE_TABLE_FULL); // error: too long
    } else {
      memcpy(&sign_data[sign_data_len], &cmd_frame[1], len - 1);
      sign_data_len += (len - 1);
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_SIGN_FINISH) {
    if (sign_data) {
      self_id.sign(&out_frame[1], sign_data, sign_data_len);

      free(sign_data); // don't need sign_data now
      sign_data = NULL;

      out_frame[0] = RESP_CODE_SIGNATURE;
      _serial->writeFrame(out_frame, 1 + SIGNATURE_SIZE);
    } else {
      writeErrFrame(ERR_CODE_BAD_STATE);
    }
  } else if (cmd_frame[0] == CMD_SEND_TRACE_PATH && len > 10 && len - 10 < MAX_PACKET_PAYLOAD-5) {
    uint8_t path_len = len - 10;
    uint8_t flags = cmd_frame[9];
    uint8_t path_sz = flags & 0x03;  // NEW v1.11+
    if ((path_len >> path_sz) > MAX_PATH_SIZE || (path_len % (1 << path_sz)) != 0) { // make sure is multiple of path_sz
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      uint32_t tag, auth;
      memcpy(&tag, &cmd_frame[1], 4);
      memcpy(&auth, &cmd_frame[5], 4);
      auto pkt = createTrace(tag, auth, flags);
      if (pkt) {
        sendDirect(pkt, &cmd_frame[10], path_len);

        uint32_t t = _radio->getEstAirtimeFor(pkt->payload_len + pkt->path_len + 2);
        uint32_t est_timeout = calcDirectTimeoutMillisFor(t, path_len >> path_sz);

        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      }
    }
  } else if (cmd_frame[0] == CMD_SET_DEVICE_PIN && len >= 5) {

    // get pin from command frame
    uint32_t pin;
    memcpy(&pin, &cmd_frame[1], 4);

    // ensure pin is zero, or a valid 6 digit pin
    if (pin == 0 || (pin >= 100000 && pin <= 999999)) {
      _prefs.ble_pin = pin;
      savePrefs();
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_GET_CUSTOM_VARS) {
    out_frame[0] = RESP_CODE_CUSTOM_VARS;
    char *dp = (char *)&out_frame[1];
    for (int i = 0; i < sensors.getNumSettings() && dp - (char *)&out_frame[1] < 140; i++) {
      if (i > 0) {
        *dp++ = ',';
      }
      strcpy(dp, sensors.getSettingName(i));
      dp = strchr(dp, 0);
      *dp++ = ':';
      strcpy(dp, sensors.getSettingValue(i));
      dp = strchr(dp, 0);
    }
#ifdef WITH_TRACKER_MODE
    // Listed with the sensors' own settings, so any app that shows custom
    // variables - the official one does - can read and set it unchanged.
    if (dp > (char *)&out_frame[1]) *dp++ = ',';
    dp += sprintf(dp, "tracker:%lu", (unsigned long)_tracker.interval());
#endif
#ifdef WITH_MESH_SLIM
    // What has been heard, and what became of it. Beside `proto` because
    // the app already reads this frame on every connect, so a base gets the
    // answer to "is anything reaching me?" with no new command and no new
    // round trip. Reset when the node restarts, like every other counter.
    {
      const slimproto::Counts& n = slimproto::counts();
      if (dp > (char *)&out_frame[1]) *dp++ = ',';
      dp += sprintf(dp, "slimrx:%lu/%lu/%lu/%lu",
                    (unsigned long)n.seen, (unsigned long)n.bad,
                    (unsigned long)n.dup, (unsigned long)n.delivered);
    }
    // The protocol this node is on, for an app that drives it as a base: the
    // tablet reads it here and moves it with "proto:1" / "proto:0".
    if (slimproto::canSwitch()) {
      if (dp > (char *)&out_frame[1]) *dp++ = ',';
      dp += sprintf(dp, "proto:%d", slimproto::active() ? 1 : 0);
    }
#endif
    _serial->writeFrame(out_frame, dp - (char *)out_frame);
  } else if (cmd_frame[0] == CMD_SET_CUSTOM_VAR && len >= 4) {
    cmd_frame[len] = 0;
    char *sp = (char *)&cmd_frame[1];
    char *np = strchr(sp, ':'); // look for separator char
    if (np) {
      *np++ = 0; // modify 'cmd_frame', replace ':' with null
#ifdef WITH_MESH_SLIM
      // The node's own protocol, from the app driving it - a tablet acting as
      // the base has no other way to move its node, and the base moves last.
      // Answered before the restart, which waits for a quiet queue.
      if (strcmp(sp, "proto") == 0) {
        const bool on = np[0] == '1';
        if ((np[0] != '0' && np[0] != '1') || np[1] != 0 || !slimproto::canSwitch()) {
          writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        } else if (on == slimproto::active() || saveSlimProto(on)) {
          if (on != slimproto::active()) {
            _proto_restart_at = futureMillis(PROTO_RESTART_QUIET_MS);
            _proto_restart_by = futureMillis(PROTO_RESTART_MAX_MS);
          }
          writeOKFrame();
        } else {
          writeErrFrame(ERR_CODE_ILLEGAL_ARG);    // no key: slim would be deaf
        }
        return;
      }
#endif
#ifdef WITH_TRACKER_MODE
      if (strcmp(sp, "round") == 0) {
        if (setRound(np)) writeOKFrame();
        else writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        return;
      }
      if (strcmp(sp, "tracker") == 0) {
        char* end;
        const unsigned long secs = strtoul(np, &end, 10);
        if (end != np && *end == 0 && setTrackerMode((uint32_t)secs)) {
          writeOKFrame();
        } else {
          writeErrFrame(ERR_CODE_ILLEGAL_ARG);   // not a number, out of range, or no GPS
        }
        return;
      }
#endif
      bool success = sensors.setSettingValue(sp, np);
      if (success) {
        #if ENV_INCLUDE_GPS == 1
        // Update node preferences for GPS settings
        if (strcmp(sp, "gps") == 0) {
          _prefs.gps_enabled = (np[0] == '1') ? 1 : 0;
          savePrefs();
        } else if (strcmp(sp, "gps_interval") == 0) {
          uint32_t interval_seconds = atoi(np);
          _prefs.gps_interval = constrain(interval_seconds, 0, 86400);
          savePrefs();
        }
        #endif
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      }
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_GET_ADVERT_PATH && len >= PUB_KEY_SIZE+2) {
    // FUTURE use:  uint8_t reserved = cmd_frame[1];
    uint8_t *pub_key = &cmd_frame[2];
    AdvertPath* found = NULL;
    for (int i = 0; i < ADVERT_PATH_TABLE_SIZE; i++) {
      auto p = &advert_paths[i];
      if (memcmp(p->pubkey_prefix, pub_key, sizeof(p->pubkey_prefix)) == 0) {
        found = p;
        break;
      }
    }
    if (found) {
      int i = 0;
      out_frame[i++] = RESP_CODE_ADVERT_PATH;
      memcpy(&out_frame[i], &found->recv_timestamp, 4); i += 4;
      out_frame[i++] = found->path_len;
      i += mesh::Packet::writePath(&out_frame[i], found->path, found->path_len);
      _serial->writeFrame(out_frame, i);
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_GET_STATS && len >= 2) {
    uint8_t stats_type = cmd_frame[1];
    if (stats_type == STATS_TYPE_CORE) {
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = STATS_TYPE_CORE;
      uint16_t battery_mv = board.getBattMilliVolts();
      uint32_t uptime_secs = _ms->getMillis() / 1000;
      uint8_t queue_len = (uint8_t)_mgr->getOutboundTotal();
      memcpy(&out_frame[i], &battery_mv, 2); i += 2;
      memcpy(&out_frame[i], &uptime_secs, 4); i += 4;
      memcpy(&out_frame[i], &_err_flags, 2); i += 2;
      out_frame[i++] = queue_len;
      _serial->writeFrame(out_frame, i);
    } else if (stats_type == STATS_TYPE_RADIO) {
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = STATS_TYPE_RADIO;
      int16_t noise_floor = (int16_t)_radio->getNoiseFloor();
      int8_t last_rssi = (int8_t)radio_driver.getLastRSSI();
      int8_t last_snr = (int8_t)(radio_driver.getLastSNR() * 4); // scaled by 4 for 0.25 dB precision
      uint32_t tx_air_secs = getTotalAirTime() / 1000;
      uint32_t rx_air_secs = getReceiveAirTime() / 1000;
      memcpy(&out_frame[i], &noise_floor, 2); i += 2;
      out_frame[i++] = last_rssi;
      out_frame[i++] = last_snr;
      memcpy(&out_frame[i], &tx_air_secs, 4); i += 4;
      memcpy(&out_frame[i], &rx_air_secs, 4); i += 4;
      _serial->writeFrame(out_frame, i);
    } else if (stats_type == STATS_TYPE_PACKETS) {
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = STATS_TYPE_PACKETS;
      uint32_t recv = radio_driver.getPacketsRecv();
      uint32_t sent = radio_driver.getPacketsSent();
      uint32_t n_sent_flood = getNumSentFlood();
      uint32_t n_sent_direct = getNumSentDirect();
      uint32_t n_recv_flood = getNumRecvFlood();
      uint32_t n_recv_direct = getNumRecvDirect();
      uint32_t n_recv_errors = radio_driver.getPacketsRecvErrors();
      memcpy(&out_frame[i], &recv, 4); i += 4;
      memcpy(&out_frame[i], &sent, 4); i += 4;
      memcpy(&out_frame[i], &n_sent_flood, 4); i += 4;
      memcpy(&out_frame[i], &n_sent_direct, 4); i += 4;
      memcpy(&out_frame[i], &n_recv_flood, 4); i += 4;
      memcpy(&out_frame[i], &n_recv_direct, 4); i += 4;
      memcpy(&out_frame[i], &n_recv_errors, 4); i += 4;
      _serial->writeFrame(out_frame, i);
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG); // invalid stats sub-type
    }
  } else if (cmd_frame[0] == CMD_FACTORY_RESET && memcmp(&cmd_frame[1], "reset", 5) == 0) {
    if (_serial) {
      MESH_DEBUG_PRINTLN("Factory reset: disabling serial interface to prevent reconnects (BLE/WiFi)");
      _serial->disable(); // Phone app disconnects before we can send OK frame so it's safe here
    }
    bool success = _store->formatFileSystem();
    if (success) {
      writeOKFrame();
      delay(1000);
      board.reboot();  // doesn't return
    } else {
      writeErrFrame(ERR_CODE_FILE_IO_ERROR);
    }
  } else if (cmd_frame[0] == CMD_SET_FLOOD_SCOPE_KEY && len >= 2 && cmd_frame[1] == 0) {
    if (len >= 2 + 16) {
      memcpy(send_scope.key, &cmd_frame[2], sizeof(send_scope.key));  // set scope override TransportKey
    } else {
      memset(send_scope.key, 0, sizeof(send_scope.key));  // reset scope override
    }
    send_unscoped = false;
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_FLOOD_SCOPE_KEY && len >= 2 && cmd_frame[1] == 1) {  // ver 12+
    send_unscoped = true;
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_DEFAULT_FLOOD_SCOPE && len >= 1) {
    if (len >= 1+31+16) {
      int n = strlen((char *) &cmd_frame[1]);
      if (n > 0 && n < 31) {
        strcpy(_prefs.default_scope_name, (char *) &cmd_frame[1]);
        memcpy(_prefs.default_scope_key, &cmd_frame[1+31], 16);
        savePrefs();
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      }
    } else {
      memset(_prefs.default_scope_name, 0, sizeof(_prefs.default_scope_name));  // set default scope to null
      memset(_prefs.default_scope_key, 0, sizeof(_prefs.default_scope_key));
      savePrefs();
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_GET_DEFAULT_FLOOD_SCOPE) {
    out_frame[0] = RESP_CODE_DEFAULT_FLOOD_SCOPE;
    if (strlen(_prefs.default_scope_name) > 0) {
      memcpy(&out_frame[1], _prefs.default_scope_name, 31);
      memcpy(&out_frame[1+31], _prefs.default_scope_key, 16);
      _serial->writeFrame(out_frame, 1+31+16);
    } else {
      _serial->writeFrame(out_frame, 1);   // no name or key means null
    }
  } else if (cmd_frame[0] == CMD_SEND_CONTROL_DATA && len >= 2 && (cmd_frame[1] & 0x80) != 0) {
    auto resp = createControlData(&cmd_frame[1], len - 1);
    if (resp) {
      sendZeroHop(resp);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else if (cmd_frame[0] == CMD_SET_AUTOADD_CONFIG) {
    _prefs.autoadd_config = cmd_frame[1];
    if (len >= 3) {
      _prefs.autoadd_max_hops = min(cmd_frame[2], (uint8_t)64);
    }
    savePrefs();
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_GET_AUTOADD_CONFIG) {
    int i = 0;
    out_frame[i++] = RESP_CODE_AUTOADD_CONFIG;
    out_frame[i++] = _prefs.autoadd_config;
    out_frame[i++] = _prefs.autoadd_max_hops;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_GET_ALLOWED_REPEAT_FREQ) {
    int i = 0;
    out_frame[i++] = RESP_ALLOWED_REPEAT_FREQ;
    for (int k = 0; k < sizeof(repeat_freq_ranges)/sizeof(repeat_freq_ranges[0]) && i + 8 < sizeof(out_frame); k++) {
      auto r = &repeat_freq_ranges[k];
      memcpy(&out_frame[i], &r->lower_freq, 4); i += 4;
      memcpy(&out_frame[i], &r->upper_freq, 4); i += 4;
    }
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_SEND_RAW_PACKET && len >= 4) {
    auto pkt = obtainNewPacket();
    if (pkt) {
      uint8_t priority = cmd_frame[1];
      if (tryParsePacket(pkt, &cmd_frame[2], len - 2)) {
        sendPacket(pkt, priority, 0);
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      }
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else {
    writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
    MESH_DEBUG_PRINTLN("ERROR: unknown command: %02X", cmd_frame[0]);
  }
}

static bool save_filter(const ContactInfo& c) {
  return c.type != ADV_TYPE_NONE;   // don't save the transient/anon entries
}

void MyMesh::saveContacts() {
  _store->saveContacts(this, save_filter);
}

void MyMesh::enterCLIRescue() {
  _cli_rescue = true;
  cli_command[0] = 0;
  Serial.println("========= CLI Rescue =========");
}

void MyMesh::checkCLIRescueCmd() {
  int len = strlen(cli_command);
  while (Serial.available() && len < sizeof(cli_command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      cli_command[len++] = c;
      cli_command[len] = 0;
    }
    Serial.print(c);  // echo
  }
  if (len == sizeof(cli_command)-1) {  // command buffer full
    cli_command[sizeof(cli_command)-1] = '\r';
  }

  if (len > 0 && cli_command[len - 1] == '\r') {  // received complete line
    cli_command[len - 1] = 0;  // replace newline with C string null terminator

    if (memcmp(cli_command, "set ", 4) == 0) {
      const char* config = &cli_command[4];
      if (memcmp(config, "pin ", 4) == 0) {
        _prefs.ble_pin = atoi(&config[4]);
        savePrefs();
        Serial.printf("  > pin is now %06d\n", _prefs.ble_pin);
      } else {
        Serial.printf("  Error: unknown config: %s\n", config);
      }
    } else if (strcmp(cli_command, "rebuild") == 0) {
      bool success = _store->formatFileSystem();
      if (success) {
        _store->saveMainIdentity(self_id);
        savePrefs();
        saveContacts();
        saveChannels();
        Serial.println("  > erase and rebuild done");
      } else {
        Serial.println("  Error: erase failed");
      }
    } else if (strcmp(cli_command, "erase") == 0) {
      bool success = _store->formatFileSystem();
      if (success) {
        Serial.println("  > erase done");
      } else {
        Serial.println("  Error: erase failed");
      }
    } else if (memcmp(cli_command, "ls", 2) == 0) {

      // get path from command e.g: "ls /adafruit"
      const char *path = &cli_command[3];

      bool is_fs2 = false;
      if (memcmp(path, "UserData/", 9) == 0) {
        path += 8; // skip "UserData"
      } else if (memcmp(path, "ExtraFS/", 8) == 0) {
        path += 7; // skip "ExtraFS"
        is_fs2 = true;
      }
      Serial.printf("Listing files in %s\n", path);

      // log each file and directory
      File root = _store->openRead(path);
      if (is_fs2 == false) {
        if (root) {
          File file = root.openNextFile();
          while (file) {
            if (file.isDirectory()) {
              Serial.printf("[dir]  UserData%s/%s\n", path, file.name());
            } else {
              Serial.printf("[file] UserData%s/%s (%d bytes)\n", path, file.name(), file.size());
            }
            // move to next file
            file = root.openNextFile();
          }
          root.close();
        }
      }

      if (is_fs2 == true || strlen(path) == 0 || strcmp(path, "/") == 0) {
        if (_store->getSecondaryFS() != nullptr) {
          File root2 = _store->openRead(_store->getSecondaryFS(), path);
          File file = root2.openNextFile();
          while (file) {
            if (file.isDirectory()) {
              Serial.printf("[dir]  ExtraFS%s/%s\n", path, file.name());
            } else {
              Serial.printf("[file] ExtraFS%s/%s (%d bytes)\n", path, file.name(), file.size());
            }
            // move to next file
            file = root2.openNextFile();
          }
          root2.close();
        }
      }
    } else if (memcmp(cli_command, "cat", 3) == 0) {

      // get path from command e.g: "cat /contacts3"
      const char *path = &cli_command[4];

      bool is_fs2 = false;
      if (memcmp(path, "UserData/", 9) == 0) {
        path += 8; // skip "UserData"
      } else if (memcmp(path, "ExtraFS/", 8) == 0) {
        path += 7; // skip "ExtraFS"
        is_fs2 = true;
      } else {
        Serial.println("Invalid path provided, must start with UserData/ or ExtraFS/");
        cli_command[0] = 0;
        return;
      }

      // log file content as hex
      File file = _store->openRead(path);
      if (is_fs2 == true) {
        file = _store->openRead(_store->getSecondaryFS(), path);
      }
      if(file){

        // get file content
        int file_size = file.available();
        uint8_t buffer[file_size];
        file.read(buffer, file_size);

        // print hex
        mesh::Utils::printHex(Serial, buffer, file_size);
        Serial.print("\n");

        file.close();

      }

    } else if (memcmp(cli_command, "rm ", 3) == 0) {
      // get path from command e.g: "rm /adv_blobs"
      const char *path = &cli_command[3];
      MESH_DEBUG_PRINTLN("Removing file: %s", path);
      // ensure path is not empty, or root dir
      if(!path || strlen(path) == 0 || strcmp(path, "/") == 0){
        Serial.println("Invalid path provided");
      } else {
      bool is_fs2 = false;
      if (memcmp(path, "UserData/", 9) == 0) {
        path += 8; // skip "UserData"
      } else if (memcmp(path, "ExtraFS/", 8) == 0) {
        path += 7; // skip "ExtraFS"
        is_fs2 = true;
      }

        // remove file
        bool removed;
        if (is_fs2) {
          MESH_DEBUG_PRINTLN("Removing file from ExtraFS: %s", path);
          removed = _store->removeFile(_store->getSecondaryFS(), path);
        } else {
          MESH_DEBUG_PRINTLN("Removing file from UserData: %s", path);
          removed = _store->removeFile(path);
        }
        if(removed){
          Serial.println("File removed");
        } else {
          Serial.println("Failed to remove file");
        }

      }

    } else if (strcmp(cli_command, "reboot") == 0) {
      board.reboot();  // doesn't return
    } else {
      Serial.println("  Error: unknown command");
    }

    cli_command[0] = 0;  // reset command buffer
  }
}

void MyMesh::checkSerialInterface() {
  size_t len = _serial->checkRecvFrame(cmd_frame);
  if (len > 0) {
    handleCmdFrame(len);
  } else if (_iter_started              // check if our ContactsIterator is 'running'
             && !_serial->isWriteBusy() // don't spam the Serial Interface too quickly!
  ) {
    ContactInfo contact;
    bool found = false;
    while (_iter.hasNext(this, contact)) {
      if (contact.type != ADV_TYPE_NONE) {
        found = true;
        break;
      }
    }

    if (found) {
      if (contact.lastmod > _iter_filter_since) { // apply the 'since' filter
        writeContactRespFrame(RESP_CODE_CONTACT, contact);
        if (contact.lastmod > _most_recent_lastmod) {
          _most_recent_lastmod = contact.lastmod; // save for the RESP_CODE_END_OF_CONTACTS frame
        }
      }
    } else { // EOF
      out_frame[0] = RESP_CODE_END_OF_CONTACTS;
      memcpy(&out_frame[1], &_most_recent_lastmod,
             4); // include the most recent lastmod, so app can update their 'since'
      _serial->writeFrame(out_frame, 5);
      _iter_started = false;
    }
  //} else if (!_serial->isWriteBusy()) {
  //  checkConnections();    // TODO - deprecate the 'Connections' stuff
  }
}

void MyMesh::loop() {
#ifdef WITH_TRACKER_MODE
  // The base's one frame per round. Cheap to ask: it returns immediately
  // unless this node is a base running rounds and the moment has come.
  roundLoop();
#endif
#ifdef WITH_MESH_SLIM
  // First, because an answer somebody is waiting on should not queue
  // behind this node's own housekeeping.
  sendOwedSlimReplies();
  // ...and the message or marker somebody is standing there watching for.
  retrySlimOutgoing();
  // Who this unit is, to anyone who has not heard of it. See
  // _slim_advert_at for why it repeats.
  if (_slim_advert_at && millisHasNowPassed(_slim_advert_at)
      && slimproto::active()) {
    sendSlimAdvert();
#ifdef WITH_TRACKER_MODE
    // Sooner if nothing is hearing this unit - see SLIM_ADVERT_LOST_MS.
    _slim_advert_at = futureMillis(_reports.searching()
                                   ? SLIM_ADVERT_LOST_MS
                                   : SLIM_ADVERT_EVERY_MS);
#else
    _slim_advert_at = futureMillis(SLIM_ADVERT_EVERY_MS);
#endif
    // ...and how it is, ten seconds later. Broadcast rather than asked
    // for: a request needs the base to reach the unit and its answer to
    // come back, and a tracker whose battery cannot be read is one whose
    // dot stops for a reason nobody can see.
    _slim_status_at = futureMillis(10000);
  }
  if (_slim_status_at && millisHasNowPassed(_slim_status_at)
      && slimproto::active()) {
    _slim_status_at = 0;
    sendSlimStatus(slim::ADDR_BROADCAST, SLIM_STATUS_ALL, -1);
  }
  // Moved to the other protocol by the base: restart once the ack is out.
  if (_proto_restart_at) {
    if (_mgr->getOutboundTotal() > 0) {
      _proto_restart_at = futureMillis(PROTO_RESTART_QUIET_MS);
    }
    if (millisHasNowPassed(_proto_restart_at)
        || millisHasNowPassed(_proto_restart_by)) {
      if (dirty_contacts_expiry) saveContacts();
      board.reboot();   // doesn't return
    }
  }
#endif
#ifdef WITH_TRACKER_MODE
  // The radio is parked between reports: nothing below has anything to hear,
  // and running the mesh loop would wake the radio to listen.
  if (trackerLoop()) return;
#endif
  BaseChatMesh::loop();

  if (_cli_rescue) {
    checkCLIRescueCmd();
  } else {
    checkSerialInterface();
  }

  // is there are pending dirty contacts write needed?
  if (dirty_contacts_expiry && millisHasNowPassed(dirty_contacts_expiry)) {
    saveContacts();
    dirty_contacts_expiry = 0;
  }

#ifdef DISPLAY_CLASS
  if (_ui) _ui->setHasConnection(_serial->isConnected());
#endif
}

bool MyMesh::advert() {
  mesh::Packet* pkt;
  if (_prefs.advert_loc_policy == ADVERT_LOC_NONE) {
    pkt = createSelfAdvert(_prefs.node_name);
  } else {
    pkt = createSelfAdvert(_prefs.node_name, sensors.node_lat, sensors.node_lon);
  }
  if (pkt) {
    sendZeroHop(pkt);
    return true;
  } else {
    return false;
  }
}

// To check if there is pending work
bool MyMesh::hasPendingWork() const {
#ifdef WITH_MESH_SLIM
  if (_proto_restart_at) return true;   // the loop times the restart
#endif
  return _mgr->getOutboundTotal() > 0 || dirty_contacts_expiry != 0;
}

#ifdef WITH_TRACKER_MODE

// ---- tracker mode ----------------------------------------------------------
// The schedule is in TrackerMode.cpp. This is only the hardware: power the GPS
// and the radio to match the phase, and send when the schedule says so.

static const char* TRACKER_FILE = "/tracker";

//: The longest find mode anyone can ask for. Half an hour is the usual one;
//: three hours is a lost dog at dusk, and beyond that a tracker reporting
//: every minute is flattening the battery it needs to be found with.
static const uint32_t FIND_MAX_MINUTES = 180;

//: How long to hold the radio open for an acknowledgement: as long as MeshCore
//: expects the round trip to take, capped. Out of range, every cycle would
//: otherwise pay the whole flood timeout for an ack that is not coming.
static uint32_t trackerAckWait(uint32_t est_timeout) {
  return est_timeout < 20000 ? est_timeout : 20000;
}

bool MyMesh::hasGps() {
  // The sensor manager lists "gps" among its settings only when it found one.
  for (int i = 0; i < sensors.getNumSettings(); i++) {
    if (strcmp(sensors.getSettingName(i), "gps") == 0) return true;
  }
  return false;
}

bool MyMesh::setTrackerMode(uint32_t interval_s) {
  if (!tracker::validInterval(interval_s)) return false;
  if (interval_s != 0 && !hasGps()) return false;
  // Find mode owns the rate until it ends, so this is the interval to go back
  // to rather than the one to run now. Saved all the same: it is what the
  // owner chose, and it must survive a flat battery.
  if (_find_until) {
    _find_usual = interval_s;
    saveTrackerInterval(interval_s);
    return true;
  }
  // A repeat changes nothing. The base sends a command again until it sees
  // the ack, and each repeat must not buy an extra report and a flash write.
  if (interval_s == _tracker.interval()) return true;
  if (!applyTrackerInterval(interval_s)) return false;
  saveTrackerInterval(interval_s);
  return true;
}

//: Find mode: as fast as tracker mode goes, for a while. A unit that is not
//: in tracker mode at all is put into it and comes out again when the time is
//: up - "find my dog" has to work on a collar that only reports when asked.
bool MyMesh::startFind(uint32_t minutes) {
  if (minutes == 0) {
    if (!_find_until) return true;
    _find_until = 0;
    return applyTrackerInterval(_find_usual);
  }
  if (!hasGps()) return false;
  if (minutes > FIND_MAX_MINUTES) minutes = FIND_MAX_MINUTES;
  // Pressed twice: the second press extends the time, and the interval to go
  // back to is still the one from before any of this started.
  if (!_find_until) _find_usual = _tracker.interval();
  if (!applyTrackerInterval(tracker::MIN_INTERVAL_S)) return false;
  _find_until = futureMillis(minutes * 60UL * 1000UL);
  return true;
}

uint32_t MyMesh::findLeft() const {
  if (!_find_until) return 0;
  const uint32_t now = _ms->getMillis();
  return _find_until > now ? (_find_until - now + 59999UL) / 60000UL : 0;
}

void MyMesh::saveTrackerInterval(uint32_t interval_s) {
  File f = _store->openWrite(TRACKER_FILE);
  if (f) {
    f.write((const uint8_t*)&interval_s, sizeof(interval_s));
    f.close();
  }
}

//: Newest first, oldest pushed off the end. Four places is what one
//: report carries, and keeping more would be keeping what cannot be sent.
void MyMesh::pushSample(double lat, double lon) {
  for (int8_t i = slimtrack::SAMPLES - 1; i > 0; i--) {
    _trail[i] = _trail[i - 1];
  }
  _trail[0].lat = lat;
  _trail[0].lon = lon;
  _trail[0].fix = true;
  if (_trail_n < slimtrack::SAMPLES) _trail_n++;
}

bool MyMesh::sendTrackerTrail() {
  if (!slimproto::active() || _trail_n == 0 || !_trail[0].fix) return false;
  uint8_t body[slimtrack::BODY_BYTES];
  const int n = slimtrack::encode(_trail, _trail_n,
                                  _tracker.sampleCadence(), body,
                                  sizeof(body));
  if (n < 0) return false;
  const tracker::Fix fix = {(float)_trail[0].lat, (float)_trail[0].lon,
                            _ms->getMillis() / 1000};
  const uint8_t seq = (uint8_t)(slimproto::router().peekNextSeq() & 0xFF);
  if (!sendSlimBurst(slim::TYPE_POS, body, (uint8_t)n,
                     slim::ADDR_BROADCAST)) {
    _reports.unsent(fix);
    return false;
  }
  // The backlog keeps the newest place only: a gap is recovered as "!H"
  // messages, which carry their own times and do not need the trail's
  // cadence to make sense of them.
  _reports.sent(fix, slimcompanion::ackId(slim::ADDR_BASE, seq));
  _ack_due = _ms->getMillis() + trackerAckWait(0);
  return true;
}

bool MyMesh::sendTrackerNoFix() {
  if (!slimproto::active()) return false;
  // tracker::noFix, not {0, 0}: this report joins the same list as any
  // other, and the list is replayed as "!H" history when the unit comes
  // back into contact. Zeroes would have been replayed as a coordinate -
  // 0N 0E, in the Gulf of Guinea - which is the same mistake as the South
  // Pole one above, arriving an hour later and harder to explain.
  const tracker::Fix fix = tracker::noFix(_ms->getMillis() / 1000);
  const uint8_t body[slimcodec::POS_BYTES] = {0, 0, 0, 0, 0, 0};
  const uint8_t seq = (uint8_t)(slimproto::router().peekNextSeq() & 0xFF);
  if (!sendSlimBurst(slim::TYPE_POS, body, slimcodec::POS_BYTES,
                     slim::ADDR_BROADCAST)) {
    return false;
  }
  // Kept on the same list as any other report. A unit under canopy that is
  // also out of radio contact still has a gap to hand over later, and its
  // no-fix reports are part of what the base did not hear.
  _reports.sent(fix, slimcompanion::ackId(slim::ADDR_BASE, seq));
  _ack_due = _ms->getMillis() + trackerAckWait(0);
  return true;
}

bool MyMesh::applyTrackerInterval(uint32_t interval_s) {
  if (interval_s == _tracker.interval()) return true;
  if (interval_s == 0) {
    _tracker.stop();
    // Back to exactly what it was: the radio listening, and the GPS as its
    // owner last set it rather than as tracker mode happened to leave it.
    _radio_parked = false;
    _tracker_gps = false;
    sensors.setSettingValue("gps", _prefs.gps_enabled ? "1" : "0");
  } else {
    if (!hasGps()) return false;
    _tracker.start(interval_s, _ms->getMillis());
  }
  // The round changed, so the slot did: recompute it rather than leaving
  // the unit transmitting on the old round's arithmetic.
  applyTrackerSlot();
  return true;
}

void MyMesh::loadTrackerMode() {
  uint32_t secs = 0;
  File f = _store->openRead(TRACKER_FILE);
  if (f) {
    if (f.read((uint8_t*)&secs, sizeof(secs)) != sizeof(secs)) secs = 0;
    f.close();
  }
  if (secs && tracker::validInterval(secs) && hasGps()) {
    _tracker.start(secs, _ms->getMillis());
  }
}

bool MyMesh::trackerCommand(const char* text, bool from_base) {
  if (!from_base) return false;
  uint32_t value;
  // Handled either way: one the unit cannot follow - out of range, no GPS -
  // is refused here, and there is nobody to show it to as a message.
  if (tracker::parseCommand(text, value)) { setTrackerMode(value); return true; }
  if (tracker::parseFindCommand(text, value)) { startFind(value); return true; }
  return false;
}

bool MyMesh::trackerChannelCommand(const char* text) {
  tracker::ChannelCmd c;
  if (!tracker::parseChannelCommand(text, c)) return false;
  // The sender is the name MeshCore writes into the body, so this is a
  // family's convention rather than proof of anything: everyone holding the
  // channel key is already trusted with it. A stranger cannot read the
  // channel at all, and a name check keeps "!T0" typed in the family chat
  // from switching somebody's tracker off by accident.
  if (!(isBaseName(c.sender) || tracker::isParentName(c.sender))) return false;
  if (!tracker::targetMatches(c.target, _prefs.node_name)) return false;
  if (c.find) startFind(c.value); else setTrackerMode(c.value);
  return true;
}

//: This unit's number is its callsign's, the round is the interval it was
//: given, and a slot is as long as the base last said. Without all three
//: it free-runs, which is what a unit must keep doing rather than guessing
//: at a moment that might belong to someone else.
void MyMesh::applyTrackerSlot() {
  uint8_t role = 0, unit = 0;
  if (!slimcompanion::unitFromCallsign(_prefs.node_name, role, unit)) {
    _tracker.setSlot(0, 0, 0);
    return;
  }
  const uint32_t interval = _tracker.interval();
  // This unit's own moment has to end before the round does. That is the
  // real requirement here - whether SIXTEEN slots fit is the base's
  // question, asked when it decides to run rounds, and asking it here
  // would stop a four-unit team slotting on a short interval for no
  // reason.
  if (_slot_s == 0 || interval == 0
      || (uint32_t)unit * _slot_s > interval) {
    _tracker.setSlot(0, 0, 0);
    return;
  }
  // The GPS starts a fix ahead of the slot so the moment is not spent
  // waiting for one. Capped at a third of the round, so a short round
  // does not leave the receiver on continuously.
  uint16_t lead = TRACKER_FIX_LEAD_S;
  if (lead > interval / 3) lead = (uint16_t)(interval / 3);
  _tracker.setSlot(unit, _slot_s, lead);
  // As many places as the interval actually has room for. Padding it with
  // samples never taken is how a trail grows places a team never stood.
  _tracker.setSamples(slimtrack::samplesFor(interval, TRACKER_SAMPLE_S),
                      TRACKER_SAMPLE_S);
}

//: The base's round-ack. Three things in seven bytes: what the time is,
//: how long a slot is, and whether this unit was among those heard.
//:
//: The time is why a unit that has never had a GPS fix can still hold a
//: slot, and the reason this is not simply a bitmap.
//: "300,7": a five-minute round of seven-second slots. Refused rather
//: than trimmed when the slots would not fit - overlapping slots are worse
//: than none, because every unit believes it owns a moment and some of
//: them are wrong.
bool MyMesh::setRound(const char* spec) {
  if (spec == NULL) return false;
  if (*spec == 0 || strcmp(spec, "0") == 0) {
    _round_s = 0;
    _round_slot_s = 0;
    return true;
  }
  char* end;
  const unsigned long round = strtoul(spec, &end, 10);
  if (end == spec || *end != ',') return false;
  const char* np = end + 1;
  const unsigned long slot = strtoul(np, &end, 10);
  if (end == np || *end != 0) return false;
  if (slot < slimround::MIN_SLOT_S || slot > slimround::MAX_SLOT_S) return false;
  if (!slimround::slotsFit((uint32_t)round, (uint8_t)slot)) return false;
  _round_s = (uint32_t)round;
  _round_slot_s = (uint8_t)slot;
  _round_heard = 0;
  _round_ack_at = 0;
  return true;
}

void MyMesh::roundLoop() {
  if (_round_s == 0 || _round_slot_s == 0) return;
  if (!slimproto::active()) return;
  if (slimproto::unit() != slim::ADDR_BASE) return;  // the base only
  // The RTC, which the app sets when it connects. A base is attached to a
  // tablet with a clock in it; a tracker has a GPS. Both are real time,
  // which is all a slot needs.
  const uint32_t epoch = getRTCClock()->getCurrentTime();
  if (epoch < 1600000000UL) return;                  // no clock yet
  const uint32_t at = epoch - (epoch % _round_s)
                      + slimround::baseSlot(_round_s, _round_slot_s);
  if (epoch < at || _round_ack_at == at) return;
  _round_ack_at = at;
  slimround::Ack a;
  a.epoch = epoch;
  a.slot_s = _round_slot_s;
  a.heard = _round_heard;
  _round_heard = 0;
  uint8_t body[slimround::ACK_BYTES];
  if (slimround::encodeAck(a, body, sizeof(body)) < 0) return;
  sendSlimBurst(slim::TYPE_ROSTER, body, slimround::ACK_BYTES,
                slim::ADDR_BROADCAST, 1);
}

void MyMesh::onRoundAck(const slim::Frame& f) {
  slimround::Ack a;
  if (!slimround::decodeAck(f.body, f.body_len, a)) return;
  _slot_s = a.slot_s;
  if (a.epoch != 0) _tracker.setClock(a.epoch, _ms->getMillis());
  applyTrackerSlot();
  uint8_t role = 0, unit = 0;
  if (!slimcompanion::unitFromCallsign(_prefs.node_name, role, unit)) return;
  if (!slimround::wasHeard(a.heard, unit)) return;
  // Named: the base has this unit's report. The same thing a per-report
  // acknowledgement would have said, and whatever of the path is owed
  // goes now, while the radio is still on.
  const tracker::Reports::Ack heard_in_round = _reports.heardInRound();
  if (heard_in_round != tracker::Reports::NOT_OURS) {
    _ack_due = 0;
    if (heard_in_round == tracker::Reports::REPORT)
      trackerSound(UIEventType::trackerHeard);
    trackerHistory();
  }
}

mesh::DispatcherAction MyMesh::onRecvPacket(mesh::Packet* pkt) {
  // On MeshCore a packet does not say who originated it without being
  // decrypted first, so there is nothing here to make the judgement with
  // and every packet still counts. The pair-of-lost-trackers case above
  // is a MESH_SLIM one in practice: that is where a report is a broadcast
  // every unit in range hears.
  trackerHeardSomething(true);
  return mesh::Mesh::onRecvPacket(pkt);
}

//: A way through has appeared. It does not matter what the packet was or
//: who it was for - it arrived, so something is in range now, and the path
//: this unit has been keeping can start going back.
//:
//: Reporting at once rather than at the next scheduled time is the whole
//: value of having listened: the way through may be a helicopter, and it
//: may not be here in five minutes.
void MyMesh::trackerHeardSomething(bool a_way_through) {
  if (!a_way_through) return;
  if (!_tracker.on() || !_reports.searching()) return;
  // Already awake and working on a report - leave it alone rather than
  // restarting its cycle out from under it.
  if (_tracker.phase() != tracker::SLEEP) return;
  _radio_parked = false;        // out of the sniffing mode, into receive
  _tracker.start(_tracker.interval(), _ms->getMillis());
}

bool MyMesh::sendTrackerFix(double lat, double lon) {
  uint8_t role = slimcodec::ROLE_TRACKER, unit = 0;
  const bool named = slimcompanion::unitFromCallsign(_prefs.node_name, role, unit);

  if (slimproto::active()) {
    // The unit number on the air is the build's; the name only says which
    // role to label it with when it comes back the other way.
    uint8_t body[slimcodec::POS_BYTES];
    slimcodec::encodePos(lat, lon, body);
    if (named) slimcompanion::setUnitRole(slimproto::unit(), role);
    // A position is broadcast, and nothing used to acknowledge a
    // broadcast - so on slim a tracker had no way of knowing it had lost
    // the base, and the walk it did in a gully was simply gone. The base,
    // and only the base, now acknowledges one: fifteen units all replying
    // would be the ack storm the broadcast exists to avoid, but exactly
    // one reply is nine bytes and is the difference between a path kept
    // and a path lost. Which node that is needs no field on the wire -
    // the base is unit 15 by the briefing scheme.
    const tracker::Fix fix = {(float)lat, (float)lon,
                              _ms->getMillis() / 1000};
    const uint8_t seq =
        (uint8_t)(slimproto::router().peekNextSeq() & 0xFF);
    if (!sendSlimBurst(slim::TYPE_POS, body, slimcodec::POS_BYTES,
                       slim::ADDR_BROADCAST)) {
      _reports.unsent(fix);
      return false;
    }
    _reports.sent(fix, slimcompanion::ackId(slim::ADDR_BASE, seq));
    _ack_due = _ms->getMillis() + trackerAckWait(0);
    return true;
  }

  // MeshCore: the "!P" self-report when the name is a scheme callsign, plain
  // coordinates otherwise. The base reads both, by DM and on the channel.
  char text[32];
  if (!named || !slimcodec::encodeSelfReport(text, role, unit, lat, lon)) {
    // Whole numbers rather than %f, which is not linked on every target.
    const long a = lround(lat * 1e6), b = lround(lon * 1e6);
    snprintf(text, sizeof(text), "%s%ld.%06ld,%s%ld.%06ld",
             a < 0 ? "-" : "", labs(a) / 1000000, labs(a) % 1000000,
             b < 0 ? "-" : "", labs(b) / 1000000, labs(b) % 1000000);
  }

  // To the base by direct message when it is in the contacts: the ack is how
  // the unit knows the base has this one, and a report without one is kept
  // for the backlog. See tracker::Reports.
  const tracker::Fix fix = {(float)lat, (float)lon, _ms->getMillis() / 1000};
  ContactInfo* base = trackerBase();
  if (base != NULL) {
    uint32_t ack = 0, timeout = 0;
    if (sendMessage(*base, getRTCClock()->getCurrentTimeUnique(), 0, text, ack,
                    timeout) == MSG_SEND_FAILED || ack == 0) {
      _reports.unsent(fix);
      return false;
    }
    _reports.sent(fix, ack);
    _ack_due = _ms->getMillis() + trackerAckWait(timeout);
    return true;
  }

  // No base to report to: the team channel, as the app sends it. Nothing
  // acknowledges a broadcast, so nothing here can know one was missed.
  ChannelDetails channel;
  if (!getChannel(0, channel)) return false;
  return sendGroupMessage(getRTCClock()->getCurrentTimeUnique(), channel.channel,
                          _prefs.node_name, text, strlen(text));
}

ContactInfo* MyMesh::trackerBase() {
  // By name, looked up each time: a base that re-keys, or a contact list the
  // app rewrites, would leave a remembered pointer on somebody else's slot.
  ContactInfo c;
  for (int i = 0; i < getNumContacts(); i++) {
    if (getContactByIdx((uint32_t)i, c) && isBaseName(c.name)) {
      return lookupContactByPubKey(c.id.pub_key, PUB_KEY_SIZE);
    }
  }
  return NULL;
}

void MyMesh::trackerHistory() {
  if (!_reports.historyDue()) return;

  if (slimproto::active()) {
    // Addressed to the base rather than broadcast, because this one does
    // have to be acknowledged: each message going out is what lets the
    // next piece of the gap follow it, inside the same listen window.
    // "!H" is base64url and the text tier is capital letters only, so it
    // rides as an app datagram - see SlimApp.h. Without that it would
    // arrive uppercased, decode, and put the team somewhere else.
    char h[slimcodec::HIST_MAX_TEXT + 1];
    const uint8_t hn = _reports.nextHistory(h, _ms->getMillis() / 1000);
    if (hn == 0) return;
    uint8_t body[slim::MAX_BODY];
    const int blen = slimapp::bodyFor(h, body, sizeof(body));
    if (blen < 0) return;
    const uint8_t seq =
        (uint8_t)(slimproto::router().peekNextSeq() & 0xFF);
    if (!sendSlimBurst(slim::TYPE_TEXT, body, (uint8_t)blen,
                       slim::ADDR_BASE)) {
      return;   // still owed; it goes after the next confirmed report
    }
    _reports.historySent(slimcompanion::ackId(slim::ADDR_BASE, seq), hn);
    _ack_due = _ms->getMillis() + trackerAckWait(0);
    return;
  }

  ContactInfo* base = trackerBase();
  if (base == NULL) return;
  char text[slimcodec::HIST_MAX_TEXT + 1];
  const uint8_t n = _reports.nextHistory(text, _ms->getMillis() / 1000);
  if (n == 0) return;
  uint32_t ack = 0, timeout = 0;
  if (sendMessage(*base, getRTCClock()->getCurrentTimeUnique(), 0, text, ack,
                  timeout) == MSG_SEND_FAILED || ack == 0) {
    return;     // still owed; it goes after the next confirmed report
  }
  _reports.historySent(ack, n);
  _ack_due = _ms->getMillis() + trackerAckWait(timeout);
}

void MyMesh::trackerSound(UIEventType e) {
  // Straight to the UI task, which owns the buzzer and already knows
  // whether it has been muted. Nothing here decides that: a board with
  // no display has no _ui and makes no sound, which is right - it has
  // no buzzer either.
  if (_ui) _ui->notify(e);
}

bool MyMesh::trackerLoop() {
  // Before the "is it on" check: find mode can be what turned it on, and
  // ending puts it back to off.
  if (_find_until && millisHasNowPassed(_find_until)) {
    _find_until = 0;
    applyTrackerInterval(_find_usual);
  }
  if (!_tracker.on()) return false;
  const uint32_t now = _ms->getMillis();
  LocationProvider* loc = sensors.getLocationProvider();

  // The GPS is powered while the schedule wants a fix - and right through
  // the cycle while the last report had no fix to carry, because a first
  // fix cannot be taken in 90-second instalments with the receiver
  // switched off in between. See TRACKER_HUNT_MAX_S.
  //
  // Both GPS drivers keep their last answer in RAM across a power cycle,
  // so a fix counts only once its timestamp has moved on - otherwise the
  // unit would wake, read where it was an interval ago, and send that as
  // new.
  if (_gps_hunting
      && (uint32_t)(now - _hunt_from) >= TRACKER_HUNT_RETRY_S * 1000UL) {
    _hunt_from = now;        // armed again: it may have been carried out
  }
  const bool hunting =
      _gps_hunting
      && (uint32_t)(now - _hunt_from) < TRACKER_HUNT_MAX_S * 1000UL;
  const bool want_gps = _tracker.phase() == tracker::FIX || hunting;
  if (want_gps != _tracker_gps) {
    sensors.setSettingValue("gps", want_gps ? "1" : "0");
    _tracker_gps = want_gps;
    _fix_ts = loc->getTimestamp();
    _fixes = 0;
  }
  if (want_gps && loc->isValid() && loc->getTimestamp() != _fix_ts) {
    _fix_ts = loc->getTimestamp();
    if (_fixes < tracker::FIX_SETTLE) _fixes++;
    // The GPS is the clock as well as the position, and a slot is a
    // moment in real time. Free: the fix was being taken anyway.
    if (_fix_ts > 1600000000UL) {
      _tracker.setClock((uint32_t)_fix_ts, now);
      applyTrackerSlot();
    }
  }

  // Quiet once nothing is queued and no acknowledgement is still expected: the
  // listen window runs from then, so a report and every piece of backlog
  // behind it get their answers before the radio sleeps.
  const bool acking = _reports.waiting() && _ack_due != 0
                      && (long)(_ack_due - now) > 0;
  // ...and a range test is a reason to listen too.
  //
  // **This is what made the range test useless in the field.** A tracker
  // parks its radio the moment nothing is queued and nothing is expected,
  // and a ping is neither once it has gone: the unit transmitted, found
  // the queue empty, slept the receiver and never heard the answer. From
  // a ridge with clear line of sight to the base antenna that reads as no
  // coverage at all - and somebody sites a repeater on it.
  //
  // An acknowledgement escapes this because a report leaves the schedule
  // in LISTEN for ten seconds afterwards. A ping is sent by hand, from
  // whatever phase the unit happens to be in.
  bool testing = false;
#ifdef WITH_MESH_SLIM
  // A range test and a message from the screen are both waiting on an
  // answer that has nothing queued behind it, so both have to keep the
  // receiver on. See linkTestOpen and MyMesh::Outgoing.
  testing = linkTestOpen() || textAckOpen();
#endif
  const bool quiet = _mgr->getOutboundTotal() == 0 && !acking && !testing;
  const bool have_fix = _fixes >= tracker::FIX_SETTLE;
  const tracker::Schedule::Action act = _tracker.tick(now, have_fix, quiet);
  if (act == tracker::Schedule::SAMPLE) {
    // A place between reports. Nothing goes on the air - it waits for the
    // slot and rides out with the others.
    pushSample(loc->getLatitude() / 1000000.0,
               loc->getLongitude() / 1000000.0);
  } else if (act == tracker::Schedule::REPORT) {
    // With a fix, where it is - and where it has been, if the trail has
    // anything in it. Without one, that it is there at all: "MTRK03, no
    // fix" is a radio working under a canopy that is not, and silence is a
    // team nobody can hear. They must not look alike.
    if (have_fix) {
      const double lat = loc->getLatitude() / 1000000.0;
      const double lon = loc->getLongitude() / 1000000.0;
      pushSample(lat, lon);
      if (!(_trail_n > 1 && sendTrackerTrail())) sendTrackerFix(lat, lon);
      _trail_n = 0;            // the next report collects its own places
      _gps_hunting = false;    // it can find the sky; back to duty cycling
    } else {
      sendTrackerNoFix();
      _trail_n = 0;
      if (!_gps_hunting) { _gps_hunting = true; _hunt_from = now; }
    }
  }

  // Reports that have stopped getting through, said out loud once each
  // time another one does. Not on the first: a single lost report is
  // ordinary and recovers by itself, and a unit that complained about
  // ordinary is a unit people switch off. TRACKER_SEARCH_AFTER is the
  // same two-in-a-row this file already trusts to mean the base is gone.
  const uint8_t missed = _reports.missed();
  if (missed > _tracker_missed && missed >= TRACKER_SEARCH_AFTER)
    trackerSound(UIEventType::trackerLost);
  _tracker_missed = missed;

  // The radio listens whenever there is anything to hear or send: the window
  // after a report, anything still queued, and always while a phone is
  // connected - a phone that looks connected and gets no answer is the one
  // state this must never produce.
  if (_tracker.phase() == tracker::LISTEN || !quiet || _serial->isConnected()) {
    _radio_parked = false;          // the mesh loop restarts receive
    return false;
  }
  if (!_radio_parked) {
    // A unit that has lost the base does not sleep the radio - it has the
    // chip sniff for a preamble instead, often enough that none can pass
    // between two looks. It is hunting for any traffic at all: the base's
    // team map, another unit's position being repeated, a helicopter
    // overhead. Any of them means there is a way through, and the path it
    // has been keeping can go now.
    //
    // The processor sleeps through all of it either way. What this costs
    // is about an eighth of listening properly, and it is only paid by
    // the units that are actually lost.
    if (!(_reports.searching() && radio_driver.startSniffing())) {
      radio_driver.sleepWarm();
    }
    _radio_parked = true;
  }
  // Short slices rather than one long sleep: a phone that connects is answered
  // within a second, and a serial GPS cannot overflow its buffer while it looks
  // for a fix. delay() blocks in FreeRTOS on the nRF52, which sleeps the CPU in
  // between - that, the parked radio and the GPS being off are the saving.
  //
  // Keyed to the GPS rather than to the phase, because a hunt leaves it
  // on through LISTEN and SLEEP: a second of NMEA at 9600 baud is a
  // thousand characters into a buffer that holds a fraction of that, and
  // a hunt that never read what it asked for would be worse than none.
  uint32_t slice = _tracker_gps ? 20 : TRACKER_SLEEP_SLICE_MS;
  const uint32_t left = _tracker.sleepLeft(now);
  if (left && left < slice) slice = left;
  delay(slice);
  return true;
}

#endif  // WITH_TRACKER_MODE
