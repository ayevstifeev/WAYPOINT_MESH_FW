#include "UITask.h"
#include <helpers/TxtDataHelpers.h>
#include "../MyMesh.h"
#include "target.h"
#ifdef WIFI_SSID
  #include <WiFi.h>
#endif

#ifndef AUTO_OFF_MILLIS
  #define AUTO_OFF_MILLIS     15000   // 15 seconds
#endif
#define BOOT_SCREEN_MILLIS   3000   // 3 seconds

#ifdef PIN_STATUS_LED
#define LED_ON_MILLIS     20
#define LED_ON_MSG_MILLIS 200
#define LED_CYCLE_MILLIS  4000
#endif

#define LONG_PRESS_MILLIS   1200

#ifndef UI_RECENT_LIST_SIZE
  #define UI_RECENT_LIST_SIZE 4
#endif

#if UI_HAS_JOYSTICK
  #define PRESS_LABEL "press Enter"
#else
  #define PRESS_LABEL "long press"
#endif

#include "icons.h"
#ifdef WITH_MESH_SLIM
  #include <helpers/slim/SlimText.h>
#endif

class SplashScreen : public UIScreen {
  UITask* _task;
  unsigned long dismiss_after;
  char _version_info[12];
#ifdef WITH_MESH_SLIM
  //: Which unit this is. Fifteen identical handhelds come out of one box
  //: at a briefing, and this is the moment somebody is deciding who gets
  //: which - so it is the moment to say.
  NodePrefs* _prefs;
#endif

public:
#ifdef WITH_MESH_SLIM
  SplashScreen(UITask* task, NodePrefs* prefs) : _task(task), _prefs(prefs) {
#else
  SplashScreen(UITask* task) : _task(task) {
#endif
    // strip off dash and commit hash by changing dash to null terminator
    // e.g: v1.2.3-abcdef -> v1.2.3
    const char *ver = FIRMWARE_VERSION;
    const char *dash = strchr(ver, '-');

    int len = dash ? dash - ver : strlen(ver);
    if (len >= sizeof(_version_info)) len = sizeof(_version_info) - 1;
    memcpy(_version_info, ver, len);
    _version_info[len] = 0;

    dismiss_after = millis() + BOOT_SCREEN_MILLIS;
  }

  int render(DisplayDriver& display) override {
#ifdef WITH_MESH_SLIM
    // The product screen, not MeshCore's. A stock build keeps theirs.
    display.setColor(DisplayDriver::BLUE);
    display.setTextSize(2);
    display.drawTextCentered(display.width() / 2, 4, "WAYPOINT");
    // A rule rather than a logo: one fillRect costs nothing and reads
    // cleanly on a 128x64 mono panel at arm's length in daylight.
    display.fillRect(16, 22, display.width() - 32, 1);

    display.setColor(DisplayDriver::LIGHT);
    const char* who = (_prefs != NULL && _prefs->node_name[0])
                      ? _prefs->node_name : "UNNAMED";
    // Two sizes, so a scheme callsign is readable across a room and a
    // long or unnamed one still fits rather than running off the edge.
    display.setTextSize(strlen(who) <= 8 ? 2 : 1);
    display.drawTextCentered(display.width() / 2,
                             strlen(who) <= 8 ? 28 : 32, who);

    display.setTextSize(1);
    display.drawTextCentered(display.width() / 2, 50, _version_info);
    return 1000;
#else
    // meshcore logo
    display.setColor(DisplayDriver::BLUE);
    int logoWidth = 128;
    display.drawXbm((display.width() - logoWidth) / 2, 3, meshcore_logo, logoWidth, 13);

    // meshcore website
    const char* website = "https://meshcore.io";
    display.setColor(DisplayDriver::LIGHT);
    display.setTextSize(1);
    uint16_t websiteWidth = display.getTextWidth(website);
    display.setCursor((display.width() - websiteWidth) / 2, 22);
    display.print(website);

    // version info
    display.setColor(DisplayDriver::LIGHT);
    display.setTextSize(1);
    display.drawTextCentered(display.width()/2, 35, _version_info);

    display.setTextSize(1);
    display.drawTextCentered(display.width()/2, 48, FIRMWARE_BUILD_DATE);

    return 1000;
#endif
  }

  void poll() override {
    if (millis() >= dismiss_after) {
      _task->gotoHomeScreen();
    }
  }
};

class HomeScreen : public UIScreen {
  enum HomePage {
    FIRST,
    RECENT,
    RADIO,
    BLUETOOTH,
#if defined(PIN_BUZZER) && defined(WITH_SLIM_UI)
    //: Next to Bluetooth because they are the same question - what this
    //: unit does when nobody is asking it to - and because muting was a
    //: triple-click, which is not a thing anybody finds.
    SOUND,
#endif
    ADVERT,
#if ENV_INCLUDE_GPS == 1
    GPS,
#endif
#if UI_SENSORS_PAGE == 1
    SENSORS,
#endif
#ifdef WITH_MESH_SLIM
    PROTOCOL,
#if UI_HAS_JOYSTICK
    //: Before SEND, because reading what the team said comes before
    //: answering it - and because a message that can only be read in the
    //: seconds after it lands is a message that gets missed.
    MESSAGES,
    SEND,
    MARKER,
#endif
#endif
#ifdef WITH_TRACKER_MODE
    TRACKER,
#endif
    SHUTDOWN,
    Count    // keep as last
  };

  UITask* _task;
#ifdef WITH_SLIM_UI
  //: Presses in a row on the info page, and when the last one was.
  uint8_t _presses = 0;
  unsigned long _last_press = 0;
#endif
  mesh::RTCClock* _rtc;
  SensorManager* _sensors;
  NodePrefs* _node_prefs;
  uint8_t _page;
  bool _shutdown_init;
  AdvertPath recent[UI_RECENT_LIST_SIZE];
#ifdef WITH_MESH_SLIM
  unsigned long _proto_armed_until = 0;   // a first press, waiting for the second
  unsigned long _restart_at = 0;          // the switch is saved; restart then
#endif


  void renderBatteryIndicator(DisplayDriver& display, uint16_t batteryMilliVolts) {
    // Convert millivolts to percentage
#ifndef BATT_MIN_MILLIVOLTS
  #define BATT_MIN_MILLIVOLTS 3000
#endif
#ifndef BATT_MAX_MILLIVOLTS
  #define BATT_MAX_MILLIVOLTS 4200
#endif
    const int minMilliVolts = BATT_MIN_MILLIVOLTS;
    const int maxMilliVolts = BATT_MAX_MILLIVOLTS;
    int batteryPercentage = ((batteryMilliVolts - minMilliVolts) * 100) / (maxMilliVolts - minMilliVolts);
    if (batteryPercentage < 0) batteryPercentage = 0; // Clamp to 0%
    if (batteryPercentage > 100) batteryPercentage = 100; // Clamp to 100%

    // battery icon
    int iconWidth = 24;
    int iconHeight = 10;
    int iconX = display.width() - iconWidth - 5; // Position the icon near the top-right corner
    int iconY = 0;
    display.setColor(DisplayDriver::GREEN);

    // battery outline
    display.drawRect(iconX, iconY, iconWidth, iconHeight);

    // battery "cap"
    display.fillRect(iconX + iconWidth, iconY + (iconHeight / 4), 3, iconHeight / 2);

    // fill the battery based on the percentage
    int fillWidth = (batteryPercentage * (iconWidth - 4)) / 100;
    display.fillRect(iconX + 2, iconY + 2, fillWidth, iconHeight - 4);

    // show muted icon if buzzer is muted
#ifdef PIN_BUZZER
    if (_task->isBuzzerQuiet()) {
      display.setColor(DisplayDriver::RED);
      display.drawXbm(iconX - 9, iconY + 1, muted_icon, 8, 8);
    }
#endif
  }

  CayenneLPP sensors_lpp;
  int sensors_nb = 0;
  bool sensors_scroll = false;
  int sensors_scroll_offset = 0;
  int next_sensors_refresh = 0;

  void refresh_sensors() {
    if (millis() > next_sensors_refresh) {
      sensors_lpp.reset();
      sensors_nb = 0;
      sensors_lpp.addVoltage(TELEM_CHANNEL_SELF, (float)board.getBattMilliVolts() / 1000.0f);
      sensors.querySensors(0xFF, sensors_lpp);
      LPPReader reader (sensors_lpp.getBuffer(), sensors_lpp.getSize());
      uint8_t channel, type;
      while(reader.readHeader(channel, type)) {
        reader.skipData(type);
        sensors_nb ++;
      }
      sensors_scroll = sensors_nb > UI_RECENT_LIST_SIZE;
#if AUTO_OFF_MILLIS > 0
      next_sensors_refresh = millis() + 5000; // refresh sensor values every 5 sec
#else
      next_sensors_refresh = millis() + 60000; // refresh sensor values every 1 min
#endif
    }
  }

public:
  HomeScreen(UITask* task, mesh::RTCClock* rtc, SensorManager* sensors, NodePrefs* node_prefs)
     : _task(task), _rtc(rtc), _sensors(sensors), _node_prefs(node_prefs), _page(0),
       _shutdown_init(false), sensors_lpp(200) {  }

  void poll() override {
    if (_shutdown_init && !_task->isButtonPressed()) {  // must wait for USR button to be released
      _task->shutdown();
    }
#ifdef WITH_MESH_SLIM
    if (_restart_at && millis() > _restart_at) {
      _restart_at = 0;
      _task->shutdown(true);   // restart, on the protocol just saved
    }
#endif
  }

  int render(DisplayDriver& display) override {
    char tmp[80];
    // node name
    display.setTextSize(1);
    display.setColor(DisplayDriver::GREEN);
    char filtered_name[sizeof(_node_prefs->node_name)];
    display.translateUTF8ToBlocks(filtered_name, _node_prefs->node_name, sizeof(filtered_name));
    display.setCursor(0, 0);
    display.print(filtered_name);

    // battery voltage
    renderBatteryIndicator(display, _task->getBattMilliVolts());

    // curr page indicator
    int y = 14;
    int x = display.width() / 2 - 5 * (HomePage::Count-1);
    for (uint8_t i = 0; i < HomePage::Count; i++, x += 10) {
      if (i == _page) {
        display.fillRect(x-1, y-1, 3, 3);
      } else {
        display.fillRect(x, y, 1, 1);
      }
    }

    if (_page == HomePage::FIRST) {
      display.setColor(DisplayDriver::YELLOW);
      display.setTextSize(2);
      sprintf(tmp, "MSG: %d", _task->getMsgCount());
      display.drawTextCentered(display.width() / 2, 20, tmp);

      #ifdef WIFI_SSID
        IPAddress ip = WiFi.localIP();
        snprintf(tmp, sizeof(tmp), "IP: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        display.setTextSize(1);
        display.drawTextCentered(display.width() / 2, 54, tmp);
      #endif
#ifdef WITH_SLIM_UI
      // What became of the last thing said from this screen.
      //
      // It takes the line ahead of "BASE OK" because it is the more urgent
      // of the two: somebody is standing there waiting to know whether what
      // they said arrived, and until this existed the screen said "Sent"
      // the moment the frame was queued and never mentioned it again.
      const uint8_t msg_state = the_mesh.textState();
      if (msg_state) {
        display.setTextSize(1);
        // A pin and a message fail the same way and must not be reported
        // the same way: somebody who dropped a pin needs to know THAT is
        // what nobody has.
        const char* what = the_mesh.textIsMarker() ? "PIN" : "MSG";
        if (msg_state == 1) {
          display.setColor(DisplayDriver::YELLOW);
          sprintf(tmp, "%s SENDING %u/3", what, (unsigned)the_mesh.textTries());
        } else if (msg_state == 2) {
          display.setColor(DisplayDriver::GREEN);
          sprintf(tmp, "%s DELIVERED", what);
        } else {
          display.setColor(DisplayDriver::RED);
          sprintf(tmp, "%s NOT DELIVERED", what);
        }
        display.drawTextCentered(display.width() / 2, 43, tmp);
      } else if (the_mesh.trackerInterval()) {
        if (the_mesh.trackerSearching()) {
          display.setColor(DisplayDriver::RED);
          display.setTextSize(1);
          // The count is the reassurance: the walk is being kept.
          const uint16_t owed = the_mesh.trackerOwed();
          if (owed) sprintf(tmp, "NO BASE - %u kept", (unsigned)owed);
          else strcpy(tmp, "NO BASE - searching");
          display.drawTextCentered(display.width() / 2, 43, tmp);
        } else if (the_mesh.trackerMissed()) {
          display.setColor(DisplayDriver::YELLOW);
          display.setTextSize(1);
          sprintf(tmp, "NO ACK x%u", (unsigned)the_mesh.trackerMissed());
          display.drawTextCentered(display.width() / 2, 43, tmp);
        } else {
          display.setColor(DisplayDriver::GREEN);
          display.setTextSize(1);
          display.drawTextCentered(display.width() / 2, 43, "BASE OK");
        }
#if ENV_INCLUDE_GPS == 1
        // The other half of the same question. "Where am I" and "can
        // anyone hear me" are the two things a searcher needs off this
        // screen, and the second is no use without the first: a unit in
        // perfect contact reporting a fix it lost an hour ago is worse
        // than one that says it cannot see sky.
        LocationProvider* gps = (_sensors != NULL)
                                ? _sensors->getLocationProvider() : NULL;
        display.setTextSize(1);
        if (gps != NULL && gps->isValid()) {
          display.setColor(DisplayDriver::LIGHT);
          sprintf(tmp, "GPS %ld sats", gps->satellitesCount());
        } else {
          display.setColor(DisplayDriver::RED);
          strcpy(tmp, "GPS: no fix");
        }
        display.drawTextCentered(display.width() / 2, 54, tmp);
#endif
      } else if (_task->hasConnection()) {
#else
      if (_task->hasConnection()) {
#endif
        display.setColor(DisplayDriver::GREEN);
        display.setTextSize(1);
        display.drawTextCentered(display.width() / 2, 43, "< Connected >");

      } else if (the_mesh.getBLEPin() != 0) { // BT pin
        display.setColor(DisplayDriver::RED);
        display.setTextSize(2);
        sprintf(tmp, "Pin:%d", the_mesh.getBLEPin());
        display.drawTextCentered(display.width() / 2, 43, tmp);
      }
    } else if (_page == HomePage::RECENT) {
      the_mesh.getRecentlyHeard(recent, UI_RECENT_LIST_SIZE);
      display.setColor(DisplayDriver::GREEN);
      int y = 20;
      for (int i = 0; i < UI_RECENT_LIST_SIZE; i++, y += 11) {
        auto a = &recent[i];
        if (a->name[0] == 0) continue;  // empty slot
        int secs = _rtc->getCurrentTime() - a->recv_timestamp;
        if (secs < 60) {
          sprintf(tmp, "%ds", secs);
        } else if (secs < 60*60) {
          sprintf(tmp, "%dm", secs / 60);
        } else {
          sprintf(tmp, "%dh", secs / (60*60));
        }

        int timestamp_width = display.getTextWidth(tmp);
        int max_name_width = display.width() - timestamp_width - 1;

        char filtered_recent_name[sizeof(a->name)];
        display.translateUTF8ToBlocks(filtered_recent_name, a->name, sizeof(filtered_recent_name));
        display.drawTextEllipsized(0, y, max_name_width, filtered_recent_name);
        display.setCursor(display.width() - timestamp_width - 1, y);
        display.print(tmp);
      }
    } else if (_page == HomePage::RADIO) {
      display.setColor(DisplayDriver::YELLOW);
      display.setTextSize(1);
      // freq / sf
      display.setCursor(0, 20);
      sprintf(tmp, "FQ: %06.3f   SF: %d", _node_prefs->freq, _node_prefs->sf);
      display.print(tmp);

      display.setCursor(0, 31);
      sprintf(tmp, "BW: %03.2f     CR: %d", _node_prefs->bw, _node_prefs->cr);
      display.print(tmp);

      // tx power,  noise floor
      display.setCursor(0, 42);
      sprintf(tmp, "TX: %ddBm", _node_prefs->tx_power_dbm);
      display.print(tmp);
      display.setCursor(0, 53);
      sprintf(tmp, "Noise floor: %d", radio_driver.getNoiseFloor());
      display.print(tmp);
    } else if (_page == HomePage::BLUETOOTH) {
      display.setColor(DisplayDriver::GREEN);
      display.drawXbm((display.width() - 32) / 2, 18,
          _task->isSerialEnabled() ? bluetooth_on : bluetooth_off,
          32, 32);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 64 - 11, "toggle: " PRESS_LABEL);
#if defined(PIN_BUZZER) && defined(WITH_SLIM_UI)
    } else if (_page == HomePage::SOUND) {
      display.setColor(DisplayDriver::GREEN);
      display.setTextSize(2);
      display.drawTextCentered(display.width() / 2, 22,
                               _task->isBuzzerQuiet() ? "Silent" : "Sound");
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 64 - 11,
                               "toggle: " PRESS_LABEL);
#endif
    } else if (_page == HomePage::ADVERT) {
      display.setColor(DisplayDriver::GREEN);
      display.drawXbm((display.width() - 32) / 2, 18, advert_icon, 32, 32);
      display.drawTextCentered(display.width() / 2, 64 - 11, "advert: " PRESS_LABEL);
#if ENV_INCLUDE_GPS == 1
    } else if (_page == HomePage::GPS) {
      LocationProvider* nmea = sensors.getLocationProvider();
      char buf[50];
      int y = 18;
      bool gps_state = _task->getGPSState();
#ifdef PIN_GPS_SWITCH
      bool hw_gps_state = digitalRead(PIN_GPS_SWITCH);
      if (gps_state != hw_gps_state) {
        strcpy(buf, gps_state ? "gps off(hw)" : "gps off(sw)");
      } else {
        strcpy(buf, gps_state ? "gps on" : "gps off");
      }
#else
      strcpy(buf, gps_state ? "gps on" : "gps off");
#endif
      display.drawTextLeftAlign(0, y, buf);
      if (nmea == NULL) {
        y = y + 12;
        display.drawTextLeftAlign(0, y, "Can't access GPS");
      } else {
        strcpy(buf, nmea->isValid()?"fix":"no fix");
        display.drawTextRightAlign(display.width()-1, y, buf);
        y = y + 12;
        display.drawTextLeftAlign(0, y, "sat");
        sprintf(buf, "%d", nmea->satellitesCount());
        display.drawTextRightAlign(display.width()-1, y, buf);
        y = y + 12;
        display.drawTextLeftAlign(0, y, "pos");
        sprintf(buf, "%.4f %.4f",
          nmea->getLatitude()/1000000., nmea->getLongitude()/1000000.);
        display.drawTextRightAlign(display.width()-1, y, buf);
        y = y + 12;
        display.drawTextLeftAlign(0, y, "alt");
        sprintf(buf, "%.2f", nmea->getAltitude()/1000.);
        display.drawTextRightAlign(display.width()-1, y, buf);
        y = y + 12;
      }
#endif
#if UI_SENSORS_PAGE == 1
    } else if (_page == HomePage::SENSORS) {
      int y = 18;
      refresh_sensors();
      char buf[30];
      char name[30];
      LPPReader r(sensors_lpp.getBuffer(), sensors_lpp.getSize());

      for (int i = 0; i < sensors_scroll_offset; i++) {
        uint8_t channel, type;
        r.readHeader(channel, type);
        r.skipData(type);
      }

      for (int i = 0; i < (sensors_scroll?UI_RECENT_LIST_SIZE:sensors_nb); i++) {
        uint8_t channel, type;
        if (!r.readHeader(channel, type)) { // reached end, reset
          r.reset();
          r.readHeader(channel, type);
        }

        display.setCursor(0, y);
        float v;
        switch (type) {
          case LPP_GPS: // GPS
            float lat, lon, alt;
            r.readGPS(lat, lon, alt);
            strcpy(name, "gps"); sprintf(buf, "%.4f %.4f", lat, lon);
            break;
          case LPP_VOLTAGE:
            r.readVoltage(v);
            strcpy(name, "voltage"); sprintf(buf, "%6.2f", v);
            break;
          case LPP_CURRENT:
            r.readCurrent(v);
            strcpy(name, "current"); sprintf(buf, "%.3f", v);
            break;
          case LPP_TEMPERATURE:
            r.readTemperature(v);
            strcpy(name, "temperature"); sprintf(buf, "%.2f", v);
            break;
          case LPP_RELATIVE_HUMIDITY:
            r.readRelativeHumidity(v);
            strcpy(name, "humidity"); sprintf(buf, "%.2f", v);
            break;
          case LPP_BAROMETRIC_PRESSURE:
            r.readPressure(v);
            strcpy(name, "pressure"); sprintf(buf, "%.2f", v);
            break;
          case LPP_ALTITUDE:
            r.readAltitude(v);
            strcpy(name, "altitude"); sprintf(buf, "%.0f", v);
            break;
          case LPP_POWER:
            r.readPower(v);
            strcpy(name, "power"); sprintf(buf, "%6.2f", v);
            break;
          default:
            r.skipData(type);
            strcpy(name, "unk"); sprintf(buf, "");
        }
        display.setCursor(0, y);
        display.print(name);
        display.setCursor(
          display.width()-display.getTextWidth(buf)-1, y
        );
        display.print(buf);
        y = y + 12;
      }
      if (sensors_scroll) sensors_scroll_offset = (sensors_scroll_offset+1)%sensors_nb;
      else sensors_scroll_offset = 0;
#endif
#ifdef WITH_MESH_SLIM
    } else if (_page == HomePage::PROTOCOL) {
      display.setColor(DisplayDriver::YELLOW);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 18, "Protocol");
      display.setColor(DisplayDriver::GREEN);
      display.setTextSize(2);
      display.drawTextCentered(display.width() / 2, 30,
                               the_mesh.slimActive() ? "SLIM" : "MeshCore");
      display.setColor(DisplayDriver::LIGHT);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 64 - 11,
          millis() < _proto_armed_until ? "again to switch" : "switch: " PRESS_LABEL);
#if UI_HAS_JOYSTICK
    } else if (_page == HomePage::MESSAGES) {
      // What the team has said, kept. The count is the point of the page:
      // it is what tells somebody walking past the screen that there is
      // something to read at all.
      display.setColor(DisplayDriver::YELLOW);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 18, "Messages");
      const int kept = _task->msgsKept(), unread = _task->msgsUnread();
      if (unread) {
        display.setColor(DisplayDriver::GREEN);
        display.setTextSize(2);
        sprintf(tmp, "%d new", unread);
      } else {
        display.setColor(kept ? DisplayDriver::GREEN : DisplayDriver::LIGHT);
        display.setTextSize(1);
        if (kept) sprintf(tmp, "%d kept", kept);
        else strcpy(tmp, "nothing yet");
      }
      display.drawTextCentered(display.width() / 2, 30, tmp);
      display.setColor(DisplayDriver::LIGHT);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 64 - 11,
                               kept ? "read: " PRESS_LABEL : "");
    } else if (_page == HomePage::SEND || _page == HomePage::MARKER) {
      // Both say the same three things: what this page does, what it
      // would act on right now, and how to open it.
      const bool marker = _page == HomePage::MARKER;
      if (marker) {
        double here_lat, here_lon;
        strcpy(tmp, the_mesh.hereNow(here_lat, here_lon)
                      ? "here" : "no fix yet");
      } else {
        // Where a reply would actually go. A Send page that does not
        // say where it sends is the one thing it must not be.
        const uint8_t to = the_mesh.slimReplyTo();
        if (to) sprintf(tmp, "to unit %u", (unsigned)to);
        else strcpy(tmp, "to base");
      }
      display.setColor(DisplayDriver::YELLOW);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 18,
                               marker ? "Place marker" : "Send a message");
      display.setColor(DisplayDriver::GREEN);
      display.drawTextCentered(display.width() / 2, 34, tmp);
      display.setColor(DisplayDriver::LIGHT);
      display.drawTextCentered(display.width() / 2, 64 - 11,
                               "open: " PRESS_LABEL);
#endif
#endif
#ifdef WITH_TRACKER_MODE
    } else if (_page == HomePage::TRACKER) {
      display.setColor(DisplayDriver::YELLOW);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 18, "Tracker mode");
      const uint32_t every = the_mesh.trackerInterval();
      if (every) sprintf(tmp, "%lu min", (unsigned long)(every / 60));
      else strcpy(tmp, "OFF");
      display.setColor(DisplayDriver::GREEN);
      display.setTextSize(2);
      display.drawTextCentered(display.width() / 2, 27, tmp);
      display.setColor(DisplayDriver::LIGHT);
      display.setTextSize(1);
      if (every) {
        // What it is doing now, and how many fixes the base has not had yet.
        static const char* PHASES[] = {"off", "getting fix", "listening", "asleep"};
        sprintf(tmp, "%s  owed %u", PHASES[the_mesh.trackerPhase()],
                (unsigned)the_mesh.trackerOwed());
        display.drawTextCentered(display.width() / 2, 43, tmp);
      }
      display.drawTextCentered(display.width() / 2, 64 - 11, "change: " PRESS_LABEL);
#endif
    } else if (_page == HomePage::SHUTDOWN) {
      display.setColor(DisplayDriver::GREEN);
      display.setTextSize(1);
      if (_shutdown_init) {
        display.drawTextCentered(display.width() / 2, 34, "hibernating...");
      } else {
        display.drawXbm((display.width() - 32) / 2, 18, power_icon, 32, 32);
        display.drawTextCentered(display.width() / 2, 64 - 11, "hibernate:" PRESS_LABEL);
      }
    }
    return 5000;   // next render after 5000 ms
  }

  bool handleInput(char c) override {
    if (c == KEY_LEFT || c == KEY_PREV) {
      _page = (_page + HomePage::Count - 1) % HomePage::Count;
      return true;
    }
    if (c == KEY_NEXT || c == KEY_RIGHT) {
      _page = (_page + 1) % HomePage::Count;
      if (_page == HomePage::RECENT) {
        _task->showAlert("Recent adverts", 800);
      }
      return true;
    }
#ifdef WITH_SLIM_UI
    // Five presses on the info page opens the range test. Not a page on
    // the carousel: everything there is something an operator does
    // during a search, and this is an engineer siting a repeater. A
    // ninth page walked past to reach Tracker is a cost everybody pays
    // for a screen one person uses occasionally.
    //
    // Five because no accident is five, and the count clears after two
    // seconds of no press - a slow hand starts again rather than
    // leaving something half-armed.
    if (c == KEY_ENTER && _page == HomePage::FIRST) {
      const unsigned long now = millis();
      if (now - _last_press > 2000) _presses = 0;
      _last_press = now;
      if (++_presses >= 5) {
        _presses = 0;
        _task->gotoLinkTest();
      }
      return true;
    }
#endif
    if (c == KEY_ENTER && _page == HomePage::BLUETOOTH) {
      if (_task->isSerialEnabled()) {  // toggle Bluetooth on/off
        _task->disableSerial();
      } else {
        _task->enableSerial();
      }
#ifdef MESH_SLIM_BLE_OFF
      // Remembered, so switching it off is done once and not again at
      // every boot. Written here rather than in enableSerial/
      // disableSerial: those are also how a reconnect and the CLI drive
      // the radio, and only the operator pressing this means "keep it".
      the_mesh.saveBleWanted(_task->isSerialEnabled());
#endif
      return true;
    }
#if defined(PIN_BUZZER) && defined(WITH_SLIM_UI)
    if (c == KEY_ENTER && _page == HomePage::SOUND) {
      _task->toggleBuzzer();   // flips it, saves prefs, says which
      return true;
    }
#endif
    if (c == KEY_ENTER && _page == HomePage::ADVERT) {
      _task->notify(UIEventType::ack);
      if (the_mesh.advert()) {
        _task->showAlert("Advert sent!", 1000);
      } else {
        _task->showAlert("Advert failed..", 1000);
      }
      return true;
    }
#if ENV_INCLUDE_GPS == 1
    if (c == KEY_ENTER && _page == HomePage::GPS) {
#ifdef WITH_TRACKER_MODE
      // Tracker mode switches the GPS itself; a toggle here would fight it.
      if (the_mesh.trackerInterval()) {
        _task->showAlert("GPS: tracker's now", 1200);
        return true;
      }
#endif
      _task->toggleGPS();
      return true;
    }
#endif
#if UI_SENSORS_PAGE == 1
    if (c == KEY_ENTER && _page == HomePage::SENSORS) {
      _task->toggleGPS();
      next_sensors_refresh=0;
      return true;
    }
#endif
#ifdef WITH_MESH_SLIM
#if UI_HAS_JOYSTICK
    if (c == KEY_ENTER && _page == HomePage::MESSAGES) {
      _task->gotoInbox();
      return true;
    }
    if (c == KEY_ENTER && (_page == HomePage::SEND
                           || _page == HomePage::MARKER)) {
      if (_page == HomePage::MARKER) _task->gotoMarker();
      else _task->gotoSend();
      return true;
    }
#endif
    if (c == KEY_ENTER && _page == HomePage::PROTOCOL) {
      // Twice, a few seconds apart: the two protocols cannot hear each other,
      // so a unit switched by a stray press has quietly left its network.
      if (millis() > _proto_armed_until) {
        _proto_armed_until = millis() + 4000;
        _task->showAlert(the_mesh.slimActive() ? "Again: to MeshCore" : "Again: to SLIM", 1500);
        return true;
      }
      _proto_armed_until = 0;
      if (!the_mesh.saveSlimProto(!the_mesh.slimActive())) {
        _task->showAlert("SLIM needs its key", 1500);
        return true;
      }
      _task->notify(UIEventType::ack);
      _task->showAlert("Switching: restart", 1500);
      _restart_at = millis() + 1500;   // long enough to read the alert
      return true;
    }
#endif
#ifdef WITH_TRACKER_MODE
    if (c == KEY_ENTER && _page == HomePage::TRACKER) {
#ifdef WITH_SLIM_UI
      // A list, on a unit that has a joystick to walk one with. Cycling
      // put every interval three presses away from every other and
      // showed the options only one at a time, which on the setting
      // that decides battery life is the wrong way round.
      _task->gotoTrackerEvery();
      return true;
#else
      // Off, then every 1, 5 and 15 minutes - the finder page's choices. An
      // interval set from elsewhere steps to off.
      static const uint32_t STEPS[] = {0, 60, 300, 900};
      const uint32_t now_every = the_mesh.trackerInterval();
      uint8_t i = 0;
      while (i < 4 && STEPS[i] != now_every) i++;
      const uint32_t next = i < 4 ? STEPS[(i + 1) % 4] : 0;
      if (!the_mesh.setTrackerMode(next)) {
        _task->showAlert("No GPS: can't track", 1500);
        return true;
      }
      char msg[24];
      if (next) sprintf(msg, "Tracker: %lu min", (unsigned long)(next / 60));
      else strcpy(msg, "Tracker: OFF");
      _task->notify(UIEventType::ack);
      _task->showAlert(msg, 1000);
      return true;
#endif
    }
#endif
    if (c == KEY_ENTER && _page == HomePage::SHUTDOWN) {
      _shutdown_init = true;  // need to wait for button to be released
      return true;
    }
    return false;
  }
};

class MsgPreviewScreen : public UIScreen {
  UITask* _task;
  mesh::RTCClock* _rtc;

  struct MsgEntry {
    uint32_t timestamp;
    char origin[62];
    char msg[78];
  };
  #define MAX_UNREAD_MSGS   32
  //: How many are new, and how many are still here to read.
  //:
  //: **These used to be one number, and reading emptied the ring.** Paging
  //: past the last unread message went home and the rest were gone: a
  //: searcher who glanced at a message while walking, or whose screen
  //: moved on by itself, had no way back to it. What was said on the radio
  //: is exactly the thing somebody needs to look at twice.
  int num_unread;
  int head = MAX_UNREAD_MSGS - 1; // index of the newest message
#ifdef WITH_SLIM_UI
  int num_msgs = 0;
  //: Which one is on screen, counting back from the newest.
  int _at = 0;
#endif
  MsgEntry unread[MAX_UNREAD_MSGS];

public:
  MsgPreviewScreen(UITask* task, mesh::RTCClock* rtc) : _task(task), _rtc(rtc) { num_unread = 0; }

#ifdef WITH_SLIM_UI
  int unreadCount() const { return num_unread; }
  int keptCount() const { return num_msgs; }
  //: Opened deliberately, from the Messages page, rather than by a message
  //: arriving. Newest first, and looking at them is what makes them read.
  void openInbox() { _at = 0; num_unread = 0; }
#endif

  void addPreview(uint8_t path_len, const char* from_name, const char* msg) {
    head = (head + 1) % MAX_UNREAD_MSGS;
    if (num_unread < MAX_UNREAD_MSGS) num_unread++;
#ifdef WITH_SLIM_UI
    if (num_msgs < MAX_UNREAD_MSGS) num_msgs++;
    _at = 0;                       // a new one is what you want to see
#endif

    auto p = &unread[head];
    p->timestamp = _rtc->getCurrentTime();
    if (path_len == 0xFF) {
      sprintf(p->origin, "(D) %s:", from_name);
    } else {
      sprintf(p->origin, "(%d) %s:", (uint32_t) path_len, from_name);
    }
    StrHelper::strncpy(p->msg, msg, sizeof(p->msg));
  }

  int render(DisplayDriver& display) override {
#ifdef WITH_SLIM_UI
    char tmp[24];
#else
    char tmp[16];
#endif
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setColor(DisplayDriver::GREEN);
#ifdef WITH_SLIM_UI
    // Where you are in the list, and how many of them are new. "3/12" is
    // what tells somebody there is more above and below, which a single
    // unread count never did.
    if (num_unread) sprintf(tmp, "%d/%d  %d new", _at + 1, num_msgs, num_unread);
    else sprintf(tmp, "%d/%d", _at + 1, num_msgs);
    display.print(tmp);

    auto p = &unread[(head + MAX_UNREAD_MSGS - _at) % MAX_UNREAD_MSGS];
#else
    sprintf(tmp, "Unread: %d", num_unread);
    display.print(tmp);

    auto p = &unread[head];
#endif

    int secs = _rtc->getCurrentTime() - p->timestamp;
    if (secs < 60) {
      sprintf(tmp, "%ds", secs);
    } else if (secs < 60*60) {
      sprintf(tmp, "%dm", secs / 60);
    } else {
      sprintf(tmp, "%dh", secs / (60*60));
    }
    display.setCursor(display.width() - display.getTextWidth(tmp) - 2, 0);
    display.print(tmp);

    display.drawRect(0, 11, display.width(), 1);  // horiz line

    display.setCursor(0, 14);
    display.setColor(DisplayDriver::YELLOW);
    char filtered_origin[sizeof(p->origin)];
    display.translateUTF8ToBlocks(filtered_origin, p->origin, sizeof(filtered_origin));
    display.print(filtered_origin);

    display.setCursor(0, 25);
    display.setColor(DisplayDriver::LIGHT);
    char filtered_msg[sizeof(p->msg)];
    display.translateUTF8ToBlocks(filtered_msg, p->msg, sizeof(filtered_msg));
    display.printWordWrap(filtered_msg, display.width());

#if AUTO_OFF_MILLIS==0 // probably e-ink
    return 10000; // 10 s
#else
    return 1000;  // next render after 1000 ms
#endif
  }

  bool handleInput(char c) override {
#ifdef WITH_SLIM_UI
    // Older, then newer. Both stop at the end of the list rather than
    // wrapping or walking out of the screen: a list you can fall off the
    // bottom of is a list you cannot read carefully, and reading the last
    // message must not be the thing that closes it.
    if (c == KEY_NEXT || c == KEY_RIGHT || c == KEY_DOWN) {
      if (_at + 1 < num_msgs) _at++;
      num_unread = 0;
      return true;
    }
    if (c == KEY_PREV || c == KEY_UP) {
      if (_at > 0) _at--;
      num_unread = 0;
      return true;
    }
    if (c == KEY_ENTER || c == KEY_CANCEL) {
      num_unread = 0;             // read, but kept: see num_msgs
      _task->gotoHomeScreen();
      return true;
    }
#else
    if (c == KEY_NEXT || c == KEY_RIGHT) {
      head = (head + MAX_UNREAD_MSGS - 1) % MAX_UNREAD_MSGS;
      num_unread--;
      if (num_unread == 0) {
        _task->gotoHomeScreen();
      }
      return true;
    }
    if (c == KEY_ENTER) {
      num_unread = 0;  // clear unread queue
      _task->gotoHomeScreen();
      return true;
    }
#endif
#ifdef WITH_SLIM_UI
    // Answering is what somebody wants next, and reading the message
    // is the only moment they know who to answer.
    if (c == KEY_LEFT) {
      num_unread = 0;
      _task->gotoSend();
      return true;
    }

#endif
    return false;
  }
};

#ifdef WITH_SLIM_UI
// ---- saying something back, from the unit itself ------------------------
//
// Until now a handheld could hear the team and not answer it. Everything a
// unit sent was automatic - a position, an ack, a backlog - and anything a
// person wanted to say had to go through the phone in their pack. On a hill,
// in rain, that phone is not coming out.
//
// Three ways to say something, in the order they cost airtime:
//
//   a canned phrase   ONE byte, because slimtext carries it as an index into
//                     a table both ends already hold
//   sixteen typed     about thirteen bytes, because the alphabet offered
//     characters      here is PACK6 and nothing else, so it cannot fall to
//                     RAW by way of a character nobody meant to choose
//   a marker          the pin, broadcast to everyone rather than sent to the
//                     base: a hazard matters most to the team walking
//                     towards it, and routing it through a base that may be
//                     out of range would mean the people closest to it are
//                     the last to hear
//
// The canned list is not a shortcut for typing. It is a different order of
// cost: "NEED ASSIST" as one byte gets through a link that sixteen
// characters will not, and it is the message most worth getting through.

//: Sixteen characters. Long enough for "AT SADDLE 1430", short enough to
//: read at arm length on a 128-pixel line, which is what actually decides it.
#define COMPOSE_LEN 16

//: The icons from markers.py, in the order they are OFFERED - the three
//: somebody reaches for while moving first.
//:
//: `value` is what goes on the air, written down rather than implied by
//: position. markers.py's table is append-only because there the index
//: IS the wire value; here it is a field, so this list can be reordered
//: for the thumb without touching what any pin means. It was two arrays
//: with the order in one and the names in the other, which is a silent
//: "Gear drop" the first time an entry is missed.
struct MarkerIcon { uint8_t value; const char* name; };
static const MarkerIcon MARKER_ICONS[] = {
  {2, "Hazard"},        {0, "Gear drop"},      {12, "Point of interest"},
  {1, "Clue / find"},   {3, "Casualty"},       {4, "Rendezvous"},
  {5, "Helicopter LZ"}, {6, "Camp"},           {7, "Hut / shelter"},
  {8, "Water crossing"},{9, "Searched"},       {10, "Track blocked"},
  {11, "Vehicle"},
};
static const uint8_t MARKER_ICON_COUNT =
  (uint8_t)(sizeof(MARKER_ICONS) / sizeof(MARKER_ICONS[0]));

//: A list scrolled with the joystick. Four rows, which is what fits above
//: the hint line, and the selection is kept in view - a list that scrolls
//: off the top is a list nobody can reach the end of.
class PickScreen : public UIScreen {
protected:
  UITask* _task;
  uint8_t _n, _sel;
  const char* _title;

  virtual const char* label(uint8_t i) = 0;
  virtual void chose(uint8_t i) = 0;

public:
  PickScreen(UITask* task, const char* title, uint8_t n)
    : _task(task), _n(n), _sel(0), _title(title) { }


  int render(DisplayDriver& display) override {
    display.setTextSize(1);
    display.setColor(DisplayDriver::YELLOW);
    display.setCursor(0, 0);
    display.print(_title);
    display.drawRect(0, 11, display.width(), 1);

    const uint8_t rows = 4;
    const uint8_t first = _sel >= rows ? (uint8_t)(_sel - rows + 1) : 0;
    for (uint8_t r = 0; r < rows && (uint8_t)(first + r) < _n; r++) {
      const uint8_t i = (uint8_t)(first + r);
      const int y = 14 + r * 12;
      display.setColor(i == _sel ? DisplayDriver::GREEN : DisplayDriver::LIGHT);
      display.setCursor(0, y);
      display.print(i == _sel ? ">" : " ");
      display.setCursor(9, y);
      display.print(label(i));
    }
    // Nothing here has a clock or an animation, and UITask::loop already
    // forces a redraw on every keypress. At 400 ms this was pushing the
    // whole 1 KB framebuffer over I2C 2.5 times a second - 23 ms a flush,
    // blocking the task that also runs the radio - to draw the same picture.
    return 10000;
  }

  bool handleInput(char c) override {
    if (c == KEY_UP) {
      _sel = (uint8_t)((_sel + _n - 1) % _n);
      return true;
    }
    if (c == KEY_DOWN) {
      _sel = (uint8_t)((_sel + 1) % _n);
      return true;
    }
    // Back to the top on the way out, so the next open starts where
    // the eye does rather than where the last thumb left it.
    if (c == KEY_ENTER) {
      const uint8_t pick = _sel;
      _sel = 0;
      chose(pick);
      return true;
    }
    if (c == KEY_LEFT || c == KEY_CANCEL) {
      _sel = 0;
      _task->gotoHomeScreen();
      return true;
    }
    return false;
  }
};

//: Sixteen characters, one at a time, out of the alphabet the cheap tier can
//: carry. Up and down change the character under the cursor, left and right
//: move it, and left off the front is the way out - the way somebody came in.
class ComposeScreen : public UIScreen {
public:
  //: Not an icon, so it says "this is a message, not a label" without
  //: a second field to keep in step. Safe: an icon is five bits on the
  //: wire and 31 is already spoken for as ICON_REMOVED.
  static const uint8_t NO_ICON = 0xFF;

private:
  UITask* _task;
  char _buf[COMPOSE_LEN + 1];
  uint8_t _cur;
  uint8_t _icon;

  static uint8_t indexOf(char c) {
    const char* a = slimtext::PACK6_ALPHABET;
    for (uint8_t i = 0; a[i]; i++) {
      if (a[i] == c) return i;
    }
    return 0;   // a space, which is where every slot starts
  }

  //: What was actually typed: the buffer without the slots nobody filled in.
  //: Trimmed here rather than left to the codec, because sixteen spaces is
  //: not a message and must not cost a transmission.
  void trimmed(char* out) const {
    int n = COMPOSE_LEN;
    while (n > 0 && _buf[n - 1] == ' ') n--;
    int s = 0;
    while (s < n && _buf[s] == ' ') s++;
    memcpy(out, &_buf[s], (size_t)(n - s));
    out[n - s] = 0;
  }

public:
  ComposeScreen(UITask* task) : _task(task) { start(NO_ICON); }

  void start(uint8_t icon) {
    _icon = icon;
    _cur = 0;
    memset(_buf, ' ', COMPOSE_LEN);
    _buf[COMPOSE_LEN] = 0;
  }

  int render(DisplayDriver& display) override {
    display.setTextSize(1);
    display.setColor(DisplayDriver::YELLOW);
    display.setCursor(0, 0);
    display.print(_icon == NO_ICON ? "Message" : "Label");
    display.drawRect(0, 11, display.width(), 1);

    display.setColor(DisplayDriver::GREEN);
    display.setCursor(0, 20);
    display.print(_buf);

    // The caret, under the character the joystick is holding. Placed from the
    // measured width of one character rather than an assumed six, so it does
    // not drift away from the letter it points at on a different display.
    const int cw = display.getTextWidth("A");
    display.setColor(DisplayDriver::LIGHT);
    display.setCursor(_cur * cw, 30);
    display.print("^");

    char out[COMPOSE_LEN + 1];
    trimmed(out);
    char hint[40];
    // A marker says "place" and does not count characters it does not need:
    // the pin is the content and the label is decoration.
    if (_icon != NO_ICON) {
      sprintf(hint, "%s  place: %s",
              out[0] ? "with label" : "no label", PRESS_LABEL);
    } else {
      sprintf(hint, "%d chr  send: %s", (int)strlen(out), PRESS_LABEL);
    }
    display.setCursor(0, 64 - 11);
    display.print(hint);
    // Nothing here has a clock or an animation, and UITask::loop already
    // forces a redraw on every keypress. At 400 ms this was pushing the
    // whole 1 KB framebuffer over I2C 2.5 times a second - 23 ms a flush,
    // blocking the task that also runs the radio - to draw the same picture.
    return 10000;
  }

  bool handleInput(char c) override {
    const uint8_t n = slimtext::PACK6_COUNT;
    if (c == KEY_UP || c == KEY_DOWN) {
      const uint8_t i = indexOf(_buf[_cur]);
      _buf[_cur] = slimtext::PACK6_ALPHABET[
        c == KEY_UP ? (uint8_t)((i + 1) % n) : (uint8_t)((i + n - 1) % n)];
      return true;
    }
    if (c == KEY_RIGHT) {
      _cur = (uint8_t)((_cur + 1) % COMPOSE_LEN);
      return true;
    }
    if (c == KEY_LEFT) {
      // Off the front is out. Somebody who has typed nothing yet should not
      // have to learn a second gesture to leave.
      if (_cur == 0) {
        _task->gotoHomeScreen();
        return true;
      }
      _cur--;
      return true;
    }
    if (c == KEY_CANCEL) {
      _task->gotoHomeScreen();
      return true;
    }
    if (c == KEY_ENTER) {
      char out[COMPOSE_LEN + 1];
      trimmed(out);
      // **A marker needs no label.** Its icon and its place are already
      // chosen; the pin is the content. Refusing an empty one meant a pin
      // could not be placed at all without typing something, so every pin
      // dropped in the field carried whatever single character the joystick
      // happened to be resting on - "[" and a quote mark, in the log.
      //
      // A message is the other way round: with no text there is nothing to
      // send and a transmission would be spent on nothing.
      if (out[0] == 0 && _icon == NO_ICON) {
        _task->showAlert("Nothing to send", 1200);
        return true;
      }
      if (_icon != NO_ICON) {
        if (!the_mesh.placeMarker(_icon, out)) {
          // Nearly always the fix, and worth saying which: a pin placed
          // where the unit has never been is worse than no pin, because
          // it is a confident mark on ground nobody has walked. But if
          // there IS a fix the cause is the name - a unit not called
          // MTRKnn has no identity to sign a marker with - and sending
          // somebody to look at the sky for that would waste their time.
          double here_lat, here_lon;
          _task->showAlert(the_mesh.hereNow(here_lat, here_lon)
                             ? "Needs a callsign" : "No fix: not placed",
                           1500);
          return true;
        }
        // "Placing", not "placed": all that has happened is that the
        // frame is queued. The home screen carries it from here.
        _task->showAlert("Placing pin...", 1000);
      } else {
        const bool sent = the_mesh.sendSlimText(out);
        // See CannedScreen::chose: the home screen says what became of it.
        _task->showAlert(sent ? "Sending..." : "Could not send", 1000);
        if (!sent) _task->notify(UIEventType::trackerLost);
      }
      _task->gotoHomeScreen();
      return true;
    }
    return false;
  }
};

//: What to say, cheapest first. The last row is how to say something that is
//: not on the list.
class CannedScreen : public PickScreen {
protected:
  const char* label(uint8_t i) override {
    return i < slimtext::CODE_COUNT ? slimtext::CODES[i] : "Type a message";
  }
  void chose(uint8_t i) override;

public:
  CannedScreen(UITask* task)
    : PickScreen(task, "Send", (uint8_t)(slimtext::CODE_COUNT + 1)) { }
};

//: How strong the link to the base is, from where this unit is standing.
//:
//: Both directions, because they are not the same link and only one of
//: them matters: the base has the better antenna and more power, so DN
//: is routinely fine from places nothing gets out of. UP is the number
//: that decides whether a searcher appears on the map, so it is first
//: and on its own line.
class LinkTestScreen : public UIScreen {
  UITask* _task;
  //: The mesh's own number, not a second one drawn here: the screen must
  //: not give up before the receiver does. See MyMesh::LINK_TEST_WAIT_MS.
  static const uint32_t WAIT_MS = MyMesh::LINK_TEST_WAIT_MS;

public:
  LinkTestScreen(UITask* task) : _task(task) {}

  int render(DisplayDriver& display) override {
    char tmp[26];
    display.setColor(DisplayDriver::YELLOW);
    display.setTextSize(1);
    display.drawTextCentered(display.width() / 2, 0, "Range test");

    const auto& t = the_mesh.linkTest();
    const uint32_t waiting = the_mesh.linkTestWaiting();

    if (waiting && waiting < WAIT_MS) {
      display.setColor(DisplayDriver::LIGHT);
      display.setTextSize(2);
      display.drawTextCentered(display.width() / 2, 18, "asking...");
      // Counted down, because twelve seconds of a motionless word is a
      // unit somebody decides has frozen and switches off.
      display.setTextSize(1);
      sprintf(tmp, "%lus", (unsigned long)((WAIT_MS - waiting + 999) / 1000));
      display.drawTextCentered(display.width() / 2, 38, tmp);
    } else if (t.got_at) {
      // Whole numbers for RSSI, one decimal for SNR: that is the
      // precision each is actually good to, and a column of decimals
      // nobody needs is a column that is harder to read at arm's length.
      // Five lines in sixty-four pixels, and they have to not touch: the
      // round trip used to be drawn at y=52 under an 8-pixel footer line
      // starting at 55, so the one number somebody had walked up a hill to
      // read was printed through another line of text.
      display.setColor(DisplayDriver::GREEN);
      display.setTextSize(2);
      sprintf(tmp, "UP %d", (int)t.up_rssi);
      display.drawTextCentered(display.width() / 2, 12, tmp);
      display.setTextSize(1);
      sprintf(tmp, "%.1f dB", (double)t.up_snr);
      display.drawTextCentered(display.width() / 2, 28, tmp);
      display.setColor(DisplayDriver::LIGHT);
      sprintf(tmp, "DN %d  %.1f dB", (int)t.dn_rssi, (double)t.dn_snr);
      display.drawTextCentered(display.width() / 2, 38, tmp);
      // **The hop count is what says what the reading means.** -97 dBm
      // over one hop is a link to the base; -97 dBm over two is the
      // repeater doing the work and no direct path at all, and siting
      // anything on the first reading would be siting it for a link that
      // is not there.
      if (t.hops > 1) display.setColor(DisplayDriver::YELLOW);
      sprintf(tmp, "%u hop%s  %lu ms", (unsigned)t.hops,
              t.hops == 1 ? "" : "s",
              (unsigned long)(t.got_at - t.sent_at));
      display.drawTextCentered(display.width() / 2, 48, tmp);
    } else if (t.sent_at) {
      // A result in itself, and the one worth knowing: from here, the
      // base cannot be reached.
      display.setColor(DisplayDriver::RED);
      display.setTextSize(2);
      display.drawTextCentered(display.width() / 2, 24, "no reply");
    } else {
      display.setColor(DisplayDriver::LIGHT);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 26,
                               "press to measure");
    }
    display.setColor(DisplayDriver::LIGHT);
    display.setTextSize(1);
    display.drawTextCentered(display.width() / 2, 64 - 8, "back: left");
    // Half a second: the only thing that changes without a press is the
    // wait turning into a result.
    return 500;
  }

  bool handleInput(char c) override {
    if (c == KEY_LEFT || c == KEY_PREV) {
      _task->gotoHomeScreen();
      return true;
    }
    if (c == KEY_ENTER) {
      // One at a time, but only while it is genuinely still waiting.
      // linkTestWaiting() keeps counting up after a timeout - it means
      // "unanswered", not "in progress" - so testing it alone locked the
      // button for ever after a failure, which is the exact moment
      // somebody wants to press it again.
      const uint32_t waiting_now = the_mesh.linkTestWaiting();
      if (waiting_now && waiting_now < WAIT_MS) return true;
      if (!the_mesh.sendLinkTest()) {
        _task->showAlert("Not on MESH_SLIM", 1200);
      }
      return true;
    }
    return false;
  }
};

//: How often this unit reports, chosen from what there is.
//:
//: Off first, because that is the one somebody reaches for in a hurry
//: when a unit is going back in the box; then the working intervals,
//: shortest first. Four minutes is what an operation runs at - a place
//: on the map every minute, for one transmission every four - so it is
//: where the eye lands rather than where it has to travel.
static const uint32_t TRACK_EVERY[] = {0, 60, 120, 240, 300, 600, 900,
                                       1800};
static const uint8_t TRACK_EVERY_COUNT =
  (uint8_t)(sizeof(TRACK_EVERY) / sizeof(TRACK_EVERY[0]));

class TrackerScreen : public PickScreen {
  char _row[TRACK_EVERY_COUNT][16];

protected:
  const char* label(uint8_t i) override { return _row[i]; }
  void chose(uint8_t i) override;

public:
  TrackerScreen(UITask* task)
    : PickScreen(task, "Report every", TRACK_EVERY_COUNT) {
    for (uint8_t i = 0; i < TRACK_EVERY_COUNT; i++) {
      const uint32_t s = TRACK_EVERY[i];
      if (!s) strcpy(_row[i], "Off");
      else if (s < 120) strcpy(_row[i], "1 minute");
      else sprintf(_row[i], "%lu minutes", (unsigned long)(s / 60));
    }
  }

  //: Open on what this unit is doing now, so the list says which one
  //: is current rather than making somebody count rows to find out.
  void openHere() {
    const uint32_t now_every = the_mesh.trackerInterval();
    for (uint8_t i = 0; i < TRACK_EVERY_COUNT; i++) {
      if (TRACK_EVERY[i] == now_every) { _sel = i; return; }
    }
    _sel = 0;
  }
};

//: What kind of pin, and then what to call it.
class MarkerScreen : public PickScreen {
protected:
  const char* label(uint8_t i) override { return MARKER_ICONS[i].name; }
  void chose(uint8_t i) override;

public:
  MarkerScreen(UITask* task)
    : PickScreen(task, "Place marker", MARKER_ICON_COUNT) { }
};


void CannedScreen::chose(uint8_t i) {
  if (i >= slimtext::CODE_COUNT) {
    _task->composeFor(ComposeScreen::NO_ICON);
    return;
  }
  const bool sent = the_mesh.sendSlimText(slimtext::CODES[i]);
  // "Sending", not "Sent": all that has happened is that the frame is
  // queued. The home screen carries it from here and chirps when the base
  // acknowledges it - see MyMesh::Outgoing.
  _task->showAlert(sent ? "Sending..." : "Could not send", 1000);
  if (!sent) _task->notify(UIEventType::trackerLost);
  _task->gotoHomeScreen();
}

void TrackerScreen::chose(uint8_t i) {
  const uint32_t every = TRACK_EVERY[i];
  if (!the_mesh.setTrackerMode(every)) {
    _task->showAlert("No GPS: can't track", 1500);
    return;
  }
  char msg[24];
  if (every) sprintf(msg, "Every %lu min", (unsigned long)(every / 60));
  else strcpy(msg, "Tracker: OFF");
  _task->notify(UIEventType::ack);
  _task->showAlert(msg, 1000);
  _task->gotoHomeScreen();
}

void MarkerScreen::chose(uint8_t i) {
  _task->composeFor(MARKER_ICONS[i].value);
}
#endif

#ifdef WITH_SLIM_UI
void UITask::gotoInbox() {
  ((MsgPreviewScreen *) msg_preview)->openInbox();
  setCurrScreen(msg_preview);
}
int UITask::msgsUnread() const {
  return ((MsgPreviewScreen *) msg_preview)->unreadCount();
}
int UITask::msgsKept() const {
  return ((MsgPreviewScreen *) msg_preview)->keptCount();
}
void UITask::gotoSend() { setCurrScreen(send_menu); }
void UITask::gotoMarker() { setCurrScreen(marker_menu); }
void UITask::gotoLinkTest() { setCurrScreen(link_test); }
void UITask::gotoTrackerEvery() {
  every_menu->openHere();
  setCurrScreen(every_menu);
}
void UITask::composeFor(uint8_t icon) {
  compose->start(icon);
  setCurrScreen(compose);
}
#endif

void UITask::begin(DisplayDriver* display, SensorManager* sensors, NodePrefs* node_prefs) {
  _display = display;
  _sensors = sensors;
  _auto_off = millis() + AUTO_OFF_MILLIS;

#if defined(PIN_USER_BTN)
  user_btn.begin();
#endif
#if defined(PIN_USER_BTN_ANA)
  analog_btn.begin();
#endif

  _node_prefs = node_prefs;

  if (_display != NULL) {
    _display->turnOn();
  }

#ifdef PIN_BUZZER
  buzzer.begin();
  buzzer.quiet(_node_prefs->buzzer_quiet);
  buzzer.startup();
#endif

#ifdef PIN_VIBRATION
  vibration.begin();
#endif

  ui_started_at = millis();
  _alert_expiry = 0;

#ifdef WITH_MESH_SLIM
  splash = new SplashScreen(this, node_prefs);
#else
  splash = new SplashScreen(this);
#endif
  home = new HomeScreen(this, &rtc_clock, sensors, node_prefs);
  msg_preview = new MsgPreviewScreen(this, &rtc_clock);
#ifdef WITH_SLIM_UI
  send_menu = new CannedScreen(this);
  marker_menu = new MarkerScreen(this);
  every_menu = new TrackerScreen(this);
  link_test = new LinkTestScreen(this);
  compose = new ComposeScreen(this);
#endif
  setCurrScreen(splash);
}

void UITask::showAlert(const char* text, int duration_millis) {
  strcpy(_alert, text);
  _alert_expiry = millis() + duration_millis;
}

void UITask::notify(UIEventType t) {
#if defined(PIN_BUZZER)
switch(t){
  case UIEventType::contactMessage:
    // gemini's pick
    buzzer.play("MsgRcv3:d=4,o=6,b=200:32e,32g,32b,16c7");
    break;
  case UIEventType::channelMessage:
    buzzer.play("kerplop:d=16,o=6,b=120:32g#,32c#");
    break;
  case UIEventType::ack:
    buzzer.play("ack:d=32,o=8,b=120:c");
    break;
#ifdef WITH_TRACKER_MODE
  // Behind the flag, so a stock image carries neither the cases nor
  // the two strings: the rule in this fork is that a build with the
  // feature off is byte-identical to the baseline, and a tune costs
  // real flash on a board that is already at 70%.
  case UIEventType::trackerHeard:
    // Two rising notes, about 40 ms each: short enough to be a tick
    // rather than a tune, and rising because it is good news.
    buzzer.play("trkok:d=32,o=7,b=200:c,e");
    break;
  case UIEventType::trackerLost:
    // Three, lower, with air between them. Three of anything reads as
    // a problem where one reads as an event, and the pauses are what
    // make it countable from inside a pack.
    buzzer.play("trklost:d=16,o=5,b=200:a,p,a,p,a");
    break;
#endif
  case UIEventType::roomMessage:
  case UIEventType::newContactMessage:
  case UIEventType::none:
  default:
    break;
}
#endif

#ifdef PIN_VIBRATION
  // Trigger vibration for all UI events except none
  if (t != UIEventType::none) {
    vibration.trigger();
  }
#endif
}


void UITask::msgRead(int msgcount) {
  _msgcount = msgcount;
  if (msgcount == 0) {
    gotoHomeScreen();
  }
}

void UITask::newMsg(uint8_t path_len, const char* from_name, const char* text, int msgcount) {
  _msgcount = msgcount;

  ((MsgPreviewScreen *) msg_preview)->addPreview(path_len, from_name, text);
  setCurrScreen(msg_preview);

  if (_display != NULL) {
    if (!_display->isOn() && !hasConnection()) {
      _display->turnOn();
    }
    if (_display->isOn()) {
    _auto_off = millis() + AUTO_OFF_MILLIS;  // extend the auto-off timer
    _next_refresh = 100;  // trigger refresh
    }
  }
}

void UITask::userLedHandler() {
#ifdef PIN_STATUS_LED
  int cur_time = millis();
  if (cur_time > next_led_change) {
    if (led_state == 0) {
      led_state = 1;
      if (_msgcount > 0) {
        last_led_increment = LED_ON_MSG_MILLIS;
      } else {
        last_led_increment = LED_ON_MILLIS;
      }
      next_led_change = cur_time + last_led_increment;
    } else {
      led_state = 0;
      next_led_change = cur_time + LED_CYCLE_MILLIS - last_led_increment;
    }
    digitalWrite(PIN_STATUS_LED, led_state == LED_STATE_ON);
  }
#endif
}

void UITask::setCurrScreen(UIScreen* c) {
  curr = c;
  _next_refresh = 100;
}

/*
  hardware-agnostic pre-shutdown activity should be done here
*/
void UITask::shutdown(bool restart){

  #ifdef PIN_BUZZER
  /* note: we have a choice here -
     we can do a blocking buzzer.loop() with non-deterministic consequences
     or we can set a flag and delay the shutdown for a couple of seconds
     while a non-blocking buzzer.loop() plays out in UITask::loop()
  */
  buzzer.shutdown();
  uint32_t buzzer_timer = millis(); // fail-safe shutdown
  while (buzzer.isPlaying() && (millis() - 2500) < buzzer_timer)
    buzzer.loop();

  #endif // PIN_BUZZER

  if (restart) {
    _board->reboot();
  } else {
    _display->turnOff();
    radio_driver.powerOff();
    _board->powerOff();
  }
}

bool UITask::isButtonPressed() const {
#ifdef PIN_USER_BTN
  return user_btn.isPressed();
#else
  return false;
#endif
}

void UITask::loop() {
  char c = 0;
#if UI_HAS_JOYSTICK
  int ev = user_btn.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_ENTER);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_ENTER);  // REVISIT: could be mapped to different key code
  }
  ev = joystick_left.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_LEFT);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_LEFT);
  }
  ev = joystick_right.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_RIGHT);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_RIGHT);
  }
#if defined(JOYSTICK_UP) && defined(WITH_MESH_SLIM)
  ev = joystick_up.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_UP);
  }
  ev = joystick_down.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_DOWN);
  }
#endif
  ev = back_btn.check();
  if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
    c = handleTripleClick(KEY_SELECT);
  }
#elif defined(PIN_USER_BTN)
  int ev = user_btn.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_NEXT);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_ENTER);
  } else if (ev == BUTTON_EVENT_DOUBLE_CLICK) {
    c = handleDoubleClick(KEY_PREV);
  } else if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
    c = handleTripleClick(KEY_SELECT);
  }
#endif
#if defined(PIN_USER_BTN_ANA)
  if (abs(millis() - _analogue_pin_read_millis) > 10) {
    int ev = analog_btn.check();
    if (ev == BUTTON_EVENT_CLICK) {
      c = checkDisplayOn(KEY_NEXT);
    } else if (ev == BUTTON_EVENT_LONG_PRESS) {
      c = handleLongPress(KEY_ENTER);
    } else if (ev == BUTTON_EVENT_DOUBLE_CLICK) {
      c = handleDoubleClick(KEY_PREV);
    } else if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
      c = handleTripleClick(KEY_SELECT);
    }
    _analogue_pin_read_millis = millis();
  }
#endif
#if defined(BACKLIGHT_BTN)
  if (millis() > next_backlight_btn_check) {
    bool touch_state = digitalRead(PIN_BUTTON2);
#if defined(DISP_BACKLIGHT)
    digitalWrite(DISP_BACKLIGHT, !touch_state);
#elif defined(EXP_PIN_BACKLIGHT)
    expander.digitalWrite(EXP_PIN_BACKLIGHT, !touch_state);
#endif
    next_backlight_btn_check = millis() + 300;
  }
#endif

  if (c != 0 && curr) {
    curr->handleInput(c);
    _auto_off = millis() + AUTO_OFF_MILLIS;   // extend auto-off timer
    _next_refresh = 100;  // trigger refresh
  }

  userLedHandler();

#ifdef PIN_BUZZER
  if (buzzer.isPlaying())  buzzer.loop();
#endif

  if (curr) curr->poll();

  if (_display != NULL && _display->isOn()) {
    if (millis() >= _next_refresh && curr) {
      _display->startFrame();
      int delay_millis = curr->render(*_display);
      if (millis() < _alert_expiry) {  // render alert popup
        _display->setTextSize(1);
        int y = _display->height() / 3;
        int p = _display->height() / 32;
        _display->setColor(DisplayDriver::DARK);
        _display->fillRect(p, y, _display->width() - p*2, y);
        _display->setColor(DisplayDriver::LIGHT);  // draw box border
        _display->drawRect(p, y, _display->width() - p*2, y);
        _display->drawTextCentered(_display->width() / 2, y + p*3, _alert);
        _next_refresh = _alert_expiry;   // will need refresh when alert is dismissed
      } else {
        _next_refresh = millis() + delay_millis;
      }
      _display->endFrame();
    }
#if AUTO_OFF_MILLIS > 0
#ifdef KEEP_DISPLAY_ON_USB
    // Opt-in: refresh the auto-off deadline while externally powered, so the
    // timer counts from the moment external power is removed. Off by default
    // because OLED panels burn in quickly; only enable for LCD targets or
    // where the display is replaceable.
    if (board.isExternalPowered()) {
      _auto_off = millis() + AUTO_OFF_MILLIS;
    }
#endif
    if (millis() > _auto_off) {
      _display->turnOff();
    }
#endif
  }

#ifdef PIN_VIBRATION
  vibration.loop();
#endif

#ifdef WITH_SLIM_UI
  // Said while something can still be done about it. The hard cutoff
  // below announces itself and shuts down in the same breath, which is
  // after the decision rather than before it.
  if (millis() > _next_batt_warn) {
    _next_batt_warn = millis() + 30000;
    const uint16_t mv = getBattMilliVolts();
    // Ignored on external power and when the reading is not available:
    // 0 means "no divider fitted", not "flat".
    if (mv > 0 && !board.isExternalPowered()) {
      const int pct = (int)slimhealth::batteryPercent(mv);
      // Ten first, so a unit switched on already below it gives the
      // warning that matters rather than the gentler one.
      if (pct <= 10 && !(_batt_warned & 2)) {
        _batt_warned |= 3;        // ...and never the 20% one after it
        notify(UIEventType::trackerLost);
        showAlert("Battery 10% - RTB", 4000);
      } else if (pct <= 20 && !(_batt_warned & 1)) {
        _batt_warned |= 1;
        notify(UIEventType::trackerLost);
        showAlert("Battery low: 20%", 3000);
      }
    }
  }
#endif

#ifdef AUTO_SHUTDOWN_MILLIVOLTS
  if (millis() > next_batt_chck) {
    uint16_t milliVolts = getBattMilliVolts();
    if (milliVolts > 0 && milliVolts < AUTO_SHUTDOWN_MILLIVOLTS) {
      if(!board.isExternalPowered()) {
        if (_display != NULL) {
          _display->startFrame();
          _display->setTextSize(2);
          _display->setColor(DisplayDriver::RED);
          _display->drawTextCentered(_display->width() / 2, 20, "Low Battery.");
          _display->drawTextCentered(_display->width() / 2, 40, "Shutting Down!");
          _display->endFrame();
          if (_display->isEink() == false) { delay(3000); }
        }
        shutdown();
      }
    }
    next_batt_chck = millis() + 8000;
  }
#endif
}

char UITask::checkDisplayOn(char c) {
  if (_display != NULL) {
    if (!_display->isOn()) {
      _display->turnOn();   // turn display on and consume event
      c = 0;
    }
    _auto_off = millis() + AUTO_OFF_MILLIS;   // extend auto-off timer
    _next_refresh = 0;  // trigger refresh
  }
  return c;
}

char UITask::handleLongPress(char c) {
  if (millis() - ui_started_at < 8000) {   // long press in first 8 seconds since startup -> CLI/rescue
    the_mesh.enterCLIRescue();
    c = 0;   // consume event
  }
  return c;
}

char UITask::handleDoubleClick(char c) {
  MESH_DEBUG_PRINTLN("UITask: double-click triggered");
  checkDisplayOn(c);
  return c;
}

char UITask::handleTripleClick(char c) {
  MESH_DEBUG_PRINTLN("UITask: triple click triggered");
  checkDisplayOn(c);
  toggleBuzzer();
  c = 0;
  return c;
}

bool UITask::getGPSState() {
  if (_sensors != NULL) {
    int num = _sensors->getNumSettings();
    for (int i = 0; i < num; i++) {
      if (strcmp(_sensors->getSettingName(i), "gps") == 0) {
        return !strcmp(_sensors->getSettingValue(i), "1");
      }
    }
  }
  return false;
}

void UITask::toggleGPS() {
    if (_sensors != NULL) {
    // toggle GPS on/off
    int num = _sensors->getNumSettings();
    for (int i = 0; i < num; i++) {
      if (strcmp(_sensors->getSettingName(i), "gps") == 0) {
        if (strcmp(_sensors->getSettingValue(i), "1") == 0) {
          _sensors->setSettingValue("gps", "0");
          _node_prefs->gps_enabled = 0;
          notify(UIEventType::ack);
        } else {
          _sensors->setSettingValue("gps", "1");
          _node_prefs->gps_enabled = 1;
          notify(UIEventType::ack);
        }
        the_mesh.savePrefs();
        showAlert(_node_prefs->gps_enabled ? "GPS: Enabled" : "GPS: Disabled", 800);
        _next_refresh = 0;
        break;
      }
    }
  }
}

void UITask::toggleBuzzer() {
    // Toggle buzzer quiet mode
  #ifdef PIN_BUZZER
    if (buzzer.isQuiet()) {
      buzzer.quiet(false);
      notify(UIEventType::ack);
    } else {
      buzzer.quiet(true);
    }
    _node_prefs->buzzer_quiet = buzzer.isQuiet();
    the_mesh.savePrefs();
    showAlert(buzzer.isQuiet() ? "Buzzer: OFF" : "Buzzer: ON", 800);
    _next_refresh = 0;  // trigger refresh
  #endif
}
