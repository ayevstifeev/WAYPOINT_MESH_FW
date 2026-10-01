#pragma once

#include <MeshCore.h>
#include <helpers/ui/DisplayDriver.h>
#include <helpers/ui/UIScreen.h>
#include <helpers/SensorManager.h>
#include <helpers/BaseSerialInterface.h>
#include <Arduino.h>
#include <helpers/sensors/LPPDataHelpers.h>

#ifndef LED_STATE_ON
  #define LED_STATE_ON 1
#endif

#ifdef PIN_BUZZER
  #include <helpers/ui/buzzer.h>
#endif
#ifdef PIN_VIBRATION
  #include <helpers/ui/GenericVibration.h>
#endif

#include "../AbstractUITask.h"
#include "../NodePrefs.h"

#ifdef WITH_SLIM_UI
//: Defined in UITask.cpp; held here only as pointers.
class ComposeScreen;
class TrackerScreen;
class LinkTestScreen;
#endif

class UITask : public AbstractUITask {
  DisplayDriver* _display;
  SensorManager* _sensors;
#ifdef PIN_BUZZER
  genericBuzzer buzzer;
#endif
#ifdef PIN_VIBRATION
  GenericVibration vibration;
#endif
  unsigned long _next_refresh, _auto_off;
#ifdef WITH_SLIM_UI
  //: Which low-battery warnings have been given. Latched, so a unit
  //: does not alert every thirty seconds for the last hour of its
  //: life - which is how people learn to ignore an alert.
  uint8_t _batt_warned = 0;
  //: Not before this. Zero would check on the first turn of the loop,
  //: when the battery ADC has not settled and a unit switched on with a
  //: full cell can read low for a moment - which is a false alarm in the
  //: first seconds of a callout, the worst possible time to cry wolf.
  unsigned long _next_batt_warn = 20000;
#endif
  NodePrefs* _node_prefs;
  char _alert[80];
  unsigned long _alert_expiry;
  int _msgcount;
  unsigned long ui_started_at, next_batt_chck;
  int next_backlight_btn_check = 0;
#ifdef PIN_STATUS_LED
  int led_state = 0;
  int next_led_change = 0;
  int last_led_increment = 0;
#endif

#ifdef PIN_USER_BTN_ANA
  unsigned long _analogue_pin_read_millis = millis();
#endif

  UIScreen* splash;
  UIScreen* home;
  UIScreen* msg_preview;
  UIScreen* curr;
#ifdef WITH_SLIM_UI
  UIScreen* send_menu;
  UIScreen* marker_menu;
  TrackerScreen* every_menu;
  UIScreen* link_test;
  ComposeScreen* compose;
#endif

  void userLedHandler();

  // Button action handlers
  char checkDisplayOn(char c);
  char handleLongPress(char c);
  char handleDoubleClick(char c);
  char handleTripleClick(char c);

  void setCurrScreen(UIScreen* c);

public:

  UITask(mesh::MainBoard* board, BaseSerialInterface* serial) : AbstractUITask(board, serial), _display(NULL), _sensors(NULL) {
    next_batt_chck = _next_refresh = 0;
    ui_started_at = 0;
    curr = NULL;
  }
  void begin(DisplayDriver* display, SensorManager* sensors, NodePrefs* node_prefs);

  void gotoHomeScreen() { setCurrScreen(home); }
#ifdef WITH_SLIM_UI
  //: The screens that let somebody answer, and mark ground, without
  //: the phone coming out of the pack.
  //: The messages this unit has been sent, newest first, kept until the
  //: ring wraps. Reached from the Messages page as well as by one
  //: arriving - a message that can only be read in the seconds after it
  //: lands is a message that gets missed.
  void gotoInbox();
  int msgsUnread() const;
  int msgsKept() const;
  void gotoSend();
  void gotoMarker();
  //: The interval list, opened on whatever this unit is doing now.
  void gotoTrackerEvery();
  //: The range test. Not on the carousel - five presses on the info
  //: page, because it is an engineer's screen, not an operator's.
  void gotoLinkTest();
  //: Open the composer: a label for a marker of that icon, or
  //: ComposeScreen::NO_ICON for an ordinary message.
  void composeFor(uint8_t icon);
#endif
  void showAlert(const char* text, int duration_millis);
  int  getMsgCount() const { return _msgcount; }
  bool hasDisplay() const { return _display != NULL; }
  bool isButtonPressed() const;

  bool isBuzzerQuiet() { 
#ifdef PIN_BUZZER
    return buzzer.isQuiet();
#else
    return true;
#endif
  }

  void toggleBuzzer();
  bool getGPSState();
  void toggleGPS();


  // from AbstractUITask
  void msgRead(int msgcount) override;
  void newMsg(uint8_t path_len, const char* from_name, const char* text, int msgcount) override;
  void notify(UIEventType t = UIEventType::none) override;
  void loop() override;

  void shutdown(bool restart = false);
};
