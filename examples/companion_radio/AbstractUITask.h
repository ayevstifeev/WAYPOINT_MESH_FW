#pragma once

#include <MeshCore.h>
#include <helpers/ui/DisplayDriver.h>
#include <helpers/ui/UIScreen.h>
#include <helpers/SensorManager.h>
#include <helpers/BaseSerialInterface.h>
#include <Arduino.h>

#ifdef PIN_BUZZER
  #include <helpers/ui/buzzer.h>
#endif

#include "NodePrefs.h"

//: The screens that let somebody answer and mark ground, and the MyMesh
//: methods they call.
//:
//: One name because the two halves were guarded on different
//: predicates - the screens on slim-and-a-joystick, placeMarker() and
//: hereNow() on tracker mode - and every current build happens to set
//: both. That is the same shape as the guard mismatch that stopped
//: SLIM_RAK_4631_base compiling: it breaks on the first variant that
//: does not, not on the one in front of you.
#if defined(WITH_MESH_SLIM) && defined(WITH_TRACKER_MODE) \
    && defined(UI_HAS_JOYSTICK) && UI_HAS_JOYSTICK
  #define WITH_SLIM_UI 1
#endif

enum class UIEventType {
    none,
    contactMessage,
    channelMessage,
    roomMessage,
    newContactMessage,
    ack,
    //: A tracker report the base acknowledged, and a tracker that has
    //: stopped getting through. Their own events rather than reusing
    //: `ack`, because they are the answer to a different question: not
    //: "did my message arrive", asked by somebody holding the unit and
    //: looking at it, but "are we still in contact", asked by somebody
    //: walking with it in a pack who will never look at it at all.
    trackerHeard,
    trackerLost
};

class AbstractUITask {
protected:
  mesh::MainBoard* _board;
  BaseSerialInterface* _serial;
  bool _connected;

  AbstractUITask(mesh::MainBoard* board, BaseSerialInterface* serial) : _board(board), _serial(serial) {
    _connected = false;
  }

public:
  void setHasConnection(bool connected) { _connected = connected; }
  bool hasConnection() const { return _connected; }
  uint16_t getBattMilliVolts() const { return _board->getBattMilliVolts(); }
  bool isSerialEnabled() const { return _serial->isEnabled(); }
  void enableSerial() { _serial->enable(); }
  void disableSerial() { _serial->disable(); }
  virtual void msgRead(int msgcount) = 0;
  virtual void newMsg(uint8_t path_len, const char* from_name, const char* text, int msgcount) = 0;
  virtual void notify(UIEventType t = UIEventType::none) = 0;
  virtual void loop() = 0;
};
