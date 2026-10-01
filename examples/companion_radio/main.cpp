#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include "MyMesh.h"
#include <helpers/ArduinoHelpers.h>


// ---------------------------------------------------------------------------
// RFID / NFC check-in - off unless the build asks for it.
//
// Add `-D WITH_RFID_CHECKIN=1` to an env's build_flags to compile it in
// (RFID_RAK_4631_companion_radio_ble does). Guarded the same way as
// WITH_MESH_SLIM, so there is one convention for "feature compiled out".
//
// Off by default because:
//   1. It halts a board with no shield: setup() loops forever if
//      getFirmwareVersion() returns nothing, so a unit without a PN532
//      never joins the mesh.
//   2. It is RAK WisBlock hardware (WB_IO pins, the RAK13600 library), so
//      no other board - the Wio Tracker L1 - can compile it.
//   3. It cannot work in slim mode: sendCheckInPacket() builds a MeshCore
//      packet, which the slim codec cannot carry.
// An RFID build also keeps sensors and sleep off, exactly as it was written,
// and waits up to 5 s for a serial monitor at boot.
// ---------------------------------------------------------------------------
#ifdef WITH_RFID_CHECKIN
#include <RAK13600-PN532.h>
#include <Wire.h>
#include <bluefruit.h>

// MeshCore destination for the check-in payloads
#define CHECKIN_TARGET_NODE_HEX "4f31c659a3165ced532096f45ebc58b42a06801881a467a6ce329e69e4bac1d4"
#define RFID_POLLING_INTERVAL_MS 500
static uint32_t last_rfid_poll = 0;
uint32_t lastCheckInTime = 0;
// WisBlock standard I2C. (Update IRQ/RST if wired explicitly)
#define PN532_IRQ   (WB_IO6)
#define PN532_RESET (WB_IO5)  // Not connected by default on the NFC Shield
NFC_PN532 nfc(PN532_IRQ, PN532_RESET);
#endif  // WITH_RFID_CHECKIN

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  #include <InternalFileSystem.h>
  #if defined(QSPIFLASH)
    #include <CustomLFS_QSPIFlash.h>
    DataStore store(InternalFS, QSPIFlash, rtc_clock);
  #else
  #if defined(EXTRAFS)
    #include <CustomLFS.h>
    CustomLFS ExtraFS(0xD4000, 0x19000, 128);
    DataStore store(InternalFS, ExtraFS, rtc_clock);
  #else
    DataStore store(InternalFS, rtc_clock);
  #endif
  #endif
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
  DataStore store(LittleFS, rtc_clock);
#elif defined(ESP32)
  #include <SPIFFS.h>
  DataStore store(SPIFFS, rtc_clock);
#endif

#ifdef ESP32
  #ifdef WIFI_SSID
    #include <helpers/esp32/SerialWifiInterface.h>
    SerialWifiInterface serial_interface;
    #ifndef TCP_PORT
      #define TCP_PORT 5000
    #endif
  #elif defined(BLE_PIN_CODE)
    #include <helpers/esp32/SerialBLEInterface.h>
    SerialBLEInterface serial_interface;
  #elif defined(SERIAL_RX)
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
    HardwareSerial companion_serial(1);
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#elif defined(RP2040_PLATFORM)
  //#ifdef WIFI_SSID
  //  #include <helpers/rp2040/SerialWifiInterface.h>
  //  SerialWifiInterface serial_interface;
  //  #ifndef TCP_PORT
  //    #define TCP_PORT 5000
  //  #endif
  // #elif defined(BLE_PIN_CODE)
  //   #include <helpers/rp2040/SerialBLEInterface.h>
  //   SerialBLEInterface serial_interface;
  #if defined(SERIAL_RX)
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
    HardwareSerial companion_serial(1);
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#elif defined(NRF52_PLATFORM)
 
  #ifdef BLE_PIN_CODE
    #include <helpers/nrf52/SerialBLEInterface.h>
    SerialBLEInterface serial_interface;
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
  
#elif defined(STM32_PLATFORM)
  #include <helpers/ArduinoSerialInterface.h>
  ArduinoSerialInterface serial_interface;
#else
  #error "need to define a serial interface"
#endif

/* GLOBAL OBJECTS */
#ifdef DISPLAY_CLASS
  #include "UITask.h"
  UITask ui_task(&board, &serial_interface);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables, store
   #ifdef DISPLAY_CLASS
      , &ui_task
   #endif
);

/* END GLOBAL OBJECTS */

void halt() {
  while (1) ;
}

/* WIFI RECONNECT TRACKERS */
#if defined(ESP32) && defined(WIFI_SSID)
  bool wifi_needs_reconnect = false;
  unsigned long last_wifi_reconnect_attempt = 0;
#endif



#ifdef WITH_RFID_CHECKIN
void sendCheckInPacket(String source, String checkInId) {
    // 5-second debounce to prevent spamming the mesh
    if (millis() - lastCheckInTime < 5000) return; 
    
    // Construct the simple JSON text payload
    String payload = "{\"src\":\"" + source + "\",\"id\":\"" + checkInId + "\"}";
    
    // 1. Construct the recipient. 
    // NOTE: You must update ".id" to match whatever your ContactInfo struct calls 
    // the node identifier (e.g., .id, .node_id, or .dest).
    ContactInfo target;
    memset(&target, 0, sizeof(target));
    target.out_path_len = OUT_PATH_UNKNOWN; 


    target.id = mesh::Identity(CHECKIN_TARGET_NODE_HEX);
    
    // 2. Prepare the output variables required by the reference parameters (&)
    uint32_t expected_ack = 0;
    uint32_t est_timeout = 0;


    uint32_t now = the_mesh.getRTCClock()->getCurrentTimeUnique();

    
    // 3. Inject the message into the mesh

    int result = the_mesh.sendMessage(
        target,             // The ContactInfo recipient object
        now,                  // Timestamp (0 usually defaults to current time)
        0,                  // Attempt counter (0 for the initial transmission)
        payload.c_str(),    // The null-terminated C-string payload
        expected_ack,       // Passed by reference; updated by the function
        est_timeout         // Passed by reference; updated by the function
    );
    
    // Check the return integer (assuming >= 0 is a success/message ID)
    if (result != MSG_SEND_FAILED) {
        Serial.println("Forwarded check-in via MeshCore: " + payload);
        Serial.print("Expected ACK in: ");
        Serial.println(est_timeout);
    } else {
        Serial.println("Error: Failed to route check-in packet.");
    }
    
    lastCheckInTime = millis();
}
#endif  // WITH_RFID_CHECKIN


#if defined(WITH_MESH_SLIM) && defined(NRF52_PLATFORM)
//: Seconds without a turn of the main loop before the firmware is
//: restarted.
//:
//: Two minutes is far beyond anything legitimate - the longest blocking
//: call here is a display flush at about 90 ms - so what it catches is a
//: hang, not a slow moment. 0 builds without one.
#ifndef MESH_SLIM_WDT_S
  #define MESH_SLIM_WDT_S 120
#endif

#if MESH_SLIM_WDT_S > 0
#include <nrfx_wdt.h>

//: Started once and never stopped - a watchdog software can switch off
//: is one a hung firmware can switch off. The HAL is used directly
//: rather than nrfx_wdt_init(), which wants an IRQ handler for a
//: timeout event that cannot be serviced anyway: by the time it fires,
//: the thing that would run the handler is what has stopped.
static void wdtBegin() {
  // RUN, not PAUSE, while the CPU sleeps - and that is safe here only
  // because of how a tracker sleeps. It does NOT block for its whole
  // interval: trackerSleep() naps in TRACKER_SLEEP_SLICE_MS slices and
  // returns to loop() each time, so the loop keeps turning at 5 Hz even
  // at a thirty-minute interval and the feed never lapses.
  //
  // Pausing would have been the cautious choice and a weaker one: it
  // measures only time the CPU is awake, so a hang that idles - a
  // deadlock, a task that never wakes - would never trip it. Those are
  // exactly the hangs a watchdog is for.
  //
  // Hibernation is unaffected: System OFF stops the WDT with everything
  // else, and coming back out of it is a reset either way.
  nrf_wdt_behaviour_set(NRF_WDT, NRF_WDT_BEHAVIOUR_RUN_SLEEP_HALT);
  // The counter runs at 32768 Hz off the low-frequency clock.
  nrf_wdt_reload_value_set(NRF_WDT, MESH_SLIM_WDT_S * 32768UL);
  nrf_wdt_reload_request_enable(NRF_WDT, NRF_WDT_RR0);
  nrf_wdt_task_trigger(NRF_WDT, NRF_WDT_TASK_START);
}

static inline void wdtFeed() {
  nrf_wdt_reload_request_set(NRF_WDT, NRF_WDT_RR0);
}
#else
static inline void wdtBegin() {}
static inline void wdtFeed() {}
#endif
#else
static inline void wdtBegin() {}
static inline void wdtFeed() {}
#endif

void setup() {
  Serial.begin(115200);


#ifdef WITH_RFID_CHECKIN
// Wait for Serial Monitor to connect, but timeout after 5 seconds
  // so the board can still boot normally when running on battery power.
  uint32_t start = millis();
  while (!Serial && (millis() - start < 5000)) {
      delay(10); 
  }

  
//-------------------------------------------
//RFID READER INIT BLOCK
//-------------------------------------------

  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(300);

  Serial.println("RFID Reader Initializing");

    nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);

  // configure board to read RFID tags
  nfc.SAMConfig();
#endif  // WITH_RFID_CHECKIN

//-------------------------------------------
//RFID READER INIT BLOCK
//-------------------------------------------






  board.begin();

#ifdef DISPLAY_CLASS
  DisplayDriver* disp = NULL;
  if (display.begin()) {
    disp = &display;
    disp->startFrame();
  #ifdef ST7789
    disp->setTextSize(2);
  #endif
    disp->drawTextCentered(disp->width() / 2, 28, "Loading...");
    disp->endFrame();
  }
#endif



  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  #if defined(QSPIFLASH)
    if (!QSPIFlash.begin()) {
      // debug output might not be available at this point, might be too early. maybe should fall back to InternalFS here?
      MESH_DEBUG_PRINTLN("CustomLFS_QSPIFlash: failed to initialize");
    } else {
      MESH_DEBUG_PRINTLN("CustomLFS_QSPIFlash: initialized successfully");
    }
  #else
  #if defined(EXTRAFS)
      ExtraFS.begin();
  #endif
  #endif
  store.begin();
  the_mesh.begin(
    #ifdef DISPLAY_CLASS
        disp != NULL
    #else
        false
    #endif
  );

#ifdef BLE_PIN_CODE
  serial_interface.begin(BLE_NAME_PREFIX, the_mesh.getNodePrefs()->node_name, the_mesh.getBLEPin());
#else
  serial_interface.begin(Serial);
#endif
  the_mesh.startInterface(serial_interface);
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  store.begin();
  the_mesh.begin(
    #ifdef DISPLAY_CLASS
        disp != NULL
    #else
        false
    #endif
  );

  //#ifdef WIFI_SSID
  //  WiFi.begin(WIFI_SSID, WIFI_PWD);
  //  serial_interface.begin(TCP_PORT);
  // #elif defined(BLE_PIN_CODE)
  //   char dev_name[32+16];
  //   sprintf(dev_name, "%s%s", BLE_NAME_PREFIX, the_mesh.getNodeName());
  //   serial_interface.begin(dev_name, the_mesh.getBLEPin());
  #if defined(SERIAL_RX)
    companion_serial.setPins(SERIAL_RX, SERIAL_TX);
    companion_serial.begin(115200);
    serial_interface.begin(companion_serial);
  #else
    serial_interface.begin(Serial);
  #endif
    the_mesh.startInterface(serial_interface);
#elif defined(ESP32)
  SPIFFS.begin(true);
  store.begin();
  the_mesh.begin(
    #ifdef DISPLAY_CLASS
        disp != NULL
    #else
        false
    #endif
  );

#ifdef WIFI_SSID
  board.setInhibitSleep(true);   // prevent sleep when WiFi is active
  WiFi.setAutoReconnect(true);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
      if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
          WIFI_DEBUG_PRINTLN("WiFi disconnected. Flagging for reconnect...");
          wifi_needs_reconnect = true;
      } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
          WIFI_DEBUG_PRINTLN("WiFi connected successfully!");
          wifi_needs_reconnect = false;
      }
  });

  WiFi.begin(WIFI_SSID, WIFI_PWD);
  serial_interface.begin(TCP_PORT);
#elif defined(BLE_PIN_CODE)
  serial_interface.begin(BLE_NAME_PREFIX, the_mesh.getNodePrefs()->node_name, the_mesh.getBLEPin());
#elif defined(SERIAL_RX)
  companion_serial.setPins(SERIAL_RX, SERIAL_TX);
  companion_serial.begin(115200);
  serial_interface.begin(companion_serial);
#else
  serial_interface.begin(Serial);
#endif
  the_mesh.startInterface(serial_interface);
#else
  #error "need to define filesystem"
#endif

#ifndef WITH_RFID_CHECKIN
  sensors.begin();   // finds and starts the GPS; off only in the RFID build
#endif

#if ENV_INCLUDE_GPS == 1
  the_mesh.applyGpsPrefs();
#endif
#ifdef WITH_TRACKER_MODE
  the_mesh.loadTrackerMode();   // after sensors.begin(): it needs the GPS found
#endif

#ifdef DISPLAY_CLASS
  ui_task.begin(disp, &sensors, the_mesh.getNodePrefs());  // still want to pass this in as dependency, as prefs might be moved
#endif

  // Last, so nothing in setup() is being timed: a slow filesystem mount
  // or a GPS that takes its time is not a hang, and starting the
  // watchdog before them would make it one. From here the loop is what
  // has to keep turning. See wdtBegin - it cannot be stopped again.
  wdtBegin();
  board.onBootComplete();
}

void loop() {
  // First thing, and from here rather than from inside any one
  // subsystem: this loop is what has to keep turning, and a feed buried
  // in the radio path would go on being fed by a firmware whose UI had
  // stopped.
  wdtFeed();
  the_mesh.loop();

  

#ifndef WITH_RFID_CHECKIN
  sensors.loop();
#endif
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();


#ifdef WITH_RFID_CHECKIN
  // --- ADDED NFC POLLING ---
  // Poll the I2C bus every 200ms
  if (millis() - last_rfid_poll > RFID_POLLING_INTERVAL_MS) {
      Serial.println("POLLING RFID...");
      uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
      uint8_t uidLength;
      
      // VERY IMPORTANT: The '50' is a 50ms timeout. 
      // If omitted, the default PN532 library blocks the main thread indefinitely 
      // until a card is read, which will completely kill your mesh routing!
      if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength,100)){
          String rfid_id = "";
          for (uint8_t i = 0; i < uidLength; i++) {
              rfid_id += String(uid[i], HEX);
          }
          Serial.println("READ RFID TAG");
          Serial.println(rfid_id);
          sendCheckInPacket("NFC", rfid_id);
      }
      //else{
       // Serial.println("NO RFID DETECTED");
     // }
      last_rfid_poll = millis();
  }
  // -------------------------
#endif  // WITH_RFID_CHECKIN



  if (!the_mesh.hasPendingWork()) {
#if defined(NRF52_PLATFORM)
#ifndef WITH_RFID_CHECKIN
    // The RFID build keeps the CPU awake to poll its reader; everything else
    // sleeps between events - the difference between a day of battery and an
    // afternoon on a tracker.
    board.sleep(0); // nrf ignores seconds param, sleeps whenever possible
#endif
#endif
  }

#if defined(ESP32) && defined(WIFI_SSID)
  // Safely attempt to reconnect every 10 seconds if flagged
  if (wifi_needs_reconnect && (millis() - last_wifi_reconnect_attempt > 10000)) {
    WIFI_DEBUG_PRINTLN("Attempting manual WiFi reconnect...");
    WiFi.disconnect();
    WiFi.reconnect();
    last_wifi_reconnect_attempt = millis();
  }
#endif
}
