#ifdef DEBUG_ESP_PORT
  #define DEBUG_MSG(...) Serial.println( __VA_ARGS__ )
  #define DEBUG_WRITE(...) Serial.write( __VA_ARGS__ )
  #define DEBUG_MSG_(...) Serial.print( __VA_ARGS__ )
#else
  #define DEBUG_MSG(...)
  #define DEBUG_WRITE(...)
  #define DEBUG_MSG_(...)
#endif

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>

#include <SinricPro.h>
#include <SinricProSwitch.h>

// Copy config.tmpl.h to config.h and replace with your own values
#include "config.h"

#define SECONDARY_LED D4
#define OUTPUT_PIN D1
#define INPUT_PIN D2

// Poorman's state pattern
void toggle();
void reverseToggle();
void noop() {};

typedef struct DoorState {
  String name;
  DoorState* motorStarted;
  DoorState* motorStopped;
  void (*receivedRequestToOpen)(void);
  void (*receivedRequestToClose)(void);
  bool valueToNotify;
} DoorState;

DoorState DOOR_CLOSING = {
  .name = "CLOSING",
  .motorStarted = &DOOR_CLOSING,
  .motorStopped = &DOOR_CLOSING, // redefined below
  .receivedRequestToOpen = reverseToggle,
  .receivedRequestToClose = noop,
  .valueToNotify = true
};

DoorState DOOR_WILL_CLOSE_ON_TOGGLE = {
  .name = "WILL_CLOSE_ON_TOGGLE",
  .motorStarted = &DOOR_CLOSING,
  .motorStopped = &DOOR_WILL_CLOSE_ON_TOGGLE,
  .receivedRequestToOpen = noop,
  .receivedRequestToClose = toggle,
  .valueToNotify = true
};

DoorState DOOR_OPENING = {
  .name = "DOOR_OPENING",
  .motorStarted = &DOOR_OPENING,
  .motorStopped = &DOOR_WILL_CLOSE_ON_TOGGLE,
  .receivedRequestToOpen = noop,
  .receivedRequestToClose = reverseToggle,
  .valueToNotify = true
};

DoorState DOOR_WILL_OPEN_ON_TOGGLE = {
  .name = "DOOR_WILL_OPEN_ON_TOGGLE",
  .motorStarted = &DOOR_OPENING,
  .motorStopped = &DOOR_WILL_OPEN_ON_TOGGLE,
  .receivedRequestToOpen = toggle,
  .receivedRequestToClose = noop,
  .valueToNotify = false
};

DoorState* currentState = &DOOR_WILL_OPEN_ON_TOGGLE;

void publishState();

bool connected = false;
bool justConnected = false;
bool connectingLed = false;

bool lastNotifiedValue = false;

void ICACHE_RAM_ATTR motorChanged();

ESP8266WebServer server(80);

void setup() {

  DOOR_CLOSING.motorStopped = &DOOR_WILL_OPEN_ON_TOGGLE;

  #ifdef DEBUG_ESP_PORT
      DEBUG_ESP_PORT.begin(74880); // NodeMCU default
  #endif    

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(SECONDARY_LED, OUTPUT); // NodeMCU secondary led
  digitalWrite(SECONDARY_LED, HIGH);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(INPUT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), motorChanged, CHANGE);

  WiFi.mode(WIFI_STA);    
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.begin();

  SinricProSwitch& mySwitch = SinricPro[DEVICE_ID];
  mySwitch.onPowerState(onPowerState);

  SinricPro.onConnected([](){ 
    Serial.printf("Connected to SinricPro\r\n"); 
    connected = true;
    justConnected = true;
    publishState();   
  }); 
  SinricPro.onDisconnected([](){ 
    Serial.printf("Disconnected from SinricPro\r\n"); 
    connected = false;
  });
  SinricPro.begin(APP_KEY, APP_SECRET);

  server.on("/", HTTP_GET, handleHttp);
  server.begin();

}

void loop() {

  updateConnectingLed();

  if (currentState->valueToNotify != lastNotifiedValue) {
    publishState();
    Serial.println(currentState->name);
  }

  SinricPro.handle();
  ArduinoOTA.handle();
  server.handleClient();
}

void toggle() {
  DEBUG_MSG("toggle");
  digitalWrite(LED_BUILTIN, LOW);  
  digitalWrite(OUTPUT_PIN, HIGH);
  delay(500);
  digitalWrite(OUTPUT_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);  
}

void reverseToggle() {
  // first stop
  toggle();
  delay(500);
  // I think that the interrupt should have changed the value of the current state in the middle, but shouldn't stop us...
  // then restart
  toggle();
}

void ICACHE_RAM_ATTR motorChanged() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) {
    bool value = digitalRead(INPUT_PIN);
    digitalWrite(SECONDARY_LED, value);
    if (value == LOW) { // input is inverted
      currentState = currentState->motorStarted;
    } else {
      currentState = currentState->motorStopped;
    }
  }
  last_interrupt_time = interrupt_time;
  
}

void updateConnectingLed() {

  static unsigned long lastLoopMillis = 0; 
  
  if (!connected) {
    unsigned long currentTimeMillis = millis();
    if (currentTimeMillis > (lastLoopMillis + 500)) {
      lastLoopMillis = currentTimeMillis;
      connectingLed = !connectingLed;
      digitalWrite(LED_BUILTIN, connectingLed ? LOW : HIGH);
    }
  }

  if (justConnected) {
    connectingLed = false;
    justConnected = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }
      
}


bool onPowerState(const String &deviceId, bool &updatedState) {
  if(updatedState) {
    currentState->receivedRequestToOpen();
  } else {
    currentState->receivedRequestToClose();    
  }
  return true; // indicate that callback handled correctly
}

void publishState() {
  SinricProSwitch& mySwitch = SinricPro[DEVICE_ID];
  mySwitch.sendPowerStateEvent(currentState->valueToNotify);
  lastNotifiedValue = currentState->valueToNotify;
}

void handleHttp() {
  if (!server.hasArg("pass")) {
    server.send(401, "text/plain", "Requires pass");
    return;
  }
  if (!server.arg("pass").equals(PASSCODE)) {
    server.send(401, "text/plain", "Invalid pass");
    return;
  }
  toggle();
  server.send(200, "text/plain", "ok");
}
