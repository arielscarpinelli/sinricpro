#ifdef DEBUG_ESP_PORT
    #define DEBUG_MSG(...) DEBUG_ESP_PORT.println( __VA_ARGS__ )
    #define DEBUG_WRITE(...) DEBUG_ESP_PORT.write( __VA_ARGS__ )
    #define DEBUG_MSG_(...) DEBUG_ESP_PORT.print( __VA_ARGS__ )
#else
    #define DEBUG_MSG(...)
    #define DEBUG_WRITE(...)
    #define DEBUG_MSG_(...)
#endif


#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

#include <SinricPro.h>
#include <SinricProSwitch.h>

// Create yours based on config.h.tmpl
#include "config.h"

void ICACHE_RAM_ATTR toggle();

// We have a very simple state. Either our light is on or off
volatile bool state = false;
volatile bool buttonPressed = false;

void setup() {

#ifdef DEBUG_ESP_PORT
    DEBUG_ESP_PORT.begin(74880); // NodeMCU default
#endif    

    WiFi.mode(WIFI_STA);    
    WiFi.begin(WLAN_SSID, WLAN_PASS);

    pinMode(LED_BUILTIN, OUTPUT);
    // Depending on the relay type, you may need to use OUTPUT_OPEN_DRAIN
    pinMode(D1, RELAY_MODE);
    applyState();

    // We are using D3 as a gnd source for the push button. It is useful in D1 mini boards where
    // there is only one gnd output that you may use to feed the relay.
    pinMode(D3, OUTPUT);
    digitalWrite(D3, LOW);

    pinMode(D2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(D2), toggle, FALLING);
    
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.begin();

    SinricProSwitch& mySwitch = SinricPro[DEVICE_ID];
    mySwitch.onPowerState(onPowerState);

    SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); }); 
    SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
    SinricPro.begin(APP_KEY, APP_SECRET);
}

void loop() {
    if (buttonPressed) {
        buttonPressed = false;
        state = !state;
        applyState();
        publishState();
    }
    SinricPro.handle();
    ArduinoOTA.handle();
}

// This is a work around crappy pushbuttons
static long lastToggleMillis = 0;
void ICACHE_RAM_ATTR toggle() {
    long currentTimeMillis = millis();
    if (currentTimeMillis >= (lastToggleMillis + 2000)) {
        buttonPressed = true;
        lastToggleMillis = currentTimeMillis;
    }
}

void applyState() {
    Serial.printf(state ? "on" : "off");
    digitalWrite(LED_BUILTIN, state ? LOW : HIGH);
    #if RELAY_MODE == OUTPUT_OPEN_DRAIN
      digitalWrite(D1, state ? LOW : HIGH);
    #else
      digitalWrite(D1, state ? HIGH : LOW);
    #endif
}

void publishState() {
    // get Switch device back
    SinricProSwitch& mySwitch = SinricPro[DEVICE_ID];
    // send powerstate event
    mySwitch.sendPowerStateEvent(state); // send the new powerState to SinricPro server   
}

bool onPowerState(const String &deviceId, bool &updatedState) {
  state = updatedState;
  applyState();
  return true; // indicate that callback handled correctly
}
