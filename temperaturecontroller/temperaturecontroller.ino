#if defined(CORE_DEBUG_LEVEL) && (CORE_DEBUG_LEVEL > 0)
  #define DEBUG_ESP_PORT Serial
#endif

#ifdef DEBUG_ESP_PORT
  #define DEBUG_MSG(...) Serial.println( __VA_ARGS__ )
  #define DEBUG_WRITE(...) Serial.write( __VA_ARGS__ )
  #define DEBUG_MSG_(...) Serial.print( __VA_ARGS__ )
  //#define SIMULATE_TEMP 1
#else
  #define DEBUG_MSG(...)
  #define DEBUG_WRITE(...)
  #define DEBUG_MSG_(...)
#endif


#include <EEPROM.h>
#include <Arduino.h>

#include <OneWire.h>

#include <ArduinoJson.h>


#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>

  #define HEAT_SOURCE_PIN D1
  #define HEAT_DESTINATION_PIN D2
  
  #define RELAY_PUMP D5
  #define RELAY_VALVE D6
  
  #define SECONDARY_LED D4

  #define LED_OFF HIGH
  #define LED_ON LOW

  #define BUTTON 0

  ESP8266WebServer server(80);

#else
  #include <WiFi.h>
  #include <WebServer.h>

  #define HEAT_SOURCE_PIN 17
  #define HEAT_DESTINATION_PIN 16 // T2

  #define RELAY_PUMP T8
  #define RELAY_VALVE T9

  #define LED_BUILTIN 2

  #define LED_OFF HIGH
  #define LED_ON LOW
  
  // #define SECONDARY_LED 1

  WebServer server(80);
  
#endif
  


// Copy config.tmpl.h to config.h and replace with your own values
#include "config.h"

#include "TempSensor.h"
#include "ConnectionManager.h"
#include "OTA.h"


#define MIN_REPORTING_INTERVAL_MILLIS 10000 // 10 secs
#define FAILURE_REPORT_THRESHOLD 100

/************ Global State (you don't need to change this!) ******************/


TempSensor heatSource(HEAT_SOURCE_PIN); 
TempSensor heatSink(HEAT_DESTINATION_PIN);


void publishPersistentState();
ErrorReason handleUpdate(JsonObject& params);

class MyConnectionManager : public ConnectionManager {
  
  virtual void onConnect() {
    publishPersistentState();
  }

  virtual ErrorReason onUpdate(JsonObject& params) {
    return handleUpdate(params);
  }

};

MyConnectionManager conn;

typedef struct PersitentState {
  char initialized;
  long version;
  bool enabled;
  float desiredTemperature;
  float upperTemperatureThreshold;
  float lowerTemperatureThreshold;
  float maxSourceTemperature;
  float minSinkTemperature;
  int timezoneOffsetSeconds;
  int cleaningStartHour;
  int cleaningStartMinute;
  int cleaningDurationMinutes;
  int drainSeconds;
  int prestartSeconds;
  int stagnantTemperatureCheckIntervalMins;
  bool singleTemperatureMode;
  bool offline;
  int startHour;
  int startMinute;
  int stopHour;
  int stopMinute;
};

struct TransientState {
  float heatSourceTemperature;
  bool heatSourceFailure;
  unsigned long heatSourceFailureCount;
  float heatSinkTemperature;
  bool heatSinkFailure;
  unsigned long heatSinkFailureCount;
  bool pumpRunning;
  bool valveOpen;
  unsigned long drainStartedMillis;
  unsigned long lastStartedMillis;
  bool forcePump;
  bool forceValve;
  bool cleaning;
  unsigned long publishedMillis;
};

PersitentState persistentState;
TransientState transientState;
TransientState lastPublishedState;

void resetPersistentState() {
  persistentState.initialized = 42;
  persistentState.version = 1;
  persistentState.enabled = true;
  persistentState.desiredTemperature = 28;
  persistentState.maxSourceTemperature = 34;
  persistentState.lowerTemperatureThreshold = 3;
  persistentState.upperTemperatureThreshold = 8;
  persistentState.minSinkTemperature = 20;
  persistentState.timezoneOffsetSeconds = 0;
  persistentState.cleaningStartHour = 9;
  persistentState.cleaningStartMinute = 0;
  persistentState.cleaningDurationMinutes = 10;
  persistentState.drainSeconds = 15;
  persistentState.prestartSeconds = 45;
  persistentState.stagnantTemperatureCheckIntervalMins = 30;
  persistentState.offline = false;
  persistentState.singleTemperatureMode = false;
  persistentState.startHour = -1;
  persistentState.startMinute = 0;
  persistentState.stopHour = 22;
  persistentState.stopMinute = 0;
}

void resetTransientState(TransientState &state) {
  state.publishedMillis = 0;
  state.heatSourceTemperature = 0;
  state.heatSinkTemperature = 0;
  state.cleaning = false;
  state.pumpRunning = false;
  state.valveOpen = false;
  state.heatSourceFailure = false;
  state.heatSourceFailureCount = 0;
  state.heatSinkFailure = false;
  state.heatSinkFailureCount = 0;
  state.drainStartedMillis = 0;
  state.lastStartedMillis = 0;
}

class JsonStateSerializer : public StateSerializer {
  JsonObject &json;
public:
  JsonStateSerializer(JsonObject &json) : json(json) {};
  virtual void sendRangeValue(const String& instance, float value) {
    json[instance] = value;
  }
  virtual void sendPowerState(bool value) {
    json["enabled"] = value;
  }
};

void serializePersistentState(StateSerializer& ser) {
  ser.sendRangeValue("targetTemperature", persistentState.desiredTemperature);
  ser.sendPowerState(persistentState.enabled);
  ser.sendRangeValue("upperTemperatureThreshold", persistentState.upperTemperatureThreshold);
  ser.sendRangeValue("lowerTemperatureThreshold", persistentState.lowerTemperatureThreshold);
  ser.sendRangeValue("maxSourceTemperature", persistentState.maxSourceTemperature);
  ser.sendRangeValue("minSinkTemperature", persistentState.minSinkTemperature);
  ser.sendRangeValue("timezoneOffsetSeconds", persistentState.timezoneOffsetSeconds);
  ser.sendRangeValue("cleaningStartHour", persistentState.cleaningStartHour);
  ser.sendRangeValue("cleaningStartMinute", persistentState.cleaningStartMinute);
  ser.sendRangeValue("cleaningDurationMinutes", persistentState.cleaningDurationMinutes);
  ser.sendRangeValue("drainSeconds", persistentState.drainSeconds);
  ser.sendRangeValue("prestartSeconds", persistentState.prestartSeconds);
  ser.sendRangeValue("stagnantTemperatureCheckIntervalMins", persistentState.stagnantTemperatureCheckIntervalMins); 
  ser.sendRangeValue("singleTemperatureMode", persistentState.singleTemperatureMode); 
  ser.sendRangeValue("startHour", persistentState.startHour);
  ser.sendRangeValue("startHour", persistentState.startHour);
  ser.sendRangeValue("startMinute", persistentState.startMinute);
  ser.sendRangeValue("stopHour", persistentState.stopHour);
  ser.sendRangeValue("stopMinute", persistentState.stopMinute);
}

void persistentStateToJson(JsonObject &json) {
  auto ser = JsonStateSerializer(json);
  serializePersistentState(ser);
}

void transientStateToJson(JsonObject& json) {
  
  if(!persistentState.singleTemperatureMode) {
    json["heatSourceTemperature"] = transientState.heatSourceTemperature;
    if (transientState.heatSourceFailure) {
      json["heatSourceFailure"] = true;
    }
  }

  json["heatSinkTemperature"] = transientState.heatSinkTemperature;
  if (transientState.heatSinkFailure) {
    json["heatSinkFailure"] = true;
  }
  
  if (transientState.pumpRunning) {  
    json["pumpRunning"] = true;
  }

  if (transientState.valveOpen) {  
    json["valveOpen"] = true;
  }

  if (transientState.cleaning) {
    json["cleaning"] = transientState.cleaning;
  }

  if (transientState.forcePump) {
    json["forcePump"] = transientState.forcePump;
  }

  if (transientState.forceValve) {
    json["forceValve"] = transientState.forceValve;
  }

}

bool timeIsUp(unsigned long startedMillis, unsigned long durationMillis, unsigned long nowMillis) {
  return ((startedMillis + durationMillis) <= nowMillis) || 
      (nowMillis < startedMillis); // watches for overflow
}

void setup() {

#ifdef ESP8266
  Serial.begin(74880);
#else
  Serial.begin(115200);
#endif  

  delay(10);

  pinMode(RELAY_PUMP, RELAY_PIN_OUTPUT_MODE);
  digitalWrite(RELAY_PUMP, RELAY_STATE_OFF);
  pinMode(RELAY_VALVE, RELAY_PIN_OUTPUT_MODE);
  digitalWrite(RELAY_VALVE, RELAY_STATE_OFF);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_OFF);
  #ifdef SECONDARY_LED
    pinMode(SECONDARY_LED, OUTPUT); // NodeMCU secondary led
    digitalWrite(SECONDARY_LED, LED_OFF);
  #endif

  #ifdef BUTTON
    pinMode(BUTTON, INPUT_PULLUP);
  #endif  

  EEPROM.begin(sizeof(PersitentState));

  resetTransientState(lastPublishedState);
  resetTransientState(transientState);
  loadState();

  conn.setup(HOSTNAME, persistentState.timezoneOffsetSeconds, persistentState.offline);

  server.on("/", HTTP_GET, handleGetStatus);
  server.on("/", HTTP_POST, handlePostConfig);
  server.begin();

  #ifdef OTA_PASSWORD
    setupOTA(OTA_PASSWORD);
  #endif
}

void loadState() {
  EEPROM.get(0, persistentState);
  if(persistentState.initialized != 42) {
    DEBUG_MSG("persistentState not initialized");
    resetPersistentState();
    saveState();
  }
  
}

void saveState() {
  EEPROM.put(0, persistentState);
  EEPROM.commit();
}

void publishPersistentState() {
  // serializePersistentState(conn);
}

ErrorReason handleUpdate(JsonObject& params) {
  bool processed = false;

  if (params.containsKey("setPowerState")) {
    JsonVariant point = params["setPowerState"];
    persistentState.enabled = point == "On";
    processed = true;
  }

  if (params.containsKey("setThermostatMode")) {
    JsonVariant point = params["setThermostatMode"];
    if (point == "HEAT") {
      transientState.forcePump = true;
      persistentState.enabled = true;
    } else if (point == "OFF") {
      transientState.forcePump = false;
      persistentState.enabled = false;
    } else {
      transientState.forcePump = false;
      persistentState.enabled = true;
    }
    processed = true;
  }

  if (params.containsKey("targetTemperature")) {
    JsonVariant point = params["targetTemperature"];
    if (point.is<float>()) {
      persistentState.desiredTemperature = point;
      processed = true;
    } else {
      return "targetTemperature";
    }
  }

  if (params.containsKey("lowerTemperatureThreshold")) {
    JsonVariant point = params["lowerTemperatureThreshold"];
    if (point.is<float>()) {
      persistentState.lowerTemperatureThreshold = point;
      processed = true;
    } else {
      return "lowerTemperatureThreshold";
    }
  }

  if (params.containsKey("upperTemperatureThreshold")) {
    JsonVariant point = params["upperTemperatureThreshold"];
    if (point.is<float>()) {
      persistentState.upperTemperatureThreshold = point;
      processed = true;
    } else {
      return "upperTemperatureThreshold";
    }
  }

  if (params.containsKey("maxSourceTemperature")) {
    JsonVariant point = params["maxSourceTemperature"];
    if (point.is<float>()) {
      persistentState.maxSourceTemperature = point;
      processed = true;
    } else {
      return "maxSourceTemperature";
    }
  }

  if (params.containsKey("minSinkTemperature")) {
    JsonVariant point = params["minSinkTemperature"];
    if (point.is<float>()) {
      persistentState.minSinkTemperature = point;
      processed = true;
    } else {
      return "minSinkTemperature";
    }
  }

  if (params.containsKey("timezoneOffsetSeconds")) {
    JsonVariant point = params["timezoneOffsetSeconds"];
    if (point.is<int>()) {
      persistentState.timezoneOffsetSeconds = point;
      processed = true;
    } else {
      return "timezoneOffsetSeconds";
    }
  }

  if (params.containsKey("cleaningStartHour")) {
    JsonVariant point = params["cleaningStartHour"];
    if (point.is<int>()) {
      persistentState.cleaningStartHour = point;
      processed = true;
    } else {
      return "cleaningStartHour";
    }
  }

  if (params.containsKey("cleaningStartMinute")) {
    JsonVariant point = params["cleaningStartMinute"];
    if (point.is<int>()) {
      persistentState.cleaningStartMinute = point;
      processed = true;
    } else {
      return "cleaningStartMinute";
    }
  }

  if (params.containsKey("cleaningDurationMinutes")) {
    JsonVariant point = params["cleaningDurationMinutes"];
    if (point.is<int>()) {
      persistentState.cleaningDurationMinutes = point;
      processed = true;
    } else {
      return "cleaningDurationMinutes";
    }
  }

  if (params.containsKey("drainSeconds")) {
    JsonVariant point = params["drainSeconds"];
    if (point.is<int>()) {
      persistentState.drainSeconds = point;
      processed = true;
    } else {
      return "drainSeconds";
    }
  }

  if (params.containsKey("prestartSeconds")) {
    JsonVariant point = params["prestartSeconds"];
    if (point.is<int>()) {
      persistentState.prestartSeconds = point;
      processed = true;
    } else {
      return "prestartSeconds";
    }
  }

  if (params.containsKey("stagnantTemperatureCheckIntervalMins")) {
    JsonVariant point = params["stagnantTemperatureCheckIntervalMins"];
    if (point.is<int>()) {
      persistentState.stagnantTemperatureCheckIntervalMins = point;
      processed = true;
    } else {
      return "stagnantTemperatureCheckIntervalMins";
    }
  }

  if (params.containsKey("forcePump")) {
    JsonVariant point = params["forcePump"];
    if (point.is<boolean>()) {
      transientState.forcePump = point;
      processed = true;
    } else if (point.is<String>()) {
      transientState.forcePump = point == "On";
      processed = true;
    } else {
      return "forcePump";
    }
  }

  if (params.containsKey("forceValve")) {
    JsonVariant point = params["forceValve"];
    if (point.is<boolean>()) {
      transientState.forceValve = point;
      processed = true;
    } else if (point.is<String>()) {
      transientState.forceValve = point == "On";
      processed = true;
    } else {
      return "forceValve";
    }
  }

  if (params.containsKey("singleTemperatureMode")) {
    JsonVariant point = params["singleTemperatureMode"];
    if (point.is<bool>()) {
      persistentState.singleTemperatureMode = point;
      processed = true;
    } else {
      return "singleTemperatureMode";
    }
  }

  if (params.containsKey("startHour")) {
    JsonVariant point = params["startHour"];
    if (point.is<int>()) {
      persistentState.startHour = point;
      processed = true;
    } else {
      return "startHour";
    }
  }

  if (params.containsKey("startMinute")) {
    JsonVariant point = params["startMinute"];
    if (point.is<int>()) {
      persistentState.startMinute = point;
      processed = true;
    } else {
      return "startMinute";
    }
  }

  if (params.containsKey("stopHour")) {
    JsonVariant point = params["stopHour"];
    if (point.is<int>()) {
      persistentState.stopHour = point;
      processed = true;
    } else {
      return "stopHour";
    }
  }

  if (params.containsKey("stopMinute")) {
    JsonVariant point = params["stopMinute"];
    if (point.is<int>()) {
      persistentState.stopMinute = point;
      processed = true;
    } else {
      return "stopMinute";
    }
  }

  if (params.containsKey("ssid") && params.containsKey("pass")) {
    JsonVariant ssid = params["ssid"];
    JsonVariant pass = params["pass"];
    if (ssid.is<String>() && pass.is<String>()) {
      conn.config(ssid, pass);
      processed = true;
    } else {
      return "ssid";
    }
  }

  if (params.containsKey("offline")) {
    JsonVariant point = params["offline"];
    if (point.is<boolean>()) {
      persistentState.offline = point;
      conn.setOffline(point);
      processed = true;
    } else {
      return "offline";
    }
  }

  if (processed) {
    saveState();
  }

  return processed ? "" : "Unknown params";
}

void publishIfNeeded() {

  if (!conn.isConnected()) {
    return;
  }

  bool changedSink = roundAndCheckIfChanged(transientState.heatSinkTemperature, lastPublishedState.heatSinkTemperature);
  bool failedSink = (transientState.heatSinkFailure != lastPublishedState.heatSinkFailure);
  bool changedSource = !persistentState.singleTemperatureMode && (transientState.heatSourceTemperature, lastPublishedState.heatSourceTemperature);
  bool failedSource = !persistentState.singleTemperatureMode && (transientState.heatSourceFailure != lastPublishedState.heatSourceFailure);
  bool changedPumpState = (transientState.pumpRunning != lastPublishedState.pumpRunning);

  bool shouldForcePublish = failedSink || failedSource || changedPumpState;

  if (changedSink || changedSource || shouldForcePublish) {
    unsigned long now = millis();
    if (timeIsUp(lastPublishedState.publishedMillis, MIN_REPORTING_INTERVAL_MILLIS, now) || shouldForcePublish)  {
      if (changedSink) {
        // conn.sendRangeValue("heatSinkTemperature", transientState.heatSinkTemperature);
        conn.sendTemperature(transientState.heatSinkTemperature);
      }
      if (failedSink) {
        conn.notifyError("Unable to read temperature");
      }
      if (changedSource) {
        conn.sendRangeValue("heatSourceTemperature", transientState.heatSourceTemperature);
      }
      if (failedSource) {
        conn.notifyError("Unable to read source temperature");
      }
      if (changedPumpState) {
        // conn.sendModeValue("pumpRunning", transientState.pumpRunning);
        conn.sendToggleValue("pumpRunning", transientState.pumpRunning);
      }
      lastPublishedState = transientState;
      lastPublishedState.publishedMillis = now;
    }
  }
}    

bool roundAndCheckIfChanged(float &temp, float last) {
  
  // round to nearest 0.5
  temp = round(temp * 2) / 2;

  return fabs(last - temp) >= 0.1;

}

unsigned long now;

#ifdef BUTTON
unsigned long buttonStarted = 0;

void readButton() {
  if(digitalRead(BUTTON) == LOW) {
    if(!buttonStarted) {
      buttonStarted = now;
      DEBUG_MSG("button down");
    } else if ((buttonStarted + 3000) <= now) {
      DEBUG_MSG("button timeup");
      conn.resetWifiConfig();
      buttonStarted = 0;
    }
  } else {
    buttonStarted = 0;
  }
}      
#endif  

void loop() {

  now = millis();

  conn.loop(now);
  bool tempsUpdated = updateTemps();
  updatePump(tempsUpdated);
  updateValve();
  publishIfNeeded();
  server.handleClient();
  loopOTA();
#ifdef SIMULATE_TEMP
  readAndSimulateTemp();
#endif
#ifdef BUTTON
  readButton();
#endif  
}

bool updateTemps() {
  
  bool tempsUpdated = false;

  if (!persistentState.singleTemperatureMode) {
    int tempRead = heatSource.read(now); 
    if (tempRead != TEMP_SENSOR_CONTINUE) {
      tempsUpdated = true;
      if(tempRead == TEMP_SENSOR_OK) {
        transientState.heatSourceTemperature = heatSource.value;
        transientState.heatSourceFailure = false;
      } else if (!transientState.heatSourceFailure) {
        transientState.heatSourceFailureCount++;
        if (transientState.heatSourceFailureCount > FAILURE_REPORT_THRESHOLD) {
          transientState.heatSourceFailure = true;
        }
      }
    }
  }
  
  int tempRead = heatSink.read(now);
  if (tempRead != TEMP_SENSOR_CONTINUE) {
    tempsUpdated = true;
    if(tempRead == TEMP_SENSOR_OK) {
      transientState.heatSinkTemperature = heatSink.value;
      transientState.heatSinkFailure = false;
    } else if (!transientState.heatSinkFailure) {
      transientState.heatSinkFailureCount++;
      if (transientState.heatSinkFailureCount > FAILURE_REPORT_THRESHOLD) {
        transientState.heatSinkFailure = true;
      }
    }
  }

  return tempsUpdated;
}

bool applyCleaning() {

  if (persistentState.cleaningDurationMinutes > 0) {

    // get current date
    time_t currentTime = time(nullptr);
    
    // validate we actually managed to get NTP synced already
    if (currentTime > 1704067200) { // 2024-01-01

      struct tm *info = localtime(&currentTime);
    
      info->tm_hour = persistentState.cleaningStartHour;
      info->tm_min = persistentState.cleaningStartMinute;
      info->tm_sec = 0;
    
      time_t cleaningStart = mktime(info);
      time_t cleaningStop = cleaningStart + persistentState.cleaningDurationMinutes * 60;
      
      if ((cleaningStart <= now) && (now <= cleaningStop)) {
        transientState.cleaning = true;
        pumpSet(true);
        return true;
      }  

      transientState.cleaning = false;

    }
  }
  return false;
  
}

void updatePump(bool tempsUpdated) {

  if(transientState.forcePump) {
    pumpSet(true);
    DEBUG_MSG("forcePump");
    return;
  }

  if(applyCleaning()) {
    DEBUG_MSG("cleaning");
    return;
  }

  if (!persistentState.enabled) {
    pumpSet(false);
    DEBUG_MSG("disabled");
    return;
  }

  if(!tempsUpdated) {
    return;
  }

  if (transientState.heatSinkTemperature < persistentState.minSinkTemperature) {
    pumpSet(false);
    DEBUG_MSG("under min temperature");
    return;
  }

  if (!checkStartStopTime()) {
    pumpSet(false);
    DEBUG_MSG("out of working hours");
    return;
  }

  float desiredTemperatureDiff = persistentState.desiredTemperature - transientState.heatSinkTemperature;

  bool pumpStartCondition;
  bool pumpStopCondition;
  bool enoughHeatSource;
  DEBUG_MSG_("c ");
  DEBUG_MSG_(transientState.heatSinkTemperature);

  DEBUG_MSG_(" d ");
  DEBUG_MSG_(desiredTemperatureDiff);

  if (!persistentState.singleTemperatureMode) {
    float heatTemperatureDiff = transientState.heatSourceTemperature - transientState.heatSinkTemperature;
    bool heatTempDiffOverLowerThreshold = (heatTemperatureDiff >= persistentState.lowerTemperatureThreshold);

    pumpStartCondition = (desiredTemperatureDiff > 0);
    pumpStopCondition = !heatTempDiffOverLowerThreshold;
    enoughHeatSource = (heatTemperatureDiff >= persistentState.upperTemperatureThreshold) || 
          ((transientState.heatSourceTemperature >= persistentState.maxSourceTemperature) && heatTempDiffOverLowerThreshold);

    DEBUG_MSG_(" s ");
    DEBUG_MSG_(transientState.heatSourceTemperature);

    DEBUG_MSG_(" h ");
    DEBUG_MSG(heatTemperatureDiff);

  } else {
    pumpStartCondition = desiredTemperatureDiff >= persistentState.lowerTemperatureThreshold;
    pumpStopCondition = (desiredTemperatureDiff <= 0) && ((-desiredTemperatureDiff) >= persistentState.upperTemperatureThreshold);
    enoughHeatSource = true;
    DEBUG_MSG("");
  }

  if (!transientState.pumpRunning) {
    if(pumpStartCondition) {
      if (enoughHeatSource) {
        pumpSet(true);
      }
    } else if (persistentState.stagnantTemperatureCheckIntervalMins > 0 
        && (enoughHeatSource)
        && timeIsUp(transientState.lastStartedMillis, persistentState.stagnantTemperatureCheckIntervalMins * 60000, now)) {
      pumpSet(true);
    }
  } else {
    if (pumpStopCondition) {
      pumpSet(false);
    }  
  }
}

bool checkStartStopTime() {

  if (persistentState.startHour < 0) {
    return true;
  }

  // get current date
  time_t currentTime = time(nullptr);

  // validate we actually managed to get NTP synced already
  if (currentTime > 1704067200) { // 2024-01-01

    struct tm *info = localtime(&currentTime);

    info->tm_hour = persistentState.startHour;
    info->tm_min = persistentState.startMinute;
    info->tm_sec = 0;

    time_t start = mktime(info);
  
    info->tm_hour = persistentState.stopHour;
    info->tm_min = persistentState.stopMinute;
    info->tm_sec = 0;

    time_t stop = mktime(info);

    return (start <= currentTime) && (currentTime <= stop);
  }
  return false;
}

inline int relayState(bool v) {
  return v ? RELAY_STATE_ON : RELAY_STATE_OFF;
}

void pumpSet(bool v) {

  if(transientState.pumpRunning != v) {
    if (v) {
      transientState.lastStartedMillis = now;
      transientState.drainStartedMillis = 0;
    } else {
      if (!transientState.drainStartedMillis) {
        DEBUG_MSG("Starting draining");
        transientState.drainStartedMillis = now;
      }
      if (!timeIsUp(transientState.drainStartedMillis, persistentState.drainSeconds * 1000, now)){
        return;
      }
      transientState.drainStartedMillis = 0;
      DEBUG_MSG("Draining ended");
    }
    int pump = relayState(v);

    DEBUG_MSG_("Setting pump to: ");
    DEBUG_MSG(v);
    digitalWrite(RELAY_PUMP, pump);
    #ifdef SECONDARY_LED
      digitalWrite(SECONDARY_LED, v ? LED_ON : LED_OFF);
    #endif  
    transientState.pumpRunning = v;
  }
  
}

void updateValve() {

  bool prestartFinished = false;

  if(transientState.pumpRunning) {
    if (persistentState.prestartSeconds > 0) {
      prestartFinished = timeIsUp(transientState.lastStartedMillis, (persistentState.prestartSeconds * 1000), now);
    } else {
      prestartFinished = true;
    };
  }


  int valve = transientState.forceValve ||
      (transientState.pumpRunning && prestartFinished);

  if (valve != transientState.valveOpen) {
    DEBUG_MSG_("Setting valve to: ");
    DEBUG_MSG(valve);
    digitalWrite(RELAY_VALVE, relayState(valve));
    transientState.valveOpen = valve;
  }
}

void handleGetStatus() {
  
  const size_t capacity = JSON_OBJECT_SIZE(80);
  DynamicJsonDocument doc(capacity);

  #ifdef ESP8266
    uint32_t chipId = ESP.getChipId();
  #else
    uint32_t chipId = 0;    
    for(int i=0; i<17; i=i+8) {
      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
  #endif
  
  doc["mac"] = String(chipId, HEX);
  doc["deviceId"] = DEVICE_ID;
  doc["ip"] = WiFi.localIP().toString();
  doc["accessPoint"] = WiFi.SSID();
  doc["currentTime"] = time(nullptr);
  doc["uptime"] = now;
  doc["wsConnected"] = conn.isConnected();
  doc["connectionUpdatedAt"] = conn.getConnectionUpdatedAt();
  doc["configVersion"] = persistentState.version;

  JsonObject obj = doc.as<JsonObject>();
  persistentStateToJson(obj);
  transientStateToJson(obj);

  String json;
  serializeJson(doc, json);  
  
  server.send(200, "application/json", json);
}

void handlePostConfig() {

  const size_t capacity = JSON_OBJECT_SIZE(40);
  DynamicJsonDocument doc(capacity);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, server.arg("plain"));

  // Test if parsing succeeds.
  if (error) {
    DEBUG_MSG_(F("deserializeJson() failed: "));
    DEBUG_MSG(error.c_str());
    badRequest(error.c_str());
    return;
  }

  JsonObject json = doc.as<JsonObject>();
  ErrorReason updateError = handleUpdate(json);
  if (!updateError.length()) {
    return handleGetStatus();
  } else {
    badRequest(updateError);
  }    
}

void badRequest(String msg) {
  const size_t capacity = JSON_OBJECT_SIZE(20);
  DynamicJsonDocument doc(capacity);

  doc["error"] = 400;
  doc["reason"] = ("Failed to parse " + msg);
  String json;
  serializeJson(doc, json);  
  
  server.send(400, "application/json", json);  
}

#ifdef SIMULATE_TEMP
void readAndSimulateTemp() {
  if(DEBUG_ESP_PORT.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 's':
        heatSource.value = Serial.parseFloat();
        heatSource.mock = true;
        break;
      case 'd':
        heatSink.value = Serial.parseFloat();
        heatSink.mock = true;
        break;
      case 'e':
        persistentState.enabled = !persistentState.enabled;
        break;
      case 'p':
        persistentState.desiredTemperature = Serial.parseFloat();
        break;
      default:
        DEBUG_MSG("invalid command");

    }
    
  }
}
#endif
