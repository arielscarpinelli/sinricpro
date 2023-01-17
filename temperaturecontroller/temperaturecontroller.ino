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
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

#include <ESP8266mDNS.h>

// Copy config.tmpl.h to config.h and replace with your own values
#include "config.h"

#include "TempSensor.h"
#include "ConnectionManager.h"
#include "OTA.h"

#define HEAT_SOURCE_PIN D1
#define HEAT_DESTINATION_PIN D2

#define RELAY_PUMP D5
#define RELAY_VALVE D6

#define SECONDARY_LED D4

#define RELAY_STATE_ON  LOW
#define RELAY_STATE_OFF HIGH

#define DEFAULT_MIN_REPORTING_INTERVAL_MILLIS 30000 // 30 secs
#define FAILURE_REPORT_THRESHOLD 10

/************ Global State (you don't need to change this!) ******************/

ESP8266WebServer server(80);

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
  bool enabled;
  float desiredTemperature;
  float upperTemperatureThreshold;
  float lowerTemperatureThreshold;
  float maxTemperature;
  float minTemperature;
  int timezoneOffsetSeconds;
  int cleaningStartHour;
  int cleaningStartMinute;
  int cleaningDurationMinutes;
  int drainSeconds;
  int prestartSeconds;
  int stagnantTemperatureCheckIntervalMins;
  int prestartCycles;
};

struct TransientState {
  float heatSourceTemperature;
  bool heatSourceFailure;
  unsigned long heatSourceFailureCount;
  float heatSinkTemperature;
  bool heatSinkFailure;
  unsigned long heatSinkFailureCount;
  float heatSourceAverageTemperatureSum;
  unsigned int heatSourceAverageTemperatureCount;
  float heatSourceMaxTemperature;
  bool pumpRunning;
  bool valveOpen;
  unsigned long drainStartedMillis;
  unsigned long lastStartedMillis;
  unsigned int prestartCurrentCycle;
  bool forcePump;
  bool forceValve;
  bool cleaning;
  unsigned long minReportingIntervalMillis;
  unsigned long publishedMillis;
};

PersitentState persistentState;
TransientState transientState;
TransientState lastPublishedState;

void resetPersistentState() {
  persistentState.initialized = 42;
  persistentState.enabled = true;
  persistentState.desiredTemperature = 28;
  persistentState.maxTemperature = 34;
  persistentState.lowerTemperatureThreshold = 3;
  persistentState.upperTemperatureThreshold = 8;
  persistentState.minTemperature = 20;
  persistentState.timezoneOffsetSeconds = 0;
  persistentState.cleaningStartHour = 9;
  persistentState.cleaningStartMinute = 0;
  persistentState.cleaningDurationMinutes = 10;
  persistentState.drainSeconds = 15;
  persistentState.prestartSeconds = 45;
  persistentState.prestartCycles = 2;
  persistentState.stagnantTemperatureCheckIntervalMins = 30;
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
  state.prestartCurrentCycle = 0;
  state.minReportingIntervalMillis = DEFAULT_MIN_REPORTING_INTERVAL_MILLIS;
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
  ser.sendRangeValue("maxTemperature", persistentState.maxTemperature);
  ser.sendRangeValue("minTemperature", persistentState.minTemperature);
  ser.sendRangeValue("timezoneOffsetSeconds", persistentState.timezoneOffsetSeconds);
  ser.sendRangeValue("cleaningStartHour", persistentState.cleaningStartHour);
  ser.sendRangeValue("cleaningStartMinute", persistentState.cleaningStartMinute);
  ser.sendRangeValue("cleaningDurationMinutes", persistentState.cleaningDurationMinutes);
  ser.sendRangeValue("drainSeconds", persistentState.drainSeconds);
  ser.sendRangeValue("prestartSeconds", persistentState.prestartSeconds);
  ser.sendRangeValue("prestartCycles", persistentState.prestartCycles);
  ser.sendRangeValue("stagnantTemperatureCheckIntervalMins", persistentState.stagnantTemperatureCheckIntervalMins); 
}

void persistentStateToJson(JsonObject &json) {
  auto ser = JsonStateSerializer(json);
  serializePersistentState(ser);
}

void transientStateToJson(JsonObject& json) {
  
  json["heatSourceTemperature"] = transientState.heatSourceTemperature;
  json["heatSourceAverageTemperature"] = transientState.heatSourceAverageTemperatureSum /  transientState.heatSourceAverageTemperatureCount;
  json["heatSourceMaxTemperature"] = transientState.heatSourceMaxTemperature;
  if (transientState.heatSourceFailure) {
    json["heatSourceFailure"] = true;
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

  if (transientState.prestartCurrentCycle) {
    json["prestartCurrentCycle"] = transientState.prestartCurrentCycle;
  }

  json["minReportingIntervalMillis"] = transientState.minReportingIntervalMillis;

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

  Serial.begin(74880);
  delay(10);

  pinMode(RELAY_PUMP, RELAY_PIN_OUTPUT_MODE);
  digitalWrite(RELAY_PUMP, RELAY_STATE_OFF);
  pinMode(RELAY_VALVE, RELAY_PIN_OUTPUT_MODE);
  digitalWrite(RELAY_VALVE, RELAY_STATE_OFF);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(SECONDARY_LED, OUTPUT); // NodeMCU secondary led
  digitalWrite(SECONDARY_LED, HIGH);

  EEPROM.begin(sizeof(PersitentState));

  resetTransientState(lastPublishedState);
  resetTransientState(transientState);
  loadState();

  conn.setup(HOSTNAME, persistentState.timezoneOffsetSeconds);

  server.on("/", HTTP_GET, handleGetStatus);
  server.on("/", HTTP_POST, handlePostConfig);
  server.begin();

  setupOTA(OTA_PASSWORD);
}

void loadState() {
  EEPROM.get(0, persistentState);
  if(persistentState.initialized != 42) {
    DEBUG_MSG("persistentState not initialized");
    resetPersistentState();
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

  if (params.containsKey("maxTemperature")) {
    JsonVariant point = params["maxTemperature"];
    if (point.is<float>()) {
      persistentState.maxTemperature = point;
      processed = true;
    } else {
      return "maxTemperature";
    }
  }

  if (params.containsKey("minTemperature")) {
    JsonVariant point = params["minTemperature"];
    if (point.is<float>()) {
      persistentState.minTemperature = point;
      processed = true;
    } else {
      return "minTemperature";
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

  if (params.containsKey("prestartCycles")) {
    JsonVariant point = params["prestartCycles"];
    if (point.is<int>()) {
      persistentState.prestartCycles = point;
      processed = true;
    } else {
      return "prestartCycles";
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

  if (params.containsKey("minReportingIntervalMillis")) {
    JsonVariant point = params["minReportingIntervalMillis"];
    if (point.is<int>()) {
      transientState.minReportingIntervalMillis = point;
      processed = true;
    } else {
      return "minReportingIntervalMillis";
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
  bool changedSource = roundAndCheckIfChanged(transientState.heatSourceTemperature, lastPublishedState.heatSourceTemperature);
  bool failedSource = (transientState.heatSourceFailure != lastPublishedState.heatSourceFailure);
  bool changedPumpState = (transientState.pumpRunning != lastPublishedState.pumpRunning);

  bool shouldForcePublish = failedSink || failedSource || changedPumpState;

  if (changedSink || changedSource || shouldForcePublish) {
    unsigned long now = millis();
    if (timeIsUp(lastPublishedState.publishedMillis, transientState.minReportingIntervalMillis, now) || shouldForcePublish)  {
      if (changedSink) {
        conn.sendRangeValue("heatSinkTemperature", transientState.heatSinkTemperature);
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
        conn.sendModeValue("pumpRunning", transientState.pumpRunning);
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

void loop() {

  now = millis();

  conn.loop(now);
  updateTemps();
  updatePump();
  updateValve();
  publishIfNeeded();
  server.handleClient();
  loopOTA();
#ifdef SIMULATE_TEMP
  readAndSimulateTemp();
#endif
}

unsigned long lastTempRead = 0;

void updateTemps() {

  // Give more CPU time to comms when trying to connect.
  if (!conn.isConnected()) {
    if (!timeIsUp(lastTempRead, 5000, now)) {
      return;
    }
    lastTempRead = now;
  }
  
  //DEBUG_MSG_("Heat source");
  if(heatSource.read()) {
    transientState.heatSourceTemperature = heatSource.value;
    transientState.heatSourceFailure = false;
  } else if (!transientState.heatSourceFailure) {
    transientState.heatSourceFailureCount++;
    if (transientState.heatSourceFailureCount > FAILURE_REPORT_THRESHOLD) {
      transientState.heatSourceFailure = true;
    }
  }

  //DEBUG_MSG_("Heat sink");
  if(heatSink.read()) {
    transientState.heatSinkTemperature = heatSink.value;
    transientState.heatSinkFailure = false;
  } else if (!transientState.heatSinkFailure) {
    transientState.heatSinkFailureCount++;
    if (transientState.heatSinkFailureCount > FAILURE_REPORT_THRESHOLD) {
      transientState.heatSinkFailure = true;
    }
  }

}

bool applyCleaning() {

  if (persistentState.cleaningDurationMinutes > 0) {

    // get current date
    time_t now = time(nullptr);
    struct tm *info = localtime(&now);
  
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
  return false;
  
}

void updatePump() {

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

  if (transientState.heatSinkTemperature < persistentState.minTemperature) {
    pumpSet(false);
    DEBUG_MSG("under min temperature");
    return;
  }

  float desiredTemperatureDiff = persistentState.desiredTemperature - transientState.heatSinkTemperature;

  bool desiredTempDiff = (desiredTemperatureDiff > 0);

  float heatTemperatureDiff = transientState.heatSourceTemperature - transientState.heatSinkTemperature;
  bool heatTempDiffOverLowerThreshold = (heatTemperatureDiff >= persistentState.lowerTemperatureThreshold);

  DEBUG_MSG_("d ");
  DEBUG_MSG_(desiredTemperatureDiff);

  DEBUG_MSG_(" h ");
  DEBUG_MSG(heatTemperatureDiff);

  if (!transientState.pumpRunning) {
    if(desiredTempDiff) {
      if ((heatTemperatureDiff >= persistentState.upperTemperatureThreshold) || 
          ((transientState.heatSourceTemperature >= persistentState.maxTemperature) && heatTempDiffOverLowerThreshold)) {
        pumpSet(true);
      }
    } else if (persistentState.stagnantTemperatureCheckIntervalMins > 0 && timeIsUp(transientState.lastStartedMillis, persistentState.stagnantTemperatureCheckIntervalMins * 60000, now)) {
      pumpSet(true);
    }
  } else {
    if (!heatTempDiffOverLowerThreshold) {
      pumpSet(false);
    } else {
      transientState.heatSourceAverageTemperatureCount++;
      transientState.heatSourceAverageTemperatureSum += transientState.heatSourceTemperature;
      if (transientState.heatSourceTemperature > transientState.heatSourceMaxTemperature) {
        transientState.heatSourceMaxTemperature = transientState.heatSourceTemperature;
      }
    }
  }
  
}

inline int relayState(bool v) {
  return v ? RELAY_STATE_ON : RELAY_STATE_OFF;
}

void pumpSet(bool v) {

  if(transientState.pumpRunning != v) {
    if (v) {
      transientState.lastStartedMillis = now;
      transientState.drainStartedMillis = 0;
      transientState.heatSourceAverageTemperatureCount = 1;
      transientState.heatSourceAverageTemperatureSum = transientState.heatSourceTemperature;
      transientState.heatSourceMaxTemperature = transientState.heatSourceTemperature;
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
    digitalWrite(SECONDARY_LED, pump);
    transientState.pumpRunning = v;
  }
  
}

void updateValve() {

  bool prestartFinishedOrOpenValveCycle = false;

  if(!transientState.pumpRunning) {
    transientState.prestartCurrentCycle = 0;
  } else {
    int totalCycles = persistentState.prestartCycles * 2 - 1;

    if (transientState.prestartCurrentCycle < totalCycles) {
      bool currentCycleFinished = timeIsUp(transientState.lastStartedMillis, (persistentState.prestartSeconds * 1000) * (transientState.prestartCurrentCycle + 1), now);

      if (currentCycleFinished) {
        transientState.prestartCurrentCycle++;
      }

      prestartFinishedOrOpenValveCycle = ((transientState.prestartCurrentCycle % 2) == 1);

    } else {
      prestartFinishedOrOpenValveCycle = true;
    }

  }


  int valve = transientState.forceValve ||
      (transientState.pumpRunning && !transientState.forcePump && prestartFinishedOrOpenValveCycle);

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

  doc["id"] = String(ESP.getChipId(), HEX);
  doc["ip"] = WiFi.localIP().toString();
  doc["accessPoint"] = WiFi.SSID();
  doc["currentTime"] = time(nullptr);
  doc["wsConnected"] = conn.isConnected();
  doc["connectionUpdatedAt"] = conn.getConnectionUpdatedAt();

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
