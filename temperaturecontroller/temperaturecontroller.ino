#ifdef DEBUG_ESP_PORT
  #define DEBUG_MSG(...) Serial.println( __VA_ARGS__ )
  #define DEBUG_WRITE(...) Serial.write( __VA_ARGS__ )
  #define DEBUG_MSG_(...) Serial.print( __VA_ARGS__ )
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

#define RELAY_1 D5
#define RELAY_2 D6

#define SECONDARY_LED D4

#define RELAY_STATE_ON  LOW
#define RELAY_STATE_OFF HIGH

#define DEFAULT_MIN_REPORTING_INTERVAL_MILLIS 30000 // 30 secs

void onMqttMessage(char* topic, byte* payload, unsigned int length);
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
};

struct TransientState {
  float heatSourceTemperature;
  bool heatSourceFailure;
  float heatSinkTemperature;
  bool heatSinkFailure;
  float heatSourceAverageTemperatureSum;
  unsigned int heatSourceAverageTemperatureCount;
  float heatSourceMaxTemperature;
  bool pumpRunning;
  unsigned long drainStartedMillis;
  bool forcePump;
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
  persistentState.cleaningStartHour = 12;
  persistentState.cleaningStartMinute = 0;
  persistentState.cleaningDurationMinutes = 10;
  persistentState.drainSeconds = 15;
}

void resetTransientState(TransientState &state) {
  state.publishedMillis = 0;
  state.heatSourceTemperature = 0;
  state.heatSinkTemperature = 0;
  state.cleaning = false;
  state.pumpRunning = false;
  state.heatSourceFailure = false;
  state.heatSinkFailure = false;
  state.drainStartedMillis = 0;
  state.minReportingIntervalMillis = DEFAULT_MIN_REPORTING_INTERVAL_MILLIS;
}

void persistentStateToJson(JsonObject &json) {
  json["thermostatMode"] = persistentState.enabled ? "heat" : "off";
  json["thermostatTemperatureSetpoint"] = persistentState.desiredTemperature;
  json["upperTemperatureThreshold"] = persistentState.upperTemperatureThreshold;
  json["lowerTemperatureThreshold"] = persistentState.lowerTemperatureThreshold;
  json["maxTemperature"] = persistentState.maxTemperature;
  json["minTemperature"] = persistentState.minTemperature;
  json["timezoneOffsetSeconds"] = persistentState.timezoneOffsetSeconds;
  json["cleaningStartHour"] = persistentState.cleaningStartHour;
  json["cleaningStartMinute"] = persistentState.cleaningStartMinute;
  json["cleaningDurationMinutes"] = persistentState.cleaningDurationMinutes;
  json["drainSeconds"] = persistentState.drainSeconds;

  // Albeit this is transient, it is a config, not a read value
  json["minReportingIntervalMillis"] = transientState.minReportingIntervalMillis;

  if (transientState.forcePump) {
    json["forcePump"] = transientState.forcePump;
  }

}

void transientStateToJson(JsonObject& json) {
  
  json["heatSourceTemperature"] = transientState.heatSourceTemperature;
  if (transientState.heatSourceFailure) {
    json["heatSourceFailure"] = true;
  }

  json["thermostatTemperatureAmbient"] = transientState.heatSinkTemperature;
  if (transientState.heatSinkFailure) {
    json["heatSinkFailure"] = true;
  }
  
  if (transientState.pumpRunning) {  
    json["pumpRunning"] = true;
  }

  bool draining = (transientState.drainStartedMillis > 0);
  if (draining) {
    json["draining"] = draining;
    json["heatSourceAverageTemperature"] = transientState.heatSourceAverageTemperatureSum /  transientState.heatSourceAverageTemperatureCount;
    json["heatSourceMaxTemperature"] = transientState.heatSourceMaxTemperature;
  }

  if (transientState.cleaning) {
    json["cleaning"] = transientState.cleaning;
  }
  
}


void setup() {

  Serial.begin(74880);
  delay(10);

  pinMode(RELAY_1, RELAY_PIN_OUTPUT_MODE);
  digitalWrite(RELAY_1, RELAY_STATE_OFF);
  pinMode(RELAY_2, RELAY_PIN_OUTPUT_MODE);
  digitalWrite(RELAY_2, RELAY_STATE_OFF);

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
  publishPersistentState();
}

void publishPersistentState() {
  
  conn.sendUpdate([](JsonObject &json) {
    persistentStateToJson(json);
  });

}

ErrorReason handleUpdate(JsonObject& params) {
  bool processed = false;

  if (params.containsKey("thermostatMode")) {
    String mode = params["thermostatMode"];
    if (mode == "heat") {
      persistentState.enabled = true;
      processed = true;
    } else if (mode == "off") {
      persistentState.enabled = false;
      processed = true;
    } else {
      // received a mode that we don't support
      return "thermostatMode";
    }

  }

  if (params.containsKey("thermostatTemperatureSetpoint")) {
    JsonVariant point = params["thermostatTemperatureSetpoint"];
    if (point.is<float>()) {
      persistentState.desiredTemperature = point;
      processed = true;
    } else {
      return "thermostatTemperatureSetpoint";
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

  if (params.containsKey("forcePump")) {
    JsonVariant point = params["forcePump"];
    if (point.is<boolean>()) {
      transientState.forcePump = point;
      processed = true;
    } else {
      return "forcePump";
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

  bool changedDest = roundAndCheckIfChanged(transientState.heatSinkTemperature, lastPublishedState.heatSinkTemperature);
  bool failedSink = (transientState.heatSinkFailure != lastPublishedState.heatSinkFailure);
  bool changedSource = roundAndCheckIfChanged(transientState.heatSourceTemperature, lastPublishedState.heatSourceTemperature);
  bool failedSource = (transientState.heatSourceFailure != lastPublishedState.heatSourceFailure);
  bool changedPumpState = (transientState.pumpRunning != lastPublishedState.pumpRunning);
  bool changedDrainingState = (transientState.drainStartedMillis != lastPublishedState.drainStartedMillis);
  bool changedCleaningState = (transientState.cleaning != lastPublishedState.cleaning);

  bool shouldForcePublish = failedSink || failedSource || changedPumpState || changedDrainingState || changedCleaningState;

  if (changedDest || changedSource || shouldForcePublish) {
    unsigned long now = millis();
    if (timeIsUp(lastPublishedState.publishedMillis, transientState.minReportingIntervalMillis, now) || shouldForcePublish)  {
      conn.sendUpdate([=](JsonObject& json) {
        transientStateToJson(json);
      });
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

void loop() {

  conn.loop();
  updateTemps();
  updateRelays();
  publishIfNeeded();
  server.handleClient();
  loopOTA();
#ifdef DEBUG_ESP_PORT
  readAndSimulateTemp();
#endif
}

void disable() {
  if (persistentState.enabled) {
    persistentState.enabled = false;
    publishPersistentState();  
  }  
}

void updateTemps() {
  
  //DEBUG_MSG_("Heat source");
  if(heatSource.read()) {
    transientState.heatSourceTemperature = heatSource.value;
    transientState.heatSourceFailure = false;
  } else {
    transientState.heatSourceFailure = true;
  }

  //DEBUG_MSG_("Heat sink");
  if(heatSink.read()) {
    transientState.heatSinkTemperature = heatSink.value;
    transientState.heatSinkFailure = false;
  } else {
    transientState.heatSinkFailure = true;
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

void updateRelays() {

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

  if ((!transientState.pumpRunning) && desiredTempDiff) {
    if ((heatTemperatureDiff >= persistentState.upperTemperatureThreshold) || 
        ((transientState.heatSourceTemperature >= persistentState.maxTemperature) && heatTempDiffOverLowerThreshold)) {
      
      
      transientState.heatSourceAverageTemperatureCount = 1;
      transientState.heatSourceAverageTemperatureSum = transientState.heatSourceTemperature;
      transientState.heatSourceMaxTemperature = transientState.heatSourceTemperature;
      pumpSet(true);
    }
  } else {
    if (!heatTempDiffOverLowerThreshold) {
      pumpSet(false);
    } else if (transientState.pumpRunning) {
      transientState.heatSourceAverageTemperatureCount++;
      transientState.heatSourceAverageTemperatureSum += transientState.heatSourceTemperature;
      if (transientState.heatSourceTemperature > transientState.heatSourceMaxTemperature) {
        transientState.heatSourceMaxTemperature = transientState.heatSourceTemperature;
      }
    }
  }
  
}

void pumpSet(bool v) {

  if(transientState.pumpRunning != v) {
    if (v) {
      transientState.drainStartedMillis = 0;
    } else {
      unsigned long now = millis();
      if (!transientState.drainStartedMillis) {
        DEBUG_MSG_("Starting draining");
        transientState.drainStartedMillis = now;
      }
      if (!timeIsUp(transientState.drainStartedMillis, persistentState.drainSeconds * 1000, now)){
        return;
      }
      transientState.drainStartedMillis = 0;
      DEBUG_MSG_("Draining ended");
    }
    int val = v ? RELAY_STATE_ON : RELAY_STATE_OFF;
    DEBUG_MSG_("Setting pump to: ");
    DEBUG_MSG(v);
    digitalWrite(RELAY_1, val);
    digitalWrite(RELAY_2, val);
    digitalWrite(SECONDARY_LED, val);
    transientState.pumpRunning = v;
  }
  
}

bool timeIsUp(unsigned long startedMillis, unsigned long durationMillis, unsigned long nowMillis) {
  return ((startedMillis + durationMillis) <= nowMillis) || 
      (nowMillis < startedMillis); // watches for overflow
}

void handleGetStatus() {
  
  const size_t capacity = JSON_OBJECT_SIZE(40);
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

#ifdef DEBUG_ESP_PORT
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
