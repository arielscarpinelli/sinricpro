#include "certificates.h"

#include <ESP8266mDNS.h>

typedef String ErrorReason;

class ConnectionManager {

  protected:
    WebSocketsClient webSocket;
    bool connectingLed = false;
    bool wifiConnected = false;
    bool configuringWifi = false;
    bool connected = false;
    time_t connectionUpdatedAt;
    unsigned long nextDelayMillis = 0;

  public:

    void setup(String hostname, int timezoneOffsetSeconds) {
      // Connect to WiFi access point.

      WiFi.mode(WIFI_STA);
      WiFi.hostname(hostname);

#ifndef WLAN_SSID
      if (WiFi.SSID() != "") {
        DEBUG_MSG("Connecting to ");
        DEBUG_MSG(WiFi.SSID());
        WiFi.begin();
      } else {
        WiFi.beginSmartConfig();
        configuringWifi = true;
      }
#else 
      WiFi.begin(WLAN_SSID, WLAN_PASS);
#endif

      //WiFi.scanNetworksAsync(prinScanResult);

      configureTime(timezoneOffsetSeconds);
    
      webSocket.beginSslWithCA("api.iotmaster.dev", 443, "/api/ws?deviceid=" DEVICE_ID "&apikey=" API_KEY, ca_cert[0]);
      webSocket.onEvent([=](WStype_t type, uint8_t * payload, size_t length) {
        this->webSocketEvent(type, payload, length);
      });
      webSocket.enableHeartbeat(60000, 10000, 2);

    }

    void delay(unsigned long millisToWait) {
      nextDelayMillis = millis() + millisToWait;
    }

    void loop() {
    
      if (millis() < nextDelayMillis) {
        delay(0);
        return;
      }

      int wifiStatus;

      if (configuringWifi) {
        if(!WiFi.smartConfigDone()) {
          connectingLed = !connectingLed;
          digitalWrite(LED_BUILTIN, connectingLed ? HIGH : LOW);
          this->delay(200);
          return;
        }
        configuringWifi = false;
        WiFi.stopSmartConfig();
      }
    
      if ((wifiStatus = WiFi.status()) != WL_CONNECTED) {
        connectingLed = !connectingLed;
        digitalWrite(LED_BUILTIN, connectingLed ? HIGH : LOW);
        wifiConnected = false;
        this->delay(500);
        DEBUG_MSG_("Wifi status: ");
        DEBUG_MSG(wifiStatus);
        return;
      }

      if (!wifiConnected) {
        wifiConnected = true;
        DEBUG_MSG("WiFi connected");
        digitalWrite(LED_BUILTIN, HIGH);
        DEBUG_MSG("IP address: "); 
        DEBUG_MSG(WiFi.localIP());

        if(!MDNS.begin(WiFi.hostname())) {
          DEBUG_MSG("can't init MDNS");
        }

        MDNS.enableArduino(8266, true);
        //MDNS.addService("http", "tcp", 80);
      }
    
      if (!connected) {
        connectingLed = !connectingLed;
        digitalWrite(LED_BUILTIN, connectingLed ? LOW : HIGH);
        this->delay(500);
      } else {
        MDNS.update();      
      }
      webSocket.loop();
    }

    void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

      switch(type) {

        case WStype_DISCONNECTED:
          DEBUG_MSG("[WSc] Disconnected!\n");
          connected = false;
          connectionUpdatedAt = time(nullptr);
          break;

        case WStype_CONNECTED:
          DEBUG_MSG("[WSc] Connected to url:");
          DEBUG_WRITE(payload, length);
          connected = true;
          connectionUpdatedAt = time(nullptr);
          onConnect();     
          break;

        case WStype_TEXT:
          DEBUG_MSG("[WSc] got text:");
          DEBUG_WRITE(payload, length);

          const size_t capacity = JSON_OBJECT_SIZE(40);
          DynamicJsonDocument doc(capacity);

          // Deserialize the JSON document
          DeserializationError error = deserializeJson(doc, payload, length);

          // Test if parsing succeeds.
          if (error) {
            DEBUG_MSG_(F("deserializeJson() failed: "));
            DEBUG_MSG(error.c_str());
            return;
          }

          if (doc["action"] == "update") {
            const char *sequence = doc["sequence"];
            JsonObject params = doc["params"];

            String error = onUpdate(params);
            
            if (!error.length()) {
              sendRecipt(0, sequence, "");
            } else {
              sendRecipt(400, sequence, error);
            }
   
          }
          
          break;
      }
    }

    void sendRecipt(int errorCode, const char *sequence, String reason) {

      const size_t capacity = JSON_OBJECT_SIZE(20);
      DynamicJsonDocument doc(capacity);

      doc["error"] = 0;
      doc["sequence"] = sequence;

      if (reason.length()) {
        doc["reason"] = reason;
      }

      String json;
      serializeJson(doc, json);  
      
      webSocket.sendTXT(json);
      
    }

    virtual void onConnect() {}

    virtual ErrorReason onUpdate(JsonObject& params) {
      return "";
    }

    void sendAction(String action, std::function<void(JsonObject&)> paramsSetter) {

      const size_t capacity = JSON_OBJECT_SIZE(40);
      DynamicJsonDocument doc(capacity);

      doc["action"] = action;
      doc["deviceid"] = DEVICE_ID;
      doc["apikey"] = API_KEY;
      JsonObject params  = doc.createNestedObject("params");
      paramsSetter(params);

      if (params.size() > 0) {
        String json;
        serializeJson(doc, json);  
        
        webSocket.sendTXT(json);
      }
    }

    void sendUpdate(std::function<void(JsonObject&)> paramsSetter) {
      sendAction("update", paramsSetter);
    }

    bool isConnected() {
      return connected;
    }

    time_t getConnectionUpdatedAt() {
      return connectionUpdatedAt;
    }

    void configureTime(int timezoneOffsetSeconds) {
      configTime(timezoneOffsetSeconds, 0, "pool.ntp.org", "time.nist.gov");
    }

};

/*
  void prinScanResult(int networksFound)
  {
  Serial.printf("%d network(s) found\n", networksFound);
  for (int i = 0; i < networksFound; i++)
  {
    Serial.printf("%d: %s, Ch:%d (%ddBm) (%s) %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.BSSIDstr(i).c_str(), WiFi.encryptionType(i) == ENC_TYPE_NONE ? "open" : "");
  }
  }
*/
