#include <ESP8266mDNS.h>

#include <SinricPro.h>
#include <SinricProDevice.h>
#include <Capabilities/ModeController.h>
#include <Capabilities/ToggleController.h>
#include <Capabilities/RangeController.h>
#include <Capabilities/PushNotification.h>
#include <Capabilities/PowerStateController.h>

class SolarPoolHeather
    : public SinricProDevice,
      public ToggleController<SolarPoolHeather>,
      public RangeController<SolarPoolHeather>,
      public ModeController<SolarPoolHeather>,
      public PushNotification<SolarPoolHeather>,
      public PowerStateController<SolarPoolHeather> {
  friend class ToggleController<SolarPoolHeather>;
  friend class RangeController<SolarPoolHeather>;
  friend class PushNotification<SolarPoolHeather>;
  friend class ModeController<SolarPoolHeather>;
  friend class PowerStateController<SolarPoolHeather>;
  friend class ConnectionManager;

public:
  SolarPoolHeather(const String &deviceId) : SinricProDevice(deviceId, "SolarPoolHeather"){};
};

class StateSerializer {
public:
  virtual void sendRangeValue(const String& instance, float value);
  virtual void sendPowerState(bool value);
};

typedef String ErrorReason;

class ConnectionManager : public StateSerializer {

protected:
  bool connectingLed = false;
  bool wifiConnected = false;
  bool configuringWifi = false;
  bool connected = false;
  time_t connectionUpdatedAt;
  unsigned long nextDelayMillis = 0;

  SolarPoolHeather &solarPoolHeather = SinricPro[DEVICE_ID];

  virtual void onConnect() {} ;
  virtual ErrorReason onUpdate(JsonObject& params) { return "Not implmented"; };

public:
  void setup(String hostname, int timezoneOffsetSeconds) {
    // Connect to WiFi access point.

    WiFi.mode(WIFI_STA);
    WiFi.hostname(hostname);

#ifndef WLAN_SSID
    if (WiFi.SSID() != "") {
      DEBUG_MSG_("Connecting to ");
      DEBUG_MSG(WiFi.SSID());
      WiFi.begin();
    } else {
      WiFi.beginSmartConfig();
      configuringWifi = true;
    }
#else
    WiFi.begin(WLAN_SSID, WLAN_PASS);
#endif

    // WiFi.scanNetworksAsync(prinScanResult);

    configureTime(timezoneOffsetSeconds);

    solarPoolHeather.registerRequestHandler([this](const SINRICPRO_NAMESPACE::SinricProRequest& request) {
      String key = request.instance == "" ? request.action : request.instance;

      if (request.request_value.containsKey("rangeValue")) {
        request.request_value[key] = request.request_value["rangeValue"];
      };
      if (request.request_value.containsKey("state")) {
        request.request_value[key] = request.request_value["state"];
      };
      bool ret = (onUpdate(request.request_value) == "");
      if (ret) {
        for (JsonPair kv : request.request_value) {
          request.response_value[kv.key()] = kv.value();
        }        
      }
      return ret;
    });

    SinricPro.onConnected([this]() { 
        DEBUG_MSG("Connected to SinricPro"); 
        connected = true;
        connectionUpdatedAt = time(nullptr);
        onConnect(); 
    });

    SinricPro.onDisconnected([this]() { 
        DEBUG_MSG("Disconnected from SinricPro"); 
        connected = false;
        connectionUpdatedAt = time(nullptr); 
    });

    SinricPro.begin(APP_KEY, APP_SECRET);
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
      if (!WiFi.smartConfigDone()) {
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

      if (!MDNS.begin(WiFi.hostname())) {
        DEBUG_MSG("can't init MDNS");
      }

      MDNS.enableArduino(8266, true);
      // MDNS.addService("http", "tcp", 80);
    }

    if (!connected) {
      connectingLed = !connectingLed;
      digitalWrite(LED_BUILTIN, connectingLed ? LOW : HIGH);
      this->delay(500);
    } else {
      MDNS.update();
    }

    SinricPro.handle();
    SinricPro.handle();
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

  virtual void sendRangeValue(const String& instance, float value) {
    solarPoolHeather.sendRangeValueEvent(instance, value);
    SinricPro.handle();
  }

  virtual void sendPowerState(bool value) {
    solarPoolHeather.sendPowerStateEvent(value);
    SinricPro.handle();
  }

  void sendToggleValue(const String& instance, bool value) {
    solarPoolHeather.sendToggleStateEvent(instance, value);
    SinricPro.handle();
  }

  void sendModeValue(const String& instance, bool value) {
    String mode = value ? "On" : "Off";
    solarPoolHeather.sendModeEvent(instance, mode, SINRICPRO_NAMESPACE::FSTR_SINRICPRO_PHYSICAL_INTERACTION);
    SinricPro.handle();
  }

  void notifyError(const String& error) {
    solarPoolHeather.sendPushNotification(error);
    SinricPro.handle();
}
};

/*
  void prinScanResult(int networksFound) {
  Serial.printf("%d network(s) found\n", networksFound);
  for (int i = 0; i < networksFound; i++) {
    Serial.printf("%d: %s, Ch:%d (%ddBm) (%s) %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.BSSIDstr(i).c_str(), WiFi.encryptionType(i) == ENC_TYPE_NONE ? "open" : "");
  }
}
*/
