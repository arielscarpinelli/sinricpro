#ifdef ESP8266
  #include <ESP8266mDNS.h>
#else
  #include <ESPmDNS.h>
#endif

#include <SinricPro.h>
#include <SinricProDevice.h>
#include <Capabilities/ModeController.h>
#include <Capabilities/ToggleController.h>
#include <Capabilities/RangeController.h>
#include <Capabilities/PushNotification.h>
#include <Capabilities/PowerStateController.h>
#include <Capabilities/TemperatureSensor.h>

#define CONFIGURING_WIFI_MODE_NOT_CONFIGURING 0
#define CONFIGURING_WIFI_MODE_SMARTCONFIG 1
#define CONFIGURING_WIFI_MODE_AP_CONFIG 2

class SolarPoolHeather
    : public SinricProDevice,
      public ToggleController<SolarPoolHeather>,
      public RangeController<SolarPoolHeather>,
      public ModeController<SolarPoolHeather>,
      public PushNotification<SolarPoolHeather>,
      public PowerStateController<SolarPoolHeather>,
      public TemperatureSensor<SolarPoolHeather> {
  friend class ToggleController<SolarPoolHeather>;
  friend class RangeController<SolarPoolHeather>;
  friend class PushNotification<SolarPoolHeather>;
  friend class ModeController<SolarPoolHeather>;
  friend class PowerStateController<SolarPoolHeather>;
  friend class TemperatureSensor<SolarPoolHeather>;
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
  int configuringWifiMode = 0;
  bool connected = false;
  bool offline = false;
  time_t connectionUpdatedAt;
  unsigned long nextDelayMillis = 0;

  SolarPoolHeather &solarPoolHeather = SinricPro[DEVICE_ID];

  virtual void onConnect() {} ;
  virtual ErrorReason onUpdate(JsonObject& params) { return "Not implmented"; };

public:
  void setup(String hostname, int timezoneOffsetSeconds, bool offline) {
    // Connect to WiFi access point.

    WiFi.mode(WIFI_STA);
    #ifdef ESP8266
      WiFi.hostname(hostname);
    #else
      WiFi.setHostname(hostname.c_str());
    #endif    

    if(!offline) {
    #ifndef WLAN_SSID
      if (WiFi.SSID() != "") {
        DEBUG_MSG_("Connecting to ");
        DEBUG_MSG(WiFi.SSID());
        WiFi.begin();
        // Timeout trying to connect, then enter smartconfig
      } else {
        this->resetWifiConfig();
      }
    #else
        WiFi.begin(WLAN_SSID, WLAN_PASS);
    #endif
    } else {
      this->offline = true;
      WiFi.softAP(hostname);
    }

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
      if (request.request_value.containsKey("temperature")) {
        request.request_value[key] = request.request_value["temperature"];
      };
      if (request.request_value.containsKey("thermostatMode")) {
        request.request_value[key] = request.request_value["thermostatMode"];
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
        digitalWrite(LED_BUILTIN, LED_ON);
        onConnect(); 
    });

    SinricPro.onDisconnected([this]() { 
        DEBUG_MSG("Disconnected from SinricPro"); 
        connected = false;
        connectingLed = false;
        digitalWrite(LED_BUILTIN, LED_OFF);
        connectionUpdatedAt = time(nullptr); 
    });

    SinricPro.begin(APP_KEY, APP_SECRET);
  }

  void loop(unsigned long now) {

    if (offline) {
      return;
    }

    if (wifiConnected) {
      SinricPro.handle();
    }

    if (now < nextDelayMillis) {
      return;
    }
    nextDelayMillis = 0;

    int wifiStatus;

    if ((wifiStatus = WiFi.status()) != WL_CONNECTED) {
      connectingLed = !connectingLed;
      digitalWrite(LED_BUILTIN, connectingLed ? LED_ON : LED_OFF);
      wifiConnected = false;
      nextDelayMillis = now + this->ledBlinkDelay();
      return;
    }

    if (!wifiConnected) {
      wifiConnected = true;
      DEBUG_MSG("WiFi connected");
      DEBUG_MSG("IP address: ");
      DEBUG_MSG(WiFi.localIP());

      if (!MDNS.begin(hostname())) {
        DEBUG_MSG("can't init MDNS");
      }

      MDNS.enableArduino(8266, true);
      // MDNS.addService("http", "tcp", 80);
    }

    if (!connected) {
      connectingLed = !connectingLed;
      digitalWrite(LED_BUILTIN, connectingLed ? LED_ON : LED_OFF);
      nextDelayMillis = now + 500;
    } else {
      #ifdef ESP8266
        MDNS.update();
      #endif  
    }
  }

  bool isConnected() {
    return connected;
  }

  #ifdef ESP8266 
    inline String hostname() {
      return WiFi.hostname();
    }
  #else 
    inline const char* hostname() {
      return WiFi.getHostname();
    }
  #endif
  
  time_t getConnectionUpdatedAt() {
    return connectionUpdatedAt;
  }

  void resetWifiConfig() {
    connected = false;
    wifiConnected = false;
    offline = false;

    switch(configuringWifiMode) {
      case CONFIGURING_WIFI_MODE_NOT_CONFIGURING:
      default:
        configuringWifiMode = CONFIGURING_WIFI_MODE_SMARTCONFIG;
        WiFi.beginSmartConfig();
        break;

      case CONFIGURING_WIFI_MODE_SMARTCONFIG:
        configuringWifiMode = CONFIGURING_WIFI_MODE_AP_CONFIG;
        WiFi.stopSmartConfig();
        WiFi.softAP("SL12345");
        break;

      case CONFIGURING_WIFI_MODE_AP_CONFIG:
        configuringWifiMode = CONFIGURING_WIFI_MODE_NOT_CONFIGURING;
        WiFi.mode(WIFI_STA);
        WiFi.begin();
        
        break;
    }

    DEBUG_MSG_("configuring mode ");
    DEBUG_MSG(configuringWifiMode);
  }

  int inline ledBlinkDelay() {
    switch(this->configuringWifiMode) {
      case CONFIGURING_WIFI_MODE_SMARTCONFIG:
        return 300;
      case CONFIGURING_WIFI_MODE_AP_CONFIG:
        return 2000;    
      default:
        return 800;
    }
  }

  void config(String ssid, String pass) {
    offline = false;
    WiFi.mode(WIFI_STA);
    WiFi.persistent(true);
    WiFi.begin(ssid, pass);
  }

  void setOffline(bool o) {
    if (o != offline) {
      offline = o;
      if (o) {
        WiFi.softAP(hostname());
      } else {
        WiFi.mode(WIFI_STA);
        WiFi.begin();        
      }
    }
  }

  void configureTime(int timezoneOffsetSeconds) {
    configTime(timezoneOffsetSeconds, 0, "pool.ntp.org", "time.nist.gov");
  }

  virtual void sendRangeValue(const String& instance, float value) {
    solarPoolHeather.sendRangeValueEvent(instance, value);
    SinricPro.handle();
  }

  virtual void sendTemperature(float value) {
    solarPoolHeather.sendTemperatureEvent(value);
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
