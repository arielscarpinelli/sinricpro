#include <Arduino.h>
#define RELAY_PIN_OUTPUT_MODE OUTPUT // OUTPUT_OPEN_DRAIN
#define RELAY_STATE_ON  LOW
#define RELAY_STATE_OFF HIGH

/************************* Testing *********************************/

//#define SIMULATED_TEMP  ((float)random(15, 40)) / 10

/************************* WiFi Access Point *********************************/

// unset to use stored ssid / smartConfig 
#define WLAN_SSID       "MyWifi"
#define WLAN_PASS       "SomePassword"

#define OTA_PASSWORD    "SomePassword"
#define HOSTNAME        "temperature"

/************************* SinricPro Setup *********************************/

#define APP_KEY         "APP_KEY"
#define APP_SECRET      "APP_SECRET"
#define DEVICE_ID       "deviceId"
