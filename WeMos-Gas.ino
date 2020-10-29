extern "C"{
#include "user_interface.h"
}

#include "ESP8266WiFi.h"
#include "Blink.h"
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include "DHT.h"

#include <MQTT.h>

#define USE_ARDUINO_OTA true

#ifdef USE_ARDUINO_OTA
#include <ArduinoOTA.h>
#endif

#define HOSTNAME "wemos-gas"

#define TOPIC_GAS "iot/home/energy/gas"
#define TOPIC_TEMP "iot/home/weather/gastemp"
#define TOPIC_HUM "iot/home/weather/gashum"

#define LED_PIN D2

#define REED_PIN D3

#define DHTPIN D4
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

Blink blinkerInternal(LED_BUILTIN);
Blink blinkerLed(LED_PIN);

ESP8266WiFiMulti wifiMulti;
//IPAddress ip(192, 168, 1, 98);
//IPAddress dns(192, 168, 1, 86);
//IPAddress gw(192, 168, 1, 1);
//IPAddress subnet(255, 255, 255, 0);

WiFiClient espClient;
MQTTClient mqttClient;
const char * mqttServer = "10.10.10.3";
const int mqttPort = 1883;
unsigned long mqttLastReconnectAttempt = 0;
wl_status_t lastWiFiState = WL_CONNECTED;
int reedState = HIGH; // initial state of reed sensor
const float gasTickValue = 0.01; // value of gas per one tick - 0.01m3
const float gasTickCountThreshold = 5; // how many tick has to happen to send data (try to relieve DB)

const unsigned long WIFI_WAIT_TIMEOUT = 30 * 5;  // wait for WiFi for 20 secs 
const unsigned long WEATHER_SENT_INTERVAL = 1000 * 60 * 10 ;  // send weather every 10 mins 
const unsigned long GAS_SENT_INTERVAL = 1000 * 60 * 60 ;      // send empty gas every 60 mins 
const unsigned long DATA_GAS_SENT_INTERVAL = 1000 * 60 * 10 ; // send any gas data every 10 mins 

enum DeviceState {
  STATE_INIT,
  STATE_OK,
  STATE_ERROR
};

DeviceState deviceState = STATE_INIT;
DeviceState lastDeviceState = STATE_INIT;

struct MyData {
  int gasTickCount;
  unsigned long lastGasSent;      // last time gas data was sent to cloud
  float temp;
  float hum;
  unsigned long lastWeatherSent;  // last time weather data was sent to cloud
};

MyData data;

void setupReed() {
  pinMode(REED_PIN, INPUT_PULLUP);
}

void setupDht() {
  dht.begin();
}

void setupOTA() {
  String hostname(HOSTNAME);
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();
}

void setupWiFi() {
  WiFi.forceSleepWake();
  delay(1);

  // Bring up the WiFi connection
  WiFi.mode(WIFI_STA);
//  WiFi.printDiag(Serial);
//  WiFi.config(ip, dns, gw, subnet); 

//  WiFi.begin("intelis", "password");

  // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
  WiFi.persistent(false);

  String hostname(HOSTNAME);
  WiFi.hostname(hostname);
  Serial.print(F("Hostname: "));
  Serial.println(hostname.c_str());

  wifiMulti.addAP("intelis", "password");
  wifiMulti.addAP("KOLDA_WLAN2", "password");
}

bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    deviceState = STATE_OK;
    return true;
  }

  String hostname(HOSTNAME);
  WiFi.hostname(hostname);
  
  if (wifiMulti.run() != WL_CONNECTED) {
//  if (WiFi.status() != WL_CONNECTED) {
    
    if (lastWiFiState == WL_CONNECTED) {
      Serial.print(F("Connecting to WiFi"));
      deviceState = STATE_ERROR;
    }
    Serial.print(F("."));
    lastWiFiState = WiFi.status();
  } else {
    if (lastWiFiState != WL_CONNECTED) {
      Serial.println(F("WiFi connected"));
      Serial.print(F("IP address: "));
      Serial.println(WiFi.localIP().toString().c_str());
      Serial.print(F("WiFi connected in: ")); 
      Serial.println(millis());
      deviceState = STATE_OK;
    }
    lastWiFiState = WiFi.status();
    return true;
  }
  return false;
}

void checkDeviceState() {
  if (deviceState != lastDeviceState) {
    Serial.print(F("Device state changed - NEW: "));
    Serial.print(deviceState);
    Serial.print(F(", OLD: "));
    Serial.println(lastDeviceState);
    if (deviceState == STATE_OK) {
      blinkerLed.init({100, 200, 100, 3000}, 0); // heartbeat
    } else {
      blinkerLed.init({10, 50}, 0); // blink quickly
    }
    blinkerLed.start();
    lastDeviceState = deviceState;
  }
}

void checkWifiConnected() {
  // restart if wifi not connected for some time
  for (int i = 0; i < WIFI_WAIT_TIMEOUT; i++) { // timeout checking for WiFi connection
    if (connectWiFi()) {
      return; // connected
    } else {
      digitalWrite(LED_PIN, i%2==0 ? HIGH : LOW);
      delay(200);
    }
  }
  deviceState = STATE_ERROR;
  Serial.println(F("WiFi not found"));
  ESP.restart(); // restart
  delay(1000);
}

void checkMqttConnected() {
  mqttClient.loop();
  delay(10);  // <- fixes some issues with WiFi stability
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
}

void mqttReconnect() {
  if (!mqttClient.connected()) {
    deviceState = STATE_ERROR;
    unsigned long now = millis();
    if (now - mqttLastReconnectAttempt > 5000) {
      mqttLastReconnectAttempt = now;
      // Attempt to reconnect
      Serial.println(F("Connecting to MQTT..."));
      // Attempt to connect
      if (mqttClient.connect(HOSTNAME)) {
        Serial.println(F("MQTT connected"));
        mqttLastReconnectAttempt = 0;
        deviceState = STATE_OK;
      } else {
        Serial.println(F("MQTT connection failed"));
      }
    }
  }
}

void readWeather(MyData& data) {
  for (int i=0; i<20;i++) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      delay(200);
    } else {
      data.temp = t;
      data.hum = h;
      break;
    }
  }
}

String createJson(String topic, String meter, float value) {
  // {"topic" : "energy", "meter" : "gas", "dev" : "wemos", "value" : 0.01}
  String result = "{\"topic\" : \"";
  result += topic;
  result += "\", \"meter\" : \"";
  result += meter;
  result += "\", \"dev\" : \"";
  result += HOSTNAME;
  result += "\", \"value\" : ";
  result += value;
  result += "}";
  return result;
}

void checkReedSensor() {
  int lastReedState = reedState;

  reedState = digitalRead(REED_PIN);
  
  // If the state has changed then publish the change
  if (lastReedState != reedState) {
    Serial.print(F("Reed state changed: "));
    Serial.println(reedState);
    if (reedState == LOW) { // tick
      blinkerInternal.init({0, 1}, 1); // tiny blink
      blinkerInternal.start();
      if (++data.gasTickCount >= gasTickCountThreshold) {
        if (mqttPublish(TOPIC_GAS, createJson("energy", "gas", data.gasTickCount * gasTickValue))) {
          data.gasTickCount = 0;
          data.lastGasSent = millis();
        }
      }
    }
  } else if (mqttClient.connected() && (millis() - data.lastGasSent > GAS_SENT_INTERVAL)) {  // once a while send gas state even no state changed 
    if (mqttPublish(TOPIC_GAS, createJson("energy", "gas", data.gasTickCount * gasTickValue))) {
      data.gasTickCount = 0;
      data.lastGasSent = millis();
    }
  } else if (mqttClient.connected() && (data.gasTickCount > 0) && (millis() - data.lastGasSent > DATA_GAS_SENT_INTERVAL)) {  // once a shorter while send gas state when any data present 
    if (mqttPublish(TOPIC_GAS, createJson("energy", "gas", data.gasTickCount * gasTickValue))) {
      data.gasTickCount = 0;
      data.lastGasSent = millis();
    }
  }
}

void sendWeather(MyData& data) {
  readWeather(data);
  
  mqttPublish(TOPIC_TEMP, createJson("weather", "temp", data.temp));
  mqttPublish(TOPIC_HUM, createJson("weather", "hum", data.hum));
  data.lastWeatherSent = millis();
  blinkerInternal.init({0, 50, 200, 50, 10}, 1); // double blink
  blinkerInternal.start();
}

bool mqttPublish(char *topic, String msg) {
  if (mqttClient.connected()) {
    if (mqttClient.publish(topic, msg.c_str())) {
      Serial.print(F("MQTT data sent: "));
      Serial.println(msg.c_str());
    } else {
      Serial.println(F("MQTT data NOT SENT!"));
    }

    blinkerInternal.init({0, 50, 10}, 1); // one blink
    blinkerInternal.start();
    delay(50);
  }
}

void setup() {
  // start with WiFi OFF to get some mA saved
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
  
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);

  Serial.println(F("Starting..."));
  
  // WeMos has inverse LED (LOW - on, HIGH - no off)
  blinkerInternal.inverse(true);

  setupReed();
  setupDht();

  #ifdef USE_ARDUINO_OTA
    setupOTA();
  #endif
  
  setupWiFi();

  mqttClient.begin(mqttServer, mqttPort, espClient);

  deviceState = STATE_ERROR;
}

void loop() {
  checkWifiConnected();
  checkMqttConnected();
  checkDeviceState();

  #ifdef USE_ARDUINO_OTA
    ArduinoOTA.handle();
  #endif
  
  yield();

  checkReedSensor();

  if (mqttClient.connected() && (millis() - data.lastWeatherSent > WEATHER_SENT_INTERVAL || data.lastWeatherSent == 0)) {
    sendWeather(data);
  }
}
