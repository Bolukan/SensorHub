#if !defined(ESP32)
#error This file is for ESP32S2 only
#endif

// ******************** compiler ********************
#define APP_NAME "Sensor ADC"
#include <version.h>  // include BUILD_NUMBER, VERSION, VERSION_SHORT

// ********************  INCLUDE  ********************

#include <WiFiConfig.h>  // user values
#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <myled.h>
//#include <WebServer.h>

// ******************** constants ********************

// sensors
#define SENSOR_COUNT 6
#define SENSORTYPE_PIR "pir"
#define SENSORTYPE_CONTACT "contact"

enum class SensorType {
  PIR,
  CONTACT
};

enum class SensorState {
  UNKNOWN,
  STATE_LOW,       // 0000-2499 = too low - invalid
  STATE_ACTIVE,    // 2500-4199 = 3300-3400 - active
  STATE_INACTIVE,  // 4200-5999 = 5000-5200 - inactive
  STATE_HIGH       // 6000-8191 = too high - invalid
};

struct SensorInfo {
  String sensorName;
  SensorType sensorType;
  String sensorLocation;
  uint8_t analogPin;
  uint8_t statusLedPin;
};

struct SensorData {
  uint16_t currentADCValue;
  uint16_t averageADCValue;
  SensorState currentState;
  MyLed statusLed;
  unsigned long previousReportTime; // last time the sensor was reported to initiate a periodic report
};

// timers
const long ADC_READ_INTERVAL_MS = 40;
const long PUBLISH_INTERVAL_MS = 1000 * 60 * 5;
const long WIFI_RECONNECT_INTERVAL_MS = 1000 * 60; // minute

// mqtt
const char MQTT_WILL_TOPIC[] = "sensor/esp32s2";

// ADC values
const uint16_t MIN_ACTIVE_ADC_VALUE = 2500;
const uint16_t MIN_INACTIVE_ADC_VALUE = 4200;
const uint16_t MAX_INACTIVE_ADC_VALUE = 5100;
const uint16_t MAX_HIGH_ADC_VALUE = 6000;

// pins
const uint8_t GPIO_S1 = 3;
const uint8_t GPIO_S2 = 5;
const uint8_t GPIO_S3 = 7;
const uint8_t GPIO_S4 = 9;
const uint8_t GPIO_S5 = 11;
const uint8_t GPIO_S6 = 12;

const uint8_t LED_CONN = 1;
const uint8_t LED_MQTT = 2;

const uint8_t LED_S1 = 14;
const uint8_t LED_S2 = 13;
const uint8_t LED_S3 = 10;
const uint8_t LED_S4 = 8;
const uint8_t LED_S5 = 6;
const uint8_t LED_S6 = 4;

const SensorInfo sensors[SENSOR_COUNT] = {
  {"A1", SensorType::PIR, "Hal", GPIO_S1, LED_S1},
  {"A2", SensorType::PIR, "Lounge", GPIO_S2, LED_S2},
  {"A3", SensorType::PIR, "Keuken", GPIO_S3, LED_S3},
  {"A4", SensorType::CONTACT, "Achterdeur", GPIO_S4, LED_S4},
  {"A5", SensorType::CONTACT, "Garagedeur", GPIO_S5, LED_S5},
  {"A6", SensorType::PIR, "Garage", GPIO_S6, LED_S6}
};

// ******************** globals ********************

SensorData sensorData[SENSOR_COUNT] = {
  {MAX_INACTIVE_ADC_VALUE, MAX_INACTIVE_ADC_VALUE, SensorState::STATE_INACTIVE, MyLed(LED_S1), 0UL},
  {MAX_INACTIVE_ADC_VALUE, MAX_INACTIVE_ADC_VALUE, SensorState::STATE_INACTIVE, MyLed(LED_S2), 0UL},
  {MAX_INACTIVE_ADC_VALUE, MAX_INACTIVE_ADC_VALUE, SensorState::STATE_INACTIVE, MyLed(LED_S3), 0UL},
  {MAX_INACTIVE_ADC_VALUE, MAX_INACTIVE_ADC_VALUE, SensorState::STATE_INACTIVE, MyLed(LED_S4), 0UL},
  {MAX_INACTIVE_ADC_VALUE, MAX_INACTIVE_ADC_VALUE, SensorState::STATE_INACTIVE, MyLed(LED_S5), 0UL},
  {MAX_INACTIVE_ADC_VALUE, MAX_INACTIVE_ADC_VALUE, SensorState::STATE_INACTIVE, MyLed(LED_S6), 0UL}
};

// WiFi and MQTT
WiFiClient mqttClient;
PubSubClient client(mqttClient);
char ssid[18] = {0}; // ssid in format "12:34:56:78:9a:bc", part of mqtt message
String clientId; // mqtt client identifier
unsigned long lastWiFiReconnectTime = 0UL;

// led
MyLed ledMQTT(LED_MQTT);
MyLed ledConn(LED_CONN);

// current time
unsigned long currentMillis = millis();
// last time ADC's read
unsigned long previousADCReadTime = 0UL; 

// ******************** functions ********************

// MQTT

/**
 * @brief debug print mqtt messages received
 * 
 * @param topic topic
 * @param payload message
 * @param length length of message
 */
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/**
 * @brief connect to MQTT
 * 
 */
void reconnectMQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    ledConn.Blink(500);
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_WILL_TOPIC, 1, true, "offline")) {
      Serial.printf("connected as %s\n", clientId.c_str());
      // Once connected, publish an announcement...
      client.publish(MQTT_WILL_TOPIC, "online", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again");
      // Wait before retrying
      delay(100);
    }
    ledConn.loop();
  }
}

/**
 * @brief MQTT publish sensor message
 * 
 * @param id sensor id
 * @param sensorState state of the sensor
 */
void publishSensorState(const int id) {
  ledMQTT.On();
  const SensorInfo &sensorInfo = sensors[id];
  const SensorData &sensorDataVar = sensorData[id];
  String topic = "sensor/" + sensorInfo.sensorName;
  String payload = "{\"mac\":\"" + String(ssid) + "\","
                   + "\"type\":\"" + (sensorInfo.sensorType == SensorType::PIR ? SENSORTYPE_PIR : SENSORTYPE_CONTACT) + "\","
                   + "\"location\":\"" + sensorInfo.sensorLocation + "\","
                   + "\"ADC\":" + sensorDataVar.currentADCValue;
  if (sensorInfo.sensorType == SensorType::PIR) {
    String jsonValue = (sensorDataVar.currentState == SensorState::STATE_ACTIVE) ? "true" : "false";
    payload += ",\"occupancy\":" + jsonValue + "}";
  } else {
    String jsonValue = (sensorDataVar.currentState == SensorState::STATE_ACTIVE) ? "false" : "true";
    payload += ",\"contact\":" + jsonValue + "}";
  }

  if (WiFi.status() == WL_CONNECTED) {
    client.publish(topic.c_str(), payload.c_str());
  }

  delay(0);
  ledMQTT.Off();
}

/**
 * @brief set ssid and clientId
 * 
 * @param ssid ssid in mqtt message
 * @param clientId MQTT client id 
 */
void getSSID(char (&ssid)[18], String& clientId) 
{
  uint8_t baseMac[6];
  char deviceid[7] = {0}; // ssid in format "789abc"
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  // ssid
  sprintf(ssid, "%02x:%02x:%02x:%02x:%02x:%02x", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  // clientId
  sprintf(deviceid, "%02x%02x%02x", baseMac[3], baseMac[4], baseMac[5]);
  clientId = "esp" + String(deviceid);
}

// ******************** setup and loop ********************

void setup() {
  // led
  ledConn.Blink(500);
  ledMQTT.Off();
  
  Serial.begin(115200);
  //delay(2000);
  Serial.println();
  Serial.println(APP_NAME);
  Serial.println(VERSION);
  Serial.println();

  // WiFi
  Serial.printf("Connecting to WiFi SSID: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (!WiFi.isConnected()) {
    Serial.print(".");
    ledConn.loop();
    delay(100);
  }
  Serial.println();
  ledConn.Blink(100);

  getSSID(ssid, clientId);

  // mqtt
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(callback);
  reconnectMQTT();
  ledConn.On();

}  // void setup()

// calculate the state of the sensor based on the ADC reading
SensorState calculateState(const uint16_t averageADC) {
  if (averageADC < MIN_ACTIVE_ADC_VALUE) return SensorState::STATE_LOW;
  if (averageADC < MIN_INACTIVE_ADC_VALUE) return SensorState::STATE_ACTIVE;
  if (averageADC < MAX_INACTIVE_ADC_VALUE) return SensorState::STATE_INACTIVE;
  return SensorState::STATE_HIGH;
}

/**
 * @brief Read sensor and process value
 * 
 */
void checkAnalogPinsAndPublish() {
  // Check ADC pins
  currentMillis = millis();
  if (currentMillis - previousADCReadTime >= ADC_READ_INTERVAL_MS) {
    previousADCReadTime = currentMillis;

    // read pins
    for (int id = 0; id < SENSOR_COUNT; id++) {
      const SensorInfo &sensorInfo = sensors[id];
      SensorData &sensorDataVar = sensorData[id];

      // read pin and calculate state
      sensorDataVar.currentADCValue = analogRead(sensorInfo.analogPin);
      sensorDataVar.averageADCValue -= sensorDataVar.averageADCValue >> 4; // replace 1/16 with new value
      sensorDataVar.averageADCValue += sensorDataVar.currentADCValue >> 4;
      delay(0);
      SensorState newSensorState = calculateState(sensorDataVar.averageADCValue);

      // publish a changed state
      if (sensorDataVar.currentState != newSensorState) {
        // update LED
        (newSensorState == SensorState::STATE_ACTIVE) ? sensorDataVar.statusLed.On() : sensorDataVar.statusLed.Off();
        // publish state
        sensorDataVar.currentState = newSensorState;
        publishSensorState(id);
        sensorDataVar.previousReportTime = millis();
      }
    }
  }
}

void publishUnchangedPins() {
  // publish unchanged pins (after 5 minutes)
  currentMillis = millis();

  for (int id = 0; id < SENSOR_COUNT; id++) {
    // const SensorInfo &sensorInfo = sensors[id];
    SensorData &sensorDataVar = sensorData[id];

    if (currentMillis - sensorDataVar.previousReportTime >= PUBLISH_INTERVAL_MS) {
      publishSensorState(id);
      sensorDataVar.previousReportTime = currentMillis;
    }
  }
}

void reconnectWiFi() {
  // if WiFi is down, try reconnecting
  if (WiFi.status() != WL_CONNECTED) {
    ledConn.Off();
    unsigned long currentMillis = millis();
    if (currentMillis - lastWiFiReconnectTime >= WIFI_RECONNECT_INTERVAL_MS) {
        Serial.print(currentMillis);
        Serial.println(" Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
        lastWiFiReconnectTime = currentMillis;
      }
  } else {
    ledConn.On();
  }
}  // void reconnectWiFi()

void loop() {
  // check wifi
  reconnectWiFi();
  // run mqtt
  if (WiFi.status() == WL_CONNECTED)
  {
    reconnectMQTT();
    client.loop();
  }
  // run leds
  ledConn.loop();
  ledMQTT.loop();

  checkAnalogPinsAndPublish();

  publishUnchangedPins();

}  // void loop()
