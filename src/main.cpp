#if !defined(ESP32)
#error This file is for ESP32S2 only
#endif

// ******************** compiler ********************
#define APPNAME "Sensor ADC"
#include <version.h>  // include BUILD_NUMBER, VERSION, VERSION_SHORT

// ********************  INCLUDE  ********************

#include <WiFiConfig.h>  // user values
#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <myled.h>
//#include <WebServer.h>

// ******************** constants ********************

// timers
const long INTERVAL_ADC = 40;
const long INTERVAL_PRINT = 2000;
const long INTERVAL_PIN = 1000 * 60 * 5;
const long INTERVAL_LED = 50;

// mqtt
const char WILL_TOPIC[] = "sensor/esp32s2";

// pins
#define COUNT_PINS 6
#define SENSORTYPE_PIR "pir"
#define SENSORTYPE_CONTACT "contact"

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

const uint8_t ADC_Pins[] = {GPIO_S1, GPIO_S2, GPIO_S3, GPIO_S4, GPIO_S5, GPIO_S6};
const String SensorName[] = {"A1", "A2", "A3", "A4", "A5", "A6"};
const String SensorType[] = {SENSORTYPE_PIR, SENSORTYPE_PIR, SENSORTYPE_PIR, SENSORTYPE_CONTACT, SENSORTYPE_CONTACT, SENSORTYPE_PIR};
const String SensorLocation[] = {"Hal", "Lounge", "Keuken", "Achterdeur", "Garagedeur", "Garage"};

// ADC values
const uint16_t ADC_LOW_ACTIVE = 2500;
const uint16_t ADC_ACTIVE_INACTIVE = 4200;
const uint16_t ADC_INACTIVE = 5100;
const uint16_t ADC_INACTIVE_HIGH = 6000;

enum state_sensor {
  unknown,
  STATE_LOW,       // 0000-2499 = low
  STATE_ACTIVE,    // 2500-4199 = 3300-3400 - active
  STATE_INACTIVE,  // 4200-5999 = 5000-5200 - inactive
  STATE_HIGH       // 6000-8191 = high
};

// ******************** globals ********************

// WiFi and MQTT
WiFiClient ethClient;
PubSubClient client(ethClient);
char ssid[18] = {0}; // ssid in format "12:34:56:78:9a:bc"
char deviceid[7] = {0}; // ssid in format "789abc"
String clientId;

// ADC pins
uint16_t ADC[COUNT_PINS] = {ADC_INACTIVE};
uint16_t avgADC[COUNT_PINS] = {ADC_INACTIVE};
state_sensor sensor[COUNT_PINS] = {STATE_INACTIVE};

// led
MyLed ledMQTT(LED_MQTT);
MyLed ledConn(LED_CONN);
MyLed ledSensor[6] = { MyLed(LED_S1), MyLed(LED_S2), MyLed(LED_S3), MyLed(LED_S4), MyLed(LED_S5), MyLed(LED_S6)};

// loop
unsigned long currentMillis = millis();
unsigned long previousADC = 0;
unsigned long previousPrint = 0;
unsigned long previousPin[COUNT_PINS] = {0};
unsigned long previousLed = 0;

// ******************** functions ********************

// MQTT

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    ledConn.Blink(500);
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId.c_str(), WILL_TOPIC, 1, true, "offline")) {
      Serial.printf("connected as %s\n", clientId);
      // Once connected, publish an announcement...
      client.publish(WILL_TOPIC, "online", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again");
      // Wait before retrying
      ledConn.loop();
      delay(100);
    }
  }
}

void publishPIR(int id, state_sensor sensor)
{
  String topic = "sensor/" + SensorName[id];
  String valueOccupancy = (sensor==STATE_ACTIVE) ? "true" : "false";
  String payload = String("{\"occupancy\":" + valueOccupancy + ","
                   + "\"mac\":\"" + ssid + "\","
                   + "\"type\":\"" + SensorType[id] + "\","
                   + "\"location\":\"" + SensorLocation[id] + "\","
                   + "\"ADC\":" + ADC[id] + "}");
  client.publish(topic.c_str(), payload.c_str());
}

void publishcontact(int id, state_sensor sensor)
{
  String topic = "sensor/" + SensorName[id];
  String valueContact = (sensor==STATE_ACTIVE) ? "false" : "true";
  String payload = String("{\"contact\":" + valueContact + ","
                   + "\"mac\":\"" + ssid + "\","
                   + "\"type\":\"" + SensorType[id] + "\","
                   + "\"location\":\"" + SensorLocation[id] + "\","
                   + "\"ADC\":" + ADC[id] + "}");
  client.publish(topic.c_str(), payload.c_str());
}

void publishState(int id, state_sensor sensor) {
  ledMQTT.On();
  if (SensorType[id] == SENSORTYPE_PIR) publishPIR(id, sensor);
  if (SensorType[id] == SENSORTYPE_CONTACT) publishcontact(id, sensor);
  delay(0);
  ledMQTT.Off();
}

// set ssid, deviceid, clientId
void getssid() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  sprintf(ssid, "%02x:%02x:%02x:%02x:%02x:%02x", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  sprintf(deviceid, "%02x%02x%02x", baseMac[3], baseMac[4], baseMac[5]);
  clientId = String("esp") + String(deviceid);
}

// ******************** setup and loop ********************

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println(APPNAME);
  Serial.println(VERSION);
  Serial.println();

  // led
  ledConn.Blink(500);
  ledMQTT.Off();

  // WiFi
  Serial.printf("Connecting to WiFi SSID: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (!WiFi.isConnected()) {
    Serial.print(".");
    delay(200);
  }
  Serial.println();
  ledConn.Blink(100);

  getssid();

  // mqtt
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(callback);
  reconnect();
  ledConn.On();

}  // void setup()

void loop() {
  // run mqtt
  reconnect();
  client.loop();
  ledConn.loop();
  ledMQTT.loop();

  // Check ADC pins
  currentMillis = millis();
  if (currentMillis - previousADC >= INTERVAL_ADC) {
    previousADC = currentMillis;
    
    // read pins
    for (int id = 0; id < COUNT_PINS; id++) {
      // read pin
      ADC[id] = analogRead(ADC_Pins[id]);
      avgADC[id] -= avgADC[id] >> 4;
      avgADC[id] += ADC[id] >> 4;
      delay(0);

      // calculate state
      state_sensor new_state;
      if (avgADC[id] < ADC_LOW_ACTIVE)
      {
        new_state = STATE_LOW;
        ledSensor[id].Off();
      }
      else if (avgADC[id] < ADC_ACTIVE_INACTIVE)
      {
        new_state = STATE_ACTIVE;
        ledSensor[id].On();
      }
      else if (avgADC[id] < ADC_INACTIVE_HIGH)
      {
        new_state = STATE_INACTIVE;
        ledSensor[id].Off(); 
      }
      else
      {
        new_state = STATE_HIGH;
        ledSensor[id].Off();
      }

      // changed state
      if (sensor[id] != new_state) {
        // publish state
        publishState(id, new_state);
        sensor[id] = new_state;
        previousPin[id] = millis();
      }
    }
  }

  // publish unchanged pins (after 5 minutes)
  for (int id = 0; id < COUNT_PINS; id++) {
    currentMillis = millis();
    if (currentMillis - previousPin[id] >= INTERVAL_PIN) {
      previousPin[id] = currentMillis;
      publishState(id, sensor[id]);
    }
  }

  /*
  // Print the pins
  currentMillis = millis();
  if (currentMillis - previousPrint >= INTERVAL_PRINT) {
    previousPrint = currentMillis;

    Serial.print("ADC's = ");
    for (int id = 0; id < COUNT_PINS; id++) {
      Serial.printf(" %d:%04d ", id, ADC[id]);
    }
    // Time passed
    // Serial.print(" Time passed: ");
    // currentMillis = millis();
    // Serial.print(currentMillis - previousPrint);
    Serial.println();
  }
  */

}  // void loop()