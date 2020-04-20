/**
    Required libraries:
      - Adafruit BME280 Library
      - Adafruit Unified Sensor
      - PubSubClient
**/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define MQTT_TOPIC_HUMIDITY "ext/bme680/humidity"
#define MQTT_TOPIC_PRESSURE "ext/bme680/pressure"
#define MQTT_TOPIC_TEMPERATURE "ext/bme680/temperature"
#define MQTT_TOPIC_VOC "ext/bme680/airquality"

#define MQTT_TOPIC_STATE "ext/bme680/status"
#define MQTT_PUBLISH_DELAY 10000
#define MQTT_CLIENT_ID "esp8266bme680"

#define BME680_ADDRESS 0x77

const char *WIFI_SSID = "Vodafone-34709121";
const char *WIFI_PASSWORD = "3yx3mzehfa8t9s3";

const char *MQTT_SERVER = "192.168.1.7";
const char *MQTT_USER = NULL; // NULL for no authentication
const char *MQTT_PASSWORD = NULL; // NULL for no authentication

float humidity;
float pressure;
float temperature;
float voc;
long lastMsgTime = 0;

Adafruit_BME680 bme;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup() {
  Serial.begin(115200);
  while (! Serial);

  if (!bme.begin(BME680_ADDRESS)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring or BME-280 address!");
    while (1);
  }

  // Use force mode so that the sensor returns to sleep mode when the measurement is finished
  /*bme.setSampling(Adafruit_BME680::MODE_FORCED,
                    Adafruit_BME680::SAMPLING_X1, // temperature
                    Adafruit_BME680::SAMPLING_X1, // pressure
                    Adafruit_BME680::SAMPLING_X1, // humidity
                    Adafruit_BME680::FILTER_OFF   );
  */
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  setupWifi();
  mqttClient.setServer(MQTT_SERVER, 1883);
}

void loop() {
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  long now = millis();
  if (now - lastMsgTime > MQTT_PUBLISH_DELAY) {
    lastMsgTime = now;

    // Reading BME280 sensor data
    // bme.takeForcedMeasurement(); // has no effect in normal mode
    humidity = bme.readHumidity();
    pressure = bme.readPressure();
    temperature = bme.readTemperature();
    voc = bme.readGas();
    /*if (isnan(humidity) || isnan(temperature) || isnan(pressure)) {
      Serial.println("BME280 reading issues");
      return;
    }
    */

    // Publishing sensor data
    mqttPublish(MQTT_TOPIC_TEMPERATURE, temperature);
    mqttPublish(MQTT_TOPIC_PRESSURE, pressure);
    mqttPublish(MQTT_TOPIC_HUMIDITY, humidity);
    mqttPublish(MQTT_TOPIC_VOC, voc);


  }
}

void setupWifi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_STATE, 1, true, "disconnected", false)) {
      Serial.println("connected");

      // Once connected, publish an announcement...
      mqttClient.publish(MQTT_TOPIC_STATE, "connected", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttPublish(char *topic, float payload) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);

  mqttClient.publish(topic, String(payload).c_str(), true);
}
