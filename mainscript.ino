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
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "SSD1306.h"
#include <WiFiUdp.h>
#include <EasyNTPClient.h>
#include <Time.h>
#include "SD.h"

const int chipSelect = D8;

WiFiUDP udp;
EasyNTPClient ntpClient(udp, "pool.ntp.org", 0); // IST = GMT + 5:30

time_t ts;
long start_ts;

String str;
String fstr;

SSD1306  display(0x3C, D2, D1);
String disp;

#define MQTT_TOPIC_HUMIDITY "ext/bme680/humidity"
#define MQTT_TOPIC_PRESSURE "ext/bme680/pressure"
#define MQTT_TOPIC_TEMPERATURE "ext/bme680/temperature"
#define MQTT_TOPIC_VOC "ext/bme680/airquality"

#define MQTT_TOPIC_HUMIDITY2 "ext/bme280/humidity"
#define MQTT_TOPIC_PRESSURE2 "ext/bme280/pressure"
#define MQTT_TOPIC_TEMPERATURE2 "ext/bme280/temperature"

#define MQTT_TOPIC_STATE "ext/bme680/status"
#define MQTT_PUBLISH_DELAY 10000
#define MQTT_CLIENT_ID "esp8266bme680"

#define BME680_ADDRESS 0x77
#define BME280_ADDRESS 0x76

const char *WIFI_SSID = "Vodafone-34709121";
const char *WIFI_PASSWORD = "3yx3mzehfa8t9s3";

const char *MQTT_SERVER = "192.168.1.7";
const char *MQTT_USER = NULL; // NULL for no authentication
const char *MQTT_PASSWORD = NULL; // NULL for no authentication

float humidity;
float pressure;
float temperature;
float voc;

float humidity2;
float pressure2;
float temperature2;


long lastMsgTime = 0;

Adafruit_BME680 bme;

Adafruit_BME280 bme2;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup() {

  display.init();
  //display.flipScreenVertically();

  Serial.begin(115200);
  while (! Serial);

  if (!bme.begin(BME680_ADDRESS)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring or BME-680 address!");
    while (1);
  }

  if (!bme2.begin(BME280_ADDRESS)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring or BME-280 address!");
    while (1);
  }

  ts = ntpClient.getUnixTime();

  // Use force mode so that the sensor returns to sleep mode when the measurement is finished
  /*bme.setSampling(Adafruit_BME680::MODE_FORCED,
                    Adafruit_BME680::SAMPLING_X1, // temperature
                    Adafruit_BME680::SAMPLING_X1, // pressure
                    Adafruit_BME680::SAMPLING_X1, // humidity
                    Adafruit_BME680::FILTER_OFF   );
  */

  bme.setTemperatureOversampling(BME680_OS_16X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  bme2.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

  setupWifi();
  mqttClient.setServer(MQTT_SERVER, 1883);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");



  //display.setTextAlignment(TEXT_ALIGN_CENTER);

}

void loop() {
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  long now = millis();
  if (now - lastMsgTime > MQTT_PUBLISH_DELAY) {
    lastMsgTime = now;   //declare a string as an array of chars

    File file6 = SD.open("bme680.txt", FILE_WRITE);


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


    bme2.takeForcedMeasurement(); // has no effect in normal mode
    humidity2 = bme2.readHumidity();
    pressure2 = bme2.readPressure();
    temperature2 = bme2.readTemperature();


    // Publishing sensor data
    mqttPublish(MQTT_TOPIC_TEMPERATURE, temperature);
    mqttPublish(MQTT_TOPIC_PRESSURE, pressure);
    mqttPublish(MQTT_TOPIC_HUMIDITY, humidity);
    mqttPublish(MQTT_TOPIC_VOC, voc);


    mqttPublish(MQTT_TOPIC_TEMPERATURE2, temperature2);
    mqttPublish(MQTT_TOPIC_PRESSURE2, pressure2);
    mqttPublish(MQTT_TOPIC_HUMIDITY2, humidity2);

    ts = ntpClient.getUnixTime();

    str = String(ts);
    Serial.println(str);
    disp = String((String)str + "\n" + "TEMP  " + (String)temperature + (String)temperature2 + " °C \n" + "RH    " + (String)humidity + (String)humidity2 + "% \n" + "PRES  " + (String)pressure + " Pa \n" + "VOC  " + (String)voc + " Ω \n");
    fstr = String((String) ts + ";" + (String)temperature + ";" + (String)humidity + ";" + (String)pressure + ";" + (String)voc + ";");
    file6.println(fstr);
    file6.close();
    //sprintf(disp, "TEMP %f", temperature);
    display.resetDisplay();
    display.drawString(0, 0, disp);
    display.display();
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

  display.resetDisplay();
  disp = String("Connected to " + (String)WIFI_SSID);
  display.drawString(0, 0, disp);

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    disp = String("\n Attempting MQTT connection...");
    display.drawString(0, 0, disp);
    // Attempt to connect
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_STATE, 1, true, "disconnected", false)) {
      Serial.println("connected");

      disp = String("\n \n Connected to " + (String)MQTT_SERVER);
      display.drawString(0, 0, disp);

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
