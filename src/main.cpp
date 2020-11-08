// Include files
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <InfluxDbClient.h>
#include <Arduino.h>
// Personal credentials, not included in GitHUB
#include <credentials.h>

// Device selection
#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266_Garage"
#endif

// Define Block
#define SEALEVELPRESSURE_HPA (1013.25)


// InfluxDB client instance for InfluxDB 1
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);

// Data point definition
Point myWifi("wifi_status");
Point myTemp("temperature");
Point myHumRel("humidity_rel");
Point myHumAbs("humidity_abs");
Point myDewPoint("dewpoint");
Point myPressRel("pressure_rel");
Point myPressAbs("pressure_abs");

// BME Instance
Adafruit_BME280 bme; //I2C

unsigned long delayTime;

void setup() {
  // Start serial console
  Serial.begin(115200);
  unsigned status;

  // Connect to WiFi
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  
  // Start BME280 communication
  status = bme.begin(0x76);

  if (!status){ 
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);    
  }

  // Set InfluxDB 1 authentication params
  client.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);

  // Add constant tags - only once
  myWifi.addTag("device", DEVICE);
  myWifi.addTag("SSID", WiFi.SSID());
  myTemp.addTag("device", DEVICE);
  myDewPoint.addTag("device", DEVICE);
  myHumRel.addTag("device", DEVICE);
  myHumAbs.addTag("device", DEVICE);
  myPressRel.addTag("device", DEVICE);
  myPressAbs.addTag("device", DEVICE);
  
  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
}

void loop() {
  // Store measured value into point
  myWifi.clearFields();
  myTemp.clearFields();
  myDewPoint.clearFields();
  myHumAbs.clearFields();
  myHumRel.clearFields();
  myPressAbs.clearFields();
  myPressRel.clearFields();

  // Gather infos from BME280
  
  myTemp.addField("temperature", bme.readTemperature());
  myPressRel.addField("pressure_rel", bme.readPressure() / 100.0F);
  myHumRel.addField("humidity_rel", bme.readHumidity());

  // Gather infos from S0 Bus


  // Report RSSI of currently connected network
  myWifi.addField("rssi", WiFi.RSSI());
  
  // Print what are we exactly writing
  Serial.print("Writing: ");
  Serial.println(myWifi.toLineProtocol());
  Serial.print("Writing: ");
  Serial.println(myTemp.toLineProtocol());
  Serial.print("Writing: ");
  Serial.println(myDewPoint.toLineProtocol());
  Serial.print("Writing: ");
  Serial.println(myHumAbs.toLineProtocol());
  Serial.print("Writing: ");
  Serial.println(myHumRel.toLineProtocol());
  Serial.print("Writing: ");
  Serial.println(myPressRel.toLineProtocol());
  Serial.print("Writing: ");
  Serial.println(myPressAbs.toLineProtocol());
  
  // If no Wifi signal, try to reconnect it
  if ((WiFi.RSSI() == 0) && (wifiMulti.run() != WL_CONNECTED))
    Serial.println("Wifi connection lost");
  // Write point
  if (!client.writePoint(myWifi)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  //Wait 60s
  Serial.println("Wait 60s");
  delay(60000);
}