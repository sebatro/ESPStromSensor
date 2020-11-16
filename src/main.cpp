/*
* 
*/

// Include files
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <math.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

// Personal credentials, not included in GitHUB
#include <credentials.h>

// Definition of sensor variant, must be unique for each sensor
#define DEVICE "GARTEN"
#define MQTT_SERVER      "192.168.1.85"
#define MQTT_SERVERPORT  1883                   // use 8883 for SSL
#define MQTT_USERNAME    "espsensor"
#define MQTT_KEY         "bkst81"

// Define Block
#define SEALEVELPRESSURE_HPA (1013.25)
#define ALTITUDE 324 //Altitude of your location (m above sea level)

// create instance of ESP8266 multiwifi
WiFiClient wifi;
 
// BME Instance
Adafruit_BME280 bme; //I2C

// NTP 
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0);

// MQTT client
PubSubClient mqtt(wifi);
#define MSG_BUFFER_SIZE	(50)
char msgTemp[MSG_BUFFER_SIZE];
char msgHumiR[MSG_BUFFER_SIZE];
char msgHumiA[MSG_BUFFER_SIZE];
char msgPressR[MSG_BUFFER_SIZE];
char msgPressA[MSG_BUFFER_SIZE];
char msgDewp[MSG_BUFFER_SIZE];
char msgRSSI[MSG_BUFFER_SIZE];
char msgRecon[MSG_BUFFER_SIZE];


char topicTemp[MSG_BUFFER_SIZE];
char topicHumiR[MSG_BUFFER_SIZE];
char topicHumiA[MSG_BUFFER_SIZE];
char topicPressR[MSG_BUFFER_SIZE];
char topicPressA[MSG_BUFFER_SIZE];
char topicDewp[MSG_BUFFER_SIZE];
char topicRecon[MSG_BUFFER_SIZE];
char topicRSSI[MSG_BUFFER_SIZE];

const float cToKOffset = 273.15F;
unsigned long delayTime = 10UL;
unsigned long lastRun = 0UL;

float absoluteHumidity(float temperature, float humidity);
float saturationVaporPressure(float temperature);
float dewPoint(float temperature, float humidity);

void MQTT_reconnect();

void setup() {
  // Start serial console
  Serial.begin(115200);
  unsigned status;

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  
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

  // start MQTT connecttio
  mqtt.setServer(MQTT_SERVER, MQTT_SERVERPORT);

  // start Timeclient
  timeClient.begin();
}

void loop() {
  
  // aktuelle Zeit holen
  timeClient.update();

  if (!mqtt.connected()) {
    MQTT_reconnect();
  }
  mqtt.loop();

  if (timeClient.getEpochTime() > (lastRun + delayTime)) {
    lastRun = timeClient.getEpochTime();
    // Print current unitime
    Serial.println(lastRun);
    Serial.println("---------------------------------");
    // Store measured value into point

    // Gather infos from BME280
    float temperature = bme.readTemperature();
    float humidity_r = bme.readHumidity();
    float humidity = absoluteHumidity(temperature, humidity_r);
    float pressure = bme.readPressure() / 100.0F;
    float pressure_r = bme.seaLevelForAltitude(ALTITUDE, pressure);
    float dew = dewPoint(temperature, humidity_r);
    
    // Gather infos from S0 Bus @sebatro 

    // Gather infos from RS485 solar inverter
      
    // convert to char
    snprintf (msgTemp, MSG_BUFFER_SIZE, "%4.2f", temperature);
    snprintf (msgHumiA, MSG_BUFFER_SIZE, "%4.2f", humidity);
    snprintf (msgHumiR, MSG_BUFFER_SIZE, "%4.2f", humidity_r);
    snprintf (msgPressR, MSG_BUFFER_SIZE, "%4.2f", pressure_r);
    snprintf (msgPressA, MSG_BUFFER_SIZE, "%4.2f", pressure);
    snprintf (msgDewp, MSG_BUFFER_SIZE, "%4.2f", dew);
    snprintf (msgRSSI, MSG_BUFFER_SIZE, "%i", WiFi.RSSI());

    snprintf (topicTemp, MSG_BUFFER_SIZE, "/SENSOR/%s/TEMPERATUR", DEVICE);
    snprintf (topicHumiA, MSG_BUFFER_SIZE, "/SENSOR/%s/LUFTFEUCHTE_ABS", DEVICE);
    snprintf (topicHumiR, MSG_BUFFER_SIZE, "/SENSOR/%s/LUFTFEUCHTE_REL", DEVICE);
    snprintf (topicPressR, MSG_BUFFER_SIZE, "/SENSOR/%s/LUFTDRUCK_REL", DEVICE);
    snprintf (topicPressA, MSG_BUFFER_SIZE, "/SENSOR/%s/LUFTDRUCK_ABS", DEVICE);
    snprintf (topicDewp, MSG_BUFFER_SIZE, "/SENSOR/%s/TAUPUNKT", DEVICE);
    snprintf (topicRSSI,MSG_BUFFER_SIZE, "/SENSOR/%s/RSSI", DEVICE);

    // Publish to MQTT
    mqtt.publish(topicTemp, msgTemp, true);
    mqtt.publish(topicHumiA, msgHumiA, true);
    mqtt.publish(topicHumiR, msgHumiR, true);
    mqtt.publish(topicPressR, msgPressR, true);
    mqtt.publish(topicPressA, msgPressA, true);
    mqtt.publish(topicDewp, msgDewp, true);
    mqtt.publish(topicRSSI, msgRSSI, true);
    
    // Print what are we exactly writing
  
    // If no Wifi signal, try to reconnect it
  }     
}

void MQTT_reconnect() {
  // max 20 reconnects then reboot
  int counter = 0;
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    counter++;
    if (counter >= 20) {
      system_upgrade_reboot();
    }
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = DEVICE;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt.connect(clientId.c_str(),MQTT_USERNAME, MQTT_KEY)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      timeClient.update();
      snprintf (topicRecon, MSG_BUFFER_SIZE, "/SENSOR/%s/RECONNECT", DEVICE);
      snprintf (msgRecon, MSG_BUFFER_SIZE, "Reconnected at: %lu", timeClient.getEpochTime());
      mqtt.publish(topicRecon, msgRecon, true);
      // ... and resubscribe
      mqtt.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


// Relative to absolute humidity
// Based on https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
float absoluteHumidity(float temperature, float humidity) {
  return (13.2471*pow(EULER,17.67*temperature/(temperature+243.5))*humidity/(cToKOffset+temperature));
}

// Calculate saturation vapor pressure
// Based on dew.js, Copyright 2011 Wolfgang Kuehn, Apache License 2.0
float saturationVaporPressure(float temperature) {
  if(temperature < 173 || temperature > 678) return -112; //Temperature out of range

  float svp = 0;
  if(temperature <= cToKOffset) {
    /**
      * -100-0°C -> Saturation vapor pressure over ice
      * ITS-90 Formulations by Bob Hardy published in 
      * "The Proceedings of the Third International 
      * Symposium on Humidity & Moisture",
      * Teddington, London, England, April 1998
      */

    svp = exp(-5.8666426e3/temperature + 2.232870244e1 + (1.39387003e-2 + (-3.4262402e-5 + (2.7040955e-8*temperature)) * temperature) * temperature + 6.7063522e-1 * log(temperature)); 
  }else{
    /**
      * 0°C-400°C -> Saturation vapor pressure over water
      * IAPWS Industrial Formulation 1997
      * for the Thermodynamic Properties of Water and Steam
      * by IAPWS (International Association for the Properties
      * of Water and Steam), Erlangen, Germany, September 1997.
      * Equation 30 in Section 8.1 "The Saturation-Pressure 
      * Equation (Basic Equation)"
      */

    const float th = temperature + -0.23855557567849 / (temperature - 0.65017534844798e3);
    const float a  = (th + 0.11670521452767e4) * th + -0.72421316703206e6;
    const float b  = (-0.17073846940092e2 * th + 0.12020824702470e5) * th + -0.32325550322333e7;
    const float c  = (0.14915108613530e2 * th + -0.48232657361591e4) * th + 0.40511340542057e6;

    svp = 2 * c / (-b + sqrt(b * b - 4 * a * c));
    svp *= svp;
    svp *= svp;
    svp *= 1e6;
  }
  
  yield();

  return svp;
}


// Calculate dew point in °C
// Based on dew.js, Copyright 2011 Wolfgang Kuehn, Apache License 2.0
float dewPoint(float temperature, float humidity)
{
  temperature += cToKOffset; //Celsius to Kelvin

  if(humidity < 0 || humidity > 100) return -111; //Invalid humidity
  if(temperature < 173 || temperature > 678) return -112; //Temperature out of range

  humidity = humidity / 100 * saturationVaporPressure(temperature);
  
  byte mc = 10;

  float xNew;
  float dx;
  float z;

  do {
    dx = temperature / 1000;
    z = saturationVaporPressure(temperature);
    xNew = temperature + dx * (humidity - z) / (saturationVaporPressure(temperature + dx) - z);
    if (abs((xNew - temperature) / xNew) < 0.0001) {
        return xNew - cToKOffset;
    }
    temperature = xNew;
    mc--;
  } while(mc > 0);

  return -113; //Solver did not get a close result
}
