/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example runs directly on ESP32 chip.

  Note: This requires ESP32 support package:
    https://github.com/espressif/arduino-esp32

  Please be sure to select the right ESP32 module
  in the Tools -> Board menu!

  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
#define BLYNK_TEMPLATE_ID "TMPL7CRzPAnI"


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "bsec.h"
#include <XNODE.h>
// Create an object of the class Bsec
Bsec iaqSensor;
// Create an object of the class XNODE
XNODE xnode(&Serial2);
// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);

#define LED_BUILTIN 12

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "W6Ufc6G373rjnifLFkpxhNkojjAcT_Mb";


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Microside_2G";
char pass[] = "MicrosidE@2019";
int period = 1000;
unsigned long time_now = 0;
String output;


void setup()
{
  // Debug console
  Serial.begin(115200);
  Serial2.begin(115200);
  Wire.begin();
  delay(5000);
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
  
  Blynk.begin(auth, ssid, pass);
}

void loop()
{
  if((unsigned long)(millis() - time_now) > period){
    time_now = millis();
    readBME688();
  }
  Blynk.run();
}

void indicadorBME688(float _iaq)
{
    float IAQ = _iaq;
    if( IAQ >= 0 && IAQ <= 50)
    {
      xnode.SendCommandWithRange("007A","1",0,255,0);
    }
    if( IAQ >= 50.1 && IAQ <= 100)
    {
      xnode.SendCommandWithRange("007A","1",255,255,0);
    }
    if( IAQ >= 100.1 && IAQ <= 150)
    {
      xnode.SendCommandWithRange("007A","1",255,128,0);
    }

    if( IAQ >= 150.1 && IAQ <= 200)
    {
      xnode.SendCommandWithRange("007A","1",255,0,0);
    }

    if( IAQ >= 200.1 && IAQ <= 300)
    {
      xnode.SendCommandWithRange("007A","1",87,35,100);
    }

    if( IAQ >= 300.1 && IAQ <= 500)
    {
      xnode.SendCommandWithRange("007A","1",0,0,0);
    }
}

void readBME688()
{
  if(iaqSensor.run()){
    Blynk.virtualWrite(V0,String(iaqSensor.rawTemperature));
    Blynk.virtualWrite(V1,String(iaqSensor.pressure));
    Blynk.virtualWrite(V2,String(iaqSensor.rawHumidity));
    Blynk.virtualWrite(V3,String(iaqSensor.gasResistance));
    Blynk.virtualWrite(V4,String(iaqSensor.iaq));
    Blynk.virtualWrite(V5,String(iaqSensor.iaqAccuracy));
    Blynk.virtualWrite(V6,String(iaqSensor.co2Equivalent));
    Blynk.virtualWrite(V7,String(iaqSensor.breathVocEquivalent));
    Blynk.virtualWrite(V8,String(iaqSensor.temperature));    
    Blynk.virtualWrite(V9,String(iaqSensor.humidity));
    indicadorBME688(float(iaqSensor.iaq));
  }
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}
