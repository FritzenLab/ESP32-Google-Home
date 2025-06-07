#include <Arduino.h>
#include <WiFi.h>
#include "SinricPro.h"
#include "SinricProSwitch.h"
#include "SinricProTemperaturesensor.h"

#define WIFI_SSID         ""    
#define WIFI_PASS         ""
#define APP_KEY           ""      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        ""   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"

#define device_ID   ""
#define TEMP_SENSOR_ID    ""   // Should look like "5dc1564130xxxxxxxxxxxxxx" (Get it from Portal -> Devices)

#define RELAY_PIN    5 
#define SWITCH_PIN   6
#define LM_PIN    A0

#define wifiLed   8   
 
#define BAUD_RATE    9600
#define DEBOUNCE_TIME 250

float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)

#define ADC_VREF_mV    3300.0 // 5000 is the voltage provided by MCU. If you connect to 3V change to 3000
#define ADC_RESOLUTION 4096.0
#define EVENT_WAIT_TIME   20000 

bool lastSwitchState = false;
unsigned long lastSwitchChange = 0;

bool relayState = false;

bool onSwitchPowerState(const String& deviceId, bool &state) {
  relayState = state;
  digitalWrite(RELAY_PIN, relayState);
  return true;
}

bool onSensorPowerState(const String &deviceId, bool &state) {
  return true; // request handled properly
}

float getTemperature() {
  #if defined(ESP8266)
    int analogValue = analogRead(LM_PIN);
    float millivolts = (analogValue / 1024.0) * ADC_VREF_mV; 
    float temperature = millivolts / 10;
    // float fahrenheit = ((temperature * 9) / 5 + 32);
    return temperature;
  #elif defined(ESP32)
    int adcVal = analogRead(LM_PIN);
    float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
    float temperature = milliVolt / 10;
    return temperature;
  #endif
}
void handleTemperaturesensor() {
  if (SinricPro.isConnected() == false) {
    Serial.printf("Not connected to Sinric Pro...!\r\n");
    return; 
  }

  static unsigned long last_millis;
  unsigned long        current_millis = millis();
  if (last_millis && current_millis - last_millis < EVENT_WAIT_TIME) return;
  last_millis = current_millis;
  
  float temperature = getTemperature();

  Serial.printf("Temperature: %2.1f °C\r\n", temperature);

  if (isnan(temperature)) { // reading failed... 
    Serial.printf("reading failed!\r\n");  // print error message
    return;                                    // try again next time
  } 

  if (temperature == lastTemperature) {
    Serial.printf("Temperature did not changed. do nothing...!\r\n");
    return; 
  }

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];  // get temperaturesensor device
  bool success = mySensor.sendTemperatureEvent(temperature, -1); // send event
  if (success) {  
    Serial.printf("Sent!\r\n");
  } else {
    Serial.printf("Something went wrong...could not send Event to server!\r\n"); // Enable ENABLE_DEBUG to see why
  }

  lastTemperature = temperature;  // save actual temperature for next compare  
}

void handleSwitch() {
  unsigned long currentMillis = millis();
  bool switchState = digitalRead(SWITCH_PIN);

  if (switchState != lastSwitchState) {
    if (currentMillis - lastSwitchChange > DEBOUNCE_TIME) {
      if (switchState) {
        relayState = !relayState;  // Toggle the relay state
        digitalWrite(RELAY_PIN, relayState);
        SinricProSwitch &mySwitch = SinricPro[device_ID];
        mySwitch.sendPowerStateEvent(relayState);
      }
      lastSwitchChange = currentMillis;
    }
    lastSwitchState = switchState;
  }
}

void setupWiFi()
{
  Serial.printf("\r\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.printf(".");
    delay(250);
  }
  digitalWrite(wifiLed, HIGH);
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %s\r\n", WiFi.localIP().toString().c_str());
}

void setupSinricPro() {
  SinricProSwitch& mySwitch = SinricPro[device_ID];
  mySwitch.onPowerState(onSwitchPowerState);
  
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  //mySensor.onPowerState(onSensorPowerState);

  SinricPro.begin(APP_KEY, APP_SECRET);
  SinricPro.restoreDeviceStates(false);
}

void setupRelay() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN,HIGH); // Initialize relay in the OFF state
}

void setupSwitch() {
  pinMode(SWITCH_PIN, INPUT);
}


void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, LOW);

  setupRelay();
  setupSwitch();
  setupWiFi();
  setupSinricPro();  
}
 

void loop() {
  SinricPro.handle();
  handleSwitch();
  handleTemperaturesensor();
}
