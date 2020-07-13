//-------------------------------------------------------------------------------------
// HX711_ADC.h
// Arduino master library for HX711 24-Bit Analog-to-Digital Converter for Weigh Scales
// Olav Kallhovd sept2017
// Tested with      : HX711 asian module on channel A and YZC-133 3kg load cell
// Tested with MCU  : Arduino Nano, ESP8266
//-------------------------------------------------------------------------------------
// This is an example sketch on how to use this library
// Settling time (number of samples) and data filtering can be adjusted in the config.h file

// This example shows how to calibrate the load cell and optionally save the calibration  
// value to EEPROM, and also how to change the value.
// The value can later be fetched from EEPROM in your project sketch.

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include "Adafruit_APDS9960.h"


#define SEALEVELPRESSURE_HPA (1013.25)

//HX711 constructor (dout pin, sck pin):
HX711_ADC LoadCell(7, 6);
Adafruit_BME280 bme; 
Adafruit_APDS9960 apds;

int eepromAdress = 0;

long t;
int val;
unsigned long delayTime;

void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("It is assumed that the mcu was started with no load applied to the load cell.");
  Serial.println("Now, place your known mass on the loadcell,");
  Serial.println("then send the weight of this mass (i.e. 100.0) from serial monitor.");
  float m = 0;
  boolean f = 0;
  while (f == 0) {
    LoadCell.update();
    if (Serial.available() > 0) {
      m = Serial.parseFloat();
      if (m != 0) {
        Serial.print("Known mass is: ");
        Serial.println(m);
        f = 1;
      }
      else {
        Serial.println("Invalid value");
      }
    }
  }
  float c = LoadCell.getData() / m;
  LoadCell.setCalFactor(c);
  Serial.print("Calculated calibration value is: ");
  Serial.print(c);
  Serial.println(", use this in your project sketch");
  f = 0;
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(eepromAdress);
  Serial.println("? y/n");
  while (f == 0) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
        #if defined(ESP8266) 
        EEPROM.begin(512);
        #endif
        EEPROM.put(eepromAdress, c);
        #if defined(ESP8266)
        EEPROM.commit();
        #endif
        EEPROM.get(eepromAdress, c);
        Serial.print("Value ");
        Serial.print(c);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(eepromAdress);
        f = 1;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        f = 1;
      }
    }
  }
  Serial.println("End calibration");
  Serial.println("For manual edit, send 'c' from serial monitor");
  Serial.println("***");
}

void changeSavedCalFactor() {
  float c = LoadCell.getCalFactor();
  boolean f = 0;
  Serial.println("***");
  Serial.print("Current value is: ");
  Serial.println(c);
  Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
  while (f == 0) {
    if (Serial.available() > 0) {
      c = Serial.parseFloat();
      if (c != 0) {
        Serial.print("New calibration value is: ");
        Serial.println(c);
        LoadCell.setCalFactor(c);
        f = 1;
      }
      else {
        Serial.println("Invalid value, exit");
        return;
      }
    }
  }
  f = 0;
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(eepromAdress);
  Serial.println("? y/n");
  while (f == 0) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
        #if defined(ESP8266)
        EEPROM.begin(512);
        #endif
        EEPROM.put(eepromAdress, c);
        #if defined(ESP8266)
        EEPROM.commit();
        #endif
        EEPROM.get(eepromAdress, c);
        Serial.print("Value ");
        Serial.print(c);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(eepromAdress);
        f = 1;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        f = 1;
      }
    }
  }
  Serial.println("End change calibration value");
  Serial.println("***");
}

void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");
  LoadCell.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Tare timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    float c = 0;
    EEPROM.get(eepromAdress, c);
    if (c != 0) {
        LoadCell.setCalFactor(c); // user set calibration value (float)
    } else {
        LoadCell.setCalFactor(1.0); // user set calibration value (float)
    }    
    Serial.println("Startup + tare is complete");
  }
    //LoadCell.setCalFactor(1.0);
  while (!LoadCell.update());
  //calibrate();

  Serial.println(F("BME280 test"));

  if (! bme.begin(0x77, &Wire)) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }

    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
                      
    // suggested rate is 1/60Hz (1m)
    delayTime = 60000; // in milliseconds

  if(!apds.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");

  //enable color sensign mode
  apds.enableColor(true);    
}

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.print(" *C / ");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.print(" hPa / ");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.print(" m / ");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.print(" % / ");

    uint16_t r, g, b, c;
    
    //wait for color data to be ready
    while(!apds.colorDataReady()){
      delay(5);
    }

    //get the data and print the different channels
    apds.getColorData(&r, &g, &b, &c);

    Serial.print("Red: ");
    Serial.print(r);
    
    Serial.print(" / Green: ");
    Serial.print(g);
    
    Serial.print(" / Blue: ");
    Serial.print(b);
    
    Serial.print(" / Clear: ");
    Serial.print(c);

    // Rain
    val = analogRead(A0);  // read the input pin
    Serial.print(" / Rain: (");
    Serial.print(val);
    Serial.print(") ");
    if(val<300) Serial.print("Heavy"); 
    else if(val<500) Serial.print("Moderate"); 
    else Serial.print("None"); 

    val = analogRead(A1);  // read the input pin
    Serial.print(" / Battery: (");
    Serial.print(val);
    Serial.print(") ");
    Serial.print( 5.0 * (float)val  / 1024.0);
    Serial.println("V");
}

void loop() {

  //update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
  //longer delay in sketch will reduce effective sample rate (be carefull with delay() in the loop)
  LoadCell.update();

  //get smoothed value from the data set
  if (millis() > t + 250) {
    float i = LoadCell.getData();
    Serial.print("RainWeight =  ");
    Serial.print(i);
    Serial.print(" g / ");

    // Only needed in forced mode! In normal mode, you can remove the next line.
    bme.takeForcedMeasurement(); // has no effect in normal mode
    printValues();

    t = millis();
  }

  //receive from serial terminal
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
    else if (inByte == 'c') changeSavedCalFactor();
  }

  //check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

}