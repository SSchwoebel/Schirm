// Workshop LED Kunst und digitale Emanzipation
// Schritt 1: LED blinken lassen

#include "pins_arduino.h"
#include <Wire.h>
#include <VL53L1X.h>


// benutzte Pins
const int LED_pin=5;

//variables


//time of flight sensor
VL53L1X sensor;

void setup() {
  delay(3000); // Wartezeit zur Erholung
  
  //setze Output Pin fuer einzelne LED
  pinMode(LED_pin,OUTPUT);

  //setup TOF sensor
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  sensor.setTimeout(500);
  sensor.init();

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  
  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);

  
  //serial output
  //Serial.begin(9600);
}
  
void loop()
{
  sensor.read();
    
  digitalWrite(LED_pin,HIGH);
  delay(sensor.ranging_data.range_mm*0.3);
  digitalWrite(LED_pin,LOW);
  delay(sensor.ranging_data.range_mm*0.3);
  //Serial.print(duration);
  //Serial.println();
  
}
