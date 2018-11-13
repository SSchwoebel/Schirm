// Workshop LED Kunst und digitale Emanzipation
// Schritt 1: LED blinken lassen

#include "pins_arduino.h"

// benutzte Pins
const int LED_pin=5;

void setup() {
  delay(3000); // Wartezeit zur Erholung
  
  //setze Output Pin fuer einzelne LED
  pinMode(LED_pin,OUTPUT);
}
  
void loop()
{
  digitalWrite(LED_pin,HIGH);
  delay(1000);
  digitalWrite(LED_pin,LOW);
  delay(1000);
  
}
