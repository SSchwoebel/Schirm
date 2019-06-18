#include "pins_arduino.h"

// benutzte Pins
const int LED_pin=5;
const int FLAME_pin=A0;

//variables
long flame_val;

void setup() {
  delay(3000); // Wartezeit zur Erholung
  
  //setze input pins
  pinMode(LED_pin,OUTPUT);
  
}
  
void loop()
{
  //read flame sensor voltage
  flame_val= analogRead(FLAME_pin);
  
  Serial.println(flame_val);
  if (flame_val >0)
  {
    digitalWrite(LED_pin,HIGH);
  }
  else
  {
    digitalWrite(LED_pin,LOW);
  }
  
}
