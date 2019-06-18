// Workshop LED Kunst und digitale Emanzipation
// Schritt 1: LED blinken lassen

#include "pins_arduino.h"

// benutzte Pins
const int LED_pin=5;
const int ECHO_pin=8;
const int ECHO_TRIGGER_pin=9;

//variables
long duration;

void setup() {
  delay(3000); // Wartezeit zur Erholung
  
  //setze Output Pin fuer einzelne LED
  pinMode(LED_pin,OUTPUT);
  pinMode(ECHO_TRIGGER_pin,OUTPUT);
  pinMode(ECHO_pin,INPUT);
  
  //serial output
  //Serial.begin(9600);
}
  
void loop()
{
  //Triggersignal senden
  digitalWrite(ECHO_TRIGGER_pin,LOW);
  delayMicroseconds(5);
  digitalWrite(ECHO_TRIGGER_pin,HIGH);
  delayMicroseconds(10);
  digitalWrite(ECHO_TRIGGER_pin,LOW);
  
  //read echo signal duration
  pinMode(ECHO_pin,INPUT);
  duration = pulseIn(ECHO_pin,HIGH);
    
  digitalWrite(LED_pin,HIGH);
  delay(duration*0.1);
  digitalWrite(LED_pin,LOW);
  delay(duration*0.1);
  //Serial.print(duration);
  //Serial.println();
  
}
