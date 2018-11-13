// Workshop LED Kunst und digitale Emanzipation
// Schritt 2: LED mit Taster steuern


#include <avr/interrupt.h>
#include "pins_arduino.h"

// benutzte Pins
const int switch_interrupt_pin=2; 
const int LED_pin=5;

void setup() {
  delay(3000); // Wartezeit zur Erholung
  
  //setze Output Pin fuer einzelne LED
  pinMode(LED_pin,OUTPUT);
  
  //setze Interrupt Pin fuer Schalter und verbinde mit Trigger Routine
  pinMode(switch_interrupt_pin,INPUT_PULLUP);
  attachInterrupt(switch_interrupt_pin-2,setSwitchTrigger,LOW); 
}
  
void loop()
{
  delay(100);   //einfach nur warten
}

//-------------------------------------------------------------------------------------------------------------------
//--- Trigger-Behandlungs-Routinen

void setSwitchTrigger()
{
  digitalWrite(LED_pin,HIGH);
  delay(100);
  digitalWrite(LED_pin,LOW);
}

