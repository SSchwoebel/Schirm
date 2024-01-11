// Erster Test fuer Box von Drizzt und Sarah


#include "FastLED.h"
#include <avr/interrupt.h>
#include "pins_arduino.h"

FASTLED_USING_NAMESPACE          //Makro von FastLED. Muss ausgefuehrt werden, damit man FastLED nutzen kann

// Parameter
const int leds_per_strip = 44;    //Anzahl der LEDs im Streifen
const int delta_hue = 7;              //steuert Breite des Farbverlaufs der RainbowColors
const int brightness = 100;           //zum Einstellen der Helligkeit des LED; Wert zwischen 0 und 255; hoechste Helligkeit nicht ratsam wenn Spannungsversorgung per USB
const int frames_per_second = 100;     //Haeufigkeit mit der LED-Streifen aufgefrischt wird
//const int reactonbeatduration = 50;   //wie lange leuchten nach beat in ms

// benutzte Pins
const int switch_interrupt_pin=2; 
const int LED_stripe_pin=27; //23-29, 31-37, nur ungerade Zahlen, Teststreifen ist auf dem 3. Pin.
//const int poti_1_pin = A0;
//const int poti_1_pin = A1;

// globale Variablen
CRGB leds[leds_per_strip];   //Speicherbereich, der die Farbwerte der LEDs im Leuchtstreifen haelt

int gHue = 0; // rotierende Basisfarbe, die von den Rainbow-Leuchtmustern verwendet wird



void setup() {
  delay(3000); // Wartezeit zur Erholung

  /*
  //setze Output Pin fuer einzelne LED
  pinMode(LED_pin,OUTPUT);
  
  //setze Interrupt Pin fuer Schalter und verbinde mit Trigger Routine
  pinMode(switch_interrupt_pin,INPUT_PULLUP);
  attachInterrupt(switch_interrupt_pin-2,setSwitchTrigger,LOW); 
  */

  // teile FastLED die LED-Streifen-Konfiguration mit
  FastLED.addLeds<WS2812B,LED_stripe_pin,GRB>(leds,leds_per_strip);

  // setze Helligkeit
  FastLED.setBrightness(brightness);
}
  
void loop()
{
  // Rufe die Muster-Funktion einmal auf um das 'leds' Array upzudaten
  RainbowColors();

  // schicke das 'leds' Array heraus an den eigentlichen LED strip
  // aber nur alle 1000/frames_per_second Millisekunden um die Framerate und die Zeit fuer die Ausgabe moderat zu halten
  EVERY_N_MILLISECONDS(1000/frames_per_second){FastLED.show();} 

  // wechsle langsam die Basisfarbe durch den Regenbogen
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
}

//-------------------------------------------------------------------------------------------------------------------
//--- Trigger-Behandlungs-Routinen

/*
void setSwitchTrigger()
{
  digitalWrite(LED_pin,HIGH);
  delay(100);
  digitalWrite(LED_pin,LOW);
}
*/

//-------------------------------------------------------------------------------------------------------------------
//--- Leuchtmusterfunktionen

void RainbowColors() 
{
  fill_rainbow( leds,leds_per_strip, gHue, delta_hue);
}
