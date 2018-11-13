// Workshop LED Kunst und digitale Emanzipation
// Schritt 5: Musterblinken mit dem Mikro steuern


#include "FastLED.h"
#include <avr/interrupt.h>
#include "pins_arduino.h"

FASTLED_USING_NAMESPACE          //Makro von FastLED. Muss ausgefuehrt werden, damit man FastLED nutzen kann

// Parameter
const int leds_per_strip = 10;    //Anzahl der LEDs im Streifen
const int delta_hue = 7;              //steuert Breite des Farbverlaufs der RainbowColors
const int brightness = 100;           //zum Einstellen der Helligkeit des LED; Wert zwischen 0 und 255; hoechste Helligkeit nicht ratsam wenn Spannungsversorgung per USB
const int frames_per_second = 100;     //Haeufigkeit mit der LED-Streifen aufgefrischt wird
const int reactonbeatduration = 50;   //wie lange leuchten nach beat in ms

// benutzte Pins
const int switch_interrupt_pin=2; 
const int knock_interrupt_pin=3;
const int LED_pin=5;
const int LED_stripe_pin=7;


// globale Variablen
CRGB leds[leds_per_strip];   //Speicherbereich, der die Farbwerte der LEDs im Leuchtstreifen haelt

int gHue = 0; // rotierende Basisfarbe, die von den Rainbow-Leuchtmustern verwendet wird
int on = 0;    // wird benutzt um den Zustand eines Musters als "on" zu setzen waehrend der "reactonbeatduration" nach dem Triggern

volatile int knockTrigger=0;  //Globaler Trigger Wert, wird von Interrupt-Funktion "setKnockTrigger" genutzt um Trigger an loop weiterzugeben. Muss dazu als "volatile" definiert werden.


void setup() {
  delay(3000); // Wartezeit zur Erholung
  
  //setze Output Pin fuer einzelne LED
  pinMode(LED_pin,OUTPUT);
  
  //setze Interrupt Pin fuer Schalter und verbinde mit Trigger Routine
  pinMode(switch_interrupt_pin,INPUT_PULLUP);
  attachInterrupt(switch_interrupt_pin-2,setSwitchTrigger,LOW); 
  
  //setze Interrupt Pin fuer Klopfsensor und verbinde mit Trigger Routine
  pinMode(knock_interrupt_pin,INPUT);
  attachInterrupt(knock_interrupt_pin-2,setKnockTrigger,HIGH);
  
  // teile FastLED die LED-Streifen-Konfiguration mit
  FastLED.addLeds<WS2812B,LED_stripe_pin,GRB>(leds,leds_per_strip);

  // setze Helligkeit
  FastLED.setBrightness(brightness);
}
  
void loop()
{
  // Rufe die Muster-Funktion einmal auf um das 'leds' Array upzudaten
  RainbowColors_react();

  // schicke das 'leds' Array heraus an den eigentlichen LED strip
  // aber nur alle 1000/frames_per_second Millisekunden um die Framerate und die Zeit fuer die Ausgabe moderat zu halten
  EVERY_N_MILLISECONDS(1000/frames_per_second){FastLED.show();} 

  // wechsle langsam die Basisfarbe durch den Regenbogen
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
}

//-------------------------------------------------------------------------------------------------------------------
//--- Trigger-Behandlungs-Routinen

void setKnockTrigger() {
  knockTrigger=1;
}

void setSwitchTrigger()
{
  digitalWrite(LED_pin,HIGH);
  delay(100);
  digitalWrite(LED_pin,LOW);
}

//-------------------------------------------------------------------------------------------------------------------
//--- Leuchtmusterfunktionen

void RainbowColors_react() 
{
  static unsigned long starttime;
  if (knockTrigger && on==0){
    starttime=millis();
    FastLED.setBrightness(int(1*brightness));
    knockTrigger = 0;
    on = 1;
  }
  else if((millis()-starttime>reactonbeatduration) && on){
    FastLED.setBrightness(int(0.3*brightness));
    on = 0;
    knockTrigger=0;
  }
  else if(!on){
    FastLED.setBrightness(int(0.3*brightness));
  }
  // streifen mit Regenbogenfarben fuellen
  fill_rainbow( leds,leds_per_strip, gHue, delta_hue);
}
