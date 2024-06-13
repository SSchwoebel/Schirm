// Workshop LED Kunst und digitale Emanzipation
// Schritt 3: RGB LED Streifen mit Muster


#include "FastLED.h"
#include <avr/interrupt.h>
#include "pins_arduino.h"

FASTLED_USING_NAMESPACE          //Makro von FastLED. Muss ausgefuehrt werden, damit man FastLED nutzen kann

// Parameter
const int leds_per_strip = 127;    //Anzahl der LEDs im Streifen
const int delta_hue = 7;              //steuert Breite des Farbverlaufs der RainbowColors
const int brightness = 25;           //zum Einstellen der Helligkeit des LED; Wert zwischen 0 und 255; hoechste Helligkeit nicht ratsam wenn Spannungsversorgung per USB
const int frames_per_second = 2;     //Haeufigkeit mit der LED-Streifen aufgefrischt wird
const int reactonbeatduration = 50;   //wie lange leuchten nach beat in ms

// benutzte Pins
const int switch_interrupt_pin=2; 
const int LED_pin=5;

// globale Variablen
CRGB leds[leds_per_strip];   //Speicherbereich, der die Farbwerte der LEDs im Leuchtstreifen haelt

int gHue = 0; // rotierende Basisfarbe, die von den Rainbow-Leuchtmustern verwendet wird

static int trigger=0;

void setup() {
  delay(3000); // Wartezeit zur Erholung
  
  //setze Interrupt Pin fuer Schalter und verbinde mit Trigger Routine
  pinMode(switch_interrupt_pin,INPUT_PULLUP);
  attachInterrupt(switch_interrupt_pin-2,interrupt_function,LOW); 

  // teile FastLED die LED-Streifen-Konfiguration mit
  FastLED.addLeds<WS2812B,LED_pin,GRB>(leds,leds_per_strip);

  // setze Helligkeit
  FastLED.setBrightness(brightness);
}
  
void loop()
{
  if(trigger==1){
    dragFunction();
  }
  else {
    CoalColors(leds_per_strip);
  }

  // schicke das 'leds' Array heraus an den eigentlichen LED strip
  // aber nur alle 1000/frames_per_second Millisekunden um die Framerate und die Zeit fuer die Ausgabe moderat zu halten
  EVERY_N_MILLISECONDS(1000/frames_per_second){
    // Rufe die Muster-Funktion einmal auf um das 'leds' Array upzudaten
    FastLED.show();
  } 

  // wechsle langsam die Basisfarbe durch den Regenbogen
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
}

//-------------------------------------------------------------------------------------------------------------------
//--- Trigger-Behandlungs-Routinen

void interrupt_function()
{
  trigger = 1;
}

void dragFunction()
{
  uint16_t dragBrightness = 100;

  int dragDelay = 25;

  FastLED.setBrightness(dragBrightness);

  FastLED.show();

  delay(dragDelay);

  for(int i=0; i<leds_per_strip; i++)
  {
    //int i = leds_per_strip - j - 1;
    CoalColors(i);
    
    FastLED.show();

    delay(dragDelay);
  }

  for(int i=0; i<50; i++){
    CoalColors(leds_per_strip);
    FastLED.show();

    delay(dragDelay);
  }

  FastLED.setBrightness(brightness);
  
  trigger = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//--- Leuchtmusterfunktionen

void CoalColors(int n) 
{
  for(int i=0; i<leds_per_strip; i++){
    if(i<n){
      leds[i] = CRGB::DarkRed;
    }
    else{
      leds[i] = CRGB::Black;
    }
  }
      
  addGlitter(n);
  addFlame(n);
}

void addGlitter(int n) 
{
  int nGlitter = random(2,n/8);
  for(int i=0; i<nGlitter; i++){
    leds[ random16(n) ] = CRGB::Olive;
  }
}

void addFlame(int n) 
{
  int nGlitter = random(2,n/8);
  for(int i=0; i<nGlitter; i++){
    leds[ random16(n) ] = CRGB::Plaid;
  }
}
