// Workshop LED Kunst und digitale Emanzipation
// Last Step: Verschiedene Muster mit Mikro und Taster

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
const int switch_deadtime = 500;    //Totzeit des Tasters in ms. Um Prellen zu verhindern.
const int barduration = 250; //wie lange der Balken im Muster "RainbowColors_bars" braucht um zu "schrumpfen", in ms

// benutzte Pins
const int switch_interrupt_pin=2; 
const int knock_interrupt_pin=3;
const int LED_pin=5;
const int LED_stripe_pin=7;


// globale Variablen
CRGB leds[leds_per_strip];   //Speicherbereich, der die Farbwerte der LEDs im Leuchtstreifen haelt

int gHue = 0; // rotierende Basisfarbe, die von den Rainbow-Leuchtmustern verwendet wird
int gCurrentPatternNumber = 0; // Index Nummber des aktuellen Leuchtmusters
int on = 0;    // wird benutzt um den Zustand eines Musters als "on" zu setzen waehrend der "reactonbeatduration" nach dem Triggern
int switchPressed=0;  // wird benutzt um den Zustand des Schalters als "pressed" zu setzen waehrend der "switch_deadtime" nach dem Triggern
int starttime;  // wird gesetzt sobald Schalter gedrueckt wurde und benutzt um eine Totzeit einzuhalten

volatile int knockTrigger=0;  //Globaler Trigger Wert, wird von Interrupt-Funktion "setKnockTrigger" genutzt um Trigger an loop weiterzugeben. Muss dazu als "volatile" definiert werden.
volatile int switchTrigger=0;  //Globaler Trigger Wert, wird von Interrupt-Funktion "setSwitchTrigger" genutzt um Trigger an loop weiterzugeben. Muss dazu als "volatile" definiert werden.




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

// Liste der Leuchtmuster durch die gewechselt wird. Jedes ist weiter unten als separate Funktion definiert.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {RainbowColors_react, White_react, RainbowColors_bars};
const int number_of_patterns = 3;

void loop()
{
  // Rufe die aktuelle Muster-Funktion einmal auf um das 'leds' Array upzudaten
  gPatterns[gCurrentPatternNumber]();

  // schicke das 'leds' Array heraus an den eigentlichen LED strip
  // aber nur alle 1000/frames_per_second Millisekunden um die Framerate und die Zeit fuer die Ausgabe moderat zu halten
  EVERY_N_MILLISECONDS(1000/frames_per_second){FastLED.show();} 

  // wechsle langsam die Basisfarbe durch den Regenbogen
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  
  //auf Taster reagieren (naechstes Muster) aber nur ausserhalb der Totzeit
  if (switchTrigger && switchPressed==0){
    starttime=millis();
    gCurrentPatternNumber = (gCurrentPatternNumber + 1) % number_of_patterns;
    switchTrigger = 0;
    switchPressed = 1;
  }
  else if (switchTrigger && switchPressed==1){    //vermeide wiederholtes Schalten bei langem druecken
    starttime=millis();
    switchTrigger = 0;
    switchPressed = 1;
  }
  else if((millis()-starttime>switch_deadtime) && switchPressed==1){
    switchPressed = 0;
    switchTrigger = 0;
  }  
}

//-------------------------------------------------------------------------------------------------------------------
//--- Trigger-Behandlungs-Routinen

void setKnockTrigger() {
  knockTrigger=1;
}

void setSwitchTrigger()
{
  switchTrigger=1;
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

void White_react() 
{
  static unsigned long starttime;
  if (knockTrigger && on==0){
    starttime=millis();
    FastLED.setBrightness(int(0.3*brightness));
    knockTrigger = 0;
    on = 1;
  }
  else if((millis()-starttime>reactonbeatduration) && on){
    FastLED.setBrightness(int(0*brightness));
    on = 0;
    knockTrigger=0;
  }
  else if(!on){
    FastLED.setBrightness(int(0*brightness));
  }
    
  
  // setze alle LEDs im Streifen auf Weiss
  for(int i = 0; i < leds_per_strip; i++) {
    leds[i] = CRGB::White;
  }
}

void RainbowColors_bars() 
{
  static unsigned long starttime;
  int timenow=millis();
  int n;
  
  FastLED.setBrightness(int(brightness));
  
  if (knockTrigger && on==0){    
    // streifen mit Regenbogenfarben fuellen
    fill_rainbow( leds,leds_per_strip, gHue, delta_hue);
    starttime=timenow;
    knockTrigger = 0;
    on = 1;
  }
  else if((timenow-starttime<barduration)){
    n = int(float(timenow-starttime)/float(barduration)*float(leds_per_strip));
    // streifen mit Regenbogenfarben fuellen
    fill_rainbow( leds,leds_per_strip, gHue, delta_hue);
    // von einer Seite dann mit Schwarz ueberschreiben bis n
    for(int i=0; i<n; i++){
      leds[i] = CRGB::Black;
    }
  }
  else if(on==0){
    // alle LEDs ím Streifen schwarz setzen
    for(int i=0; i<leds_per_strip; i++){
      leds[i] = CRGB::Black;
    }
  }
  for(int i=0; i< leds_per_strip;i++){
      leds[i].nscale8_video( 85);                         //ganz boeser hack von sarah, tut nichts, entfernt ruckeln, anstatt brightness zu setzen
  }
  if((timenow-starttime>reactonbeatduration) && on){
    on = 0;
    knockTrigger=0;
  }
}
