// Workshop LED Kunst und digitale Emanzipation
// Last Step

#include "FastLED.h"
#include <avr/interrupt.h>
#include "pins_arduino.h"

FASTLED_USING_NAMESPACE          //Makro von FastLED. Muss ausgefuehrt werden, damit man FastLED nutzen kann

#define NUM_LEDS_PER_STRIP 10    //Anzahl der LEDs im Streifen
#define DELTA_HUE 7              //steuert Breite des Farbverlaufs der RainbowColors
#define BRIGHTNESS 200           //zum Einstellen der Helligkeit des LED (Maximalwert bei Mustern mit wechselnder Helligkeit
#define FRAMES_PER_SECOND  100
#define REACTONBEATDURATION 50   //wie lange leuchten nach beat in ms
#define SWITCH_DEADTIME 500      //Totzeit des Tasters in ms. Um Prellen zu verhindern.

CRGB leds[NUM_LEDS_PER_STRIP];

const uint8_t BRIGHTNESS;
const uint8_t gHue = 0; // rotating "base color" used by many of the patterns
const uint8_t barduration = 250; //persistence time of the bar in RainbowColors_bars

const int switch_interrupt_pin=2; 
const int knock_interrupt_pin=3;
const int LED_pin=5;
const int LED_stipe_pin=7;


uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t on = 0;    //used to set state of a pattern as "on" during REACTONBEATDURATION
uint8_t switchPressed=0;  //used to set state of trigger as pressed during deadtime

volatile uint8_t knockTrigger=0;  //Globaler Trigger Wert, wird von Interrupt-Funktion "setKnockTrigger" genutzt um Trigger an loop weiterzugeben. Muss dazu als "volatile" definiert werden.
volatile uint8_t switchTrigger=0;  //Globaler Trigger Wert, wird von Interrupt-Funktion "setSwitchTrigger" genutzt um Trigger an loop weiterzugeben. Muss dazu als "volatile" definiert werden.

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {RainbowColors_react, White_react, RainbowColors_bars_fast};
const int number_of_patterns = 3;


void setup() {
  delay(3000); // 3 second delay for recovery
  

  // tell FastLED about the LED strip configuration
  // pin seems to have to be calculated at compile time.....
  FastLED.addLeds<WS2812B,LED_stripe_pin,GRB>(leds,NUM_LEDS_PER_STRIP);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
    
  //set interrupt pin for Knock Sensor
  pinMode(knock_interrupt_pin,INPUT);
  attachInterrupt(knock_interrupt_pin-2,setKnockTrigger,HIGH);
  
  //set interrupt pin for Pattern Switch
  pinMode(switch_interrupt_pin,INPUT);
  attachInterrupt(switch_interrupt_pin-2,setSwitchTrigger,HIGH); 
  
  //initialize all LEDs in stripe with color red
  for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
    leds[x][i] = CRGB::Red;
  }
}
  
void loop()
{
  static unsigned long starttime;
  
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  // but only every 1000/FRAMES_PER_SECOND to keep the framerate modest and time for output modest
  EVERY_N_MILLISECONDS(1000/FRAMES_PER_SECOND){FastLED.show();} 

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  
  //auf Taster reagieren (naechstes Muster) aber nur ausserhalb der Totzeit
  if (switchTrigger && switchPressed==0){
    starttime=millis();
    gCurrentPatternNumber = (gCurrentPatternNumber + 1) % number_of_patterns;
    switchTrigger = 0;
    switchPressed = 1;
  }
  if((millis()-starttime>deadtime) && switchPressed==1){
    switchPressed = 0;
    switchTrigger = 0;
  } 
  
  
  
}

//-------------------------------------------------------------------------------------------------------------------
//--- Trigger-Behandlungs-Routinen

void setKnockTrigger() {
  trigger=1;
}

void nextPatterTrigger()
{
  switchTrigger=1;  
}

//-------------------------------------------------------------------------------------------------------------------
//--- Leuchtmusterfunktionen

void RainbowColors_react() 
{
  static unsigned long starttime;
  if (trigger && on==0){
    starttime=millis();
    FastLED.setBrightness(int(1*BRIGHTNESS));
    trigger = 0;
    on = 1;
  }
  else if((millis()-starttime>REACTONBEATDURATION) && on){
    FastLED.setBrightness(int(0.3*BRIGHTNESS));
    on = 0;
    trigger=0;
  }
  else if(!on){
    FastLED.setBrightness(int(0.3*BRIGHTNESS));
  }
  
  
  // streifen mit Regenbogenfarben fuellen
  fill_rainbow( leds,NUM_LEDS_PER_STRIP, gHue, DELTA_HUE);
}

void White_react() 
{
  static unsigned long starttime;
  if (trigger && on==0){
    starttime=millis();
    FastLED.setBrightness(int(1*BRIGHTNESS));
    trigger = 0;
    on = 1;
  }
  else if((millis()-starttime>REACTONBEATDURATION) && on){
    FastLED.setBrightness(int(0*BRIGHTNESS));
    on = 0;
    trigger=0;
  }
  else if(!on){
    FastLED.setBrightness(int(0*BRIGHTNESS));
  }
    
  
  // set all LEDs in stripe to white
  for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
    leds[i] = CRGB::White;
  }
}

void RainbowColors_bars() 
{
  static unsigned long starttime;
  int timenow=millis();
  int n;
  
  FastLED.setBrightness(int(BRIGHTNESS));
  
  if (trigger && on==0){    
    // streifen mit Regenbogenfarben fuellen
    fill_rainbow( leds,NUM_LEDS_PER_STRIP, gHue, DELTA_HUE);
    starttime=timenow;
    trigger = 0;
    on = 1;
  }
 else if((timenow-starttime<barduration)){
    n = int(float(timenow-starttime)/float(barduration)*float(NUM_LEDS_PER_STRIP));
    ParaPaletteColors_bar(currentPalette, n);
    // streifen mit Regenbogenfarben fuellen
    fill_rainbow( leds,NUM_LEDS_PER_STRIP, gHue, DELTA_HUE);
    // von einer Seite dann mit Schwarz ueberschreiben bis n
    for(int i=0; i<n; i++){
      leds[i] = CRGB::Black;
    }
  }
  else if(on==0){
    // alle LEDs ím Streifen schwarz setzen
    for(int i=0; i<NUM_LEDS_PER_STRIP; i++){
      leds[i] = CRGB::Black;
  }
  for(int i=0; i< NUM_LEDS_PER_STRIP;i++){
      leds[i].nscale8_video( 85);                         //ganz boeser hack von sarah, tut nichts, entfernt ruckeln, anstatt BRIGHTNESS zu setzen
  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }
}
