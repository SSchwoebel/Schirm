#include "FastLED.h"
#include <avr/interrupt.h>
#include "pins_arduino.h"

FASTLED_USING_NAMESPACE

// Entwurf fuer den Leuchtschirm basierend auf dem Beispiel "DemoReel100" und https://github.com/FastLED/FastLED/wiki/Multiple-Controller-Examples
//
// albi sagt hallo
// auch yannic gruesst die Welt

//#define AUFTRITT
#define DATA_PINS_START    5
#define NUM_STRIPS 8
#ifdef AUFTRITT
#define NUM_LEDS_PER_STRIP 30
#define BARS_INSIDE 1
#define DELTA_HUE -3
#define GLITTER_N 30
#endif
#ifndef AUFTRITT
#define NUM_LEDS_PER_STRIP 9
#define BARS_INSIDE 0
#define DELTA_HUE 7
#define GLITTER_N 10
#endif

CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];
CRGB *leds_flat;                   //Pointer zur Aufnahme der Anfangsadresse des 2D-Arrays, falls man doch alle LEDS als ein 1D-array ansprechen moechte

//#define BRIGHTNESS          20    //define variable instead compiler directive
#define FRAMES_PER_SECOND  100
#define BLINK_RATE_DEFAULT  120   //Blinken pro Minute
#define REACTONBEATDURATION 50   //wie lange leuchten nach beat in ms
#define ROTATION_RATE_DEFAULT 60   //Rotationen pro Minute, z.B. bei goaround()

uint8_t blink_rate;
uint8_t rotation_rate;
uint8_t BRIGHTNESS;

const int knock_interrupt_pin=3; 
const int switch_interrupt_pin=2;
//const int knockDigital=13; 
const int brightnessPoti = A0; // brightness poti is connected to analog pin 0

void setup() {
  delay(3000); // 3 second delay for recovery
  //Serial.begin(9600);    //richte serielle Schnittstelle ein fuer das Debugging
  

  // tell FastLED about the LED strip configuration
  // pin seems to have to be calculated at compile time.....
  FastLED.addLeds<WS2812B,DATA_PINS_START+0,GRB>(leds[0],NUM_LEDS_PER_STRIP);//BRG
  FastLED.addLeds<WS2812B,DATA_PINS_START+1,GRB>(leds[1],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B,DATA_PINS_START+2,GRB>(leds[2],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B,DATA_PINS_START+3,GRB>(leds[3],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B,DATA_PINS_START+4,GRB>(leds[4],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B,DATA_PINS_START+5,GRB>(leds[5],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B,DATA_PINS_START+6,GRB>(leds[6],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B,DATA_PINS_START+7,GRB>(leds[7],NUM_LEDS_PER_STRIP);
  // set master brightness control
  BRIGHTNESS=analogRead(brightnessPoti)/8;
  FastLED.setBrightness(BRIGHTNESS);
  
  //set Digital Input for Knock Sensor
  //pinMode(knockDigital,INPUT);
  
  //set interrupt pin for Knock Sensor
  pinMode(knock_interrupt_pin,INPUT);
  //  attachInterrupt(digitalPinToInterrupt(knock_interrupt_pin),setTrigger,RISING);
  attachInterrupt(knock_interrupt_pin-2,setTrigger,HIGH); //knock_interrupt_pin-2 is a dirty workarround
  
  //set interrupt pin for Pattern Switch
  pinMode(switch_interrupt_pin,INPUT);
  //  attachInterrupt(digitalPinToInterrupt(knock_interrupt_pin),setTrigger,RISING);
  attachInterrupt(switch_interrupt_pin-2,nextPatterTrigger,HIGH);  //switch_interrupt_pin-2 is a dirty workarround
  
  

  leds_flat = (CRGB *)leds;                                  //hier wird dem Pointer leds_flat die Anfangsaddresse des 2D-Arrays zugewiesen und dabei eine Typenkonversion (cast) durchgeführt, so dass sich leds_flat jetzt wie ein 1D-Array verhält
  blink_rate=BLINK_RATE_DEFAULT;
  rotation_rate=ROTATION_RATE_DEFAULT;
  
     for(int x = 0; x < NUM_STRIPS; x++) {
      for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
        leds[x][i] = CRGB::Red;
      }
     }
  
  //Serial.begin(9600);       // use the serial port

}


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();

SimplePatternList gPatterns = {RainbowColors_fade,
RainbowStripeColors_fade, 
OceanColors_fade, 
LavaColors_fade, 
ForestColors_fade, 
White_fade,
RainbowColors_bars_fast, 
OceanColors_bars_fast, 
ForestColors_bars_fast,
RainbowColors_bars_slow, 
OceanColors_bars_slow, 
ForestColors_bars_slow,
RainbowColors_react, 
White_react,
RainbowColors_withGlitter_react, 
RainbowStripeColors_withGlitter_react, 
OceanColors_withGlitter_react, 
LavaColors_withGlitter_react, 
ForestColors_withGlitter_react,
rainbow, 
rainbowWithGlitter, 
confetti, 
sinelon, 
juggle, 
bpm, 
goaround, 
rainbow2, 
rainbowWithGlitter2, 
confetti2, 
sinelon2, 
juggle2, 
bpm2 };
int n_patterns = 32;
//SimplePatternList gPatterns = {RainbowColors_fade, RainbowStripeColors_fade, OceanColors_fade, LavaColors_fade, ForestColors_fade, White_fade,
//RainbowColors_bars_fast, OceanColors_bars_fast, LavaColors_bars_fast, ForestColors_bars_fast,
//RainbowColors_bars_slow, OceanColors_bars_slow, LavaColors_bars_slow, ForestColors_bars_slow,
//RainbowColors_react, White_react,
//RainbowColors_withGlitter_react, RainbowStripeColors_withGlitter_react, OceanColors_withGlitter_react, LavaColors_withGlitter_react, ForestColors_withGlitter_react,
//rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm, goaround};

// String-Liste der Namen dieser Pattern um diese dann auf dem Serial Monitor ausgeben zu können. Muss manuell geaendert werden.
const char *PatternNames[] = { "reactonbeat", "rainbow", "rainbowWithGlitter", "confetti", "sinelon", "juggle", "bpm" , "rainbow2", "rainbowWithGlitter2", "confetti2", "sinelon2", "juggle2", "bpm2" , "reactonbeat", "goaround"};

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
//uint8_t trigger = 0; // Globaler Trigger Wert, benutzbar um ein Triggerevent an Pattern-Funktion weiterzuleiten
uint8_t on = 0;
uint8_t switchPressed=0;

volatile uint8_t trigger=0;  //Globaler Trigger Wert, wird von Interrupt-Funktion "setTrigger" genutzt um Trigger an loop weiterzugeben. Muss dazu als "volatile" definiert werden.
volatile uint8_t switchTrigger=0;  //Globaler Trigger Wert, wird von Interrupt-Funktion "setTrigger" genutzt um Trigger an loop weiterzugeben. Muss dazu als "volatile" definiert werden.
  
void loop()
{
  //hierhin mit den Sensorabfragen, die dann von den Patternfunktionen verwertet werden sollen
  //EVERY_N_MILLISECONDS ( 1000 ){trigger= 1; }  // Simuliere 120bpm input vom "beatsensor"  EVERY_N_MILLISECONDS
    
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  // but only every 1000/FRAMES_PER_SECOND to keep the framerate modest and time for output modest
  EVERY_N_MILLISECONDS(1000/FRAMES_PER_SECOND){FastLED.show();} 
  //FastLED.delay(1000/FRAMES_PER_SECOND); 
  
  
  //EVERY_N_MILLISECONDS( 1 ){ trigger=digitalRead(knockDigital); }// Serial.println("Knock!"); Serial.println(sensorReading);}}

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  //EVERY_N_SECONDS( 30 ) { nextPattern_random(); } // change patterns periodically
  nextPatternSwitch();
  
  
  EVERY_N_MILLISECONDS(100){ BRIGHTNESS=analogRead(brightnessPoti)/8;}
  
}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
  //Falls das Array von Pattern Namen gleich lang ist wie das der Pattern-Funktionen (deutete auf gewissenhaftes Ausfuellen hin), dann Namen ausgeben
  //if(ARRAY_SIZE(gPatterns)==ARRAY_SIZE(PatternNames)){
  //  Serial.println(PatternNames[gCurrentPatternNumber]);
  //}
}

void nextPattern_random()
{
  // add one to the current pattern number, and wrap around at the end
  int n = random16(n_patterns);
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + n) % ARRAY_SIZE( gPatterns);
  //Falls das Array von Pattern Namen gleich lang ist wie das der Pattern-Funktionen (deutete auf gewissenhaftes Ausfuellen hin), dann Namen ausgeben
  //if(ARRAY_SIZE(gPatterns)==ARRAY_SIZE(PatternNames)){
  //  Serial.println(PatternNames[gCurrentPatternNumber]);
  //}
}

void nextPatternSwitch()
{
   static unsigned long starttime;
   int deadtime=500;
   
   if (switchTrigger && switchPressed==0){
      starttime=millis();
      nextPattern();
      switchTrigger = 0;
      switchPressed = 1;
    }
    if((millis()-starttime>deadtime) && switchPressed==1){
      switchPressed = 0;
      switchTrigger = 0;
    } 
}
  
void nextPatterTrigger()
{
  switchTrigger=1;  
}

void fill_PaletteColors(struct CRGB *pFirstLED, int numToFill,  CRGBPalette16 palette, uint8_t initialhue, uint8_t deltahue=5) //predefined "palette" variables: - CloudColors, + LavaColors_p, + OceanColors_p, + ForestColors_p,RainbowColors_p, ++ RainbowStripeColors_p,-PartyColors_p,-HeatColors_p
{
 for( int i = 0; i < numToFill; i++) {
 pFirstLED[i] = ColorFromPalette( palette, initialhue);
 initialhue += deltahue;
 }
}

void ParaPaletteColors(CRGBPalette16 palette)                                      //Parallel jeden Arm mit den Farben aus der currentPalette fuellen
{
  fill_PaletteColors( leds[0],NUM_LEDS_PER_STRIP, palette, gHue, DELTA_HUE);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

void ParaPaletteColors_bar(CRGBPalette16 palette, int n)                                      //Parallel jeden Arm mit den Farben aus der currentPalette fuellen
{
  fill_PaletteColors( leds[0],NUM_LEDS_PER_STRIP, palette, gHue, DELTA_HUE);
  if(BARS_INSIDE){    
    for(int i=NUM_LEDS_PER_STRIP-1; i>=NUM_LEDS_PER_STRIP-n; i--){
      leds[0][i] = CRGB::Black;
    }
  }
  else {
    for(int i=0; i<n; i++){
      leds[0][i] = CRGB::Black;
    }
  }
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds[0],NUM_LEDS_PER_STRIP, gHue, DELTA_HUE);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void rainbow_bar(int n) 
{
  //n determines the amount of black LEDs
  // FastLED's built-in rainbow generator
  fill_rainbow( leds[0],NUM_LEDS_PER_STRIP, gHue, DELTA_HUE);
  for(int i=0; i<n; i++){
    leds[0][i] = CRGB::Black;
  }
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void rainbow2() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds_flat,NUM_STRIPS*NUM_LEDS_PER_STRIP, gHue, DELTA_HUE);
  FastLED.setBrightness(BRIGHTNESS);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void rainbowWithGlitter2() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
  FastLED.setBrightness(BRIGHTNESS);
}


void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds_flat[ random16(NUM_STRIPS*NUM_LEDS_PER_STRIP) ] += CRGB::White;
  }
}

void addGlitter_more( fract8 chanceOfGlitter, int n) 
{
  if( random8() < chanceOfGlitter) {
    for(int i=0; i<n; i++){
      leds_flat[ random16(NUM_STRIPS*NUM_LEDS_PER_STRIP) ] += CRGB::White;
    }
  }
}

//---------------------------------------------------------------------------------------

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds[0], NUM_LEDS_PER_STRIP, 10);
  int pos = random16(NUM_LEDS_PER_STRIP);
  leds[0][pos] += CHSV( gHue + random8(64), 200, 255);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void confetti2() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds_flat, NUM_LEDS_PER_STRIP*NUM_STRIPS, 10);
  int pos = random16(NUM_LEDS_PER_STRIP*NUM_STRIPS);
  leds_flat[pos] += CHSV( gHue + random8(64), 200, 255);
  FastLED.setBrightness(BRIGHTNESS);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails; Parallel fuer jeden Arm gleich
  
  fadeToBlackBy( leds[0], NUM_LEDS_PER_STRIP, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS_PER_STRIP-1 );
  leds[0][pos] += CHSV( gHue, 255, 192);
  for(int i=1;i<NUM_STRIPS;i++){
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
    //memcpy(&leds[i][i], &leds[i-1][i-1], (NUM_LEDS_PER_STRIP-i) *sizeof(CRGB));
    //memcpy(&leds[i][0], &leds[i-1][NUM_LEDS_PER_STRIP-i], i *sizeof(CRGB));
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void sinelon2()
{
  // a colored dot sweeping back and forth, with fading trails; Parallel fuer jeden Arm gleich
  
  fadeToBlackBy( leds_flat, NUM_STRIPS*NUM_LEDS_PER_STRIP, 20);
  int pos = beatsin16( 13, 0, NUM_STRIPS*NUM_LEDS_PER_STRIP-1 );
  leds_flat[pos] += CHSV( gHue, 255, 192);
  FastLED.setBrightness(BRIGHTNESS);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( blink_rate, 64, 255);
  for( int i = 0; i < NUM_LEDS_PER_STRIP; i++) { //9948
    leds[0][i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void bpm2()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( blink_rate, 64, 255);
  for( int i = 0; i < NUM_STRIPS*NUM_LEDS_PER_STRIP; i++) { //9948
    leds_flat[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds[0], NUM_LEDS_PER_STRIP, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[0][beatsin16( i+7, 0, NUM_LEDS_PER_STRIP )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void juggle2() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds_flat, NUM_STRIPS*NUM_LEDS_PER_STRIP, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds_flat[beatsin16( i+7, 0, NUM_STRIPS*NUM_LEDS_PER_STRIP )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void reactonbeat() {
  static unsigned long starttime;
  if (trigger && on==0){
      starttime=millis();
    for(int x = 0; x < NUM_STRIPS; x++) {
      for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
        leds[x][i] = CRGB::Black;
      }
    }
    trigger = 0;
    on = 1;
  }
  if((millis()-starttime>REACTONBEATDURATION) && on){
    for(int x = 0; x < NUM_STRIPS; x++) {
      for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
        leds[x][i] = CRGB::Red;
      }
    }
    on = 0;
    trigger =0;
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void reactonbeat2() {
  static unsigned long starttime;
  if (trigger && on==0){
      starttime=millis();
    for(int x = 0; x < NUM_STRIPS; x++) {
      for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
        leds[x][i] = CRGB::Red;
      }
    }
    trigger = 0;
    on = 1;
  }
  if((millis()-starttime>REACTONBEATDURATION) && on){
    for(int x = 0; x < NUM_STRIPS; x++) {
      for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
        leds[x][i] = CRGB::Black;
      }
    }
    on = 0;
    trigger=0;
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void goaround(){
  uint8_t maxbright=192;
  uint8_t minbright=100;
  uint8_t beat = beat8(rotation_rate);
  for(int i = 0; i<NUM_STRIPS; i++){
    fill_gradient(leds[i], 0, CHSV( gHue, 255, scale8(sin8((i*32-beat)/2),minbright) ), NUM_LEDS_PER_STRIP, CHSV( gHue, 255, scale8(sin8((i*32-beat)/2),maxbright) ) );
  }
 FastLED.setBrightness(BRIGHTNESS);
  
}

void ameise() {
  //Serial.println("ameise");
  for(int x = 0; x < NUM_STRIPS; x++) {
    // This inner loop will go over each led in the current strip, one at a time
    for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[x][i] = CRGB::Red;
      FastLED.show();
      leds[x][i] = CRGB::Black;
      delay(100);
    }
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void spiral() {
  //Serial.println("spiral");
  for(int x = 0; x < NUM_LEDS_PER_STRIP; x++) {
    for(int k = 0; k < NUM_STRIPS; k++) {
      // This inner loop will go over each led in the current 
        for(int i = 0; i < NUM_LEDS_PER_STRIP; i++){
          leds[k][i] = CRGB::Black;
        }
        leds[k][(k+x)%NUM_LEDS_PER_STRIP] = CRGB::Red;
        //leds[k][(k+x+2)%NUM_LEDS_PER_STRIP] = CRGB::Blue;
        leds[k][(k+x+4)%NUM_LEDS_PER_STRIP] = CRGB::Yellow;
        
    }
    //addGlitter(80);
    FastLED.show();
    delay(100);
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void sort_numbers_8_strips() {
  //Serial.println("sort");
  // This inner loop will go over each led in the current strip, one at a time
  for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
    leds[0][i] = CRGB(255,255,255);
    leds[1][i] = CRGB::Violet;
    leds[2][i] = CRGB::Blue;
    leds[3][i] = CRGB::Green;
    leds[4][i] = CRGB::Yellow;
    leds[5][i] = CRGB::Orange;
    leds[6][i] = CRGB::Red;
    leds[7][i] = CRGB(0,0,0);
    }
    FastLED.show();
    delay(5000);
    for(int x = 0; x < NUM_STRIPS; x++) {
    // This inner loop will go over each led in the current strip, one at a time
    for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[x][i] = CRGB::Black;
    }
  }
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.show();
  delay(500);
}

void setTrigger() {
  trigger=1;
}

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
  
  
  // FastLED's built-in rainbow generator
  fill_rainbow( leds[0],NUM_LEDS_PER_STRIP, gHue, DELTA_HUE);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
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
    
  
  // FastLED's built-in rainbow generator
  fill_white();
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

void fill_white()
{
    for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[0][i] = CRGB::White;
    }
}

//-------------------------------------------------------------------

void PaletteColors_withGlitter_react(CRGBPalette16 currentPalette)
{
  static unsigned long starttime;
  int fadeduration=250;
  int timenow=millis();
  
  FastLED.setBrightness(int(1*BRIGHTNESS));                     //Master-Brightness auf 2-fach
  ParaPaletteColors(currentPalette);                             //Arme mit Palettenfarben fuellen
  for(int i=0; i< NUM_STRIPS*NUM_LEDS_PER_STRIP;i++){
      leds_flat[i].nscale8_video( 85);                         //alle soeben gesetzten Regenbogenfarben in der Helligkeit dritteln, so dass sich Glitzer hervorhebt.
  }
  
  
  if (trigger && on==0){
    starttime=timenow;
    addGlitter_more(100, GLITTER_N);
    trigger = 0;
    on = 1;
  }
  if(on){
    addGlitter_more(100, GLITTER_N);

  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }
}

void RainbowColors_withGlitter_react() 
{ 
  PaletteColors_withGlitter_react(RainbowColors_p) ;
}

void LavaColors_withGlitter_react() 
{ 
  PaletteColors_withGlitter_react(LavaColors_p) ;
}

void OceanColors_withGlitter_react() 
{ 
  PaletteColors_withGlitter_react(OceanColors_p) ;
}

void ForestColors_withGlitter_react() 
{ 
  PaletteColors_withGlitter_react(ForestColors_p) ;
}

void RainbowStripeColors_withGlitter_react() 
{ 
  PaletteColors_withGlitter_react(RainbowStripeColors_p) ;
}

void rainbowWithGlitter_react() 
{
  PaletteColors_withGlitter_react(RainbowColors_p) ;
}



//-------------------------------------------------------------------

void PaletteColors_fade(CRGBPalette16 currentPalette)                         //for convenience, is called by the specialized PatternFunctions
{
  
  static unsigned long starttime;
  int fadeduration=250;
  int timenow=millis();

  if (trigger && on==0){
    starttime=timenow;
    FastLED.setBrightness(int(1*BRIGHTNESS));
    trigger = 0;
    on = 1;
  }
  
  if((timenow-starttime<fadeduration)){
    FastLED.setBrightness(int((0.7*((fadeduration-timenow+starttime)/float(fadeduration))+0.3)*BRIGHTNESS));

  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }
  if(!on){
    FastLED.setBrightness(int(0.3*BRIGHTNESS));
  }
  ParaPaletteColors(currentPalette);                              //Arme mit Palettenfarben fuellen
  for(int i=0; i< NUM_STRIPS*NUM_LEDS_PER_STRIP;i++){
      leds_flat[i].nscale8_video( 255);                         //ganz boeser hack von sarah, tut nichts, entfernt ruckeln
  } 
}

void RainbowColors_fade() 
{
  PaletteColors_fade(RainbowColors_p) ;
}


void LavaColors_fade() 
{ 
  PaletteColors_fade(LavaColors_p) ;
}

void OceanColors_fade() 
{ 
  PaletteColors_fade(OceanColors_p) ;
}

void ForestColors_fade() 
{ 
  PaletteColors_fade(ForestColors_p) ;
}

void RainbowStripeColors_fade() 
{ 
  PaletteColors_fade(RainbowStripeColors_p) ;
}

void White_fade()                         
{
  
  static unsigned long starttime;
  int fadeduration=250;
  int timenow=millis();

  if (trigger && on==0){
    starttime=timenow;
    FastLED.setBrightness(int(1*BRIGHTNESS));
    trigger = 0;
    on = 1;
  }
  
  if((timenow-starttime<fadeduration)){
    FastLED.setBrightness(int((1*((fadeduration-timenow+starttime)/float(fadeduration))+0.0)*BRIGHTNESS));

  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }
  if (!on){
    FastLED.setBrightness(int(0*BRIGHTNESS));
  }
  fill_white();
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

//-------------------------------------------------------------------

// base function for bars

void PaletteColors_bars(CRGBPalette16 currentPalette, int barduration) 
{
  static unsigned long starttime;
  int timenow=millis();
  int n;
  
  FastLED.setBrightness(int(BRIGHTNESS));
  
  if (trigger && on==0){    
    ParaPaletteColors(currentPalette);
    starttime=timenow;
    trigger = 0;
    on = 1;
  }
 else if((timenow-starttime<barduration)){
    n = int(float(timenow-starttime)/float(barduration)*float(NUM_LEDS_PER_STRIP));
    ParaPaletteColors_bar(currentPalette, n);
  }
  else if(on==0){
    ParaPaletteColors_bar(currentPalette, NUM_LEDS_PER_STRIP);
  }
  for(int i=0; i< NUM_STRIPS*NUM_LEDS_PER_STRIP;i++){
      leds_flat[i].nscale8_video( 85);                         //ganz boeser hack von sarah, tut nichts, entfernt ruckeln
  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }
}

// functions with color palette: fast

void RainbowColors_bars_fast() 
{
  int barduration = 250;
  PaletteColors_bars(RainbowColors_p, barduration);
}

void LavaColors_bars_fast() 
{
  int barduration = 250;
  PaletteColors_bars(LavaColors_p, barduration);
}

void OceanColors_bars_fast() 
{
  int barduration = 250;
  PaletteColors_bars(OceanColors_p, barduration);
}

void ForestColors_bars_fast() 
{
  int barduration = 250;
  PaletteColors_bars(ForestColors_p, barduration);
}

// functions with color palette: slow

void RainbowColors_bars_slow() 
{
  int barduration = 500;
  PaletteColors_bars(RainbowColors_p, barduration);
}

void LavaColors_bars_slow() 
{
  int barduration = 500;
  PaletteColors_bars(LavaColors_p, barduration);
}

void OceanColors_bars_slow() 
{
  int barduration = 500;
  PaletteColors_bars(OceanColors_p, barduration);
}

void ForestColors_bars_slow() 
{
  int barduration = 500;
  PaletteColors_bars(ForestColors_p, barduration);
}
