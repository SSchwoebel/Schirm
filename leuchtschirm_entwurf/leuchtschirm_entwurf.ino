#include "FastLED.h"
#include <avr/interrupt.h>
#include "pins_arduino.h"

FASTLED_USING_NAMESPACE

// Entwurf fuer den Leuchtschirm basierend auf dem Beispiel "DemoReel100" und https://github.com/FastLED/FastLED/wiki/Multiple-Controller-Examples
//
// albi sagt hallo
// auch yannic gruesst die Welt


#define DATA_PINS_START    4
#define NUM_STRIPS 8
#define NUM_LEDS_PER_STRIP 27
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];
CRGB *leds_flat;                   //Pointer zur Aufnahme der Anfangsadresse des 2D-Arrays, falls man doch alle LEDS als ein 1D-array ansprechen moechte

#define BRIGHTNESS          20
#define FRAMES_PER_SECOND  100
#define BLINK_RATE_DEFAULT  120   //Blinken pro Minute
#define REACTONBEATDURATION 50   //wie lange leuchten nach beat in ms
#define ROTATION_RATE_DEFAULT 60   //Rotationen pro Minute, z.B. bei goaround()

uint8_t blink_rate;
uint8_t rotation_rate;

const int knock_interrupt_pin=2; 
const int switch_interrupt_pin=3;
//const int knockDigital=13; 
const int knockSensor = A0; // the piezo is connected to analog pin 0
const int threshold = 375;  // threshold value to decide when the detected sound is a knock or not


// these variables will change:
int sensorReading = 0;      // variable to store the value read from the sensor pin


void setup() {
  delay(3000); // 3 second delay for recovery
  Serial.begin(9600);    //richte serielle Schnittstelle ein fuer das Debugging
  

  // tell FastLED about the LED strip configuration
  // pin seems to have to be calculated at compile time.....
  FastLED.addLeds<WS2811,DATA_PINS_START+0,BRG>(leds[0],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2811,DATA_PINS_START+1,BRG>(leds[1],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2811,DATA_PINS_START+2,BRG>(leds[2],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2811,DATA_PINS_START+3,BRG>(leds[3],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2811,DATA_PINS_START+4,BRG>(leds[4],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2811,DATA_PINS_START+5,BRG>(leds[5],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2811,DATA_PINS_START+6,BRG>(leds[6],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2811,DATA_PINS_START+7,BRG>(leds[7],NUM_LEDS_PER_STRIP);
  // set master brightness control
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
  
  Serial.begin(9600);       // use the serial port
}


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();

SimplePatternList gPatterns = {LavaColors_fade, OceanColors_fade,  ForestColors_fade,RainbowStripeColors_fade, bars_react, rainbowWithGlitter_react, rainbow_fade, rainbow_react,reactonbeat2, reactonbeat, rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm , rainbow2, rainbowWithGlitter2, confetti2, sinelon2, juggle2, bpm2 , reactonbeat, goaround};

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
  //EVERY_N_SECONDS( 3000 ) { nextPattern(); } // change patterns periodically
  nextPatternSwitch();
  
}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
  //Falls das Array von Pattern Namen gleich lang ist wie das der Pattern-Funktionen (deutete auf gewissenhaftes Ausfuellen hin), dann Namen ausgeben
  if(ARRAY_SIZE(gPatterns)==ARRAY_SIZE(PatternNames)){
    Serial.println(PatternNames[gCurrentPatternNumber]);
  }
}

void nextPatternSwitch()
{
   static unsigned long starttime;
   int deadtime=500;
   
   if (switchTrigger && switchPressed==0){
      starttime=millis();
      FastLED.setBrightness(int(1.0*BRIGHTNESS));
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

void fill_PaletteColors(struct CRGB *pFirstLED, int numToFill,  CRGBPalette16 palette, uint8_t initialhue, uint8_t deltahue=5) //predefined "palette" variables: CloudColors_p, LavaColors_p, Ocean_Colors_p, Forest_Colors_p,Rainbow_Colors,RainbowStripeColors_p,PartyColors_p,Heat_Colors_p
{
 uint8_t brightness = 255;
 
 for( int i = 0; i < numToFill; i++) {
 pFirstLED[i] = ColorFromPalette( palette, initialhue);
 initialhue += deltahue;
 }
}

//---------------------------------------------------------------------------------------

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds[0],NUM_LEDS_PER_STRIP, gHue, 7);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

void rainbow_bar(int n) 
{
  //n determines the amount of black LEDs
  // FastLED's built-in rainbow generator
  fill_rainbow( leds[0],NUM_LEDS_PER_STRIP, gHue, 7);
  for(int i=0; i<n; i++){
    leds[0][i] = CRGB::Black;
  }
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

void rainbow2() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds_flat,NUM_STRIPS*NUM_LEDS_PER_STRIP, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

void rainbowWithGlitter2() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
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

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds[0], NUM_LEDS_PER_STRIP, 10);
  int pos = random16(NUM_LEDS_PER_STRIP);
  leds[0][pos] += CHSV( gHue + random8(64), 200, 255);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

void confetti2() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds_flat, NUM_LEDS_PER_STRIP*NUM_STRIPS, 10);
  int pos = random16(NUM_LEDS_PER_STRIP*NUM_STRIPS);
  leds_flat[pos] += CHSV( gHue + random8(64), 200, 255);
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
}

void sinelon2()
{
  // a colored dot sweeping back and forth, with fading trails; Parallel fuer jeden Arm gleich
  
  fadeToBlackBy( leds_flat, NUM_STRIPS*NUM_LEDS_PER_STRIP, 20);
  int pos = beatsin16( 13, 0, NUM_STRIPS*NUM_LEDS_PER_STRIP-1 );
  leds_flat[pos] += CHSV( gHue, 255, 192);
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
}

void bpm2()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( blink_rate, 64, 255);
  for( int i = 0; i < NUM_STRIPS*NUM_LEDS_PER_STRIP; i++) { //9948
    leds_flat[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
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
}

void juggle2() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds_flat, NUM_STRIPS*NUM_LEDS_PER_STRIP, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds_flat[beatsin16( i+7, 0, NUM_STRIPS*NUM_LEDS_PER_STRIP )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
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
}

void goaround(){
  uint8_t maxbright=192;
  uint8_t minbright=100;
  uint8_t beat = beat8(rotation_rate);
  for(int i = 0; i<NUM_STRIPS; i++){
    fill_gradient(leds[i], 0, CHSV( gHue, 255, scale8(sin8((i*32-beat)/2),minbright) ), NUM_LEDS_PER_STRIP, CHSV( gHue, 255, scale8(sin8((i*32-beat)/2),maxbright) ) );
  }
 
  
}

void ameise() {
  Serial.println("ameise");
  for(int x = 0; x < NUM_STRIPS; x++) {
    // This inner loop will go over each led in the current strip, one at a time
    for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[x][i] = CRGB::Red;
      FastLED.show();
      leds[x][i] = CRGB::Black;
      delay(100);
    }
  }
}

void spiral() {
  Serial.println("spiral");
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
}

void sort_numbers_8_strips() {
  Serial.println("sort");
  // This inner loop will go over each led in the current strip, one at a time
  for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
    leds[0][i] = CRGB(255,255,255);
    leds[1][i] = CRGB::Red;
    leds[2][i] = CRGB::Violet;
    leds[3][i] = CRGB::Blue;
    leds[4][i] = CRGB::Green;
    leds[5][i] = CRGB::Yellow;
    leds[6][i] = CRGB::Orange;
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
  FastLED.show();
  delay(500);
}

void setTrigger() {
  trigger=1;
}

void rainbow_react() 
{
  static unsigned long starttime;
  if (trigger && on==0){
    starttime=millis();
    FastLED.setBrightness(int(3*BRIGHTNESS));
    trigger = 0;
    on = 1;
  }
  if((millis()-starttime>REACTONBEATDURATION) && on){
    FastLED.setBrightness(int(1*BRIGHTNESS));
    on = 0;
    trigger=0;
  }
  
  // FastLED's built-in rainbow generator
  fill_rainbow( leds[0],NUM_LEDS_PER_STRIP, gHue, 7);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

void rainbow_fade() 
{
  static unsigned long starttime;
  int fadeduration=250;
  int timenow=millis();
  
  if (trigger && on==0){
    starttime=timenow;
    FastLED.setBrightness(int(3*BRIGHTNESS));
    trigger = 0;
    on = 1;
  }
  
  if((timenow-starttime<fadeduration)){
    FastLED.setBrightness(int((3.0*(fadeduration-timenow+starttime)/fadeduration+1.0)*BRIGHTNESS));
  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }
  
  // FastLED's built-in rainbow generator
  fill_rainbow( leds[0],NUM_LEDS_PER_STRIP, gHue, 7);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}


void rainbowWithGlitter_react() 
{
  static unsigned long starttime;
  int fadeduration=250;
  int timenow=millis();
  
  rainbow();
  if (trigger && on==0){
    starttime=timenow;
    addGlitter_more(100, 10);
    trigger = 0;
    on = 1;
  }
  if(on){
    addGlitter_more(100, 10);

  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }
}


void LavaColors_fade() 
{ 

  static unsigned long starttime;
  int fadeduration=250;
  int timenow=millis();
  

  CRGBPalette16 currentPalette=LavaColors_p; //predefined "palette" variables: - CloudColors, + LavaColors_p, + Ocean_Colors_p, + Forest_Colors_p,Rainbow_Colors, ++ RainbowStripeColors_p,-PartyColors_p,-Heat_Colors_p
  
  if (trigger && on==0){
    starttime=timenow;
    FastLED.setBrightness(int(3*BRIGHTNESS));
    trigger = 0;
    on = 1;
  }
  
  if((timenow-starttime<fadeduration)){
    FastLED.setBrightness(int((3.0*(fadeduration-timenow+starttime)/fadeduration+1.0)*BRIGHTNESS));

  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }

  
 
  fill_PaletteColors( leds[0],NUM_LEDS_PER_STRIP, currentPalette, gHue, 7);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }

}

void OceanColors_fade() 
{ 

  static unsigned long starttime;
  int fadeduration=250;
  int timenow=millis();
  

  CRGBPalette16 currentPalette=OceanColors_p; //predefined "palette" variables: - CloudColors, + LavaColors_p, + Ocean_Colors_p, + Forest_Colors_p,Rainbow_Colors, ++ RainbowStripeColors_p,-PartyColors_p,-Heat_Colors_p
  
  if (trigger && on==0){
    starttime=timenow;
    FastLED.setBrightness(int(3*BRIGHTNESS));
    trigger = 0;
    on = 1;
  }
  
  if((timenow-starttime<fadeduration)){
    FastLED.setBrightness(int((3.0*(fadeduration-timenow+starttime)/fadeduration+1.0)*BRIGHTNESS));

  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }

  
 
  fill_PaletteColors( leds[0],NUM_LEDS_PER_STRIP, currentPalette, gHue, 7);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }

}

void ForestColors_fade() 
{ 

  static unsigned long starttime;
  int fadeduration=250;
  int timenow=millis();
  

  CRGBPalette16 currentPalette=ForestColors_p; //predefined "palette" variables: - CloudColors, + LavaColors_p, + Ocean_Colors_p, + Forest_Colors_p,Rainbow_Colors, ++ RainbowStripeColors_p,-PartyColors_p,-Heat_Colors_p
  
  if (trigger && on==0){
    starttime=timenow;
    FastLED.setBrightness(int(3*BRIGHTNESS));
    trigger = 0;
    on = 1;
  }
  
  if((timenow-starttime<fadeduration)){
    FastLED.setBrightness(int((3.0*(fadeduration-timenow+starttime)/fadeduration+1.0)*BRIGHTNESS));

  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }

  
 
  fill_PaletteColors( leds[0],NUM_LEDS_PER_STRIP, currentPalette, gHue, 7);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }

}

void RainbowStripeColors_fade() 
{ 

  static unsigned long starttime;
  int fadeduration=250;
  int timenow=millis();
  

  CRGBPalette16 currentPalette=RainbowStripeColors_p; //predefined "palette" variables: - CloudColors, + LavaColors_p, + Ocean_Colors_p, + Forest_Colors_p,Rainbow_Colors, ++ RainbowStripeColors_p,-PartyColors_p,-Heat_Colors_p
  
  if (trigger && on==0){
    starttime=timenow;
    FastLED.setBrightness(int(3*BRIGHTNESS));
    trigger = 0;
    on = 1;
  }
  
  if((timenow-starttime<fadeduration)){
    FastLED.setBrightness(int((3.0*(fadeduration-timenow+starttime)/fadeduration+1.0)*BRIGHTNESS));

  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }

  
 
  fill_PaletteColors( leds[0],NUM_LEDS_PER_STRIP, currentPalette, gHue, 7);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }

}

void bars_react() 
{
  static unsigned long starttime;
  int fadeduration=250;
  int timenow=millis();
  int n;
  
  if (trigger && on==0){    
    rainbow();
    starttime=timenow;
    trigger = 0;
    on = 1;
  }
  if((timenow-starttime<500)){
    n = int((timenow-starttime)/500*8);
    rainbow_bar(n);
  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }
//  if(on==0){
//    rainbow_bar(8);
//  }
}
