#include "FastLED.h"
#include <avr/interrupt.h>
#include "pins_arduino.h"
#include "arduinoFFT.h"
#include "math.h"

FASTLED_USING_NAMESPACE

// Entwurf fuer den Leuchtschirm basierend auf dem Beispiel "DemoReel100" und https://github.com/FastLED/FastLED/wiki/Multiple-Controller-Examples


#define DATA_PINS_START 22
#define NUM_STRIPS 3
#define NUM_LEDS_PER_STRIP 40
#define BARS_INSIDE 0
#define DELTA_HUE 2
#define DELTA_gHUE_BASE 1
#define GLITTER_N 30
#define BRIGHTNESS_START 100
#define BRIGHTNESS_INC 16


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
const int switch_pin=18;
const int brightnessPoti = A4; // brightness poti is connected to analog pin 4

const int SAMPLES=128;            //Anzahl der Samples fuer FFT. Must be a power of 2 
const int SAMPLING_FREQUENCY=10000;//Sampling Frequenz fuer FFT. Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
uint8_t delta_gHue = DELTA_gHUE_BASE; // step for rotating the base color
//uint8_t trigger = 0; // Globaler Trigger Wert, benutzbar um ein Triggerevent an Pattern-Funktion weiterzuleiten
uint8_t on = 0;
double vReal[SAMPLES];
double vImag[SAMPLES];
const int LEDsPerBin=1;
const int LengthFFTBins=NUM_LEDS_PER_STRIP/LEDsPerBin;
uint8_t FFTBins[LengthFFTBins];
double FFTBinsXj[LengthFFTBins];
double FFTBinsXk[LengthFFTBins];
int lowerCutoff=4;
double c = double(SAMPLES-1-lowerCutoff)/log(double(LengthFFTBins));

volatile uint8_t trigger=0;  //Globaler Trigger Wert, wird von Interrupt-Funktion "setTrigger" genutzt um Trigger an loop weiterzugeben. Muss dazu als "volatile" definiert werden.
volatile uint8_t switch_trigger=0;
  

// Initialize Arduino FFTvolatile
arduinoFFT FFT = arduinoFFT();

void setup() {
  delay(3000); // 3 second delay for recovery
  //Serial.begin(9600);    //richte serielle Schnittstelle ein fuer das Debugging

  // tell FastLED about the LED strip configuration
  // pin seems to have to be calculated at compile time.....
  FastLED.addLeds<WS2812B,DATA_PINS_START+0,GRB>(leds[0],NUM_LEDS_PER_STRIP);//BRG
  FastLED.addLeds<WS2812B,DATA_PINS_START+2,GRB>(leds[1],NUM_LEDS_PER_STRIP);
  //FastLED.addLeds<WS2812B,DATA_PINS_START+4,GRB>(leds[2],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B,DATA_PINS_START+6,GRB>(leds[2],NUM_LEDS_PER_STRIP);
  //FastLED.addLeds<WS2812B,DATA_PINS_START+8,GRB>(leds[4],NUM_LEDS_PER_STRIP);
  //FastLED.addLeds<WS2812B,DATA_PINS_START+10,GRB>(leds[5],NUM_LEDS_PER_STRIP);
  //FastLED.addLeds<WS2812B,DATA_PINS_START+12,GRB>(leds[6],NUM_LEDS_PER_STRIP);
  //FastLED.addLeds<WS2812B,DATA_PINS_START+14,GRB>(leds[7],NUM_LEDS_PER_STRIP);
  // set master brightness control
  BRIGHTNESS=BRIGHTNESS_START;
  FastLED.setBrightness(BRIGHTNESS);
  

  
  //set interrupt pin for Knock Sensor
  pinMode(knock_interrupt_pin,INPUT);
  //  attachInterrupt(digitalPinToInterrupt(knock_interrupt_pin),setTrigger,RISING);
  attachInterrupt(digitalPinToInterrupt(knock_interrupt_pin),setTrigger,HIGH); 
  
  //set pin mode for Pattern Switch
  pinMode(switch_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switch_pin),setSwitchTrigger,LOW);

  leds_flat = (CRGB *)leds;                                  //hier wird dem Pointer leds_flat die Anfangsaddresse des 2D-Arrays zugewiesen und dabei eine Typenkonversion (cast) durchgeführt, so dass sich leds_flat jetzt wie ein 1D-Array verhält
  blink_rate=BLINK_RATE_DEFAULT;
  rotation_rate=ROTATION_RATE_DEFAULT;
  
     for(int x = 0; x < NUM_STRIPS; x++) {
      for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
        leds[x][i] = CRGB::Red;
      }
     }

  //calculate sampling period for FFT from Sampling Frequency
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  
  //Serial.begin(9600);       // use the serial port

  //create array for x_j values for FFTBins
  for (int i=0;i<LengthFFTBins ; i++)
  {
    FFTBinsXj[i]=c*log(double(LengthFFTBins)/double(LengthFFTBins-i))+lowerCutoff;
    FFTBinsXk[i]= i*double(LengthFFTBins)/SAMPLES;
  }
  

}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();

//SimplePatternList gPatterns = {OceanColors_FFT};
SimplePatternList gPatterns = {RainbowColors_fade,RainbowColors_FFT, OceanColors_FFT};


/*SimplePatternList gPatterns = {RainbowColors_FFT, OceanColors_FFT,RainbowColors_fade, RainbowStripeColors_fade, OceanColors_fade, LavaColors_fade, ForestColors_fade, CloudColors_fade, PartyColors_fade, //White_fade,
RainbowColors_bars_fast, OceanColors_bars_fast, ForestColors_bars_fast, CloudColors_bars_fast, PartyColors_bars_fast,
RainbowColors_bars_slow, OceanColors_bars_slow, ForestColors_bars_slow, CloudColors_bars_slow, PartyColors_bars_slow,
RainbowColors_react, White_react,
RainbowColors_withGlitter_react, RainbowStripeColors_withGlitter_react, OceanColors_withGlitter_react, LavaColors_withGlitter_react, ForestColors_withGlitter_react, CloudColors_withGlitter_react, PartyColors_withGlitter_react,
rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm, goaround, rainbow2, rainbowWithGlitter2, confetti2, sinelon2, juggle2, bpm2 };
*/


void loop()
{ 
    // SAMPLING 
  EVERY_N_MILLISECONDS(50){
    for(int i=0; i<SAMPLES; i++)
    {
      microseconds = micros();    //Overflows after around 70 minutes!
       
      vReal[i] = analogRead(3);
      vImag[i] = 0.0;
    
      while(micros() < (microseconds + sampling_period_us)){
      }
    }
    // FFT
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    
    //FFT_peak =  FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    
    double norm=4/2;
    for (int i=0; i<LengthFFTBins; i++)
    {
      FFTBins[i] = int(norm*(f(FFTBinsXj[i])+f(FFTBinsXk[i])));
    }
  }

    
  gPatterns[gCurrentPatternNumber]();
  

  // send the 'leds' array out to the actual LED strip
  // but only every 1000/FRAMES_PER_SECOND to keep the framerate modest and time for output modest
  EVERY_N_MILLISECONDS(1000/FRAMES_PER_SECOND){FastLED.show();} 
  //FastLED.delay(1000/FRAMES_PER_SECOND); 
  
  
  //EVERY_N_MILLISECONDS( 1 ){ trigger=digitalRead(knockDigital); }// Serial.println("Knock!"); Serial.println(sensorReading);}}

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue = gHue + delta_gHue; } // slowly cycle the "base color" through the rainbow
  EVERY_N_SECONDS( 600 ) { nextPattern_random(); } // change patterns periodically
  EVERY_N_MILLISECONDS( 1000 ) {
    if (switch_trigger == 1)
      nextPattern(); 
      switch_trigger=0;      
  }

  EVERY_N_MILLISECONDS(100){ 
    BRIGHTNESS=analogRead(brightnessPoti)/8;
    FastLED.setBrightness(int(BRIGHTNESS));
    }

  //debug
  //Serial.print(gCurrentPatternNumber);
  //Serial.println();
  
  
  
}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

double f(double x)
{
  return (vReal[int(x)+1]-vReal[int(x)])*(x-int(x))+vReal[int(x)];
}

void setSwitchTrigger()
{
  switch_trigger=1;
}

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void nextPattern_random()
{
  // add one to the current pattern number, and wrap around at the end
  int n = random16(ARRAY_SIZE( gPatterns));
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + n) % ARRAY_SIZE( gPatterns);
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
    FastLED.setBrightness(min(255, 3*int(BRIGHTNESS)));
    trigger = 0;
    on = 1;
  }
  else if((millis()-starttime>REACTONBEATDURATION) && on){
    FastLED.setBrightness(int(BRIGHTNESS));
    on = 0;
    trigger=0;
  }
  else if(!on){
    FastLED.setBrightness(int(BRIGHTNESS));
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

void Red_react() 
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
  fill_red();
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

void fill_red()
{
    for(int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[0][i] = CRGB::Red;
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

void CloudColors_withGlitter_react() 
{ 
  PaletteColors_withGlitter_react(CloudColors_p) ;
}

void PartyColors_withGlitter_react() 
{ 
  PaletteColors_withGlitter_react(PartyColors_p) ;
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
    FastLED.setBrightness(min(255,3*int(BRIGHTNESS)));
    trigger = 0;
    on = 1;
  }
  
  if((timenow-starttime<fadeduration)){
    FastLED.setBrightness(min(255, int((2*((fadeduration-timenow+starttime)/float(fadeduration))+1)*BRIGHTNESS)));

  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }
  if(!on){
    FastLED.setBrightness(int(BRIGHTNESS));
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

void CloudColors_fade() 
{ 
  PaletteColors_withGlitter_react(CloudColors_p) ;
}

void PartyColors_fade() 
{ 
  PaletteColors_withGlitter_react(PartyColors_p) ;
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

void CloudColors_bars_fast() 
{ 
  PaletteColors_withGlitter_react(CloudColors_p) ;
}

void PartyColors_bars_fast() 
{ 
  PaletteColors_withGlitter_react(PartyColors_p) ;
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

void CloudColors_bars_slow() 
{ 
  PaletteColors_withGlitter_react(CloudColors_p) ;
}

void PartyColors_bars_slow() 
{ 
  PaletteColors_withGlitter_react(PartyColors_p) ;
}

//-------------------------------------------------------------------

// base function for stiop

void PaletteColors_stop(CRGBPalette16 currentPalette)                         //for convenience, is called by the specialized PatternFunctions
{
  
  static unsigned long starttime;
  int fadeduration=150;
  int timenow=millis();

  if (trigger && on==0){
    starttime=timenow;
    delta_gHue = -1*DELTA_gHUE_BASE;
    trigger = 0;
    on = 1;
  }
  
  if((timenow-starttime<fadeduration)){
    delta_gHue = -1*DELTA_gHUE_BASE;

  }
  if((timenow-starttime>REACTONBEATDURATION) && on){
    on = 0;
    trigger=0;
  }
  if(!on){
    FastLED.setBrightness(int(0.3*BRIGHTNESS));
    delta_gHue = DELTA_gHUE_BASE;
  }
  ParaPaletteColors(currentPalette);                              //Arme mit Palettenfarben fuellen
  for(int i=0; i< NUM_STRIPS*NUM_LEDS_PER_STRIP;i++){
      leds_flat[i].nscale8_video( 255);                         //ganz boeser hack von sarah, tut nichts, entfernt ruckeln
  } 
}

void RainbowColors_stop() 
{
  PaletteColors_stop(RainbowColors_p) ;
}


void LavaColors_stop() 
{ 
  PaletteColors_stop(LavaColors_p) ;
}

void OceanColors_stop() 
{ 
  PaletteColors_stop(OceanColors_p) ;
}

void ForestColors_stop() 
{ 
  PaletteColors_stop(ForestColors_p) ;
}

void RainbowStripeColors_stop() 
{ 
  PaletteColors_stop(RainbowStripeColors_p) ;
}

void CloudColors_stop() 
{ 
  PaletteColors_withGlitter_react(CloudColors_p) ;
}

void PartyColors_stop() 
{ 
  PaletteColors_withGlitter_react(PartyColors_p) ;
}

//---------------------------------------------------
//-- FFT faehige Leuchtmuster

void PaletteColors_FFT(CRGBPalette16 currentPalette) 
{
  for(int i = 0; i < NUM_LEDS_PER_STRIP-1; i++) {
    leds[0][i] = ColorFromPalette(currentPalette,min(FFTBins[i/LEDsPerBin],128),min(FFTBins[i/LEDsPerBin],255));

  }
  leds[0][NUM_LEDS_PER_STRIP-1]=CRGB::Black;
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

void OceanColors_FFT()
{
  PaletteColors_FFT(OceanColors_p);
}

void RainbowColors_FFT()
{
  PaletteColors_FFT(RainbowColors_p);
}
