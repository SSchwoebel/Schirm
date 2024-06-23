#include "FastLED.h"
#include <avr/interrupt.h>
#include "pins_arduino.h"
#include "arduinoFFT.h"
#include "math.h"

FASTLED_USING_NAMESPACE


// defines fuer globale Variablen, vor allem Parameter kommen hier hin

#define BRIGHTNESS_SUMPATTERNS 255
#define SUMPATTERNSGAIN_FACTOR 0.00001
#define DATA_PINS_START 2
#define SWITCH_PIN 21
#define AUDIO_PIN1 5
#define AUDIO_PIN2 6
#define MIC_PIN 0
#define ANALOG_AUDIO MIC_PIN
#define BRIGHTNESS_POTI A4 // brightness poti is connected to analog pin 4
#define GAIN_POTI A3 // gain poti is connected to analog pin 7

#define NUM_STRIPS 1
#define NUM_LEDS_PER_STRIP 32
#define BRIGHTNESS_START 100;
#define SAMPLING_FREQUENCY 5000; // in Hz, muss kleiner gleich 10000 wegen ADC
#define NUM_SAMPLES 128 // muss power of 2 sein
#define NUM_FFT_BINS NUM_LEDS_PER_STRIP

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#define GLITTER_N NUM_LEDS_PER_STRIP

#define GHUE_BASE_SPEED 1


// globale Variablen Definitionen

CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];
int peak_to_peak;

int FFT_bins[NUM_FFT_BINS];
int FFT_bins_freqs[NUM_FFT_BINS];

uint8_t brightness = BRIGHTNESS_START;
uint8_t curr_pattern_number = 0;
double gain = 1.0;


volatile uint8_t switch_trigger = 0;

const unsigned int sampling_period_us = 1000000 / SAMPLING_FREQUENCY;

int gHue = 0;
int delta_gHue = GHUE_BASE_SPEED;
const int delta_hue = 7;   

// Initialize Arduino FFT as volatile, muss vor setup passieren
arduinoFFT FFT = arduinoFFT();


// Hier kommt die Bootroutine
void setup() {
  delay(3000);

  // FastLED Pin Config
  FastLED.addLeds<WS2812B, DATA_PINS_START+0, GRB>(leds[0], NUM_LEDS_PER_STRIP); //BRG
  FastLED.addLeds<WS2812B, DATA_PINS_START+1, GRB>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PINS_START+2, GRB>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PINS_START+3, GRB>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PINS_START+6, GRB>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PINS_START+7, GRB>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PINS_START+8, GRB>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PINS_START+9, GRB>(leds[0], NUM_LEDS_PER_STRIP);

  FastLED.setBrightness(brightness);

  // set pin mode for pattern switch
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), setSwitchTrigger, LOW);

  // initialize LEDs to red
  for(int i=0; i<NUM_STRIPS; i++) {
    for(int j=0; j<NUM_LEDS_PER_STRIP; j++) {
      leds[i][j] = CRGB::Red;
    }
  }

  // initialize frequency values that are represented in each FFT bin, used for binning and interpolating the frequency spectrum.
  for(int i=0; i<NUM_FFT_BINS; i++) {
    FFT_bins_freqs[i] = i*NUM_SAMPLES/float(NUM_FFT_BINS);
  }

  // serial for debugging
  //Serial.begin(9600);

}

// Pattern list

typedef void (*PatternList[])();

PatternList patterns = {RainbowColors_minusSpeed, RainbowColors_withGlitter_react,RainbowColors_speed,ForestColors_withGlitter_react,Bar_pattern_ChoiceColor,BarInverse_pattern_ChoiceColor,Breathing_pattern_ChoiceColor,FFTpattern_ChoiceColor,FFTpattern_Heat,FFTpattern_ColorWhite,FFTpattern_HeatColors, FFTpattern_OceanColors,FFTpattern_LavaColors,FFTpattern_ForestColors, FFTpattern_CloudColors,FFTpattern_RainbowColors,FFTpattern_PartyColors};

// hier kommt das tatsaechliche Programm
void loop() {

  // sampling
  peak_to_peak = 0;
  int minimum=1024;
  int maximum=0;
  int value;
  unsigned long int microseconds;
  for(int i=0; i<NUM_SAMPLES; i++) {
    
    microseconds = micros();

    value = analogRead(ANALOG_AUDIO);
    if (value> maximum) {
      maximum=value;
    }
    else if (value<minimum) {
      minimum = value;
    }
    vReal[i] = (float)value;
    vImag[i] = 0;

    while(micros() < (microseconds + sampling_period_us)) {}

  }
  /*
  microseconds = micros();
  
  while (micros()-microseconds< 50000)
  {
    value = analogRead(ANALOG_AUDIO);
    if (value> maximum) {
      maximum=value;
    }
    else if (value<minimum) {
      minimum = value;
    }
  }
  */

  peak_to_peak= maximum-minimum;
 

  patterns[curr_pattern_number]();
  FastLED.show();
 

  EVERY_N_MILLISECONDS( 1000 ) {
    if (switch_trigger == 1)
      nextPattern(); 
      switch_trigger=0;      
  }

  EVERY_N_MILLISECONDS(100){ 
    gain = analogRead(GAIN_POTI)*SUMPATTERNSGAIN_FACTOR;
    brightness=analogRead(BRIGHTNESS_POTI)/8;
    FastLED.setBrightness(int(brightness));
    }
  // debug
  //Serial.print(curr_pattern_number);
  //Serial.println();

  // wechsle langsam die Basisfarbe durch den Regenbogen
  EVERY_N_MILLISECONDS( 20 ) { gHue=gHue+delta_gHue; } // slowly cycle the "base color" through the rainbow

}


// interrupt Funktionen

void setSwitchTrigger() {
  switch_trigger = 1;
  delta_gHue = GHUE_BASE_SPEED;
}


// helper functions

double interpolation(double f) {
  return (vReal[int(f)+1] - vReal[int(f)]) * (f - int(f)) + vReal[int(f)];
}

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  curr_pattern_number = (curr_pattern_number + 1) % ARRAY_SIZE( patterns);
}

void calculateFFT()
{
    // FFT
    FFT.Windowing(vReal, NUM_SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
    FFT.Compute(vReal, vImag, NUM_SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, NUM_SAMPLES);

  // Berechnung FFT Bins

    for(int i=0; i<NUM_FFT_BINS; i++) {
      FFT_bins[i] = int(interpolation(FFT_bins_freqs[i]));
    }
}

//---------------Lautstaerkenpatterns

void Bar_pattern_ChoiceColor() {
  double limit= float(NUM_LEDS_PER_STRIP)*peak_to_peak*gain;
  for(int i=0; i<NUM_LEDS_PER_STRIP; i++) {
    if (i<limit){
      
      leds[0][i] = CHSV(int(brightness),float(i)/limit* 255,float(i+1)/limit*254);
    }
    else {
      leds[0][i] = CRGB(0,0,0);
    }
  }

  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }
  FastLED.setBrightness(BRIGHTNESS_SUMPATTERNS);
}

void BarInverse_pattern_ChoiceColor() {
  double scaling=25;
  double limit= float(NUM_LEDS_PER_STRIP)*peak_to_peak*gain;
  int value=0;
  for(int i=NUM_LEDS_PER_STRIP-1; i>=0; i--) {
    value=NUM_LEDS_PER_STRIP-i;
    if (value<limit){
      leds[0][i] = CHSV(int(brightness),float(value)/limit* 255,float(value+1)/limit*254);
    }
    else {
      leds[0][i] = CRGB(0,0,0);
    }
  }

  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }


  FastLED.setBrightness(BRIGHTNESS_SUMPATTERNS);
}


void Breathing_pattern_ChoiceColor() {
  double value = 100*peak_to_peak*gain;
  for(int i=0; i<NUM_LEDS_PER_STRIP; i++) {
    leds[0][i] = CHSV(int(brightness),2*value,value);
  }

  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }

  FastLED.setBrightness(BRIGHTNESS_SUMPATTERNS);
}
// -------------- FFT-Patterns

//FFT Base
void FFTpattern_Palette(CRGBPalette16 currentPalette) {
   calculateFFT();
  for(int i=0; i<NUM_LEDS_PER_STRIP; i++) {
    leds[0][i] = ColorFromPalette(currentPalette, min(FFT_bins[i],128), min(FFT_bins[i],255));
  }

  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }

  FastLED.setBrightness(int(brightness));
}


void FFTpattern_OceanColors()
{
  FFTpattern_Palette(OceanColors_p);
}

void FFTpattern_LavaColors()
{
  FFTpattern_Palette(LavaColors_p);
}

void FFTpattern_ForestColors()
{
  FFTpattern_Palette(ForestColors_p);
}

void FFTpattern_CloudColors()
{
  FFTpattern_Palette(CloudColors_p);
}

void FFTpattern_RainbowColors()
{
  FFTpattern_Palette(RainbowColors_p);
}

void FFTpattern_PartyColors()
{
  FFTpattern_Palette(PartyColors_p);
}

void FFTpattern_HeatColors()
{
  FFTpattern_Palette(HeatColors_p);
}

// -------------- FFT-Patterns One Color

//Base
void FFTpattern_Color(int hue, int saturation) {
  calculateFFT();
  for(int i=0; i<NUM_LEDS_PER_STRIP; i++) {
    leds[0][i] = CHSV(hue, saturation, min(FFT_bins[i],255));
  }

  for(int i=1; i<NUM_STRIPS; i++) {  
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }

  FastLED.setBrightness(int(brightness));
}

void FFTpattern_ColorWhite()
{
  FFTpattern_Color(0,0);
}

void FFTpattern_ColorOrange()
{
  FFTpattern_Color(32,255);
}

void FFTpattern_ColorYellow()
{
  FFTpattern_Color(64,255);
}


// -------------- FFT-Patterns Heat

void FFTpattern_Heat() {
  calculateFFT();
  for(int i=0; i<NUM_LEDS_PER_STRIP; i++) {
    leds[0][i] = HeatColor( min(FFT_bins[i],255));
  }

  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }

  FastLED.setBrightness(int(brightness));
}

// -------------- FFT-Patterns ChoiceColor

void FFTpattern_ChoiceColor() {
  calculateFFT();
  for(int i=0; i<NUM_LEDS_PER_STRIP; i++) {
    leds[0][i] = CHSV(int(brightness), 255, min(FFT_bins[i],255));;
  }

  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }

  FastLED.setBrightness(BRIGHTNESS_SUMPATTERNS);
}


void RainbowColor_react() {
  double limit= float(BRIGHTNESS_SUMPATTERNS)*peak_to_peak*gain;

  fill_rainbow( leds[0], NUM_LEDS_PER_STRIP, gHue, delta_hue);

  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }
  FastLED.setBrightness(limit);
}

void fill_PaletteColors(struct CRGB *pFirstLED, int numToFill,  CRGBPalette16 palette, uint8_t initialhue, uint8_t deltahue=5) //predefined "palette" variables: - CloudColors, + LavaColors_p, + OceanColors_p, + ForestColors_p,RainbowColors_p, ++ RainbowStripeColors_p,-PartyColors_p,-HeatColors_p
{
 for( int i = 0; i < numToFill; i++) {
 pFirstLED[i] = ColorFromPalette( palette, initialhue);
 initialhue += deltahue;
 }
}

void addGlitter( fract8 chanceOfGlitter, int n) 
{
  if( random8() < chanceOfGlitter) {
    for(int i=0; i<n; i++){
      leds[0][ random16(NUM_STRIPS*NUM_LEDS_PER_STRIP) ] = CRGB::White;
    }
  }
}

void ParaPaletteColors(CRGBPalette16 palette)                                      //Parallel jeden Arm mit den Farben aus der currentPalette fuellen
{
  fill_PaletteColors( leds[0],NUM_LEDS_PER_STRIP, palette, gHue, delta_hue);
  for(int i=1;i<NUM_STRIPS;i++){                                 //Parallel fuer jeden Arm gleich
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP *sizeof(CRGB) );
  }
}

void PaletteColors_speed(CRGBPalette16 currentPalette)
{
  //yeah die zahl ist ein guess und durch probieren entstanden...
  if (peak_to_peak>2500*gain) 
  {
    delta_gHue = -5;
  }
  else
  {
    delta_gHue = 5;
  }
  //delta_gHue = int((peak_to_peak*gain-0.5)*5);
  
  ParaPaletteColors(currentPalette);                             //Arme mit Palettenfarben fuellen

  //delta_hue = DELTA_HUE_BASE;
  
  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }
  FastLED.setBrightness(int(brightness));

}

void RainbowColors_speed() 
{ 
  PaletteColors_speed(RainbowColors_p) ;
}

void LavaColors_speed() 
{ 
  PaletteColors_speed(LavaColors_p) ;
}

void OceanColors_speed() 
{ 
  PaletteColors_speed(OceanColors_p) ;
}

void ForestColors_speed() 
{ 
  PaletteColors_speed(ForestColors_p) ;
}

void RainbowStripeColors_speed() 
{ 
  PaletteColors_speed(RainbowStripeColors_p) ;
}

void CloudColors_speed() 
{ 
  PaletteColors_speed(CloudColors_p) ;
}

void PartyColors_speed() 
{ 
  PaletteColors_speed(PartyColors_p) ;
}

void PaletteColors_minusSpeed(CRGBPalette16 currentPalette)
{
  //yeah die zahl ist ein guess und durch probieren entstanden...
  if (peak_to_peak>2500*gain) 
  {
    delta_gHue = 5;
  }
  else
  {
    delta_gHue = -5;
  }
  //delta_gHue = int((peak_to_peak*gain-0.5)*5);
  
  ParaPaletteColors(currentPalette);                             //Arme mit Palettenfarben fuellen

  //delta_hue = DELTA_HUE_BASE;
  
  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }
  FastLED.setBrightness(int(brightness));

}

void RainbowColors_minusSpeed() 
{ 
  PaletteColors_minusSpeed(RainbowColors_p) ;
}

void LavaColors_minusSpeed() 
{ 
  PaletteColors_minusSpeed(LavaColors_p) ;
}

void OceanColors_minusSpeed() 
{ 
  PaletteColors_minusSpeed(OceanColors_p) ;
}

void ForestColors_minusSpeed() 
{ 
  PaletteColors_minusSpeed(ForestColors_p) ;
}

void RainbowStripeColors_minusSpeed() 
{ 
  PaletteColors_minusSpeed(RainbowStripeColors_p) ;
}

void CloudColors_minusSpeed() 
{ 
  PaletteColors_minusSpeed(CloudColors_p) ;
}

void PartyColors_minusSpeed() 
{ 
  PaletteColors_minusSpeed(PartyColors_p) ;
}

void PaletteColors_withGlitter_react(CRGBPalette16 currentPalette)
{
  
  double limit= float(BRIGHTNESS_SUMPATTERNS)*peak_to_peak*gain;
  
  ParaPaletteColors(currentPalette);                             //Arme mit Palettenfarben fuellen

  // Achtung: ich habe noch einen brightness factor fuer den anderen Poti hinzugefuegt, dann kann man nochmal die anzahl glitter unabhaengig vom gain aendern.
  addGlitter(int(255*(float(brightness)/float(255))), int(float(NUM_LEDS_PER_STRIP)*peak_to_peak*gain));//GLITTER_N);gain*
  
  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }
  FastLED.setBrightness(limit);//int(brightness));

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
