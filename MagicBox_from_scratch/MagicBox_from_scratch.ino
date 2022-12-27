#include "FastLED.h"
#include <avr/interrupt.h>
#include "pins_arduino.h"
#include "arduinoFFT.h"
#include "math.h"

FASTLED_USING_NAMESPACE


// defines fuer globale Variablen, vor allem Parameter kommen hier hin

#define DATA_PINS_START 22
#define SWITCH_PIN 18
#define AUDIO_PIN1 5
#define AUDIO_PIN2 6
#define BRIGHTNESS_POTI A4 // brightness poti is connected to analog pin 4

#define NUM_STRIPS 4
#define NUM_LEDS_PER_STRIP 65
#define BRIGHTNESS_START 100;
#define SAMPLING_FREQUENCY 10000; // in Hz, muss kleiner gleich 10000 wegen ADC
#define NUM_SAMPLES 128 // muss power of 2 sein
#define NUM_FFT_BINS NUM_LEDS_PER_STRIP

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))


// globale Variablen Definitionen

CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];

int FFT_bins[NUM_FFT_BINS];
int FFT_bins_freqs[NUM_FFT_BINS];

uint8_t brightness = BRIGHTNESS_START;
uint8_t curr_pattern_number = 0;

volatile uint8_t switch_trigger = 0;

const unsigned int sampling_period_us = 1000000 / SAMPLING_FREQUENCY;


// Initialize Arduino FFT as volatile, muss vor setup passieren
arduinoFFT FFT = arduinoFFT();


// Hier kommt die Bootroutine
void setup() {
  delay(3000);

  // FastLED Pin Config
  FastLED.addLeds<WS2812B, DATA_PINS_START+0, GRB>(leds[0], NUM_LEDS_PER_STRIP); //BRG
  FastLED.addLeds<WS2812B, DATA_PINS_START+2, GRB>(leds[1], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PINS_START+4, GRB>(leds[2], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PINS_START+6, GRB>(leds[3], NUM_LEDS_PER_STRIP);

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

PatternList patterns = {FFTpattern_ChoiceColor,FFTpattern_Heat,FFTpattern_ColorWhite,FFTpattern_HeatColors, FFTpattern_OceanColors,FFTpattern_LavaColors,FFTpattern_ForestColors,
                        FFTpattern_CloudColors,FFTpattern_RainbowColors,FFTpattern_PartyColors};

// hier kommt das tatsaechliche Programm
void loop() {

  // sampling
  unsigned long int microseconds;
  for(int i=0; i<NUM_SAMPLES; i++) {
    
    microseconds = micros();

    vReal[i] = analogRead(AUDIO_PIN1);
    vImag[i] = 0;

    while(micros() < (microseconds + sampling_period_us)) {}
    
  }

  // FFT

  FFT.Windowing(vReal, NUM_SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT.Compute(vReal, vImag, NUM_SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, NUM_SAMPLES);

  // Berechnung FFT Bins

  for(int i=0; i<NUM_FFT_BINS; i++) {
    FFT_bins[i] = int(interpolation(FFT_bins_freqs[i]));
  }

  patterns[curr_pattern_number]();

  // alle 10ms zeichnen wir die LEDs neu!
  EVERY_N_MILLISECONDS(10) {
    FastLED.show();
  }

  EVERY_N_MILLISECONDS( 1000 ) {
    if (switch_trigger == 1)
      nextPattern(); 
      switch_trigger=0;      
  }

  EVERY_N_MILLISECONDS(100){ 
    brightness=analogRead(BRIGHTNESS_POTI)/8;
    FastLED.setBrightness(int(brightness));
    }
  // debug
  //Serial.print(curr_pattern_number);
  //Serial.println();

}


// interrupt Funktionen

void setSwitchTrigger() {
  switch_trigger = 1;
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


// -------------- FFT-Patterns

//FFT Base
void FFTpattern_Palette(CRGBPalette16 currentPalette) {

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

  for(int i=0; i<NUM_LEDS_PER_STRIP; i++) {
    leds[0][i] = CHSV(int(brightness), 255, min(FFT_bins[i],255));;
  }

  for(int i=1; i<NUM_STRIPS; i++) {
    memcpy(&leds[i], &leds[0], NUM_LEDS_PER_STRIP * sizeof(CRGB));
  }

  FastLED.setBrightness(255);
}
