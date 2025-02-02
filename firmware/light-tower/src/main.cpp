#include <Arduino.h>
#include <FastLED.h>

#define DATA_PIN 12
#define NUM_LEDS 60
CRGB leds[NUM_LEDS];
 
void setup() { 
    FastLED.addLeds<WS2811, DATA_PIN, BRG>(leds, NUM_LEDS); //BRG is the order of this specific strip, usually they are RGB.
}
 
void loop() { 
  // Turn the LED on, then pause 
  for (int i = 0; i <= 32; i++) { // each band contains 8 segments of LEDs, 0 is first, 44 is last (top).
    leds[i] = CRGB(255,255,255);
  }
  for (int i = 32; i <= 44; i++) {
    leds[i] = CRGB(255,255,255);
  }
}