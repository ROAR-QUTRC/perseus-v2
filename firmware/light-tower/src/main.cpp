#include <Arduino.h>
#include <FastLED.h>

#define DATA_PIN 15
#define NUM_LEDS 90 + 60 * 4
CRGB leds[NUM_LEDS];

void setup()
{
    pinMode(15, OUTPUT);
    FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);  // BRG is the order of this specific strip, usually they are RGB.
}

void light(unsigned band, CRGB color, int brightness = 255)
{
    int ledBandSize = 90;  // number of leds in a band
    int ledBuffer = 0;     // the number of leds to skip

    if (band > 0)
    {
        ledBandSize = 60;
        ledBuffer = 90 + (band - 1) * 60;
    }

    for (int i = 0; i < ledBandSize; i++)
    {
        leds[i + ledBuffer] = color;
    }
    FastLED.setBrightness(brightness);
    FastLED.show();
}

void loop()
{
    light(0, CRGB::Red, 128);
    delay(1000);
    light(0, CRGB::Red);
    delay(1000);
    light(0, CRGB::Orange, 128);
    delay(1000);
    light(0, CRGB::Orange);
    delay(1000);
    light(0, CRGB::Yellow, 128);
    delay(1000);
    light(0, CRGB::Yellow);
    delay(1000);
    light(0, CRGB::Green, 128);
    delay(1000);
    light(0, CRGB::Green);
    delay(1000);
    light(0, CRGB::Blue);
    delay(1000);
    light(0, CRGB::Indigo);
    delay(1000);
    light(0, CRGB::Violet);
    delay(1000);
    light(0, CRGB::White);
    delay(1000);
    light(0, CRGB::Black);
    delay(1000);
}