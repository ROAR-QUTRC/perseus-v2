/*
This firmware is intended to control Perseus' arm payload sub-assembly.acosh

Primarily there is a Linear actuator that needs to be controlled to raise and lower
the robotic arm platform.

Additionally, there is a i2c controlled Ultrasonic sensor that needs to be read 
(this is the piicodev ultrasonic sensor). This reads the position of the platform.

The onboard RGB LED on pin 38 is used for status/debug.

//TODO
* add CANBUS
* add h-bridge control for linear actuator
* add ultrasonic sensor
* add RFID reader



***Stretch goals
* add PID for linear actuator
* add a LED matrix for status display

*/




#include <Arduino.h>
#include <FastLED.h>

// Define the LED pin - ESP32-S3 RGB LED on pin 38
#define LED_PIN  38
#define NUM_LEDS 1

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup()
{
    // Initialize LED pin as output first
    pinMode(LED_PIN, OUTPUT);

    // Start Serial for debugging
    Serial.begin(115200);
    delay(2000);  // Give time for serial to initialize

    Serial.println("\n\nESP32-S3 RGB LED Test Starting...");
    Serial.print("Using LED PIN: ");
    Serial.println(LED_PIN);

    // Initialize FastLED with ESP32-S3's onboard LED
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    Serial.println("FastLED initialized");

    // Set 25% brightness for testing
    FastLED.setBrightness(64);
    Serial.println("Brightness set to 25%");

    // Show we're alive by setting to white briefly
    leds[0] = CRGB::White;
    FastLED.show();
    delay(500);

    // Then off
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(500);

    Serial.println("Setup complete - entering main loop");
}

void loop()
{
    // Red on
    leds[0] = CRGB::Red;
    FastLED.show();
    Serial.println("LED set to RED");
    delay(1000);

    // Off
    leds[0] = CRGB::Black;
    FastLED.show();
    Serial.println("LED set to OFF");
    delay(1000);
}