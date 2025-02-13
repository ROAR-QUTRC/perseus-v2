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
* add ultrasonic sensor [DONE]
* add RFID reader
* add CAN watchdog timers
* add checks to see if linear actuators already at limits before moving
* use both cores with FreeRTOS (one for CANBUS?)


***Stretch goals
* add PID for linear actuator
* add a LED matrix for status display

*/

#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>

// Define the LED pin - ESP32-S3 RGB LED on pin 38
#define LED_PIN  38
#define NUM_LEDS 1

// PiicoDev Ultrasonic sensor constants
#define PIICODEV_ULTRASONIC_ADDR       0x35
#define PIICODEV_ULTRASONIC_REG_STATUS 0x08
#define PIICODEV_ULTRASONIC_REG_RAW    0x05
#define PIICODEV_ULTRASONIC_REG_PERIOD 0x06
#define PIICODEV_ULTRASONIC_REG_LED    0x07

// Class to handle the PiicoDev Ultrasonic sensor
class PiicoDevUltrasonic
{
public:
    PiicoDevUltrasonic(TwoWire& wire = Wire, uint8_t addr = PIICODEV_ULTRASONIC_ADDR)
        : wire_(wire), addr_(addr)
    {
    }

    bool begin()
    {
        wire_.begin();
        // Set default sample period to 20ms
        setSamplePeriod(20);
        return true;
    }

    // Returns true when a new range sample is available
    bool newSampleAvailable()
    {
        uint8_t status = readRegister(PIICODEV_ULTRASONIC_REG_STATUS);
        return status & 0x01;
    }

    // Returns the pulse round-trip time in microseconds
    uint16_t getRoundTripTime()
    {
        return readRegister16(PIICODEV_ULTRASONIC_REG_RAW);
    }

    // Returns the measured distance in millimeters
    float getDistanceMm()
    {
        const float millimeters_per_microsecond = 0.343;
        return round(getRoundTripTime() * millimeters_per_microsecond / 2);
    }

    // Set the sample period in milliseconds
    void setSamplePeriod(uint16_t period_ms)
    {
        writeRegister16(PIICODEV_ULTRASONIC_REG_PERIOD, period_ms);
    }

private:
    TwoWire& wire_;
    uint8_t addr_;

    uint8_t readRegister(uint8_t reg)
    {
        wire_.beginTransmission(addr_);
        wire_.write(reg);
        wire_.endTransmission(false);
        wire_.requestFrom(addr_, (uint8_t)1);
        return wire_.read();
    }

    uint16_t readRegister16(uint8_t reg)
    {
        wire_.beginTransmission(addr_);
        wire_.write(reg);
        wire_.endTransmission(false);
        wire_.requestFrom(addr_, (uint8_t)2);
        uint16_t value = wire_.read() << 8;
        value |= wire_.read();
        return value;
    }

    void writeRegister16(uint8_t reg, uint16_t value)
    {
        wire_.beginTransmission(addr_);
        wire_.write(reg);
        wire_.write(value >> 8);
        wire_.write(value & 0xFF);
        wire_.endTransmission();
    }
};

// Define the array of leds
CRGB leds[NUM_LEDS];

// Create ultrasonic sensor instance
PiicoDevUltrasonic ultrasonic;

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

    // Initialize ultrasonic sensor
    if (!ultrasonic.begin())
    {
        Serial.println("Failed to initialize ultrasonic sensor!");
        leds[0] = CRGB::Red;  // Show error state
        FastLED.show();
    }
    else
    {
        Serial.println("Ultrasonic sensor initialized");
    }

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
    static unsigned long lastPrint = 0;
    const unsigned long PRINT_INTERVAL = 1000;  // Print every second

    // Read distance if new sample available
    if (ultrasonic.newSampleAvailable())
    {
        float distance = ultrasonic.getDistanceMm();

        // Print reading every PRINT_INTERVAL ms
        if (millis() - lastPrint >= PRINT_INTERVAL)
        {
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" mm");
            lastPrint = millis();

            // Visual feedback - LED color based on distance
            if (distance < 100)
            {
                leds[0] = CRGB::Red;  // Too close
            }
            else if (distance < 300)
            {
                leds[0] = CRGB::Yellow;  // Medium range
            }
            else
            {
                leds[0] = CRGB::Green;  // Good distance
            }
            FastLED.show();
        }
    }
}