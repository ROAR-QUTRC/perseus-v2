// #include <Arduino.h>
// #include <Wire.h>

// // PiicoDev RFID constants - matching Python implementation
// #define PIICODEV_RFID_ADDR 0x2C
// #define DEVICE_ID 578
// #define TAG_CMD_REQIDL 0x26
// #define TAG_AUTH_KEY_A 0x60
// #define TAG_CMD_ANTCOL1 0x93

// class PiicoDevRFID {
// public:
//     PiicoDevRFID(TwoWire& wire = Wire, uint8_t addr = PIICODEV_RFID_ADDR) 
//         : wire_(wire), addr_(addr) {
//     }

//     bool begin() {
//         wire_.begin();
//         delay(50);
//         return true;
//     }

//     bool tagPresent() {
//         uint8_t status = readRegister(0x08);  // _REG_STATUS register
//         return (status & 0x01) != 0;
//     }

//     bool writeText(const String& text) {
//         if (!tagPresent()) {
//             return false;
//         }

//         // Maximum of 144 characters for NTAG
//         if (text.length() > 144) {
//             return false;
//         }

//         // Add null terminator
//         String textWithNull = text + '\0';
        
//         // Write command
//         wire_.beginTransmission(addr_);
//         wire_.write(0x87);  // Write command
//         wire_.write((const uint8_t*)textWithNull.c_str(), textWithNull.length());
//         return wire_.endTransmission() == 0;
//     }

// private:
//     TwoWire& wire_;
//     uint8_t addr_;

//     uint8_t readRegister(uint8_t reg) {
//         wire_.beginTransmission(addr_);
//         wire_.write(reg);
//         wire_.endTransmission(false);
//         wire_.requestFrom(addr_, (uint8_t)1);
//         if (wire_.available()) {
//             return wire_.read();
//         }
//         return 0;
//     }
// };

// PiicoDevRFID rfid;

// void setup() {
//     Serial.begin(115200);
//     delay(2000);

//     Serial.println("\nPiicoDev RFID Writer");
//     Serial.println("Initializing...");
    
//     // Start I2C
//     Wire.begin();

//     // Wait for serial monitor and init RFID
//     delay(1000);
    
//     if (!rfid.begin()) {
//         Serial.println("Failed to initialize RFID module!");
//         while (1) { delay(100); }
//     }

//     Serial.println("RFID module initialized successfully");
//     Serial.println("Place a tag near the RFID module to write text...");
// }

// void loop() {
//     const String textToWrite = "Hello ROAR World, LFG Perseus!";
//     static unsigned long lastCheck = 0;
//     const unsigned long CHECK_INTERVAL = 100;  // Check every 100ms

//     if (millis() - lastCheck >= CHECK_INTERVAL) {
//         if (rfid.tagPresent()) {
//             Serial.println("Tag detected! Writing text...");
            
//             if (rfid.writeText(textToWrite)) {
//                 Serial.println("Success! Text written to tag.");
//                 Serial.println("Text written: " + textToWrite);
//                 Serial.println("\nYou can now remove the tag.");
                
//                 // Wait for tag removal
//                 while (rfid.tagPresent()) {
//                     delay(100);
//                 }
//                 Serial.println("Tag removed. Ready for next tag.");
//             } else {
//                 Serial.println("Failed to write to tag. Please try again.");
//             }
//         }
//         lastCheck = millis();
//     }
//     delay(50);
// }
/*
This firmware is intended to control Perseus' arm payload sub-assembly.

Primarily there is a Linear actuator that needs to be controlled to raise and lower
the robotic arm platform.

Additionally, there is a i2c controlled Ultrasonic sensor that needs to be read 
(this is the piicodev ultrasonic sensor). This reads the position of the platform.

The onboard RGB LED on pin 38 is used for status/debug.

//TODO
* add CANBUS
* add h-bridge control for linear actuator
* add ultrasonic sensor [DONE]
* add RFID reader [DONE]
* add CAN watchog timers
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
#define PIICODEV_ULTRASONIC_ADDR 0x35
#define PIICODEV_ULTRASONIC_REG_STATUS 0x08
#define PIICODEV_ULTRASONIC_REG_RAW 0x05
#define PIICODEV_ULTRASONIC_REG_PERIOD 0x06
#define PIICODEV_ULTRASONIC_REG_LED 0x07

// Distance thresholds for LED control (in mm)
#define MIN_DISTANCE 50
#define MAX_DISTANCE 400

// PiicoDev RFID constants
#define PIICODEV_RFID_ADDR 0x2C

// Define the array of leds
CRGB leds[NUM_LEDS];

// Class to handle the PiicoDev Ultrasonic sensor
class PiicoDevUltrasonic {
public:
    PiicoDevUltrasonic(TwoWire& wire = Wire, uint8_t addr = PIICODEV_ULTRASONIC_ADDR) 
        : wire_(wire), addr_(addr) {
    }

    bool begin() {
        wire_.begin();
        // Set default sample period to 20ms
        setSamplePeriod(20);
        return true;
    }

    bool newSampleAvailable() {
        uint8_t status = readRegister(PIICODEV_ULTRASONIC_REG_STATUS);
        return status & 0x01;
    }

    uint16_t getRoundTripTime() {
        return readRegister16(PIICODEV_ULTRASONIC_REG_RAW);
    }

    float getDistanceMm() {
        const float millimeters_per_microsecond = 0.343;
        return round(getRoundTripTime() * millimeters_per_microsecond / 2);
    }

    void setSamplePeriod(uint16_t period_ms) {
        writeRegister16(PIICODEV_ULTRASONIC_REG_PERIOD, period_ms);
    }

private:
    TwoWire& wire_;
    uint8_t addr_;

    uint8_t readRegister(uint8_t reg) {
        wire_.beginTransmission(addr_);
        wire_.write(reg);
        wire_.endTransmission(false);
        wire_.requestFrom(addr_, (uint8_t)1);
        return wire_.read();
    }

    uint16_t readRegister16(uint8_t reg) {
        wire_.beginTransmission(addr_);
        wire_.write(reg);
        wire_.endTransmission(false);
        wire_.requestFrom(addr_, (uint8_t)2);
        uint16_t value = wire_.read() << 8;
        value |= wire_.read();
        return value;
    }

    void writeRegister16(uint8_t reg, uint16_t value) {
        wire_.beginTransmission(addr_);
        wire_.write(reg);
        wire_.write(value >> 8);
        wire_.write(value & 0xFF);
        wire_.endTransmission();
    }
};

// Class to handle the PiicoDev RFID reader
class PiicoDevRFID {
public:
    PiicoDevRFID(TwoWire& wire = Wire, uint8_t addr = PIICODEV_RFID_ADDR) 
        : wire_(wire), addr_(addr) {
    }

    bool begin() {
        wire_.begin();
        delay(50);
        
        // Following Python reference implementation initialization
        writeRegister(0x2A, 0x80);  // _REG_T_MODE
        writeRegister(0x2B, 0xA9);  // _REG_T_PRESCALER
        writeRegister(0x2C, 0x03);  // _REG_T_RELOAD_HI
        writeRegister(0x2D, 0xE8);  // _REG_T_RELOAD_LO
        writeRegister(0x15, 0x40);  // _REG_TX_ASK
        writeRegister(0x11, 0x3D);  // _REG_MODE
        writeRegister(0x03, 0x80);  // _REG_DIV_I_EN
        writeRegister(0x02, 0x20);  // _REG_COM_I_EN
        
        // Turn antenna on
        uint8_t value = readRegister(0x14);  // _REG_TX_CONTROL
        if (~(value & 0x03)) {
            writeRegister(0x14, value | 0x83);
        }
        
        return true;
    }

    bool tagPresent() {
        // Reset the registers before checking
        writeRegister(0x01, 0x00);  // _CMD_IDLE
        writeRegister(0x05, 0x00);  // _REG_FIFO_LEVEL
        
        // Send REQA command
        writeRegister(0x0D, 0x07);  // _REG_BIT_FRAMING, 7 bits
        
        uint8_t cmd[] = {0x26};  // REQA command
        writeData(cmd, 1);
        
        writeRegister(0x0A, 0x80);  // Clear FIFO buffer
        writeRegister(0x01, 0x0C);  // _CMD_TRANSCEIVE
        
        delay(1);  // Wait for response
        
        uint8_t status = readRegister(0x06);  // Read error register
        uint8_t irqFlags = readRegister(0x04);  // Read interrupt flags
        
        Serial.print("RFID Status: 0x"); Serial.println(status, HEX);
        Serial.print("IRQ Flags: 0x"); Serial.println(irqFlags, HEX);
        
        return (status == 0) && (irqFlags & 0x01);
    }

    String readText() {
        String text = "";
        if (!tagPresent()) {
            return text;
        }

        // Read from NTAG memory starting at page 4
        for (uint8_t page = 4; page < 40; page += 4) {
            uint8_t buffer[16];
            if (readDataPage(page, buffer)) {
                for (int i = 0; i < 16; i++) {
                    if (buffer[i] == 0) return text;  // Found null terminator
                    text += (char)buffer[i];
                }
            }
        }

        return text;
    }

private:
    TwoWire& wire_;
    uint8_t addr_;

    void writeRegister(uint8_t reg, uint8_t value) {
        wire_.beginTransmission(addr_);
        wire_.write(reg);
        wire_.write(value);
        if (wire_.endTransmission() != 0) {
            Serial.print("Error writing register 0x");
            Serial.println(reg, HEX);
        }
    }

    uint8_t readRegister(uint8_t reg) {
        wire_.beginTransmission(addr_);
        wire_.write(reg);
        if (wire_.endTransmission(false) != 0) {
            Serial.print("Error reading register 0x");
            Serial.println(reg, HEX);
            return 0;
        }
        
        if (wire_.requestFrom(addr_, (uint8_t)1) != 1) {
            Serial.println("Error requesting data");
            return 0;
        }
        
        return wire_.read();
    }
    
    void writeData(uint8_t* data, uint8_t length) {
        wire_.beginTransmission(addr_);
        wire_.write(0x09);  // FIFO data register
        for (uint8_t i = 0; i < length; i++) {
            wire_.write(data[i]);
        }
        wire_.endTransmission();
    }
    
    bool readDataPage(uint8_t page, uint8_t* buffer) {
        uint8_t cmd[] = {0x30, page};  // READ command
        writeData(cmd, 2);
        
        // Request 16 bytes (one page)
        wire_.requestFrom(addr_, (uint8_t)16);
        if (wire_.available() != 16) {
            Serial.println("Error reading page data");
            return false;
        }
        
        for (int i = 0; i < 16; i++) {
            buffer[i] = wire_.read();
        }
        return true;
    }
};

// Create sensor instances
PiicoDevUltrasonic ultrasonic;
PiicoDevRFID rfid;

void setup() {
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

    // Set full brightness
    FastLED.setBrightness(255);
    Serial.println("Brightness set to 100%");

    // Initialize ultrasonic sensor
    if (!ultrasonic.begin()) {
        Serial.println("Failed to initialize ultrasonic sensor!");
        leds[0] = CRGB::Red;  // Show error state
        FastLED.show();
    } else {
        Serial.println("Ultrasonic sensor initialized");
    }

    // Initialize RFID reader
    if (!rfid.begin()) {
        Serial.println("Failed to initialize RFID reader!");
        leds[0] = CRGB::Red;  // Show error state
        FastLED.show();
    } else {
        Serial.println("RFID reader initialized");
    }

    // Show we're alive by flashing white
    leds[0] = CRGB::White;
    FastLED.show();
    delay(500);
    leds[0] = CRGB::Black;
    FastLED.show();

    Serial.println("Setup complete - entering main loop");
}

void loop() {
    static unsigned long lastPrint = 0;
    static unsigned long lastRFIDCheck = 0;
    static unsigned long lastHeartbeat = 0;
    const unsigned long PRINT_INTERVAL = 100;      // Print every 100ms
    const unsigned long RFID_CHECK_INTERVAL = 500; // Check RFID every 500ms
    const unsigned long HEARTBEAT_INTERVAL = 2000; // Heartbeat every 2 seconds
    
    // Heartbeat to confirm loop is running
    if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        Serial.println("\nHeartbeat - system running");
        lastHeartbeat = millis();
    }

    // Read distance if new sample available
    if (ultrasonic.newSampleAvailable()) {
        float distance = ultrasonic.getDistanceMm();
        
        // Print reading every PRINT_INTERVAL ms
        if (millis() - lastPrint >= PRINT_INTERVAL) {
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" mm");
            lastPrint = millis();
            
            // Constrain distance to our range
            float constrainedDistance = constrain(distance, MIN_DISTANCE, MAX_DISTANCE);
            
            // Map distance to LED brightness (inverted - closer = brighter)
            uint8_t brightness = map(constrainedDistance, MIN_DISTANCE, MAX_DISTANCE, 255, 0);
            
            // Set pure red color with mapped brightness
            leds[0] = CRGB(brightness, 0, 0);
            FastLED.show();
        }
    }

    // Check for RFID tags
    if (millis() - lastRFIDCheck >= RFID_CHECK_INTERVAL) {
        Serial.println("Checking for RFID tag...");
        if (rfid.tagPresent()) {
            Serial.println("Tag detected!");
            String text = rfid.readText();
            if (text.length() > 0) {
                Serial.print("Message read: ");
                Serial.println(text);
            } else {
                Serial.println("No text found on tag");
            }
            // Flash LED white to indicate tag read
            leds[0] = CRGB::White;
            FastLED.show();
            delay(100);
        }
        lastRFIDCheck = millis();
    }

    delay(10); // Short delay to prevent overwhelming I2C bus
}