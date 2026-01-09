#include <Arduino.h>

// #include <board_support.hpp>
#include <chrono>
// #include <hi_can_twai.hpp>
#include <optional>
#include <thread>
#include "RSB.h"

SMS_STS servo;

void setup()
{
 Serial.begin(115200);
    Serial.println("Testing SyncRotateByAngle - Synchronized Rotation Control");

    // intialise RSBL85 driver
    servo.begin(1000000, 18, 19, -1);
    delay(1000);
    // intialise servos 1 and 2 to Wheel Mode & torque
    Serial.println("Setting servos to Wheel Mode...");
    servo.WheelMode(0);
    servo.WheelMode(1);
    delay(500);

    Serial.println("Enabling torque...");
    servo.EnableTorque(0, 1);
    servo.EnableTorque(1, 1);
    delay(500);

    Serial.println("Setup complete. Starting rotation tests...");
    delay(1000);
}

void loop()
{
  delay(1)
}