#include "main.hpp"

#include <Arduino.h>

#include <hi_can.hpp>

void setup()
{
    hi_can::Packet packet;
    Serial.begin(115200);
}
void loop()
{
    delay(100);
}