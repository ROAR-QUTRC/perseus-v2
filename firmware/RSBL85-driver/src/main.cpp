// #include <Arduino.h>

// #include <hi_can_twai.hpp>

// #include <Arduino.h>
// #include "RSB.h"

// SMS_STS st;

// void setup()
// {
//   // Debug serial
//   Serial.begin(115200);

//   // Initialise Servo Serial
//   // Baud rate: 1000000
//   // RX Pin: 18
//   // TX Pin: 19
//   // Direction Pin: -1
//   st.begin(1000000, 18, 19, -1);

//   delay(1000);
//   Serial.println("Servo Driver Initialised");

//   st.WritePosEx(1, 0, 0, 0);
// }

// int pos = 0;

// void loop()
// {
//   // ID: 0, Position: 1000, Speed: 0, Acc: 0
//   // st.WritePosEx(1, 4096, 0, 0);
//   delay(1000);
//   // delay(1000 * 60 * 60 * 24); // 24 hours delay

//   // st.WritePosEx(1, 2000, 0, 0);
//   // delay(1000);

//   Serial.print("Reading Servo Status");
//   Serial.print(", Position: ");
//   Serial.print(st.ReadPos(1));
//   Serial.print(", Speed: ");
//   Serial.print(st.ReadSpeed(1));
//   Serial.print(", Load: ");
//   Serial.print(st.ReadLoad(1));
//   Serial.print(", Voltage: ");
//   Serial.print(st.ReadVoltage(1));
//   Serial.print(", Temperature: ");
//   Serial.print(st.ReadTemper(1));
//   Serial.print(", Moving: ");
//   Serial.println(st.ReadMove(1));
// }

#include <Arduino.h>

// #include <hi_can_twai.hpp>

#include "RSB.h"

SMS_STS st;

void setup()
{
    // Debug serial
    Serial.begin(115200);

    // Initialise Servo Serial
    // Baud rate: 1000000
    // RX Pin: 18
    // TX Pin: 19
    // Direction Pin: -1
    st.begin(1000000, 18, 19, -1);

    delay(1000);
    Serial.println("Servo Driver Initialised");
}

void loop()
{
    st.WritePosEx(1, 1000, 0xffff / 2, 0);
    delay(2000);
    st.WritePosEx(1, 3000, 0xffff / 2, 0);
    delay(2000);
}