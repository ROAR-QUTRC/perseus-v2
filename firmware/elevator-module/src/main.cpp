#include <Arduino.h>

#define IN1 13
#define IN2 14
#define IN3 21
#define IN4 47

#define ENA 48
#define ENB 45

#define FORWARD 0b10
#define REVERSE 0b01
#define BRAKE 0

void actuate(int direction, int rate) 
{

    digitalWrite(IN1, direction > 0);
    digitalWrite(IN2, direction < 0);
    analogWrite(ENA, rate);
}


void setup()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    pinMode(IN4, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(ENB, OUTPUT);
}

void loop()
{
    for(int i = 0; i <= 255; i++)
    {
        actuate(1, i);
        delay(100);
    }
    for(int i = 255; i >= 0; i--)
    {
        actuate(1, i);
        delay(100);
    }
    // digitalWrite(IN1, 0);
    // digitalWrite(IN2, 0);
    // delay(2000);
    // digitalWrite(IN1, 1);
    // digitalWrite(IN2, 0);
    // digitalWrite(ENA, 1);
    // delay(5000);

}

