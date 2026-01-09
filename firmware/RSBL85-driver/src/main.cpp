#include <Arduino.h>
#include "RSB.h"

SMS_STS servo;

void setup()
{
    Serial.begin(115200);
    Serial.println("Testing SyncRotateByAngle - Synchronized Rotation Control");

    servo.begin(1000000, 18, 19, -1);
    delay(1000);

    //Serial.println("Setting servos to Wheel Mode...");
    //servo.WheelMode(0);
    // servo.WheelMode(1);
    // servo.WheelMode(2);
    //delay(500);

    servo.unLockEprom(1);
    servo.writeByte(1, SMS_STS_MODE, 3);  // Step mode
    servo.LockEprom(1);
    delay(100);

    for(uint8_t id = 0; id <= 2; id++) {
        servo.unLockEprom(id);
        servo.writeByte(id, SMS_STS_MODE, 3);  // Step mode
        servo.LockEprom(id);
        servo.EnableTorque(id, 1);
        delay(100);
    }

    Serial.println("Step Mode setup complete.");
}


void loop()
{
    // TESTING Control of multiple servos
    uint8_t ids[] = {0, 1, 2};
    int16_t angles[] = {0, 0, 0};
    uint8_t accs[] = {255, 255, 255};
//
    angles[0] = -180;
    angles[1] = 90;
    angles[2] = 360;
    //servo.SyncMoveToAngle(ids, 3, angles, 4000, accs);
    //delay(1000);

    //angles[0] = 360;
    //angles[1] = 180;
    //servo.SyncMoveToAngle(ids, 2, angles, 2000, accs);
    //delay(1000);

    servo.MoveToAngle(0, 180, 2000, 255);
    servo.MoveToAngle(2, 180, 4000, 255);
    delay(1000);
    servo.MoveToAngle(0, -180, 2000, 255);
    servo.MoveToAngle(2, 180, 5000, 255);
    delay(1000);
}
  
