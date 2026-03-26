#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

#include <chrono>
#include <optional>
#include <vector>

#include "RSB.h"

#define MAX_ENCODER_COUNT 4095

SMS_STS servo;

constexpr int TILT = 1;
constexpr int PAN = 2;
constexpr int ELBOW = 3;

std::vector<int> past_positions(3, 0);
std::vector<int> offsets(3, 0);
std::vector<double> target_positions(3, 0.0);

void setup()
{
    Serial.begin(115200);
    Serial.println("RSBL85 CAN Bridge - Initializing");

    servo.begin(1000000, 18, 19, -1);
    Serial2.setRxBufferSize(1024);
    Serial2.setTxBufferSize(1024);
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    delay(1000);

    Serial.println("Motor 1 online: " + String(TILT == servo.Ping(TILT) ? "true" : "false"));
    Serial.println("Motor 2 online: " + String(PAN == servo.Ping(PAN) ? "true" : "false"));
    Serial.println("Motor 3 online: " + String(ELBOW == servo.Ping(ELBOW) ? "true" : "false"));

    while (servo.ReadPos(ELBOW) == -1) {
        //wait for motors to come online
    }
    past_positions[TILT - 1] = servo.ReadPos(TILT);
    past_positions[PAN - 1] = servo.ReadPos(PAN);
    past_positions[ELBOW - 1] = servo.ReadPos(ELBOW);

    // read current values and store them as offsets

    // set the offsets as the startup positions so that servos dont zero on startup
    target_positions[TILT - 1] = 0;
    target_positions[PAN - 1] = 0;
    target_positions[ELBOW - 1] = 0;
}

std::vector<double> positions(3, 0);
std::vector<int16_t> full_rev_count(3, 0);
void write_angle(int id, double angle, double speed)
{
    // add offset to angle so that all positions are relative to starting position
    target_positions[id - 1] = angle;
    // TODO: set max speed
}

const std::vector<uint8_t> ids = {TILT, PAN, ELBOW};
// const std::vector<uint8_t> ids = {ELBOW};

/*
UART Protocol:
- Start character: <
- End character: >
- Packet label:
    a -> current angles -> <a,tilt_angle,pan_angle,elbow_angle>
    t -> target angles  -> <t,id,target_angle,speed>
- all values are doubles except for id which is a uint8_t
*/

void loop()
{
    // Serial.printf("offset %d, %d, %d\n", offsets[TILT - 1], offsets[PAN - 1], offsets[ELBOW - 1]);
    std::vector<uint8_t> data;

    bool start_byte_found = false;
    bool frame_started = false;

    String s = "";

    while (Serial2.available() > 0)
    {
        uint8_t byte = Serial2.read();
        if (byte == '<')
        {  // Start byte so reset the string
            s = "";
        }
        else if (byte == '>')
        {
            // message complete so parse it
            if (!s.isEmpty() && s[0] == 't')
            {
                // target angle command
                int id_end = s.indexOf(',', 2);
                int angle_end = s.indexOf(',', id_end + 1);
                if (id_end == -1 || angle_end == -1)
                {
                    Serial.println("Invalid command format");
                }
                else
                {
                    uint8_t id = s.substring(2, id_end).toInt();
                    double target_angle = s.substring(id_end + 1, angle_end).toDouble();
                    double speed = s.substring(angle_end + 1).toDouble();
                    write_angle(id, target_angle, speed);
                    Serial.println("Received command: id=" + String(id) + ", target_angle=" + String(target_angle) + ", speed=" + String(speed));
                }
            }
        }
        else
        {
            s += (char)byte;
        }
    }
    Serial2.print("<a," + String(positions[TILT - 1]) + "," + String(positions[PAN - 1]) + "," + String(positions[ELBOW - 1]) + ">");

    for (unsigned i = 0; i < ids.size(); i++)
    {
        // id starts at 1 but vector index starts at 0
        const int id = ids[i] - 1;
        const int read_pos = servo.ReadPos(id + 1);
        // Serial.printf("id: %d, read_pos: %d\n", id + 1, read_pos);
        const int diff = read_pos - past_positions[id];

        if (abs(diff) > 2000)
        {  // If the jump is greater than half a revolution then the encoder has wrapped around
            bool clockwise = diff < 0;
            full_rev_count[id] += clockwise ? 1 : -1;
        }

        past_positions[id] = read_pos;

        const bool negative = full_rev_count[id] < 0;
        const int raw_angle = MAX_ENCODER_COUNT * ((negative ? 1 : 0) + full_rev_count[id]) + (negative ? (MAX_ENCODER_COUNT - read_pos) * -1 : read_pos);
        positions[id] = ((double)raw_angle / (double)MAX_ENCODER_COUNT) * 360.0;  // degrees
        // * PI * 2.0; // radians

        constexpr double MAX_SPEED = 30000.0;
        const double error = target_positions[id] - positions[id];

        // Serial.printf("id: %d, read_pos: %d, full_rev_count: %d, raw_angle: %d, position: %f, target_position: %f, error: %f\n",
        //               id + 1, read_pos, full_rev_count[id], raw_angle, positions[id], target_positions[id], error);
        if (abs(error) <= 1)
        {
            servo.WriteSpe(id + 1, 0, 0);
        }
        else if (abs(error) < 360)
        {  // Reduce speed when 1 revolution from target
            servo.WriteSpe(id + 1, error / 360.0 * MAX_SPEED, 0);
        }
        else
            servo.WriteSpe(id + 1, error > 0 ? MAX_SPEED : -MAX_SPEED, 0);
    }

    delay(1);

    Serial.printf("online: [%s, %s, %s], Target angles [%f, %f, %f], Motor loads -> Tilt: %d, Pan: %d, Elbow: %d\n",
                  servo.Ping(TILT) == TILT ? "true" : "false",
                  servo.Ping(PAN) == PAN ? "true" : "false",
                  servo.Ping(ELBOW) == ELBOW ? "true" : "false",
                  target_positions[TILT - 1], target_positions[PAN - 1], target_positions[ELBOW - 1],
                  servo.ReadLoad(TILT), servo.ReadLoad(PAN), servo.ReadLoad(ELBOW));
}