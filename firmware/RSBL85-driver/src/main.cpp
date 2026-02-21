#include <Arduino.h>
#include <math.h>

#include <chrono>
#include <optional>
#include <vector>

#include "RSB.h"

#define MAX_ENCODER_COUNT 4095

SMS_STS servo;

constexpr int TILT = 0;
constexpr int PAN = 2;
constexpr int ELBOW = 1;

std::vector<int> past_positions(3, 0);
std::vector<double> target_positions = { 25.0 * 0.0, 25.0 * 0.0, 25 * 0.0 };

void setup() {
    Serial.begin(115200);
    Serial.println("RSBL85 CAN Bridge - Initializing");

    servo.begin(1000000, 18, 19, -1);
    delay(1000);

    Serial.println("Motor 1 online: " + String(TILT == servo.Ping(TILT) ? "true" : "false"));
    Serial.println("Motor 2 online: " + String(PAN == servo.Ping(PAN) ? "true" : "false"));
    Serial.println("Motor 3 online: " + String(ELBOW == servo.Ping(ELBOW) ? "true" : "false"));

    past_positions[TILT] = servo.ReadPos(TILT);
    past_positions[PAN] = servo.ReadPos(PAN);
    past_positions[ELBOW] = servo.ReadPos(ELBOW);

    double tilt_target = 0.0;
    double pan_target = 0.0;
    double elbow_target = 0.0;

    target_positions[TILT] = 25.0 * tilt_target;
    target_positions[PAN] = 25.0 * pan_target;
    target_positions[ELBOW] = 25.0 * elbow_target;
}

std::vector<double> positions(3, 0);
std::vector<int16_t> full_rev_count(3, 0);
void write_angle(int id, double angle) {
    target_positions[id] = angle;
}

const int ids [] = { TILT, PAN, ELBOW };

void loop() {
    for (unsigned i = 0; i < 3; i++) {
        const int id = ids[i];
        const int read_pos = servo.ReadPos(id);
        const int diff = read_pos - past_positions[id];

        if (abs(diff) > 2000) {  // If the jump is greater than half a revolution then the encoder has wrapped around
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

        if (abs(error) <= 1) {
            servo.WriteSpe(id, 0, 0);
        } else if (abs(error) < 360) {  // Reduce speed when 1 revolution from target
            servo.WriteSpe(id, error / 360.0 * MAX_SPEED, 0);
        } else
            servo.WriteSpe(id, error > 0 ? MAX_SPEED : -MAX_SPEED, 0);

        const int key = Serial.read();
        if (key != -1) {
            switch (key) {
                case 'w':
                    target_positions[ELBOW] += 5.0;
                    break;
                case 's':
                    target_positions[ELBOW] -= 5.0;
                    break;
                case 'a':
                    target_positions[TILT] -= 5.0;
                    break;
                case 'd':
                    target_positions[TILT] += 5.0;
                    break;
            }
        }
    }

    Serial.printf("Motor loads -> Tilt: %d, Pan: %d, Elbow: %d\n", servo.ReadLoad(TILT), servo.ReadLoad(PAN), servo.ReadLoad(ELBOW));
}
