#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>
#include <ESP32Servo.h>

class MotorManager {
private:
    Servo motors[4];
    const int motorPins[4] = {27, 26, 33, 25};
    const int minIdle = 1120; // 최저 회전 속도

public:
    void begin() {
        for (int i = 0; i < 4; i++) {
            motors[i].attach(motorPins[i], 1000, 2000);
            motors[i].writeMicroseconds(1000);
        }
    }

    void update(int throttle, float rAdj, float pAdj, float yAdj) {
        // 믹싱 계산 (FL, FR, RL, RR)
        int m0 = throttle - (int)pAdj + (int)rAdj - (int)yAdj; // FL
        int m1 = throttle - (int)pAdj - (int)rAdj + (int)yAdj; // FR
        int m2 = throttle + (int)pAdj + (int)rAdj + (int)yAdj; // RL
        int m3 = throttle + (int)pAdj - (int)rAdj - (int)yAdj; // RR

        m0 = constrain(m0, minIdle, 2000);
        m1 = constrain(m1, minIdle, 2000);
        m2 = constrain(m2, minIdle, 2000);
        m3 = constrain(m3, minIdle, 2000);

        motors[0].writeMicroseconds(m0);
        motors[1].writeMicroseconds(m1);
        motors[2].writeMicroseconds(m2);
        motors[3].writeMicroseconds(m3);
    }

    void stopAll() {
        for (int i = 0; i < 4; i++) motors[i].writeMicroseconds(1000);
    }
};

#endif