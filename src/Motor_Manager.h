#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>
#include <ESP32Servo.h>

class MotorManager {
private:
    Servo motors[4];
    const int motorPins[4] = {12, 13, 14, 27};
    const int minIdle = 1120; 

public:
    void begin() {
        // [수정] ESP32 하드웨어 타이머 4개를 순차적으로 강제 할당
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);

        for (int i = 0; i < 4; i++) {
            motors[i].setPeriodHertz(50);
            motors[i].attach(motorPins[i], 1000, 2000);
            motors[i].writeMicroseconds(1000); // 아밍 신호
            delay(100); 
        }
        Serial.println("Motor Timers Allocated.");
    }

    void update(int throttle, float rAdj, float pAdj, float yAdj) {
        int m[4];
        m[0] = throttle + (int)pAdj + (int)rAdj - (int)yAdj; // FL
        m[1] = throttle + (int)pAdj - (int)rAdj + (int)yAdj; // FR
        m[2] = throttle - (int)pAdj + (int)rAdj + (int)yAdj; // RL
        m[3] = throttle - (int)pAdj - (int)rAdj - (int)yAdj; // RR

        for (int i = 0; i < 4; i++) {
            if (throttle < 1050) m[i] = 1000;
            else m[i] = constrain(m[i], minIdle, 2000);
            motors[i].writeMicroseconds(m[i]);
        }
    }

    void stopAll() {
        for (int i = 0; i < 4; i++) motors[i].writeMicroseconds(1000);
    }
};

#endif