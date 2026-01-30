#include <Arduino.h>
#include <IBusBM.h>
#include "MPU6050_Manager.h"
#include "Motor_Manager.h"

IBusBM ibus;
MPU6050Manager mpuManager;
MotorManager motorManager;

// [PID 게인]
float kp = 4.5f;   
float kd = 0.15f;  
float ky = 0.6f;   

// [트림 보정] 
// 왼쪽으로 기운다면 이 값을 '음수(-)' 방향으로 더 키우세요 (예: -2.0, -3.0)
// 현재 오른쪽 잡는 힘은 강한데 왼쪽이 밀린다면, 기준점을 왼쪽으로 더 옮겨야 합니다.
float trimRoll = -3.5f; 
float trimPitch = 0.0f;

float rollAdjust = 0, pitchAdjust = 0, yawAdjust = 0;

unsigned long lastLoopTime = 0;
const int LOOP_INTERVAL = 5; 

void setup() {
    Serial.begin(115200);
    ibus.begin(Serial2);
    mpuManager.begin();
    motorManager.begin();
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
        lastLoopTime = currentTime;

        mpuManager.update();

        int rawT = ibus.readChannel(2);
        int rawR = ibus.readChannel(0);
        int rawP = ibus.readChannel(1);
        int rawY = ibus.readChannel(3);

        if (rawT < 1050) {
            motorManager.stopAll();
            rollAdjust = 0; pitchAdjust = 0; yawAdjust = 0;
            return;
        }

        // 데드밴드 처리
        int rErr = (abs(rawR - 1500) < 15) ? 0 : rawR - 1500;
        int pErr = (abs(rawP - 1500) < 15) ? 0 : rawP - 1500;
        int yErr = (abs(rawY - 1500) < 20) ? 0 : rawY - 1500;

        // 목표 각도 계산 (트림 포함)
        float targetRoll  = (-rErr * 0.04f) + trimRoll;
        float targetPitch = (pErr * 0.04f) + trimPitch;
        float targetYawRate = yErr * 0.8f;

        // PD 제어 계산
        // 현재 각도에서 목표 각도를 뺀 편차에 kp를 곱합니다.
        rollAdjust  = ((mpuManager.getAngleX() - targetRoll) * kp) + (mpuManager.getGyroX() * kd);
        pitchAdjust = ((mpuManager.getAngleY() - targetPitch) * kp) + (mpuManager.getGyroY() * kd);
        yawAdjust   = (mpuManager.getGyroZ() - targetYawRate) * ky;

        motorManager.update(rawT, rollAdjust, pitchAdjust, yawAdjust);
    }

    static unsigned long lastPrint = 0;
    if (currentTime - lastPrint > 200) {
        // 시리얼 모니터에서 X값이 0 근처인지 확인하세요. 
        // 바닥에 가만히 뒀을 때 X값이 trimRoll 값과 비슷하게 찍혀야 정상입니다.
        Serial.printf("X:%.1f Y:%.1f | R_Adj:%.0f P_Adj:%.0f\n", 
                      mpuManager.getAngleX(), mpuManager.getAngleY(), 
                      rollAdjust, pitchAdjust);
        lastPrint = currentTime;
    }
}