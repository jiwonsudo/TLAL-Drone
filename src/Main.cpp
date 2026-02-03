#include <Arduino.h>
#include <Wire.h>
#include <IBusBM.h>
#include <TinyGPS++.h>
#include "MPU6050_Manager.h"
#include "Motor_Manager.h"
#include "Battery_Manager.h"

// 객체 선언
TinyGPSPlus gps;
IBusBM ibus;
MPU6050Manager mpuManager;
MotorManager motorManager;
BatteryManager batteryManager(34);

// 제어 상수 및 변수
float kp = 2.5f, kd = 0.15f, ki = 0.05f, ky = 0.6f;
float rollErrorI = 0, pitchErrorI = 0;
unsigned long lastLoopTime = 0;
const int LOOP_INTERVAL = 5;
const float DT = 0.005f;

void setup()
{
    // 1. 시리얼 모니터 시작 (속도 115200 확인 필수)
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n[SYSTEM] TRI_Drone Booting...");

    // 2. I2C 버스 시작
    Serial.print("[INIT] GYRO (MPU6050)... ");
    Wire.begin(21, 22);
    Wire.setClock(400000);
    mpuManager.begin();
    Serial.println("GYRO OK");

    // 3. 통신 포트 설정
    Serial.print("[INIT] iBus (Serial1)... ");
    Serial1.begin(115200, SERIAL_8N1, 4, -1); // TX는 사용 안하므로 -1 설정 (충돌 방지)
    ibus.begin(Serial1, 1);
    Serial.println("IBUS OK");

    Serial.print("[INIT] GPS (Serial2)... ");
    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    Serial.println("GPS OK");

    // 4. 모터 매니저 시작
    Serial.print("[INIT] Motors... ");
    motorManager.begin();
    Serial.println("Motor OK");

    Serial.print("[INIT] Battery Checker... ");
    batteryManager.begin();
    Serial.println("Battery Checker OK");

    Serial.println("[SYSTEM] All Systems Ready. Entering Loop...");
}

void loop()
{
    // GPS 데이터 수신
    while (Serial2.available() > 0)
    {
        gps.encode(Serial2.read());
    }

    unsigned long currentTime = millis();

    // 200Hz 제어 루프
    if (currentTime - lastLoopTime >= LOOP_INTERVAL)
    {
        lastLoopTime = currentTime;
        mpuManager.update();
        batteryManager.update();

        int rawT = ibus.readChannel(2);
        int rawR = ibus.readChannel(0);
        int rawP = ibus.readChannel(1);
        int rawY = ibus.readChannel(3);

        // [디버깅 출력] 0.5초마다 데이터 통합 출력
        static unsigned long lastLog = 0;
        if (currentTime - lastLog > 500) {
            // 1. GPS 정보 문자열 생성 (수신 전/후 구분)
            char gpsPos[48];
            if (gps.location.isValid()) {
                snprintf(gpsPos, sizeof(gpsPos), "%.6f,%.6f", gps.location.lat(), gps.location.lng());
            } else {
                snprintf(gpsPos, sizeof(gpsPos), "No Fix(%d)", gps.satellites.value());
            }

            // 2. 통합 시리얼 출력 (중복 제거 및 위도/경도 추가)
            Serial.printf("[BATT] %.2fV(%d%%) | [GPS] %s | [IMU] P:%.1f R:%.1f | [THR] %d\n", 
                          batteryManager.getVoltage(),
                          batteryManager.getPercentage(),
                          gpsPos,
                          mpuManager.getAngleX(), 
                          mpuManager.getAngleY(), 
                          rawT);
            
            lastLog = currentTime;
        }

        // 시동 대기 상태 (Throttle이 낮으면 모터 정지)
        if (rawT < 1050)
        {
            motorManager.stopAll();
            rollErrorI = 0;
            pitchErrorI = 0;
            return;
        }

        // PID 및 모터 제어 로직
        float rollError = mpuManager.getAngleY() - (-(rawR - 1500) * 0.04f);
        float pitchError = mpuManager.getAngleX() - ((rawP - 1500) * 0.04f);
        float targetYawRate = (rawY - 1500) * 0.8f;

        rollErrorI = constrain(rollErrorI + rollError * DT, -150, 150);
        pitchErrorI = constrain(pitchErrorI + pitchError * DT, -150, 150);

        float rAdj = (rollError * kp) + (rollErrorI * ki) + (mpuManager.getGyroY() * kd);
        float pAdj = (pitchError * kp) + (pitchErrorI * ki) + (mpuManager.getGyroX() * kd);
        float yAdj = (mpuManager.getGyroZ() - targetYawRate) * ky;

        motorManager.update(rawT, rAdj, pAdj, yAdj);
    }
}