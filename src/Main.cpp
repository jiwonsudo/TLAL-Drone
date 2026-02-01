#include <Arduino.h>
#include <Wire.h>
#include <IBusBM.h>
#include <BluetoothSerial.h>
#include <TinyGPS++.h>
#include "MPU6050_Manager.h"
#include "Motor_Manager.h"

// 객체 선언
BluetoothSerial SerialBT;
TinyGPSPlus gps;
IBusBM ibus;
MPU6050Manager mpuManager;
MotorManager motorManager;

// 제어 상수 및 변수
float kp = 2.5f, kd = 0.15f, ki = 0.05f, ky = 0.6f;
float rollErrorI = 0, pitchErrorI = 0;
unsigned long lastLoopTime = 0;
const int LOOP_INTERVAL = 5; 
const float DT = 0.005f;

void setup() {
    Serial.begin(115200);
    
    // 1. 블루투스 초기화 (맥북에서 TRI_Drone으로 페어링 필수)
    if(!SerialBT.begin("TRI_Drone")) {
        Serial.println("BT Init Failed!");
    }

    // 2. 하드웨어 버스 시작
    Wire.begin(21, 22);
    Wire.setClock(400000);
    
    // 3. 통신 포트 설정
    Serial1.begin(115200, SERIAL_8N1, 4, 5);   // iBus (RX=4)
    Serial2.begin(9600, SERIAL_8N1, 16, 17);   // GPS (RX=16)
    
    ibus.begin(Serial1, 1);
    motorManager.begin();
    mpuManager.begin();

    Serial.println(">>> TRI_Drone System Ready.");
    SerialBT.println(">>> TRI_Drone Bluetooth Telemetry Connected.");
}

void loop() {
    // GPS 데이터 수신
    while (Serial2.available() > 0) {
        gps.encode(Serial2.read());
    }

    unsigned long currentTime = millis();

    // 200Hz 제어 루프
    if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
        lastLoopTime = currentTime;
        mpuManager.update();

        int rawT = ibus.readChannel(2);
        int rawR = ibus.readChannel(0);
        int rawP = ibus.readChannel(1);
        int rawY = ibus.readChannel(3);

        // [데이터 송신 로직] 1초마다 블루투스로 데이터 출력
        static unsigned long lastBT = 0;
        if (currentTime - lastBT > 1000) {
            if (gps.location.isValid()) {
                SerialBT.printf("[GPS] Lat: %.6f, Lon: %.6f, Alt: %.1fm, Sats: %d\n", 
                                gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.satellites.value());
            } else {
                SerialBT.printf("[GPS] Searching... (Sats: %d)\n", gps.satellites.value());
            }
            // 현재 비행 데이터 출력
            SerialBT.printf("[IMU] Pitch: %.2f, Roll: %.2f | [CMD] Thr: %d\n", 
                            mpuManager.getAngleX(), mpuManager.getAngleY(), rawT);
            SerialBT.println("------------------------------------");
            lastBT = currentTime;
        }

        // 시동 대기 상태
        if (rawT < 1050) {
            motorManager.stopAll();
            rollErrorI = 0; pitchErrorI = 0;
            return;
        }

        // PID 및 모터 업데이트 로직 (기존 유지)
        float rollError  = mpuManager.getAngleY() - (-(rawR - 1500) * 0.04f);
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