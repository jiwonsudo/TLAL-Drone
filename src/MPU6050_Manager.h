#ifndef MPU6050_MANAGER_H
#define MPU6050_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

class MPU6050Manager {
private:
    MPU6050 mpu;

public:
    MPU6050Manager() : mpu(Wire) {}

    void begin() {
        // [수정] main에서 Wire.begin()이 이미 호출되었으므로 바로 시작
        byte status = mpu.begin();
        if(status != 0) {
            Serial.println("!!! MPU6050 Connection Failed !!!");
            while(1); 
        }

        // 하드웨어 필터 설정
        Wire.beginTransmission(0x68);
        Wire.write(0x1A); 
        Wire.write(0x03); 
        Wire.endTransmission();

        Serial.println("Quick Calibration... DO NOT MOVE");
        mpu.calcOffsets(); 
        Serial.println("MPU6050 Ready!");
    }

    void update() { mpu.update(); }
    float getAngleX() { return mpu.getAngleX(); } // Pitch
    float getAngleY() { return mpu.getAngleY(); } // Roll
    float getGyroX()  { return mpu.getGyroX(); }
    float getGyroY()  { return mpu.getGyroY(); }
    float getGyroZ()  { return mpu.getGyroZ(); }
};

#endif