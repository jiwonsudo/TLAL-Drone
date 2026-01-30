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
        Wire.begin(21, 22);
        byte status = mpu.begin();
        if(status != 0) {
            Serial.println("!!! MPU6050 Connection Failed !!!");
            while(1);
        }
        Serial.println("Calibration: keep the drone level...");
        delay(2000);
        mpu.calcOffsets();
        Serial.println("Calibration Done!");
    }

    void update() { mpu.update(); }

    // Angle for P-control (Left: +, Right: -, Front: -, Back: +)
    float getAngleX() { return mpu.getAngleX(); }
    float getAngleY() { return mpu.getAngleY(); }

    // Gyro Rate for D-control (deg/s)
    float getGyroX()  { return mpu.getGyroX(); }
    float getGyroY()  { return mpu.getGyroY(); }
    float getGyroZ()  { return mpu.getGyroZ(); }
};

#endif