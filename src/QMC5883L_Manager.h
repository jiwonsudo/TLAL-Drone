#ifndef QMC5883L_MANAGER_H
#define QMC5883L_MANAGER_H

#include <Arduino.h>
#include <Wire.h>

class QMC5883LManager {
private:
    const uint8_t ADDR = 0x2C; // 스캐너로 확인된 주소
    
    // 지원님 고유 보정값
    const float X_MIN_FIXED = -100.0, X_MAX_FIXED = -39.0;
    const float Y_MIN_FIXED = -300.0, Y_MAX_FIXED = -270.0;
    const float NORTH_OFFSET = -271.0; 
    
    float filteredX = 0, filteredY = 0, lastHeading = 0;
    bool firstRun = true;

public:
    void begin() {
        // 1. Soft Reset
        writeRegister(0x0A, 0x80);
        delay(50);
        
        // 2. Period 설정 (0 고정 방지 핵심!)
        writeRegister(0x0B, 0x01);
        delay(50);

        // 3. Continuous Mode (200Hz, 8G)
        writeRegister(0x09, 0x1D);
        delay(50);
        
        Serial.println(">>> QMC5883L (0x2C) Wake-up Success");
    }

    void update() {
        Wire.beginTransmission(ADDR);
        Wire.write(0x06); // Status check
        if (Wire.endTransmission() != 0) return;

        Wire.requestFrom(ADDR, (uint8_t)1);
        if (Wire.available() && (Wire.read() & 0x01)) {
            Wire.beginTransmission(ADDR);
            Wire.write(0x00);
            Wire.endTransmission();
            
            if (Wire.requestFrom(ADDR, (uint8_t)6) >= 6) {
                int16_t x = (int16_t)(Wire.read() | (Wire.read() << 8));
                int16_t y = (int16_t)(Wire.read() | (Wire.read() << 8));
                int16_t z = (int16_t)(Wire.read() | (Wire.read() << 8));

                if (x == 0 && y == 0) return;

                float rawX = (float)x; 
                float rawY = (float)y;

                if (firstRun) { filteredX = rawX; filteredY = rawY; firstRun = false; }
                filteredX = (0.1f * rawX) + (0.9f * filteredX);
                filteredY = (0.1f * rawY) + (0.9f * filteredY);

                float offsetX = (X_MAX_FIXED + X_MIN_FIXED) / 2.0f;
                float offsetY = (Y_MAX_FIXED + Y_MIN_FIXED) / 2.0f;
                float rangeX = (X_MAX_FIXED - X_MIN_FIXED) / 2.0f;
                float rangeY = (Y_MAX_FIXED - Y_MIN_FIXED) / 2.0f;

                float calX = (filteredX - offsetX) / (rangeX != 0 ? rangeX : 1);
                float calY = (filteredY - offsetY) / (rangeY != 0 ? rangeY : 1);

                // 축 매핑: X=Pitch, Y=Roll
                float heading = atan2(calY, calX) * 180.0f / PI + NORTH_OFFSET; 
                while (heading < 0) heading += 360;
                while (heading >= 360) heading -= 360;

                float delta = heading - lastHeading;
                if (delta > 180) delta -= 360;
                if (delta < -180) delta += 360;
                lastHeading += 0.2f * delta;
                
                if (lastHeading < 0) lastHeading += 360;
                if (lastHeading >= 360) lastHeading -= 360;
            }
        }
    }

    float getHeading() { return lastHeading; }

private:
    void writeRegister(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(ADDR);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }
};
#endif