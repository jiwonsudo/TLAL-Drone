#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <Arduino.h>

class BatteryManager {
private:
    const int PIN;
    const float DIVIDER_RATIO;
    float currentVoltage = 0.0f;

public:
    // R1=10k, R2=3k 설정 반영
    BatteryManager(int pin = 34) 
        : PIN(pin), DIVIDER_RATIO((10000.0f + 3000.0f) / 3000.0f) {}

    void begin() {
        analogReadResolution(12);
        pinMode(PIN, INPUT);
    }

    void update() {
        int raw = analogRead(PIN);
        // ESP32 ADC 오차 보정을 위해 3.3 대신 3.4~3.45를 넣는 것이 더 정확할 수 있음
        float pinVoltage = (raw * 3.4f) / 4095.0f; 
        float measured = pinVoltage * DIVIDER_RATIO;

        if (currentVoltage == 0.0f) currentVoltage = measured;
        else currentVoltage = currentVoltage * 0.92f + measured * 0.08f; // 노이즈 필터 강화
    }

    float getVoltage() { return currentVoltage; }

    // 3S 배터리 잔량 (%) 계산
    int getPercentage() {
        // 3S 기준: 12.6V(100%), 10.5V(0%)
        float pct = (currentVoltage - 10.5f) / (12.6f - 10.5f) * 100.0f;
        return (int)constrain(pct, 0, 100);
    }
};

#endif