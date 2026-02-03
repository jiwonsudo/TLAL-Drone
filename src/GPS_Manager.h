#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <TinyGPS++.h>

class GPSManager {
private:
    TinyGPSPlus gps;
    HardwareSerial& gpsSerial;

public:
    // ESP32의 Serial2를 기본으로 사용
    GPSManager(HardwareSerial& serial) : gpsSerial(serial) {}

    void begin() {
        // 대부분의 GPS 기본 BaudRate는 9600입니다.
        // 핀 16(RX), 17(TX) 사용
        gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
        Serial.println(">>> GPS UART (16, 17) Started at 9600bps");
    }

    void update() {
        while (gpsSerial.available() > 0) {
            gps.encode(gpsSerial.read());
        }
    }

    void displayInfo() {
        if (gps.location.isValid()) {
            Serial.printf("LAT: %.6f | LNG: %.6f | ALT: %.1fm | SAT: %d\n",
                          gps.location.lat(),
                          gps.location.lng(),
                          gps.altitude.meters(),
                          gps.satellites.value());
        } else {
            // 위성이 아직 안 잡혔을 때
            Serial.printf("GPS Waiting for Satellites... [SAT: %d]\n", gps.satellites.value());
        }
    }
};

#endif