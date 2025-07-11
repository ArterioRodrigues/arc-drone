#pragma once
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Default MPU6050 Pinout is GPIO 21 (SDA) and GPIO 22 (SCL) for ESP32

#define FAIL_TO_LOAD_MPU6050 true

class MPU6050 {
    public: 
        MPU6050();
        MPU6050(int sdaPin, int sclPin);
        
        sensort_event_t getAcceleration();
        sensort_event_t getGyro();
        sensort_event_t getTemperature();

        void dumpSensorDetails();
    private:
        Adafruit_MPU6050 _mpu;
        sensort_event_t _acceleration;
        sensort_event_t _gyro;
        sensort_event_t _temperature;
        void displayAccelerometerRange();
        void displayGyroRange();
        void displayFilterBandwidth();
}