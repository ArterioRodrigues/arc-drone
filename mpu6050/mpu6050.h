#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Default MPU6050 Pinout is GPIO 21 (SDA) and GPIO 22 (SCL) for ESP32

#define FAIL_TO_LOAD_MPU6050 true

class MPU6050 {
    public: 
        MPU6050();
        void setup();
        MPU6050(int sdaPin, int sclPin);
        
        sensors_vec_t getAcceleration();
        sensors_vec_t getGyro();
        float getTemperature();

        void dumpSensorDetails();
    private:
        Adafruit_MPU6050 _mpu;
        sensors_event_t _acceleration;
        sensors_event_t _gyro;
        sensors_event_t _temperature;
        void displayAccelerometerRange();
        void displayGyroRange();
        void displayFilterBandwidth();
};
