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

        // Samples one accel/gyro pair over I2C. Prefer this plus the last*()
        // accessors in a control loop: the get*() calls each trigger their own
        // read, so using two of them doubles the bus traffic and returns
        // readings from two different instants.
        void read();
        sensors_vec_t lastAcceleration();
        sensors_vec_t lastGyro();

        // Averages the gyro while the craft is held still to measure its zero
        // offset, which is then subtracted from every reading. The raw offset is
        // a few degrees per second; integrated by the filter it walks the
        // attitude estimate away from level, so the controller holds a tilt the
        // craft does not actually have. Must be called with the craft stationary.
        void calibrateGyro(uint16_t samples = 500);
        sensors_vec_t getGyroBias();

        void dumpSensorDetails();
    private:
        Adafruit_MPU6050 _mpu;
        sensors_vec_t _gyroBias;
        sensors_event_t _acceleration;
        sensors_event_t _gyro;
        sensors_event_t _temperature;
        void displayAccelerometerRange();
        void displayGyroRange();
        void displayFilterBandwidth();
};
