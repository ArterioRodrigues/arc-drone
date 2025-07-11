#include "mpu6050.h"

void MPU6050::displayAccelerometerRange() {
    Serial.print("Accelerometer range set to: ");
    switch (this->_mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
}

void MPU6050::displayGyroRange() {
    Serial.print("Gyro range set to: ");
    switch (this->_mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
    case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
    case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
    case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
  }
}

void MPU6050::displayFilterBandwidth() {
    Serial.print("Filter bandwidth set to: ");
    switch (this->_mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
    case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
    case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
    case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
    case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
    case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
    case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
  }
}

void MPU6050::setup() {
    Serial.println("Searching for MPU6050 setup...");
    Serial.println("Initializing MPU6050 with custom SDA 20 and SCL 21 pins...");
    if (!this->_mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip....");
        Serial.println("Please check the wiring and try again.");
        while(FAIL_TO_LOAD_MPU6050) {
            delay(10);
        }
    }

    Serial.println("MPU6050 Found!");

    this->_mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    this->_mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    this->_mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    displayAccelerometerRange();
    displayGyroRange();
    displayFilterBandwidth();

    this->_mpu.getEvent(&this->_acceleration, &this->_gyro, &this->_temperature);
}

MPU6050::MPU6050(){}

MPU6050::MPU6050(int sdaPin, int sclPin) {
    Wire.begin(sdaPin, sclPin);
    Serial.println("Initializing MPU6050 with custom SDA and SCL pins...");
}

sensors_vec_t MPU6050::getAcceleration() {
    this->_mpu.getEvent(&_acceleration, &_gyro, &_temperature);
    return this->_acceleration.acceleration;
}
sensors_vec_t MPU6050::getGyro() {
    this->_mpu.getEvent(&_acceleration, &_gyro, &_temperature);
    return this->_gyro.gyro;
}
float MPU6050::getTemperature() {
    this->_mpu.getEvent(&_acceleration, &_gyro, &_temperature);
    return this->_temperature.temperature;
}

void MPU6050::dumpSensorDetails() {
    this->_mpu.getEvent(&_acceleration, &_gyro, &_temperature);
    Serial.print("Acceleration X: ");
    Serial.print(this->_acceleration.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(this->_acceleration.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(this->_acceleration.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(this->_gyro.gyro.x);
    Serial.print(", Y: ");
    Serial.print(this->_gyro.gyro.y);
    Serial.print(", Z: ");
    Serial.print(this->_gyro.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(this->_temperature.temperature);
    Serial.println(" degC");

    Serial.println("");
}
