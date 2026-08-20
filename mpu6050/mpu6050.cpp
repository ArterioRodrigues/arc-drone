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
    // The on-chip low-pass costs group delay, and delay in a feedback path is
    // what makes corrections feel late and forces gains down to stay stable.
    // The 5 Hz setting delays gyro and accel by ~19 ms; 44 Hz costs ~4.9 ms for
    // the same job, and is the usual choice for a flight controller.
    this->_mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    // Default 100 kHz makes a 14-byte sample burst take ~1.5 ms of the loop.
    // The MPU6050 is rated for 400 kHz, which cuts that to ~0.4 ms.
    Wire.setClock(400000);

    displayAccelerometerRange();
    displayGyroRange();
    displayFilterBandwidth();

    this->_mpu.getEvent(&this->_acceleration, &this->_gyro, &this->_temperature);
}

MPU6050::MPU6050(){
    this->_gyroBias.x = 0;
    this->_gyroBias.y = 0;
    this->_gyroBias.z = 0;
}

MPU6050::MPU6050(int sdaPin, int sclPin) {
    Wire.begin(sdaPin, sclPin);
    this->_gyroBias.x = 0;
    this->_gyroBias.y = 0;
    this->_gyroBias.z = 0;
    Serial.println("Initializing MPU6050 with custom SDA and SCL pins...");
}

void MPU6050::read() {
    this->_mpu.getEvent(&_acceleration, &_gyro, &_temperature);
}

sensors_vec_t MPU6050::lastAcceleration() { return this->_acceleration.acceleration; }

sensors_vec_t MPU6050::lastGyro() {
    sensors_vec_t gyro = this->_gyro.gyro;
    gyro.x -= this->_gyroBias.x;
    gyro.y -= this->_gyroBias.y;
    gyro.z -= this->_gyroBias.z;
    return gyro;
}

void MPU6050::calibrateGyro(uint16_t samples) {
    Serial.println("Calibrating gyro - hold the craft still and level...");

    this->_gyroBias.x = 0;
    this->_gyroBias.y = 0;
    this->_gyroBias.z = 0;

    double sumX = 0, sumY = 0, sumZ = 0;
    for (uint16_t i = 0; i < samples; i++) {
        this->read();
        sumX += this->_gyro.gyro.x;
        sumY += this->_gyro.gyro.y;
        sumZ += this->_gyro.gyro.z;
        delay(2);
    }

    this->_gyroBias.x = sumX / samples;
    this->_gyroBias.y = sumY / samples;
    this->_gyroBias.z = sumZ / samples;

    Serial.printf("Gyro bias: x=%.4f y=%.4f z=%.4f rad/s\n", this->_gyroBias.x,
                  this->_gyroBias.y, this->_gyroBias.z);
}

sensors_vec_t MPU6050::getGyroBias() { return this->_gyroBias; }

sensors_vec_t MPU6050::getAcceleration() {
    this->read();
    return this->_acceleration.acceleration;
}
sensors_vec_t MPU6050::getGyro() {
    this->read();
    return this->lastGyro();
}
float MPU6050::getTemperature() {
    this->read();
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
