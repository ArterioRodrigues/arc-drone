#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "mpu6050/mpu6050.h"
Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  MPU6050 mpu();
}

void loop() {
    mpu.dumpSensorDetails();
    delay(500);
}