#include "mpu6050.h"
#include <Wire.h>
#include <MPU6050.h>

MPU6050Sensor::MPU6050Sensor() {}

void MPU6050Sensor::init() {
    Wire.begin();
    mpu.initialize();
}

void MPU6050Sensor::readSensorData() {
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
}

void MPU6050Sensor::getAcceleration(int16_t& ax, int16_t& ay, int16_t& az) {
    ax = this->ax;
    ay = this->ay;
    az = this->az;
}

void MPU6050Sensor::getRotation(int16_t& gx, int16_t& gy, int16_t& gz) {
    gx = this->gx;
    gy = this->gy;
    gz = this->gz;
}

void setup() {
    Serial.begin(115200);
    MPU6050Sensor mpu;
    mpu.init();
}

void loop() {
    MPU6050Sensor mpu;
    mpu.readSensorData();
    
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(ax, ay, az);
    mpu.getRotation(gx, gy, gz);
    
    // Print the sensor data
    Serial.print("Accel: ");
    Serial.print("X = "); Serial.print(ax);
    Serial.print(", Y = "); Serial.print(ay);
    Serial.print(", Z = "); Serial.println(az);
    
    Serial.print("Gyro: ");
    Serial.print("X = "); Serial.print(gx);
    Serial.print(", Y = "); Serial.print(gy);
    Serial.print(", Z = "); Serial.println(gz);
    
    delay(500);  // Adjust delay as needed
}
