#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <MPU6050.h>

class MPU6050Sensor {
private:
    MPU6050 mpu;
    int16_t ax, ay, az, gx, gy, gz;

public:
    MPU6050Sensor();
    void init();
    void readSensorData();
    void getAcceleration(int16_t& ax, int16_t& ay, int16_t& az);
    void getRotation(int16_t& gx, int16_t& gy, int16_t& gz);
};

#endif
