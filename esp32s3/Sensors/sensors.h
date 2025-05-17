// sensors.h
#ifndef SENSORS_H
#define SENSORS_H

// Include dependencies
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h>
#include "bmm150.h"
#include "bmm150_defs.h"
#include "Adafruit_VL53L0X.h"

// ----- BAROMETER -----
extern Adafruit_BMP280 bmp;
void init_baro();
int barometer_reading();

// ----- TOF SENSOR -----
extern Adafruit_VL53L0X lox;

void init_tof();
int tof_reading();

// ----- COMPASS -----
extern BMM150 bmm;
extern bmm150_mag_data value, value_offset;
extern float Heading[3];
extern float Degrees[3];
void init_campass();
void campass_calibrate(uint32_t timeout);
int campass_reading();

// ----- MPU -----
extern TwoWire I2CBus;
extern MPU9250_asukiaaa imu1;
extern MPU9250_asukiaaa imu2;
extern float axBias, ayBias, azBias;
extern float gxBias, gyBias, gzBias;
extern unsigned long prevTime;

class KalmanFilter {
public:
  float Q_angle = 0.001;
  float Q_bias = 0.003;
  float R_measure = 0.03;

  float angle = 0.0; // Reset angle
  float bias = 0.0;

  float P[2][2] = { {0, 0}, {0, 0} };

  float getAngle(float newAngle, float newRate, float dt);
};

extern KalmanFilter kalmanPitch1, kalmanRoll1;
extern KalmanFilter kalmanPitch2, kalmanRoll2;

void init_mpu();
int* mpu_reading();

#endif // SENSORS_H
