#include <MPU9250_asukiaaa.h>
#include <Wire.h>
#include <math.h>

MPU9250_asukiaaa imu1(0x68);  // GY-91 #1
MPU9250_asukiaaa imu2(0x69);  // GY-91 #2 with AD0 = HIGH
float axBias = 0, ayBias = 0, azBias = 0;
float gxBias = 0, gyBias = 0, gzBias = 0;

TwoWire I2CBus = TwoWire(0);

unsigned long prevTime = 0;

class KalmanFilter {
public:
  float Q_angle = 0.001;
  float Q_bias = 0.003;
  float R_measure = 0.03;

  float angle = 0.0; // Reset angle
  float bias = 0.0;

  float P[2][2] = { {0, 0}, {0, 0} };

  float getAngle(float newAngle, float newRate, float dt) {
    // Predict
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    float S = P[0][0] + R_measure;
    float K[2] = { P[0][0] / S, P[1][0] / S };

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0], P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }
};

// Kalman filters for pitch and roll
KalmanFilter kalmanPitch1, kalmanRoll1;
KalmanFilter kalmanPitch2, kalmanRoll2;

void setup() {
  Serial.begin(115200);
  I2CBus.begin(8, 9);
  delay(2000);

  imu1.setWire(&I2CBus);
  imu2.setWire(&I2CBus);

  imu1.beginAccel(); imu1.beginGyro();
  imu2.beginAccel(); imu2.beginGyro();

  calibrate(imu1);
  calibrate(imu2);

  prevTime = millis();
  Serial.println("Dual GY-91 with Kalman filter ready!");
}

void loop() {
  imu1.accelUpdate(); imu1.gyroUpdate();
  imu2.accelUpdate(); imu2.gyroUpdate();

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // --- GY-91 #1
  float ax1 = imu1.accelX() - axBias;
  float ay1 = imu1.accelY() - ayBias;
  float az1 = imu1.accelZ() - azBias;
  float gx1 = imu1.gyroX() - gxBias;
  float gy1 = imu1.gyroY() - gyBias;

  float accPitch1 = atan2(ay1, sqrt(ax1 * ax1 + az1 * az1)) * 180.0 / PI;
  float accRoll1  = atan2(-ax1, az1) * 180.0 / PI;

  float pitch1 = kalmanPitch1.getAngle(accPitch1, gx1, dt);
  float roll1  = kalmanRoll1.getAngle(accRoll1, gy1, dt);

  // --- GY-91 #2
  float ax2 = imu2.accelX(), ay2 = imu2.accelY(), az2 = imu2.accelZ();
  float gx2 = imu2.gyroX(),  gy2 = imu2.gyroY();

  float accPitch2 = atan2(ay2, sqrt(ax2 * ax2 + az2 * az2)) * 180.0 / PI;
  float accRoll2  = atan2(-ax2, az2) * 180.0 / PI;

  float pitch2 = kalmanPitch2.getAngle(accPitch2, gx2, dt);
  float roll2  = kalmanRoll2.getAngle(accRoll2, gy2, dt);

  // Combine or compare both sensors
  float avgPitch = (pitch1 + pitch2) / 2.0;
  float avgRoll  = (roll1 + roll2) / 2.0;

  // --- Output
  Serial.print("Pitch1: "); Serial.print(pitch1, 2);
  Serial.print("  Pitch2: "); Serial.print(pitch2, 2);
  Serial.print("  Avg: "); Serial.print(avgPitch, 2);

  Serial.print("  ||  Roll1: "); Serial.print(roll1, 2);
  Serial.print("  Roll2: "); Serial.print(roll2, 2);
  Serial.print("  Avg: "); Serial.println(avgRoll, 2);
  Serial.print(avgPitch);
  Serial.print("\t");
  Serial.println(avgRoll);
  delay(100);
}

void calibrate(MPU9250_asukiaaa sensor) {
  const int samples = 500;
  float axSum = 0, aySum = 0, azSum = 0;
  float gxSum = 0, gySum = 0, gzSum = 0;

  Serial.println("Calibrating imu1... Keep it still");

  for (int i = 0; i < samples; i++) {
    sensor.accelUpdate();
    sensor.gyroUpdate();

    axSum += sensor.accelX();
    aySum += sensor.accelY();
    azSum += sensor.accelZ() - 1.25;  // assuming gravity on Z

    gxSum += sensor.gyroX();
    gySum += sensor.gyroY();
    gzSum += sensor.gyroZ();

    delay(5);
  }

  axBias = axSum / samples;
  ayBias = aySum / samples;
  azBias = azSum / samples;
  gxBias = gxSum / samples;
  gyBias = gySum / samples;
  gzBias = gzSum / samples;

  Serial.println("Calibration complete.");
}

