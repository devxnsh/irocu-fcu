#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "pins.h"

Adafruit_BMP3XX bmp;

void init_baro()
{  
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); // Optional
}

int barometer_reading() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    delay(5);
    return 0;
  }
  return bmp.readAltitude(1013.25); // Adjust sea-level pressure as needed
}
