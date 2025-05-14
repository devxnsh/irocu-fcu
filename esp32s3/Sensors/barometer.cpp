#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

// Create an instance of the sensor
Adafruit_BMP3XX bmp;

// Optional: define custom I2C pins for ESP32
#define I2C_SDA 8
#define I2C_SCL 9

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("BMP390 test");

  // Initialize I2C with custom pins (ESP32 only)
  Wire.begin(I2C_SDA, I2C_SCL);

  // Begin BMP390 using I2C
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
    while (1);
  }

  // Set oversampling and filter for better accuracy
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); // Optional
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    delay(500);
    return;
  }

  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0); // Convert to hPa
  Serial.println(" hPa");

  Serial.print("Approx Altitude = ");
  Serial.print(bmp.readAltitude(1013.25)); // Adjust sea-level pressure as needed
  Serial.println(" m");

  Serial.println();
  delay(1000);
}
