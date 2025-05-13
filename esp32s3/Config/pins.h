#ifndef PINS_H
#define PINS_H

// Motor ESC Pins
#define ESC_PIN_1 21    // ESC 1 Pin (Motor 1)
#define ESC_PIN_2 22    // ESC 2 Pin (Motor 2)
#define ESC_PIN_3 23    // ESC 3 Pin (Motor 3)
#define ESC_PIN_4 24    // ESC 4 Pin (Motor 4)

// Sensor Pins
#define MPU6050_SDA_PIN 21   // I2C Data Pin for MPU6050
#define MPU6050_SCL_PIN 22   // I2C Clock Pin for MPU6050
#define BARO_SDA_PIN 21      // I2C Data Pin for Barometer
#define BARO_SCL_PIN 22      // I2C Clock Pin for Barometer
#define COMPASS_SDA_PIN 21      // I2C Data Pin for Gyro
#define COMPASS_SCL_PIN 22      // I2C Clock Pin for Gyro
#define TOF_SDA_PIN 21      // I2C Data Pin for TimeofFlightSensor
#define TOF_SCL_PIN 22      // I2C Clock Pin for TimeofFlightSensor
#define BMP390_INT 10         // Interrupt for BMP390

// Communication Pins (if needed)
#define TELEM_TX 49    // Pin for receiving data from telemetry
#define TELEM_RX 50    // Pin for transmitting data
#define TELEM_CTS 52  //UART CTS PIN
#define TELEM_RTS 51  //UART RTS PIN

// PPM Emergency Switch Pin
#define MANUAL_PPM 27   // Pin for PPM signal (from transmitter)


#endif // PINS_H
