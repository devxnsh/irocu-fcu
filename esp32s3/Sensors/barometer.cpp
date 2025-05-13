#include "barometer.h"
#include <Wire.h>

Barometer::Barometer() {}

void Barometer::init() {
    Wire.begin();
    // Initialize your barometer here
}

float Barometer::getAltitude() {
    // Return the altitude in meters
    return 100.0;  // Dummy value
}

float Barometer::getPressure() {
    // Return the pressure in Pascals
    return 101325.0;  // Dummy value
}

float Barometer::getTemperature() {
    // Return the temperature in Celsius
    return 25.0;  // Dummy value
}

void setup() {
    Serial.begin(115200);
    Barometer barometer;
    barometer.init();
}

void loop() {
    Barometer barometer;
    
    // Read sensor data
    float altitude = barometer.getAltitude();
    float pressure = barometer.getPressure();
    float temperature = barometer.getTemperature();
    
    // Print the sensor data
    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" m");
    
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" Pa");
    
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    
    delay(1000);  // Adjust delay as needed
}
