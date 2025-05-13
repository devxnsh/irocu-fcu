#include "compass.h"
#include <Wire.h>

Compass::Compass() {}

void Compass::init() {
    Wire.begin();
    // Initialize your compass here
}

float Compass::getHeading() {
    // Return heading in degrees
    return 45.0;  // Dummy value
}

void setup() {
    Serial.begin(115200);
    Compass compass;
    compass.init();
}

void loop() {
    Compass compass;
    
    // Read sensor data
    float heading = compass.getHeading();
    
    // Print the heading
    Serial.print("Heading: ");
    Serial.print(heading);
    Serial.println(" degrees");
    
    delay(1000);  // Adjust delay as needed
}
