#include "motor_control.h"

void motorControlInit() {
    // Initialize motors (if needed)
    Serial.println("Motor Control Initialized");
}

void set_motor_speeds(int motor1, int motor2, int motor3, int motor4) {
    // Send PWM signals to motors (example)
    Serial.print("Motor 1 Speed: ");
    Serial.println(motor1);
    Serial.print("Motor 2 Speed: ");
    Serial.println(motor2);
    Serial.print("Motor 3 Speed: ");
    Serial.println(motor3);
    Serial.print("Motor 4 Speed: ");
    Serial.println(motor4);
}

void setup() {
    Serial.begin(115200);
    motorControlInit();
}

void loop() {
    // Test motors by setting speeds
    set_motor_speeds(100, 100, 100, 100);  // Example values
    delay(1000);  // Adjust delay as needed
    set_motor_speeds(0, 0, 0, 0);  // Stop motors
    delay(1000);  // Adjust delay as needed
}
