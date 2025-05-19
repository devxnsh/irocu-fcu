#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox;  // This is the only place where lox is defined

void init_tof() {
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}

int tof_reading() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false);

  delay(5);

  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return 0;
  }
}
