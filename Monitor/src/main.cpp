#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

void setup(void) {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n--- [INA219 ENERGY MONITORING SYSTEM] ---");

  Wire.begin(5, 6);

  if (!ina219.begin()) {
    Serial.println("❌ ERROR: Failed to find INA219 chip!");
    Serial.println("Check the SDA and SCL wiring.");
    while (1) { delay(10); }
  }
  
  Serial.println("✅ INA219 Found and Initialized.");
  Serial.println("Starting power consumption readings...\n");
  Serial.println("---------------------------------------------------");
}

void loop(void) {
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  Serial.printf("Battery (Load Voltage) : %.2f V\n", loadvoltage);
  Serial.printf("ESP32 Voltage (Bus)    : %.2f V\n", busvoltage);
  Serial.printf("Current Consumed       : %.2f mA\n", current_mA);
  Serial.printf("Instantaneous Power    : %.2f mW\n", power_mW);
  Serial.println("---------------------------------------------------");

  delay(5000);
}