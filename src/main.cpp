#include <Wire.h>
#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial to initialize (especially on USB)

  Serial.println("Raspberry Pi Pico I2C Scanner");
  Wire.begin();  // Uses default SDA/SCL pins for Pico (GP4 = SDA, GP5 = SCL by default)

  scanI2C();
}

void loop() {
  // Re-run the scan every 5 seconds
  delay(5000);
  scanI2C();
}

void scanI2C() {
  Serial.println("\nScanning I2C bus...");

  uint8_t count = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("✔ Device found at 0x");
      Serial.println(address, HEX);
      count++;
    } else if (error == 4) {
      Serial.print("⚠ Unknown error at 0x");
      Serial.println(address, HEX);
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Total devices found: ");
    Serial.println(count);
  }
}
