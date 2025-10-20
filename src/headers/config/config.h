#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// HARDWARE PIN CONFIGURATION
// ============================================================================

// BMP390/BMP580 Barometric Sensor Pins
#define BMP_SCK 23
#define BMP_MISO 19
#define BMP_MOSI 18
#define BMP390_CS 5
#define BMP580_CS 4

// ADXL343 Accelerometer Pins
#define ADXL343_SCK 13
#define ADXL343_MISO 12
#define ADXL343_MOSI 11
#define ADXL343_CS 10

// BNO055 IMU I2C Configuration
#define BNO_ADDRESS 0x29
#define SDA_PIN 21
#define SCL_PIN 22

// SD Card SPI Pins
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 5

// ============================================================================
// BNO055 REGISTER CONFIGURATION
// ============================================================================

#define CONFIG_MODE 0x00
#define ACCONLY_MODE 0x01
#define ACC_CONFIG_REGISTER 0x08

// ============================================================================
// PHYSICAL CONSTANTS
// ============================================================================

#define GRAVITY 9.81  // m/s^2

// ============================================================================
// FLIGHT PARAMETERS
// ============================================================================

// Apogee and flight time targets (TARC competition specifications)
#define DESIRED_APOGEE 240.792        // m (790 ft specified by TARC guidelines)
#define DESIRED_FLIGHT_TIME 42500     // milliseconds (41-44 sec regulation 2025)

// Parachute and drag flap configuration
#define PARACHUTE_TERM_VELOCITY 3.256 // m/s
#define COEF_DRAG_FLAPDEPLOYED 0.002213 // accelY / v / v of the rocket when flaps fully deployed

// Servo deployment angles
#define DRAG_FLAPS_DEPLOYED_ANGLE 45
#define DRAG_FLAPS_UNDEPLOYED_ANGLE 135
#define PARACHUTE_DEPLOYED_ANGLE 170
#define PARACHUTE_UNDEPLOYED_ANGLE 10

// ============================================================================
// DATA LOGGING
// ============================================================================

#define MAX_STRING_SIZE 300
#define DATAPRECISION 4  // decimals for SD card printing

#endif // CONFIG_H
