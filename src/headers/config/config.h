#ifndef CONFIG_H
#define CONFIG_H

#define FAKE_IMU true
#define DEBUG true

// ============================================================================
// HARDWARE PIN CONFIGURATION
// ============================================================================

// Servo Pins
#define SERVO_K 26 // 1
#define SERVO_L 25 // 2
#define SERVO_M 35 // 3
#define SERVO_N 34 // 4

// BMP390/BMP580 Barometric Sensor Pins
#define BMP_SCK 13 //clk // scl
#define BMP_MISO 27 //sdo // 
#define BMP_MOSI 14 //sda //
#define BMP390_CS 33
#define BMP580_CS 32

// ADXL343 Accelerometer Pins, -- uses i2c now
/*#define ADXL343_SCK 13
#define ADXL343_MISO 12
#define ADXL343_MOSI 11
#define ADXL343_CS 10*/
#define ADXL343_ADR 0x53
#define ADXL343_SCL 22
#define ADXL343_SDA 5

 //BNO055 IMU I2C Configuration
#define BNO_ADDRESS 0x29
#define SDA_PIN 21
#define SCL_PIN 22


// SD Card SPI Pins
#define SD_SCK 19
#define SD_MISO 21//correct
#define SD_MOSI 3//23
#define SD_CS 23//18

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
