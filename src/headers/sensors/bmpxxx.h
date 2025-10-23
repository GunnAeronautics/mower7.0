#ifndef BMPXXX_H
#define BMPXXX_H

#include "../config/config.h"
#include "../utils/RollingAverage.h"

// Enum to identify which sensor is active
enum BMPSensor {
  BMP_NONE = 0,
  BMP_390 = 1,
  BMP_580 = 2
};

// Struct to hold barometric data
struct BaroData {
  float pressure;           // hPa
  float temperature;        // C
  float altitude;           // m
  float altitudeV;          // m/s
  float groundPressure;     // hPa
  float groundTemperature;  // C
  BMPSensor sensorType;     // Which sensor this data came from
};

// Global data structures
extern BaroData baroData_unified;
extern BMPSensor activeSensor;
extern float lastAltitude;
extern float lastAltitudeBuiltIn;

// ============================================================================
// INITIALIZATION AND SETUP
// ============================================================================

/**
 * Initialize barometric sensor (auto-detects BMP390 or BMP580)
 * Returns the type of sensor found
 */
BMPSensor baroSetup();

// ============================================================================
// DATA ACQUISITION
// ============================================================================

/**
 * Read new data from active sensor
 */
void baroDataRead();

/**
 * Process altitude and velocity calculations
 * @param deltaT Time delta in milliseconds
 */
void altitudeProcessing(int deltaT);

// ============================================================================
// DATA ACCESS - UNIFIED
// ============================================================================

/**
 * Get unified barometric data from whichever sensor is active
 */
BaroData getBaroData();

/**
 * Get individual data fields
 */
float getPressure();
float getTemperature();
float getAltitude();
float getAltitudeVelocity();
float getGroundPressure();
float getGroundTemperature();
BMPSensor getActiveSensor();

// ============================================================================
// DATA ACCESS - SENSOR SPECIFIC
// ============================================================================

/**
 * Get data from BMP390 specifically (returns 0 if not active)
 */
BaroData getBaroData_BMP390();

/**
 * Get data from BMP580 specifically (returns 0 if not active)
 */
BaroData getBaroData_BMP580();

/**
 * Check if BMP390 is available
 */
bool isBMP390Active();

/**
 * Check if BMP580 is available
 */
bool isBMP580Active();

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Convert pressure to altitude
 * @param pres Pressure in hecta pascals (hPa)
 * @return Altitude in meters
 */
float pressToAlt(float pres);

#endif // BMPXXX_H
