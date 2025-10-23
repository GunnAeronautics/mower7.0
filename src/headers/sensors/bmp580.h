#ifndef BMP580_H
#define BMP580_H

#include <Adafruit_BMP5xx.h>
#include "../config/config.h"
#include "../utils/RollingAverage.h"

extern RollingAverage pressureRoll;
extern RollingAverage temperatureRoll;
extern RollingAverage altitudeVRoll;
extern RollingAverage altitudeBuiltInVRoll;

extern float pressure, temperature; // Pa, C
extern float lastAltitude, lastAltitudeBuiltIn;
extern float altitudeV;     // in m/s

//extern float groundTemperature, groundPressure, altitude, builtInAltitude; // alt in meters

extern float altitudeBuiltInV;

void baroSetup();
void altitudeProcessing(int deltaT);
void baroData();
float pressToAlt(float pres);

#endif // BMP580_H
