
#include <Adafruit_BMP3xx.h>
#include "Adafruit_BMP5xx.h"

#include "RollingAverage.h"

extern RollingAverage pressureRoll;
extern RollingAverage temperatureRoll;
extern RollingAverage altitudeVRoll;
extern RollingAverage altitudeBuiltInVRoll;

#define BMP_SCK 23
#define BMP_MISO 19
#define BMP_MOSI 18
#define BMP390_CS 5
#define BMP580_CS 4

#define GRAVITY 9.81

extern float pressure, temperature; // Pa C
extern float lastAltitude, lastAltitudeBuiltIn;
extern float altitudeV;     // in m/s

extern float groundTemperature, groundPressure, altitude, builtInAltitude; // alt in meters

extern float altitudeBuiltInV;

void baroSetup();
void altitudeProcessing(int deltaT);
void baroData();
float pressToAlt(float pres);