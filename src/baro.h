
#include <Adafruit_BMP3XX.h>
#include "RollingAverage.h"

extern RollingAverage pressureRoll;
extern RollingAverage temperatureRoll;
extern RollingAverage altitudeVRoll;
extern RollingAverage altitudeBuiltInVRoll;

#define BMP_SCK 23
#define BMP_MISO 19
#define BMP_MOSI 18
#define BMP_CS 5
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