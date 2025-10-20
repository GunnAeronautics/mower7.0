#ifndef ADXL_H
#define ADXL_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include "../config/config.h"

extern double xAccel;
extern double yAccel;
extern double zAccel;

void adxlSetup(void);
void getADXLData(void);

#endif // ADXL_H
