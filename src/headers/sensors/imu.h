#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "../math/Quaternion.h"
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include "../config/config.h"

extern Quaternion orientation;
extern Quaternion bnoOrientation;

extern float GyroX, GyroY, GyroZ; // deg / sec
extern float AccX, AccY, AccZ;    // Gs
extern float MagX, MagY, MagZ;    // Gs

extern double zAccel; // in m/s^2
extern double xAccel; // in m/s^2
extern double yAccel; // in m/s^2

extern float heading;        // in radians
extern float zenith;         // in radians

extern float yaw;
extern float pitch;
extern float roll;

extern float bnoW;
extern float bnoI;
extern float bnoJ;
extern float bnoK;
extern uint8_t systemCali, gyroCali, accelCali, magCali;

void writeRegister(uint8_t reg, uint8_t value);
void IMU_BNO055setup();
void IMUdata(int deltaT);
float verticalAcceleration(Quaternion orientation, float accelX, float accelY, float accelZ);
float axisComponent(Quaternion orientation, float x, float y, float z, String dir);
float angleBetweenAxis(Quaternion orientation, String dir);
float radiansToDegrees(float angle);
float degreesToRadians(float angle);

#endif // IMU_H
