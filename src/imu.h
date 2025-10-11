#include <Arduino.h>
#include <quaternion.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>

#define BNO_ADDRESS 0x29
#define CONFIG_MODE 0x00
#define ACCONLY_MODE 0x01
#define ACC_CONFIG_REGISTER 0x08
#define GRAVITY 9.81
#define SDA_PIN 21
#define SCL_PIN 22
extern Quaternion orientation;
extern Quaternion bnoOrientation;

extern float GyroX, GyroY, GyroZ; // deg / sec
extern float AccX, AccY, AccZ;    // Gs
extern float MagX, MagY, MagZ;    // Gs

extern float zAccel; // in m/s^2
extern float xAccel; // in m/s^2
extern float yAccel; // in m/s^2

extern float heading;        // in radians
extern float zenith;        // in radians


extern float yaw;
extern float pitch;
extern float roll;


extern float bnoW;
extern float bnoI;
extern float bnoJ;
extern float bnoK;
extern uint8_t systemCali,gyroCali,accelCali,magCali;

void writeRegister(uint8_t reg, uint8_t value);
void IMU_BNO055setup();
void IMUdata(int deltaT);
float verticalAcceleration(Quaternion orientation, float accelX, float accelY, float accelZ);
float axisComponent(Quaternion orientation, float x, float y, float z, String dir );
float angleBetweenAxis(Quaternion orientation, String dir);
float radiansToDegrees(float angle);
float degreesToRadians(float angle);