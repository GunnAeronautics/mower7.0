#ifndef QUATERNION_H
#define QUATERNION_H

#include <Arduino.h>
#include "../config/config.h"

class Quaternion {
  public:
    float w;
    float i;
    float j;
    float k;
    
    Quaternion(float w, float i, float j, float k);
    Quaternion();
    void setQuat(float w, float i, float j, float k);
    void normalize(float tolerance = 0.0001);
    void normto(float length, float tolerance = 0.0001); // error for some reason??
    void fromAngleVec(float angle, float heading, float elevation);
    void mult(Quaternion quart);
    void rot(Quaternion quart);
    void gyro(float x, float y, float z);
    void add(Quaternion quart);
    void sub(Quaternion quart);
    float getLength();
    Quaternion inverse();
    Quaternion copy();
};

#endif // QUATERNION_H
