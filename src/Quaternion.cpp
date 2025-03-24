//made by Gunn Aero members way before me (ancients)
#include "quaternion.h"


float w;
float i;
float j;
float k;

Quaternion::Quaternion(float w, float i, float j, float k){
  this->w = w;
  this->i = i;
  this->j = j;
  this->k = k;
}

Quaternion::Quaternion(){
  w = 1.0;
  i = 0.0;
  j = 0.0;
  k = 0.0;
}
void Quaternion::setQuat(float w, float i, float j, float k){
  this->w = w;
  this->i = i;
  this->j = j;
  this->k = k;
}
void Quaternion::normalize(float tolerance){
  float bigness = w*w + i*i + j*j + k*k;
  if(abs(1.0 - bigness) >= tolerance){
    bigness = sqrt(bigness);
    w /= bigness;
    i /= bigness;
    j /= bigness;
    k /= bigness;
  }
}

void Quaternion::normto(float length, float tolerance){
  float bigness = w*w + i*i + j*j + k*k;
  if(abs(length - bigness) >= tolerance){
    bigness = sqrt(bigness);
    w *= length/bigness;
    i *= length/bigness;
    j *= length/bigness;
    k *= length/bigness;
  }
}

void Quaternion::fromAngleVec(float angle, float heading, float elevation){
  angle *= 0.5;
  this->w = cos(angle);
  this->i = cos(heading) * cos(elevation) * sin(angle);
  this->j = sin(elevation) * sin(angle);
  this->k = sin(heading) * cos(elevation) * sin(angle);
}

void Quaternion::mult(Quaternion quart){
  Quaternion temp = Quaternion(this->w, this->i, this->j, this->k);
  this->w = temp.w*quart.w - temp.i*quart.i - temp.j*quart.j - temp.k*quart.k;
  this->i = temp.w*quart.i + temp.i*quart.w + temp.j*quart.k - temp.k*quart.j;
  this->j = temp.w*quart.j - temp.i*quart.k + temp.j*quart.w + temp.k*quart.i;
  this->k = temp.w*quart.k + temp.i*quart.j - temp.j*quart.i + temp.k*quart.w;
}

void Quaternion::rot(Quaternion quart){
  this->mult(quart);
  Quaternion temp = quart.inverse();
  temp.mult(*this);
  this->w = temp.w; this->i = temp.i; this->j = temp.j; this->k = temp.k;
}

void Quaternion::gyro(float x, float y, float z){
  //dont ask me how this works i have no clue
  //i found it on a stanford pdf, so it must be good
  //i dont even know what the gyro data means
  //oh also it wants radians
  float angle = sqrt(x*x + y*y + z*z); // magnitude of rotation vector or something
  if(angle != 0){
    this->w = cos(angle/2);
    this->i = x/angle * sin(angle/2);
    this->j = y/angle * sin(angle/2);
    this->k = z/angle * sin(angle/2);
  }else{
    this->w = 1;
    this->i = 0;
    this->j = 0;
    this->k = 0;
  }
}

void Quaternion::add(Quaternion quart){
  this->w += quart.w;
  this->i += quart.i;
  this->j += quart.j;
  this->k += quart.k;
}

void Quaternion::sub(Quaternion quart){
  this->w -= quart.w;
  this->i -= quart.i;
  this->j -= quart.j;
  this->k -= quart.k;
}

float Quaternion::getLength(){
  return sqrt(w*w + i*i + j*j + k*k);
}

Quaternion Quaternion::inverse(){
  return Quaternion(this->w, -this->i, -this->j, -this->k);
}

Quaternion Quaternion::copy(){
  return Quaternion(this->w, this->i, this->j, this->k);
}
