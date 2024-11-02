#include <Arduino.h>
#include <Wire.h>


#define MPUADDRESS 0x68
#define LED_PIN 5
float GyroX, GyroY,GyroZ;// deg / sec
float AccX,AccY,AccZ;//Gs
// put function declarations here:
int myFunction(int, int);
void accelData(void){
  Wire.beginTransmission(MPUADDRESS);//address of mpu6050
  Wire.write(0x1A);//switch on low pass filter
  Wire.write(0x02);//~100 hz
  Wire.endTransmission();

  Wire.beginTransmission(MPUADDRESS);
  Wire.write(0x1C);//accelerometer config
  Wire.write(0x18);//configure range 16g
  Wire.endTransmission();

  Wire.beginTransmission(MPUADDRESS);
  Wire.write(0x3B);//start of Accel out table
  Wire.endTransmission();
  Wire.requestFrom(MPUADDRESS, 6);
  //6 registers, 3 for each dimension (xyz) and 2 for the accuracy
  //each register has 8 bits
  int16_t AccelXLSB = Wire.read() << 8 | Wire.read();//X
  int16_t AccelYLSB = Wire.read() << 8 | Wire.read();//Y
  int16_t AccelZLSB = Wire.read() << 8 | Wire.read();//Z
  AccX = (float)AccelXLSB/2048;
  AccY = (float)AccelYLSB/2048;
  AccZ = (float)AccelZLSB/2048;
}
void gyroData(void){
  Wire.beginTransmission(MPUADDRESS);//address of mpu6050
  Wire.write(0x1A);//switch on low pass filter
  Wire.write(0x02);//~100 hz
  Wire.endTransmission();

  Wire.beginTransmission(MPUADDRESS);
  Wire.write(0x1B);//gyroscope config
  Wire.write(0x00);//configure range 250 deg/s
  Wire.endTransmission();

  Wire.beginTransmission(MPUADDRESS);
  Wire.write(0x43);//start of Baro out table
  Wire.endTransmission();
  Wire.requestFrom(MPUADDRESS,6);
  int16_t GyroXLSB = Wire.read() << 8 | Wire.read();//X
  int16_t GyroYLSB = Wire.read() << 8 | Wire.read();//Y
  int16_t GyroZLSB = Wire.read() << 8 | Wire.read();//Z  
  GyroX = (float)GyroXLSB/ 131;
  GyroY = (float)GyroXLSB/ 131;
  GyroZ = (float)GyroXLSB/ 131;

}
void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(MPUADDRESS);//address of mpu6050
  Wire.write(0x6B);
  Wire.write(0x00);//reseting MPU
  Wire.endTransmission();

 }

void loop() {
  accelData();
  gyroData();
  Serial.print("accelX:");
  Serial.print(AccX);
  Serial.print(", accelY:");
  Serial.print(AccY);
  Serial.print(", accelZ:");
  Serial.print(AccZ);
  Serial.print(", gyroX:");
  Serial.print(GyroX);
  Serial.print(", gyroY:");
  Serial.print(GyroY);
  Serial.print(", gyroZ:");
  Serial.print(GyroZ);
  Serial.println( );
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}