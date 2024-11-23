#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

#define BMP_SCK 18
#define BMP_MISO 19
#define BMP_MOSI 23
#define BMP_CS 5
Adafruit_BMP3XX bmp;

#define MPUADDRESS 0x68
#define LED_PIN 5
#define GRAVITY 9.81//m/s
float GyroX, GyroY,GyroZ;// deg / sec
float AccX,AccY,AccZ;//Gs
float pressure, temperature;
float lastAltitude;
float altitudeV;
unsigned long lastT;
int deltaT;
// put function declarations here:
class RollingAverage{

  public:

  int rollingLen;
  float *rawData;//I can't get the size of this array to be parametric
  //for some reason it always intializes as an array of inf and changing values in the constructor
  //doesn't seem to work, this seems like the best option as nothing else FUCKING WORKS GAH!
  //ok what the actual fuck initializing the array as {0,0,0,0...} doesnt fucking work either
  int dataIndex;
  int startFlag;
  float average;

  RollingAverage(){
    rollingLen = 50;//default
    dataIndex = 0;
    average = 0.0;
    rawData = new float[rollingLen]();
  }

  void newData(float data){

    average = average + data;
    average = average - rawData[dataIndex];
    rawData[dataIndex] = data;
    dataIndex ++;
    dataIndex %= rollingLen;
  }
  float getData(){

    return average / rollingLen;
  }

};
RollingAverage pressureRoll;
RollingAverage temperatureRoll;
RollingAverage altitudeVRoll;
int myFunction(int, int);
float groundTemperature, groundPressure, altitude;
void IMUsetup(){
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(MPUADDRESS);//address of mpu6050
  Wire.write(0x6B);
  Wire.write(0x00);//reseting MPU
  Wire.endTransmission();

  Wire.beginTransmission(MPUADDRESS);//address of mpu6050
  Wire.write(0x1A);//switch on low pass filter
  Wire.write(0x02);//~100 hz
  Wire.endTransmission();

  Wire.beginTransmission(MPUADDRESS);
  Wire.write(0x1B);//gyroscope config
  Wire.write(0x18);//configure range 250 deg/s
  Wire.endTransmission();

  Wire.beginTransmission(MPUADDRESS);
  Wire.write(0x1C);//accelerometer config
  Wire.write(0x18);//configure range 16g
  Wire.endTransmission();
}
void baroSetup(){
  if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // hardware SPI mode  
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
}
float pressToAlt(float pres){ //returns alt (m) from pressure in hecta pascals and temperature in celsius - FULLY TESTED
  return (float)(((273.0+groundTemperature)/(-.0065))*((pow((pres/groundPressure),((8.314*.0065)/(GRAVITY*.02896))))-1.0)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}
void accelData(void){
  

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
  

  Wire.beginTransmission(MPUADDRESS);
  Wire.write(0x43);//start of Baro out table
  Wire.endTransmission();
  Wire.requestFrom(MPUADDRESS,6);
  int16_t GyroXLSB = Wire.read() << 8 | Wire.read();//X
  int16_t GyroYLSB = Wire.read() << 8 | Wire.read();//Y
  int16_t GyroZLSB = Wire.read() << 8 | Wire.read();//Z  
  GyroX = (float)GyroXLSB/ 131;
  GyroY = (float)GyroYLSB/ 131;
  GyroZ = (float)GyroZLSB/ 131;

}
void baroData(void){
    if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  temperature = bmp.temperature;//C
  temperatureRoll.newData(temperature);

  pressure = (bmp.pressure / 100.0);//hPa
  pressureRoll.newData(pressure);
}
void setup() {
  Serial.begin(115200);
  IMUsetup();
  baroSetup();
  delay(1000);
  for (int i = 0; i < 60; i++){
    baroData();
  }
  groundTemperature = temperatureRoll.getData();
  groundPressure = pressureRoll.getData();

  Serial.println("start");
  lastT = micros();
  lastAltitude = pressToAlt(pressureRoll.getData());
 }

void loop() {
  deltaT = micros()-lastT;
  lastT = micros();
  accelData();
  gyroData();
  baroData();
  
  altitude = pressToAlt(pressureRoll.getData());
  altitudeVRoll.newData((altitude - lastAltitude)/deltaT*1000000);
  lastAltitude = altitude;
   //Serial.print(",accelX:");
   //Serial.print(groundPressure);
  // Serial.print(", accelY:");
  // Serial.print(AccY);
  // Serial.print(", accelZ:");
  // Serial.print(AccZ);
  
  /*
  Serial.print(", gyroX:");
  Serial.print(GyroX);
  Serial.print(", gyroY:");
  Serial.print(GyroY);
  Serial.print(", gyroZ:");
  Serial.print(GyroZ);
  */
  
  
  altitude = pressToAlt(pressureRoll.getData());
  
  //Serial.print(", pressure:");
  //Serial.print(pressure);
  //Serial.print(", temperature:");
  //Serial.print(temperature);
  Serial.print(", time:");
  Serial.print(altitudeVRoll.getData());
  
  //Serial.print(", startingPressure:");
  //Serial.print(groundPressure);
  Serial.print(", altitude2.0:");
  Serial.print(altitude);

  Serial.println( );
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}