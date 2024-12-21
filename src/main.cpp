//Sup guys this is Tony the main programmer on the ASTRA24-25 tarc team 
//make sure to use camelCase for variables and functions
//use ALLCAPS for definitions
//and utilize ifdefs

#include <WiFi.h>
//#include <BluetoothSerial.h>
#include <Arduino.h>
//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <RollingAverage.h>
#include <ESP32Servo.h>
#define BARO
//#define MPU6050
//#define BNO055
#define groundTesting
//#define SIMULATE
#define SERVO1_PIN 1;
#ifdef BARO
#include <Adafruit_BMPXX.h>
#define BMP_SCK 18
#define BMP_MISO 19
#define BMP_MOSI 23
#define BMP_CS 5
Adafruit_BMP3XX bmp;
#endif

#ifdef MPU6050
#define MPU_ADDRESS 0x68
#define MPU_SDA 13
#define MPU_SCL 12
#endif

#ifdef BNO055
#include <Adafruit_BNO055.h>
#define BNO_ADDRESS 0x28
#define BNO_SDA 13
#define BNO_SCL 12

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS, &Wire);

#endif

#define GRAVITY 9.81//m/s
#define DESIRED_APOGEE 250//m specified by TARC guidelines
#define DESIRED_FLIGHT_TIME 45 //seconds
#define PARACHUTE_DRAG_COEF 0.05//area included
#define DRAG_FLAP_RANGE 90// 0 for full retraction, 90 for full extension
//#define PARACHUTE_TERM_VELOCITY //m/s
#define GRAPH_NORMALIZED_MAX 1000

float GyroX, GyroY,GyroZ;// deg / sec
float AccX,AccY,AccZ;//Gs

float pressure, temperature;//Pa C
float lastAltitude;
float altitudeV;// in m/s
float verticalAcceleration;// in m/s^2
float zenith;// in radians
//ground references for pressure function to work
float groundTemperature, groundPressure, altitude; //alt in meters

float angularRocketDragCoef = 0;//area included
float rocketDragCoef = 0;//
float angularAirBreakDragCoef = 0;


unsigned long lastT;
int deltaT;
int startTimeStamp;// ms
int timeElapsed;//time elapsed in flight (ms)
//webhooktest
// put function declarations here:
Servo servo1;//create servo

RollingAverage pressureRoll(60);
RollingAverage temperatureRoll(60);
RollingAverage altitudeVRoll(60);

//RollingAverage angularRocketDragCoefRoll;
RollingAverage rocketDragCoefRoll(40);
RollingAverage angularAirBreakDragCoefRoll(40);

#ifdef SIMULATE
float pressureFunction(){//function of pressure to replace sensors
  return 1000
}
#endif
#ifdef BNO055
void IMU_BNO055setup(){
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

}
#endif
void checkMax(float newData, float* maxData){//updates a max value by a new datum
  if (newData > *maxData){
    *maxData = newData;
  }
}
#ifdef MPU6050
void IMUsetup(){
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(MPU_ADDRESS);//address of mpu6050
  Wire.write(0x6B);
  Wire.write(0x00);//reseting MPU
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);//address of mpu6050
  Wire.write(0x1A);//switch on low pass filter
  Wire.write(0x02);//~100 hz
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B);//gyroscope config
  Wire.write(0x18);//configure range 250 deg/s
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C);//accelerometer config
  Wire.write(0x18);//configure range 16g
  Wire.endTransmission();
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
#endif
#ifdef BARO
void baroSetup(){
  if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // hardware SPI mode  
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
}
float pressToAlt(float pres){ //returns alt (m) from pressure in hecta pascals and temperature in celsius - FULLY TESTED
  return (float)(((273.0+groundTemperature)/(-.0065))*((pow((pres/groundPressure),((8.314*.0065)/(GRAVITY*.02896))))-1.0)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}

float altitudeMax = 1;
float altitudeVMax = 1;
float pressureMax = 1;
void baroData(void){
  #ifdef SIMULATE
    temperature = 20;
    pressure = pressureFunction();
    return
  #endif
  if (!bmp.performReading()) {
    Serial.println("BMP390 Failed to perform reading :(");
    return;
  }
  temperature = bmp.temperature;//C
  temperatureRoll.newData(temperature);

  pressure = (bmp.pressure / 100.0);//hPa
  pressureRoll.newData(pressure);
  checkMax(pressureRoll.getData(),&pressureMax);
}



void altitudeProcessing(){
  deltaT = micros()-lastT;
  lastT = micros();
  altitude = pressToAlt(pressureRoll.getData());
  altitudeVRoll.newData((altitude - lastAltitude)/deltaT*1000000);
  altitudeV = altitudeVRoll.getData();

  checkMax(altitude,&altitudeMax);//fluff
  checkMax(altitudeVRoll.getData(),&altitudeVMax);

}
#endif


void logData(){
  Serial.print(millis());Serial.print(",");
  Serial.print(pressure);Serial.print(",");
  Serial.print(altitude);Serial.print(",");
  Serial.print(altitudeV);Serial.print(",");
  Serial.print("magnometer x");Serial.print(",");
  Serial.print("magnometer y");Serial.print(",");
  Serial.print("magnometer z");Serial.print(",");
  Serial.print("accel x");Serial.print(",");
  Serial.print("accel y");Serial.print(",");
  Serial.print("accel z");Serial.print(",");
  Serial.print("gyro x");Serial.print(",");
  Serial.print("gyro y");Serial.print(",");
  Serial.print("gyro z");Serial.print(",");
  Serial.print("gyro z");Serial.print(",");
  Serial.print("quat w");Serial.print(",");
  Serial.print("quat x");Serial.print(",");
  Serial.print("quat y");Serial.print(",");
  Serial.print("quat z");Serial.print(",");
  Serial.print("solenoid value");Serial.print(",");
  Serial.print("flap angle");Serial.print(",");
  Serial.print("state");Serial.print(",");


}
int flightState = 1;//state of the rocket's control
int flightStateAdvancementTrigger = 0;//counts number of times state switching event occurs
int flighti = 0;//something advanced
void setup() {
  Serial.begin(115200);
  servo1.attach(SERVO1_PIN);

  // Disable Wi-Fi
  //WiFi.mode(WIFI_OFF);
  //WiFi.disconnect(true);
  // Disable Bluetooth
  //btStop();
  Serial.println("Start Setup");
  delay(1000);
  #ifdef BNO055
  IMU_BNO055setup();
  #endif
  #ifdef MPU6050
  IMUsetup();
  #endif
  #ifdef BARO
  baroSetup();
  #endif
  delay(1000);
  #ifdef BARO
  for (int i = 0; i < 60; i++){
    baroData();//fill up the rolling averages
  }
  groundTemperature = temperatureRoll.getData();
  groundPressure = pressureRoll.getData();
  #endif
  for (int i = 0; i < 180; i++){
    Serial.println("write to servo");
    servo1.write(i);
    delay(10);
  }
  for (int i = 180; i > 0; i--){
    Serial.println("write to servo");
    servo1.write(i);
    delay(10);
  }
  Serial.println("start");
  lastT = micros();
  //lastAltitude = pressToAlt(pressureRoll.getData());
  }
int servoAngle;//servo angle for drag flaps, 0 for fully retracted 90 for fully extended, random variable
bool solenoidState;//state of the solenoid 0 for unreleased 1 to release

float predictApogee(float alt,float v,float dragCoef){
  return alt + log((dragCoef*v*v/9.81)+1)/(2.0*dragCoef);//copied from mower6.0
}
float inverseApogee() { // Binary Search
  float searchRangeH = angularAirBreakDragCoefRoll.getData();
  float searchRangeL = 0;//SEARCH RANGE IS 0<m<20
  float mid;//required drag coef from the air breaks
  for (int i = 0; i < 10; i++) {
    mid = (searchRangeL + searchRangeH) / 2.0;
    float prediction = predictApogee(altitude, altitudeV, mid + rocketDragCoefRoll.getData());

    if (prediction < DESIRED_APOGEE) { // to the left of desired
      searchRangeH = mid;
    } else { //if (prediction > DESIRED_APOGEE) {
      searchRangeL = mid;
    }
  }
  return mid/searchRangeH*DRAG_FLAP_RANGE;//returns angle for ddrag flaps to occupy
}
void loop() {

  #ifdef MPU6050
  accelData();
  gyroData();
  #endif
  #ifdef BARO
  baroData();
  altitudeProcessing();
  #endif

  //main control things
  switch(flightState){
    case 0://happy data printing mode
      Serial.print(altitude/altitudeMax*GRAPH_NORMALIZED_MAX);Serial.print(",");
      Serial.print(altitudeV/altitudeVMax*GRAPH_NORMALIZED_MAX);Serial.print(",");
      Serial.print(pressure/pressureMax*GRAPH_NORMALIZED_MAX);Serial.print(",");
      Serial.println();
      break;
    case 1://on the launch pad waiting to be ignited
      //do main rocket logic hereVVV
      Serial.println("waiting on launch pad");


      //flight switching code_______________
      if ((verticalAcceleration > 20)||(altitude > 10)){//tune these thresholds and statements
        flightStateAdvancementTrigger ++;
        if (flightStateAdvancementTrigger > 3){//tune this thresholds
          flightState ++;
          flightStateAdvancementTrigger = 0;
          startTimeStamp = millis();//start the flight timer here
        }
      }
      else{
        flightStateAdvancementTrigger = 0;
      }
      //switching code end__________________
      break;
    case 2://Accelerating Stage
      //do main rocket logic here VVV

      //flight switching code_______________
      if ((verticalAcceleration < 0)||(timeElapsed > 3000)){//tune these thresholds and statements
        flightStateAdvancementTrigger ++;
        if (flightStateAdvancementTrigger > 3){//tune this thresholds
          flightState ++;
          flightStateAdvancementTrigger = 0;
        }
      }
      else{
        flightStateAdvancementTrigger = 0;
      }
      //switching code end__________________
      logData();
      break;
    case 3://Upward freefall
      flighti ++;
      //do main rocket logic here VVV
      //Vi^2 = 2aD
      float totalDragCoef = verticalAcceleration / pow(altitudeV,2);

      if  (flighti < 100){//servos at 0
      Serial.println("Retract Servos Completely");
      rocketDragCoefRoll.newData(totalDragCoef);
      }
      else if (flighti < 200){//servos at 90 
      Serial.println("Fully Extend Servos");
      angularAirBreakDragCoefRoll.newData(totalDragCoef - rocketDragCoefRoll.getData());
      }
      else{
        Serial.println(inverseApogee());
        
      }
      



      //flight switching code_______________
      if ((altitudeV < 0)){//tune these thresholds and statements
        flightStateAdvancementTrigger ++;
        if (flightStateAdvancementTrigger > 3){//tune this thresholds
          flightState = 2;
          flightStateAdvancementTrigger = 0;
        }
      }
      else{
        flightStateAdvancementTrigger = 0;
      }
      //switching code end__________________
      logData();
      break;
    case 4://decent
      //do main rocket logic here VVV

      //flight switching code_______________
      if (((-1 < altitudeV)&&(altitudeV < 1))||(timeElapsed > 100000)){//tune these thresholds and statements
        flightStateAdvancementTrigger ++;
        if (flightStateAdvancementTrigger > 3){//tune this thresholds
          flightState = 2;
          flightStateAdvancementTrigger = 0;
        }
      }
      else{
        flightStateAdvancementTrigger = 0;
      }
      //switching code end__________________
      logData();
      break;
    case 5://on the ground yippee
      //do whatever u want here
      break;
    }
  servo1.write(servoAngle);
  // sensors_event_t BNO;
  // bno.getEvent(&BNO);
  // Serial.print(", ");
  // Serial.print(BNO.gyro.x);
  // Serial.print(", ");
  // Serial.print(BNO.gyro.y);
  // Serial.print(", ");
  // Serial.print(BNO.gyro.z);
  // Serial.println();
  // Serial.print(BNO.gyro.z);
  // Serial.println();

  // Quaternion data
  // imu::Quaternion quat = bno.getQuat();
  // Serial.print("\t");
  // Serial.print(quat.w(), 4);
  // Serial.print("\t");
  // Serial.print(quat.x(), 4);
  // Serial.print("\t");
  // Serial.print(quat.y(), 4);
  // Serial.print("\t");
  // Serial.print(quat.z(), 4);
  // Serial.print("\n");

  // lastAltitude = altitude;
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
  
  
  //altitude = pressToAlt(pressureRoll.getData());
  
  //Serial.print(", pressure:");
  //Serial.print(pressure);
  //Serial.print(", temperature:");
  //Serial.print(temperature);
  //Serial.print(", time:");
  //Serial.print(altitudeVRoll.getData());
  
  //Serial.print(", startingPressure:");
  //Serial.print(groundPressure);
  //Serial.print(", altitude2.0:");
  //Serial.print(altitude);

  //Serial.println( );
}
