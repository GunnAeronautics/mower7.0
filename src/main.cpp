#include <WiFi.h>
//#include <BluetoothSerial.h>
#include <Arduino.h>
//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <RollingAverage.h>

//#define BARO
//#define MPU6050
#define BNO055

#ifdef BARO
#include <Adafruit_BMP3XX.h>
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
float GyroX, GyroY,GyroZ;// deg / sec
float AccX,AccY,AccZ;//Gs
float pressure, temperature;
float lastAltitude;
float altitudeV;
unsigned long lastT;
int deltaT;
// put function declarations here:

RollingAverage pressureRoll;
RollingAverage temperatureRoll;
RollingAverage altitudeVRoll;
float groundTemperature, groundPressure, altitude;
#ifdef BNO055
void IMU_BNO055setup(){
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

}
#endif
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
#endif
#ifdef MPU6050
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
#endif
#ifdef BARO
void altitudeProcessing(){
  deltaT = micros()-lastT;
  lastT = micros();
  altitude = pressToAlt(pressureRoll.getData());
  altitudeVRoll.newData((altitude - lastAltitude)/deltaT*1000000);
}
#endif
void setup() {
  Serial.begin(9600);

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
    baroData();
  }
  groundTemperature = temperatureRoll.getData();
  groundPressure = pressureRoll.getData();
  #endif
  Serial.println("start");
  lastT = micros();
  //lastAltitude = pressToAlt(pressureRoll.getData());
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


  sensors_event_t BNO;
  bno.getEvent(&BNO);
  Serial.print(", X ");
  Serial.print(BNO.orientation.x, 4);
  Serial.print(", Y ");
  Serial.print(BNO.orientation.y, 4);
  Serial.print("Z: ");
  Serial.print(BNO.orientation.z, 4);
  Serial.println();

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
