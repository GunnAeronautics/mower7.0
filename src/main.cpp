// Sup guys this is Tony the main programmer on the ASTRA24-25 tarc team
// make sure to use camelCase for variables and functions
// use ALLCAPS for definitions
// and utilize ifdefs

// #include <WiFi.h>
// #include <BluetoothSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <RollingAverage.h>
#include <Quaternion.h>
#include <ESP32Servo.h>
#include <math.h>         //for inverse trig functions
#define DUMMY_ROCKET true // if dummy rocket then no servo or solenoid

#define BARO
// #define MPU6050
#define BNO055
#define SDCARD
// #define groundTesting
// #define SIMULATE
// #define SERVO

#define DATAPRECISION 6// decimals
#ifdef SERVO
#define SERVO1_PIN 4
#endif
#define SDA_PIN 21
#define SCL_PIN 22

#ifdef SDCARD
#include <FS.h>
#include <SD.h>
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 4
//#define SD_SCK 16
//#define SD_MISO 17
//#define SD_MOSI 15
//#define SD_CS 4
#endif

#ifdef BARO
#include <Adafruit_BMP3XX.h>
// #define BMP390_I2C_ADDRESS 0x77
// SPI:
#define BMP_SCK 16
#define BMP_MISO 17
#define BMP_MOSI 15
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
#define CONFIG_MODE 0x00
#define ACCONLY_MODE 0x01
#define ACC_CONFIG_REGISTER 0x08
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS, &Wire);

#endif
char buffer[20]; 
#define GRAVITY 9.81              // m/s^2
#define DESIRED_APOGEE 250        // m specified by TARC guidelines
#define DESIRED_FLIGHT_TIME 45000 // miliseconds
// #define PARACHUTE_DRAG_COEF 0.05//area included
#define DRAG_FLAP_RANGE 90         // 0 for full retraction, 90 for full extension
#define PARACHUTE_TERM_VELOCITY 10 // m/s
// #define GRAPH_NORMALIZED_MAX 1000

float GyroX, GyroY, GyroZ; // deg / sec
float AccX, AccY, AccZ;    // Gs
float MagX, MagY, MagZ;    // Gs

float pressure, temperature; // Pa C
float lastAltitude;
float altitudeV;     // in m/s
float verticalAccel; // in m/s^2
float xAccel; // in m/s^2
float yAccel; // in m/s^2

float heading;        // in radians
float zenith;        // in radians

// ground references for pressure function to work
float groundTemperature, groundPressure, altitude; // alt in meters

float totalDragCoef;
float angularRocketDragCoef = 0; // area included
float rocketDragCoef = 0;        //
float angularAirBreakDragCoef = 0;

float yaw;
float pitch;
float roll;

unsigned long lastT;
int deltaT;
int startTimeStamp = 0; // ms
int timeElapsed = 0;    // time elapsed in flight (ms)

int flightState = 0;                   // state of the rocket's control
int flightStateAdvancementTrigger = 0; // counts number of times state switching event occurs
int flighti = 0;                       // something advanced
int servoAngle;                        // servo angle for drag flaps, 0 for fully retracted 90 for fully extended, random variable
bool solenoidState;                    // state of the solenoid 0 for unreleased 1 to release
float bnoW;
float bnoI;
float bnoJ;
float bnoK;
// webhooktest
//  put function declarations here:
#ifdef SERVO
Servo servo1; // create servo
#endif
RollingAverage pressureRoll(60);
RollingAverage temperatureRoll(60);
RollingAverage altitudeVRoll(60);

// RollingAverage angularRocketDragCoefRoll;
RollingAverage rocketDragCoefRoll(40);
RollingAverage angularAirBreakDragCoefRoll(40);

RollingAverage deltaTRoll(300);
Quaternion orientation;
Quaternion angularSpeed;
SPIClass hspi(HSPI);
SPIClass vspi(VSPI);
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x28); // BNO055 I2C address
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
#ifdef SIMULATE
float pressureFunction()
{ // function of pressure to replace sensors
  return 1000
}
#endif
#ifdef BNO055
void IMU_BNO055setup()
{

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_CONFIG);
  delay(25); // Allow sensor to stabilize
  writeRegister(ACC_CONFIG_REGISTER, 0x0C); // Set range to ±16g
  Serial.println("Accelerometer set to ±16g range.");
  // Switch to ACCONLY mode
  bno.setMode(OPERATION_MODE_NDOF_FMC_OFF);
  delay(25); // Allow sensor to stabilize


}
#endif
void checkMax(float newData, float *maxData)
{ // updates a max value by a new datum
  if (newData > *maxData)
  {
    *maxData = newData;
  }
}

#ifdef SDCARD

String logFilename;
String getNewLogFilename()
{
  int fileIndex = 1;
  String filename;
  do
  {
    filename = "/datalog" + String(fileIndex) + ".csv";
    fileIndex++;
  } while (SD.exists(filename.c_str()));
  return filename;
}

void sdSetup()
{
  if (!SD.begin(4, hspi))
  {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
  {
    Serial.println("MMC");
  }
  else if (cardType == CARD_SD)
  {
    Serial.println("SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    Serial.println("SDHC");
  }
  else
  {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  logFilename = getNewLogFilename();
  Serial.println(logFilename);
  //writeCSVLine(SD, logFilename.c_str(), "Time, Baro");
}

// appendFile(SD, "/hello.txt", "World!\n");
void writeCSVLine(fs::FS &fs, const char *path, const char *message)
{
  //Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    //Serial.println("Message appended");
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}

#endif
#ifdef BARO

void baroSetup()
{
  // SPI
  while (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI))
  { // hardware SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
  // I2C
  //  if (!bmp.begin_I2C(0x77)) {
  //        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
  //        while (1);
  //    }

  //bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_1X);
  //bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  //bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);

}
float pressToAlt(float pres)
{                                                                                                                                            // returns alt (m) from pressure in hecta pascals and temperature in celsius - FULLY TESTED
  return (float)(((273.0 + groundTemperature) / (-.0065)) * ((pow((pres / groundPressure), ((8.314 * .0065) / (GRAVITY * .02896)))) - 1.0)); // https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula
}
void baroData(void)
{ // gets barometric data from the sensor
#ifdef SIMULATE
  temperature = 20;
  pressure = pressureFunction();
  return
#endif
      if (!bmp.performReading())
  {
    Serial.println("BMP390 Failed to perform reading :(");
    return;
  }

  temperature = bmp.temperature; // C
  temperatureRoll.newData(temperature);

  pressure = (bmp.pressure / 100.0); // hPa
  pressureRoll.newData(pressure);
}
void altitudeProcessing()
{ // takes pressure data and transforms it into altitude and altitude velocity
  lastAltitude = altitude;
  altitude = pressToAlt(pressureRoll.getData());
  altitudeVRoll.newData((altitude - lastAltitude) / deltaT * 1000000);
  altitudeV = altitudeVRoll.getData();
}
#endif
float predictApogee(float alt, float v, float dragCoef)
{
  return alt + log((dragCoef * v * v / 9.81) + 1) / (2.0 * dragCoef); // copied from mower6.0
}
float inverseApogee()
{ // Binary Search
  float searchRangeH = angularAirBreakDragCoefRoll.getData();
  float searchRangeL = 0; // SEARCH RANGE IS 0<m<20
  float mid;              // required drag coef from the air breaks
  for (int i = 0; i < 10; i++)
  {
    mid = (searchRangeL + searchRangeH) / 2.0;
    float prediction = predictApogee(altitude, altitudeV, mid + rocketDragCoefRoll.getData());

    if (prediction < DESIRED_APOGEE)
    { // to the left of desired
      searchRangeH = mid;
    }
    else
    { // if (prediction > DESIRED_APOGEE) {
      searchRangeL = mid;
    }
  }
  return mid / searchRangeH * DRAG_FLAP_RANGE; // returns angle for ddrag flaps to occupy
}
void setupDataLog()
{
  String dataString = (String)groundTemperature + ',' +
                      (String)groundPressure + ',' +
                      (String)groundPressure + ',';
  // writeCSVLine(dataString);
}
String datalogHeader;
void logData()
{
  //Serial.println("Logging Data");
  // DATA INPUTS
  //String dataStringTest = "1";
  String dataString = String(timeElapsed) + ',' +   // rocket flight time
                      String(pressure,DATAPRECISION) + ',' +      // pressure
                      String(altitude,DATAPRECISION) + ',' +      // alt
                      String(altitudeV,DATAPRECISION) + ',' +     // velocity - baro derived
                      String(xAccel,DATAPRECISION) + ',' + // accel - imu derived
                      String(yAccel,DATAPRECISION) + ',' +
                      String(verticalAccel,DATAPRECISION) + ',' +
                      String(zenith,DATAPRECISION) + ',' +        // angle from the vertical

                      String(orientation.w,DATAPRECISION) + ',' + // rocket orientations
                      String(orientation.i,DATAPRECISION) + ',' +
                      String(orientation.j,DATAPRECISION) + ',' +
                      String(orientation.k,DATAPRECISION) + ',' +
                      String(bnoW,DATAPRECISION) + ',' +
                      String(bnoI,DATAPRECISION) + ',' +
                      String(bnoJ,DATAPRECISION) + ',' +
                      String(bnoK,DATAPRECISION) + ',' +
                      String(MagX,DATAPRECISION) + ',' +
                      String(MagY,DATAPRECISION) + ',' +
                      String(MagZ,DATAPRECISION) + ',' +
                      String(AccX,DATAPRECISION) + ',' +
                      String(AccY,DATAPRECISION) + ',' +
                      String(AccZ,DATAPRECISION) + ',' +

                      //String()
                      // // CONTROL
                      // (String)predictApogee(altitude, altitudeV, rocketDragCoef) + ',' + // apogee prediction
                      // (String)servoAngle + ',' +                                         // flap angle
                      // (String)solenoidState + ',' +                                      // solenoid
                      (String)flightState + ',' +                                        // flightState
                      // DRAG
                      // (String)totalDragCoef + ',' +         // totalDrag
                      // (String)angularRocketDragCoef + ',' + //
                      // (String)rocketDragCoef + ',' +
                      // (String)angularAirBreakDragCoef +
                      "\n";
  // EXTRA
  writeCSVLine(SD, logFilename.c_str(), dataString.c_str());
  //Serial.print(dataString);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial Begin");
  hspi.begin(SD_SCK,SD_MISO,SD_MOSI,SD_CS);
#ifdef SERVO
  servo1.attach(SERVO1_PIN);
  Serial.println("Servo Attached");
#endif
  // Disable Wi-Fi
  // WiFi.mode(WIFI_OFF);
  // WiFi.disconnect(true);
  // Disable Bluetooth
  // btStop();
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(1000);

#ifdef BARO
  baroSetup();
  Serial.println("BMP390 Attached");

#endif

#ifdef BNO055
  IMU_BNO055setup();
  Serial.println("BNO055 Attached");
  delay(500);

#endif
#ifdef SERVO
  for (int i = 0; i < 180; i++)
  {
    servo1.write(i);
    delay(10);
  }
  for (int i = 180; i > 0; i--)
  {
    servo1.write(i);
    delay(10);
  }
  Serial.println("Servo Test Done");
#endif
#ifdef BARO
  for (int i = 0; i < 120; i++)
  {
    baroData(); // fill up the rolling averages
    delay(10);
  }
  groundTemperature = temperatureRoll.getData();
  groundPressure = pressureRoll.getData();

  Serial.println("Ground Pressure " + (String)groundPressure);
  Serial.println("Ground Temperature " + (String)groundTemperature);
  lastAltitude = pressToAlt(pressureRoll.getData());

#endif
#ifdef BNO055
  imu::Quaternion quat = bno.getQuat();
  orientation.w = quat.w();
  orientation.i = quat.x();
  orientation.j = quat.y();
  orientation.k = quat.z();

  Serial.println("Orientation Initialized");
#endif
#ifdef SDCARD
  datalogHeader = "timeElapsed,pressure,alt,altV,xAccel,yAccel,verticalAccel,zenith,w,i,j,k,W,I,J,K,magX,magY,magZ,accX,accY,accZ,flightState\n";
  sdSetup();
  writeCSVLine(SD, logFilename.c_str(), datalogHeader.c_str());
#endif
  lastT = micros();
  Serial.println("start");
  delay(5000);
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
}

#ifdef BNO055

float degreesToRadians(float angle)
{
  return angle / 180 * PI;
}
float radiansToDegrees(float angle)
{
  return angle * 180 / PI;
}

float angleBetweenAxis(Quaternion orientation, String dir)
{
  Quaternion axis;//bruh
  if      (dir == "X+"){axis.setQuat(0, 1, 0, 0);}//shit code
  else if (dir == "X-"){axis.setQuat(0, -1, 0, 0);}
  else if (dir == "Y+"){axis.setQuat(0, 0, 1, 0);}
  else if (dir == "Y-"){axis.setQuat(0, 0, -1, 0);}
  else if (dir == "Z+"){axis.setQuat(0, 0, 0, 1);}
  else if (dir == "Z-"){axis.setQuat(0, 0, 0, -1);}

  Quaternion copy = orientation;
  orientation.mult(axis);
  orientation.mult(copy.inverse());
  // angle between vectors formula:
  return radiansToDegrees(acos(orientation.k / orientation.getLength()));
  // return orientation
}


float axisComponent(Quaternion orientation, float x, float y, float z, String dir ){
  Quaternion quatVector(0, x, y, z);
  Quaternion copy = orientation;
  orientation.mult(quatVector);
  orientation.mult(copy.inverse());
  //orientation is now a vector normalized from the inputted vector in the global x y & z directions
  if      (dir == "X+"){return orientation.i;}
  else if (dir == "X-"){return -orientation.i;}
  else if (dir == "Y+"){return orientation.j;}
  else if (dir == "Y-"){return -orientation.j;}
  else if (dir == "Z+"){return orientation.k;}
  else if (dir == "Z-"){return -orientation.k;}
  return 0;
}
float verticalAcceleration(Quaternion orientation, float accelX, float accelY, float accelZ)
{
  Quaternion accel(0, accelX, accelY, accelZ);
  Quaternion copy = orientation;
  orientation.mult(accel);
  orientation.mult(copy.inverse());
  // return i if x direction is facing top, j if y direction is facing top, and k if z direction is facing top
  // the values may be reversed if the direction is flipped
  return orientation.k;
}


void IMUdata()
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quat = bno.getQuat();
  /* Display the floating point data */
  // yaw += gyro.x()*deltaT/1000000;
  // pitch += gyro.y()*deltaT/1000000;//perpindicular to the ground
  // roll += gyro.z()*deltaT/1000000;
  // orientation.gyro(degreesToRadians(yaw),degreesToRadians(pitch),degreesToRadians(roll));
  // Quaternion orientation(quat.w(),quat.x(),quat.y(),quat.z());
  angularSpeed.gyro(degreesToRadians(gyro.x() * deltaT / 1000000),
                    degreesToRadians(gyro.y() * deltaT / 1000000),
                    degreesToRadians(gyro.z() * deltaT / 1000000));
  orientation.mult(angularSpeed);
  // imu::Quaternion quat = bno.getQuat();
  zenith = angleBetweenAxis(orientation,"Z+");
  verticalAccel = axisComponent(orientation, accel.x(), accel.y(), accel.z(), "Z+") - GRAVITY;
  xAccel = axisComponent(orientation, accel.x(), accel.y(), accel.z(), "X+");
  yAccel = axisComponent(orientation, accel.x(), accel.y(), accel.z(), "Y+");

  //trueAngle = magAngle(orientation, mag.x(), mag.y(), mag.z());
  bnoW = quat.w();
  bnoI = quat.x();
  bnoJ = quat.y();
  bnoK = quat.z();
  MagX = mag.x();
  MagY = mag.y();
  MagZ = mag.z();
  AccX = accel.x();
  AccY = accel.y();
  AccZ = accel.z();

  // Serial.print(zenith);
  // Serial.print(", ");
  // Serial.print(verticalAccel);
  // Serial.print(", ");
  // Serial.print(trueAngle);
  // Serial.print(", ");

  // Serial.print(orientation.w);Serial.print(", ");
  // Serial.print(orientation.i);Serial.print(", ");
  // Serial.print(orientation.j);Serial.print(", ");
  // Serial.print(orientation.k);Serial.print(", ");

  // Serial.print(accel.x());Serial.print(", ");
  // Serial.print(accel.y());Serial.print(", ");
  // Serial.print(accel.z());//Serial.print();

  // Serial.println();
  //  bno.getEvent(&BNO);
  //  Serial.print(", ");
  //  Serial.print(BNO.gyro.x);
  //  Serial.print(", ");
  //  Serial.print(BNO.gyro.y);
  //  Serial.print(", ");
  //  Serial.print(BNO.gyro.z);
  //  Serial.println();
  //  Serial.print(BNO.gyro.z);
  //Serial.println();

  // Quaternion data
  // imu::Quaternion quat = bno.getQuat();
  // Serial.print(quat.w()+3);Serial.print(",");
  // Serial.print(quat.x()+3);Serial.print(",");
  // Serial.print(quat.y()+3);Serial.print(",");
  // Serial.print(quat.z()+3);Serial.print(",");
}
#endif

void loop()
{
  deltaT = micros() - lastT;
  lastT = micros();
  if (startTimeStamp = 0)
  {
    timeElapsed = 0;
  }
  else
  {
    timeElapsed = millis() - startTimeStamp;
  }
  deltaTRoll.newData(deltaT);
  Serial.println(deltaTRoll.getData());

#ifdef BARO
  baroData();
  altitudeProcessing();
#endif
#ifdef BNO055
  IMUdata();
#endif
  // main control things
  switch (flightState)
  {
  case 0: // happy data printing mode
    logData();

    // Serial.print(deltaT);Serial.print(",");

    /*
      Serial.print(altitude);Serial.print(",");

      Serial.print(altitudeV);Serial.print(",");
      Serial.print(pressureRoll.getData());Serial.print(",");
      Serial.print(groundPressure);Serial.print(",");
      Serial.print(groundTemperature);Serial.print(",");*/
    // Serial.print(pressure/pressureMax*GRAPH_NORMALIZED_MAX);Serial.print(",");
    // Serial.println();
    break;
  case 1: // on the launch pad waiting to be ignited
    // do main rocket logic hereVVV
    Serial.println("waiting on launch pad");

    // flight switching code_______________
    if ((verticalAccel > 20) || (altitude > 10))
    { // tune these thresholds and statements
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 3)
      { // tune this thresholds
        flightState++;
        flightStateAdvancementTrigger = 0;
        startTimeStamp = millis(); // start the flight timer here
      }
    }
    else
    {
      flightStateAdvancementTrigger = 0;
    }
    // switching code end__________________
    break;
  case 2: // Accelerating Stage
    // do main rocket logic here VVV

    // flight switching code_______________
    if (!DUMMY_ROCKET && ((verticalAccel < 0) || (timeElapsed > 3000)))
    { // tune these thresholds and statements
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 3)
      { // tune this thresholds
        flightState++;
        flightStateAdvancementTrigger = 0;
      }
    }
    else
    {
      flightStateAdvancementTrigger = 0;
    }
    // switching code end__________________
    logData();
    break;
  case 3: // Upward freefall
    flighti++;
    // VARIABLES
    // Altitude
    // Altitude Acceleration
    // Time Elapsed
    // Zenith
    // Vertical Acceleration
    // do main rocket logic here VVV
    // Vi^2 = 2aD
    totalDragCoef = verticalAccel / pow(altitudeV, 2);
    if (altitude > DESIRED_APOGEE)
    {
      servoAngle = 90;
    }
    if (flighti < 100)
    { // servos at 0
      servoAngle = 0;
      rocketDragCoefRoll.newData(totalDragCoef);
    }
    else if (flighti < 200)
    { // servos at 90
      servoAngle = 90;
      angularAirBreakDragCoefRoll.newData(totalDragCoef - rocketDragCoefRoll.getData());
    }
    else
    {
      servoAngle = inverseApogee();
    }

    // flight switching code_______________
    if ((altitudeV < 0))
    { // tune these thresholds and statements
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 3)
      { // tune this thresholds
        flightState = 2;
        flightStateAdvancementTrigger = 0;
      }
    }
    else
    {
      flightStateAdvancementTrigger = 0;
    }
    // switching code end__________________
    logData();
    break;
  case 4: // decent
    // do main rocket logic here VVV
    if (altitude < (DESIRED_FLIGHT_TIME - timeElapsed) * PARACHUTE_TERM_VELOCITY)
    {
      solenoidState = 1;
    }
    // flight switching code_______________
    if (((-1 < altitudeV) && (altitudeV < 1)) || (timeElapsed > 100000))
    { // tune these thresholds and statements
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 3)
      { // tune this thresholds
        flightState = 2;
        flightStateAdvancementTrigger = 0;
      }
    }
    else
    {
      flightStateAdvancementTrigger = 0;
    }
    // switching code end__________________
    logData();
    break;
  case 5: // on the ground yippee
    // do whatever u want here
    break;
  }
#ifdef SERVO
  servo1.write(servoAngle);
#endif
  // Serial.print(",accelX:");
  // Serial.print(groundPressure);
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

  // altitude = pressToAlt(pressureRoll.getData());

  // Serial.print(", pressure:");
  // Serial.print(pressure);
  // Serial.print(", temperature:");
  // Serial.print(temperature);
  // Serial.print(", time:");
  // Serial.print(altitudeVRoll.getData());

  // Serial.print(", startingPressure:");
  // Serial.print(groundPressure);
  // Serial.print(", altitude2.0:");
  // Serial.print(altitude);

  // Serial.println( );
}
