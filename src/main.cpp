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

#ifdef SERVO
#define SERVO1_PIN 4
#endif
#define SDA_PIN 21
#define SCL_PIN 22

#ifdef SDCARD
#include <FS.h>
#include <SD.h>
#define SD_SCK 14
#define SD_MISO 15
#define SD_MOSI 2
#define SD_CS 4
#endif

#ifdef BARO
#include <Adafruit_BMP3XX.h>
// #define BMP390_I2C_ADDRESS 0x77
// SPI:
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

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS, &Wire);

#endif

#define GRAVITY 9.81              // m/s^2
#define DESIRED_APOGEE 250        // m specified by TARC guidelines
#define DESIRED_FLIGHT_TIME 45000 // miliseconds
// #define PARACHUTE_DRAG_COEF 0.05//area included
#define DRAG_FLAP_RANGE 90         // 0 for full retraction, 90 for full extension
#define PARACHUTE_TERM_VELOCITY 10 // m/s
// #define GRAPH_NORMALIZED_MAX 1000

float GyroX, GyroY, GyroZ; // deg / sec
float AccX, AccY, AccZ;    // Gs

float pressure, temperature; // Pa C
float lastAltitude;
float altitudeV;     // in m/s
float verticalAccel; // in m/s^2
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

Quaternion orientation;
Quaternion angularSpeed;

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
}
#endif
void checkMax(float newData, float *maxData)
{ // updates a max value by a new datum
  if (newData > *maxData)
  {
    *maxData = newData;
  }
}
#ifdef MPU6050
void IMUsetup()
{
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(MPU_ADDRESS); // address of mpu6050
  Wire.write(0x6B);
  Wire.write(0x00); // reseting MPU
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS); // address of mpu6050
  Wire.write(0x1A);                    // switch on low pass filter
  Wire.write(0x02);                    //~100 hz
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B); // gyroscope config
  Wire.write(0x18); // configure range 250 deg/s
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C); // accelerometer config
  Wire.write(0x18); // configure range 16g
  Wire.endTransmission();
}

void accelData(void)
{

  Wire.beginTransmission(MPUADDRESS);
  Wire.write(0x3B); // start of Accel out table
  Wire.endTransmission();
  Wire.requestFrom(MPUADDRESS, 6);
  // 6 registers, 3 for each dimension (xyz) and 2 for the accuracy
  // each register has 8 bits
  int16_t AccelXLSB = Wire.read() << 8 | Wire.read(); // X
  int16_t AccelYLSB = Wire.read() << 8 | Wire.read(); // Y
  int16_t AccelZLSB = Wire.read() << 8 | Wire.read(); // Z
  AccX = (float)AccelXLSB / 2048;
  AccY = (float)AccelYLSB / 2048;
  AccZ = (float)AccelZLSB / 2048;
}
void gyroData(void)
{

  Wire.beginTransmission(MPUADDRESS);
  Wire.write(0x43); // start of Baro out table
  Wire.endTransmission();
  Wire.requestFrom(MPUADDRESS, 6);
  int16_t GyroXLSB = Wire.read() << 8 | Wire.read(); // X
  int16_t GyroYLSB = Wire.read() << 8 | Wire.read(); // Y
  int16_t GyroZLSB = Wire.read() << 8 | Wire.read(); // Z
  GyroX = (float)GyroXLSB / 131;
  GyroY = (float)GyroYLSB / 131;
  GyroZ = (float)GyroZLSB / 131;
}
#endif
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
  if (!SD.begin(5))
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
}

// appendFile(SD, "/hello.txt", "World!\n");
void writeCSVLine(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    Serial.println("Message appended");
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
  if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI))
  { // hardware SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
  // I2C
  //  if (!bmp.begin_I2C(0x77)) {
  //        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
  //        while (1);
  //    }
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

void logData()
{
  Serial.println("Logging Data");
  // DATA INPUTS
  String dataString = (String)timeElapsed + ',' +   // rocket flight time
                      (String)pressure + ',' +      // pressure
                      (String)altitude + ',' +      // alt
                      (String)altitudeV + ',' +     // velocity - baro derived
                      (String)verticalAccel + ',' + // accel - imu derived
                      (String)zenith + ',' +        // angle from the vertical
                      (String)orientation.w + ',' + // rocket orientations
                      (String)orientation.i + ',' +
                      (String)orientation.j + ',' +
                      (String)orientation.k + ',' +
                      // CONTROL
                      (String)predictApogee(altitude, altitudeV, rocketDragCoef) + ',' + // apogee prediction
                      (String)servoAngle + ',' +                                         // flap angle
                      (String)solenoidState + ',' +                                      // solenoid
                      (String)flightState + ',' +                                        // flightState
                      // DRAG
                      (String)totalDragCoef + ',' +         // totalDrag
                      (String)angularRocketDragCoef + ',' + //
                      (String)rocketDragCoef + ',' +
                      (String)angularAirBreakDragCoef +
                      "\n";
  // EXTRA
  writeCSVLine(SD, logFilename.c_str(), dataString.c_str());
  Serial.println(dataString);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial Begin");

#ifdef SERVO
  servo1.attach(SERVO1_PIN);
  Serial.println("Servo Attached");
#endif
  // Disable Wi-Fi
  // WiFi.mode(WIFI_OFF);
  // WiFi.disconnect(true);
  // Disable Bluetooth
  // btStop();
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);

#ifdef MPU6050
  IMUsetup();
#endif
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
  sdSetup();
#endif
  lastT = micros();
  Serial.println("start");
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
float angleBetweenVertical(Quaternion orientation)
{
  // Quaternion vertical(0,0,1,0);//some lateral direction
  Quaternion vertical(0, 0, 0, 1); // Z direction
  // angle between Z axis: 0,0,0,1
  // angle between Y axis: 0,0,1,0
  // angle between X axis: 0,1,0,0
  // if flippde then reverse the axis ex: 0,-1,0,04
  // Quaternion vertical(0,0,0,0);//in the opposite lateral direction to the top

  Quaternion copy = orientation;
  orientation.mult(vertical);
  orientation.mult(copy.inverse());
  // angle between vectors formula:
  return radiansToDegrees(acos(orientation.k / orientation.getLength()));
  // return orientation
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
float magAngle(Quaternion orientation, float magX, float magY, float magZ)
{
  Quaternion mag(0, magX, magY, magZ);
  Quaternion copy = orientation;
  orientation.mult(mag);
  orientation.mult(copy.inverse());
  // return i if x direction is facing top, j if y direction is facing top, and k if z direction is facing top
  // the values may be reversed if the direction is flipped
  return radiansToDegrees(atan(orientation.i / orientation.j));
}
float trueAngle;
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
  // imu::Quaternion quat = bno.getQuat();
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
  zenith = angleBetweenVertical(orientation);
  verticalAccel = verticalAcceleration(orientation, accel.x(), accel.y(), accel.z()) - GRAVITY;
  trueAngle = magAngle(orientation, mag.x(), mag.y(), mag.z());
  Serial.print(zenith);
  Serial.print(", ");
  Serial.print(verticalAccel);
  Serial.print(", ");
  Serial.print(trueAngle);
  Serial.print(", ");

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
  Serial.println();

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
#ifdef MPU6050
  accelData();
  gyroData();
#endif
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
