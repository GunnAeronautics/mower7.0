// Sup guys this is Tony the main programmer on the ASTRA24-25 tarc team
// make sure to use camelCase for variables and functions
// use ALLCAPS for definitions
// and utilize ifdefs

// #include <WiFi.h>
// #include <BluetoothSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <math.h>         //for inverse trig functions
#include <quaternion.h>
//homebrew libraries:
#include <baro.h>
#include <imu.h>
#include <sdcard.h>
#include <servos.h>
#define DUMMY_ROCKET true // if dummy rocket then no servo or solenoid

#define BARO
// #define MPU6050
#define BNO055
//#define SDCARD
//#define BLUETOOTH
// #define groundTesting
// #define SIMULATE
#define SERVO

#define DATAPRECISION 2// decimals

#define GRAVITY 9.81              // m/s^2
#define DESIRED_APOGEE 250        // m specified by TARC guidelines
#define DESIRED_FLIGHT_TIME 45000 // miliseconds
// #define PARACHUTE_DRAG_COEF 0.05//area included
#define DRAG_FLAP_RANGE 90         // 0 for full retraction, 90 for full extension
#define PARACHUTE_TERM_VELOCITY 10 // m/s
// #define GRAPH_NORMALIZED_MAX 1000

// ground references for pressure function to work

float totalDragCoef;
float angularRocketDragCoef = 0; // area included
float rocketDragCoef = 0;        //
float angularAirBreakDragCoef = 0;

unsigned long lastT;
static int deltaT;
int startTimeStamp = 0; // ms
int timeElapsed = 0;    // time elapsed in flight (ms)

int flightState = 0;                   // state of the rocket's control
int flightStateAdvancementTrigger = 0; // counts number of times state switching event occurs
int flighti = 0;                       // something advanced
int servoDragForce = 0;                        // servo angle for drag flaps, 0 for fully retracted 90 for fully extended, random variable
// webhooktest
//  put function declarations here:
bool lastLED = true;

// RollingAverage angularRocketDragCoefRoll;
RollingAverage rocketDragCoefRoll(40);
RollingAverage angularAirBreakDragCoefRoll(40);

RollingAverage deltaTRoll(300);

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
#ifdef SDCARD
#endif


void dataLogging(){
  String dataString = String(timeElapsed) + ',' +   // rocket flight time
                      String(pressure,DATAPRECISION) + ',' +      // pressure
                      String(altitude,DATAPRECISION) + ',' +      // alt
                      String(altitudeV,DATAPRECISION) + ',' +     // velocity - baro derived
                      String(xAccel,DATAPRECISION) + ',' + // accel - imu derived
                      String(yAccel,DATAPRECISION) + ',' +
                      String(verticalAccel,DATAPRECISION) + ',' +
                      String(zenith,DATAPRECISION) + ',' +        // angle from the vertical

                      // String(orientation.w,DATAPRECISION) + ',' + // rocket orientations
                      // String(orientation.i,DATAPRECISION) + ',' +
                      // String(orientation.j,DATAPRECISION) + ',' +
                      // String(orientation.k,DATAPRECISION) + ',' +
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
                      (String)servoDragForce + ',' +                                         // flap angle
                      // (String)solenoidState + ',' +                                      // solenoid
                      (String)flightState + ',' +                                        // flightState
                      // DRAG
                      (String)totalDragCoef + ',' +         // totalDrag
                      (String)angularRocketDragCoef + ',' + //
                      (String)rocketDragCoef + ',' +
                      (String)angularAirBreakDragCoef +
                      "\n";
  logData(dataString);
}
void setup()
{
  Serial.begin(115200);
  Serial.println("Serial Begin");

  baroSetup();
  Serial.println("BMP390 Attached");

  IMU_BNO055setup();
  Serial.println("BNO055 Attached");

  delay(500);

  Serial.println("Ground Pressure " + (String)groundPressure);
  Serial.println("Ground Temperature " + (String)groundTemperature);
  lastAltitude = pressToAlt(pressureRoll.getData());
  lastAltitudeBuiltIn = builtInAltitude;
  
  sdSetup();
  

#
  lastT = micros();
  Serial.println("start");
  delay(5000);
  
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
}


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
  //Serial.println(deltaTRoll.getData());

#ifdef BARO
  baroData();
  altitudeProcessing(deltaT);
#endif
#ifdef BNO055
  IMUdata(deltaT);
#endif
  // main control things
  switch (flightState)
  {
  case 0: // happy data printing mode
    //logData();
    Serial.print(altitude);
    Serial.print(",");
    Serial.print(altitudeV);
    Serial.print(",");
    Serial.print(altitudeBuiltInV);
    Serial.print(",");
    Serial.println(builtInAltitude);
    //Serial.print(",");
    //Serial.println(deltaT/1000.);
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
    if ((verticalAccel > 10) || (altitude > 5))
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
    if ((verticalAccel < 0) || (timeElapsed > 2000))
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
    dataLogging();
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
      servoDragForce = 90;
    }
    if (flighti < 100)
    { // servos at 0
      servoDragForce = 0;
      rocketDragCoefRoll.newData(totalDragCoef);
    }
    else if (flighti < 200)
    { // servos at 90
      servoDragForce = 90;
      angularAirBreakDragCoefRoll.newData(totalDragCoef - rocketDragCoefRoll.getData());
    }
    else
    {
      servoDragForce = inverseApogee();
    }

    // flight switching code_______________
    if ((altitudeV < 0))
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
    dataLogging();
    break;
  case 4: // decent
    // do main rocket logic here VVV
    if (altitude < (DESIRED_FLIGHT_TIME - timeElapsed) * PARACHUTE_TERM_VELOCITY
        || altitude < 60)
    {
      deployChute();
    }
    // flight switching code_______________
    if (((-1 < altitudeV) && (altitudeV < 1)) || (timeElapsed > 100000))
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
    dataLogging();
    break;
  case 5: // on the ground yippee
    Serial.println("Done");
    // do whatever u want here
    //flighti =0;
    //flightState = 1;
    break;
  }
#ifdef SERVO
  
#endif
}