// Sup guys this is Tony the main programmer on the ASTRA24-25 tarc team
// make sure to use camelCase for variables and functions
// use ALLCAPS for definitions
// and utilize ifdefs

// #include <WiFi.h>
// #include <BluetoothSerial.h>
#include <Arduino.h>
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

#define DATAPRECISION 6// decimals

#define GRAVITY 9.81              // m/s^2
#define DESIRED_APOGEE 250        // m specified by TARC guidelines
#define DESIRED_FLIGHT_TIME 45000 // miliseconds
// #define PARACHUTE_DRAG_COEF 0.05//area included
#define DRAG_FLAP_RANGE 90         // 0 for full retraction, 90 for full extension
#define PARACHUTE_TERM_VELOCITY 3.256 // m/s
// #define GRAPH_NORMALIZED_MAX 1000

// ground references for pressure function to work
float deployed = false;
float totalDragCoef;
float angularRocketDragCoef = 0; // area included
float rocketDragCoef = 0;        //
float angularAirBreakDragCoef = 0;
float numberOfNegatives = 0;
unsigned long lastT;
static int deltaT;
int startTimeStamp = 0; // ms
int timeElapsed = 0;    // time elapsed in flight (ms)

int flightState = 1;                   // state of the rocket's control
int flightStateAdvancementTrigger = 0; // counts number of times state switching event occurs
int flighti = 0;                       // something advanced
int servoDragForce = 0;                        // servo angle for drag flaps, 0 for fully retracted 90 for fully extended, random variable
// webhooktest
//  put function declarations here:

// RollingAverage angularRocketDragCoefRoll;
RollingAverage angularAirBreakDragCoefRoll(40);

RollingAverage deltaTRoll(300);

float predictApogee(float alt, float v, float accel)
{
  return alt + log((accel / 9.81) + 1) / (2.0 * accel /v/v); // copied from mower6.0
}
float inverseApogee()
{ // Binary Search
  float searchRangeH = angularAirBreakDragCoefRoll.getData();
  float searchRangeL = 0; // SEARCH RANGE IS 0<m<20
  float mid;              // required drag coef from the air breaks
  for (int i = 0; i < 10; i++)
  {
    mid = (searchRangeL + searchRangeH) / 2.0;
    float prediction = predictApogee(altitude, altitudeV, mid + 1);

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
                      String(zAccel,DATAPRECISION) + ',' +
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
                      (String)predictApogee(altitude, altitudeV, zAccel) + ',' + // apogee prediction
                      (String)deployed + ',' +

                      // (String)servoDragForce + ',' +                                         // flap angle
                      // (String)solenoidState + ',' +                                      // solenoid
                      (String)flightState + ',' +                                        // flightState
                      // DRAG
                      // (String)totalDragCoef + ',' +         // totalDrag
                      // (String)angularRocketDragCoef + ',' + //
                      // (String)rocketDragCoef + ',' +
                      // (String)angularAirBreakDragCoef +
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
  
  servoSetup();
  
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

  baroData();
  altitudeProcessing(deltaT);
  IMUdata(deltaT)
  ;
  // main control things
  switch (flightState)
  {
  case 0: // happy data printing mode
    dataLogging();

    //Serial.print(",");
    //Serial.println(deltaT/1000.);
    // Serial.print(deltaT);Serial.print(",");

    
    Serial.print(predictApogee(altitude,altitudeV,zAccel));Serial.print(",");
    Serial.println();

    // Serial.print(pressure/pressureMax*GRAPH_NORMALIZED_MAX);Serial.print(",");
    // Serial.println();
    break;
  case 1: // on the launch pad waiting to be ignited
    // do main rocket logic hereVVV
    Serial.println("waiting on launch pad");

    // flight switching code_______________
    if ((zAccel > 10) || (altitude > 5))
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
  case 2: // flight
    dataLogging();

    // d = vt
    if ((DESIRED_FLIGHT_TIME - timeElapsed * PARACHUTE_TERM_VELOCITY) > altitude ){//main logic
      deployChute();
    }
    
    if (altitude > 150){
      moveFlaps(90);
    }
    else{
      moveFlaps(0);
    }
    
    if (altitudeV < 0){
      numberOfNegatives ++;
    }
    else {
      numberOfNegatives = 0;
    }
    
    if ((numberOfNegatives > 50)&&(altitude < 50.)){//saftey (falling down and altitude less than 50)
      deployChute();
      deployed = true;

    }
 
  
    if ((millis() - startTimeStamp) > 100000)//100 sec timer
    { // tune these thresholds and statements
        flightState++;
      }
    break;    
  case 3:
    flightState = 1;//just in case yk
    //done yippee landed
    break;
  }

}