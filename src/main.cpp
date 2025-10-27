// Sup guys this is Tony the main programmer on the ASTRA24-25 tarc team
// make sure to use camelCase for variables and functions
// use ALLCAPS and UNDER_SCORES for definitions 
#include <Arduino.h>
#include <math.h>

// Configuration and constants
#include "headers/config/config.h"

// Homebrew libraries:
#include "headers/sensors/bmp390.h"
#include "headers/sensors/imu.h"
#include "headers/sensors/adxl.h"
#include "headers/utils/sdcard.h"
#include "headers/utils/servos.h"
#include "headers/math/KalmanVelocityEstimator.h"

// Last tuned before 2nd launch 3/22/25 pls don't make the servo horns slip or ur gonna have to do ts again
// Servo deployment angles configured in config.h:
//   - Drag flaps deployed: 45°
//   - Drag flaps undeployed: 135°
//   - Parachute deployed: 170°
//   - Parachute undeployed: 10°

// Time variables
unsigned long lastT;
static int deltaT;
unsigned long startTimeStamp = 0; // ms
long timeElapsed = 0;    // time elapsed in flight (ms)
long lastMovementT = 0;   // ms

// Deployment states
bool dragFlapDeployed = false;
bool parachuteDeployed = false;

// States & triggers
// VVV (IMPORTANT) set 0 for testing logging data, 1 for arming the rocket 
int flightState = 1;                   // state of the rocket's control
int flightStateAdvancementTrigger = 0; // counts number of times state switching event occurs
int flapDeploymentTrigger = 0;         // counts # of times you should deploy flaps
int numberOfNegatives = 0;
KalmanVelocityEstimator kalman(1, 3);

//everything related to timing
void timeStuff(){
  deltaT = micros() - lastT;//deltaT
  lastT = micros();
  //to set a timer for when the rocket launches
  if ((startTimeStamp == 0) && (flightState != 0))
  {
    timeElapsed = 0;
  }
  else
  {
    timeElapsed = millis() - startTimeStamp;
  }
}

float predictApogee(float alt, float v, float dragCoef)
{
  return alt + log((dragCoef * v * v / 9.81) + 1) / (2.0 * dragCoef); // copied from mower6.0
}
float predictApogeeIdeal(float alt, float v){
  return alt + pow(v,2)/2./GRAVITY;
}
float coefOfDrag(float accel,float v){
  float coef = abs(accel/v/v);
  return coef;
}
void writeDataHeader(){
  String datalogHeader = 
  "timeElapsed,"
  "pressure,"
  "alt,"
  "altV,"
  "globalVert"
  "so on"
  "\n";
  logData(datalogHeader);
}
void dataLogging(){
  String dataString = String(timeElapsed) + ',' +   // rocket flight time
                      String(pressure,DATAPRECISION) + ',' +      // pressure
                      String(altitude,DATAPRECISION) + ',' +      // alt
                      String(altitudeV,DATAPRECISION) + ',' +     // velocity - baro derived
                      String(zAccel,DATAPRECISION) + ',' +//global axis
                      String(AccY,DATAPRECISION) + ',' +//local axis
                      String(zenith,DATAPRECISION) + ',' +        // angle from the vertical
                      String(predictApogee(altitude, altitudeV, coefOfDrag(AccY,altitudeV)),DATAPRECISION) + ',' + // apogee prediction
                      String(coefOfDrag(AccY,altitudeV)) + ',' + 
                      (String)parachuteDeployed + ',' +
                      (String)dragFlapDeployed + ',' + 
                      (String)flightState + ',' + 
                      "\n";
  logData(dataString);
}
void setup()
{
  Serial.begin(115200);
  Serial.println("Serial Begin");

  baroSetup();
  Serial.println("BMP390 Attached");

  // IMU_BNO055setup();
  // Serial.println("BNO055 Attached");
  adxlSetup();
  Serial.println("Ground Pressure " + (String)groundPressure);
  Serial.println("Ground Temperature " + (String)groundTemperature);
  lastAltitude = pressToAlt(pressureRoll.getData());
  lastAltitudeBuiltIn = builtInAltitude;
  
  sdSetup();
  writeDataHeader();
  servoSetup();
  
#
  lastT = micros();
  Serial.println("start");

  pinMode(2,OUTPUT);//blue LED
  digitalWrite(2,HIGH);

  delay(5000);

}



void downwardLogic(){
  if (((float(DESIRED_FLIGHT_TIME - timeElapsed)/1000) * PARACHUTE_TERM_VELOCITY) > altitude ){//main logic works!!!
    deployChute();
    parachuteDeployed = true;
  }
  if (altitude < 20.){//saftey (falling down and altitude less than 20)
    deployChute();
    parachuteDeployed = true;
  }
}
void upwardLogic(){
  if (lastMovementT + 400 < millis()){
    if ((predictApogeeIdeal(altitude, altitudeV) < DESIRED_APOGEE)){//cooked
      moveFlaps(0);
      dragFlapDeployed = false;
    }
    else{
    if (altitude > DESIRED_APOGEE){//cooked
      moveFlaps(90);
      dragFlapDeployed = true;
    }
    if ((AccY < 0) && (AccY > -10)){//then u are inside of the freefall upward
      if (predictApogee(altitude,altitudeV,coefOfDrag(zAccel,altitudeV)) > DESIRED_APOGEE){
        flapDeploymentTrigger ++;
      }
      else{
        flapDeploymentTrigger = 0;
      }
    }
    if (flapDeploymentTrigger >= 3) {
      moveFlaps(90);
      dragFlapDeployed = true;
      lastMovementT = millis();
    }
    else{
      moveFlaps(0);
      dragFlapDeployed = false;
    }

    }
  }
}
float accelVelo = 0;
float predAlt = 0;
float error;
void loop()
{
  //get data stuff
  timeStuff();
  baroData();
  altitudeProcessing(deltaT);
  adxlSetup();
  // IMUdata(deltaT);


  //variables you have access to:
    // altitude
    // altitudeV
    // zAccel

  Serial.println(altitude);
  // main control things
  switch (flightState)
  {
  case 0: // happy data printing mode
    dataLogging();
  
    // accelVelo += (zAccel-9.8) * deltaT / 1000000;
    // error = altitude - predAlt;
    // accelVelo += error / deltaT*1000000 * 0.1;
    // predAlt = altitude + accelVelo *deltaT/1000000;
    
    // Serial.print(accelVelo);Serial.print(",");

    // Serial.print(altitudeV);Serial.print(",");
    // Serial.print(AccZ-9.8);Serial.print(",");
    // // Serial.print(String(AccX,DATAPRECISION));Serial.print(",");
    // // Serial.print(String(AccY,DATAPRECISION));Serial.print(",");
    // Serial.print(systemCali);Serial.print(",");
    // Serial.print(accelCali);Serial.print(",");
    // Serial.print(gyroCali);Serial.print(",");
    // Serial.print(magCali);Serial.print(",");

    // Serial.print(String(9.8-zAccel,DATAPRECISION));Serial.print(",");
    
    Serial.println(deltaT);
    break;

  case 1: // on the launch pad waiting to be ignited
    // do main rocket logic hereVVV
    Serial.println("waiting on launch pad");

    // flight switching code_______________
    if ((altitude > 230)){ // tune these thresholds and statements
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 3)
      { // tune this thresholds

        startTimeStamp = millis(); // start the flight timer here
        deployChute();      

        flightState++;
      }
    }

    else{flightStateAdvancementTrigger = 0;}
    // switching code end__________________
    break;
  case 2: // flight
    // dataLogging();
    Serial.print(timeElapsed);
    Serial.print(",");
    Serial.println(startTimeStamp);

    if (altitude < 10){numberOfNegatives ++;}
    else {numberOfNegatives = 0;}

    
    if ((numberOfNegatives > 10))//100 sec timer
    {
        // flightState++;
        undeployChute();
      }
    break;    
  // case 3:
  //   dataLogging();

  //   if (altitude <.5){numberOfNegatives ++;}
  //   else {numberOfNegatives = 0;}

    
  //   if ((numberOfNegatives > 20))//100 sec timer
  //   {
  //     deployChute();      
  //   }
  //   break;
  // }
    }
}
//wow looks like the code is super neat < 200 lines woaw
/*
If u are reading this it looks like ur the 1 other person who has read the code
Heres a donut:
                       $@@@$$$######                                           
                     $@@@@@@$$#**!=!!**##                                      
                   #$$@@@@@$$$##!=:::;=!*###                                   
                  *#$$$@@@@@$$$$#*=.~:;;!!*#$$$                                
                 =*##$$$$@@@@$$$$##*;,-:=!*#$$$@$#                             
                 =*###$$$$$$$$$$$$$$#*!-~=!*#$$@@@$#                           
                 ;!!*####$$$$$$$$$$$$$##*!==!*#$$$@$$*                         
                 ;=!**####$$$$$$$$$$$$$$###**!!*##$$$$#                        
                  ;=!!***######$$$$$$$$$$$$$############                       
                  ~;=!!!!**#*######$#$$$$$$$############*                      
                   ~:;=!!!*!***#*######################**=                     
                    -~;;=!!!!!********#############****!!=                     
                      -:;;;==!!!!!!**!****************!!=;                     
                       .-~::;;====!!!!!!*!!*!**!!!!!!==;;~                     
                         .--~::;;=;====!!=!==!!=!!===;;:~                      
                            .,-~~:::;:;;;;=;;==;;;;;::~-                       
                               ..,--~~~~::::::::::~--.                         
*/