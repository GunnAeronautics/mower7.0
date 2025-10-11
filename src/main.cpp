// Sup guys this is Tony the main programmer on the ASTRA24-25 tarc team
// make sure to use camelCase for variables and functions
// use ALLCAPS and UNDER_SCORES for definitions 
#include <Arduino.h>
#include <math.h>
//homebrew libraries:
#include <baro.h>
#include <imu.h>
#include <sdcard.h>
#include <servos.h>
#include "KalmanVelocityEstimator.h"

#define GRAVITY 9.81              // m/s^2 hmm I wonder what that is


//"aim for the body and you'll miss, aim for the chest and you'll hit -Tony"
#define DESIRED_APOGEE 240.792        // m  790 ft specified by TARC guidelines
#define DESIRED_FLIGHT_TIME 42500 // miliseconds 41-44 sec regulation 2025 

//THESE NEED TO BE SET
#define PARACHUTE_TERM_VELOCITY 3.256 // m/s
#define COEF_DRAG_FLAPDEPLOYED 0.002213 //accelY / v / v   of the rocket when drag flaps are fully deployed

//last tuned before 2nd launch 3/22/25 pls don't make the servo horns slip or ur gonna have to do ts again

//drag flaps deployed angle: 45
//drag flaps undeployed angle: 135

//parachute deployed angle: 170
//parachute undeployed angle: 10

#define DATAPRECISION 4// decimals for sd card printing

//time
unsigned long lastT;
static int deltaT;
unsigned long startTimeStamp = 0; // ms
long timeElapsed = 0;    // time elapsed in flight (ms)
int lastMovementT = 0;//ms

//deployment states
bool dragFlapDeployed = false;
bool parachuteDeployed = false;

//states & triggers
//VVV (IMPORTANT) set 0 for testing logging data, 1 for arming the rocket 
int flightState = 0;                   // state of the rocket's control
int flightStateAdvancementTrigger = 0; // counts number of times state switching event occurs
int flapDeploymentTrigger = 0; //counts # of times yyou should deploy flaps
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
  Serial.begin(9600);
  Serial.println("Serial Begin");

  baroSetup();
  Serial.println("BMP390 Attached");

  IMU_BNO055setup();
  Serial.println("BNO055 Attached");

  Serial.println("Ground Pressure " + (String)groundPressure);
  Serial.println("Ground Temperature " + (String)groundTemperature);
  lastAltitude = pressToAlt(pressureRoll.getData());
  lastAltitudeBuiltIn = builtInAltitude;
  
  sdSetup();
  
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
  IMUdata(deltaT);


  // main control things
  switch (flightState)
  {
  case 0: // happy data printing mode
    // dataLogging();
  
    accelVelo += (zAccel-9.8) * deltaT / 1000000;
    error = altitude - predAlt;
    accelVelo += error / deltaT*1000000 * 0.1;
    predAlt = altitude + accelVelo *deltaT/1000000;
    
    Serial.print(accelVelo);Serial.print(",");

    Serial.print(altitudeV);Serial.print(",");
    Serial.print(AccZ-9.8);Serial.print(",");
    // Serial.print(String(AccX,DATAPRECISION));Serial.print(",");
    // Serial.print(String(AccY,DATAPRECISION));Serial.print(",");
    Serial.print(systemCali);Serial.print(",");
    Serial.print(accelCali);Serial.print(",");
    Serial.print(gyroCali);Serial.print(",");
    Serial.print(magCali);Serial.print(",");

    // Serial.print(String(9.8-zAccel,DATAPRECISION));Serial.print(",");
    
    Serial.println();
    break;

  case 1: // on the launch pad waiting to be ignited
    // do main rocket logic hereVVV
    Serial.println("waiting on launch pad");

    // flight switching code_______________
    if ((altitude > 10)){ // tune these thresholds and statements
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 3)
      { // tune this thresholds
        startTimeStamp = millis(); // start the flight timer here

        flightState++;
      }
    }

    else{flightStateAdvancementTrigger = 0;}
    // switching code end__________________
    break;
  case 2: // flight
    dataLogging();
    Serial.print(timeElapsed);
    Serial.print(",");
    Serial.println(startTimeStamp);
    if (altitudeV < 0){numberOfNegatives ++;}
    else {numberOfNegatives = 0;}

    if ((numberOfNegatives > 10)||(timeElapsed > 10000)){//FALLING DOWN
      downwardLogic();
    }
    else{//GOING UP
      upwardLogic();
    }


    if (timeElapsed > 100000)//100 sec timer
    {
        flightState++;
      }
    break;    
  case 3:
    flightState = 1;//just in case yk
    //done yippee landed
    break;
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