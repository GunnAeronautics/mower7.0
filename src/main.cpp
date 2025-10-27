// Sup guys this is Tony the main programmer on the ASTRA24-25 tarc team
// make sure to use camelCase for variables and functions
// use ALLCAPS and UNDER_SCORES for definitions 
// 
#include <Arduino.h>
#include <math.h>

// Configuration and constants
#include "headers/config/config.h"

// Homebrew libraries:
#include "headers/sensors/bmpxxx.h"
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
int lastMovementT = 0;   // ms

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
void dataLogging(){
  BaroData baro = getBaroData();
  String dataString = String(timeElapsed) + ',' +   // rocket flight time
                      String(baro.pressure,DATAPRECISION) + ',' +      // pressure
                      String(baro.altitude,DATAPRECISION) + ',' +      // alt
                      String(baro.altitudeV,DATAPRECISION) + ',' +     // velocity - baro derived
                      String(zAccel,DATAPRECISION) + ',' +//global axis
                      String(AccY,DATAPRECISION) + ',' +//local axis
                      String(zenith,DATAPRECISION) + ',' +        // angle from the vertical
                      String(predictApogee(baro.altitude, baro.altitudeV, coefOfDrag(AccY,baro.altitudeV)),DATAPRECISION) + ',' + // apogee prediction
                      String(coefOfDrag(AccY,baro.altitudeV)) + ',' + 
                      (String)parachuteDeployed + ',' +
                      (String)dragFlapDeployed + ',' + 
                      (String)flightState + ',' + 
                      "\n";
  logData(dataString);
}
void setup()
{
  
  // Serial1.begin(115200, SERIAL_8N1, 16, 17); // debug serial
  Serial.begin(115200);
  Serial.println("Serial Begin");

  baroSetup();
  Serial.println("Barometric Sensor Setup Complete");

  // IMU_BNO055setup();
  // Serial.println("BNO055 Attached");
  adxlSetup();
  Serial.println("ADXL343 Attached");

  Serial.println("Ground Pressure " + (String)getGroundPressure());
  Serial.println("Ground Temperature " + (String)getGroundTemperature());
  lastAltitude = pressToAlt(getGroundPressure());
  lastAltitudeBuiltIn = getAltitude();
  
  sdSetup();
  
  servoSetup();

  lastT = micros();
  Serial.println("start");

  pinMode(2,OUTPUT);//blue LED
  digitalWrite(2,HIGH);

  delay(5000);

}



void downwardLogic(){
  if (((float(DESIRED_FLIGHT_TIME - timeElapsed)/1000) * PARACHUTE_TERM_VELOCITY) > getAltitude() ){//main logic works!!!
    deployChute();
    parachuteDeployed = true;
  }
  if (getAltitude() < 20.){//saftey (falling down and altitude less than 20)
    deployChute();
    parachuteDeployed = true;
  }
}
void upwardLogic(){
  if (lastMovementT + 400 < millis()){
    if ((predictApogeeIdeal(getAltitude(), getAltitudeVelocity()) < DESIRED_APOGEE)){//cooked
      moveFlaps(0);
      dragFlapDeployed = false;
    }
    else{
    if (getAltitude() > DESIRED_APOGEE){//cooked
      moveFlaps(90);
      dragFlapDeployed = true;
    }
    if ((AccY < 0) && (AccY > -10)){//then u are inside of the freefall upward
      if (predictApogee(getAltitude(),getAltitudeVelocity(),coefOfDrag(zAccel,getAltitudeVelocity())) > DESIRED_APOGEE){
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
  #ifdef DEBUG 
    delay(1000); // you only want a delay because serial printing is gets bogged down 
  #endif
  Serial.println("---------------------");
  //get data stuff
  Serial.println("Starting loop");
  Serial.println("Time Elapsed: " + String(timeElapsed));
  timeStuff();
  Serial.println("Delta T: " + String(deltaT));
  Serial.println("Main BARO ALT: " + String(getAltitude()) );
  Serial.println("BMP580 temp and pressure: " + String(getBaroData_BMP580_temp()) + 
  ", " + String(getBaroData_BMP580_pressure()));
  Serial.println("BMP390 temp and pressure: " + String(getBaroData_BMP390_temp()) + 
  ", " + String(getBaroData_BMP390_pressure()));
  Serial.println();
  baroDataRead();
  Serial.println("altitudeProcessing");
  altitudeProcessing(deltaT);
  // adxlSetup();
  // IMUdata(deltaT);

  Serial.println(getAltitude());
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
    if ((getAltitude() > 230)){ // tune these thresholds and statements
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

    if (getAltitude() < 10){numberOfNegatives ++;}
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
persons reading this count: 2
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