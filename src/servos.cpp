#include <servos.h>
float minServo1 = 0;
float maxServo1 = 100;

#define SERVO1_PIN 13//45 is unarmed, 135 is release parachute
#define SERVO2_PIN 27
#define SERVO3_PIN 26
#define SERVO4_PIN 25

#define MIN_FLAP_STATE 0//angle for minimum flap
#define MAX_FLAP_STATE 90//angle for max flaps

#define PARACHUTE_UNDEPLOYED_STATE 135
#define PARACHUTE_DEPLOYED_STATE 45

Servo servo1; // create servo
Servo servo2; // create servo
Servo servo3; // create servo
Servo servo4; // create servo

int angleToBlink(double power){// scale of 0 to 90
    return 180 - (int)power;//tony rocket method
  }

void deployChute(){
    servo1.write(90);
  }
void moveFlaps(int power){
    servo2.write(angleToBlink(power));
    servo3.write(angleToBlink(power));
    servo4.write(angleToBlink(power));  
}
void servoSetup(){
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
  
    Serial.println("Servo Attached");


    for (int i = 90; i < 180; i++)
    {
      servo2.write(i);
      servo3.write(i);
      servo4.write(i);
  
      delay(10);
    }
    for (int i = 180; i > 90; i--)
    {
      servo2.write(i);
      servo3.write(i);
      servo4.write(i);
      delay(10);
    }
    servo1.write(180);
    servo2.write(angleToBlink(0));
    servo3.write(angleToBlink(0));
    servo4.write(angleToBlink(0));

    Serial.println("Servo Test Done");

}
