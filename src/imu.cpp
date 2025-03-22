


#include <imu.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS, &Wire);
Quaternion orientation;
Quaternion angularSpeed;

float GyroX, GyroY, GyroZ; // deg / sec
float AccX, AccY, AccZ;    // Gs
float MagX, MagY, MagZ;    // Gs

float verticalAccel; // in m/s^2
float xAccel; // in m/s^2
float yAccel; // in m/s^2

float heading;        // in radians
float zenith;        // in radians


float yaw;
float pitch;
float roll;


float bnoW;
float bnoI;
float bnoJ;
float bnoK;

void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(0x28); // BNO055 I2C address
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

void IMU_BNO055setup()
  {
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(3000);
  
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

    imu::Quaternion quat = bno.getQuat();
    orientation.w = quat.w();
    orientation.i = quat.x();
    orientation.j = quat.y();
    orientation.k = quat.z();
  
    Serial.println("Orientation Initialized");
  
  }
  

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
  
  
  void IMUdata(int deltaT)
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