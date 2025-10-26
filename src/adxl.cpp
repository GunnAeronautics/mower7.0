#include "headers/sensors/adxl.h"

/* Pin definitions for I2C:
#define ADXL343_SDA 21
#define ADXL343_SCL 22
*/

/* Variable definitions */
extern double xAccel = 0.0;
extern double yAccel = 0.0;
extern double zAccel = 0.0;

/* Assign a unique ID to this sensor at the same time */
/* Using I2C on default Wire bus */
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch(accel.getDataRate())
  {
    case ADXL343_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL343_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL343_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL343_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL343_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL343_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL343_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL343_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL343_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL343_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL343_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL343_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL343_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL343_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL343_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL343_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("I Don't know the data rate???? ");
      break;
  }
  Serial.println(" Hz");
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- ");

  switch(accel.getRange())
  {
    case ADXL343_RANGE_16_G:
      Serial.print  ("16 ");
      break;
    case ADXL343_RANGE_8_G:
      Serial.print  ("8 ");
      break;
    case ADXL343_RANGE_4_G:
      Serial.print  ("4 ");
      break;
    case ADXL343_RANGE_2_G:
      Serial.print  ("2 ");
      break;
    default:
      Serial.print  ("?? ");
      break;
  }
  Serial.println(" g");
}

void adxlSetup(void)
{
  /* Initialize I2C with custom pins */
  Wire.begin(ADXL343_SDA, ADXL343_SCL);

  /* Initialise the sensor */
  if(!accel.begin(ADXL343_ADR))  // Address 0x53 if SDO = GND , &Wire
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Could not find a valid ADXL343 sensor, check wiring!");
    while(1);
  }

  Serial.println("ADXL343 found!");

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_16_G);
  accel.setDataRate(ADXL343_DATARATE_100_HZ);
  // accel.setRange(ADXL343_RANGE_8_G);
  // accel.setRange(ADXL343_RANGE_4_G);
  // accel.setRange(ADXL343_RANGE_2_G);

  /* Display some basic information on this sensor */
  accel.printSensorDetails();
  displayDataRate();
  displayRange();
  Serial.println("Setup ADXL complete.");
}

void getADXLData(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  xAccel = event.acceleration.x;
  yAccel = event.acceleration.y;
  zAccel = event.acceleration.z;
  /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  // Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  // Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
}
