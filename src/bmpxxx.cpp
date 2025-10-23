#include "headers/sensors/bmpxxx.h"
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BMP5xx.h>
#include <bmp5_defs.h>

Adafruit_BMP3XX bmp390;
Adafruit_BMP5xx bmp580;
SPIClass mySPI;

BMPSensor activeSensor = BMP_NONE;
BaroData baroData_unified = {0};

// Rolling averages for sensor data
RollingAverage pressureRoll(4);
RollingAverage temperatureRoll(8);
RollingAverage altitudeVRoll(10);

// Raw sensor data
float lastAltitude = 0;
float lastAltitudeBuiltIn = 0;
float altitudeBuiltInV = 0;

BMPSensor baroSetup()
{
  mySPI.begin(BMP_SCK, BMP_MISO, BMP_MOSI);

  // Initialize both sensors
  bool bmp390Found = bmp390.begin_SPI(BMP390_CS, BMP_SCK, BMP_MISO, BMP_MOSI);
  bool bmp580Found = bmp580.begin(BMP580_CS, &mySPI);

  if (!bmp390Found && !bmp580Found)
  {
    Serial.println("No BMP sensors found!");
    activeSensor = BMP_NONE;
    return BMP_NONE;
  }

  if (bmp390Found)
  {
    Serial.println("BMP390 sensor found!");
    bmp390.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp390.setOutputDataRate(BMP3_ODR_100_HZ);
  }

  if (bmp580Found)
  {
    Serial.println("BMP580 sensor found!");
    bmp580.setPressureOversampling(BMP5XX_OVERSAMPLING_32X);
    bmp580.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    bmp580.setOutputDataRate(BMP5XX_ODR_100_2_HZ);
  }

  // Set active sensor - prefer dual reading if both available
  if (bmp390Found && bmp580Found)
  {
    Serial.println("Both sensors found! Using dual sensor mode with averaging.");
    activeSensor = BMP_390;  // Use this as flag, but will read both
  }
  else if (bmp390Found)
  {
    activeSensor = BMP_390;
  }
  else
  {
    activeSensor = BMP_580;
  }

  // Fill up rolling averages with ground calibration data
  for (int i = 0; i < 100; i++)
  {
    baroDataRead();
    delay(10);
  }

  baroData_unified.groundTemperature = temperatureRoll.getData();
  baroData_unified.groundPressure = pressureRoll.getData();

  return activeSensor;
}

void baroDataRead()
{
#ifdef SIMULATE
  baroData_unified.temperature = 20;
  baroData_unified.pressure = 1013.25;  // sea level
  return;
#endif

  float temp_390 = 0, pressure_390 = 0;
  float temp_580 = 0, pressure_580 = 0;
  bool bmp390_success = false;
  bool bmp580_success = false;

  // Read from BMP390 if available
  if (bmp390.begin_SPI(BMP390_CS, BMP_SCK, BMP_MISO, BMP_MOSI) || activeSensor == BMP_390)
  {
    if (bmp390.performReading())
    {
      temp_390 = bmp390.temperature;
      pressure_390 = (bmp390.pressure / 100.0);  // Convert to hPa
      bmp390_success = true;
    }
    else
    {
      Serial.println("BMP390 failed to perform reading!");
    }
  }

  // Read from BMP580 if available
  if (bmp580.begin(BMP580_CS, &mySPI) || activeSensor == BMP_580)
  {
    if (bmp580.performReading())
    {
      temp_580 = bmp580.temperature;
      pressure_580 = (bmp580.pressure / 100.0);  // Convert to hPa
      bmp580_success = true;
    }
    else
    {
      Serial.println("BMP580 failed to perform reading!");
    }
  }

  // Average the data from both sensors if both are available
  if (bmp390_success && bmp580_success)
  {
    baroData_unified.temperature = (temp_390 + temp_580) / 2.0;
    baroData_unified.pressure = (pressure_390 + pressure_580) / 2.0;
    baroData_unified.sensorType = BMP_390;  // Mark as dual reading
    Serial.println("Dual sensor reading - averaging BMP390 and BMP580");
  }
  else if (bmp390_success)
  {
    baroData_unified.temperature = temp_390;
    baroData_unified.pressure = pressure_390;
    baroData_unified.sensorType = BMP_390;
  }
  else if (bmp580_success)
  {
    baroData_unified.temperature = temp_580;
    baroData_unified.pressure = pressure_580;
    baroData_unified.sensorType = BMP_580;
  }
  else
  {
    Serial.println("No sensors available for reading!");
    return;
  }

  temperatureRoll.newData(baroData_unified.temperature);
  pressureRoll.newData(baroData_unified.pressure);
}

void altitudeProcessing(int deltaT)
{
  lastAltitude = baroData_unified.altitude;
  baroData_unified.altitude = pressToAlt(pressureRoll.getData());
  altitudeVRoll.newData((baroData_unified.altitude - lastAltitude) / deltaT * 1000000);
  baroData_unified.altitudeV = altitudeVRoll.getData();
}

float pressToAlt(float pres)
{
  // Returns altitude (m) from pressure in hecta pascals and temperature in celsius
  // https://www.mide.com/air-pressure-at-altitude-calculator
  // https://en.wikipedia.org/wiki/Barometric_formula
  return (float)(((273.0 + baroData_unified.groundTemperature) / (-.0065)) *
                 ((pow((pres / baroData_unified.groundPressure),
                       ((8.314 * .0065) / (GRAVITY * .02896)))) -
                  1.0));
}

BaroData getBaroData()
{
  return baroData_unified;
}

float getPressure()
{
  return baroData_unified.pressure;
}

float getTemperature()
{
  return baroData_unified.temperature;
}

float getAltitude()
{
  return baroData_unified.altitude;
}

float getAltitudeVelocity()
{
  return baroData_unified.altitudeV;
}

float getGroundPressure()
{
  return baroData_unified.groundPressure;
}

float getGroundTemperature()
{
  return baroData_unified.groundTemperature;
}

BMPSensor getActiveSensor()
{
  return activeSensor;
}

BaroData getBaroData_BMP390()
{
  if (activeSensor == BMP_390)
  {
    return baroData_unified;
  }
  // Return zeroed struct if BMP390 is not active
  BaroData empty = {0};
  empty.sensorType = BMP_NONE;
  return empty;
}

BaroData getBaroData_BMP580()
{
  if (activeSensor == BMP_580)
  {
    return baroData_unified;
  }
  // Return zeroed struct if BMP580 is not active
  BaroData empty = {0};
  empty.sensorType = BMP_NONE;
  return empty;
}

bool isBMP390Active()
{
  return activeSensor == BMP_390;
}

bool isBMP580Active()
{
  return activeSensor == BMP_580;
}
