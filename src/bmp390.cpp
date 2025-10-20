#include "headers/sensors/bmp390.h"

RollingAverage pressureRoll(4);
RollingAverage temperatureRoll(8);
RollingAverage altitudeVRoll(10);
RollingAverage altitudeBuiltInVRoll(10);
Adafruit_BMP3XX bmp390;
Adafruit_BMP5xx bmp580;
SPIClass mySPI;


float pressure, temperature; // Pa C
float lastAltitude, lastAltitudeBuiltIn;
float altitudeV;     // in m/s

float groundTemperature, groundPressure, altitude, builtInAltitude; // alt in meters

float altitudeBuiltInV;
void baroSetup()
{
  mySPI.begin(BMP_SCK, BMP_MISO, BMP_MOSI);

  // SPI
  while (!bmp390.begin_SPI(BMP390_CS, BMP_SCK, BMP_MISO, BMP_MOSI))
  { // hardware SPI mode
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
  }
  while (!bmp580.begin(BMP580_CS, &mySPI))
  { // hardware SPI mode
    Serial.println("Could not find a valid BMP580 sensor, check wiring!");
  }
  // I2C
  //  if (!bmp.begin_I2C(0x77)) {
  //        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
  //        while (1);
  //    }

  //bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_1X);
  bmp390.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp390.setOutputDataRate(BMP3_ODR_100_HZ);
  for (int i = 0; i < 100; i++)
  {
    baroData(); // fill up the rolling averages
    delay(10);
  }
  groundTemperature = temperatureRoll.getData();
  groundPressure = pressureRoll.getData();
}
float pressToAlt(float pres)
{// returns alt (m) from pressure in hecta pascals and temperature in celsius - FULLY TESTED
  return (float)(((273.0 + groundTemperature) / (-.0065)) * ((pow((pres / groundPressure), ((8.314 * .0065) / (GRAVITY * .02896)))) - 1.0)); // https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula
}                                                                                                                                            
void baroData(void)
{ // gets barometric data from the sensor
#ifdef SIMULATE
  temperature = 20;
  pressure = pressureFunction();
  return
#endif
      if (!bmp390.performReading())
  {
    Serial.println("BMP390 Failed to perform reading :(");
    return;
  }

  temperature = bmp390.temperature; // C
  temperatureRoll.newData(temperature);

  pressure = (bmp390.pressure / 100.0); // hPa
  pressureRoll.newData(pressure);
  lastAltitudeBuiltIn = builtInAltitude;
  //builtInAltitude = bmp.readAltitude(groundPressure);
  Serial.println("BMP390: " + (String)bmp390.pressure);
  Serial.println("BMP580: " + (String)bmp580.pressure);

}
void altitudeProcessing(int deltaT)// in millis
{ // takes pressure data and transforms it into altitude and altitude velocity
  lastAltitude = altitude;
  altitude = pressToAlt(pressureRoll.getData());
  altitudeVRoll.newData((altitude - lastAltitude) / deltaT * 1000000);
  altitudeBuiltInV = ((builtInAltitude - lastAltitudeBuiltIn)/deltaT*1000000);
  altitudeV = altitudeVRoll.getData();
  //altitudeBuiltInV = altitudeBuiltInVRoll.getData();
}