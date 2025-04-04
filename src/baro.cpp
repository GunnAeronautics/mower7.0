#include <baro.h>

RollingAverage pressureRoll(4);
RollingAverage temperatureRoll(8);
RollingAverage altitudeVRoll(10);
RollingAverage altitudeBuiltInVRoll(10);
Adafruit_BMP3XX bmp;

float pressure, temperature; // Pa C
float lastAltitude, lastAltitudeBuiltIn;
float altitudeV;     // in m/s

float groundTemperature, groundPressure, altitude, builtInAltitude; // alt in meters

float altitudeBuiltInV;
void baroSetup()
{
  // SPI
  while (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI))
  { // hardware SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
  // I2C
  //  if (!bmp.begin_I2C(0x77)) {
  //        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
  //        while (1);
  //    }

  //bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_1X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);
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
      if (!bmp.performReading())
  {
    Serial.println("BMP390 Failed to perform reading :(");
    return;
  }

  temperature = bmp.temperature; // C
  temperatureRoll.newData(temperature);

  pressure = (bmp.pressure / 100.0); // hPa
  pressureRoll.newData(pressure);
  lastAltitudeBuiltIn = builtInAltitude;
  //builtInAltitude = bmp.readAltitude(groundPressure);
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