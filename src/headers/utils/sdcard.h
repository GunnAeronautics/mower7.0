#ifndef SDCARD_H
#define SDCARD_H

#include <FS.h>
#include <SD.h>
#include <Arduino.h>
#include <SPI.h>
#include "../config/config.h"

void sdSetup();
String getNewLogFilename();
void writeCSVLine(fs::FS &fs, const char *path, const char *message);
void sdTask(void *parameter);
void logData(String dataString);

#endif // SDCARD_H
