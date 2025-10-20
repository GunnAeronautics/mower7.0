#include <FS.h>
#include <SD.h>
#include <Arduino.h>
#include <SPI.h>
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 5
#define MAX_STRING_SIZE 300


void sdSetup();
String getNewLogFilename();
void writeCSVLine(fs::FS &fs, const char *path, const char *message);
void sdTask(void *parameter);
void logData(String dataString);
