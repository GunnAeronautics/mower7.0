#include <FS.h>
#include <SD.h>
#include <Arduino.h>
#include <SPI.h>
#define SD_SCK 15
#define SD_MISO 4
#define SD_MOSI 17
#define SD_CS 16
#define MAX_STRING_SIZE 300


void sdSetup();
String getNewLogFilename();
void writeCSVLine(fs::FS &fs, const char *path, const char *message);
void sdTask(void *parameter);
void logData(String dataString);
