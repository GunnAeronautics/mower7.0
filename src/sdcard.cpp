
#include <sdcard.h>
String logFilename;
String datalogHeader;
SPIClass hspi(HSPI);
SPIClass vspi(VSPI);
TaskHandle_t Task1;
QueueHandle_t xQueue;
volatile int howManyDatas = 0;
bool lastLED = true;

String getNewLogFilename(){
  int fileIndex = 1;
  String filename;
  do
  {
    filename = "/datalog" + String(fileIndex) + ".csv";
    fileIndex++;
  } while (SD.exists(filename.c_str()));
  return filename;
}
// appendFile(SD, "/hello.txt", "World!\n");
void writeCSVLine(fs::FS &fs, const char *path, const char *message)
{
  //Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    //Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    //Serial.println("Message appended");
    digitalWrite(2,!lastLED);
    lastLED = !lastLED;
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}

void sdTask(void *parameter) {
  char *message = (char *)parameter;  
  writeCSVLine(SD, logFilename.c_str(), message);
}
void sdSetup()
{
  hspi.begin(SD_SCK,SD_MISO,SD_MOSI,SD_CS);

  if (!SD.begin(SD_CS, hspi))
  {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
  {
    Serial.println("MMC");
  }
  else if (cardType == CARD_SD)
  {
    Serial.println("SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    Serial.println("SDHC");
  }
  else
  {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  logFilename = getNewLogFilename();
  Serial.println(logFilename);

  datalogHeader = "timeElapsed,pressure,alt,altV,globalVertAccel,localVertAccel,zenith,predApogee,parachuteDeployed?,flapsDeployed?,flightState\n";
  writeCSVLine(SD, logFilename.c_str(), datalogHeader.c_str());


  // xQueue = xQueueCreate(10,MAX_STRING_SIZE);//~300 bytes per line
  // delay(1000);

  
  //writeCSVLine(SD, logFilename.c_str(), "Time, Baro");
}



void logData(String dataString)
{
  //Serial.println("Logging Data");
  // DATA INPUTS
  //String dataStringTest = "1";
  // if (howManyDatas < 10){// enable or disable logging
  // char buffer[MAX_STRING_SIZE] = "";
  //if (xQueueSend(xQueue, "1", portMAX_DELAY) != pdPASS){
// //UNDO COMMENTS BELOWO
  // if (xQueueSend(xQueue, dataString.c_str(), 0) != pdPASS){
  //   Serial.println("Queue Full! Data Lost.");
  // } 
  
  
  writeCSVLine(SD, logFilename.c_str(), dataString.c_str());
  // xTaskCreatePinnedToCore(
  //   sdTask,      // Function to execute
  //   "Blink Task",   // Task name
  //   8192,           // Stack size (in words)
  //   dataString.c_str(),           // Task input parameter
  //   1,              // Priority (higher number = higher priority)
  //   &Task1,         // Task handle
  //   1               // Core 1
  

  //Serial.println(dataString);
//NOW STOP UNDO
    //Serial.println(uxQueueSpacesAvailable(xQueue));

    // Serial.println("SD queue full");
    // queueOverflowCoolown = millis()+200;
  
  // EXTRA
  //writeCSVLine(SD, logFilename.c_str(), dataString.c_str());
  //writeCSVLine(SD, logFilename.c_str(), dataString.c_str());
  //Serial.println(dataString);
 }

