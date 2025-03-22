
#include <sdcard.h>
String logFilename;
String datalogHeader;
SPIClass hspi(HSPI);
SPIClass vspi(VSPI);
TaskHandle_t Task1;
QueueHandle_t xQueue;
volatile int howManyDatas = 0;

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
    Serial.println("Message appended");
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}

void sdTask(void *parameter) {
  char receivedData[MAX_STRING_SIZE];
  char lumpData[5000] = "";
  int lastTsdWrite;
  int lastWriteTTTTT;//timebetween writing
  datalogHeader = "timeElapsed,pressure,alt,altV,xAccel,yAccel,verticalAccel,zenith,w,i,j,k,W,I,J,K,magX,magY,magZ,accX,accY,accZ,flightState\n";

  writeCSVLine(SD, logFilename.c_str(), datalogHeader.c_str());
  delay(1000);
  while (1) {
    if ((howManyDatas < 10)){
      
      if (xQueueReceive(xQueue, &receivedData,0) == pdPASS){//if theres new data in the queue
      strcat(lumpData,receivedData);
      howManyDatas += 1;
      }
      else if (howManyDatas != 0){
        writeCSVLine(SD, logFilename.c_str(), lumpData);
        //Serial.println(lumpData);
        memset(lumpData, 0, sizeof(lumpData));
        howManyDatas = 0;
      }
    }
    
    else{
      writeCSVLine(SD, logFilename.c_str(), lumpData);
      //Serial.println(lumpData);
      memset(lumpData, 0, sizeof(lumpData));

      howManyDatas = 0;
    }
    
    // }
    // else {
    //   // if ((howManyDatas <= 5)){//((millis()-lastTsdWrite) < 50)||
    //   //   continue;
    //   // }
    //   lastTsdWrite = millis();



    //   // Serial.print("emptyLumpData datas going to upload: ");
    //   // Serial.print(howManyDatas);
    //   // Serial.print(" timeTakenBetweenEmpty: ");
    //   // Serial.print(micros()-lastTsdWrite);

    //   howManyDatas = 0;
    //   //delay(70);
      // lastWriteTTTTT = micros();
      // delete[] zzz//reset the memory of the lumpData
      // Serial.print("Time taken to upload: ");
      // Serial.print(micros()-lastWriteTTTTT);
      
    }
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


  xQueue = xQueueCreate(10,MAX_STRING_SIZE);//~300 bytes per line
  delay(1000);
  xTaskCreatePinnedToCore(
    sdTask,      // Function to execute
    "Blink Task",   // Task name
    8192,           // Stack size (in words)
    NULL,           // Task input parameter
    1,              // Priority (higher number = higher priority)
    &Task1,         // Task handle
    1               // Core 1
  );

  
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
  #ifdef SDCARD
  if (xQueueSend(xQueue, dataString.c_str(), 0) != pdPASS){
    Serial.println("Queue Full! Data Lost.");
  } 
  #else
  Serial.println(dataString);
  #endif
//NOW STOP UNDO
    //Serial.println(uxQueueSpacesAvailable(xQueue));

    // Serial.println("SD queue full");
    // queueOverflowCoolown = millis()+200;
  
  // EXTRA
  //writeCSVLine(SD, logFilename.c_str(), dataString.c_str());
  //writeCSVLine(SD, logFilename.c_str(), dataString.c_str());
  //Serial.println(dataString);
 }

