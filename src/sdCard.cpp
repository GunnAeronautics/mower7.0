#include <sdCard.h>
#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>

#define datalogHeader "timeElapsed,pressure,alt,altV,xAccel,yAccel,verticalAccel,zenith,w,i,j,k,W,I,J,K,magX,magY,magZ,accX,accY,accZ,flightState\n"

sdCard::sdCard(char sdCSPin, SPIClass sdSPI)
{
    cardMounted = true;
    if (!SD.begin(sdCSPin, sdSPI))
    {
        cardMounted = true;
        Serial.println("Card Mount Failed");
        return;
    }
    this->cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    switch (cardType)
    {
    case CARD_MMC:
        Serial.println("MMC");
        break;
    case CARD_SD:
        Serial.println("SDSC");
        break;
    case CARD_SDHC:
        Serial.println("SDHC");
        break;
    default:
        Serial.println("UNKNOWN");
        break;
    }

    this->cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    this->logFilename = getNewLogFilename();
    Serial.println(logFilename);
    writeCSVLine(datalogHeader);
}
void sdCard::writeCSVLine(String message)
{
    const char *convertedMessage = message.c_str();
    if (!cardMounted)
    {
        Serial.println("Card not mounted");
        return;
    }
    Serial.printf("Appending to file: %s\n", logFilename);
    File file = SD.open(logFilename, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(*convertedMessage))
    {
        Serial.println("Message appended");
    }
    else
    {
        Serial.println("Append failed");
    }
    file.close();
}
String sdCard::getNewLogFilename()
{
    int fileIndex = 1;
    String filename;
    do
    {
        filename = "/datalog" + String(fileIndex) + ".csv";
        fileIndex++;
    } while (SD.exists(filename.c_str()));
    return filename;
}
