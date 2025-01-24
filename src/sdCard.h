#ifndef SD_CARD_H
#define SD_CARD_H

#include <Arduino.h>
#include <SPI.h>

class sdCard
{
public:
    sdCard(char sdCSPin, SPIClass sdSPI);
    void writeCSVLine(String message);
    uint8_t cardType;
    uint64_t cardSize;
    String logFilename;
    bool cardMounted;

private:
    String getNewLogFilename();
    char sdCSPin;
};

#endif
