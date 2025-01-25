#ifndef SD_CARD_H
#define SD_CARD_H

#include <Arduino.h>
#include <SPI.h>

class sdCard
{
public:
    sdCard(char sdCSPin_IN, char sdSCKPin_IN, char sdMISOPin_IN, char sdMOSIPin_IN);
    void setupSD(SPIClass &sdSPI_IN);
    void writeCSVLine(String message);
    uint8_t cardType;
    uint64_t cardSize;
    String logFilename;
    bool cardMounted;

private:
    String getNewLogFilename();
    char sdCSPin;
    char sdSCKPin;
    char sdMISOPin;
    char sdMOSIPin;
    SPIClass sdSPI;
};

#endif
