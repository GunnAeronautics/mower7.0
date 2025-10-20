#ifndef ROLLING_AVERAGE_H
#define ROLLING_AVERAGE_H

#include "../config/config.h"

class RollingAverage {
  public:
    RollingAverage(int rollLen);
    void newData(float data);
    float getData();
  
  private: 
    int rollingLen;
    float *rawData;
    int dataIndex;
    int startFlag;
    float average;
};

#endif // ROLLING_AVERAGE_H
