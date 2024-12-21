
#include <RollingAverage.h>



RollingAverage::RollingAverage(int rollLen){
    rollingLen = rollLen;//default
    dataIndex = 0;
    average = 0.0;
    rawData = new float[rollingLen]();
  }

void RollingAverage::newData(float data){

    average = average + data;
    average = average - rawData[dataIndex];
    rawData[dataIndex] = data;
    dataIndex ++;
    dataIndex %= rollingLen;
  }

float RollingAverage::getData(){

    return average / rollingLen;
  }
