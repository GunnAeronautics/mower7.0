
#include RollingAverage.h


RollingAverage::RollingAverage(){
    rollingLen = 50;//default
    dataIndex = 0;
    average = 0.0;
    rawData = new float[rollingLen]();
  }

RollingAverage::newData(float data){

    average = average + data;
    average = average - rawData[dataIndex];
    rawData[dataIndex] = data;
    dataIndex ++;
    dataIndex %= rollingLen;
  }

RollingAverage::getData(){

    return average / rollingLen;
  }
