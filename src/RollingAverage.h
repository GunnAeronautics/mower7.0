class RollingAverage{

  public:
  int rollingLen;
  RollingAverage(int rollLen);
  void newData(float data);
  float getData();
  
  private: 
  float *rawData;
  int dataIndex;
  int startFlag;
  float average;
};