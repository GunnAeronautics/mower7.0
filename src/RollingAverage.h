class RollingAverage{

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