class RollingAverage{

  public:
  int rollingLen;
  RollingAverage();
  void newData(float data);
  float getData();
  
  private: 
  float *rawData;
  int dataIndex;
  int startFlag;
  float average;
};