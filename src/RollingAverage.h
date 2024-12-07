class RollingAverage{

  public:

  int rollingLen;
  float *rawData;//I can't get the size of this array to be parametric
  //for some reason it always intializes as an array of inf and changing values in the constructor
  //doesn't seem to work, this seems like the best option as nothing else FUCKING WORKS GAH!
  //ok what the actual fuck initializing the array as {0,0,0,0...} doesnt fucking work either
  int dataIndex;
  int startFlag;
  float average;
  RollingAverage();
  void newData(float data);
  float getData();
};