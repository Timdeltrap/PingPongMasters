#ifndef GuiNodes_h
#define GuiNodes_h

#include "Arduino.h"

class Nodes
{
  public:
    Nodes(int nodeNum, int numOfInputs, char title[]);
    void connectNodes(int nodeIn,  int inPoint);
    void disconnectNodes(int nodeIn, int inPoint);
    void appendNode();
    void deleteNode();
    int mapValue(int minIn, int maxIn, int minOut, int maxOut);
    int getAnalog(int analogPin);
    void numericReadout();
  private:
    int _nodeNum;
    int _nodeInput;
    int _numOfInputs;
    int _pointNum;
    char _title[10];
};
#endif
