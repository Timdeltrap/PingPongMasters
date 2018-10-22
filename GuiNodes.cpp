#include "Arduino.h"
#include "GuiNodes.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

Adafruit_SSD1306 displayNode;

int xNode[10];
int yNode[10];
int xPointIn[10];
int yPointIn[10][3];
int xPointOut[10];
int yPointOut[10];
int _nodeInState[10];
int _nodeOutState[10];
int _nodeNum;
char _title[10];
int outputValue[10];
int inputValue[10][3];
int numericValue[10];
int oldNumericValue[10];

Nodes::Nodes(int nodeNum, int numOfInputs, char title[])
{
  strcpy(_title, title);
  _nodeNum = nodeNum;
  _numOfInputs = numOfInputs;
  xNode[_nodeNum] = (_nodeNum * 32) + (7 * (_nodeNum + 1));
  yNode[_nodeNum] = _nodeNum * 0;
  xPointIn[_nodeNum] = xNode[_nodeNum];
  for(int i = 0; i < _numOfInputs; i++){
    yPointIn[_nodeNum][i] = 18 - (((_numOfInputs - 1) * 4) - (i * 8));
  }
  xPointOut[_nodeNum] = xNode[_nodeNum] + 31;
  yPointOut[_nodeNum] = 18;
}

void nodeIn(int _nodeNum, int pointNum, int nodeInState)
{
  _nodeInState[_nodeNum] = nodeInState;
  if(_nodeInState[_nodeNum] == 0){
    displayNode.fillCircle(xPointIn[_nodeNum],yPointIn[_nodeNum][pointNum],3,BLACK);
    displayNode.drawCircle(xPointIn[_nodeNum],yPointIn[_nodeNum][pointNum],3,WHITE);
  }
  else if(_nodeInState[_nodeNum] == 1){
    displayNode.fillCircle(xPointIn[_nodeNum],yPointIn[_nodeNum][pointNum],3,WHITE);
  }
}

void nodeOut(int nodeOutState)
{
  _nodeOutState[_nodeNum] = nodeOutState;
  if(_nodeOutState[_nodeNum] == 0){
    displayNode.fillCircle(xPointOut[_nodeNum],yPointOut[_nodeNum],3,BLACK);
    displayNode.drawCircle(xPointOut[_nodeNum],yPointOut[_nodeNum],3,WHITE);
  }
  else if(_nodeOutState[_nodeNum] == 1){
    displayNode.fillCircle(xPointOut[_nodeNum],yPointOut[_nodeNum],3,WHITE);
  }
}

void Nodes::connectNodes(int _nodeInput, int _pointNum){
  displayNode.drawLine(xPointOut[_nodeNum],yPointOut[_nodeNum],xPointIn[_nodeInput], yPointIn[_nodeInput][_pointNum], WHITE);
  nodeOut(1);
  nodeIn(_nodeInput, _pointNum, 1);
  inputValue[_nodeInput][_pointNum] = outputValue[_nodeNum];
}

void Nodes::disconnectNodes(int _nodeInput, int _pointNum){
  displayNode.drawLine(xPointOut[_nodeNum],yPointOut[_nodeNum],xPointIn[_nodeInput], yPointIn[_nodeInput][_pointNum], BLACK);
  nodeOut(0);
  nodeIn(_nodeInput, _pointNum, 0);
  inputValue[_nodeInput][_pointNum] = 0;
}

void Nodes::appendNode(){
  displayNode.drawRect(xNode[_nodeNum], yNode[_nodeNum], 32, 32, WHITE);
  displayNode.fillRect(xNode[_nodeNum], yNode[_nodeNum], 32, 9, WHITE);
  displayNode.fillCircle(xPointOut[_nodeNum],yPointOut[_nodeNum],3,BLACK);
  displayNode.drawCircle(xPointOut[_nodeNum],yPointOut[_nodeNum],3,WHITE);
  for(int i = 0; i < _numOfInputs; i++){
    displayNode.fillCircle(xPointIn[_nodeNum],yPointIn[_nodeNum][i],3,BLACK);
    displayNode.drawCircle(xPointIn[_nodeNum],yPointIn[_nodeNum][i],3,WHITE);
  }
  displayNode.setCursor(xNode[_nodeNum]+1,yNode[_nodeNum]+1);
  displayNode.setTextSize(1);
  displayNode.setTextColor(BLACK);
  displayNode.print(_title);
}

void Nodes::deleteNode(){
  displayNode.drawRect(xNode[_nodeNum], yNode[_nodeNum], 32, 32,BLACK);
  displayNode.fillRect(xNode[_nodeNum], yNode[_nodeNum], 32, 9, BLACK);
  displayNode.fillCircle(xPointOut[_nodeNum],yPointOut[_nodeNum],3,BLACK);
  for(int i = 0; i < _numOfInputs; i++){
    displayNode.fillCircle(xPointIn[_nodeNum],yPointIn[_nodeNum][i],3,BLACK);
  }
}

int Nodes::getAnalog(int inputPin){
  displayNode.setCursor(xNode[_nodeNum]+7,yNode[_nodeNum]+16);
  displayNode.setTextColor(WHITE);
  displayNode.print("A");
  displayNode.print(inputPin);
  outputValue[_nodeNum] = analogRead(inputPin);
  return outputValue[_nodeNum];
}

int Nodes::mapValue(int minIn, int maxIn, int minOut, int maxOut){
  outputValue[_nodeNum] = map(inputValue[_nodeNum][_pointNum], minIn, maxIn, minOut, maxOut);
  return outputValue[_nodeNum];
}

void Nodes::numericReadout(){
  numericValue[_nodeNum] = inputValue[_nodeNum][_pointNum];
  if(numericValue[_nodeNum] != oldNumericValue[_nodeNum]){
    displayNode.setCursor(xNode[_nodeNum]+5,yNode[_nodeNum]+16);
    displayNode.setTextColor(BLACK);
    displayNode.print(oldNumericValue[_nodeNum]);
    displayNode.setCursor(xNode[_nodeNum]+5,yNode[_nodeNum]+16);
    displayNode.setTextColor(WHITE);
    displayNode.print(numericValue[_nodeNum]);
    oldNumericValue[_nodeNum] = numericValue[_nodeNum];
  }
  outputValue[_nodeNum] = numericValue[_nodeNum];
}