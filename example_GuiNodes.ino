#include "GuiNodes.h"
#include "Adafruit_SSD1306.h"
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// new node by adding Nodes nodeX = Nodes(place, numOfINputs, name);
Nodes node0 = Nodes(0, 0, "Pin");
Nodes node1 = Nodes(1, 1, "Map");
Nodes node2 = Nodes(2, 1, "OUT2");

void setup() {
  //Here we init the display with I2C connection
  //0x3C is the address of the display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.clearDisplay();
  delay(100);
}

void loop() {
  node0.appendNode();
  node1.appendNode();
  node2.appendNode();
  node0.getAnalog(A0);
  node1.mapValue(0,1023,0,300);
  node0.connectNodes(1, 0);
  node1.connectNodes(2, 0);
  node2.numericReadout();
  display.display();
}
