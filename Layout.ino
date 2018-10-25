#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void loop() {
  display.drawRect(7, 0, 32, 32, WHITE);
  display.fillRect(7, 0, 32, 9, WHITE);
  display.fillCircle(39, 18, 3, WHITE);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(8, 1);
  display.print("Meter");
  display.setTextColor(WHITE);
  display.setCursor(11, 16);
  display.print("2.00");
  
  display.fillCircle(45, 18, 3, WHITE);
  display.drawRect(46, 0, 32, 32, WHITE);
  display.fillRect(46, 0, 32, 9, WHITE);
  display.fillCircle(78, 18, 3, WHITE);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(51, 1);
  display.print("Hoek");
  display.setTextColor(WHITE);
  display.setCursor(50, 16);
  display.print("2.00");
  
  display.fillCircle(84, 18, 3, WHITE);
  display.drawRect(85, 0, 38, 32, WHITE);
  display.fillRect(85, 0, 38, 9, WHITE);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(86, 1);
  display.print("Hoogte");
  display.setTextColor(WHITE);
  display.setCursor(92, 16);
  display.print("2.00");
  
  display.display();
}
