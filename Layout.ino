#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

int state = 0;      
int buttonState;           
int previousState = LOW;    
long time = 0;         
long debounce = 200;   

double afstandTarget = 0.0;
double hoogteTarget = 0.0;
double hoek = 0.0;

float meter;
float height;
float angle;
float value1;
float value2;
float value3;

void setup() {
  Serial.begin(9600);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void loop() {
  instelMenu();

  Serial.print("Meter: ");
  Serial.print(afstandTarget);
  Serial.print(" Height: ");
  Serial.print(hoogteTarget);
  Serial.print(" Hoek: ");
  Serial.println(hoek);
  
}


void instelMenu(){
  getButton();

   if(state == 0)
   {
        display.clearDisplay();

        display.drawRect(7, 0, 120, 32, WHITE);
        display.fillRect(7, 0, 16, 32, WHITE);
        display.setTextSize(3);
        display.setTextColor(BLACK);
        display.setCursor(7, 6);
        display.print(">");
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(26, 8);
        display.println("Press the button");
        display.setCursor(26, 16);
        display.println("to adjust values");    

        display.display();
   }
   else if(state == 1)
   {
        display.clearDisplay();

        value1 = analogRead(0);
        meter = (map(value1, 0, 1023, 0, 30)) / 10.0;
        display.drawRect(7, 0, 32, 32, WHITE);
        display.fillRect(7, 0, 32, 9, WHITE);
        display.fillCircle(39, 18, 3, WHITE);
        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.setCursor(8, 1);
        display.print("Meter");
        display.setTextColor(WHITE);
        display.setCursor(11, 16);
        display.print(meter);
      
        height = hoogteTarget;
        display.fillCircle(45, 18, 3, WHITE);
        display.drawRect(46, 0, 38, 32, WHITE);
        display.fillRect(46, 0, 38, 9, WHITE);
        display.fillCircle(84, 18, 3, WHITE);
        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.setCursor(47, 1);
        display.print("Height");
        display.setTextColor(WHITE);
        display.setCursor(52, 16);
        display.print(height);
      
        angle = hoek;
        display.fillCircle(90, 18, 3, WHITE);
        display.drawRect(91, 0, 36, 32, WHITE);
        display.fillRect(91, 0, 36, 9, WHITE);
        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.setCursor(92, 1);
        display.print("Angle");
        display.setTextColor(WHITE);
        display.setCursor(96, 16);
        display.print(angle);
        
        display.display();
   }
   else if(state == 2)
   {
        display.clearDisplay();
        
        meter = afstandTarget;
        display.drawRect(7, 0, 32, 32, WHITE);
        display.fillRect(7, 0, 32, 9, WHITE);
        display.fillCircle(39, 18, 3, WHITE);
        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.setCursor(8, 1);
        display.print("Meter");
        display.setTextColor(WHITE);
        display.setCursor(11, 16);
        display.print(meter);

        value2 = analogRead(0);
        height = (map(value2, 0, 1023, 0, 10)) / 10.0;
        display.fillCircle(45, 18, 3, WHITE);
        display.drawRect(46, 0, 38, 32, WHITE);
        display.fillRect(46, 0, 38, 9, WHITE);
        display.fillCircle(84, 18, 3, WHITE);
        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.setCursor(47, 1);
        display.print("Height");
        display.setTextColor(WHITE);
        display.setCursor(52, 16);
        display.print(height);
      
        angle = hoek;
        display.fillCircle(90, 18, 3, WHITE);
        display.drawRect(91, 0, 36, 32, WHITE);
        display.fillRect(91, 0, 36, 9, WHITE);
        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.setCursor(92, 1);
        display.print("Angle");
        display.setTextColor(WHITE);
        display.setCursor(96, 16);
        display.print(angle);
        
        display.display();
   }
   else if(state == 3)
   {
        display.clearDisplay();
        
        meter = afstandTarget;
        display.drawRect(7, 0, 32, 32, WHITE);
        display.fillRect(7, 0, 32, 9, WHITE);
        display.fillCircle(39, 18, 3, WHITE);
        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.setCursor(8, 1);
        display.print("Meter");
        display.setTextColor(WHITE);
        display.setCursor(11, 16);
        display.print(meter);
      
        height = hoogteTarget;
        display.fillCircle(45, 18, 3, WHITE);
        display.drawRect(46, 0, 38, 32, WHITE);
        display.fillRect(46, 0, 38, 9, WHITE);
        display.fillCircle(84, 18, 3, WHITE);
        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.setCursor(47, 1);
        display.print("Height");
        display.setTextColor(WHITE);
        display.setCursor(52, 16);
        display.print(height);

        value3 = analogRead(0);
        angle = map(value3, 0, 1023, 0, 25);
        display.fillCircle(90, 18, 3, WHITE);
        display.drawRect(91, 0, 36, 32, WHITE);
        display.fillRect(91, 0, 36, 9, WHITE);
        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.setCursor(92, 1);
        display.print("Angle");
        display.setTextColor(WHITE);
        display.setCursor(96, 16);
        display.print(angle);;
        
        display.display();
   }
   else if(state == 4)
   {
        display.clearDisplay();

        display.drawRect(7, 0, 120, 32, WHITE);
        display.fillRect(7, 0, 16, 32, WHITE);
        display.setTextSize(3);
        display.setTextColor(BLACK);
        display.setCursor(7, 6);
        display.print(">");
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(26, 8);
        display.println("The changes are");
        display.setCursor(26, 16);
        display.println("submitted!");    

        display.display();
   }
    
    afstandTarget = meter;
    hoogteTarget = height;
    hoek = angle;
}

int getButton() {
  buttonState = digitalRead(7);
  if (buttonState == HIGH && previousState == LOW && millis() - time > debounce) 
  {
    if (state == 0)
    {
      state = 1;
    }
    else if (state == 1) 
    {
      state = 2;
    }
    else if (state == 2)
    {
      state = 3;
    }
    else if (state == 3)
    {
      state = 4;
    }
    else if (state == 4)
    {
      state = 0;
    }
  }
  previousState = buttonState;
  return state;
}
