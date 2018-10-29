#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include <math.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

unsigned long time1[2];
unsigned long time2[2];
unsigned long timeIntegral1[2];
unsigned long timeIntegral2[2];
unsigned long timeDerivative1[2];
unsigned long timeDerivative2[2];
unsigned int motorPin[2] = {5,6};                
unsigned int interruptPin1 = 2;             
unsigned int interruptPin2 = 3;
float error[2];                           
float previous_error[2];
float pidIntegral[2];
float pidDerivative[2];
float power[2];                            
float POWER[2];
float timeInterval[2];                     
float kp;                                  
float ki;
float kd;
float desiredSpeed;                  
float rpm_speed[2];                           

//constants to be changed
double afstandTarget = 2.0;
double hoogteTarget = 0.0;
double hoek = 5.0;

double topPart;
double bottomPart;
double sx0 = 0.2;
double sy0 = 0.24;
double xCoordinaat;
double yCoordinaat;
double alpha;
double beta;
double factor;
double velocity;
double omtrek;
float meter;
float height;
float angle;
float value1;
float value2;
float value3;

int potCount = 1;                 
int potPin[1] = {A0};    
int sensorValue[1];        
float EMA_a = 0.2;                
int EMA_S[1];             
int previousEMA_S[1];

int state = 0;      
int buttonState;           
int previousState = LOW;    
long time = 0;         
long debounce = 200;   

void setup() {
   Serial.begin(9600);
   pinMode(4, INPUT);
   display.begin(SSD1306_SWITCHCAPVCC,0x3C);
   display.clearDisplay();
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(0,0);
  
   analogWrite(motorPin[0], 100);                       
   analogWrite(motorPin[1], 100);
   
   attachInterrupt(digitalPinToInterrupt(interruptPin1),rpm1,RISING);
   attachInterrupt(digitalPinToInterrupt(interruptPin2),rpm2,RISING);
   
   Serial.print("xcoordinaat: ");
   Serial.println(xCoordinaat);
   Serial.print("ycoordinaat: ");
   Serial.println(yCoordinaat);
   Serial.print("Desired Speed: ");
   Serial.println(desiredSpeed);
   delay(2000);     
                            
}

void loop() {
   instelMenu();

   Serial.print("Meter: ");
   Serial.print(afstandTarget);
   Serial.print(" Height: ");
   Serial.print(hoogteTarget);
   Serial.print(" Hoek: ");
   Serial.println(hoek);
   
   balSchieten();
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

        value1 = getPot(0);
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

        value2 = getPot(0);
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

        value3 = getPot(0);
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
      afstandTarget = meter;
      hoogteTarget = height;
      hoek = angle;
      calculateVelocity(afstandTarget, hoogteTarget, hoek);
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
   
}

void rpm1()
{
 time1[0]= time2[0];
 time2[0] = micros();
}

void rpm2()
{
 time1[1] = time2[1];
 time2[1] = micros();
}

int getPot(int potNum) 
{
    sensorValue[potNum] = analogRead(potPin[potNum]);
    EMA_S[potNum] = (EMA_a*sensorValue[potNum]) + ((1-EMA_a)*EMA_S[potNum]);
    if(EMA_S[potNum] != previousEMA_S[potNum])
    {
      previousEMA_S[potNum] = EMA_S[potNum];
    }
    return EMA_S[potNum];
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

void balSchieten(){
   for(int i=0; i<2; i++){
            timeInterval[i] = (time2[i]-time1[i])/1000000.0;
            rpm_speed[i] = 60.0/timeInterval[i];
            
            if (rpm_speed[i] > 10000)                     
            {
               delay(200);
               return;                                
            }
        
            previous_error[i] = error[i];
            error[i] = desiredSpeed - rpm_speed[i];         
        
            timeIntegral1[i] = timeIntegral2[i];
            timeIntegral2[i] = micros();
        
            pidIntegral[i] = pidIntegral[i] + (timeIntegral2[i] - timeIntegral1[i]) * (error[i] + previous_error[i]) / 2.0 / 1000000;
        
            timeDerivative1[i] = timeDerivative2[i];
            timeDerivative2[i] = micros();
           
            pidDerivative[i] = 1000000 * (error[i] - previous_error[i]) / (timeDerivative2[i] - timeDerivative1[i]);
            
            kp = 0.0;
            ki = 0.0;
            kd = 0.04;
        
            power[i] = (kp * error[i]) + (ki * pidIntegral[i]) + (kd * pidDerivative[i]);                       
            POWER[i] = constrain(70 + power[i], 10, 200);    
        
            analogWrite(motorPin[i], POWER[i]);             
      }
      Serial.print(rpm_speed[0]);
      Serial.print("---");
      Serial.println(rpm_speed[1]);
}

void calculateVelocity(float hoek, float meter, float height){
   if (hoek == 0.00)
   {
     factor =  1.146;
   }
   else if (hoek == 5.00)
   {
     factor = 1.12;
   }
   else if (hoek == 10.00)
   {
     factor = 1.08;
   }
   else if (hoek == 15.00)
   {
     factor = 1.15;
   }
   else
   {
     factor = 1.11;
   }
   
   alpha = ((hoek)*PI) / 180.0;
   beta = ((3.3+hoek)*PI) / 180.0;
   xCoordinaat = afstandTarget - ((cos(alpha) * sx0) - (sin(alpha)*sy0));
   yCoordinaat = -((sin(alpha) * sx0) + (cos(alpha)*sy0)) + hoogteTarget; 
   topPart = (xCoordinaat * xCoordinaat) * 9.81;
   bottomPart = (xCoordinaat * sin(2.0 * beta)) - (2.0 * yCoordinaat * (cos(beta)*cos(beta)));
   velocity = factor * (sqrt(topPart/bottomPart));
   omtrek = 0.047 * PI;
   desiredSpeed = (velocity/omtrek) * 60.0;
}
