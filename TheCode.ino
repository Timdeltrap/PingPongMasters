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
float desiredSpeed;;                  
float rpm_speed[2];                           

float topPart;
float bottomPart;
float xCoordinaat = 2 - 0.2;
float yCoordinaat = -0.24;
float alpha = 0.0;
float factor = 1.113;
float velocity;
float omtrek;

int potCount = 2;                 
int potPin[2] = {A0,A1};    
int sensorValue[2];        
float EMA_a = 0.2;                
int EMA_S[2];             
int previousEMA_S[2];

void setup() {
 Serial.begin(9600);
 
 display.begin(SSD1306_SWITCHCAPVCC,0x3C);
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(WHITE);
 display.setCursor(0,0);
 
 attachInterrupt(digitalPinToInterrupt(interruptPin1),rpm1,RISING);
 attachInterrupt(digitalPinToInterrupt(interruptPin2),rpm2,RISING);
 analogWrite(motorPin[0], 100);                       
 analogWrite(motorPin[1], 100);

 topPart = pow(xCoordinaat,2) * 9.81;
 bottomPart = (xCoordinaat * sin(2.0 * alpha)) - (2.0 * yCoordinaat * pow(cos(alpha),2));
 velocity = factor * (sqrt(topPart/bottomPart));
 omtrek = 0.047 * PI;
 desiredSpeed = (velocity/omtrek) * 60.0;
 
 delay(2000);                              
}

void loop() {
 Serial.print("Desired RPM: ");
 Serial.println(desiredSpeed);
 
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
