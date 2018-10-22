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
unsigned int motorPin[2] = {5,6};                 //motor pin
unsigned int interruptPin1 = 2;              //interrupt pin
unsigned int interruptPin2 = 3;
float error[2];                            //control parameters
float previous_error[2];
float pidIntegral[2];
float pidDerivative[2];
float power[2];                            //control parameters
float POWER[2];
float timeInterval[2];                     //control parameters
float kp;                                   //P control
float ki;
float kd;
float desiredSpeed;;                  //P control
float rpm_speed[2];                            //this is the speed

float topPart;
float bottomPart;
float xCoordinaat = 2 - 0.2;
float yCoordinaat = -0.24;
float alpha = 0.0;
float velocity;
float omtrek;

int potCount = 2;                 // number of potentiometers
int potPin[potCount] = {A0,A1};   // pins on which the potentiometers are connected 
int sensorValue[potCount];        // array of measured pot values
float EMA_a = 0.2;                // Constant in EMA formula
int EMA_S[potCount];              // Array of Pot values after EMA
int previousEMA_S[potCount];

void setup() {
 Serial.begin(9600);
 
 display.begin(SSD1306_SWITCHCAPVCC,0x3C);
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(WHITE);
 display.setCursor(0,0);
 
 attachInterrupt(digitalPinToInterrupt(interruptPin1),rpm1,RISING);
 attachInterrupt(digitalPinToInterrupt(interruptPin2),rpm2,RISING);
 analogWrite(motorPin[0], 100);                       //here we start the motor, 150 is a guess, you can put other numbers
 analogWrite(motorPin[1], 100);

 topPart = pow(xCoordinaat,2) * 9.81;
 bottomPart = (xCoordinaat * sin(2.0 * alpha)) - (2.0 * yCoordinaat * pow(cos(alpha),2));
 velocity = sqrt(topPart/bottomPart);
 omtrek = 0.047 * PI;
 desiredSpeed = (velocity/omtrek) * 60.0;
 
 delay(2000);                               //delay 2 seconds for motor to get some speed
}

void loop() {
 Serial.print("Desired RPM: ");
 Serial.println(desiredSpeed);
 
 for(int i=0; i<2; i++){
    timeInterval[i] = (time2[i]-time1[i])/1000000.0;
    rpm_speed[i] = 60.0/timeInterval[i];
    
    if (rpm_speed[i] > 10000)                     // in the case the reading is not valid, sometimes noise may influence the quality signals, we select 10000 as it is an impossible speed
    {
       delay(200);
       return;                                 //start again
    }

    previous_error[i] = error[i];
    error[i] = desiredSpeed - rpm_speed[i];          //error should be desired speed (desiredSpeed ) minus the actual speed

    timeIntegral1[i] = timeIntegral2[i];
    timeIntegral2[i] = micros();

    pidIntegral[i] = pidIntegral[i] + (timeIntegral2[i] - timeIntegral1[i]) * (error[i] + previous_error[i]) / 2.0 / 1000000;

    timeDerivative1[i] = timeDerivative2[i];
    timeDerivative2[i] = micros();
   
    pidDerivative[i] = 1000000 * (error[i] - previous_error[i]) / (timeDerivative2[i] - timeDerivative1[i]);
    
    kp = 0.0;
    ki = 0.0;
    kd = 0.04;

    power[i] = (kp * error[i]) + (ki * pidIntegral[i]) + (kd * pidDerivative[i]);                        //P control: the output of the controller (power) should be Kp*error
    POWER[i] = constrain(70 + power[i], 10, 150);    //here we tried, 70 is nearly the minimal power to start motor, constrain the power within 50 (lower boundary) and 255 (maximal power), you are free to adjust these 3 numbers, check constrain code in arduino website

    analogWrite(motorPin[i], POWER[i]);              //send the power to the motor
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

int getPot(int potNum) {
    sensorValue[potNum] = analogRead(potPin[potNum]);
    EMA_S[potNum] = (EMA_a*sensorValue[potNum]) + ((1-EMA_a)*EMA_S[potNum]);
    if(EMA_S[potNum] != previousEMA_S[potNum]){
      previousEMA_S[potNum] = EMA_S[potNum];
    }
    return EMA_S[potNum];
}
