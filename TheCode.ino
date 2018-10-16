unsigned long time1;
unsigned long time2;
unsigned long timeIntegral1 = 0;
unsigned long timeIntegral2 = 0;
unsigned long timeDerivative1 = 0;
unsigned long timeDerivative2 = 0;
unsigned int ledPin = 7;                    //LED pin
unsigned int motorPin = 6;                  //motor pin
unsigned int interruptPin = 2;              //interrupt pin
float error = 0;                            //control parameters
float previous_error;
float pidIntegral = 0;
float pidDerivative = 0;
float power = 0;                            //control parameters
float POWER;
float timeInterval = 0;                     //control parameters
float kp;                                   //P control
float ki;
float kd;
float desiredSpeed = 3000;                  //P control
float rpm_speed;                            //this is the speed
int kpPin = 0;
int kiPin = 2;
int kdPin = 4;  

int sensorPin = 0;
int sensorValue;
float EMA_a = 1;
int EMA_S;
int previousEMA_S;
int motorSpeed;

void setup() {
 Serial.begin(9600);
 pinMode(ledPin, OUTPUT);
 attachInterrupt(digitalPinToInterrupt(interruptPin),rpm,RISING);
 analogWrite(6, 100);                       //here we start the motor, 150 is a guess, you can put other numbers
 delay(2000);                               //delay 2 seconds for motor to get some speed
}

void loop() {
 digitalWrite(ledPin, HIGH);
 
 timeInterval = (time2-time1)/1000000.0;
 rpm_speed = 60.0/timeInterval/2.0;
 
 if (rpm_speed > 10000)                     // in the case the reading is not valid, sometimes noise may influence the quality signals, we select 10000 as it is an impossible speed
 {
    delay(200);
    return;                                 //start again
 };

 previous_error = error;
 error = desiredSpeed - rpm_speed;          //error should be desired speed (desiredSpeed ) minus the actual speed

 timeIntegral1 = timeIntegral2;
 timeIntegral2 = micros();

 pidIntegral = pidIntegral + (timeIntegral2 - timeIntegral1) * (error + previous_error) / 2.0 / 1000000;

 timeDerivative1 = timeDerivative2;
 timeDerivative2 = micros();
 
 pidDerivative = 1000000 * (error - previous_error) / (timeDerivative2 - timeDerivative1);

 kp = analogRead(kpPin) / 1023;
 ki = analogRead(kiPin) / 1023;
 kd = analogRead(kdPin) / 1023;

 power = (kp * error) + (ki * pidIntegral) + (kd * pidDerivative);                        //P control: the output of the controller (power) should be Kp*error
 POWER = constrain(70 + power, 10, 150);    //here we tried, 70 is nearly the minimal power to start motor, constrain the power within 50 (lower boundary) and 255 (maximal power), you are free to adjust these 3 numbers, check constrain code in arduino website

 analogWrite(motorPin, POWER);              //send the power to the motor
 
 Serial.print(rpm_speed);                   //output status to serial monitor
 Serial.print(" ");
 Serial.println(POWER);
 
 delay(200);                                //this delay give motor sometime to react to the power value, which you just sent to the motor
}                                           //Delay op 20 zetten kan helpen. Delay van 200 is in sommige gevallen iets te lang.

void rpm()
{
 time1 = time2;
 time2 = micros();
}

void getPots() {
  for(int i; i < 3; i++){
    sensorValue[i] = analogRead(potPin[i]);
    EMA_S[i] = (EMA_a*sensorValue[i]) + ((1-EMA_a)*EMA_S[i]);
    if(EMA_S[i] != previousEMA_S[i]){
      potValue1 = EMA_S[0];
      potValue2 = EMA_S[1];
      potValue3 = EMA_S[2];
      previousEMA_S[i] = EMA_S[i];
    }
  }
}
