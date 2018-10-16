unsigned long time1[2];
unsigned long time2[2];
unsigned long timeIntegral1 = 0;
unsigned long timeIntegral2 = 0;
unsigned long timeDerivative1 = 0;
unsigned long timeDerivative2 = 0;
unsigned int motorPin1 = 6;
unsigned int motorPin2 = 5;                  //motor pin
unsigned int interruptPin1 = 2;              //interrupt pin
unsigned int interruptPin2 = 3;
float error = 0;                            //control parameters
float previous_error;
float pidIntegral = 0;
float pidDerivative = 0;
float power = 0;                            //control parameters
float POWER;
float timeInterval[2];                     //control parameters
float kp;                                   //P control
float ki;
float kd;
float desiredSpeed = 3000;                  //P control
float rpm_speed[2];                            //this is the speed

int potPin[3] = {A0,A1,A2};
int sensorValue[3];
const float EMA_a = 0.2;
int EMA_S[3];
int previousEMA_S[3];

void setup() {
 Serial.begin(9600);
 attachInterrupt(digitalPinToInterrupt(interruptPin1),rpm1,RISING);
 attachInterrupt(digitalPinToInterrupt(interruptPin2),rpm2,RISING);
 analogWrite(motorPin1, 100);                       //here we start the motor, 150 is a guess, you can put other numbers
 analogWrite(motorPin2, 100);
 delay(2000);                               //delay 2 seconds for motor to get some speed
}

void loop() {
 getPots();
 for(int i; i<2; i++){
  timeInterval[i] = (time2[i]-time1[i])/1000000.0;
  rpm_speed[i] = 60.0/timeInterval[i]/2.0;
  if (rpm_speed[i] > 10000)                     // in the case the reading is not valid, sometimes noise may influence the quality signals, we select 10000 as it is an impossible speed
  {
    delay(200);
    return;                                 //start again
 }
 }
 
 

 previous_error = error;
 error = desiredSpeed - rpm_speed;          //error should be desired speed (desiredSpeed ) minus the actual speed

 timeIntegral1 = timeIntegral2;
 timeIntegral2 = micros();

 pidIntegral = pidIntegral + (timeIntegral2 - timeIntegral1) * (error + previous_error) / 2.0 / 1000000;

 timeDerivative1 = timeDerivative2;
 timeDerivative2 = micros();
 
 pidDerivative = 1000000 * (error - previous_error) / (timeDerivative2 - timeDerivative1);

 kp = potValue1 / 1023;
 ki = potValue2 / 1023;
 kd = potValue3 / 1023;

 power = (kp * error) + (ki * pidIntegral) + (kd * pidDerivative);                        //P control: the output of the controller (power) should be Kp*error
 POWER = constrain(70 + power, 10, 150);    //here we tried, 70 is nearly the minimal power to start motor, constrain the power within 50 (lower boundary) and 255 (maximal power), you are free to adjust these 3 numbers, check constrain code in arduino website

 analogWrite(motorPin1, POWER);              //send the power to the motor
 
 Serial.print(rpm_speed);                   //output status to serial monitor
 Serial.print(" ");
 Serial.println(POWER);
 
 delay(200);                                //this delay give motor sometime to react to the power value, which you just sent to the motor
}                                           //Delay op 20 zetten kan helpen. Delay van 200 is in sommige gevallen iets te lang.

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
