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
float desiredSpeed = 3000;                  //P control
float rpm_speed[2];                            //this is the speed

int potPin[3] = {A0,A1,A2};
int sensorValue[3];
const float EMA_a = 0.2;
int EMA_S[3];
int previousEMA_S[3];
int potValue1;
int potValue2;
int potValue3;

void setup() {
 Serial.begin(9600);
 attachInterrupt(digitalPinToInterrupt(interruptPin1),rpm1,RISING);
 attachInterrupt(digitalPinToInterrupt(interruptPin2),rpm2,RISING);
 analogWrite(motorPin[0], 100);                       //here we start the motor, 150 is a guess, you can put other numbers
 analogWrite(motorPin[1], 100);
 delay(2000);                               //delay 2 seconds for motor to get some speed
}

void loop() {
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
    getPots();
    kp = analogRead(A0) / 1023.0;
    ki = analogRead(A1) / 1023.0;
    kd = analogRead(A2) / 1023.0;

    power[i] = (kp * error[i]) + (ki * pidIntegral[i]) + (kd * pidDerivative[i]);                        //P control: the output of the controller (power) should be Kp*error
    POWER[i] = constrain(70 + power[i], 10, 150);    //here we tried, 70 is nearly the minimal power to start motor, constrain the power within 50 (lower boundary) and 255 (maximal power), you are free to adjust these 3 numbers, check constrain code in arduino website

    analogWrite(motorPin[i], POWER[i]);              //send the power to the motor
//    Serial.print("kp: ");
//    Serial.println(kp);                   //output status to serial monitor
//    Serial.print("ki: ");
//    Serial.println(ki);                   //output status to serial monitor
//    Serial.print("kd: ");
    Serial.println(kd);                   //output status to serial monitor
//  Serial.println(rpm_speed[0]);
    //delay(200);  
    
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
