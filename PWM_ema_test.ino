int sensorPin = 0;
int sensorValue;
float EMA_a = 1;
int EMA_S;
int previousEMA_S;
int motorSpeed;

void setup() {
  pinMode(A9, OUTPUT);
  EMA_S = analogRead(0);
}

void loop() {
  for(int i; i < 3; i++){
    sensorValue = analogRead(sensorPin);
    EMA_S = (EMA_a*sensorValue) + ((1-EMA_a)*EMA_S);
    if(EMA_S != previousEMA_S){
      //SerialUSB.println(EMA_S);
      previousEMA_S = EMA_S;
    }
  }
  motorSpeed = map(sensorValue, 0, 1023, 50, 200);
  SerialUSB.println(motorSpeed);
  analogWrite(A9, motorSpeed);
}
