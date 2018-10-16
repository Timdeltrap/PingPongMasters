int potPin[3] = {A0,A1,A3};
int sensorValue[3];
float EMA_a = 0.2;
int EMA_S[3];
int previousEMA_S[3];
int potValue1;
int potValue2;
int potValue3;


void setup() {
  for(int i; i<3; i++)
  {
    pinMode(potPin[i], INPUT);
    analogReadResolution(10);
    EMA_S[i] = analogRead(potPin[i]);
  }
}

void loop(){
  getPots();
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
