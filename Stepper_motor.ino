#define dirPin1 5
#define stepPin1 6
#define dirPin2 9
#define stepPin2 8
#define stepsPerRevolution 200

void setup() {
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  //pinMode(stepPin2, OUTPUT);
  //pinMode(dirPin2, OUTPUT);
}
void loop() {
    digitalWrite(dirPin1, HIGH);
    for (int i = 0; i <5*stepsPerRevolution; i++) {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(500);
  }
  /*digitalWrite(dirPin2, HIGH);
  for (int i = 0; i <5*stepsPerRevolution; i++) {
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(500);
  } */
}
