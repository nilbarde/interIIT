#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

long timer = 0;

int trigPin1=51;
int echoPin1=53;

int trigPin2= 4;
int echoPin2= 5;

int trigPin3= 8;
int echoPin3= 9;

int trigPin4= 10;
int echoPin4= 11;

int trigPin5= 6;
int echoPin5= 7;

long duration1;
int distance1;

long duration2;
int distance2;

long duration3;
int distance3;

long duration4;
int distance4;

long duration5;
int distance5;

int right_front_motor = 41;
int left_front_motor =43;
int right_back_motor =45;
int left_back_motor =47;

void setup() {
pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication

pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication

Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

pinMode(trigPin3, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin3, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication

pinMode(trigPin4, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin4, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication

pinMode(trigPin5, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin5, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication

  pinMode(right_front_motor,OUTPUT);
  pinMode(left_front_motor,OUTPUT);
  pinMode(right_back_motor ,OUTPUT);
  pinMode(left_back_motor,OUTPUT);
}
void loop() {

digitalWrite(trigPin1, LOW);
delayMicroseconds(2);

digitalWrite(trigPin1, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin1, LOW);

duration1= pulseIn(echoPin1, HIGH);

distance1= duration1*0.034/2;

Serial.print("Distance1: ");
Serial.println(distance1);
delay(1000);

digitalWrite(trigPin2, LOW);
delayMicroseconds(2);

digitalWrite(trigPin2, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin2, LOW);

duration2= pulseIn(echoPin2, HIGH);

distance2= duration2*0.034/2;

Serial.print("Distance2: ");
Serial.println(distance2);
delay(1000);

mpu6050.update();

  if(millis() - timer > 1000){
    
    Serial.println("=======================================================");
    Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("  accY : ");Serial.print(mpu6050.getAccY());
    Serial.print("  accZ : ");Serial.println(mpu6050.getAccZ());
  
    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("  gyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("  gyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
    Serial.print("  accAngleY : ");Serial.println(mpu6050.getAccAngleY());
  
    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    Serial.print("  gyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
    Serial.print("  gyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
    
    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    Serial.print("  angleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("  angleZ : ");Serial.println(mpu6050.getAngleZ());
    Serial.println("=======================================================");

    timer = millis();


digitalWrite(trigPin3, LOW);
delayMicroseconds(2);

digitalWrite(trigPin3, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin3, LOW);

duration3= pulseIn(echoPin3, HIGH);

distance3= duration3*0.034/2;

Serial.print("Distance3: ");
Serial.println(distance3);
delay(1000);

digitalWrite(trigPin4, LOW);
delayMicroseconds(2);

digitalWrite(trigPin4, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin4, LOW);

duration4= pulseIn(echoPin4, HIGH);

distance4= duration4*0.034/2;

Serial.print("Distance4: ");
Serial.println(distance4);

digitalWrite(trigPin5, LOW);
delayMicroseconds(2);

digitalWrite(trigPin5, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin5, LOW);

duration5= pulseIn(echoPin5, HIGH);

distance5= duration5*0.034/2;

Serial.print("Distance5: ");
Serial.println(distance5);

if(distance1>20){
    digitalWrite(right_front_motor,HIGH);
    digitalWrite(left_front_motor,HIGH);
    digitalWrite(right_back_motor ,LOW);
    digitalWrite(left_back_motor,LOW);
  }
if(distance1<20){
  digitalWrite(right_front_motor,HIGH);
  digitalWrite(left_front_motor,LOW);
  digitalWrite(right_back_motor ,LOW);
  digitalWrite(left_back_motor,HIGH);
  }
 }
} 
