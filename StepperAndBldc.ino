#include <Servo.h>

Servo ServoBLDC;
Servo ServoSeed;

#define BLDCpin 33
#define ServoPin 34

#define leftDRhigh 3
#define leftDRlow 4 // direction
#define leftPUhigh 5 
#define leftPUlow 6 // pulse
#define leftMFhigh 7 
#define leftMFlow 8 // enable

#define rightDRhigh 9
#define rightDRlow 10
#define rightPUhigh 11
#define rightPUlow 12
#define rightMFhigh 13
#define rightMFlow 14

#define frontDRhigh 15
#define frontDRlow 16 // direction
#define frontPUhigh 17 
#define frontPUlow 18 // pulse
#define frontMFhigh 19 
#define frontMFlow 20 // enable

#define backDRhigh 21
#define backDRlow 22
#define backPUhigh 23
#define backPUlow 24
#define backMFhigh 25
#define backMFlow 26

#define centralDRhigh 27
#define centralDRlow 28 // direction
#define centralPUhigh 29 
#define centralPUlow 30 // pulse
#define centralMFhigh 31 
#define centralMFlow 32 // enable

int ServoStep = 5;
int ServoAngle = 0;
int ServoActive = 0;

void setup()
{
	ServoBLDC.attach(BLDCpin);
	ServoSeed.attach(ServoPin);

	pinMode(leftDRhigh,OUTPUT);
	pinMode(leftDRlow,OUTPUT);
	pinMode(leftPUhigh,OUTPUT);
	pinMode(leftPUlow,OUTPUT);
	pinMode(leftMFhigh,OUTPUT);
	pinMode(leftMFlow,OUTPUT);

	pinMode(rightDRhigh,OUTPUT);
	pinMode(rightDRlow,OUTPUT);
	pinMode(rightPUhigh,OUTPUT);
	pinMode(rightPUlow,OUTPUT);
	pinMode(rightMFhigh,OUTPUT);
	pinMode(rightMFlow,OUTPUT);

	pinMode(frontDRhigh,OUTPUT);
	pinMode(frontDRlow,OUTPUT);
	pinMode(frontPUhigh,OUTPUT);
	pinMode(frontPUlow,OUTPUT);
	pinMode(frontMFhigh,OUTPUT);
	pinMode(frontMFlow,OUTPUT);

	pinMode(backDRhigh,OUTPUT);
	pinMode(backDRlow,OUTPUT);
	pinMode(backPUhigh,OUTPUT);
	pinMode(backPUlow,OUTPUT);
	pinMode(backMFhigh,OUTPUT);
	pinMode(backMFlow,OUTPUT);

	pinMode(centralDRhigh,OUTPUT);
	pinMode(centralDRlow,OUTPUT);
	pinMode(centralPUhigh,OUTPUT);
	pinMode(centralPUlow,OUTPUT);
	pinMode(centralMFhigh,OUTPUT);
	pinMode(centralMFlow,OUTPUT);


	digitalWrite(leftDRhigh,HIGH);
	digitalWrite(leftPUhigh,HIGH);
	digitalWrite(leftMFhigh,HIGH);

	digitalWrite(rightDRhigh,HIGH);
	digitalWrite(rightPUhigh,HIGH);
	digitalWrite(rightMFhigh,HIGH);

	digitalWrite(frontDRhigh,HIGH);
	digitalWrite(frontPUhigh,HIGH);
	digitalWrite(frontMFhigh,HIGH);

	digitalWrite(backDRhigh,HIGH);
	digitalWrite(backPUhigh,HIGH);
	digitalWrite(backMFhigh,HIGH);

	digitalWrite(centralDRhigh,HIGH);
	digitalWrite(centralPUhigh,HIGH);
	digitalWrite(centralMFhigh,HIGH);


	digitalWrite(leftDRlow,HIGH);
	digitalWrite(leftMFlow,HIGH);

	digitalWrite(rightDRlow,HIGH);
	digitalWrite(rightMFlow,HIGH);

	digitalWrite(frontDRlow,HIGH);
	digitalWrite(frontMFlow,HIGH);

	digitalWrite(backDRlow,HIGH);
	digitalWrite(backMFlow,HIGH);

	digitalWrite(centralDRlow,HIGH);
	digitalWrite(centralMFlow,HIGH);

	Serial.begin(9600);

}

void moveServoMotor(int pin){
	for(int i=0;i<10;i++)
	{
		digitalWrite(pin,HIGH);
		delayMicroseconds(600);
		digitalWrite(pin,LOW);
		delayMicroseconds(600);
	}
}

void loop() {
	if(Serial.available()>0)
	{
		int instruct = (int)Serial.read();
		Serial.println(instruct);
		if((instruct & 1) and (instruct & 16))
		{
			digitalWrite(centralDRlow,HIGH);
			moveServoMotor(centralPUlow);
			Serial.println("CF");
		}
		else if((instruct & 2) and (instruct & 32))
		{
			digitalWrite(centralDRlow,LOW);
			moveServoMotor(centralPUlow);
			Serial.println("CB");
		}
		else if((instruct & 4) and (instruct & 64))
		{
			ServoBLDC.write(140);
			Serial.println("SS");
		}
		else if((instruct & 8) and (instruct & 128))
		{
			ServoBLDC.write(0);
			Serial.println("ST");
		}
		else if((instruct & 1) and (instruct & 8))
		{
			ServoActive = 1;
		}
		else if((instruct & 4) and (instruct & 16))
		{
			ServoActive = 0;
		}
		else
		{
			if(instruct & 1)
			{
				digitalWrite(leftDRlow, HIGH);
				moveServoMotor(leftPUlow);
				Serial.println("LF");
			}
			if(instruct & 2)
			{
				digitalWrite(rightDRlow, HIGH);
				moveServoMotor(rightPUlow);
				Serial.println("RF");
			}
			if(instruct & 4)
			{
				digitalWrite(frontDRlow, HIGH);
				moveServoMotor(frontPUlow);
				Serial.println("FU");
			}
			if(instruct & 8)
			{
				digitalWrite(backDRlow, HIGH);
				moveServoMotor(backPUlow);
				Serial.println("BU");
			}
			if(instruct & 16)
			{
				digitalWrite(leftDRlow, LOW);
				moveServoMotor(leftPUlow);
				Serial.println("LB");      
			}
			if(instruct & 32)
			{
				digitalWrite(rightDRlow, LOW);
				moveServoMotor(rightPUlow);
				Serial.println("RF");
			}
			if(instruct & 64)
			{
				digitalWrite(frontDRlow, LOW);
				moveServoMotor(frontPUlow);
				Serial.println("FD");
			}
			if(instruct & 128)
			{
				digitalWrite(backDRlow, LOW);
				moveServoMotor(backPUlow);
				Serial.println("BD");
			}
		}
	}
	if(ServoActive)
	{
		ServoAngle += ServoStep;
		if(ServoAngle>=180)
		{
			ServoAngle = 180;
			ServoStep *= -1;
		}
		else if(ServoAngle<=0)
		{
			ServoAngle = 0;
			ServoStep *= -1;
		}
		ServoSeed.write(ServoAngle);
		delay(10);
	}
}
