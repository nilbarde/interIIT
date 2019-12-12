#include <Servo.h>
Servo servo_test;         
int angle = 0;    
 
void setup() 
{ 
  servo_test.attach(4);      
} 
  
void loop() 
{ 
  for(angle = 0; angle < 184; angle += 3)    
  {     
    Serial.println(angle);                             
    servo_test.write(angle);
    delay(10);                                      
  } 
 
  delay(100);
  
  for(angle = 184; angle>=1; angle-=3)    
  {                                
    servo_test.write(angle);              
    delay(10);                       
  } 

    delay(100);
}
