#include <Servo.h>
#include <math.h>

Servo servo_1;  // initializes servo object

double laser_value = 0;
int servo_pin = 10;  // need to initialize a pin to run the servo off of, must be PWM compatible 
int laser_pin = 14;
uint16_t minpulse = 0.5;
uint16_t maxpulse = 2.5;
int servoPosition = 10;
int increment = 10;


void setup() {
  servo_1.attach(servo_pin, minpulse, maxpulse); // lets the servo pin know it is operating a servo 
  SerialUSB.begin(9600); // serial initialize
  while(!SerialUSB) ;  // wait for serial to connect  
  pinMode(15, OUTPUT);   // sets the pin as output
  
}


  
void loop() 
{
  if (SerialUSB.available())
  {
  laser_value = checkLaser(laser_pin);
  SerialUSB.println(laser_value);
  


  scanServo(servoPosition, increment, &servoPosition, &increment);
  SerialUSB.println("Servo_position: ");
  SerialUSB.println(servoPosition);
  SerialUSB.println("Servo_increment: ");
  SerialUSB.println(increment);
  
  }

}



double checkLaser(int pin_select)
{
  double value;
  //analogWrite(15, 255);
  value = analogRead(pin_select);
  return value;
  
  }
  
// this function has the servo rotate back and forth, in a "scanning" mode
void scanServo(int pos,int servoIncrement, int *servoPosition, int *increment)
{
 servo_1.write(pos); 

 *servoPosition = pos +servoIncrement;
 delay(300);
 //analogWrite(15, 0);
 if(pos == 180)
 {
  
  *servoPosition = pos-servoIncrement;
  *increment = servoIncrement*-1;
  
  }
 if(pos == 0)
 {
    *servoPosition = pos-servoIncrement;
    *increment = servoIncrement*-1;
    
  }

 
 
 }
