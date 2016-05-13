#include <Servo.h>
#include <math.h>
#include "wiring_private.h"

Servo servo_1;  // initializes servo object

// defining analog input pins
#define ULTRA_SONIC_PIN A3
#define LINE_SENSOR_LEFT_PIN A2
#define LINE_SENSOR_RIGHT_PIN A1
#define LASER_PIN A0
#define SERVO_PIN 10 // need to initialize a pin to run the servo off of, must be PWM compatible 
#define BLUE_LED 13
#define RX_LED PIN_LED_RXL
#define TX_LED PIN_LED_TXL
#define TRIG 4
#define ECHO 5

const int pwmSpeed = 70;
const int ultrasonicDistance = 20;
const int lineSensor = 1000;

//intialize sensors
double laser_value = 0;
double ultraDistance = 0;
int lineLeft = 0;
int lineRight = 0;
double duration = 0;
double distance = 0;

bool ledState = LOW;

//intialize speed
int leftSpeed = pwmSpeed;
int rightSpeed = pwmSpeed;

// servo initialization code
uint16_t minpulse = 0.5;
uint16_t maxpulse = 2.5;
int servoPosition = 10;
int increment = 10;
int count = 0;

void setup() {
  servo_1.attach(SERVO_PIN, minpulse, maxpulse); // lets the servo pin know it is operating a servo 
  // Serial output code
//  SerialUSB.begin(9600); // serial initialize
//  while(!SerialUSB) ;  // wait for serial to connect 

  pinMode(15, OUTPUT);   // sets the pin as output
  bool ledState = 0;

  // motor pin setup
  pinMode(RX_LED, OUTPUT);
  pinMode(TX_LED, OUTPUT);
  digitalWrite(RX_LED, HIGH);
  digitalWrite(TX_LED, HIGH);
   
  //h bridge 1
  pinPeripheral(BLUE_LED, PIO_TIMER);
  pinPeripheral(11, PIO_TIMER);
  pinMode(BLUE_LED, OUTPUT); // pwm pin on h bridge
  pinMode(12, OUTPUT); // direction in b 
  pinMode(11, OUTPUT); // speed in a

  //h bridge 2
  pinPeripheral(8, PIO_TIMER);
  pinPeripheral(6, PIO_TIMER);
  pinMode(8, OUTPUT); // speed in a 
  pinMode(7, OUTPUT); // direction in b
  pinMode(6, OUTPUT); // pwm pin on h bridge
  analogReadResolution(10);
  
  //ultrasonic sensor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  moveStraight(pwmSpeed, pwmSpeed);
}

void loop() 
{
  ledState = !ledState;
  digitalWrite(RX_LED, ledState);
  digitalWrite(TX_LED, !ledState);

  //start servo
  scanServo(servoPosition, increment, &servoPosition, &increment);

  //start moving
  moveStraight(rightSpeed, leftSpeed);

  //gather sensor data
  laser_value = checkLaser(LASER_PIN);
  ultraDistance = checkUltra();
  lineLeft = analogRead(A1);
  lineRight = analogRead(A2);

  //make decision
  if (lineLeft > lineSensor || lineRight > lineSensor) { 
//  SerialUSB.println("line detected");
//  SerialUSB.println(lineLeft);
//  SerialUSB.println(lineRight);
  moveBackward(pwmSpeed, pwmSpeed);
  delay(2000);
  //turnRight();  
  }
  else if (distance < ultrasonicDistance && distance > 0) {
 //   SerialUSB.println("obstacle detected");
    turnRight();
  }
  else if (laser_value >= 100 && servoPosition >= 90) {
   //    SerialUSB.println("laser detected 1");
       //speed up the right motor
      leftSpeed = leftSpeed + 20;
  }
  else if (laser_value >= 100 & servoPosition < 90) {
  //    SerialUSB.println("laser detected 2");
      //speed up the left motor
      rightSpeed = rightSpeed + 20;
  }
   
//    if (count >= 8) {
//      rightSpeed = pwmSpeed;
//      leftSpeed = pwmSpeed;
//      count = 0;
//      }
//
//    count++;
  
} // end loop

double checkUltra() {
  //send ultrasonic signal and determines distance
  digitalWrite(TRIG, LOW); 
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = (duration/2) / 29.1; 
  return distance;
 }

double checkLaser(int pin_select) {
  double value;
  //analogWrite(15, 255);
  value = analogRead(pin_select);
  return value; 
} // end check laser
  
// this function has the servo rotate back and forth, in a "scanning" mode
void scanServo(int pos,int servoIncrement, int *servoPosition, int *increment) {
 servo_1.write(pos); 
 *servoPosition = pos + servoIncrement;
 delay(300);
 //analogWrite(15, 0);
 
 if(pos == 180) {
  *servoPosition = pos-servoIncrement;
  *increment = servoIncrement*-1; 
 }
  
 if(pos == 0) {
    *servoPosition = pos-servoIncrement;
    *increment = servoIncrement*-1;
  } // end if

 } // end scan servo

void moveStraight(int right_pwm, int left_pwm) {
  //h bridge 1 cw
  analogWrite(BLUE_LED, 255); //pwm pin
  digitalWrite(12, HIGH);
  analogWrite(11, right_pwm);

//  //h bridge 2 ccw
  analogWrite(6, 255); //pwm pin
  digitalWrite(7, LOW);
  analogWrite(8, left_pwm);
} // end move straight


void turnLeft() {
   //h bridge 1 cw
  analogWrite(BLUE_LED, 255);
  digitalWrite(12, HIGH);
  analogWrite(11, 255);

  //h bridge 2 cw
  analogWrite(8, 255);
  digitalWrite(7, HIGH);
  analogWrite(6, 255);
} // end turn left

void turnRight() {
  //h bridge 1 cw
  analogWrite(BLUE_LED, 255);
  digitalWrite(12, LOW);
  analogWrite(11, 255);

  //h bridge 2 cw
  analogWrite(8, 255);
  digitalWrite(7, LOW);
  analogWrite(6, 255);
} // end turn right


void moveBackward(int right_pwm, int left_pwm) {
   //h bridge 1 ccw
  analogWrite(BLUE_LED, 255);
  digitalWrite(12, LOW);
  analogWrite(11, right_pwm);

  //h bridge 2 cw
  analogWrite(8, left_pwm);
  digitalWrite(7, HIGH);
  analogWrite(6, 255);
} // end move backwards

void shortBreak() {
   //h bridge 1 break
  analogWrite(11, 0);
  //h bridge 2 break
  analogWrite(8,  0);
} // end short break
