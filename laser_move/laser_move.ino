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
const int ultrasonicDistance = 50;
const int lineSensor = 800;

double laser_value = 0;
bool ledState = LOW;

//ultrasonic init
long duration;
long distance;


// servo initialization code
uint16_t minpulse = 0.5;
uint16_t maxpulse = 2.5;
int servoPosition = 10;
int increment = 10;
int LEFT_SPEED = pwmSpeed;
int RIGHT_SPEED = pwmSpeed;
int count = 0;

void setup() {
  servo_1.attach(SERVO_PIN, minpulse, maxpulse); // lets the servo pin know it is operating a servo 
  // Serial output code
  //SerialUSB.begin(9600); // serial initialize
  //while(!SerialUSB) ;  // wait for serial to connect 

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

  /*if (analogRead(A0) > 800 || analogRead(A1) > 800) { 
    moveBackward();
    delay(2000);
    turnRight();  
  }*/

  
  
  scanServo(servoPosition, increment, &servoPosition, &increment);
  
  moveStraight(RIGHT_SPEED, LEFT_SPEED);

  laser_value = checkLaser(LASER_PIN);

  if (laser_value >= 100){
    if (servoPosition >= 90 ){
      //speed up the right motor
      LEFT_SPEED = LEFT_SPEED + 20;
      
    }

    else {
      //speed up the left motor
      RIGHT_SPEED = RIGHT_SPEED + 20;
    }
     
    
    }// end laser val if

    if (count >= 8) {
      RIGHT_SPEED = pwmSpeed;
      LEFT_SPEED = pwmSpeed;
      count = 0;
      }

    count++;
  
  // check line sensor left
  //int line_val_left = analogRead(LINE_SENSOR_LEFT_PIN);

  // check line sensor right
  //int line_val_right = analogRead(LINE_SENSOR_RIGHT_PIN);

  // check ultrasonic sensor
  //int ulta_value = checkUlta();
  
  // get laser position
  //int laser_value = checkLaser(LASER_PIN);
  
  // make decision - is obstacle close? is robot on table edge 
  
  // execute!

  // short delay

  
  //if (SerialUSB.available()) {
  //laser_value = checkLaser(LASER_PIN);
  //SerialUSB.println(laser_value);
  
  
  //SerialUSB.println("Servo_position: ");
  //SerialUSB.println(servoPosition);
  //SerialUSB.println("Servo_increment: ");
  //SerialUSB.println(increment);
  //} // end if statement

} // end loop

double checkUlta(){
  digitalWrite(TRIG, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = (duration/2) / 29.1; 
  return distance
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
 *servoPosition = pos +servoIncrement;
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
  digitalWrite(BLUE_LED, HIGH); //pwm pin
  digitalWrite(12, LOW);
  analogWrite(11, right_pwm);

  //h bridge 2 ccw
  digitalWrite(6, HIGH); //pwm pin
  digitalWrite(7, HIGH);
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


void moveBackward() {
   //h bridge 1 ccw
  analogWrite(BLUE_LED, pwmSpeed);
  digitalWrite(12, HIGH);
  analogWrite(11, pwmSpeed); // why is this pwmSpeed?

  //h bridge 2 cw
  analogWrite(8, pwmSpeed);
  digitalWrite(7, LOW);
  analogWrite(6, pwmSpeed); // why is this pwmSpeed?
} // end move backwards

void shortBreak() {
   //h bridge 1 break
  analogWrite(11, 0);
  //h bridge 2 break
  analogWrite(8,  0);
} // end short break
