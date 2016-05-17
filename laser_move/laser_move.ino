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
#define TRIG_LEFT 4
#define ECHO_LEFT 5
#define TRIG_RIGHT 2
#define ECHO_RIGHT 3
#define DEATH_SPIRAL_PIN 9

const int straightSpeed = 120;
const int turnSpeed = 150;
const int ultrasonicDistance = 30;
const int lineSensor = 800;
const int laserTrigger = 300;

//intialize sensors
double laserPos = 0;
double ultraLeftDistance = 0;
double ultraRightDistance = 0;
int lineLeft = 0;
int lineRight = 0;
double duration = 0;
double distance = 0;
bool ledState = LOW;
bool rightTurnBool = true;

//intialize speed
int leftSpeed = straightSpeed;
int rightSpeed = straightSpeed;

// servo initialization code
uint16_t minpulse = 0.5;
uint16_t maxpulse = 2.5;
int increment = 10;
int servoDelay = 100;

void setup() {
  servo_1.attach(SERVO_PIN, minpulse, maxpulse); // lets the servo pin know it is operating a servo 
  // Serial output code
//  SerialUSB.begin(9600); // serial initialize
//  while(!SerialUSB) ;  // wait for serial to connect 

  // pinMode(15, OUTPUT);
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
  
  //ultrasonic sensors
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  //death spiral
  pinMode(DEATH_SPIRAL_PIN, OUTPUT);
  analogWrite(DEATH_SPIRAL_PIN, 255);

  moveStraight(straightSpeed, straightSpeed);
}

void loop() 
{
  rightSpeed = straightSpeed;
  leftSpeed = straightSpeed; 
  ledState = !ledState;
  digitalWrite(RX_LED, ledState);
  digitalWrite(TX_LED, !ledState);

  analogWrite(DEATH_SPIRAL_PIN, 255);

  shortBreak();

  //start moving
//  moveStraight(rightSpeed, leftSpeed);

//  SerialUSB.println("loop");
  //gather sensor data
  laserPos = scanServo();
//  ultraLeftDistance = checkUltraLeft();
//  ultraRightDistance = checkUltraRight();
//  lineLeft = analogRead(A1);
//  lineRight = analogRead(A2);
//  SerialUSB.println(lineLeft);
//  SerialUSB.println(lineRight);
//  SerialUSB.println("left");
//  SerialUSB.println(ultraLeftDistance);
//  SerialUSB.println("right");
//  SerialUSB.println(ultraRightDistance);

 //delay(1000);

 // SerialUSB.println("loop");
  //make decision
  //checks if the cup has been found and follows it
  if (laserPos != -1 && !(lineLeft > lineSensor || lineRight > lineSensor)) {
  //  SerialUSB.println("cup detected");
  //  SerialUSB.println(laserPos);
    if (laserPos < 90) {
   //   SerialUSB.println("in first if");
      turnRight(90 - (laserPos/2 + 45));
      rightSpeed = 195;
      leftSpeed = 225;
      //found the cup!
      while(laserPos != -1) {
     //   SerialUSB.println("laserPosbegin");
        moveStraight(rightSpeed, leftSpeed);
        laserPos = scanServoPursue(laserPos);
     //   SerialUSB.println("laserPos");
     //   SerialUSB.println(laserPos);
        if (laserPos < 90 && laserPos > 0) {
     //     SerialUSB.println("turn");
     //     SerialUSB.println(90 - (laserPos/2 + 45));
          turnRight(90 - (laserPos/2 + 45));
        //turn right when pursuing, needs to be tweaked
        }
        else if (laserPos > 90) {
     //     SerialUSB.println("turn");
     //     SerialUSB.println(laserPos/2 - 90 + 45);
          turnLeft(laserPos/2 - 90 + 45);
        }
      }
     // SerialUSB.println("right");
    }
    else {
     // SerialUSB.println("left");
     //SerialUSB.println("in second if");
      turnLeft(laserPos/2 - 90 + 45);
      rightSpeed = 195;
      leftSpeed = 225;
      while(laserPos != -1) {
        //SerialUSB.println("laserPosbegin");
        moveStraight(rightSpeed, leftSpeed);
        laserPos = scanServoPursue(laserPos);
        //SerialUSB.println("laserPos");
        //SerialUSB.println(laserPos);
    //    SerialUSB.println(laserPos);
        if (laserPos < 90 && laserPos > 0) {
     //     SerialUSB.println("turn");
     //     SerialUSB.println(90 - (laserPos/2 + 45));
          turnRight(90 - (laserPos/2 + 45));
        }
        else if (laserPos > 90) {
      //    SerialUSB.println("turn");
      //    SerialUSB.println(laserPos/2 - 90 + 45);
          turnLeft(laserPos/2 - 90 + 45);
        }
        //moveStraight(rightSpeed, leftSpeed);
      }
    }
  }

  else {
    rightSpeed = straightSpeed;
    leftSpeed = straightSpeed;

   // SerialUSB.println(rightSpeed);
  //dind't fine the cup so it moves forward
    for (int j = 0; j < 20; j++) {

   //   SerialUSB.println("walking");
      moveStraight(rightSpeed, leftSpeed);

    // SerialUSB.println(j);
     
//      if ((j % 25) == 0) {
//        //SerialUSB.println("increase");
//        rightSpeed ++;
//        leftSpeed --; 
//      }

  //get sensor data
    ultraLeftDistance = checkUltraLeft();
    ultraRightDistance = checkUltraRight();
    lineLeft = analogRead(A1);
    lineRight = analogRead(A2);

//      SerialUSB.println(lineLeft);
//      SerialUSB.println(lineRight);

      if (lineLeft > lineSensor || lineRight > lineSensor) { 
//  SerialUSB.println("line detected");
//  SerialUSB.println(lineLeft);
//  SerialUSB.println(lineRight);
        if (lineRight > lineSensor) {
     //     SerialUSB.println("first if");
          moveBackward(straightSpeed, straightSpeed);
          delay(700);
          turnRight(60); 
          break; 
        }
//        else if (lineLeft > lineSensor) {
//      //    SerialUSB.println("second if");
//          turnRight(90);
//          break;
//        }
        else {
      //    SerialUSB.println("third if");
          moveBackward(straightSpeed, straightSpeed);
          delay(700);
          turnLeft(60);
          break;
        }
      }
      else if (abs((ultraLeftDistance < ultrasonicDistance) && (ultraRightDistance < ultrasonicDistance) && (ultraLeftDistance - ultraRightDistance) < 10) && ultraLeftDistance > 0 && ultraRightDistance > 0) { //check if 0 and make into one check
    //completely turn
        turnRight(90);
        break;
//    SerialUSB.println("complete turn");
      }
      else if (ultraLeftDistance < ultrasonicDistance) {
        turnLeft(45);
        break;
//    SerialUSB.println("slight turn left");
      }
      else if (ultraRightDistance < ultrasonicDistance) {
        turnRight(45);
        break;
//    SerialUSB.println("slight turn right");
      }
    } //end for loop
    if (rightTurnBool == true) {
      turnRight(45);
      rightTurnBool = false;
    }
    else {
      turnLeft(45);
      rightTurnBool = true;
    }
  } // end else
} // end loop

double checkUltraLeft() {
  //send left ultrasonic signal and determine distance
  digitalWrite(TRIG_LEFT, LOW); 
  delayMicroseconds(2);
  digitalWrite(TRIG_LEFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_LEFT, LOW);
  duration = pulseIn(ECHO_LEFT, HIGH);
  distance = (duration/2) / 29.1; 
  return distance;
 }

 double checkUltraRight() {
  //send left ultrasonic signal and determine distance
  digitalWrite(TRIG_RIGHT, LOW); 
  delayMicroseconds(2);
  digitalWrite(TRIG_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_RIGHT, LOW);
  duration = pulseIn(ECHO_RIGHT, HIGH);
  distance = (duration/2) / 29.1; 
  return distance;
 }

double checkLaser(int pin_select) {
  double value;
  //analogWrite(15, 255);
  value = analogRead(pin_select);
  return value; 
} // end check laser

  int scanServo() {
    servo_1.write(0);
    delay(750); // wait for servo to get back to position 0
    double tempValue = 0;
    double maxLaserValue = laserTrigger;
    int tempLaserPos = -1;
//    SerialUSB.println("reset");
//    SerialUSB.println("tempLaserPos");
//    SerialUSB.println(tempLaserPos);
//    SerialUSB.println("maxLaserValue");
//    SerialUSB.println(maxLaserValue);
//    SerialUSB.println("tempValue");
//    SerialUSB.println(tempValue);
    for (int i = 0; i < 180; i = i + 10) {
      //int i = 90;
      //SerialUSB.println(i);
      servo_1.write(i);
      delay(servoDelay);
      tempValue = analogRead(LASER_PIN);
      //SerialUSB.println(tempValue);
      if (tempValue > maxLaserValue) {
        maxLaserValue = tempValue;
        tempLaserPos = i;
      } 
    }
//    SerialUSB.println("tempLaserPos");
//    SerialUSB.println(tempLaserPos);
//    SerialUSB.println("maxLaserValue");
//    SerialUSB.println(maxLaserValue);
    return tempLaserPos;
  }

  int scanServoPursue(int initPos) {
    servo_1.write(90);
    delay(750); // wait for servo to get back to position 0
    double tempValue1 = 0;
    double tempValue2 = 0;
    int tempLaserPos1 = -1;
    int tempLaserPos2 = -1;
    int pursueIncrement = 15;
    int count = 0;
    while (tempValue1 < laserTrigger && tempValue2 < laserTrigger && pursueIncrement <= 90) {
      servo_1.write(90 + count*pursueIncrement);
      delay(servoDelay);
      tempValue1 = analogRead(LASER_PIN);
      tempLaserPos1 = 90 + count*pursueIncrement;
      servo_1.write(90 - count*pursueIncrement);
      delay(servoDelay);
      tempValue2 = analogRead(LASER_PIN);
      tempLaserPos2 = 90 - count*pursueIncrement;
      pursueIncrement += 15;
      count++;
     // SerialUSB.println("while");
    }
    if (tempValue1 > laserTrigger && tempValue1 >= tempValue2) {
      //SerialUSB.println(tempLaserPos1);
      return tempLaserPos1;
    }
    else if (tempValue2 > laserTrigger && tempValue2 > tempValue1) {
     // SerialUSB.println(tempLaserPos2);
      return tempLaserPos2;
    }
    else {
      rightSpeed = straightSpeed;
      leftSpeed = straightSpeed;
     // SerialUSB.println("else");
      return -1;
    }
  }


void turnRight(int deg) {
  //h bridge 1 cw
  analogWrite(BLUE_LED, 255);
  digitalWrite(12, LOW);
  analogWrite(11, turnSpeed);

  //h bridge 2 cw
  analogWrite(8, turnSpeed);
  digitalWrite(7, LOW);
  analogWrite(6, 255);

  delay(40/9 * deg); 
}

void turnLeft(int deg) {
   //h bridge 1 cw
  analogWrite(BLUE_LED, 255);
  digitalWrite(12, HIGH);
  analogWrite(11, turnSpeed);

  //h bridge 2 cw
  analogWrite(8, turnSpeed);
  digitalWrite(7, HIGH);
  analogWrite(6, 255);

  delay(40/9 * deg); 
} // end turn left

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
