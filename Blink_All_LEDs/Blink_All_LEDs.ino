const int BLUE_LED = 13; // Blue "stat" LED on pin 13
const int RX_LED = PIN_LED_RXL; // RX LED on pin 25, we use the predefined PIN_LED_RXL to make sure
const int TX_LED = PIN_LED_TXL; // TX LED on pin 26, we use the predefined PIN_LED_TXL to make sure
const int TRIG =  4;
const int ECHO = 5;

#include "wiring_private.h"

long duration, distance;

bool ledState = LOW;
int pwmSpeed = 60;
int ultrasonicDistance = 50;
int lineSensor = 800;

void setup() 
{
  SerialUSB.begin(9600); // serial initialize
  while(!SerialUSB);  // wait for serial to connect  
  
  pinMode(RX_LED, OUTPUT);
  pinMode(TX_LED, OUTPUT);
  digitalWrite(RX_LED, HIGH);
  digitalWrite(TX_LED, HIGH);
   
  //h bridge 1
  pinPeripheral(BLUE_LED, PIO_TIMER);
  pinPeripheral(11, PIO_TIMER);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT); 

  //h bridge 2
  pinPeripheral(8, PIO_TIMER);
  pinPeripheral(6, PIO_TIMER);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  analogReadResolution(10);

  //ultrasonic sensor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() 
{

  ledState = !ledState;
  digitalWrite(RX_LED, ledState);
  digitalWrite(TX_LED, !ledState);
    
  moveStraight();

  //send ultrasonic signal
  digitalWrite(TRIG, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = (duration/2) / 29.1;   
  if (analogRead(A2) > lineSensor || analogRead(A1) > lineSensor) { 
//    SerialUSB.println("a");
//    SerialUSB.println(analogRead(A2));
//    SerialUSB.println("b");
//    SerialUSB.println(analogRead(A1));
    moveBackward();
    delay(2000);
    turnRight();  
  }
  else if (SerialUSB.available()) {
//    SerialUSB.println("c");
//    SerialUSB.println(distance);
    if (distance < ultrasonicDistance && distance > 0) {
      turnRight();
    }
  }
  
 }
  
//
//  turnRight();
//
//  delay(1000);
//
//  shortBreak();
//  
//  delay(1000);
  
//}

void moveStraight() {
  //h bridge 1 cw
  analogWrite(BLUE_LED, 255); //pwm
  digitalWrite(12, LOW);
  analogWrite(11, pwmSpeed);

  //h bridge 2 ccw
  analogWrite(6, 255); //pwm
  digitalWrite(7, HIGH);
  analogWrite(8, pwmSpeed);
}

void turnLeft() {
   //h bridge 1 cw
  analogWrite(BLUE_LED, 255);
  digitalWrite(12, HIGH);
  analogWrite(11, 255);

  //h bridge 2 cw
  analogWrite(8, 255);
  digitalWrite(7, HIGH);
  analogWrite(6, 255);

  delay(225);
}

void turnRight() {
  //h bridge 1 cw
  analogWrite(BLUE_LED, 255);
  digitalWrite(12, LOW);
  analogWrite(11, 255);

  //h bridge 2 cw
  analogWrite(8, 255);
  digitalWrite(7, LOW);
  analogWrite(6, 255);

  delay(225);
}


void moveBackward() {
   //h bridge 1 ccw
  analogWrite(BLUE_LED, 255);
  digitalWrite(12, HIGH);
  analogWrite(11, pwmSpeed);

  //h bridge 2 cw
  analogWrite(6, 255);
  digitalWrite(7, LOW);
  analogWrite(8, pwmSpeed);
}

void shortBreak() {
   //h bridge 1 break
  analogWrite(11, 0);

  //h bridge 2 break
  analogWrite(8,  0);
}
