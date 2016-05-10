const int BLUE_LED = 13; // Blue "stat" LED on pin 13
const int RX_LED = PIN_LED_RXL; // RX LED on pin 25, we use the predefined PIN_LED_RXL to make sure
const int TX_LED = PIN_LED_TXL; // TX LED on pin 26, we use the predefined PIN_LED_TXL to make sure
#include "wiring_private.h"

bool ledState = LOW;
int pwmSpeed = 127;

void setup() 
{
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
}

void loop() 
{
  // Toggle RX and TX LED's
  ledState = !ledState;
  digitalWrite(RX_LED, ledState);
  digitalWrite(TX_LED, !ledState);
    
  moveStraight();
  
  delay(5000);
  
  turnRight();

  delay(225);

  shortBreak();

  delay(1000);
//
//  turnRight();
//
//  delay(1000);
//
//  shortBreak();
//  
//  delay(1000);
  
}

void moveStraight() {
  //h bridge 1 cw
  analogWrite(BLUE_LED, 255); //pwm
  digitalWrite(12, LOW);
  analogWrite(11, 32);

  //h bridge 2 ccw
  analogWrite(6, 255); //pwm
  digitalWrite(7, HIGH);
  analogWrite(8, 32);
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
}


void moveBackward() {
   //h bridge 1 ccw
  analogWrite(BLUE_LED, pwmSpeed);
  digitalWrite(12, HIGH);
  analogWrite(11, pwmSpeed);

  //h bridge 2 cw
  analogWrite(8, pwmSpeed);
  digitalWrite(7, LOW);
  analogWrite(6, pwmSpeed);
}

void shortBreak() {
   //h bridge 1 break
  analogWrite(11, 0);

  //h bridge 2 break
  analogWrite(8,  0);
}

