const int BLUE_LED = 13; // Blue "stat" LED on pin 13
const int RX_LED = PIN_LED_RXL; // RX LED on pin 25, we use the predefined PIN_LED_RXL to make sure
const int TX_LED = PIN_LED_TXL; // TX LED on pin 26, we use the predefined PIN_LED_TXL to make sure
#include "wiring_private.h"

bool ledState = LOW;

void setup() 
{
 pinMode(RX_LED, OUTPUT);
 pinMode(TX_LED, OUTPUT);
 digitalWrite(RX_LED, HIGH);
 digitalWrite(TX_LED, HIGH);
 
 //h bridge 1
 pinPeripheral(BLUE_LED, PIO_PWM);
 pinPeripheral(11, PIO_PWM);
 pinMode(BLUE_LED, OUTPUT);
 pinMode(12, OUTPUT);
 pinMode(11, OUTPUT); 
 digitalWrite(12, HIGH);

 //h bridge 2
  pinPeripheral(8, PIO_TIMER);
  pinPeripheral(6, PIO_TIMER);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(7, LOW);
 
}

void loop() 
{
 // Toggle RX and TX LED's
 ledState = !ledState;
 digitalWrite(RX_LED, ledState);
 digitalWrite(TX_LED, !ledState);

 // Ramp the PWM waves up:
    
   //h bridge 1
   analogWrite(11, 255);
   analogWrite(BLUE_LED, 255);
   //analogWrite(12, 255);

   //h bridge 2
    analogWrite(8, 255);
    analogWrite(6, 255);


 // Ramp the PWM waves down:
    delay(1000);
    //break
    analogWrite(11, 0);
    analogWrite(8,  0);
    delay(1000);

  
}// end main
