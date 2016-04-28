const int BLUE_LED = 13; // Blue "stat" LED on pin 13
const int RX_LED = PIN_LED_RXL; // RX LED on pin 25, we use the predefined PIN_LED_RXL to make sure
const int TX_LED = PIN_LED_TXL; // TX LED on pin 26, we use the predefined PIN_LED_TXL to make sure
#include "wiring_private.h"

bool ledState = LOW;

void setup() 
{
  pinPeripheral(BLUE_LED, PIO_TIMER);
  pinPeripheral(12, PIO_TIMER);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(12, OUTPUT);
  //pinMode(11, OUTPUT);
  pinMode(RX_LED, OUTPUT);
  pinMode(TX_LED, OUTPUT);
  digitalWrite(RX_LED, HIGH);
  digitalWrite(TX_LED, HIGH);
 // digitalWrite(12, HIGH);
//  digitalWrite(BLUE_LED, HIGH);
}

void loop() 
{
  // Toggle RX and TX LED's
  ledState = !ledState;
  digitalWrite(RX_LED, ledState);
  digitalWrite(TX_LED, !ledState);

//   Ramp the blue LED up:
  for (int i=0; i<256; i++)
  {
    analogWrite(BLUE_LED, 255);
    analogWrite(12, 255);
  }
  // Ramp the blue LED down:
//  for (int i=255; i>=0; i--)
//  {
//    analogWrite(BLUE_LED, 0);
//    analogWrite(12,  i);
//    delay(2);
//  }
}

