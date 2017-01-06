/* 
 LED blink program using Timer1 library

 Carl Timms 
 December 31 2016
*/


// include Timer1 library
#include "TimerOne.h"


volatile int blink = 0; // 1 or 0


void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // initialise Timer1 and call function every second
  Timer1.initialize();
  Timer1.attachInterrupt(flip_ISR, 1000000);
}


// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, blink);
}


// toggle blink variable
void flip_ISR()
{
  blink = !blink;
}

