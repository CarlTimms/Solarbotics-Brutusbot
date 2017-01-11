/* 
 A program which uses the Timer1 Library to count seconds

 Carl Timms 
 December 31 2016
*/


// include Timer1 library
#include "TimerOne.h"


// seconds since program start 
volatile int seconds = 0; 
// flag - timed events only happen once in specified second
volatile int checked = 0;


void setup() 
{

  // initialise Timer1 and call function every second
  Timer1.initialize();
  Timer1.attachInterrupt(sec_ISR, 1000000);

  // start serial communication
  Serial.begin(9600);
}


void loop() 
{
  // output to serial
  if(checked)
  {
    Serial.println(seconds);
    checked = 0;

  } 
  
  if (seconds > 5)
  {
    seconds = 0;
  }
}


// increment seconds. 0-5 
void sec_ISR()
{
  seconds++;
  checked = 1;
}


