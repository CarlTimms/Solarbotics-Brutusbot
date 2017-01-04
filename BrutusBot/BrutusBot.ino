

/*
 *  SolarBotics BrutusBot
 *
 *  Part 1. Navigation by bump sensors and accelerometer only
 *
 *
 *  Author:   Carl F Timms
 *  Date:     January 2 2017
 */
 
 
// libraries
#include "TimerOne.h"


// pins
const byte  switchPinLeft =   12; //left feeler digital pin 12
const byte  switchPinRight =  13; //left feeler digital pin 13
const int   yPin =             4; //y-axis accelerometer analogue pin 4
const int   xPin =             5; //x-axis accelerometer analogue pin 5


// definitions
#define X_TILT_THRESH         20      //accelerometer tilt threshold left/right
#define Y_TILT_THRESH         50      //accelerometer tilt threshold forward/backward
#define MAX_ACCEL_SAMPLES     10      //sample size of accelerometer read functions


// global variables
volatile int seconds = 0; //seconds since program start 
volatile int fullSec = 0; //flag - timed events only happen once in specified second
volatile int driveMode = 0;
volatile int program = 1;


 
 
void setup() 
{
  // initialise Timer1 and call function every second
  Timer1.initialize();
  Timer1.attachInterrupt(sec_ISR, 1000000);

  // bump switches
  pinMode (switchPinLeft, INPUT);   // Feeler pin left -> Input
  pinMode (switchPinRight, INPUT);  // Feeler pin right -> Input

  // begin serial communication
  Serial.begin(9600);
  
  delay(10000);
  Serial.println("!!! START !!!");
}




void loop() 
{
    brutusbot();
}




// *** Functions ***


void drive(int command)
/* 
 *  Defines the pre-set driving modes
 */
{
  
  /*
   * 0     Stop
   * 1     Forward
   * 2     Reverse 
   * 3     Slow spin Left
   * 4     Slow spin right
   */

  // 3 & 5 are Left Motor
  // 6 & 11 are Right Motor

  switch (command)
  {
    /*
     * digitalWrite:  use for only LOW or HIGH
     * analogWrite:   use for PWM signal 0-255
     */  

   
    // Stop
    case 0: 
      digitalWrite(3, LOW);     // Left Forward 
      digitalWrite(5, LOW);     // Left Reverse
      digitalWrite(6, LOW);     // Right Forward
      digitalWrite(11, LOW);    // Right Reverse
      break;
    
    // Forward
    case 1:
      analogWrite(3, 100);      // Left Forward 
      digitalWrite(5, LOW);     // Left Reverse
      analogWrite(6, 100);      // Right Forward
      digitalWrite(11, LOW);    // Right Reverse
      break;
      
    // Reverse
    case 2:
      digitalWrite(3, LOW);     // Left Forward 
      analogWrite(5, 100);      // Left Reverse
      digitalWrite(6, LOW);     // Right Forward
      analogWrite(11, 100);     // Right Reverse
      break;
    
    // slow spin Left
    case 3: 
      digitalWrite(3,LOW);      // Left Forward 
      analogWrite(5,50);        // Left Reverse
      analogWrite(6,50);        // Right Forward
      digitalWrite(11,LOW);     // Right Reverse
      break;
    
    // slow pin Right
    case 4: 
      analogWrite(3,50);        // Left Forward 
      digitalWrite(5,LOW);      // Left Reverse
      digitalWrite(6,LOW);      // Right Forward
      analogWrite(11,50);       // Right Reverse
      break;        
  }
}


bool feelerContact() 
/*
 * Returns TRUE if either feeler is in contact with an object.
 * Returns False otherwise
 */
{
  if ( digitalRead(switchPinLeft) && digitalRead(switchPinRight) ) //if both feelers are not in contact (feelers read TRUE when not in contact with an object)
  { 
    return false;
  } 
  else //if at least one feeler is in contact
  { 
    return true;
  }
}


// returns an averaged sample of values from the accelelerometer X axis (L/R)
int sampleX() 
{
  int maxSamples = MAX_ACCEL_SAMPLES; 
  int noSamples = 0;
  int xTally = 0;

  while (noSamples <= maxSamples) 
  {
    int xRead = analogRead(xPin);
    int xx = map(xRead, 265, 400, -90, 90);
    noSamples += 1;
    xTally += xx;
  }

  int avReadX = (xTally / maxSamples);
  return avReadX;
}


// returns an averaged sample of values from the accelelerometer Y axis (F/B)
int sampleY() 
{
  int maxSamples = MAX_ACCEL_SAMPLES; 
  int noSamples = 0;
  int yTally = 0;

  while (noSamples <= maxSamples) 
  {
    int yRead = analogRead(yPin);
    int yy = map(yRead, 265, 400, -90, 90);
    noSamples += 1;
    yTally += yy;
  }

  int avReadY = (yTally / maxSamples);
  return avReadY;
}


//if BB tilted too far in any direction, return false.
bool stability() 
{
  //read the analog values from the accelerometer
  int xRead = sampleX();
  int yRead = sampleY();

  if ((xRead < (-X_TILT_THRESH)) || (xRead > X_TILT_THRESH)) 
  {
    return false;
  }
  else if ((yRead < (-Y_TILT_THRESH)) || (yRead > Y_TILT_THRESH)) 
  {
    return false;
  } 
  else 
  {
    return true;
  }
}



// increment one second 
void sec_ISR()
{
  seconds++;
  fullSec = 1; //fullSec flag reset to TRUE by interrupt once per second
}




// ****** Main function ******

void brutusbot()
{
  while(1)
  {
    //check if unstable
    if(!stability())              //checkStability() -> if BB tilted too far in any direction, return false.
    {
      //bb is unstable. correct so he does not tip
        
      //read the analog values from the accelerometer
      int xRead = sampleX();
      int yRead = sampleY();
    
      if (yRead > Y_TILT_THRESH) { //if too far forward
        Serial.println("too far forward");
        drive(0);
        //TODO - decide how to react
        continue;
      } 
      
      if (yRead < (-Y_TILT_THRESH)) { //if too far back
        Serial.println("too far back");
        drive(0);
        //TODO - decide how to react.       
        continue;
      }  
      
      if (xRead < (-X_TILT_THRESH)) { //if too far left
        Serial.println("too far left");
        drive(3); //spin left to avoid slope
        while(xRead < (-X_TILT_THRESH))
        {
          xRead = sampleX(); 
        }
        continue;
      }
      
      if (xRead > X_TILT_THRESH) { //if too far right
        Serial.println("too far right");
        drive(4); //spin right to avoid slope
        while(xRead > X_TILT_THRESH)
        {
          xRead = sampleX(); 
        }
        continue;
      }
    }
    else // BB is stable. check if feelers are in contact with an object
    {
      if (feelerContact())
      {  
        // if left feeler in contact with object, reverse and turn right
        if(!digitalRead(switchPinLeft))
        {
          Serial.println("*** Contact Left ***");
          drive(2);//back
          delay(1000);
          drive(4);//right
          delay(1000);
        }
        
        // if right feeler in contact with object, reverse and turn left
        if(!digitalRead(switchPinRight))
        {
          Serial.println("*** Contact Right ***");
          drive(2);//back
          delay(1000);
          drive(3);// left
          delay(1000);       
        }
        /*
         * NOTES: 
         * can get occasional false contact signal. Implement debounce?
         * can get stuck in corners. will be fixed by addition of other sensors
         */
      }
      else //BB is stable and feelers are clear of objects.  
      {
        drive(1);
      }
    }
  }
}
/*
 * NOTE:
 * any condition which requires a timing must reset 'fullSec' variable to 0 
 * so it executes only once per relevant second. i.e. is not executed for every
 * loop within that second period
 * e.g.
 *   // output to serial
  if(checked)
  {
    Serial.println(seconds);
    checked = 0;

  } 

  consider limiting number of seconds recorded
  if (seconds > 5)
  {
    seconds = 0;
  }
 */


