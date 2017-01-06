

/*
 *  SolarBotics BrutusBot
 *
 *  Author:   Carl F Timms
 *  Date:     January 6 2017
 */
 
 
// libraries
#include "TimerOne.h"
#include <Servo.h> 
#include <NewPing.h>
#include <SharpIR.h>


// preprocessor directives
#define ACCEL_X_TILT_THRESH   20      //accelerometer tilt threshold left/right
#define ACCEL_Y_TILT_THRESH   50      //accelerometer tilt threshold forward/backward
#define SERVO_MAX_RIGHT       600     //540 microseconds max right
#define SERVO_MAX_LEFT        2400    //2400 microseconds max left
#define US_TRIGGER_PIN        2       //ultrasonic trigger pin 
#define US_ECHO_PIN           4       //ultrasonic echo pin          
#define US_MAX_DISTANCE       500     //ultrasonic max range
#define IR_PIN                5       //the pin where your sensor is attached  
#define IR_READINGS           6       //the number of readings the library will make before calculating an average distance.          
#define IR_DIFFERENCE         95      //the difference between two consecutive measurements to be taken as valid (in %)          
#define IR_MODEL              1080    //is an int that determines your sensor:  1080 for GP2Y0A21Y, 20150 for GP2Y0A02Y          


// pins
const byte  switchPinLeft   =   12; //left feeler digital pin 12
const byte  switchPinRight  =   13; //left feeler digital pin 13
const int   yPin            =   4;  //y-axis accelerometer analogue pin 4
const int   xPin            =   5;  //x-axis accelerometer analogue pin 5


// global variables 
volatile int seconds = 0; //seconds since program start 
volatile int fullSec = 0; //flag - timed events only happen once in specified second
volatile int driveMode = 0;
volatile int program = 1;
Servo servo;
NewPing sonar(US_TRIGGER_PIN, US_ECHO_PIN, US_MAX_DISTANCE);
SharpIR sharp(IR_PIN, IR_READINGS, IR_DIFFERENCE, IR_MODEL); 


// function prototypes
void drive(int command);
bool feelerContact() ;
int sampleX();
int sampleY();
bool stability();
void panServo(int deg);
int ultrasonic();
int infrared();
void sec_ISR();
void brutusbot();


// *** SETUP ***
void setup() 
{
  // initialise Timer1 and call function every second
  Timer1.initialize();
  Timer1.attachInterrupt(sec_ISR, 1000000);

  // bump switches
  pinMode (switchPinLeft, INPUT);   // Feeler pin left -> Input
  pinMode (switchPinRight, INPUT);  // Feeler pin right -> Input

  // attach servo to D10
  servo.attach(10);
  
  // begin serial communication
  Serial.begin(9600);
  
  delay(10000);
  Serial.println("!!! START !!!");
}




// *** Loop ***
void loop() 
{
    brutusbot();
}




// *** Functions ***


void drive(int command)
/* 
 *  Defines the pre-set driving modes.
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
     *  digitalWrite:  use for only LOW or HIGH
     *  analogWrite:   use for PWM signal 0-255
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
 *  Returns TRUE if either feeler is in contact with an object.
 *  Returns FALSE otherwise.
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


int sampleX() 
/*
 *  Returns an averaged sample of values from the accelelerometer X axis (L/R).
 */
{
  int maxSamples = 10; 
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


// 
int sampleY() 
/*
 *  Returns an averaged sample of values from the accelelerometer Y axis (F/B).
 */
{
  int maxSamples = 10; 
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


bool stability()
/* 
 *  Measures tilt angles.
 *  If BB tilted too far in any direction, return FALSE.
 */
{
  //read the analog values from the accelerometer
  int xRead = sampleX();
  int yRead = sampleY();

  if ((xRead < (-ACCEL_X_TILT_THRESH)) || (xRead > ACCEL_X_TILT_THRESH)) 
  {
    return false;
  }
  else if ((yRead < (-ACCEL_Y_TILT_THRESH)) || (yRead > ACCEL_Y_TILT_THRESH)) 
  {
    return false;
  } 
  else 
  {
    return true;
  }
}


void panServo(int deg)
/*
 *  Rotates the servo to given angle.
 *  Full right:  0
 *  Full left:   180
 */
{
  int mapDegrees = map(deg, 0, 180, SERVO_MAX_RIGHT, SERVO_MAX_LEFT); //map(value, fromLow, fromHigh, toLow, toHigh)
  servo.writeMicroseconds(mapDegrees);
}


int ultrasonic() 
{
  /*
   *  Returns reading from ultrasonic sensor in cm
   */

  int USping = sonar.ping_median(10);
  int UScm = sonar.convert_cm(USping);

  if(USping == 0) {
    return 500;
  }
  
  return UScm;
}



int infrared()
/*  
 *  Returns Infrared sensor reading
 */
{
  int IRping = sharp.distance();

//  Serial.print("Infrared: ");
//  Serial.println(IRping);

  return IRping;
}



void sec_ISR()
/*
 * Increment one second. 
 * Set fullSec flag to 1
 */
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
    if(!stability())              
    {
      //bb is unstable. correct so he does not tip
        
      //read the analog values from the accelerometer
      int xRead = sampleX();
      int yRead = sampleY();
    
      if (yRead > ACCEL_Y_TILT_THRESH) { //if too far forward
        Serial.println("too far forward");
        drive(0);
        //TODO - decide how to react
        continue;
      } 
      
      if (yRead < (-ACCEL_Y_TILT_THRESH)) { //if too far back
        Serial.println("too far back");
        drive(0);
        //TODO - decide how to react.       
        continue;
      }  
      
      if (xRead < (-ACCEL_X_TILT_THRESH)) { //if too far left
        Serial.println("too far left");
        drive(3); //spin left to avoid slope
        while(xRead < (-ACCEL_X_TILT_THRESH))
        {
          xRead = sampleX(); 
        }
        continue;
      }
      
      if (xRead > ACCEL_X_TILT_THRESH) { //if too far right
        Serial.println("too far right");
        drive(4); //spin right to avoid slope
        while(xRead > ACCEL_X_TILT_THRESH)
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
        //
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


