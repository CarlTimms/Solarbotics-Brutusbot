



/*
 *  !!! SolarBotics BrutusBot !!!
 *
 *  A sketch for the BrutusBot kit by Solarbotics, Canada
 *    https://solarbotics.com/product/ubb/
 *    
 *  My kit has been improved by the addition of:
 *  - Ultrasonic distance sensor, HC-SRO4
 *  - Sensor bracket, Lynxmotion Aluminum Multi-Purpose Sensor Bracket MPSH-01 
 *  - Limit switch bump sensors
 *  - Accelerometer, Adafruit ADXL-335
 *  
 *  Sketch expands on 'brutusbot_cmd_sharp.ino' by Mark Martens (BRuTuS)
 *  
 *  Sketch size: 8,492 bytes / 32,256 bytes max (26%) 
 *  
 *  Author:   Carl F Timms
 *  Date:     January 10 2017
 */
 


 
//libraries
#include <Servo.h> 
#include <NewPing.h>
#include <SharpIR.h>


// preprocessor directives
#define ACCEL_X_TILT_THRESH   30      //accelerometer tilt threshold left/right
#define ACCEL_Y_TILT_THRESH   45      //accelerometer tilt threshold forward/backward
#define SERVO_MAX_RIGHT       600     //out of 540 microseconds max right
#define SERVO_MAX_LEFT        2400    //out of 2400 microseconds max left
#define US_TRIGGER_PIN        2       //ultrasonic trigger pin 
#define US_ECHO_PIN           4       //ultrasonic echo pin          
#define US_MAX_DISTANCE       400     //ultrasonic max range
#define US_NEAR_THRESHOLD     50      //distance threshold for close objects (cm)
#define US_FAR_THRESHOLD      150     //distance threshold for further away objects (cm)
#define IR_PIN                3       //the pin where your sensor is attached  
#define IR_READINGS           10      //the number of readings the library will make before calculating an average distance.          
#define IR_DIFFERENCE         90      //the difference between two consecutive measurements to be taken as valid (in %)          
#define IR_MODEL              1080    //is an int that determines your sensor:  1080 for GP2Y0A21Y, 20150 for GP2Y0A02Y          
#define IR_THRESHOLD          50      //distance threshold for Sharp IR sensor


//pins
const byte  switchPinLeft   =   12; //left feeler digital pin 12
const byte  switchPinRight  =   13; //left feeler digital pin 13
const int   yPin            =   4;  //y-axis accelerometer analogue pin 4
const int   xPin            =   5;  //x-axis accelerometer analogue pin 5


//global variables 
volatile int seconds = 0;                                     //number of seconds since program began 
volatile int fullSec = 0;                                     //flag - timed events only happen once in specified second
volatile int driveMode = 0;                                   //which pre-defined motor speeds to use 
Servo servo;                                                  //servo object
NewPing sonar(US_TRIGGER_PIN, US_ECHO_PIN, US_MAX_DISTANCE);  //ultrasonic sensor object
SharpIR sharp(IR_PIN, IR_READINGS, IR_DIFFERENCE, IR_MODEL);  //infrared sensor object


// function prototypes
void drive(int command);
bool feelerContact() ;
int sampleX();
int sampleY();
bool stability();
void panServo(int deg);
int ultrasonic();
int infrared();
int look();
void brutusbot();
void test();


// *** SETUP ***
void setup() 
{
  //randomize seed based on unconnected pin.
  randomSeed(analogRead(2));
  
  //attach servo to D10
  servo.attach(10);
  
  //bump switches setup
  pinMode (switchPinLeft, INPUT);   
  pinMode (switchPinRight, INPUT); 

  //begin serial communication
  Serial.begin(9600);
}


// *** Loop ***
void loop() 
{
  delay(5000);
  Serial.println("!!! START !!!");
  
  brutusbot();
//  test();
}


// ****** Functions ******

void drive(int command)
/* 
 *  Defines the pre-set driving modes.
 */
{
  
  /*
   * 0    Stop
   * 1    Forward
   * 2    Reverse 
   * 3    Spin Left
   * 4    Spin right
   * 5    Drift left
   * 6    Drift right
   * 7    Sharp left    
   * 8    Sharp right
   */

  //PWM pins 3 & 5 are Left Motor
  //PWM pins 6 & 11 are Right Motor

  switch (command)
  {
    /*
     *  digitalWrite:  use for only LOW or HIGH
     *  analogWrite:   use for PWM signal 0-255
     */  

    //stop
    case 0: 
      digitalWrite(3, LOW);     //Left Forward 
      digitalWrite(5, LOW);     //Left Reverse
      digitalWrite(6, LOW);     //Right Forward
      digitalWrite(11, LOW);    //Right Reverse
      break;
    
    //forward
    case 1:
      analogWrite(3, 150);      //Left Forward 
      digitalWrite(5, LOW);     //Left Reverse
      analogWrite(6, 150);      //Right Forward
      digitalWrite(11, LOW);    //Right Reverse
      break;
      
    //reverse
    case 2:
      digitalWrite(3, LOW);     //Left Forward 
      analogWrite(5, 150);      //Left Reverse
      digitalWrite(6, LOW);     //Right Forward
      analogWrite(11, 150);     //Right Reverse
      break;
    
    //spin Left
    case 3: 
      digitalWrite(3,LOW);      //Left Forward 
      analogWrite(5,150);       //Left Reverse
      analogWrite(6,150);       //Right Forward
      digitalWrite(11,LOW);     //Right Reverse
      break;
    
    //spin Right
    case 4: 
      analogWrite(3,150);       //Left Forward 
      digitalWrite(5,LOW);      //Left Reverse
      digitalWrite(6,LOW);      //Right Forward
      analogWrite(11,150);      //Right Reverse
      break;

    //drift Left
    case 5: 
      analogWrite(3, 100);      //Left Forward 
      digitalWrite(5, LOW);     //Left Reverse
      analogWrite(6, 150);      //Right Forward
      digitalWrite(11, LOW);    //Right Reverse
      break;
    
    //drift Right
    case 6: 
      analogWrite(3, 150);      //Left Forward 
      digitalWrite(5, LOW);     //Left Reverse
      analogWrite(6, 100);      //Right Forward
      digitalWrite(11, LOW);    //Right Reverse
      break;

    //sharp Left
    case 7: 
      analogWrite(3, 50);       //Left Forward 
      digitalWrite(5, LOW);     //Left Reverse
      analogWrite(6, 150);      //Right Forward
      digitalWrite(11, LOW);    //Right Reverse
      break;
    
    //sharp Right
    case 8: 
      analogWrite(3, 150);      //Left Forward 
      digitalWrite(5, LOW);     //Left Reverse
      analogWrite(6, 50);       //Right Forward
      digitalWrite(11, LOW);    //Right Reverse
      break;         
  }
}


bool feelerContact() 
/*
 *  Returns TRUE if either feeler is in contact with an object.
 *  Returns FALSE otherwise.
 */
{
  if ( digitalRead(switchPinLeft) && digitalRead(switchPinRight) ) //if both feelers are not in contact with an object(feelers read TRUE when not in contact with an object)
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
 *  If BrutusBot tilted too far in any direction, return FALSE.
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
    return true; //BrutusBot stable
  }
}


void panServo(int deg)
/*
 *  Rotates the servo to given angle.
 *  Full right:  0
 *  Full left:   180
 */
{
  int mapDegrees = map(deg, 0, 180, SERVO_MAX_RIGHT, SERVO_MAX_LEFT); //map(measurement variable, fromLow, fromHigh, toLow, toHigh)
  servo.writeMicroseconds(mapDegrees);
}


int ultrasonic() 
/*
 *  Returns reading from ultrasonic sensor in cm
 */
{
  int USping = sonar.ping_median(10); 

  if(USping == 0) { 
    return 400;
  }
  else
  {
    USping = sonar.convert_cm(USping); //convert to cm
    if(USping < 2)
    {
      return 2;
    }
    else
    {
      return USping;
    }
  }
}


int infrared()
/*  
 *  Returns Infrared sensor reading
 */
{
  int ir = sharp.distance() - 2; //IR reads slightly higher than US sensor so subtract 2 cm to match  
  if(ir < 10) //minimum range of sensor 
  {
    return 10;
  }
  else if(ir > 80) //maximum range of sensor
  {
    return 80;
  }
  else
  {
    return ir;  
  } 
}


int look(int angle)
/*
 *  Rotates servo to given angle and returns ultrasonic sensor reading
 */
{
  panServo(angle);
  return ultrasonic();
}


// ****** Main function ******

void brutusbot()
{
  //local variables
  unsigned long last_looked_around;
  int ultrasonicAhead = 0;    
  int ultrasonicLeft = 0;     
  int ultrasonicRight = 0;    
  int randomAngle = 0;        
  int ultrasonicRandom = 0;   
  int infraredAhead = 0;
  int dir = 0;
  
  last_looked_around = millis();
  
  while(1)
  {
    //check if unstable
    if(!stability())              
    {
      //BrutusBot is unstable
      //Correct to avoid tipping over
       
      //read the analog values from the accelerometer
      int xRead = sampleX();
      int yRead = sampleY();
    
      if (yRead > ACCEL_Y_TILT_THRESH) { //if too far forward
        drive(0); //stop
        //TODO: better response
      } 
      
      if (yRead < (-ACCEL_Y_TILT_THRESH)) { //if too far back
        Serial.println("too far back");
        drive(0);
        //TODO: better response.       
      }  
      
      if (xRead < (-ACCEL_X_TILT_THRESH)) { //if too far left
        //spin left to avoid slope
        drive(3); 
        while(xRead < (-ACCEL_X_TILT_THRESH))
        {
          xRead = sampleX(); 
        }
        drive(0);
      }
      
      if (xRead > ACCEL_X_TILT_THRESH) { //if too far right
        //spin right to avoid slope
        drive(4); 
        while(xRead > ACCEL_X_TILT_THRESH)
        {
          xRead = sampleX(); 
        }
        drive(0);
      }
    }
    else //BrutusBot is stable. check if feelers are in contact with an object
    {
      if (feelerContact())
      {  
        //if left feeler in contact with object on left, reverse and turn right
        if(!digitalRead(switchPinLeft))
        {
          drive(2);//back
          delay(1000);
          drive(4);//right
          delay(1000);
          drive(0);
        }
        
        //if right feeler in contact with object on right, reverse and turn left
        if(!digitalRead(switchPinRight))
        {
          drive(2); //back
          delay(1000);
          drive(3); //left
          delay(1000); 
          drive(0);
        }
      }
      else //BrutusBot is stable and feelers are clear of objects.  
      {
        //take distance sensor readings dead ahead
        ultrasonicAhead = look(90);
        infraredAhead = infrared();

        if((ultrasonicAhead < US_NEAR_THRESHOLD) || (infraredAhead < IR_THRESHOLD)) //if object ahead within threshold detected by either US or IR sensor 
        {
          drive(0); //stop
          
          //take readings right, left, centre
          ultrasonicRight = look(70);
          ultrasonicLeft = look(110);
          ultrasonicAhead = look(90);

          if(ultrasonicRight < ultrasonicLeft) //object right
          {
            dir = 3; //spin left
          }
          else //object left
          {
            dir = 4; //spin right
          }

          drive(dir); //start rotation to either left or right 
        
          while((ultrasonicAhead < US_NEAR_THRESHOLD) || (infraredAhead < IR_THRESHOLD)) //false when both above thresholds
          {
            // update readings
            ultrasonicAhead = look(90);
            infraredAhead = infrared();
          }
          drive(0); //stop
        }
        else //no objects dead ahead
        {
          if((millis() - last_looked_around) >= 2000) //two second interval between measurements
          /*
           *  While BrutusBot is in motion look around with the ultrasonic 
           *  sensor every two seconds.
           *  Modify the drive mode to avoid any detected objects
           */
          {
            //reset to current time
            last_looked_around = millis();
            
            //choose a random angle 0-180
            randomAngle = random(0, 181);
            
            //take ultrasonic measurement at randomly chosen angle
            ultrasonicRandom = look(randomAngle);
        
            if (randomAngle >= 120) //between 120 & 180 degrees
            {
              if (ultrasonicRandom < US_NEAR_THRESHOLD)
              {
                driveMode = 8; //sharp right
              }
              else
              {
                driveMode = 1; //straight ahead
              }
            }
            else if (randomAngle >= 90 ) //beween 90 and 120 degrees
            {
              if (ultrasonicRandom < US_NEAR_THRESHOLD)
              {
                driveMode = 8; //sharp turn right
              }
              else if (ultrasonicRandom < US_FAR_THRESHOLD)
              {
                driveMode = 6; //drift right
              }
              else
              {
                driveMode = 1; //straight ahead
              }
            }
            else if (randomAngle >= 60) //beween 60 and 90 degrees
            {
              if (ultrasonicRandom < US_NEAR_THRESHOLD)
              {
                driveMode = 7;//sharp turn left
              }
              else if (ultrasonicRandom < US_FAR_THRESHOLD)
              {
                driveMode = 5;//drift left
              }
              else
              {
                driveMode = 1; //straight ahead
              }
            }
            else //between 0 and 60 degrees
            {
              if (ultrasonicRandom < US_NEAR_THRESHOLD)
              {
                driveMode = 7;//sharp turn left
              }
              else
              {
                driveMode = 1;
              }
            }
          }
          else //in between full seconds
          {
            drive(driveMode);
          }
        }
      }   
    }
  }
}


void test()
/*
 *  Function for debugging, testing new code etc
 */
{
  while(1)
  {
    drive(2);
  }
}

