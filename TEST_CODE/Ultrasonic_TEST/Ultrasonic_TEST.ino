/* 
 A program which tests ultrasonic sensor

 Carl Timms 
 December 31 2016
*/


#include <NewPing.h>


#define TRIGGER_PIN           2       //ultrasonic trigger pin 
#define ECHO_PIN              4       //ultrasonic echo pin          
#define MAX_DISTANCE          500     //ultrasonic max range 


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 


void setup() 
{
  Serial.begin (9600);
}

void loop() 
{
  Serial.println(USread());
  delay(500);
}


int USread() {
  /*
   * Returns reading from ultrasonic sensor in cm
   */

  int USping = sonar.ping_median(10);
  int UScm = sonar.convert_cm(USping);

  if(USping == 0) {
    return 500;
  }
  
  return UScm;
}




