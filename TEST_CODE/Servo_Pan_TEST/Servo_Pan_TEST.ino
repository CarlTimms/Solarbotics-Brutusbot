/*
 * Servo test
 */


// library
#include <Servo.h> 
Servo servo;

#define SERVO_MAX_RIGHT 600 //540 max left
#define SERVO_MAX_LEFT  2400


void setup() {
  servo.attach(10);
  Serial.begin(9600);
  delay(5000);
}


void loop() {
  panServo(0);
  delay(5000);

  panServo(90);
  delay(5000);

  panServo(180);
  delay(10000);
}


void panServo(int deg)
/*
 * Rotates the servo to given angle.
 * Full right:  0
 * Full left:   180
 */
{
  int mapDegrees = map(deg, 0, 180, SERVO_MAX_RIGHT, SERVO_MAX_LEFT); //map(value, fromLow, fromHigh, toLow, toHigh)
  servo.writeMicroseconds(mapDegrees);
}




