/*
 *  Test code for Sharp infrared distance sensor
 */


#include <SharpIR.h>


#define IR_PIN 3             //  Infrared sensor
#define IR_READINGS 6        //          ||
#define IR_DIFFERENCE 95     //          ||
#define IR_MODEL 1080        //          ||


SharpIR sharp(IR_PIN, IR_READINGS, IR_DIFFERENCE, IR_MODEL);


void setup() 
{
  Serial.begin(9600);
}


void loop() 
{
  Serial.println(infrared());
}


int infrared()
/*  
 *  Returns Infrared sensor reading
 */
{
  return sharp.distance();
}


