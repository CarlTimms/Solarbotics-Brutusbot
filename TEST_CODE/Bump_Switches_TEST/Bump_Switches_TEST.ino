/*
 *  SolarBotics BrutusBot
 *
 *  Test sketch: limit switch bumpers
 *
 */


// pins
const byte switchPinLeft = 12;  //left feeler digital pin 12
const byte switchPinRight = 13; //left feeler digital pin 13


void setup() {
  // bump switches
  pinMode (switchPinLeft, INPUT);   // Feeler pin left -> Input
  pinMode (switchPinRight, INPUT);  // Feeler pin right -> Input

  // begin serial communication
  Serial.begin(9600);
  
  delay(3000);
}

void loop() {
  bumpSwitches();
}


//checkFeelers() -> if either if BBs feelers is in contact with an object, return true.
bool feelerContact() {
  if ( digitalRead(switchPinLeft) && digitalRead(switchPinRight) ) //if both feelers are not in contact (feelers high when not in contact with an object)
  { 
    return false;
  } 
  else //if at least one feeler is in contact
  { 
    return true;
  }
}


void bumpSwitches()
{
  if (feelerContact())
  {
    Serial.println("*** CONTACT ***");
  }
  else
  {
    Serial.println("~");
  }
  
  delay(1000);
}

