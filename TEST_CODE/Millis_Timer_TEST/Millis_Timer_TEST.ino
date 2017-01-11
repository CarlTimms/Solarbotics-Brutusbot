void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  int last_looked_around;
  
  Serial.println(millis());

  last_looked_around = millis();

  while(1)
  {
    if ((millis() - last_looked_around) >= 2000)
    {
      last_looked_around = millis();
      Serial.println(last_looked_around);
    }
  }
  
}
