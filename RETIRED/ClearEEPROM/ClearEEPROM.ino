#include <EEPROM.h>
void setup() 
{
  Serial.begin(57600);
  Serial.println("Start EEPROM cleaning...");

  for(uint16_t i=0;i< EEPROM.length();i++)
    EEPROM.write(i,0xFF);

  Serial.println("EEPROM cleared!");

}

void loop() {
  // put your main code here, to run repeatedly:

}
