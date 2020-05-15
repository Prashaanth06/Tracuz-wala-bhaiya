#include <EEPROM.h>
int val=50;
void setup()
{
  Serial.begin(115200);
  EEPROM.begin(512);
 
}

void loop()
{
  EEPROM.write(20,val);
  EEPROM.commit();
//      val=EEPROM.read(20);
//    Serial.println(val);
//    delay(1000);
}
