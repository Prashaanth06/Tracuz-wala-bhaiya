
#include<time.h>

String formattedTime;
String dayStamp;
String timeStamp;

int current_hour= 16;
int current_minutes= 10;


void setup() {
  Serial.begin(115200);
}

void loop() {
formattedTime = getFormattedTime();
  Serial.println(formattedTime);
  int splitT = formattedTime.indexOf("T");
  dayStamp = formattedTime.substring(0, splitT);
  timeStamp = formattedTime.substring(splitT+1, formattedTime.length()-1);
  delay(1000);
  
}
