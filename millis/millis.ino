 unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long gap = 2000; 

 void setup(void) {
  Serial.begin(115200);
pinMode(2,OUTPUT);
 startMillis = millis();
 } 
 void loop(void) {
 currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  Serial.println(millis());
  if (currentMillis - startMillis >= gap)  //test whether the period has elapsed
  {
  digitalWrite(2,HIGH);
  // End of your programme loop
  delay(500);
  startMillis = currentMillis;
  }
  else
  {
    digitalWrite(2,LOW);
  }
 }
