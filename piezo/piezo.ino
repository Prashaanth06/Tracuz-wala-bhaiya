#define piezoPin 39
#define buzzerPin 25
int initial,finalv;

void setup() {
  // put your setup code here, to run once:
pinMode(piezoPin,INPUT);
pinMode(buzzerPin,OUTPUT);
 Serial.begin(112500);
}

void loop() {
  // put your main code here, to run repeatedly:

Serial.println(analogRead(piezoPin ));

   delay(500);
    initial=finalv;
    finalv= analogRead(piezoPin);
    if(abs(finalv-initial)>=200)
    {
     Serial.print("a=");
     Serial.println(abs(finalv-initial));
    digitalWrite(buzzerPin, HIGH);
    }
   
}
  
