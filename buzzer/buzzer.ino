int freq;
int channel = 10;
//int resolution = 40;
void setup() {
 Serial.begin(115200);
 //ledcSetup(channel, freq, resolution);
 ledcAttachPin(25, channel);
}
void loop() {
// ledcWriteTone(channel, 2000);
// for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle=dutyCycle+10){
//   Serial.println(dutyCycle);
//   ledcWrite(channel, dutyCycle);
//   delay(1000);
// }
 
 //ledcWrite(channel, 10000);
// for (int freq = 255; freq < 5000; freq=freq+100){
//    Serial.println(freq);
    ledcWriteTone(channel, freq=1755);
    delay(3000);
 
}
