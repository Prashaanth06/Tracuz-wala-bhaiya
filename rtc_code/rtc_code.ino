#include<WiFi.h>
#include<NTPClient.h>
#include<WiFiUdp.h>
#include<time.h>

const char* ssid     = "TRACUZ";
const char* password = "Tracuz@1234";


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
String formattedTime;
String dayStamp;
String timeStamp;

void setup() {
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("NOT CONNECTED");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  timeClient.begin();
  timeClient.setTimeOffset(19800);
   while(!timeClient.update()) {
   timeClient.forceUpdate();
    WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
  }

  while(1)
  {
    formattedTime = timeClient.getFormattedTime();   // normal time runs for sometime
  Serial.println(formattedTime);
  int splitT = formattedTime.indexOf("T");
dayStamp = formattedTime.substring(0, splitT);
timeStamp = formattedTime.substring(splitT+1, formattedTime.length()-1);
   delay(1000);
  }
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
}

void loop() {
  
  
}
