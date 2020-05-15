#include "RTClib.h"
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  50        /* Time ESP32 will go to sleep (in seconds) */
#define TIME_TO_ENABLE  10 


RTC_Millis rtc;

int a,b,c;

RTC_DATA_ATTR int bootCount = 0;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void rtcfunction()
{
   DateTime now = rtc.now();
   a=now.hour();
    Serial.print(a);
    Serial.print(':');
    b=now.minute();
    Serial.print(b);
    Serial.print(':');
    c=now.second();
    Serial.print(c);
    Serial.println();
    Serial.println();   
    delay(1000);
}

void setup(){
  Serial.begin(115200);
  esp_sleep_enable_timer_wakeup(TIME_TO_ENABLE * uS_TO_S_FACTOR);
  //delay(1000); //Take some time to open up the Serial Monitor
   // following line sets the RTC to the date & time this sketch was compiled
    rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    //rtc.adjust(DateTime(2019, 10, 21, 12, 59, 56));
   Serial.println("HI");
  rtcfunction();

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  //print_wakeup_reason();

  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
   
}

void loop(){
  //This is not going to be called
  rtcfunction();
   print_wakeup_reason();
   
}
