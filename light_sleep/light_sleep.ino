#include <sys/time.h>
#define BUTTON_PIN_BITMASK 0x200000000
RTC_DATA_ATTR timeval sleepTime;

void setup() {

  Serial.begin(115200);
  if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
  {
    timeval timeNow, timeDiff;
    gettimeofday(&timeNow, NULL);
    timersub(&timeNow,&sleepTime,&timeDiff);
    printf("Now: %"PRIu64"ms, Duration: %"PRIu64"ms\n", (timeNow.tv_sec * (uint64_t)1000) + (timeNow.tv_usec / 1000), (timeDiff.tv_sec * (uint64_t)1000) + (timeDiff.tv_usec / 1000));
    delay(2000);
  }
  gettimeofday(&sleepTime, NULL);
  printf("Sleeping...\n");
 esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);
  esp_deep_sleep_start();
}
void loop(){}
