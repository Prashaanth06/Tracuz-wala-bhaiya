#include <soc/rtc.h>
#define BUTTON_PIN_BITMASK 0x200000000
extern "C" {
  #include <esp_clk.h>
}

RTC_DATA_ATTR uint64_t sleepTime;

void setup() {
  Serial.begin(115200);
  if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
  {
    uint64_t timeNow, timeDiff;
    timeNow = rtc_time_slowclk_to_us(rtc_time_get(), esp_clk_slowclk_cal_get());
    timeDiff = timeNow - sleepTime;
    printf("Now: %"PRIu64"ms, Duration: %"PRIu64"ms\n", timeNow / 1000, timeDiff / 1000);
    delay(2000);
  }
  sleepTime = rtc_time_slowclk_to_us(rtc_time_get(), esp_clk_slowclk_cal_get());
  printf("sleep_time: %"PRIu64"ms\n", sleepTime / 1000);
  printf("Sleeping...\n");
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);
  esp_deep_sleep_start();
}

void loop()
{}
