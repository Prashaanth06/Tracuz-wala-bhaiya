#include <EEPROM.h>
//#define EEPROM_SIZE 1
#define FASTLED_ALLOW_INTERRUPTS 0
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex

#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLE2902.h>
#include <BLEAdvertisedDevice.h>
//#include <Adafruit_NeoPixel.h>
//#include <NeoPixelBus.h>
#include <FastLED.h> 
#include "LedControl.h"
#include <string>

#define powerButtonPin 33
#define ledButtonPin 26
#define clockButtonPin 27
#define rightButtonPin 5
#define leftButtonPin 14
#define zipPin 32
#define currentSensorPin 34
#define pollutionSensorPin 36
#define ldrPin 35
#define piezoPin 39
#define directionLedPowerPin 17
#define directionLedLeftPin 21
#define directionLedRightPin 22
#define circleLedPin 15

#define innerLedPin 12
#define buzzerPin 25
#define relayPin 16
const int RightMotorPin = 4;  // 13 corresponds to GPIO16
const int LeftMotorPin = 13;  // 14 corresponds to GPIO16
#define mqPowerPin 2

#define debounceDelayMax 1000
//
//#define SERVICE_UUID "94e709f1-4958-4b3a-a744-abcdc7e77449"
//#define CHARACTERISTIC_UUID_RX "2dd36d0f-4642-4f65-8bc1-bca629a6db06"
//#define CHARACTERISTIC_UUID_TX "d2c32143-9879-43bb-840b-052e0fc94f91"
//
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

//#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
//#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
//#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic_TX;
BLECharacteristic *pCharacteristic_RX;
BLEScan* pBLEScan;

LedControl lc = LedControl(23,18,19,1);

//Adafruit_NeoPixel left_strip = Adafruit_NeoPixel(3, directionLedLeftPin, NEO_GRB + NEO_KHZ800);
//Adafruit_NeoPixel right_strip = Adafruit_NeoPixel(3, directionLedRightPin, NEO_GRB + NEO_KHZ800);

//NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> left_strip(3, 21);
//NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> right_strip(3, 22);

CRGB left_leds[3];
CRGB right_leds[3];
CRGB circle_leds[12];

int secureModeCheck = 0;

int navigation_cmd = 0;
//Led Strip Patterns
const byte led_indicator_pattern[] = {255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,255,0,0,255,0,0,0,0,0,0,0,0,0,0,0};
const byte led_ring_pattern[] = {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255};

typedef struct ledFrameData {
  uint64_t ledFrames[22];
  int num_frames;
  int ledIntensity[22];
  int ledFrameDelay[22];
  int ledDisplayMode;
} ledFrameData;

ledFrameData primaryLedFrameData = {{0x0000000000000000}, 1, {8}, {100}, 0};
ledFrameData *primaryLedFramePtr = &primaryLedFrameData;

ledFrameData backupLedFrameData;

const uint64_t IMAGES[] = {
  0x70381c06061c3870,
  0x0e1c387070381c0e,
  0x0081c3e77e3c1800,
  0x5252525e5e521252,
  0x2810929254381010,
  0x7fa1a1e121120912,
  0x3c3c3c3c24243c18,
  0x00a412a412a412a4,
  0x0181c9c9ebefefff,
  0xff7e363614101000,
  0x0040a091110a0400,
  0x0727772707ffffff,  
  0x242418c3ffc31818,
  0x000042c3ffc34200,
  0x7e3e3e0204081020,
  0xff00307878300000,
  0x0000000000000000,
  0x003c425a5a423c00,
  0x242418dbffc31818,
  0x00183c7eff7e3c18,
  0x0408100804081020,
  0x24545424040a1100,
  0x1818181818001818,
  0x1028448282926c00,
  0x2424ff2424ff2424,
  0x281454583c021a18,
  0x482414987e190c0c,
  0x241414997e180c0c,
  0x241414997e180c0c,
  0x904828987e190c0c,
  0x07060c183061ffff,
  0x002020d708080800,
  0x0077777700777777,
  0x081a2c18182c1a08,
  0x1800009999000018,
  0x00104438827c0000,
  0x03030018180080c0,
  0x030100081020c0c0,
  0xffff7e3c99004a89,
  0x42a5a542183de600,
  0x003c424200666600,
  0x003c420000666600,
  0x7e7e7e7e24242418,
  0x8411001899008210,
  0x8142241818244281,
  0x181824242418815a,
  0x1818001818181818,
  0x995a24dbdb245a99,
  0x181818ffff181818,
  0x00183062c6ccf8f0,
  0x02060f193060c080,
  0x18314a864a311800,
  0x187e814242241818,
  0x3844221222443800,
  0x22361c3e7f1c0808,
  0x1899db7e3c181818,
  0x387c7c3814141400,
  0x181c3e3d3c040404,
  0x8484ff848494c4ff,
  0x000042c3c342423c,
  0x042c642424263420,
  0x0000018181800800,
  0xa4a4989afc809818,
  0x8040201f1f204080
};

const uint64_t coffee_battery_gif[] = {
  0xfebefe2210240800,
  0xfebefe2200141401,
  0xfebefe2204120803,
  0xfebefe2214140007,
  0xfebefe221024040f,
  0xfebefe220014281f,
  0xfebefe220412083f,
  0xfebefe220414107f,
  0xfebefe2200140aff
};

const uint64_t univ_gif[] = {
  0x0148001918800210,
  0x1000821818018009,
  0x20040018990000a4,
  0x9000029818010048
};

const uint64_t star_blink_gif[] = {
  0x8002100100440820,
  0x7ffdeffeffbbf7df
};

const uint64_t compass_gif[] = {
  0x00183062c6ccf8f0,
  0xf0f8ccc660301000,
  0x0f1f3363060c0800,
  0x00080c0663331f0f
};

const uint64_t gps_gif[] = {
  0x1800009999000018,
  0x0c00809819010030,
  0x0442801818014220,
  0x2042011818804204,
  0x300001199880000c
};

const uint64_t phone_charge_gif[] = {
  0x24424a524a52423c,
  0x244a524a5242423c,
  0x24424a524a52423c,
  0x2442424a524a523c
};

const uint64_t pingpong_gif[] = {
  0x0080808103010000,
  0x0000808081050100,
  0x0000008181810800,
  0x0000010181808010,
  0x0000018181802000,
  0x0001010180c08000,
  0x00000181a1800000,
  0x0101819080000000,
  0x0081898100000000,
  0x8185810000000000,
  0x0281818100000000,
  0x0381818000000000,
  0x0085818100000000,
  0x0080898101000000,
  0x0000809181010000,
  0x00008181a1000000,
  0x0000008081c10100,
  0x0000008181812000,
  0x0000808181010010,
  0x0080818101000800,
  0x0080808101050000,
  0x0000808183010000
};

const uint64_t plane_gif[] = {
  0x140808492a1c0808,
  0x00140808492a1c08,
  0x0000140808492a1c,
  0x000000140808492a,
  0x0800000014080849,
  0x0808000000140808,
  0x1c08080000001408,
  0x2a1c080800000014,
  0x492a1c0808000000,
  0x08492a1c08080000,
  0x0808492a1c080800
};

const uint64_t heart_gif[] = {
  0x00001824425a2400,
  0x00183c66c3db7e24,
  0x0018244281815a24,
  0x00183c66c3db7e24
};

const uint64_t weather_lightning_gif[] = {
  0x0000040804020408,
  0x0000040804020448,
  0x0002040884422448,
  0x0000204084422448,
  0x0010204080402448,
  0x0000000080402040,
  0x0000000000000040,
  0x0000000000020408
};

const uint64_t running_gif[] = {
  0x1068181a5c380c0c,
  0x3018183838180c0c
};

const uint64_t zip_gif[] = {
  0x000000ffff000000,
  0x0000807f7f800000,
  0x0080403f3f408000,
  0x8040201f1f204080,
  0x0080403f3f408000,
  0x0000807f7f800000
};

const uint64_t lock_gif[] = {
  0x7e5a5a427e242418,
  0x7e5a5a427e202418,
  0x7e5a5a427e202018,
  0x7e525a427e202010,
  0x7e5252427e202000,
  0x7e5242427e200000,
  0x7e4242427e000000,
  0x7e5242427e200000,
  0x7e5252427e202000,
  0x7e525a427e202010,
  0x7e5a5a427e202018,
  0x7e5a5a427e202418
};

const uint64_t music_gif[] = {
  0xfffffd6d64604000,
  0xffff6f6565242000,
  0xffffffae22220000,
  0xffffffde96920200
};

const uint64_t yo_gif[] = {
  0x38383c3c3c000000,
  0x38383c3e3c240000,
  0x38383c3e3c242400,
  0x38383c3e3d242424
};

const uint64_t radar_gif[] = {
  0x0002001818040201,
  0x0040001818101010,
  0x0002001818204080,
  0x004000f818000000,
  0x8040201818004000,
  0x0848081818000000,
  0x0102041818000200,
  0x0040001f18000000
};

const uint64_t smiley_gif[] = {
  0x003c420000666600,
  0x003c428100666600
};

const uint64_t sunrise_gif[] = {
  0xffff7e3c00000000,
  0xffff7e3c00812442,
  0xffff7e3c00810042
};

const uint64_t left_arrow_gif[] = {
  0x1c0e078181070e1c,
  0x0e0783c0c083070e,
  0x0783c16060c18307,
  0x83c1e03030e0c183,
  0xc1e070181870e0c1,
  0xe070380c0c3870e0,
  0x70381c06061c3870
};

const uint64_t right_arrow_gif[] = {
  0x3870e08181e07038,
  0x70e0c10303c1e070,
  0xe0c183060683c1e0,
  0xc183070c0c0783c1,
  0x83070e18180e0783,
  0x070e1c30301c0e07,
  0x0e1c386060381c0e
};

const uint64_t up_arrow_gif[] = {
  0x18000081c3e7663c,
  0x3c18000081c3e766,
  0x663c18000081c3e7,
  0xe7663c18000081c3,
  0xc3e7663c18000081,
  0x81c3e7663c180000,
  0x0081c3e7663c1800
};

const uint64_t logo_frame[] = {
  0x07060c183061ffff 
};

const uint64_t blank_frame[] = {
  0x0000000000000000
};

const uint64_t LED_ON[] = {
  0x00996624183c7e7e,
  0x00996600183c7e7e,
  0x00990000183c7e7e,
  0x00000000183c7e7e
};

const uint64_t LED_OFF[] = {
  0x18183c7e7e7e3c18,
  0x1818244242422418
};

const uint64_t Ride_Mode[] = {
  0x42a5423c58102000,
  0x844a8478b0204000,
  0x081408f020408000,
  0x102810e040800000,
  0x205020c080000000,
  0x40a0408000000000,
  0x8040800000000000,
  0x0080000000000000,
  0x0205020100010000,
  0x040a040305010200,
  0x081408070b020400,
  0x1028100f16040800,
  0x2152211e2c081000
};

const uint64_t Secure_Mode_ON[] = {
  0x7e5a5a427e000000,
  0x7e5a5a427e040000,
  0x7e5a5a427e040400,
  0x7e5a5a427e040408,
  0x7e5a5a427e040418,
  0x7e5a5a427e042418,
  0x7e5a5a427e242418
};

const uint64_t Secure_Mode_OFF[] = {
  0x7e5a5a427e242418,
  0x7e5a5a427e042418,
  0x7e5a5a427e040418,
  0x7e5a5a427e040408,
  0x7e5a5a427e040400,
  0x7e5a5a427e040000,
  0x7e5a5a427e000000
};

const uint64_t Boot[] = {
  0x0000001818000000,
  0x00003c3c3c3c0000,
  0x007e7e7e7e7e7e00,
  0xffffffffffffffff,
  0x007e7e7e7e7e7e00,
  0x00003c3c3c3c0000,
  0x0000001818000000
};

const uint64_t Time_1[] = {
  0x0000000818181808
};
const uint64_t Time_2[] = {
  0x0000000818284808
};
const uint64_t Time_3[] = {
  0x0000001828480808
};
const uint64_t Time_4[] = {
  0x0000000878080808
};
const uint64_t Time_5[] = {
  0x0040201808080808
};
const uint64_t Time_6[] = {
  0x0020100808080808
};
const uint64_t Time_7[] = {
  0x0010101808080808
};
const uint64_t Time_8[] = {
  0x0004081010101010
};
const uint64_t Time_9[] = {
  0x0000020418101010
};
const uint64_t Time_10[] = {
  0x000000001e101010
};
const uint64_t Time_11[] = {
  0x0000001814121010
};
const uint64_t Time_12[] = {
  0x0000001018141010
};

int gif_num_frames[] = {
  9,
  4,
  2,
  4,
  5,
  4,
  22,
  11,
  4,
  8,
  2,
  6,
  12,
  4,
  4,
  8,
  2,
  3,
  7,
  7,
  7,
  1,
  1,
  4,
  2,
  13,
  7,
  7,
  7,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1
};

const uint64_t *gifs[] = {
  coffee_battery_gif,
  univ_gif,
  star_blink_gif,
  compass_gif,
  gps_gif,
  phone_charge_gif,
  pingpong_gif,
  plane_gif,
  heart_gif,
  weather_lightning_gif,
  running_gif,
  zip_gif,
  lock_gif,
  music_gif,
  yo_gif,
  radar_gif,
  smiley_gif,
  sunrise_gif,
  left_arrow_gif,
  right_arrow_gif,
  up_arrow_gif,
  logo_frame,
  blank_frame,
  LED_ON,
  LED_OFF,
  Ride_Mode,
  Secure_Mode_ON,
  Secure_Mode_OFF,
  Boot,
  Time_1,
  Time_2,
  Time_3,
  Time_4,
  Time_5,
  Time_6,
  Time_7,
  Time_8,
  Time_9,
  Time_10,
  Time_11,
  Time_12,
};

SemaphoreHandle_t xLedFrameDataUpdateSemaphore;
SemaphoreHandle_t xInnerIlluminationUpdateSemaphore;
SemaphoreHandle_t xBLEAddressesUpdateSemaphore;
SemaphoreHandle_t xUpdateModeSemaphore;
SemaphoreHandle_t xSecureModeToggle;

SemaphoreHandle_t xRideModeStartSemaphore;
SemaphoreHandle_t xRideModeEndSemaphore;
SemaphoreHandle_t xLeftToggleSemaphore;
SemaphoreHandle_t xRightToggleSemaphore;
SemaphoreHandle_t xStraightToggleSemaphore;
SemaphoreHandle_t xClearToggleSemaphore;

SemaphoreHandle_t xNotificationsToggle;
SemaphoreHandle_t xNotificationsUpdate;

TaskHandle_t xSecureModeTaskHandle = NULL;
TaskHandle_t xRideModeTaskHandle = NULL;

String beaconAddresses[20] = {};
int numBeacons = 0;

int system_mode = 0;
int inner_illumination = 0;
bool secure_mode = false;
int alarm_mode = 0;
int zipStateGlobal_button=0;
//Settings
String direction_led_color = "FF0000";
int missing_item_notif[3] = {1, 1, 1};
int secure_mode_notif[3] = {1, 1, 1};
int power_notif[3] = {1, 1, 1};
int ble_scan_interval = 5;
int schedule_mode = 0;
int vibration_intensity = 127;
int zipStateGlobal = 0;
int inner_illumination_state=1;
int buttonInterrupt=0;

int buzzerDisable = 0;
int zipCloseOn=0;

//Power Settings
int charge_mode = 0;
int charge_capacity = 0;

// MQ Settings
unsigned long old_time_MQ= millis();
unsigned long new_time_MQ=millis();
float mqReading=0; 

//SecureMode Notification
int secure_notification = 0;

char characters[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'}; 
const uint64_t ledMatrixCharacters[] = {
  0x6666667e66663c00,
  0x3e66663e66663e00,
  0x3c66060606663c00,
  0x3e66666666663e00,
  0x7e06063e06067e00,
  0x0606063e06067e00,
  0x3c66760606663c00,
  0x6666667e66666600,
  0x3c18181818183c00,
  0x1c36363030307800,
  0x66361e0e1e366600,
  0x7e06060606060600,
  0xc6c6c6d6feeec600,
  0xc6c6e6f6decec600,
  0x3c66666666663c00,
  0x06063e6666663e00,
  0x603c766666663c00,
  0x66361e3e66663e00,
  0x3c66603c06663c00,
  0x18181818185a7e00,
  0x7c66666666666600,
  0x183c666666666600,
  0xc6eefed6c6c6c600,
  0xc6c66c386cc6c600,
  0x1818183c66666600,
  0x7e060c1830607e00,
  0x7c667c603c000000,
  0x3e66663e06060600,
  0x3c6606663c000000,
  0x7c66667c60606000,
  0x3c067e663c000000,
  0x0c0c3e0c0c6c3800,
  0x3c607c66667c0000,
  0x6666663e06060600,
  0x3c18181800180000,
  0x1c36363030003000,
  0x66361e3666060600,
  0x1818181818181800,
  0xd6d6feeec6000000,
  0x6666667e3e000000,
  0x3c6666663c000000,
  0x06063e66663e0000,
  0xf0b03c36363c0000,
  0x060666663e000000,
  0x3e403c027c000000,
  0x1818187e18180000,
  0x7c66666666000000,
  0x183c666600000000,
  0x7cd6d6d6c6000000,
  0x663c183c66000000,
  0x3c607c6666000000,
  0x3c0c18303c000000,
  0x7e1818181c181800,
  0x7e060c3060663c00,
  0x3c66603860663c00,
  0x30307e3234383000,
  0x3c6660603e067e00,
  0x3c66663e06663c00,
  0x1818183030667e00,
  0x3c66663c66663c00,
  0x3c66607c66663c00,
  0x3c66666e76663c00
};

const int freq = 5000;
const int LeftMotorChannel = 0;
const int RightMotorChannel = 1;
const int buzzerChannel = 2;
const int resolution = 8;

bool deviceConnected = false;



const size_t capacity = JSON_ARRAY_SIZE(10) + JSON_OBJECT_SIZE(5) + 280;

DynamicJsonDocument doc(2048);

void updateLedMatrixFrame(int num_frames, int display_mode, JsonArray frame_delay, JsonArray led_frames_str, JsonArray intensities);
void updateLedMatrixFrameFromString(const char* text, int intensity, int frame_delay, int num_chars);
void updateLedMatrixFramePredefined(int animation_id, bool backup, int display_mode_doc);
void updateSystemMode(int system_mode_doc);
void updateInnerIllumination(int inner_illumination_doc);
void updateSecureMode(int secure_mode_doc);
void updateDeepSleep(int State);
void updateBLEAddresses(int num_beacons_doc, JsonArray addresses);
void updateSettings(const char* direction_led_color_doc, JsonArray missing_item_notif_doc, JsonArray secure_mode_notif_doc, JsonArray power_notif_doc, int ble_scan_interval_doc, int schedule_mode_doc, int vibration_intensity_doc);
void updatePowerSettings(int charge_mode_doc, int charge_capacity_doc);
void disableAlarm(int alarm_toggle);
void updateNavigation(int navigation);

class MyCallbacks: public BLECharacteristicCallbacks {
  
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {

      char* jsonString = (char*)rxValue.c_str();

      Serial.print("BLE RX DATA: ");
      Serial.println(jsonString);

      DeserializationError error = deserializeJson(doc, jsonString);

      if(error){
        Serial.print("Deserialization error: ");
        Serial.println(error.c_str());
      }

      int command_id = doc["command_id"];

      switch(command_id){
      
        case 0:
          updateLedMatrixFramePredefined(doc["animation_id"], false, doc["display_mode"]);
          break;
        case 1:
          updateLedMatrixFrame(doc["num_frames"], doc["display_mode"], doc["frame_delay"], doc["led_frames"], doc["intensities"]);
          break;
        case 2:
          updateLedMatrixFrameFromString(doc["text"], doc["intensity"], doc["frame_delay"], doc["num_chars"]);
          break;
        case 3:
          updateSystemMode(doc["mode"]);
          break;
        case 4:
          updateInnerIllumination(doc["inner_illumination"]);
          break;
        case 5:
          updateSecureMode(doc["secure_mode"]);
          break;
        case 6:
          updateDeepSleep(doc["State"]);
          break;
        case 7:
          updateBLEAddresses(doc["num_beacons"], doc["addresses"]);
          break;
        case 8:
          updateSettings(doc["direction_led_color"], doc["missing_item_notif"], doc["secure_mode_notif"], doc["power_notif"], doc["ble_scan_interval"], doc["schedule_mode"], doc["vibration_intensity"]);
          break;
        case 9:
          updatePowerSettings(doc["charge_mode"], doc["charge_capacity"]);     
          break;
        case 10:
          disableAlarm(doc["data"]);
          break;
        case 16:
          updateNavigation(doc["navigation"]);
          break;
      }
    }
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    updateSecureMode(1);
    ledcWrite(LeftMotorChannel, vibration_intensity);
    ledcWrite(RightMotorChannel, vibration_intensity);
    digitalWrite(buzzerPin, HIGH);
    vTaskDelay(5000);
    ledcWrite(LeftMotorChannel, 0);
    ledcWrite(RightMotorChannel, 0);
    digitalWrite(buzzerPin, LOW);
    
  }
};

void setup() {

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1);
  
  pinMode(powerButtonPin, INPUT);
  pinMode(ledButtonPin, INPUT);
  pinMode(clockButtonPin, INPUT);
  pinMode(rightButtonPin, INPUT);
  pinMode(leftButtonPin, INPUT);
  pinMode(zipPin, INPUT);
  pinMode(currentSensorPin, INPUT);
  pinMode(pollutionSensorPin, INPUT);
  pinMode(ldrPin, INPUT);
  pinMode(piezoPin, INPUT);
  digitalWrite(directionLedPowerPin,HIGH);

  pinMode(innerLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
//  pinMode(leftMotorPin, OUTPUT);
//  pinMode(rightMotorPin, OUTPUT);
  
  pinMode(mqPowerPin, OUTPUT);
  pinMode(directionLedPowerPin, OUTPUT);
  pinMode(directionLedRightPin, OUTPUT);
  pinMode(directionLedLeftPin, OUTPUT);
//  digitalWrite(mqPowerPin, HIGH);
  ledcSetup(LeftMotorChannel, freq, resolution);
  ledcSetup(RightMotorChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(LeftMotorPin, LeftMotorChannel);
  ledcAttachPin(RightMotorPin, RightMotorChannel);
  
  lc.shutdown(0,false);
  lc.setIntensity(0,8);
  lc.clearDisplay(0);

  

  Serial.begin(112500);
  EEPROM.begin(512);

//  right_strip.Begin();
//  right_strip.Show();
//  left_strip.Begin();
//  left_strip.Show();

  FastLED.addLeds<WS2812B,21,RGB>(left_leds,3);
  FastLED.addLeds<WS2812B,22,RGB>(right_leds,3);
  FastLED.addLeds<WS2812B,15,RGB>(circle_leds,12);

//  ledcSetup(0, 5000, 8);
//  ledcSetup(1, 5000, 8);
//
//  ledcAttachPin(leftMotorPin, 0);
//  ledcAttachPin(rightMotorPin, 1);


  BLEDevice::init("TRACUZ_BAG");
  BLEDevice::setMTU(512);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic_TX = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic_RX = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic_TX->addDescriptor(new BLE2902());

  pCharacteristic_RX->setCallbacks(new MyCallbacks());

  pService->start();
  
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  pBLEScan = BLEDevice::getScan();

   secure_mode = EEPROM.read(0);
  inner_illumination = EEPROM.read(1);
   
    for(int i=0; i<12; i++){
     circle_leds[i] = CRGB(255, 255, 255);
     FastLED.show();
     delay(10);
    }
   
              
             // break;
  //digitalWrite(innerLedPin, EEPROM.read(1));
    
//  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  xTaskCreate(displayLedMatrixFrameTask, "Led Matrix Display Task", 20000, NULL, 2, NULL);
  xTaskCreate(innerIlluminationTask, "Inner Illumination Control Task", 1000, NULL, 2, NULL);
  xTaskCreate(updateModeTask, "Updating Mode Task", 1000, NULL, 2, NULL);
  xTaskCreate(rideModeTask, "Ride Mode Task", 1000, NULL, 2, &xRideModeTaskHandle);
  xTaskCreate(secureModeTask, "Secure Mode Task", 1000, NULL, 2, NULL);
  xTaskCreate(notificationTask, "Notification Task", 1000, NULL, 2, NULL);
  xTaskCreate(ButtonPollingTask, "Power Button Control Task", 2000, NULL, 2, NULL);
  xTaskCreate(bleScannerTask, "BLE Scanner Task", 10000, NULL, 2, NULL);

if (inner_illumination == 0){
          updateInnerIllumination(0);
      }
  else {
          updateInnerIllumination(1);
       }
boot();
}

void loop() {
  if(secure_notification == 1)
  {
   secureModeNotification();  
  }
  if(digitalRead(zipPin)== LOW && zipStateGlobal==0 && secureModeCheck == 0)
  {
    zipStateOpen(1);
    zipCloseOn=1;
  }
  if(digitalRead(zipPin)== HIGH && zipStateGlobal_button == 0 && zipCloseOn==1)
  { 
    zipStateOpen(0);
    zipStateGlobal=0;
    buttonInterrupt=0;
    inner_illumination_state=1;
    zipCloseOn=0;
  }
  if(new_time_MQ - old_time_MQ > 10000)
  {
    mqPower(1);
  }
  if(new_time_MQ - old_time_MQ > 30000)
  {
    mqRead();
  }
  new_time_MQ=millis();
}

void updateLedMatrixFrame(int num_frames, int display_mode, JsonArray frame_delay, JsonArray led_frames_str, JsonArray intensities){

  int i;
  uint64_t led_frames[22];

  for(i = 0; i < num_frames; i++){
    primaryLedFrameData.ledFrames[i] = strtoull(led_frames_str[i], NULL, 0);
    primaryLedFrameData.ledFrameDelay[i] = frame_delay[i];
    primaryLedFrameData.ledIntensity[i] = intensities[i];
  }

  for(i = num_frames; i <= 21; i++){
    primaryLedFrameData.ledFrames[i] = strtoull("0x0000000000000000", NULL, 0);
  }

  primaryLedFrameData.ledDisplayMode = display_mode;
  primaryLedFrameData.num_frames = num_frames;

  if(xSemaphoreGive(xLedFrameDataUpdateSemaphore) == pdTRUE){
    Serial.println("LED MATRIX: Global Data Updated Successfully (FRAME METHOD)");
  }
}

void displayLedMatrixFrameTask(void * pvParameters){
  xLedFrameDataUpdateSemaphore = xSemaphoreCreateBinary();
  
  ledFrameData localLedFrameData;
  localLedFrameData = primaryLedFrameData;

  uint64_t blank_frame = 0x0000000000000000;

  int i;
  int frame_index = 0;
  int shift_index = 0;

  Serial.println("LED MATRIX: Task Initialized...");

  while(1){
    //Semaphore Handler
    if(xSemaphoreTake(xLedFrameDataUpdateSemaphore, 0) == pdTRUE){
      localLedFrameData = primaryLedFrameData;
      frame_index = 0;
      shift_index = 0;
      Serial.println("LED MATRIX: Local Variable Updated Successfully");
    }
    
    //Display Function
    if(localLedFrameData.ledDisplayMode == 0){
      set_display_uint64(localLedFrameData.ledFrames[frame_index], localLedFrameData.ledIntensity[frame_index]);
    }
    else{
      if(frame_index == localLedFrameData.num_frames - 1){
        set_display_uint64_scroll(localLedFrameData.ledFrames[frame_index], blank_frame, shift_index);
      }
      else{
        set_display_uint64_scroll(localLedFrameData.ledFrames[frame_index], localLedFrameData.ledFrames[frame_index + 1], shift_index);
      }
    }
    
    vTaskDelay(localLedFrameData.ledFrameDelay[frame_index]);

    if(localLedFrameData.ledDisplayMode == 0){
      frame_index++;
    }
    else{
      shift_index++;
      if(shift_index >= 8){
        shift_index = 0;
        frame_index++;
      }
    }
    
    if(frame_index >= localLedFrameData.num_frames){
      frame_index = 0;
    }

    vTaskDelay(1);
  }
}

void set_display_uint64(uint64_t led_frame, int intensity){

  int i = 0;
  uint8_t temp_row;

  lc.setIntensity(0, intensity);

  for (int i = 0; i < 8; i++){
    byte row = (led_frame >> i * 8) & 0xFF;
    for (int j = 0; j < 8; j++){
      lc.setLed(0, i, j, bitRead(row, j));
    }
  }
  
}

void set_display_uint64_scroll(uint64_t led_frame_1, uint64_t led_frame_2, int shift_index){

  int i,j;
  
  byte row_1, row_2, row;
  uint64_t joined = 0X0000000000000000;

  for (i = 0; i < 8; i++){

    row_1 = (led_frame_1 >> i * 8) & 0xFF;
    row_2 = (led_frame_2 >> i * 8) & 0xFF;

    row_1 = (row_1 >> shift_index) & 0xFF;
    row_2 = (row_2 << (8 - shift_index)) & 0xFF;

    row = row_1 | row_2;

    for (j = 0; j < 8; j++){
      lc.setLed(0, i, j, bitRead(row, j));
    }
  }
}


void updateLedMatrixFrameFromString(const char* text, int intensity, int frame_delay, int num_chars){

  int i;

  
  for(i = 0; i < num_chars; i++){
    
    primaryLedFrameData.ledFrames[i] = ledMatrixCharacters[getCharacterIndex(text[i])];
    primaryLedFrameData.ledIntensity[i] = intensity;
    primaryLedFrameData.ledFrameDelay[i] = frame_delay;
    
  }

  primaryLedFrameData.num_frames = num_chars;
  primaryLedFrameData.ledDisplayMode = 1;

  if(xSemaphoreGive(xLedFrameDataUpdateSemaphore) == pdTRUE){
    Serial.println("LED MATRIX: Global Data Updated Successfully (TEXT METHOD)");
  }
}

int getCharacterIndex(char c){
  
  int i = 0;
  
  for(i = 0; i < (sizeof(characters)/sizeof(characters[0])); i++){
    if(c == characters[i]){
      return i;
    }
  }
  
  return 0;
}

void updateLedMatrixFramePredefined(int animation_id, bool backup, int display_mode_doc){

  int num_frames = gif_num_frames[animation_id];

  if (backup == true){

//    backupLedFrameData.num_frames = primaryLedFrameData.num_frames;
//    backupLedFrameData.ledDisplayMode = primaryLedFrameData.ledDisplayMode;
//
//    for (int i = 0; i < primaryLedFrameData.num_frames; i++){
//      backupLedFrameData.ledFrames[i] = primaryLedFrameData.ledFrames[i];
//      backupLedFrameData.ledIntensity[i] = primaryLedFrameData.ledIntensity[i];
//      backupLedFrameData.ledFrameDelay[i] = primaryLedFrameData.ledFrameDelay[i];
//    }
    
    backupLedFrameData = primaryLedFrameData;
  }

  for(int i = 0; i < num_frames; i++){
    primaryLedFrameData.ledFrames[i] = gifs[animation_id][i];
    primaryLedFrameData.ledIntensity[i] = 8;
    primaryLedFrameData.ledFrameDelay[i] = 100;
  }

  primaryLedFrameData.num_frames = num_frames;
  primaryLedFrameData.ledDisplayMode = display_mode_doc;

  if(xSemaphoreGive(xLedFrameDataUpdateSemaphore) == pdTRUE){
    Serial.println("LED MATRIX: Global Data Updated Successfully (PREDEFINED METHOD)");
  }  
}

void restoreLedMatrixFrameBackup(){
  primaryLedFrameData = backupLedFrameData;

  if(xSemaphoreGive(xLedFrameDataUpdateSemaphore) == pdTRUE){
    Serial.println("LED MATRIX: Backup Restored");
  }  
}

void updateInnerIllumination(int inner_illumination_doc){

   
   
  if (inner_illumination_doc != inner_illumination){
    if (inner_illumination_doc == 0){
      updateAnimationTemp(23 , 400);
    }
    else if (inner_illumination_doc == 1){
      updateAnimationTemp(24, 500);
    }
  }
  
  inner_illumination = inner_illumination_doc;
  
  if(xSemaphoreGive(xInnerIlluminationUpdateSemaphore) == pdTRUE){
    Serial.println("INNER LED: Global Variable Updated Successfully");
  }
}

void innerIlluminationTask(void * pvParameters){
  
  xInnerIlluminationUpdateSemaphore = xSemaphoreCreateBinary();

  while(1){
    xSemaphoreTake(xInnerIlluminationUpdateSemaphore, portMAX_DELAY);
    digitalWrite(innerLedPin, inner_illumination);
    Serial.print("INNER LED: State changed successfully to: ");
    Serial.println(inner_illumination);
    EEPROM.write(1, inner_illumination);
    EEPROM.commit();
    Serial.println("Innerillumination saved in flash memory");
  }

  
}

void updateSecureMode(int secure_mode_doc){
  secure_mode = secure_mode_doc;
 
    EEPROM.write(0, secure_mode);
    EEPROM.commit();
    Serial.println("Secure mode saved in flash memory");

  if(secure_mode == 1){
    if (system_mode == 2){
      updateSystemMode(0);
    }
    if(xSemaphoreGive(xSecureModeToggle) == pdTRUE){
      Serial.println("SECURE MODE: ENABLE");
      updateAnimationTemp(26, 700);
      secureModeCheck = 1;
      updateLedMatrixFramePredefined(21, false, 0);
    }
  }
  else if(secure_mode == 0){
    if(xSemaphoreGive(xSecureModeToggle) == pdTRUE){
      Serial.println("SECURE MODE:  DISABLE");
      updateAnimationTemp(27, 700);
      secureModeCheck = 0;
      updateLedMatrixFramePredefined(21, false, 0);
    }
  }
}

void secureModeTask(void * pvParameters){

  xSecureModeToggle = xSemaphoreCreateBinary();
  
  int zipState = 1;
  int prevZipState = 1;

  bool secure_mode_local = false;

  while(1){

    if (secure_mode_local == true){
      if(xSemaphoreTake(xSecureModeToggle, 0 == pdTRUE)){
        secure_mode_local = secure_mode;
      }

      if (!secure_mode_local){
        alarm_mode = 0;
        if(xSemaphoreGive(xNotificationsToggle) == pdTRUE){
          Serial.println("NOTIFICATION TOGGLE: DISABLE (SECURE MODE) lol");
        }
        continue;
      }
      
    } else {
      xSemaphoreTake(xSecureModeToggle, portMAX_DELAY);

      secure_mode_local = secure_mode;

      if (!secure_mode_local){
        continue;
      }
      
    }

    zipState = digitalRead(zipPin);

    Serial.print("Zip State: ");
    Serial.println(zipState);
    
    if(zipState != prevZipState){
      
      if(zipState == 0){
        alarm_mode = 2;
        secure_notification = 1;
        if(xSemaphoreGive(xNotificationsToggle) == pdTRUE){
          Serial.println("NOTIFICATION TOGGLE: ENABLE (SECURE MODE)");
        }
      }
      else if(zipState == 1){
        alarm_mode = 0;
        secure_notification = 0;
        updateLedMatrixFramePredefined(21, false, 0);
        if(xSemaphoreGive(xNotificationsToggle) == pdTRUE){
          Serial.println("NOTIFICATION TOGGLE: DISABLE (SECURE MODE)");
        }
      } 
    }

    prevZipState = zipState;

    vTaskDelay(1000);
  }
}

void updateBLEAddresses(int num_beacons_doc, JsonArray addresses){

  numBeacons = num_beacons_doc;

  for(int i = 0; i < numBeacons; i++){
    beaconAddresses[i] = addresses[i].as<String>();
  }

  if(xSemaphoreGive(xBLEAddressesUpdateSemaphore) == pdTRUE){
    updateAddressEeprom();
    Serial.println("BLE Addresses Updated Successfully");
  }
  
}

void bleScannerTask(void * pvParameters){

  xBLEAddressesUpdateSemaphore = xSemaphoreCreateBinary();
  
  int deviceCount;
  int i,j;
  bool beaconFound;

  BLEAdvertisedDevice device;
  BLEAddress *deviceAddress;

  String localBeaconAddresses[20] = beaconAddresses;
  int localNumBeacons = numBeacons;

  bool beaconsFoundStatus[20];

  StaticJsonDocument<128> doc;
  String output;

  for (i = 0; i < 20; i++){
    beaconsFoundStatus[i] = true;
  }

  while(1){

    if(xSemaphoreTake(xBLEAddressesUpdateSemaphore, 0) == pdTRUE){
      
      localNumBeacons = numBeacons;
      for(i = 0; i < localNumBeacons; i++){
        localBeaconAddresses[i] = beaconAddresses[i];
      }      
      Serial.println("Local BLE Address Data Updated");
    }
    
    BLEScanResults foundDevices = pBLEScan->start(ble_scan_interval);
    deviceCount = foundDevices.getCount();

    for (i = 0; i < localNumBeacons; i++){
      beaconFound = false;
      
      for (j = 0; j < deviceCount; j++){
        device = foundDevices.getDevice(j);
        deviceAddress = new BLEAddress(device.getAddress());

        if(strcmp(deviceAddress->toString().c_str(), localBeaconAddresses[i].c_str()) == 0){
          beaconFound = true;
        }
      }

      beaconsFoundStatus[i] = beaconFound;
      
    }

    for(i = 0; i < localNumBeacons; i++){
      if(beaconsFoundStatus[i] == false){
        Serial.println("Device Missing:");
        Serial.println(beaconAddresses[i]);
        
        doc["command_id"] = 11;
        doc["address"] = beaconAddresses[i];
      //  doc["name"] = "wallet";

        serializeJson(doc, output);
        
        pCharacteristic_TX->setValue(output.c_str());
        pCharacteristic_TX->notify();
        
      }
      else {
        Serial.println("Device Found:");
        Serial.println(beaconAddresses[i]);
      }
    }
  }
}

void updateSettings(const char* direction_led_color_doc, JsonArray missing_item_notif_doc, JsonArray secure_mode_notif_doc, JsonArray power_notif_doc, int ble_scan_interval_doc, int schedule_mode_doc, int vibration_intensity_doc){

  Serial.println("Old System Settings: ");
  Serial.println(direction_led_color);
  
  for (int i = 0; i < 3; i++){
    Serial.print("First Index Notifs");
    Serial.print(missing_item_notif[i]);
    Serial.print(secure_mode_notif[i]);
    Serial.println(power_notif[i]);
  }

  Serial.print("BLE Scan Interval: ");
  Serial.println(ble_scan_interval);

  Serial.print("Schedule Mode: ");
  Serial.println(schedule_mode);
  
  direction_led_color = direction_led_color_doc;

  for(int i = 0; i < 3; i++){
    
    missing_item_notif[i] = missing_item_notif_doc[i];
    secure_mode_notif[i] = secure_mode_notif_doc[i];
    power_notif[i] = power_notif_doc[i];
    
  }  
  
  ble_scan_interval = ble_scan_interval_doc;
  schedule_mode =  schedule_mode_doc;
  vibration_intensity = vibration_intensity_doc;

  Serial.println("New System Settings: ");
  Serial.println(direction_led_color);
  
  for (int i = 0; i < 3; i++){
    Serial.print("First Index Notifs");
    Serial.print(missing_item_notif[i]);
    Serial.print(secure_mode_notif[i]);
    Serial.println(power_notif[i]);
  }

  Serial.print("BLE Scan Interval: ");
  Serial.println(ble_scan_interval);

  Serial.print("Schedule Mode: ");
  Serial.println(schedule_mode);
  
}

void updatePowerSettings(int charge_mode_doc, int charge_capacity_doc){

  charge_mode = charge_mode_doc;
  charge_capacity = charge_capacity_doc;
  
}

void updateRelaySwitch()
{    
    unsigned long old_time= millis();
    unsigned long new_time=millis();
  while(new_time - old_time < 1000)
  {  
    digitalWrite(relayPin, HIGH);
    Serial.print("Relay Triggered");
    new_time=millis();
  }
    digitalWrite(relayPin, LOW);
}

void ButtonPollingTask(void * pvParameters){

  int i;
  uint8_t a = 24;
  int animCycle = 0;
  int anim_id = 0;

  int buttonStates[5]     = {0,0,0,0,0};
  int prevButtonStates[5] = {0,0,0,0,0};
  int buttonReadings[5]   = {0,0,0,0,0};
  int timingFlags[5]      = {0,0,0,0,0};
  int timingStates[5]     = {0,0,0,0,0};

  int buttonPins[5]       = {33, 26, 27, 5, 14};

  unsigned long microsTimes[5];
  unsigned long millisTimes[5];

  for (i = 0; i < 5; i++){
    microsTimes[i] = micros();
    millisTimes[i] = millis();
  }
  
  Serial.println("Button Polling Task Initialized");

  while(1){

   vTaskDelay(20); 

    for (i = 0; i < 5; i++){
      buttonReadings[i] = digitalRead(buttonPins[i]);

      if(buttonReadings[i] != prevButtonStates[i]){
        microsTimes[i] = micros();
      }

      if(micros() - microsTimes[i] >= debounceDelayMax){
        if(buttonStates[i] != buttonReadings[i]){
          buttonStates[i] = buttonReadings[i];
        }
      }

      if(buttonStates[i] == HIGH){
        if(timingFlags[i] == 0){
          millisTimes[i] = millis();
          timingFlags[i] = 1;
        }
      }

      if(buttonStates[i] == LOW){
        if(timingFlags[i] == 1){
          
          if(millis() - millisTimes[i] >= 2000){
            timingStates[i] = 1;
          }
          if(millis() - millisTimes[i] >= 3000){
            timingStates[i] = 2;
          }

          switch(i){
            case 0:
              Serial.print("PowerButtonState: ");
              Serial.println(timingStates[0]);

              switch(timingStates[0]){
                case 0:
                  if (secure_mode == 0){
                    updateSecureMode(1);
                    buzzerDisable=0;
                  } else {
                    updateSecureMode(0);
                  }
                  break;
                case 1:
                  updateDeepSleep(1);
                  break;
                case 2:
                  updateDeepSleep(1);
                  break;
              }
              
              break;
            
            case 1:
              Serial.print("LedButtonState: ");
              Serial.println(timingStates[1]);

//
//              if (system_mode == 2){
//                updateSystemMode(0);
//              }
          if(timingStates[1]==0)
          {
              if (inner_illumination == 0){
                updateInnerIllumination(1);
                zipStateGlobal_button=1;
              } else {
                updateInnerIllumination(0);
                zipStateGlobal_button=0;
                inner_illumination_state=0;
                zipStateGlobal=1;
                buttonInterrupt=0;
                ledTimer(0);
                delay(100);
              }
          }
              if(timingStates[1]==1 || timingStates[1]==2)
              {
                updateRelaySwitch();
                }
              break;
            
            case 2:
              Serial.print("ClockButtonState: ");
              Serial.println(timingStates[2]);

              switch(timingStates[2]){
                case 0:
                
                  if (animCycle == 1){
                    anim_id = anim_id + 1;
                    if (anim_id == 22){
                      anim_id = 0;
                    }
                    Serial.print("Animation Cycled. New animation id: ");
                    Serial.println(anim_id);
                    updateLedMatrixFramePredefined(anim_id, false, 0);
                  }
                  else {
                    updateSystemMode(2);
                    buzzerDisable=1;
                  }
                  break;
                  
                case 1:
                
                  if (animCycle == 1){
                    animCycle = 0;
                    anim_id = 0;
                    Serial.println("Animation Cycling Mode Deactivated");
                    updateAnimationTemp(4, 1000);
                    vTaskDelay(1200);
                    updateLedMatrixFramePredefined(0, false, 0);
                  }
                  else if (animCycle == 0){
                    animCycle = 1;
                    Serial.println("Animation Cycling Mode Activated");
                    updateAnimationTemp(4, 1000);
                  }
                  break;
                  
                case 2:
                
                  if (animCycle == 1){
                    animCycle = 0;
                    anim_id = 0;
                    Serial.println("Animation Cycling Mode Deactivated");
                    updateAnimationTemp(4, 1000);
                    vTaskDelay(1200);
                    updateLedMatrixFramePredefined(0, false, 0);
                  }
                  else if (animCycle == 0){
                    animCycle = 1;
                    Serial.println("Animation Cycling Mode Activated");
                    updateAnimationTemp(4, 1000);
                  }
                  break;
              }            
              break;
           
            case 3:
              Serial.print("RightButtonState: ");
              Serial.println(timingStates[3]);
              
              if ((timingStates[3] == 1) || (timingStates[3] == 2)){

                if(system_mode != 3){
                  updateAnimationTemp(25, 900);
                  system_mode = 3;
                  Serial.println("Ride Mode Enable: Global Variable Updated");
                }
                else{
                  system_mode = 0;
                  Serial.println("Ride Mode Disable: Global Variable Updated");
                }

                if(xSemaphoreGive(xUpdateModeSemaphore) == pdTRUE){
                  Serial.println("System Mode Updated");
                }
                
              }
              
              else {
                
                if (system_mode == 3){
                  navigation_cmd = 0;
                  if(xSemaphoreGive(xRightToggleSemaphore) == pdTRUE){
                    Serial.println("Right Toggle Signal Given");
                  }
                }
              }
              
              break;
           
            case 4:
              Serial.print("LeftButtonState: ");
              Serial.println(timingStates[4]);
              
              if (system_mode == 3){
                if ((timingStates[4] == 1) || (timingStates[4] == 2)){
                  navigation_cmd = 0;
                  if(xSemaphoreGive(xStraightToggleSemaphore) == pdTRUE){
                    Serial.println("Straight Toggle Signal Given");
                  }
                }
                else {
                  navigation_cmd = 0;
                  if (xSemaphoreGive(xLeftToggleSemaphore) == pdTRUE){
                    Serial.println("Left Toggle Signal Given");
                  }
                }
              }

              if(timingStates[4] == 0)
              {

                }
              
              break;
          }          
        }

        timingStates[i] = 0;
        timingFlags[i] = 0;
        
      }

      prevButtonStates[i] = buttonReadings[i];
      
    }
  }
}

void updateSystemMode(int system_mode_doc){
  system_mode = system_mode_doc;
  
  if (xSemaphoreGive(xUpdateModeSemaphore) == pdTRUE){
    Serial.println("System Mode Updated");
  }
}

void updateModeTask(void * pvParameters){

  int system_mode_local;
  system_mode_local = system_mode;

  xUpdateModeSemaphore = xSemaphoreCreateBinary();

  Serial.println("UpdateModeTask Initialized");

  while(1){
    xSemaphoreTake(xUpdateModeSemaphore, portMAX_DELAY);

    if (system_mode != 2){
      if(system_mode == system_mode_local){
        continue;
      }
    }    
    
    if (system_mode_local == 3){
      if (system_mode != 3){
        if (xSemaphoreGive(xRideModeEndSemaphore) == pdTRUE){
          restoreLedMatrixFrameBackup();
          Serial.println("Signal to End Ride Mode Given");
        }
      }
    }

    if (system_mode_local == 2){
      if (system_mode != 2){
        lc.shutdown(0,false);
      }
    }
    
    system_mode_local = system_mode;

    switch(system_mode_local){
      
      case 0:
        Serial.println("No Mode Selected");
        break;
      case 1:
        Serial.println("Walk Mode Selected");
        break;
      case 2:
        Serial.println("Silent Mode Selected");
        silentMode();
        break;
      case 3:
        Serial.println("Ride Mode Selected");
        if(xSemaphoreGive(xRideModeStartSemaphore) == pdTRUE){
          Serial.println("Signal to Start Ride Mode Given");
          updateLedMatrixFramePredefined(20, true, 0);
        }
        break;
      case 4:
        Serial.println("Clock Mode Selected");
        break;
      case 5:
        Serial.println("Music Mode Selected");
        break;
      case 6:
        Serial.println("Relax Mode Selected");
        break;
      
    }
  }  
}

void rideModeTask(void * pvParamters){


  xRideModeStartSemaphore = xSemaphoreCreateBinary();
  xRideModeEndSemaphore = xSemaphoreCreateBinary();
  
  xLeftToggleSemaphore = xSemaphoreCreateBinary();
  xRightToggleSemaphore = xSemaphoreCreateBinary();
  xStraightToggleSemaphore = xSemaphoreCreateBinary();
  xClearToggleSemaphore = xSemaphoreCreateBinary();

  bool left_state, right_state;
  left_state = right_state = false;

  int left_anim_state = 0;
  int right_anim_state = 0;
  int i, pixel_index, index;
  
  bool vibration_toggle;
  bool vibration_state;

  unsigned long left_millis;
  unsigned long right_millis;

  left_millis = millis();
  right_millis = millis();

  Serial.println("Ride Mode Task Initialized");

  if (system_mode != 3){
    if(xSemaphoreGive(xRideModeEndSemaphore) == pdTRUE){
      Serial.println("Ride Mode Task Correctly Initialized");
    }
  }

  while(1){
    
    if(xSemaphoreTake(xRideModeEndSemaphore, 0) == pdTRUE){

      left_state = right_state = 0;

      for (i = 0; i < 3; i++){
        left_leds[i] = CRGB::Black;
        right_leds[i] = CRGB::Black;
      }

//      digitalWrite(directionLedPowerPin, LOW);
      Serial.println("Ride Mode Disabled");
      
      xSemaphoreTake(xRideModeStartSemaphore, portMAX_DELAY);
      
      digitalWrite(directionLedPowerPin, HIGH);    
      Serial.println("Ride Mode Enabled");
      
    }

    if(xSemaphoreTake(xLeftToggleSemaphore, 0) == pdTRUE){

      if (right_state == 1 & left_state == 1){
        left_state = 1;
        right_state = 0;
      }
      else {
        left_state = !left_state;
      
        if (right_state == 1 and left_state == 1){
            right_state = 0;
        }
      }
      
      if (right_state == 0){
        right_anim_state = 0;
        for (i = 0; i < 3; i++){
          right_leds[i] = CRGB::Black;
        }
      }

      if (left_state == 0){
        left_anim_state = 0;
        for (i = 0; i < 3; i++){
          left_leds[i] = CRGB::Black;
        }
      }

      if (left_state == 0 & right_state == 0){
        updateLedMatrixFramePredefined(20, false, 0);
      }
      else if (left_state == 1) {
        updateLedMatrixFramePredefined(18, false, 0);
      }
      else if (right_state == 1) {
        updateLedMatrixFramePredefined(19, false, 0);
      }
        
    }

    if(xSemaphoreTake(xRightToggleSemaphore, 0) == pdTRUE){
      
      if (right_state == 1 & left_state == 1){
        left_state = 0;
        right_state = 1;
      }
      else {
        right_state = !right_state;
      
        if (right_state == 1 and left_state == 1){
            left_state = 0;
        }
      }

      if (left_state == 0){
        left_anim_state = 0;
        for (i = 0; i < 3; i++){
          left_leds[i] = CRGB::Black;
        }
      }

      if (right_state == 0){
        right_anim_state = 0;
        for (i = 0; i < 3; i++){
          right_leds[i] = CRGB::Black;
        }
      }

      if (left_state == 0 & right_state == 0){
        updateLedMatrixFramePredefined(20, false, 0);
      }
      else if (left_state == 1) {
        updateLedMatrixFramePredefined(18, false, 0);
      }
      else if (right_state == 1) {
        updateLedMatrixFramePredefined(19, false, 0);
      }
    }

    if(xSemaphoreTake(xStraightToggleSemaphore, 0) == pdTRUE){

       left_state = 1;
       right_state = 1;

       left_anim_state = 0;
       right_anim_state = 0;

       updateLedMatrixFramePredefined(20, false, 0);
      
    }

    if(xSemaphoreTake(xClearToggleSemaphore, 0) == pdTRUE){

      left_state = 0;
      right_state = 0;

      left_anim_state = 0;
      right_anim_state = 0;

      updateLedMatrixFramePredefined(20, false, 0);
      
    }

    if(left_state){
      for(i = 0; i < 3; i++){
        left_leds[i] = CRGB::Black;
        pixel_index = i % 3;
        index = left_anim_state*9 + pixel_index*3;

        left_leds[i].r = led_indicator_pattern[index];
        left_leds[i].g = led_indicator_pattern[index+1];
        left_leds[i].b = led_indicator_pattern[index+2];
      }
    }

    if(right_state){
      for(i = 0; i < 3; i++){
        right_leds[i] = CRGB::Black;
        pixel_index = i % 3;
        index = right_anim_state*9 + pixel_index*3;

        right_leds[i].r = led_indicator_pattern[index];
        right_leds[i].g = led_indicator_pattern[index+1];
        right_leds[i].b = led_indicator_pattern[index+2];
      }
    }

    Serial.print("Left State: ");
    Serial.println(left_state);
    Serial.print("Right State: ");
    Serial.println(right_state);
    Serial.print("Left Anim State :");
    Serial.println(left_anim_state);
    Serial.print("Right Anim State :");
    Serial.println(right_anim_state);

    left_anim_state++;
    right_anim_state++;

    if (left_anim_state >= 6) left_anim_state = 0;
    if (right_anim_state >= 6) right_anim_state = 0;

 if (left_state == 0 & right_state == 0){
   vibration_toggle = 0;
   ledcWrite(LeftMotorChannel, 0);
   ledcWrite(RightMotorChannel, 0);
  }
  else {
   vibration_toggle = 1;
  }

    if (vibration_toggle){

      vibration_state = !vibration_state;

      if (right_state == 1 & left_state == 1){
        if (navigation_cmd == 1){
          if (vibration_state){
            if (left_state){
              ledcWrite(LeftMotorChannel, vibration_intensity);
            } else {
              ledcWrite(LeftMotorChannel, 0);
            }
            if (right_state){
              ledcWrite(RightMotorChannel, vibration_intensity);
            } else {
              ledcWrite(RightMotorChannel, 0);
            }
          }
          else {
            ledcWrite(LeftMotorChannel, 0);
            ledcWrite(RightMotorChannel, 0);
          }
        }
        else {
          ledcWrite(LeftMotorChannel, 0);
          ledcWrite(RightMotorChannel, 0);
        }
      }
      else {
        if (vibration_state){
          if (left_state){
            ledcWrite(LeftMotorChannel, vibration_intensity);
          } else {
            ledcWrite(LeftMotorChannel, 0);
          }
          if (right_state){
            ledcWrite(RightMotorChannel, vibration_intensity);
          } else {
            ledcWrite(RightMotorChannel, 0);
          }
        }
        else {
          ledcWrite(LeftMotorChannel, 0);
          ledcWrite(RightMotorChannel, 0);
        }
      }
    }
    
    FastLED.show();
        
    vTaskDelay(500);
    
  }
}

void notificationTask(void * pvParameters){

  xNotificationsToggle = xSemaphoreCreateBinary();
  xNotificationsUpdate = xSemaphoreCreateBinary();

  int alarm_interval;

  int missing_item_notif_local[3];
  int secure_mode_notif_local[3];
  int power_notif_local[3];

  int alarm_mode_local = 0;

  int vibrate_notif, audio_notif, visual_notif;
  bool toggle = false;

   for(int i = 0; i < 3; i++){
    
    missing_item_notif_local[i] = missing_item_notif[i];
    secure_mode_notif_local[i] = secure_mode_notif[i];
    power_notif_local[i] = power_notif[i];
    
  }

  alarm_mode_local = alarm_mode;

  while(1){

    if (alarm_mode_local != 0){
      
      if(xSemaphoreTake(xNotificationsToggle, 0) == pdTRUE){

        alarm_mode_local = alarm_mode;

        switch(alarm_mode_local){
          case 0:
            restoreLedMatrixFrameBackup();
            digitalWrite(buzzerPin, LOW);
            digitalWrite(LeftMotorPin, LOW);
            digitalWrite(RightMotorPin, LOW);
            continue;
            break;
          case 1:
            vibrate_notif = missing_item_notif_local[0];
            audio_notif = missing_item_notif_local[1];
            visual_notif = missing_item_notif_local[2];
            if(visual_notif){
              updateLedMatrixFramePredefined(5, true, 0);
            } else {
              restoreLedMatrixFrameBackup();
            }
            alarm_interval = 200;
            break;
          case 2:
            vibrate_notif = secure_mode_notif_local[0];
            audio_notif = secure_mode_notif_local[1];
            visual_notif = secure_mode_notif_local[2];
            secureModeNotification();
            if(visual_notif){
              updateLedMatrixFramePredefined(7, true, 0);
            } else {
              restoreLedMatrixFrameBackup();
            }
            alarm_interval = 400;
            break;
          case 3:
            vibrate_notif = power_notif_local[0];
            audio_notif = power_notif_local[1];
            visual_notif = power_notif_local[2];
            if(visual_notif){
              updateLedMatrixFramePredefined(9, true, 0);
            } else {
              restoreLedMatrixFrameBackup();
            }
            alarm_interval = 600;
            break;
        } 
      }
    } else {

      xSemaphoreTake(xNotificationsToggle, portMAX_DELAY);

      alarm_mode_local = alarm_mode;

      for(int i = 0; i < 3; i++){
         missing_item_notif_local[i] = missing_item_notif[i];
         secure_mode_notif_local[i] = secure_mode_notif[i];
         power_notif_local[i] = power_notif[i];
       }
      
      switch(alarm_mode_local){
          case 0:
            continue;
            break;
          case 1:
            vibrate_notif = missing_item_notif_local[0];
            audio_notif = missing_item_notif_local[1];
            visual_notif = missing_item_notif_local[2];
            if(visual_notif){
              updateLedMatrixFramePredefined(5, true, 0);
            }
            alarm_interval = 200;
            break;
          case 2:
            vibrate_notif = secure_mode_notif_local[0];
            audio_notif = secure_mode_notif_local[1];
            visual_notif = secure_mode_notif_local[2];
            if(visual_notif){
              updateLedMatrixFramePredefined(7, true, 0);
            }
            alarm_interval = 400;
            break;
          case 3:
            vibrate_notif = power_notif_local[0];
            audio_notif = power_notif_local[1];
            visual_notif = power_notif_local[2];
            if(visual_notif){
              updateLedMatrixFramePredefined(9, true, 0);
            }
            alarm_interval = 600;
            break;
      } 
    }

//    if (xSemaphoreTake(xNotificationsUpdate, 0) == pdTRUE){
//       
//       for(int i = 0; i < 3; i++){
//         missing_item_notif_local[i] = missing_item_notif[i];
//         secure_mode_notif_local[i] = secure_mode_notif[i];
//         power_notif_local[i] = power_notif[i];
//       }
//       
//       switch (alarm_mode_local){
//         case 1:
//           vibrate_notif = missing_item_notif_local[0];
//           audio_notif = missing_item_notif_local[1];
//           visual_notif = missing_item_notif_local[2]
//           break;
//         case 2:
//           vibrate_notif = secure_mode_notif_local[0];
//           audio_notif = secure_mode_notif_local[1];
//           visual_notif = secure_mode_notif_local[2]
//         case 3:
//           vibrate_notif = power_notif_local[0];
//           audio_notif = power_notif_local[1];
//           visual_notif = power_notif_local[2]
//       }
//    }

    if (vibrate_notif){
      digitalWrite(LeftMotorPin, toggle);
      digitalWrite(RightMotorPin, toggle);
    }

    if (audio_notif){
      digitalWrite(buzzerPin, toggle);
    }

    toggle = !toggle;
    vTaskDelay(alarm_interval);
    
  }
}

void disableAlarm(int alarm_toggle){

  updateSecureMode(alarm_toggle);
  
}

void updateNavigation(int navigation){

  if (system_mode == 3){
    if (navigation == 0){
      navigation_cmd = 1;
      if(xSemaphoreGive(xStraightToggleSemaphore) == pdTRUE){
        Serial.println("Straight Toggle Signal Given (NAVIGATION)");
      }
    }
    else if (navigation == 1){
      navigation_cmd = 1;
      if(xSemaphoreGive(xLeftToggleSemaphore) == pdTRUE){
        Serial.println("Left Toggle Signal Given (NAVIGATION)");
      }
    }
    else if (navigation == 2){
      navigation_cmd = 1;
      if(xSemaphoreGive(xRightToggleSemaphore) == pdTRUE){
        Serial.println("Right Toggle Signal Given (NAVIGATION)");
      }
    }
    else if (navigation == 3){
      navigation_cmd = 1;
      if(xSemaphoreGive(xClearToggleSemaphore) == pdTRUE){
        Serial.println("Clear Toggle Signal Given (NAVIGATION)");
      }
    }
  }
  else {
    Serial.println("Invalid Command: Bag is not in Ride Mode");
  }
}

void updateAnimationTemp(int animation_id, int duration){
  updateLedMatrixFramePredefined(animation_id, true, 0);
  vTaskDelay(duration);
  restoreLedMatrixFrameBackup();
}

void silentMode(){
  unsigned long old_time= millis();
  unsigned long new_time=millis();
  updateAnimationTemp(12, 1000);
  vTaskDelay(1000);
  lc.shutdown(0,true);
  updateLedMatrixFramePredefined(22, false, 0);
  ledcWrite(LeftMotorChannel, 0);
  ledcWrite(RightMotorChannel, 0);
  while(new_time-old_time<10000)
  {
  digitalWrite(buzzerPin, LOW);
  new_time=millis();
  }
}

void updateSettingsEeprom(){

  long number = (long) strtol(&direction_led_color[0], NULL, 16);

  byte r = number >> 16;
  byte g = number >> 8 & 0xFF;
  byte b = number & 0xFF;

  //Direction LED Colour Loading into EEPROM
  //(Address - Data)
  //(0 - Red)
  //(1 - Blue)
  //(2 - Green)

  EEPROM.write(0, r);
  EEPROM.commit();
  EEPROM.write(1, g);
  EEPROM.commit();
  EEPROM.write(2, b);
  EEPROM.commit();

  //Missing Item Notification Loading into EEPROM
  //Address: 3-5

  EEPROM.write(3, missing_item_notif[0]);
  EEPROM.commit();
  EEPROM.write(4, missing_item_notif[1]);
  EEPROM.commit();
  EEPROM.write(5, missing_item_notif[2]);
  EEPROM.commit();
  //Secure Mode Notification Loading into EEPROM
  //Address: 6-8

  EEPROM.write(6, secure_mode_notif[0]);
  EEPROM.commit();
  EEPROM.write(7, secure_mode_notif[1]);
  EEPROM.commit();
  EEPROM.write(8, secure_mode_notif[2]);
  EEPROM.commit();

  //Power Notification Loading into EEPROM
  //Address: 9-11

  EEPROM.write(9, power_notif[0]);
  EEPROM.commit();
  EEPROM.write(10, power_notif[1]);
  EEPROM.commit();
  EEPROM.write(11, power_notif[2]);

  //BLE Scan Interval loading into EEPROM
  //Address: 12
EEPROM.commit();
  EEPROM.write(12, ble_scan_interval);

  //Schedule Mode loading into EEPROM
  //Address: 13
EEPROM.commit();
  EEPROM.write(13, schedule_mode);
EEPROM.commit();
  //Vibration Intensity loading into EEPROM
  //Address: 14

  EEPROM.write(14, vibration_intensity);
  EEPROM.commit();
}

void updateAddressEeprom(){
  //Addresses: 
  //16 Num Beacons
  //17 - 200 Beacon Addresses (18 per beacon)

  Serial.println("Update BLE addresses to EEPROM Begun");

  EEPROM.begin(1);

  EEPROM.write(16, 3);

  int i,j;
  int add_start = 0;
  String temp_string;

  for (i = 0; i < numBeacons; i++){
    add_start = i*18 + 17;
    temp_string = beaconAddresses[i];
    
    for (j = 0; j < 17; j++){
      EEPROM.write(add_start + j, temp_string[j]);
    }
    EEPROM.write(add_start + 17,'\0');
  }

  EEPROM.commit();
}

void loadAddressEeprom(){

  EEPROM.begin(1);

  int numBeaconsTemp = EEPROM.read(16);

  Serial.print("Num Beacons Temp: ");
  Serial.println(numBeaconsTemp);
  
  int i, j, add_start;
  String temp_string;

  for (i = 0; i < numBeaconsTemp; i++){
    add_start = i*18 + 17;
    temp_string = read_string_eeprom(add_start);
    beaconAddresses[i] = temp_string;
  }

  numBeacons = numBeaconsTemp;

  EEPROM.end();
}

String read_string_eeprom(int add_start)
{
  int i;
  char data_read[18]; //Max 100 Bytes
  int len=0;
  unsigned char k;
  k=EEPROM.read(add_start);
  while(k != '\0' && len<500)   //Read until null character
  {    
    k=EEPROM.read(add_start+len);
    data_read[len]=k;
    len++;
  }
  data_read[len]='\0';
  return String(data_read);
}

void updateDeepSleep(int State)
{
  if(State==1)
  {
       lc.shutdown(0,true);
       esp_deep_sleep_start();
    }  
}

int ldrValue=0;
void zipStateOpen(int StatusOfZip)
{
  ldrValue = analogRead(ldrPin);
  if(StatusOfZip==1 && ldrValue>3000)
  {
     updateInnerIllumination(1);  
     ledTimer(1);
  }
  if(StatusOfZip==0 || ldrValue<3000)
  { 
     updateInnerIllumination(0);  
  }  
}

void ledTimer(int ledTimer)
{
  unsigned long old_time= millis();
  unsigned long new_time=millis();
  while(inner_illumination_state==1 )
    {   
        if (new_time-old_time>10000 || ledTimer==0)
           {
            updateInnerIllumination(0);
            zipStateGlobal=1;
            inner_illumination_state=0;
           }
        new_time=millis();
    }
    
    ledTimer=0;
}

void boot()
{
    digitalWrite(directionLedPowerPin,HIGH);
    for(int i=0; i<12; i++){
        circle_leds[i] = CRGB(0, 255, 0);
        FastLED.show();
        }
    updateAnimationTemp(28 , 1000);
    updateLedMatrixFramePredefined(21, false, 0);
}

void mqPower(int state)
{
  if(state == 1)
  {
   digitalWrite(mqPowerPin, HIGH); 
  }
  if(state == 0)
  {
   digitalWrite(mqPowerPin, LOW); 
  }
}

void mqRead()
{
  StaticJsonDocument<128> doc;
  String output;
  
  old_time_MQ= millis();
  mqReading = digitalRead(pollutionSensorPin);
  doc["command_id"] = 14;
  doc["data"] = mqReading;
  Serial.println(mqReading);
  serializeJson(doc, output);
        
  pCharacteristic_TX->setValue(output.c_str());
  pCharacteristic_TX->notify();
  mqPower(0);
}

void secureModeNotification()
{
   StaticJsonDocument<128> doc;
   String output;
   doc["command_id"] = 12;
   doc["zip_alert"] = 1;
   doc["movement_alert"] = 1;

   serializeJson(doc, output);
        
   pCharacteristic_TX->setValue(output.c_str());
   pCharacteristic_TX->notify();
}
