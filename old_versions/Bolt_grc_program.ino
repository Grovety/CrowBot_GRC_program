#include "BLEDevice.h"
#include "BLEScan.h"
//#include <Wire.h>
//#include <LiquidCrystal.h>
//LiquidCrystal lcd(0);
#include <FastLED.h>
#include <IRremote.h>
#include <Ticker.h>  //Call the ticker. H Library
#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/message_buffer.h>
#include <freertos/projdefs.h>
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/21, /* data=*/22, /* reset=*/U8X8_PIN_NONE);  // Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED
Ticker ticker;
Ticker ticker1;
#define RECV_PIN 19       //Infrared receiving pin
IRrecv irrecv(RECV_PIN);  //Definition of infrared remote control pin
decode_results results;   //Define infrared remote control function

CRGB leds[4];               //Bottom RGB light
CRGB RGBleds[6];            //Ultrasonic RGB lamp
CRGB myRGBcolor(0, 0, 0);   //Bottom RGB
CRGB myRGBcolor6(0, 0, 0);  //Ultrasonic RGB
BLEScan* pBLEScan = NULL;
// The remote service we wish to connect to.
static BLEUUID serviceUUID("FFE0");  //Host service UUID
static BLEUUID charUUID("FFE1");
static boolean doConnect = false;                        //Connection sign
static boolean connected = false;                        //Connection success flag
static boolean doSacn = true;                            //Scan flag
static BLERemoteCharacteristic* pRemoteCharacteristic;   //Characteristic value sent by handle
static BLEAdvertisedDevice* pServer;
//Arduino docs It is emphasized that volatile must be used in variable declaration in case of concurrent threads (such as interrupts)
volatile int pwm_value = 0;  //Low level duration --- interruption
volatile int prev_time = 0;  //Record ultrasonic time --- interruption
volatile int distance = 0;   //Ultrasonic distance
int Close_Flag = 0;          //Turn off task flag
int BLE_Close = 0;           //Bluetooth off flag
int Ce_shi_Flag = 0;         //Test program flag
static int IR_Flag = 88;     //Infrared remote control sign
int shock_Flag = 0;          //Vibration sign
char CloseData;              //Receive serial port data during test
static int L = 0;            //Number of switch light signs
int num_L = 0;               //Save the left light seeking value
int num_R = 0;               //Save the right light seeking value
int i = 0;
static int rgb = 0;   //Bottom RGB
static int rgb6 = 0;  //Ultrasonic RGB
const int leftgo_Pin = 12;
const int lefttrun_Pin = 13;
const int rightgo_Pin = 14;
const int righttrun_Pin = 15;
const int Photodiode_R = 34, Photodiode_L = 35;  //Light seeking pin
int Ultrasonic_Pin = 27, Button_pin = 18;        //Ultrasonic and switch key pins
int QTI_Max, QTI_L = 39, QTI_R = 36;             //Line patrol pin
static int state_L;
uint32_t state_N;                     //Infrared remote control data recording
char LU[10];                                     //Save ultrasonic distance data
std::string value;
BLEClient* pClient = NULL;
BLERemoteService* pRemoteService = NULL;
#define  G4 392
#define  A4 440
#define  B4 494
#define  C5 523
#define  D5 587
#define  E5 659
#define  F5 698
#define  G5 784
#define C4 262
#define D4 294
#define E4 330
#define F4 349
int Tone[] = { 
             };
float Time[] = {
               }; 
int Tone_length = sizeof(Tone) / sizeof(Tone[0]);
SemaphoreHandle_t xPlayMusicSemaphore;
SemaphoreHandle_t xDisconnectedSemaphore;
MessageBufferHandle_t xGrcCmdBuffer;

typedef struct {
  const int* tones;     
  const float* times;   
  int length;           
} Melody;


const int happy_tones[] = {
  G4, G4, A4, G4, C5, B4,
  G4, G4, A4, G4, D5, C5,
  G4, G4, G5, E5, C5, B4, A4,
  F5, F5, E5, C5, D5, C5
};

const float happy_times[] = {
  0.5,0.5,1.0,1.0,1.0,2.0,
  0.5,0.5,1.0,1.0,1.0,2.0,
  0.5,0.5,1.0,1.0,1.0,1.0,2.0,
  0.5,0.5,1.0,1.0,1.0,2.0
};


const int twinkle_tones[] = {
  C4,C4,G4,G4,A4,A4,G4,
  F4,F4,E4,E4,D4,D4,C4
};

const float twinkle_times[] = {
  0.5,0.5,0.5,0.5,0.5,0.5,1.0,
  0.5,0.5,0.5,0.5,0.5,0.5,1.0
};

const int main_tones[] = {
    B4, B4, B4, B4, B4, B4,
    B4, D5, G4, A4, B4, C5,
    C5, C5, C5, C5, B4, B4, B4, D5,
    D5, C5, A4, G4, G5
};

const float main_times[] = {
    0.25, 0.25, 0.5, 0.25, 0.25, 0.5,
    0.25, 0.25, 0.375, 0.125, 1, 0.25,
    0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25,
    0.25, 0.25, 0.25, 0.5, 0.5
};
Melody melodies[] = {
  { happy_tones, happy_times, sizeof(happy_tones)/sizeof(happy_tones[0]) },
  { twinkle_tones, twinkle_times, sizeof(twinkle_tones)/sizeof(twinkle_tones[0]) },
  {main_tones, main_times, sizeof(main_tones)/sizeof(main_tones[0])}
};

// ====== music -> dance communication ======
typedef struct {
  int freq;          
  int duration_ms;   
} MusicNoteEvent;

QueueHandle_t xMusicQueue;
volatile bool dance_mode = false; 



const int melodyCount = sizeof(melodies) / sizeof(melodies[0]);

#define MAX_GRC_MSG_LEN 20

#define MOTOR_MIN_VALUE 15
#define MOTOR_MAX_VALUE 255

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
    return std::less<T>{}(v, lo) ? lo : std::less<T>{}(hi, v) ? hi : v;
}

void poly_transform(const float params[12], float x, float y, float *_x, float *_y) {
  *_x = params[0] + params[1] * x + params[2] * y +
              params[3] * powf(x, 2) + params[4] * x * y +
              params[5] * powf(y, 2);
  *_y = params[6 + 0] + params[6 + 1] * x + params[6 + 2] * y +
              params[6 + 3] * powf(x, 2) + params[6 + 4] * x * y +
              params[6 + 5] * powf(y, 2);
}

void led_callback()  //Callback function
{
  FastLED.show();
}
//Variables for the CalcTurnTime function
const int  baseSpeed   = 100;       
const float msPerDeg   = 2.55f;     
const float overheadMs = 0.0f;      
const int Speed        = 150;


const int pauseTime    = 300;      
int calcTurnTime(int angle) {
  float t90 = overheadMs + msPerDeg * (baseSpeed / float(Speed)) * 90.0f;
  return int(t90 * (angle / 90.0f) + 0.5f);
}

static bool moving_backwards = false;
void buzzer_callback()  //Callback function
{
  if (moving_backwards) {
    ledcWriteTone(2, G4);  //Buzzer
    delay(150);
    ledcWrite(2, 0);
  }
}

void play_music_task(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(xPlayMusicSemaphore, portMAX_DELAY) == pdTRUE) {
      int index = random(0, melodyCount);
      Melody m = melodies[index];

      for (int i = 0; i < m.length; i++) {
        int freq = m.tones[i];
        int dur_ms = int(m.times[i] * 1000.0f);

        MusicNoteEvent ev = { freq, dur_ms };
        if (xMusicQueue != NULL) {
          xQueueSend(xMusicQueue, &ev, pdMS_TO_TICKS(10)); 
        }

        ledcWriteTone(33, freq);
        vTaskDelay(pdMS_TO_TICKS(dur_ms));
      }
      ledcWriteTone(33, 0); 
      if (xMusicQueue != NULL) {
        MusicNoteEvent evEnd = { 0, 0 };
        xQueueSend(xMusicQueue, &evEnd, pdMS_TO_TICKS(10));
      }
    }
  }
}
void dance_task(void *pvParameters) {
  MusicNoteEvent ev;
  for (;;) {
    if (xQueueReceive(xMusicQueue, &ev, portMAX_DELAY) == pdTRUE) {
      if (!dance_mode || ev.freq == 0) {
        if (ev.duration_ms > 0) vTaskDelay(pdMS_TO_TICKS(ev.duration_ms));
        Motor(0,0,0,0);
        fill_solid(leds, 4, CRGB::Black);
        FastLED.show();
        continue;
      }
      if (ev.freq < 300) {
        // heavy step forward + flash
        Motor(200,0,200,0);
        fill_solid(leds, 4, CRGB::Blue);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(std::min(ev.duration_ms, 400)));
        Motor(0,0,0,0);
      } else if (ev.freq < 600) {
        // spin right a little
        Motor(180,0,0,180);
        fill_solid(leds, 4, CRGB::Green);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(std::min(ev.duration_ms, 300)));
        Motor(0,0,0,0);
      } else {
        // wiggle / little jumps + bright flash
        for (int k=0; k<3 && k*100 < ev.duration_ms; ++k) {
          Motor(200,0,200,0);
          fill_solid(leds, 4, CRGB::White);
          FastLED.show();
          vTaskDelay(pdMS_TO_TICKS(80));
          Motor(0,0,0,0);
          fill_solid(leds, 4, CRGB::Black);
          FastLED.show();
          vTaskDelay(pdMS_TO_TICKS(60));
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}


void grc_cmd_task(void*arg)
{
  const float d_speed = 0.5;
  float speed = 1;
  bool imu_control_mode = false;
  auto init_imu_control_state = [&imu_control_mode]() {
    Serial.printf("Enable imu control mode\n");
    imu_control_mode = true;
    ticker.attach_ms(300, led_callback);  //lighting task
    ticker1.attach_ms(900, buzzer_callback);  //buzzer task
  };
  auto release_imu_control_state = [&imu_control_mode, &moving_backwards]() {
    Serial.printf("Disable imu control mode\n");
    imu_control_mode = false;
    moving_backwards = false;
    Motor(0, 0, 0, 0);
    ledcWrite(2, 0);
    fill_solid(leds, 4, CRGB::Black);
    FastLED.show();
    ticker.detach();
    ticker1.detach();
  };
  auto front_lights_cmd = [](bool front_lights_en) {
    uint8_t val = 0; 
    if (front_lights_en) {
      FastLED.setBrightness(255);  //RGB lamp brightness range: 0-255
      val = 255;
    }
    myRGBcolor6.r = val;
    myRGBcolor6.g = val;
    myRGBcolor6.b = val;
    fill_solid(RGBleds, 6, myRGBcolor6);
    FastLED.show();
  };
  char msg[MAX_GRC_MSG_LEN];
    for (;;) {
    size_t length = xMessageBufferReceive(xGrcCmdBuffer, &msg, MAX_GRC_MSG_LEN, portMAX_DELAY);
    std::string value(msg, length);
    const std::string::size_type coords_offset = value.find("XY"); 
    const bool recv_imu_coords = coords_offset != std::string::npos;
    if (recv_imu_coords && !imu_control_mode) {
      init_imu_control_state();
    }

    //********************************GRC voice command******************************************
    if (!imu_control_mode) {
      if (value == "GO FORWARD" || value == "xiang qian zou")  //forward
      {
        leds[2] = CRGB::Green;
        leds[3] = CRGB::Green;
        fill_solid(RGBleds, 6, myRGBcolor6);
        FastLED.show();
        Motor(float(160) * speed, 0, float(160) * speed, 0);
        delay(600);
        Motor(0, 0, 0, 0);
        fill_solid(leds, 4, CRGB::Black);
        FastLED.show();
      }
            if (value == "DO DANCE") {
        dance_mode = true;
        Serial.println("Dance mode ON");
      }

      if (value == "STOP DANCE") {
        dance_mode = false;
        Serial.println("Dance mode OFF");
        Motor(0,0,0,0);
        fill_solid(leds, 4, CRGB::Black);
        FastLED.show();
      }
      if (value == "DANCE TANGO") {

      Motor(160, 0, 160, 0);   
      delay(500);              
      Motor(0,0,0,0);
      delay(100);

      Motor(Speed, 0, 0, Speed);   
      delay(calcTurnTime(180));    
      Motor(0,0,0,0);
      delay(100);

      Motor(0,160,0,160);      
      delay(500);
      Motor(0,0,0,0);
      delay(100);

      Motor(Speed, 0, 0, Speed);
      delay(calcTurnTime(180));
      Motor(0,0,0,0);
    }


      if (value == "GO BACK")  //backward
      {
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Red;
        fill_solid(RGBleds, 6, myRGBcolor6);
        FastLED.show();
        Motor(0, float(160) * speed, 0, float(160) * speed);
        delay(600);
        Motor(0, 0, 0, 0);
        fill_solid(leds, 4, CRGB::Black);
        FastLED.show();
      }

      if (value == "GO RIGHT")  //towards the right
      {
        leds[2] = CRGB::Green;
        fill_solid(RGBleds, 6, myRGBcolor6);
        FastLED.show();
        Motor(80, 0, 0, 80);
        delay(350);
        Motor(0, 0, 0, 0);
        fill_solid(leds, 4, CRGB::Black);
        FastLED.show();
      }

      if (value == "GO LEFT")  //towards the left
      {
        leds[3] = CRGB::Green;
        fill_solid(RGBleds, 6, myRGBcolor6);
        FastLED.show();
        Motor(0, 80, 80, 0);
        delay(350);
        Motor(0, 0, 0, 0);
        fill_solid(leds, 4, CRGB::Black);
        FastLED.show();
      }
      if (value == "DO SQUARE")
      {
      int tForward = calcTurnTime(0); 
      for (int i = 0; i < 4; ++i) {
        Motor(float(160) * speed, 0, float(160) * speed, 0);
        delay(tForward > 0 ? tForward : 1200);
        Motor(0, 0, 0, 0);
        delay(pauseTime);
        Motor(Speed, 0, 0, Speed);
        delay(calcTurnTime(95));
        Motor(0, 0, 0, 0);
        delay(pauseTime);
        }
      }
      if (value == "DO TRIANGLE"){
        int tForward = calcTurnTime(0);
        for (int i = 0; i < 3; ++i) {
          Motor(float(160) * speed, 0, float(160) * speed, 0);
          delay(tForward > 0 ? tForward : 1200);
          Motor(0,0,0,0);
          delay(pauseTime);
          Motor(Speed, 0, 0, Speed);
          delay(calcTurnTime(125));
          Motor(0,0,0,0);
          delay(pauseTime);
        }
      }
      if (value == "DO SNAKE") {
      int tForward = calcTurnTime(0) > 0 ? calcTurnTime(0) : 200;
      const int snakeAngle = 90;
      const int segments = 15;
      Motor(0,  Speed,  Speed, 0);
      delay(calcTurnTime(45));
      Motor(0, 0, 0, 0);
      for (int i = 0; i < segments; ++i) {
        Motor(float(160) * speed, 0, float(160) * speed, 0);
        delay(tForward);
        Motor(0, 0, 0, 0);
        delay(50);

        if (i % 2 == 0) {
          Motor( Speed, 0, 0,  Speed);
        } else {
          Motor(0,  Speed + 3,  Speed + 30, 0);
        }
        delay(calcTurnTime(snakeAngle));
        Motor(0, 0, 0, 0);
        delay(50);
      }
    }

      if (value == "FASTER SPEED")  // change speed
      {
        speed = std::min(1.5f, speed + d_speed);
      }

      if (value == "SLOWER SPEED")  // change speed
      {
        speed = std::max(0.5f, speed - d_speed);
      }

      if (value == "LIGHTS ON")  // enable lights
      {
        front_lights_cmd(true);
      }

      if (value == "LIGHTS OFF")  // disable lights
      {
        front_lights_cmd(false);
      }

      if (value == "PLAY MUSIC")  // toggle music
      {
        xSemaphoreGive(xPlayMusicSemaphore);
      }

      if (value == "MANUAL CONTROL")  // enable imu_control_mode
      {
        init_imu_control_state();
      }
    } else {
      if (value == "VOICE CONTROL")  // disable imu_control_mode
      {
        release_imu_control_state();
      }
      if (value == "LIGHTS ON")  // enable lights
      {
        front_lights_cmd(true);
      }
      if (value == "LIGHTS OFF")  // disable lights
      {
        front_lights_cmd(false);
      }
      if (value == "PLAY MUSIC")  // toggle music
      {
        xSemaphoreGive(xPlayMusicSemaphore);
      }
      //********************************GRC imu command******************************************
      if (recv_imu_coords) {
        int8_t ix, iy;
        memcpy(&ix, &value.c_str()[coords_offset + 2], sizeof(int8_t));
        memcpy(&iy, &value.c_str()[coords_offset + 3], sizeof(int8_t));

        float x, y;
        x = clamp(float(ix), -45.f, 45.f);
        y = clamp(float(iy), -45.f, 45.f);

        static const float params[] = {
          1.689410597460024e-14,   1.3274074074074107,    3.792592592592593,
          -1.0409086843526395e-17, 0.012641975308641975,  -2.995326321410946e-18,
          3.215022330213294e-08,   -1.3274074134624825,   3.7925925925925945,
          8.344758175431135e-18,   -0.012641975308641976, -2.3814980193096928e-11};
        float l, r;
        poly_transform(params, x, y, &l, &r);

        if (l <= MOTOR_MIN_VALUE && l >= -MOTOR_MIN_VALUE) {
          l = 0;
        }
        if (r <= MOTOR_MIN_VALUE && r >= -MOTOR_MIN_VALUE) {
          r = 0;
        }      

        fill_solid(leds, 4, CRGB::Black);
        int lf, lb, rf, rb;
        if (r < 0.f) {
          rf = 0;
          rb = std::min(int(-r), MOTOR_MAX_VALUE);
        } else {
          rf = std::min(int(r), MOTOR_MAX_VALUE);
          rb = 0;
          myRGBcolor.r = 0;
          myRGBcolor.g = rf;
          myRGBcolor.b = 0;
          leds[3] = myRGBcolor;
        }
        if (l < 0.f) {
          lf = 0;
          lb = std::min(int(-l), MOTOR_MAX_VALUE);
        } else {
          lf = std::min(int(l), MOTOR_MAX_VALUE);
          lb = 0;
          myRGBcolor.r = 0;
          myRGBcolor.g = lf;
          myRGBcolor.b = 0;
          leds[2] = myRGBcolor;
        }
        if (r < 0.f && l < 0.f) {
          myRGBcolor.r = rb;
          myRGBcolor.g = 0;
          myRGBcolor.b = 0;
          leds[0] = myRGBcolor;
          myRGBcolor.r = lb;
          myRGBcolor.g = 0;
          myRGBcolor.b = 0;
          leds[1] = myRGBcolor;
        }
        Motor(lf, lb, rf, rb);
        if (l < -MOTOR_MIN_VALUE && r < -MOTOR_MIN_VALUE) {
          moving_backwards = true;
        } else {
          moving_backwards = false;
        }
      }
    }
  }
}

static void NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  xMessageBufferSend(xGrcCmdBuffer, pData, length, 0);
}

// Callback function for connection and disconnection between client and server
class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {}
    void onDisconnect(BLEClient* pclient) {
      doSacn = true;
      connected = false;
      Serial.println("Lost connection to device");
      pBLEScan->setActiveScan(true);  //Turn on scanning
      pBLEScan->setInterval(100);
      pBLEScan->setWindow(99);
      // TODO: stop all activity on disconnect
      Motor(0, 0, 0, 0);
      xSemaphoreGive(xDisconnectedSemaphore);
    }
};


bool ConnectToServer(void) {
  // Create client
  pClient = BLEDevice::createClient();
  //Serial.println("Create client");
  Serial.println("Creating a Client");

  // Add callback function for connection and disconnection between client and server
  pClient->setClientCallbacks(new MyClientCallback());  // Add callback function for connection and disconnection between client and server

  // Try to connect the device
  if (!pClient->connect(pServer)) {  // Try to connect the device
    Serial.println("Failed to connect to the device");
    return false;
  }
  //Serial.println("Connect device successfully");
  Serial.println("Bluetooth connection successful");
  // Try to get the service in the device
  pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    pClient->disconnect();
    return false;
  }
  //Serial.println("Obtain service successfully");
  Serial.println("Obtaining Services successfully");

  // Try to get the characteristics in the service
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to get attribute");
    pClient->disconnect();
    return false;
  }
  //Serial.println("Get features successfully");
  Serial.println("Obtaining the function successfully");
  // If the characteristic value can be read, read the data
  if (pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue();
    //    Serial.print("The characteristic value can be read and the current value is: ");
    //    Serial.println(value.c_str());
  }

  // If push is enabled for feature values, push receiving processing is added
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(NotifyCallback);
  }
  connected = true;
  return true;
}


class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      //Serial.print("BLE Advertised Device found: ");
      //Serial.println(advertisedDevice.toString().c_str());

      // Callback function when a device is found
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)
      ) {
        //Serial.print("Find device！  address: ");
        advertisedDevice.getScan()->stop();                   // Stop current scan
        pServer = new BLEAdvertisedDevice(advertisedDevice);  // Temporary storage device
        doConnect = true;
        doSacn = false;
        //Serial.println("Find the device you want to connect");
        Serial.println("The device to connect has been found");
      }
    }
};


//Wheel movement
void Motor(int L1, int L2, int R1, int R2) {
  ledcWrite(13, L1);
  ledcWrite(12, L2);
  ledcWrite(15, R1);
  ledcWrite(14, R2);
}

//Interrupt function-----RISING
void risingCallback() {
  attachInterrupt(digitalPinToInterrupt(Ultrasonic_Pin), fallingCallback, FALLING);  //Enable descent interrupt
  prev_time = micros();                                                              //Start timing
}

//Interrupt function-----FALLING
void fallingCallback() {
  pwm_value = micros() - prev_time;  //Low level duration
  if ((pwm_value < 60000) && (pwm_value > 1)) {
    distance = pwm_value / 58;
  }
}

void callback1()  //Callback function
{
  RGB_LED6();  //Ultrasonic RGB lamp
}


//Ultrasonic ranging
void Get_Distance() {
  pinMode(Ultrasonic_Pin, OUTPUT);
  digitalWrite(Ultrasonic_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Ultrasonic_Pin, HIGH);
  delayMicroseconds(20);
  digitalWrite(Ultrasonic_Pin, LOW);
  pinMode(Ultrasonic_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Ultrasonic_Pin), risingCallback, RISING);  //Enable rise interrupt
  //  int Time_Echo_us = pulseIn(Ultrasonic_Pin, HIGH);
  //  if ((Time_Echo_us < 10000) && (Time_Echo_us > 1)) {
  //    distance = Time_Echo_us / 58;
  //    Serial.print("Ultrasonic distance：");
  //    Serial.println(distance);
  //    itoa(distance, LU, 10);
  //    pRemoteCharacteristic2->writeValue(LU);
  //  }
}

//Baseboard RGB lamp ----- on
void RGB_LED() {
  FastLED.setBrightness(255);  //RGB lamp brightness range: 0-255
  myRGBcolor.r = random(0, 255);
  myRGBcolor.g = random(0, 255);
  myRGBcolor.b = random(0, 255);
  fill_solid(leds, 4, myRGBcolor);
  FastLED.show();
}

void RGB_LED_norandom() {
  FastLED.setBrightness(255);  //RGB lamp brightness range: 0-255

  for (int i = 0; i <= 3; i++) {
    leds[i] = CRGB::Red;
    FastLED.show();
    delay(500);

    leds[i] = CRGB::Black;
    FastLED.show();
    delay(500);  
  }
  myRGBcolor.r = 255;  //random(0, 255);
  myRGBcolor.g = 100;  //random(0, 255);
  myRGBcolor.b = 100;  //random(0, 255);
  fill_solid(leds, 4, myRGBcolor);
  FastLED.show();
  delay(2000);
}

//Backplane RGB lamp ----- off
void RGB_LED_Close() {
  FastLED.setBrightness(0);  //RGB lamp brightness range: 0-255
  myRGBcolor.r = 0;
  myRGBcolor.g = 0;
  myRGBcolor.b = 0;
  fill_solid(leds, 4, myRGBcolor);
  FastLED.show();
}

//Ultrasonic RGB lamp ----- on
void RGB_LED6() {
  FastLED.setBrightness(255);  //RGB lamp brightness range: 0-255
  myRGBcolor6.r = random(0, 255);
  myRGBcolor6.g = random(0, 255);
  myRGBcolor6.b = random(0, 255);
  fill_solid(RGBleds, 6, myRGBcolor6);
  FastLED.show();
}

void RGB_LED6_norandom() {
  FastLED.setBrightness(255);  //RGB lamp brightness range: 0-255
  myRGBcolor6.r = 255;         //random(0, 255);
  myRGBcolor6.g = 0;           //random(0, 255);
  myRGBcolor6.b = 255;         //random(0, 255);
  fill_solid(RGBleds, 6, myRGBcolor6);
  FastLED.show();
}

//Ultrasonic RGB lamp ----- off
void RGB_LED6_Close() {
  FastLED.setBrightness(0);  //RGB lamp brightness range: 0-255
  myRGBcolor6.r = 0;
  myRGBcolor6.g = 0;
  myRGBcolor6.b = 0;
  fill_solid(RGBleds, 6, myRGBcolor6);
  FastLED.show();
}

//infrared remote control
void IR_remote_control() {
  switch (state_N) {
    case 0xE718FF00UL:
      IR_Flag = 11;
      Serial.println("Forward");
      break;
    case 0xAD52FF00UL:
      IR_Flag = 22;
      Serial.println("Backward");
      break;
    case 0xF708FF00UL:
      IR_Flag = 33;
      Serial.println("Turn left");
      break;
    case 0xA55AFF00UL:
      IR_Flag = 44;
      Serial.println("Turn right");
      break;
    case 0xE31CFF00UL:
      IR_Flag = 55;
      Serial.println("Stop");
      break;
    case 0xBA45FF00UL:  //1
      IR_Flag = 1;
      break;
    case 0xB946FF00UL:  //2
      IR_Flag = 2;
      break;
    case 0xB847FF00UL:  //3
      IR_Flag = 3;
      break;
    case 0xBB44FF00UL:  //4
      IR_Flag = 4;
      break;
    case 0xE619FF00UL:  //0
      IR_Flag = 0;
      break;
    case 0xE916FF00UL:  //*
      IR_Flag = 6;
      break;
    case 0xF20DFF00UL:  //#
      IR_Flag = 7;
      break;
  }
}

//Line patrol procedure
void Tracking() {
  QTI_Max = digitalRead(QTI_L) * 2 + digitalRead(QTI_R);  //Black line return 1
  //Serial.println( QTI_Max);
  switch (QTI_Max) {
    case 3:
      Motor(160, 0, 160, 0);
      break;
    case 2:
      Motor(70, 0, 140, 0);
      break;
    case 1:
      Motor(140, 0, 70, 0);
      break;
    case 0:
      Motor(0, 120, 0, 120);
      break;
  }
}

//Line patrol - for testing

//Tracing procedure
void Find_light() {
  num_L = analogRead(Photodiode_L);
  num_R = analogRead(Photodiode_R);
  //    Serial.print("Left：");
  //    Serial.println(num_L);
  //    Serial.print("Right：");
  //    Serial.println(num_R);
  if (((num_L - num_R) > 250) && (num_R < 3700) && (num_L < 3700)) {  //Strong light on the right, right
    Motor(120, 0, 60, 0);
  } else if (((num_R - num_L) > 250) && (num_R < 3700) && (num_L < 3700)) {  //Strong light on the left, left
    Motor(60, 0, 120, 0);
  } else if ((((num_L - num_R) < 70) || ((num_R - num_L) < 70)) && (num_R < 3650) && (num_L < 3650)) {  //The light is straight ahead. Go straight
    Motor(120, 0, 120, 0);
  } else if (num_R > 3700 && num_L > 3700) {
    Motor(0, 0, 0, 0);
  }
}
void Find_light_ceshi() {
  num_L = analogRead(Photodiode_L);
  num_R = analogRead(Photodiode_R);
  Serial.print("Left：");
  Serial.println(num_L);
  Serial.print("Right：");
  Serial.println(num_R);
  if (((num_L - num_R) > 250) && (num_R < 3700) && (num_L < 3700)) {  //Strong light on the right, right
    //Motor(120, 0, 60, 0);
  } else if (((num_R - num_L) > 250) && (num_R < 3700) && (num_L < 3700)) {  //Strong light on the left, left
    //Motor(60, 0, 120, 0);
  } else if ((((num_L - num_R) < 70) || ((num_R - num_L) < 70)) && (num_R < 3650) && (num_L < 3650)) {  //The light is straight ahead. Go straight
    //Motor(120, 0, 120, 0);
  } else if (num_R > 3700 && num_L > 3700) {
    //Motor(0, 0, 0, 0);
  }
}


//Initialization program
void setup() {
  Serial.begin(115200);
  //Buzzer
  const uint8_t BUZZER_PIN  = 33;
const uint32_t BUZZER_FREQ = 5000;  // базовая ШИМ-частота, 5 кГц
const uint8_t  BUZZER_RES  = 8;     // 8‑бит разрешение

if (!ledcAttach(BUZZER_PIN, BUZZER_FREQ, BUZZER_RES)) {
  Serial.println("Ошибка: не удалось настроить LEDC для пина 33");
}
  //Patrol
  pinMode(QTI_L, INPUT);
  pinMode(QTI_R, INPUT);
  //RGB lamp
  FastLED.addLeds<WS2812, 25, GRB>(leds, 4);     //RGB lamp pin 25, the number of lamps is 4
  FastLED.addLeds<WS2812, 26, GRB>(RGBleds, 6);  //RGB lamp pin 25, the number of lamps is 4
  FastLED.setBrightness(0);                      //RGB lamp brightness range: 0-255
  FastLED.show();
  //wheel
  pinMode(12, OUTPUT);  //Left wheel forward
  pinMode(13, OUTPUT);  //Left wheel backward
  pinMode(14, OUTPUT);  //Right wheel forward
  pinMode(15, OUTPUT);  //Right wheel backward
  //BT Switch
  pinMode(32, INPUT);  //BT Switch
  //TEST
  pinMode(4, OUTPUT);
  pinMode(23, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(23, LOW);
  //OLED
  u8g2.begin();
  u8g2.enableUTF8Print();  // enable UTF8 support for the Arduino print() function
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.setFontDirection(0);
  //Bind wheels
  for (int pin = 12; pin <= 15; pin++) {  // Motor ports 12,13,14,15
  // Настраиваем каждый пин на 255 Гц, 8 бит
  if (!ledcAttach(pin, 255, 8)) {
    Serial.printf("Ошибка настройки LEDC на пине %d\n", pin);
    // можно добавить обработку ошибки, например, бесконечный цикл
  }
}
  //infrared remote control
  irrecv.enableIRIn();
  pinMode(RECV_PIN, INPUT);
  //BLE
  //Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P9);  //Set transmission frequency
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);  //Turn on scanning
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(80);

  xPlayMusicSemaphore = xSemaphoreCreateBinary();
  xDisconnectedSemaphore = xSemaphoreCreateBinary();
  xGrcCmdBuffer = xMessageBufferCreate(MAX_GRC_MSG_LEN * 5);
  xMusicQueue = xQueueCreate(20, sizeof(MusicNoteEvent));
  if (xMusicQueue == NULL) {
    Serial.println("Ошибка: не удалось создать xMusicQueue");
  } else {
    xTaskCreate(dance_task, "dance_task", 2048, NULL, 1, NULL);
  }
  xTaskCreate(play_music_task, "play_music_task", 1024, NULL, 1, NULL);
  xTaskCreate(grc_cmd_task, "grc_cmd_task", 4 * 1024, NULL, 1, NULL);
}


//while（1）
void loop() {
  //*************************************If Bluetooth switch -----on**************************************
  //*************************************If Bluetooth switch -----on**************************************
  //*************************************If Bluetooth switch -----on**************************************
  if (digitalRead(32) == LOW) {
    if (BLE_Close == 0) {
      ledcWrite(2, 0);  //Turn off the buzzer
      RGB_LED_Close();
      delay(30);
      RGB_LED6_Close();
      delay(20);
      Motor(0, 0, 0, 0);
      pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
      pBLEScan->setActiveScan(true);  //Turn on scanning
      pBLEScan->setInterval(100);
      pBLEScan->setWindow(99);
      BLE_Close = 1;
    }

    // Scan if needed
    if (doSacn == true) {
      Serial.println("Start looking for Bluetooth devices...");
      //Serial.println("Start searching for devices");
      BLEDevice::getScan()->clearResults();
      BLEDevice::getScan()->start(1);  // Continuously search for devices
    }

    //If the connection is successful
    if (doConnect == true) {
      if (ConnectToServer() == true) {
        //Serial.println("We are now connecting to the server...");
        Serial.println("We are now connecting to the server...");
        Serial.println("3");
        delay(500);
        Serial.println("2");
        delay(500);
        Serial.println("1");
        delay(500);
        Serial.println("The connection is successful...");

        FastLED.setBrightness(255);  //RGB lamp brightness range: 0-255
        fill_solid(leds, 4, CRGB::Black);
        fill_solid(RGBleds, 6, CRGB::Black);
        FastLED.show();
        Motor(0, 0, 0, 0);
        ledcWrite(2, 0);
        connected = true;
      } else {
        doSacn = true;
      }
      doConnect = false;
    }

    //If the device is connected
    if (connected == true) {
      xSemaphoreTake(xDisconnectedSemaphore, portMAX_DELAY);
    }

  } else {
    //*********************************normal mode***************************************
  
    //If the Bluetooth switch ----- is off
    if (BLE_Close == 1) {
      BLE_Close = 0;
      RGB_LED_Close();
      delay(5);
      RGB_LED6_Close();
      delay(5);
      Motor(0, 0, 0, 0);
      pBLEScan->setActiveScan(false);  //Stop the scan
      doConnect = false;
      connected = false;
      doSacn = true;
      pClient->disconnect();
    }
    //infrared remote control
    Close_Flag = 1;
    while (Close_Flag == 1 && digitalRead(32) == HIGH) {
      if (irrecv.decode()) {
        irrecv.printIRResultShort(&Serial); // Print complete received data in one line
        if (irrecv.decodedIRData.decodedRawData, HEX == 0xFFFFFFFFF) {
          state_N = state_L;
        } else {
          state_N = irrecv.decodedIRData.decodedRawData, HEX;
          state_L = irrecv.decodedIRData.decodedRawData, HEX;
        }

        IR_remote_control();
        switch (IR_Flag) {
          case 11:  //Forward
            Motor(240, 0, 240, 0);
            break;
          case 22:  //retreat
            Motor(0, 220, 0, 220);
            break;
          case 33:  //turn left
            Motor(0, 180, 180, 0);
            break;
          case 44:  //turn right
            Motor(180, 0, 0, 180);
            break;
          case 55:  //stop
            Motor(0, 0, 0, 0);
            ledcWriteTone(2, Tone[3]);  //Buzzer
            delay(150);
            ledcWrite(2, 0);
            break;



          case 1:  //*********************1********************
            Serial.println("Turn on ultrasonic mode");
            Close_Flag = 1;
            if (shock_Flag == 0) {
              ledcWriteTone(2, Tone[3]);  //Buzzer
              delay(300);
              ledcWrite(2, 0);  
              delay(700);
              shock_Flag = 1;
            }
            while (Close_Flag == 1) {
              RGB_LED6();  //Ultrasonic RGB lamp
              delay(40);
              Get_Distance();
              Serial.print("Ultrasonic wave distance：");
              Serial.print(distance);
              Serial.print("cm");
              Serial.println();
              if (distance < 30) {
                Motor(220, 0, 0, 220);
              } else {
                Motor(220, 0, 220, 0);
              }
              delay(20);
              if (irrecv.decode()) {
                if (irrecv.decodedIRData.decodedRawData, HEX == 0xFFFFFFFFF) {
                  state_N = state_L;
                } else {
                  state_N = irrecv.decodedIRData.decodedRawData, HEX;
                  state_L = irrecv.decodedIRData.decodedRawData, HEX;
                }
                IR_remote_control();
                switch (IR_Flag) {
                  case 7:  //Buzzer
                    Serial.println("Trun on Buzzer");
                    ledcWriteTone(2, Tone[3]);  //Buzzer
                    delay(150);
                    ledcWrite(2, 0);
                    break;
                }
                irrecv.resume();
              }
              if (IR_Flag == 6 || IR_Flag == 7)  //Restore to 1 after one run
              {
                IR_Flag = 1;
              }
              if (IR_Flag == 55)  //Exit
              {
                Close_Flag = 0;
                break;
              }
            }
            if (IR_Flag == 55)  //Exit
            {
              IR_Flag = 88;
              shock_Flag = 0;
              Close_Flag = 0;
              ledcWrite(2, 0);
              RGB_LED6_Close();  //Ultrasonic RGB lamp off
              delay(15);
              RGB_LED_Close();  //Bottom RGB light off
              delay(15);
              Motor(0, 0, 0, 0);  //Close the steering gear
              Serial.println("Exit ultrasonic mode");
            }
            break;

          case 2:  //*********************2********************************
            Serial.println("Turn on the patrol mode");
            Close_Flag = 1;
            if (shock_Flag == 0) {
              ledcWriteTone(2, Tone[3]);  //Buzzer
              delay(300);
              ledcWrite(2, 0);  //trun off Buzzer
              delay(700);
              shock_Flag = 1;
            }
            while (Close_Flag == 1) {
              Tracking();  //line patrol
              delay(2);
              if (irrecv.decode()) {
                if (irrecv.decodedIRData.decodedRawData, HEX == 0xFFFFFFFFF) {
                  state_N = state_L;
                } else {
                  state_N = irrecv.decodedIRData.decodedRawData, HEX;
                  state_L = irrecv.decodedIRData.decodedRawData, HEX;
                }
                IR_remote_control();
                switch (IR_Flag) {
                  case 6:  //Ultrasonic RGB lamp
                    L += 1;
                    if (L % 2 == 1) {
                      RGB_LED6_norandom();  //turn on Ultrasonic RGB lamp
                      delay(30);
                      Serial.println("Turn on ultrasonic RGB");
                    } else {
                      RGB_LED6_Close();  //Ultrasonic RGB lamp off
                      delay(30);
                      Serial.println("Turn off ultrasonic RGB");
                    }
                    break;
                  case 7:  //Buzzer
                    Serial.println("trun on Buzzer");
                    ledcWriteTone(2, Tone[3]);  //Buzzer
                    delay(150);
                    ledcWrite(2, 0);
                    break;
                }
                irrecv.resume();
              }
              if (IR_Flag == 6 || IR_Flag == 7)  //Run this command once and restore to 2
              {
                IR_Flag = 2;
              }
              if (IR_Flag == 55)  //Exit
              {
                Close_Flag = 0;
                break;
              }
            }
            if (IR_Flag == 55)  //Exit
            {
              IR_Flag = 88;
              shock_Flag = 0;
              Close_Flag = 0;
              ledcWrite(2, 0);
              RGB_LED6_Close();  //Ultrasonic RGB lamp off
              delay(15);
              RGB_LED_Close();  //Bottom RGB light off
              delay(15);
              Motor(0, 0, 0, 0);  //Close the steering gear
              Serial.println("Exit Line patrol mode");
            }
            break;

          case 3:  //*********************3********************************
            Serial.println("Enable light seeking mode");
            Close_Flag = 1;
            if (shock_Flag == 0) {
              ledcWriteTone(2, Tone[3]);  //Buzzer
              delay(300);
              ledcWrite(2, 0);  //Trun off Buzzer
              delay(700);
              shock_Flag = 1;
              //ticker.attach(2, callback1);   //callback1 is called every second
            }
            while (Close_Flag == 1) {
              Find_light();
              if (irrecv.decode()) {
                if (irrecv.decodedIRData.decodedRawData, HEX == 0xFFFFFFFFF) {
                  state_N = state_L;
                } else {
                  state_N = irrecv.decodedIRData.decodedRawData, HEX;
                  state_L = irrecv.decodedIRData.decodedRawData, HEX;
                }
                IR_remote_control();
                switch (IR_Flag) {
                  case 6:  //Ultrasonic RGB lamp
                    L += 1;
                    if (L % 2 == 1) {
                      RGB_LED6_norandom();  //turn on Ultrasonic RGB lamp
                      delay(30);
                      Serial.println("Turn on ultrasonic RGB");
                    } else {
                      RGB_LED6_Close();  //Ultrasonic RGB lamp off
                      delay(30);
                      Serial.println("Turn off ultrasonic RGB");
                    }
                    break;
                  case 7:  //Buzzer
                    Serial.println("Trun on Buzzer");
                    ledcWriteTone(2, Tone[3]);  //Buzzer
                    delay(150);
                    ledcWrite(2, 0);
                    break;
                }
                irrecv.resume();
              }
              if (IR_Flag == 6 || IR_Flag == 7)  //Run this command once and restore to 2
              {
                IR_Flag = 3;
              }
              if (IR_Flag == 55)  //Exit
              {
                Close_Flag = 0;
                break;
              }
            }
            if (IR_Flag == 55)  //Exit
            {
              IR_Flag = 88;
              shock_Flag = 0;
              Close_Flag = 0;
              ledcWrite(2, 0);
              RGB_LED6_Close();  //Ultrasonic RGB lamp off
              delay(15);
              RGB_LED_Close();  //Bottom RGB light off
              delay(15);
              Motor(0, 0, 0, 0);  //Close the steering gear
              Serial.println("Exit Indicates the optical search mode");
            }
            break;

          case 4:  //*********************4********************************
            Serial.println("Enable RGB and sing modes");
            Close_Flag = 1;
            if (shock_Flag == 0) {
              ledcWriteTone(2, Tone[3]);  //Buzzer
              delay(300);
              ledcWrite(2, 0); 
              delay(700);
              shock_Flag = 1;
            }
            while (Close_Flag == 1) {
              for (int i = 0; i < Tone_length; i++) {          //Buzzer
                RGB_LED();                                     //Backplane RGB lamp
                RGB_LED6();                                    //Ultrasonic RGB lamp
                ledcWriteTone(2, Tone[i]);                     //Buzzer
                for (int j = 0; j < (Time[i] / 0.020); j++) {  //Buzzer
                  delay(20);
                  if (irrecv.decode()) {
                    if (irrecv.decodedIRData.decodedRawData, HEX == 0xFFFFFFFFF) {
                      state_N = state_L;
                    } else {
                      state_N = irrecv.decodedIRData.decodedRawData, HEX;
                      state_L = irrecv.decodedIRData.decodedRawData, HEX;
                    }
                    IR_remote_control();
                    if (IR_Flag == 55)  //Exit
                    {
                      Close_Flag = 0;
                      break;
                    }
                    irrecv.resume();
                  }
                }
                if (irrecv.decode()) {
                  if (irrecv.decodedIRData.decodedRawData, HEX == 0xFFFFFFFFF) {
                    state_N = state_L;
                  } else {
                    state_N = irrecv.decodedIRData.decodedRawData, HEX;
                    state_L = irrecv.decodedIRData.decodedRawData, HEX;
                  }
                  IR_remote_control();
                  if (IR_Flag == 55)  //Exit
                  {
                    Close_Flag = 0;
                    ledcWrite(2, 0);
                    RGB_LED6_Close();  //Ultrasonic RGB lamp off
                    delay(15);
                    RGB_LED_Close();  //Bottom RGB light off
                    delay(15);
                    break;
                  }
                  irrecv.resume();
                }
              }
              ledcWrite(2, 0);  //trun off Buzzer
              if (irrecv.decode()) {
                if (irrecv.decodedIRData.decodedRawData, HEX == 0xFFFFFFFFF) {
                  state_N = state_L;
                } else {
                  state_N = irrecv.decodedIRData.decodedRawData, HEX;
                  state_L = irrecv.decodedIRData.decodedRawData, HEX;
                }
                IR_remote_control();
                if (IR_Flag == 55)  //Exit
                {
                  Close_Flag = 0;
                  ledcWrite(2, 0);
                  RGB_LED6_Close();  //Ultrasonic RGB lamp off
                  delay(15);
                  RGB_LED_Close();  
                  delay(15);
                  break;
                }
                irrecv.resume();
              }
            }
            if (IR_Flag == 55)  //Exit
            {
              IR_Flag = 88;
              shock_Flag = 0;
              Close_Flag = 0;
              ledcWrite(2, 0);
              RGB_LED6_Close();  //Ultrasonic RGB lamp off
              delay(15);
              RGB_LED_Close();  
              delay(15);
              Motor(0, 0, 0, 0); 
              Serial.println("Exit RGB and sing modes");
            }
            break;

          case 0:  //*********************0********************************
            Serial.println("Enable the 4-in-1 mode");
            Close_Flag = 1;
            if (shock_Flag == 0) {
              ledcWriteTone(2, Tone[3]);  //Buzzer
              delay(300);
              ledcWrite(2, 0);  //关闭Buzzer
              delay(700);
              shock_Flag = 1;
            }
            while (Close_Flag == 1) {
              for (int i = 0; i < Tone_length; i++) {          //Buzzer
                RGB_LED();                                     //Backplane RGB lamp
                RGB_LED6();                                    //Ultrasonic RGB lamp
                ledcWriteTone(2, Tone[i]);                     //Buzzer
                for (int j = 0; j < (Time[i] / 0.020); j++) {  //Buzzer
                  Get_Distance();
                  Serial.print("Ultrasonic wave distance：");
                  Serial.println(distance);
                  if (distance < 30) {
                    Motor(0, 0, 0, 0);
                  } else {
                    Tracking();  
                  }
                  delay(20);
                  if (irrecv.decode()) {
                    if (irrecv.decodedIRData.decodedRawData, HEX == 0xFFFFFFFFF) {
                      state_N = state_L;
                    } else {
                      state_N = irrecv.decodedIRData.decodedRawData, HEX;
                      state_L = irrecv.decodedIRData.decodedRawData, HEX;
                    }
                    IR_remote_control();
                    if (IR_Flag == 55)  //Exit
                    {
                      Close_Flag = 0;
                      break;
                    }
                    irrecv.resume();
                  }
                }
                if (irrecv.decode()) {
                  if (irrecv.decodedIRData.decodedRawData, HEX == 0xFFFFFFFFF) {
                    state_N = state_L;
                  } else {
                    state_N = irrecv.decodedIRData.decodedRawData, HEX;
                    state_L = irrecv.decodedIRData.decodedRawData, HEX;
                  }
                  IR_remote_control();
                  if (IR_Flag == 55)  //Exit
                  {
                    Close_Flag = 0;
                    ledcWrite(2, 0);
                    RGB_LED6_Close();  //Ultrasonic RGB lamp off
                    delay(10);
                    RGB_LED_Close();  
                    delay(10);
                    break;
                  }
                  irrecv.resume();
                }
              }
              ledcWrite(2, 0); 
              if (irrecv.decode()) {
                if (irrecv.decodedIRData.decodedRawData, HEX == 0xFFFFFFFFF) {
                  state_N = state_L;
                } else {
                  state_N = irrecv.decodedIRData.decodedRawData, HEX;
                  state_L = irrecv.decodedIRData.decodedRawData, HEX;
                }
                IR_remote_control();
                if (IR_Flag == 55)  //Exit
                {
                  Close_Flag = 0;
                  ledcWrite(2, 0);
                  RGB_LED6_Close();  //Ultrasonic RGB lamp off
                  delay(15);
                  RGB_LED_Close();  
                  delay(15);
                  break;
                }
                irrecv.resume();
              }
            }
            if (IR_Flag == 55)  //Exit
            {
              IR_Flag = 88;
              shock_Flag = 0;
              Close_Flag = 0;
              ledcWrite(2, 0);
              RGB_LED6_Close();  //Ultrasonic RGB lamp off
              delay(15);
              RGB_LED_Close(); 
              delay(15);
              Motor(0, 0, 0, 0);  
              Serial.println("Exit 4 in 1 mode");
            }
            break;

          case 6:  //**********************6********************************
            Serial.println("Open the light");
            L += 1;
            if (L % 2 == 1) {
              RGB_LED6();  //Ultrasonic RGB lamp
              delay(50);
              Serial.println("Turn on ultrasonic RGB");
            } else {
              RGB_LED6_Close();  //Ultrasonic RGB lamp off
              delay(50);
              Serial.println("Turn off ultrasonic RGB");
            }
            break;

          case 7:  //**********************7********************************
            Serial.println("turn on Buzzer");
            ledcWriteTone(2, Tone[3]);  //Buzzer
            break;
          default:
            Motor(0, 0, 0, 0);
            break;
        }

        irrecv.resume();
      } else {
        Motor(0, 0, 0, 0);
        ledcWrite(2, 0);
      }
      delay(150);

      
    }
  }
}
