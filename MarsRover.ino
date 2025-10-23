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
const int melodyCount = sizeof(melodies) / sizeof(melodies[0]);
QueueHandle_t xMusicQueue;
#define MAX_GRC_MSG_LEN 20

#define MOTOR_MIN_VALUE 15
#define MOTOR_MAX_VALUE 255

// --------- Racing tuning API ----------
struct RaceParams {
  float max_speed_scale = 1.0f;    // общий множитель скорости (1.0 = дефолт)
  float turn_scale = 1.0f;         // множитель для поворота (1.0 = дефолт)
  float accel_coeff = 1.0f;        // коэффициент ускорения (новая переменная, 1.0 = дефолт)
};
RaceParams race; // текущие параметры
// ---------- ADD AFTER RaceParams definition ----------
constexpr float MAX_SPEED_MIN = 0.5f;
constexpr float MAX_SPEED_MAX = 1.6f;

constexpr float TURN_MIN = 0.6f;
constexpr float TURN_MAX = 2.0f;

constexpr float ACCEL_MIN = 0.5f;
constexpr float ACCEL_MAX = 2.0f;

void presetSpeedDemon() {
  race.max_speed_scale = 1.4f;
  race.turn_scale = 1.05f;
  race.accel_coeff = 1.5f;
}

void presetDriftKing() {
  race.max_speed_scale = 1.1f;
  race.turn_scale = 1.6f;
  race.accel_coeff = 1.2f;
}

void presetSmoothRacer() {
  race.max_speed_scale = 0.9f;
  race.turn_scale = 0.9f;
  race.accel_coeff = 0.8f;
}

void presetEcoMode() {
  race.max_speed_scale = 0.7f;
  race.turn_scale = 1.0f;
  race.accel_coeff = 0.6f;
}

// Немного удобный clamp для чисел
template<typename T>
T clampVal(T v, T lo, T hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}

// Инициализация по умолчанию (дефолтные значения)
void initRaceDefaults() {
  race.max_speed_scale = 1.0f;
  race.turn_scale = 1.0f;
  race.accel_coeff = 1.0f;
}
// Добавьте в глобальные переменные
SemaphoreHandle_t xSerialSemaphore;



// Создайте функцию для безопасного вывода
void safePrint(const char* format, ...) {
  if (xSerialSemaphore != NULL) {
    if (xSemaphoreTake(xSerialSemaphore, portMAX_DELAY) == pdTRUE) {
      va_list args;
      va_start(args, format);
      vprintf(format, args);
      va_end(args);
      xSemaphoreGive(xSerialSemaphore);
    }
  }
}

// Создайте функцию для безопасного вывода строки
void safePrintln(const char* message) {
  if (xSerialSemaphore != NULL) {
    if (xSemaphoreTake(xSerialSemaphore, portMAX_DELAY) == pdTRUE) {
      Serial.println(message);
      xSemaphoreGive(xSerialSemaphore);
    }
  }
}

// моторный рампинг с учетом accel_coeff
static int target_lf=0, target_lb=0, target_rf=0, target_rb=0;
static int cur_lf=0, cur_lb=0, cur_rf=0, cur_rb=0;
static unsigned long last_ramp_ts = 0;
const int RAMP_INTERVAL_MS = 20;  // интервал обновления рампа

// Установить целевые значения моторов (с рампингом)
void setTargetMotor(int lf, int lb, int rf, int rb) {
  target_lf = lf;
  target_lb = lb;
  target_rf = rf;
  target_rb = rb;
  last_ramp_ts = millis();
}

// Обновление моторов с рампингом (вызывать в loop или задаче)
void updateMotorRamp() {
  unsigned long now = millis();
  if (now - last_ramp_ts >= RAMP_INTERVAL_MS) {
    float step = race.accel_coeff * (MOTOR_MAX_VALUE / 10.0f);  // шаг зависит от accel_coeff
    cur_lf += (target_lf > cur_lf) ? step : -step;
    cur_lb += (target_lb > cur_lb) ? step : -step;
    cur_rf += (target_rf > cur_rf) ? step : -step;
    cur_rb += (target_rb > cur_rb) ? step : -step;

    cur_lf = clampVal(cur_lf, 0, MOTOR_MAX_VALUE);
    cur_lb = clampVal(cur_lb, 0, MOTOR_MAX_VALUE);
    cur_rf = clampVal(cur_rf, 0, MOTOR_MAX_VALUE);
    cur_rb = clampVal(cur_rb, 0, MOTOR_MAX_VALUE);

    Motor(cur_lf, cur_lb, cur_rf, cur_rb);
    last_ramp_ts = now;
  }
}

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

static bool moving_backwards = false;
void buzzer_callback()  //Callback function
{
  if (moving_backwards) {
    ledcWriteTone(2, G4);  //Buzzer
    delay(150);
    ledcWrite(2, 0);
  }
}
void adjustRaceFloat(float &field, float delta, float lo, float hi, const char *name) {
  field = clampVal(field + delta, lo, hi);
  Serial.printf("%s -> %.3f\n", name, field);
  // TODO: persist to Preferences if нужно
}

void adjustRaceInt(int &field, int delta, int lo, int hi, const char *name) {
  field = clampVal(field + delta, lo, hi);
  Serial.printf("%s -> %d\n", name, field);
  // TODO: persist to Preferences если нужно
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
// Задача логирования дистанции (вставьте глобально)
void distance_log_task(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // Посылаем измерение
    Get_Distance();
    // даём прерываниям сработать и distance стабилизироваться
    vTaskDelay(pdMS_TO_TICKS(60)); // 60 ms — достаточно для получения эха
    int d = distance; // считываем volatile в локальную переменную для атомарности
    //safePrint("ULTRASONIC distance = %d cm\n", d); 
    // ждать оставшуюся часть секунды
    vTaskDelay(pdMS_TO_TICKS(940)); // 940 ms -> ~1 с общий период
  }
}
// Вспомогательная функция: поворот вправо на указанное количество градусов.
// Делает timed turn используя turnRightMs; опирается на calibr TURN_LARGE_MS == 90°
///
/// Параметры:
///   deg - градусы (например 30)
///   turnSpeed - мощность (по умолчанию TURN_SPEED)

// ------------------- TURN HELPERS & SIMPLE BOX-BYPASS ALGORITHM -------------------
// ---------- TUNABLE PARAMETERS ----------
int TURN_DEG = 46;                      // empirical "check" angle (adjust)
constexpr int TURN_LARGE_MS = 350;      // ms for ~90° (calibrate)
constexpr int BASE_TURN_POWER = 120;    // base turning power (0..MOTOR_MAX_VALUE)

// delays/timings (increased for stable turns)
constexpr int TURN_POST_MS = 200;       // post-turn stabilization delay
constexpr int TURN_MIN_MS = 40;         // min turn time safety

// mutex for critical sections used by bypass/learn/replay tasks
static portMUX_TYPE bypassMux = portMUX_INITIALIZER_UNLOCKED;

// --- structures for storing learned trajectory (per-side) ---
enum ActionType : uint8_t { ACT_FORWARD = 0, ACT_TURN = 1 };
enum TurnDir : int8_t { DIR_LEFT = -1, DIR_RIGHT = 1 };

struct LearnedAction {
  ActionType type;
  int duration_ms; // for ACT_FORWARD
  int deg;         // for ACT_TURN
  int8_t dir;      // DIR_LEFT / DIR_RIGHT
};

static constexpr int MAX_LEARNED_ACTIONS = 32;

// Right-bypass buffer
static LearnedAction learnedActionsRight[MAX_LEARNED_ACTIONS];
static int learnedCountRight = 0;
static bool learnedAvailableRight = false;

// Left-bypass buffer
static LearnedAction learnedActionsLeft[MAX_LEARNED_ACTIONS];
static int learnedCountLeft = 0;
static bool learnedAvailableLeft = false;

// --- task handles and activity flags ---
static TaskHandle_t learnRightTaskHandle = NULL;
static bool learnRightActive = false;

static TaskHandle_t learnLeftTaskHandle = NULL;
static bool learnLeftActive = false;

static TaskHandle_t replayRightTaskHandle = NULL;
static bool replayRightActive = false;

static TaskHandle_t replayLeftTaskHandle = NULL;
static bool replayLeftActive = false;

// --- helpers for clearing / adding records (separate for each side) ---
static void clearLearnedTrajectoryRight() {
  taskENTER_CRITICAL(&bypassMux);
  learnedCountRight = 0;
  learnedAvailableRight = false;
  taskEXIT_CRITICAL(&bypassMux);
}

static void clearLearnedTrajectoryLeft() {
  taskENTER_CRITICAL(&bypassMux);
  learnedCountLeft = 0;
  learnedAvailableLeft = false;
  taskEXIT_CRITICAL(&bypassMux);
}

static void addLearnedTurnRight(int8_t dir, int deg) {
  taskENTER_CRITICAL(&bypassMux);
  if (learnedCountRight < MAX_LEARNED_ACTIONS) {
    learnedActionsRight[learnedCountRight].type = ACT_TURN;
    learnedActionsRight[learnedCountRight].dir = dir;
    learnedActionsRight[learnedCountRight].deg = deg;
    learnedActionsRight[learnedCountRight].duration_ms = 0;
    learnedCountRight++;
  } else {
    safePrintln("[LEARN RIGHT] buffer full, can't add TURN");
  }
  taskEXIT_CRITICAL(&bypassMux);
}

static void addLearnedForwardRight(int ms) {
  taskENTER_CRITICAL(&bypassMux);
  if (learnedCountRight < MAX_LEARNED_ACTIONS) {
    learnedActionsRight[learnedCountRight].type = ACT_FORWARD;
    learnedActionsRight[learnedCountRight].duration_ms = ms;
    learnedActionsRight[learnedCountRight].deg = 0;
    learnedActionsRight[learnedCountRight].dir = 0;
    learnedCountRight++;
  } else {
    safePrintln("[LEARN RIGHT] buffer full, can't add FORWARD");
  }
  taskEXIT_CRITICAL(&bypassMux);
}

static void markLearnedAvailableRight() {
  taskENTER_CRITICAL(&bypassMux);
  learnedAvailableRight = (learnedCountRight > 0);
  taskEXIT_CRITICAL(&bypassMux);
}

static void addLearnedTurnLeft(int8_t dir, int deg) {
  taskENTER_CRITICAL(&bypassMux);
  if (learnedCountLeft < MAX_LEARNED_ACTIONS) {
    learnedActionsLeft[learnedCountLeft].type = ACT_TURN;
    learnedActionsLeft[learnedCountLeft].dir = dir;
    learnedActionsLeft[learnedCountLeft].deg = deg;
    learnedActionsLeft[learnedCountLeft].duration_ms = 0;
    learnedCountLeft++;
  } else {
    safePrintln("[LEARN LEFT] buffer full, can't add TURN");
  }
  taskEXIT_CRITICAL(&bypassMux);
}

static void addLearnedForwardLeft(int ms) {
  taskENTER_CRITICAL(&bypassMux);
  if (learnedCountLeft < MAX_LEARNED_ACTIONS) {
    learnedActionsLeft[learnedCountLeft].type = ACT_FORWARD;
    learnedActionsLeft[learnedCountLeft].duration_ms = ms;
    learnedActionsLeft[learnedCountLeft].deg = 0;
    learnedActionsLeft[learnedCountLeft].dir = 0;
    learnedCountLeft++;
  } else {
    safePrintln("[LEARN LEFT] buffer full, can't add FORWARD");
  }
  taskEXIT_CRITICAL(&bypassMux);
}

static void markLearnedAvailableLeft() {
  taskENTER_CRITICAL(&bypassMux);
  learnedAvailableLeft = (learnedCountLeft > 0);
  taskEXIT_CRITICAL(&bypassMux);
}

// --- compute turning power (unchanged) ---
static int computeTurnPowerFromRace() {
  float ts = clampVal(race.turn_scale, TURN_MIN, TURN_MAX);
  int p = int(BASE_TURN_POWER * ts + 0.5f);
  if (p < 0) p = 0;
  if (p > MOTOR_MAX_VALUE) p = MOTOR_MAX_VALUE;
  return p;
}

// Low-level timed turns
void turnRightMs(int ms, int power) {
  if (ms <= 0) return;
  setTargetMotor(power, 0, 0, power);
  vTaskDelay(pdMS_TO_TICKS(ms));
  setTargetMotor(0,0,0,0);
  vTaskDelay(pdMS_TO_TICKS(TURN_POST_MS));
}

void turnLeftMs(int ms, int power) {
  if (ms <= 0) return;
  setTargetMotor(0, power, power, 0);
  vTaskDelay(pdMS_TO_TICKS(ms));
  setTargetMotor(0,0,0,0);
  vTaskDelay(pdMS_TO_TICKS(TURN_POST_MS));
}

void turnRightDeg(int deg) {
  if (deg <= 0) return;
  int ms = (deg * TURN_LARGE_MS) / 90;
  if (ms < TURN_MIN_MS) ms = TURN_MIN_MS;
  int power = computeTurnPowerFromRace();
  turnRightMs(ms, power);
}

void turnLeftDeg(int deg) {
  if (deg <= 0) return;
  int ms = (deg * TURN_LARGE_MS) / 90;
  if (ms < TURN_MIN_MS) ms = TURN_MIN_MS;
  int power = computeTurnPowerFromRace();
  turnLeftMs(ms, power);
}

// ------------------ LEARN BYPASS RIGHT (learning, uses front sensor) ------------------
void learn_bypass_right_task(void *pvParameters) {
  (void) pvParameters;

  // guard: don't start if already active
  taskENTER_CRITICAL(&bypassMux);
  if (learnRightActive) {
    safePrintln("LEARN RIGHT: start requested but already active -> exiting task");
    learnRightTaskHandle = NULL;
    taskEXIT_CRITICAL(&bypassMux);
    vTaskDelete(NULL);
    return;
  }
  learnRightActive = true;
  learnRightTaskHandle = xTaskGetCurrentTaskHandle();
  taskEXIT_CRITICAL(&bypassMux);

  // clear previous right trajectory
  clearLearnedTrajectoryRight();

  const int STOP_THRESHOLD_CM = 13;
  const int SAFE_CLEAR_CM = 35;
  const int MOVE_SPEED = 60;
  const int FORWARD_STEP_MS = 700;
  const int FORWARD_AFTER_PASS_MS = 300;
  const int DIST_CHECK_DELAY_MS = 120;
  const float SAME_DISTANCE_MULT = 1.5f;
  const int MAX_TOTAL_ATTEMPTS = 200;

  safePrintln("LEARN RIGHT: simple box bypass learning started");

  int sides_passed = 0;
  bool obstacle_detected = false;

  // approach
  while (learnRightActive && !obstacle_detected) {
    Get_Distance();
    vTaskDelay(pdMS_TO_TICKS(DIST_CHECK_DELAY_MS));
    int front = distance;
    safePrint("[LEARN RIGHT] approaching: front=%d cm, sides_passed=%d\n", front, sides_passed);

    if (front > STOP_THRESHOLD_CM) {
      setTargetMotor(MOVE_SPEED, 0, MOVE_SPEED, 0);
      vTaskDelay(pdMS_TO_TICKS(80));
      continue;
    }

    obstacle_detected = true;
    safePrintln("[LEARN RIGHT] obstacle detected -> start bypassing");

    int initial_front = (front > 0) ? front : 1;

    setTargetMotor(0,0,0,0);
    vTaskDelay(pdMS_TO_TICKS(200));

    // turn RIGHT to become parallel (right-bypass)
    safePrint("[LEARN RIGHT] turning RIGHT (TURN_DEG=%d) to become parallel\n", TURN_DEG);
    turnRightDeg(TURN_DEG);
    vTaskDelay(pdMS_TO_TICKS(80 + TURN_POST_MS / 2));

    // main bypass loop
    int attempts = 0;
    int acc_forward_ms = 0;
    while (learnRightActive && sides_passed < 4 && attempts < MAX_TOTAL_ATTEMPTS) {
      attempts++;
      safePrint("[LEARN RIGHT] bypass attempt %d\n", attempts);

      // small forward step
      setTargetMotor(MOVE_SPEED, 0, MOVE_SPEED, 0);
      vTaskDelay(pdMS_TO_TICKS(FORWARD_STEP_MS));
      setTargetMotor(0,0,0,0);
      acc_forward_ms += FORWARD_STEP_MS;
      vTaskDelay(pdMS_TO_TICKS(100));

      // test by turning LEFT (check forward)
      safePrint("[LEARN RIGHT] turning LEFT (TURN_DEG=%d) to check forward distance\n", TURN_DEG);
      turnLeftDeg(TURN_DEG);
      vTaskDelay(pdMS_TO_TICKS(80 + TURN_POST_MS / 2));

      Get_Distance();
      vTaskDelay(pdMS_TO_TICKS(DIST_CHECK_DELAY_MS));
      int check = distance;
      safePrint("[LEARN RIGHT] check after left turn: %d cm (initial=%d)\n", check, initial_front);

      if ((float)check <= SAME_DISTANCE_MULT * (float)initial_front) {
        safePrintln("[LEARN RIGHT] still close -> return to parallel and continue");
        turnRightDeg(TURN_DEG); // return parallel
        vTaskDelay(pdMS_TO_TICKS(80 + TURN_POST_MS / 2));
        continue;
      }

      if (check >= SAFE_CLEAR_CM || (float)check >= 2.0f * (float)initial_front) {
        safePrint("[LEARN RIGHT] side appears cleared (check=%d) -> passing corner\n", check);

        int total_forward_ms = acc_forward_ms + FORWARD_AFTER_PASS_MS;
        addLearnedForwardRight(total_forward_ms);
        // record the turn we actually performed to change direction: LEFT
        addLearnedTurnRight(DIR_LEFT, TURN_DEG);

        acc_forward_ms = 0;
        sides_passed++;

        // drive forward after corner
        setTargetMotor(MOVE_SPEED, 0, MOVE_SPEED, 0);
        vTaskDelay(pdMS_TO_TICKS(FORWARD_AFTER_PASS_MS));
        setTargetMotor(0,0,0,0);
        vTaskDelay(pdMS_TO_TICKS(100));
        safePrint("[LEARN RIGHT] side cleared. total sides_passed=%d\n", sides_passed);
        continue;
      }

      // ambiguous -> return to parallel and retry
      safePrint("[LEARN RIGHT] ambiguous result (check=%d) -> return to parallel and try again\n", check);
      turnRightDeg(TURN_DEG);
      vTaskDelay(pdMS_TO_TICKS(80 + TURN_POST_MS / 2));
    }

    // check completion
    if (sides_passed < 4) {
      safePrint("[LEARN RIGHT] failed to clear all sides after %d attempts -> aborting\n", attempts);
      ledcWriteTone(2, 400);
      vTaskDelay(pdMS_TO_TICKS(600));
      ledcWrite(2, 0);
      taskENTER_CRITICAL(&bypassMux);
      learnRightActive = false;
      taskEXIT_CRITICAL(&bypassMux);
    } else {
      safePrintln("[LEARN RIGHT] learning complete - sides_passed == 4");
      markLearnedAvailableRight();
    }
  } // end approach

  setTargetMotor(0,0,0,0);
  safePrint("LEARN RIGHT: finished simple box bypass; sides_passed=%d\n", sides_passed);
  if (sides_passed >= 4) {
    ledcWriteTone(2, 900);
    vTaskDelay(pdMS_TO_TICKS(200));
    ledcWrite(2, 0);
  } else {
    ledcWriteTone(2, 400);
    vTaskDelay(pdMS_TO_TICKS(200));
    ledcWrite(2, 0);
  }

  taskENTER_CRITICAL(&bypassMux);
  learnRightActive = false;
  learnRightTaskHandle = NULL;
  taskEXIT_CRITICAL(&bypassMux);

  vTaskDelete(NULL);
}

// ------------------ LEARN BYPASS LEFT (mirror of right) ------------------
void learn_bypass_left_task(void *pvParameters) {
  (void) pvParameters;

  taskENTER_CRITICAL(&bypassMux);
  if (learnLeftActive) {
    safePrintln("LEARN LEFT: start requested but already active -> exiting task");
    learnLeftTaskHandle = NULL;
    taskEXIT_CRITICAL(&bypassMux);
    vTaskDelete(NULL);
    return;
  }
  learnLeftActive = true;
  learnLeftTaskHandle = xTaskGetCurrentTaskHandle();
  taskEXIT_CRITICAL(&bypassMux);

  clearLearnedTrajectoryLeft();

  const int STOP_THRESHOLD_CM = 13;
  const int SAFE_CLEAR_CM = 35;
  const int MOVE_SPEED = 60;
  const int FORWARD_STEP_MS = 700;
  const int FORWARD_AFTER_PASS_MS = 300;
  const int DIST_CHECK_DELAY_MS = 120;
  const float SAME_DISTANCE_MULT = 1.5f;
  const int MAX_TOTAL_ATTEMPTS = 200;

  safePrintln("LEARN LEFT: simple box bypass learning started");

  int sides_passed = 0;
  bool obstacle_detected = false;

  // approach
  while (learnLeftActive && !obstacle_detected) {
    Get_Distance();
    vTaskDelay(pdMS_TO_TICKS(DIST_CHECK_DELAY_MS));
    int front = distance;
    safePrint("[LEARN LEFT] approaching: front=%d cm, sides_passed=%d\n", front, sides_passed);

    if (front > STOP_THRESHOLD_CM) {
      setTargetMotor(MOVE_SPEED, 0, MOVE_SPEED, 0);
      vTaskDelay(pdMS_TO_TICKS(80));
      continue;
    }

    obstacle_detected = true;
    safePrintln("[LEARN LEFT] obstacle detected -> start bypassing");

    int initial_front = (front > 0) ? front : 1;

    setTargetMotor(0,0,0,0);
    vTaskDelay(pdMS_TO_TICKS(200));

    // turn LEFT to become parallel (left-bypass)
    safePrint("[LEARN LEFT] turning LEFT (TURN_DEG=%d) to become parallel\n", TURN_DEG);
    turnLeftDeg(TURN_DEG);
    vTaskDelay(pdMS_TO_TICKS(80 + TURN_POST_MS / 2));

    // main bypass loop
    int attempts = 0;
    int acc_forward_ms = 0;
    while (learnLeftActive && sides_passed < 4 && attempts < MAX_TOTAL_ATTEMPTS) {
      attempts++;
      safePrint("[LEARN LEFT] bypass attempt %d\n", attempts);

      // small forward step
      setTargetMotor(MOVE_SPEED, 0, MOVE_SPEED, 0);
      vTaskDelay(pdMS_TO_TICKS(FORWARD_STEP_MS));
      setTargetMotor(0,0,0,0);
      acc_forward_ms += FORWARD_STEP_MS;
      vTaskDelay(pdMS_TO_TICKS(100));

      // test by turning RIGHT (check forward)
      safePrint("[LEARN LEFT] turning RIGHT (TURN_DEG=%d) to check forward distance\n", TURN_DEG);
      turnRightDeg(TURN_DEG);
      vTaskDelay(pdMS_TO_TICKS(80 + TURN_POST_MS / 2));

      Get_Distance();
      vTaskDelay(pdMS_TO_TICKS(DIST_CHECK_DELAY_MS));
      int check = distance;
      safePrint("[LEARN LEFT] check after right turn: %d cm (initial=%d)\n", check, initial_front);

      if ((float)check <= SAME_DISTANCE_MULT * (float)initial_front) {
        safePrintln("[LEARN LEFT] still close -> return to parallel and continue");
        turnLeftDeg(TURN_DEG); // return parallel
        vTaskDelay(pdMS_TO_TICKS(80 + TURN_POST_MS / 2));
        continue;
      }

      if (check >= SAFE_CLEAR_CM || (float)check >= 2.0f * (float)initial_front) {
        safePrint("[LEARN LEFT] side appears cleared (check=%d) -> passing corner\n", check);

        int total_forward_ms = acc_forward_ms + FORWARD_AFTER_PASS_MS;
        addLearnedForwardLeft(total_forward_ms);
        // record the turn performed to change direction: RIGHT
        addLearnedTurnLeft(DIR_RIGHT, TURN_DEG);

        acc_forward_ms = 0;
        sides_passed++;

        // drive forward after corner
        setTargetMotor(MOVE_SPEED, 0, MOVE_SPEED, 0);
        vTaskDelay(pdMS_TO_TICKS(FORWARD_AFTER_PASS_MS));
        setTargetMotor(0,0,0,0);
        vTaskDelay(pdMS_TO_TICKS(100));
        safePrint("[LEARN LEFT] side cleared. total sides_passed=%d\n", sides_passed);
        continue;
      }

      // ambiguous -> return to parallel and retry
      safePrint("[LEARN LEFT] ambiguous result (check=%d) -> return to parallel and try again\n", check);
      turnLeftDeg(TURN_DEG);
      vTaskDelay(pdMS_TO_TICKS(80 + TURN_POST_MS / 2));
    }

    if (sides_passed < 4) {
      safePrint("[LEARN LEFT] failed to clear all sides after %d attempts -> aborting\n", attempts);
      ledcWriteTone(2, 400);
      vTaskDelay(pdMS_TO_TICKS(600));
      ledcWrite(2, 0);
      taskENTER_CRITICAL(&bypassMux);
      learnLeftActive = false;
      taskEXIT_CRITICAL(&bypassMux);
    } else {
      safePrintln("[LEARN LEFT] learning complete - sides_passed == 4");
      markLearnedAvailableLeft();
    }
  } // end approach

  setTargetMotor(0,0,0,0);
  safePrint("LEARN LEFT: finished simple box bypass; sides_passed=%d\n", sides_passed);
  if (sides_passed >= 4) {
    ledcWriteTone(2, 900);
    vTaskDelay(pdMS_TO_TICKS(200));
    ledcWrite(2, 0);
  } else {
    ledcWriteTone(2, 400);
    vTaskDelay(pdMS_TO_TICKS(200));
    ledcWrite(2, 0);
  }

  taskENTER_CRITICAL(&bypassMux);
  learnLeftActive = false;
  learnLeftTaskHandle = NULL;
  taskEXIT_CRITICAL(&bypassMux);

  vTaskDelete(NULL);
}

// ------------------ REPLAY RIGHT (replay learned right trajectory, no sensors) ------------------
void replay_learned_right_task(void *pvParameters) {
  (void) pvParameters;

  taskENTER_CRITICAL(&bypassMux);
  if (replayRightActive) {
    safePrintln("REPLAY RIGHT: already active -> exiting");
    replayRightTaskHandle = NULL;
    taskEXIT_CRITICAL(&bypassMux);
    vTaskDelete(NULL);
    return;
  }
  if (!learnedAvailableRight || learnedCountRight == 0) {
    safePrintln("REPLAY RIGHT: no learned right trajectory available -> exiting");
    replayRightTaskHandle = NULL;
    taskEXIT_CRITICAL(&bypassMux);
    vTaskDelete(NULL);
    return;
  }

  replayRightActive = true;
  replayRightTaskHandle = xTaskGetCurrentTaskHandle();
  taskEXIT_CRITICAL(&bypassMux);

  safePrintln("REPLAY RIGHT: replaying learned right trajectory (no sensor checks)");

  LearnedAction* localActions = nullptr;
  int localCount = 0;

  taskENTER_CRITICAL(&bypassMux);
  localCount = learnedCountRight;
  if (localCount > 0) {
    localActions = (LearnedAction*) pvPortMalloc(sizeof(LearnedAction) * localCount);
    if (localActions != nullptr) {
      for (int i = 0; i < localCount; ++i) localActions[i] = learnedActionsRight[i];
    }
  }
  taskEXIT_CRITICAL(&bypassMux);

  if (localActions == nullptr || localCount == 0) {
    safePrintln("REPLAY RIGHT: failed to allocate/copy learned trajectory -> aborting");
    taskENTER_CRITICAL(&bypassMux);
    replayRightActive = false;
    replayRightTaskHandle = NULL;
    taskEXIT_CRITICAL(&bypassMux);
    if (localActions) vPortFree(localActions);
    vTaskDelete(NULL);
    return;
  }

  const int MOVE_SPEED = 60;

  for (int i = 0; i < localCount && replayRightActive; ++i) {
    LearnedAction a = localActions[i];
    if (a.type == ACT_FORWARD) {
      safePrint("[REPLAY RIGHT] FORWARD %d ms\n", a.duration_ms);
      setTargetMotor(MOVE_SPEED, 0, MOVE_SPEED, 0);
      vTaskDelay(pdMS_TO_TICKS(a.duration_ms));
      setTargetMotor(0,0,0,0);
      vTaskDelay(pdMS_TO_TICKS(100));
    } else if (a.type == ACT_TURN) {
      if (a.dir == DIR_LEFT) {
        safePrint("[REPLAY RIGHT] TURN LEFT %d deg\n", a.deg);
        turnLeftDeg(a.deg);
      } else {
        safePrint("[REPLAY RIGHT] TURN RIGHT %d deg\n", a.deg);
        turnRightDeg(a.deg);
      }
      vTaskDelay(pdMS_TO_TICKS(80 + TURN_POST_MS / 2));
    }
  }

  setTargetMotor(0,0,0,0);
  ledcWriteTone(2, 900);
  vTaskDelay(pdMS_TO_TICKS(200));
  ledcWrite(2, 0);

  vPortFree(localActions);

  taskENTER_CRITICAL(&bypassMux);
  replayRightActive = false;
  replayRightTaskHandle = NULL;
  taskEXIT_CRITICAL(&bypassMux);

  safePrintln("REPLAY RIGHT: finished replay");
  vTaskDelete(NULL);
}

// ------------------ REPLAY LEFT (replay learned left trajectory, no sensors) ------------------
void replay_learned_left_task(void *pvParameters) {
  (void) pvParameters;

  taskENTER_CRITICAL(&bypassMux);
  if (replayLeftActive) {
    safePrintln("REPLAY LEFT: already active -> exiting");
    replayLeftTaskHandle = NULL;
    taskEXIT_CRITICAL(&bypassMux);
    vTaskDelete(NULL);
    return;
  }
  if (!learnedAvailableLeft || learnedCountLeft == 0) {
    safePrintln("REPLAY LEFT: no learned left trajectory available -> exiting");
    replayLeftTaskHandle = NULL;
    taskEXIT_CRITICAL(&bypassMux);
    vTaskDelete(NULL);
    return;
  }

  replayLeftActive = true;
  replayLeftTaskHandle = xTaskGetCurrentTaskHandle();
  taskEXIT_CRITICAL(&bypassMux);

  safePrintln("REPLAY LEFT: replaying learned left trajectory (no sensor checks)");

  LearnedAction* localActions = nullptr;
  int localCount = 0;

  taskENTER_CRITICAL(&bypassMux);
  localCount = learnedCountLeft;
  if (localCount > 0) {
    localActions = (LearnedAction*) pvPortMalloc(sizeof(LearnedAction) * localCount);
    if (localActions != nullptr) {
      for (int i = 0; i < localCount; ++i) localActions[i] = learnedActionsLeft[i];
    }
  }
  taskEXIT_CRITICAL(&bypassMux);

  if (localActions == nullptr || localCount == 0) {
    safePrintln("REPLAY LEFT: failed to allocate/copy learned trajectory -> aborting");
    taskENTER_CRITICAL(&bypassMux);
    replayLeftActive = false;
    replayLeftTaskHandle = NULL;
    taskEXIT_CRITICAL(&bypassMux);
    if (localActions) vPortFree(localActions);
    vTaskDelete(NULL);
    return;
  }

  const int MOVE_SPEED = 60;

  for (int i = 0; i < localCount && replayLeftActive; ++i) {
    LearnedAction a = localActions[i];
    if (a.type == ACT_FORWARD) {
      safePrint("[REPLAY LEFT] FORWARD %d ms\n", a.duration_ms);
      setTargetMotor(MOVE_SPEED, 0, MOVE_SPEED, 0);
      vTaskDelay(pdMS_TO_TICKS(a.duration_ms));
      setTargetMotor(0,0,0,0);
      vTaskDelay(pdMS_TO_TICKS(100));
    } else if (a.type == ACT_TURN) {
      if (a.dir == DIR_LEFT) {
        safePrint("[REPLAY LEFT] TURN LEFT %d deg\n", a.deg);
        turnLeftDeg(a.deg);
      } else {
        safePrint("[REPLAY LEFT] TURN RIGHT %d deg\n", a.deg);
        turnRightDeg(a.deg);
      }
      vTaskDelay(pdMS_TO_TICKS(80 + TURN_POST_MS / 2));
    }
  }

  setTargetMotor(0,0,0,0);
  ledcWriteTone(2, 900);
  vTaskDelay(pdMS_TO_TICKS(200));
  ledcWrite(2, 0);

  vPortFree(localActions);

  taskENTER_CRITICAL(&bypassMux);
  replayLeftActive = false;
  replayLeftTaskHandle = NULL;
  taskEXIT_CRITICAL(&bypassMux);

  safePrintln("REPLAY LEFT: finished replay");
  vTaskDelete(NULL);
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
        float globalSpeed = race.max_speed_scale * speed;
        setTargetMotor(int(160.0f * globalSpeed), 0, int(160.0f * globalSpeed), 0);  // Используем рамп
        delay(600);
        setTargetMotor(0, 0, 0, 0);
        fill_solid(leds, 4, CRGB::Black);
        FastLED.show();
      }
      if (value == "RACE FAST") {
        presetSpeedDemon();
        Serial.println("Preset: Speed Demon");
      }
      if (value == "RACE DRIFT") {
        presetDriftKing();
        Serial.println("Preset: Drift King");
      }
      if (value == "RACE SMOOTH") {
        presetSmoothRacer();
        Serial.println("Preset: Smooth Racer");
      }
      if (value == "RACE ECO") {
        presetEcoMode();
        Serial.println("Preset: Eco Mode");
      }
      // ---- voice command handler: new commands (Russian): "обход справа", "обход слева", "повтори обход" ----

if (value == "PASS ON THE RIGHT") {
  taskENTER_CRITICAL(&bypassMux);
  bool already = learnRightActive;
  taskEXIT_CRITICAL(&bypassMux);

  if (already) {
    Serial.println("LEARN RIGHT: already running");
  } else {
    // stop motors explicitly before start
    setTargetMotor(0,0,0,0);
    BaseType_t r = xTaskCreate(
      learn_bypass_right_task, // function
      "learn_right",           // name
      2048,                    // stack
      NULL,                    // param
      2,                       // priority
      &learnRightTaskHandle
    );
    if (r == pdPASS) {
      Serial.println("LEARN RIGHT: task created");
    } else {
      Serial.println("LEARN RIGHT: failed to create task");
    }
  }
}

if (value == "PASS ON THE LEFT") {
  taskENTER_CRITICAL(&bypassMux);
  bool already = learnLeftActive;
  taskEXIT_CRITICAL(&bypassMux);

  if (already) {
    Serial.println("LEARN LEFT: already running");
  } else {
    // stop motors explicitly before start
    setTargetMotor(0,0,0,0);
    BaseType_t r = xTaskCreate(
      learn_bypass_left_task, // function
      "learn_left",           // name
      2048,                   // stack
      NULL,
      2,
      &learnLeftTaskHandle
    );
    if (r == pdPASS) {
      Serial.println("LEARN LEFT: task created");
    } else {
      Serial.println("LEARN LEFT: failed to create task");
    }
  }
}

if (value == "GO TURNING") {
  // choose which learned trajectory to replay:
  // - if only one available -> replay it
  // - if both available -> default to replay RIGHT (changeable)
  taskENTER_CRITICAL(&bypassMux);
  bool rightAvailable = learnedAvailableRight && (learnedCountRight > 0);
  bool leftAvailable  = learnedAvailableLeft  && (learnedCountLeft  > 0);
  bool replayRightBusy = replayRightActive;
  bool replayLeftBusy  = replayLeftActive;
  taskEXIT_CRITICAL(&bypassMux);

  if (!rightAvailable && !leftAvailable) {
    Serial.println("REPLAY: no learned trajectory available");
  } else {
    // If both available, default to RIGHT. If only one available, pick that one.
    if (rightAvailable && !leftAvailable) {
      // replay right
      if (replayRightBusy) {
        Serial.println("REPLAY RIGHT: already running");
      } else {
        setTargetMotor(0,0,0,0);
        BaseType_t r = xTaskCreate(
          replay_learned_right_task,
          "replay_right",
          4096,
          NULL,
          2,
          &replayRightTaskHandle
        );
        if (r == pdPASS) Serial.println("REPLAY RIGHT: task created");
        else Serial.println("REPLAY RIGHT: failed to create task");
      }
    } else if (!rightAvailable && leftAvailable) {
      // replay left
      if (replayLeftBusy) {
        Serial.println("REPLAY LEFT: already running");
      } else {
        setTargetMotor(0,0,0,0);
        BaseType_t r = xTaskCreate(
          replay_learned_left_task,
          "replay_left",
          4096,
          NULL,
          2,
          &replayLeftTaskHandle
        );
        if (r == pdPASS) Serial.println("REPLAY LEFT: task created");
        else Serial.println("REPLAY LEFT: failed to create task");
      }
    } else {
      // both available: choose default (RIGHT). You can change this policy.
      Serial.println("REPLAY: both LEFT and RIGHT trajectories available — replaying RIGHT by default");
      if (replayRightBusy) {
        Serial.println("REPLAY RIGHT: already running");
      } else {
        setTargetMotor(0,0,0,0);
        BaseType_t r = xTaskCreate(
          replay_learned_right_task,
          "replay_right",
          4096,
          NULL,
          2,
          &replayRightTaskHandle
        );
        if (r == pdPASS) Serial.println("REPLAY RIGHT: task created");
        else Serial.println("REPLAY RIGHT: failed to create task");
      }
    }
  }
}
      if (value == "GO BACK")  //backward
      {
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Red;
        fill_solid(RGBleds, 6, myRGBcolor6);
        FastLED.show();
        setTargetMotor(0, float(160) * speed, 0, float(160) * speed);
        delay(600);
        setTargetMotor(0, 0, 0, 0);
        fill_solid(leds, 4, CRGB::Black);
        FastLED.show();
      }

      if (value == "GO RIGHT")  //towards the right
      {
        leds[2] = CRGB::Green;
        fill_solid(RGBleds, 6, myRGBcolor6);
        FastLED.show();
        setTargetMotor(80, 0, 0, 80);
        delay(350);
        setTargetMotor(0, 0, 0, 0);
        fill_solid(leds, 4, CRGB::Black);
        FastLED.show();
      }

      if (value == "GO LEFT")  //towards the left
      {
        leds[3] = CRGB::Green;
        fill_solid(RGBleds, 6, myRGBcolor6);
        FastLED.show();
        setTargetMotor(0, 80, 80, 0);
        delay(350);
        setTargetMotor(0, 0, 0, 0);
        fill_solid(leds, 4, CRGB::Black);
        FastLED.show();
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
      if (value == "FASTER SPEED") {
        speed = std::min(1.5f, speed + d_speed);
      }
      if (value == "SLOWER SPEED") {
        speed = std::max(0.5f, speed - d_speed);
      }
      if (value == "RACE FAST") {
        presetSpeedDemon();
        Serial.println("Preset: Speed Demon");
      }
      if (value == "RACE DRIFT") {
        presetDriftKing();
        Serial.println("Preset: Drift King");
      }
      if (value == "RACE SMOOTH") {
        presetSmoothRacer();
        Serial.println("Preset: Smooth Racer");
      }
      if (value == "RACE ECO") {
        presetEcoMode();
        Serial.println("Preset: Eco Mode");
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

        float globalSpeed = race.max_speed_scale * speed;  // Масштабируем скорость
// После вычисления l и r из poly_transform
l *= globalSpeed;
r *= globalSpeed;

// Добавить влияние turn_scale на разность между моторами
if (abs(l - r) > 0.1f) {  // если есть поворот
    float turn_factor = race.turn_scale;
    float center = (l + r) / 2.0f;
    l = center + (l - center) * turn_factor;
    r = center + (r - center) * turn_factor;
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
setTargetMotor(lf, lb, rf, rb);  // Используем рамп
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
void ramp_task(void *pvParameters) {
  for (;;) {
    updateMotorRamp();
    vTaskDelay(pdMS_TO_TICKS(RAMP_INTERVAL_MS));
  }
}

//Line patrol procedure
void Tracking() {
  QTI_Max = digitalRead(QTI_L) * 2 + digitalRead(QTI_R);  //Black line return 1
  //Serial.println( QTI_Max);
  switch (QTI_Max) {
    case 3:
      setTargetMotor(160, 0, 160, 0);
      break;
    case 2:
      setTargetMotor(70, 0, 140, 0);
      break;
    case 1:
      setTargetMotor(140, 0, 70, 0);
      break;
    case 0:
      setTargetMotor(0, 120, 0, 120);
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
    setTargetMotor(120, 0, 60, 0);
  } else if (((num_R - num_L) > 250) && (num_R < 3700) && (num_L < 3700)) {  //Strong light on the left, left
    setTargetMotor(60, 0, 120, 0);
  } else if ((((num_L - num_R) < 70) || ((num_R - num_L) < 70)) && (num_R < 3650) && (num_L < 3650)) {  //The light is straight ahead. Go straight
    setTargetMotor(120, 0, 120, 0);
  } else if (num_R > 3700 && num_L > 3700) {
    setTargetMotor(0, 0, 0, 0);
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
  xTaskCreate(play_music_task, "play_music_task", 4096, NULL, 1, NULL);
  xTaskCreate(grc_cmd_task, "grc_cmd_task", 8192, NULL, 1, NULL);
  xTaskCreate(ramp_task, "ramp_task", 2048, NULL, 1, NULL);
  xTaskCreate(distance_log_task, "dist_log", 4096, NULL, 1, NULL);
  // В setup() после создания других семафоров добавьте:
xSerialSemaphore = xSemaphoreCreateMutex();

}


//while（1）
void loop() {
  updateMotorRamp();  // Обновляем рамп моторов каждый цикл

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
            setTargetMotor(240, 0, 240, 0);
            break;
          case 22:  //retreat
            setTargetMotor(0, 220, 0, 220);
            break;
          case 33:  //turn left
            setTargetMotor(0, 180, 180, 0);
            break;
          case 44:  //turn right
            setTargetMotor(180, 0, 0, 180);
            break;
          case 55:  //stop
            setTargetMotor(0, 0, 0, 0);
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
                setTargetMotor(220, 0, 0, 220);
              } else {
                setTargetMotor(220, 0, 220, 0);
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
              setTargetMotor(0, 0, 0, 0);  //Close the steering gear
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
              setTargetMotor(0, 0, 0, 0);  //Close the steering gear
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
              setTargetMotor(0, 0, 0, 0);  //Close the steering gear
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
              setTargetMotor(0, 0, 0, 0); 
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
                    setTargetMotor(0, 0, 0, 0);
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
              setTargetMotor(0, 0, 0, 0);  
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
            setTargetMotor(0, 0, 0, 0);
            break;
        }

        irrecv.resume();
      } else {
        setTargetMotor(0, 0, 0, 0);
        ledcWrite(2, 0);
      }
      delay(150);

      
    }
  }
}