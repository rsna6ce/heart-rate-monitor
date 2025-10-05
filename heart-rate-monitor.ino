#include <Wire.h>
#include "MAX30105.h" // SparkFun MAX3010xライブラリ
#include "heartRate.h" // 心拍数計算用

MAX30105 particleSensor;

// グローバル変数（タスク間で共有）
volatile long irValue = 0; // IR値
volatile int beatAvg = 0;  // 平均心拍数

const byte RATE_SIZE = 8; // 心拍数の平均を取るサンプル数
byte rates[RATE_SIZE];    // 心拍数の配列
byte rateSpot = 0;
long lastBeat = 0;        // 最後の心拍検出時刻
float beatsPerMinute;     // 計算された心拍数

// ブザー設定
#define BUZZER_PIN 15 // ブザーを接続するピン
#define DRAIN_PIN 4   // ブザーのGND
#define BUZZER_FREQ 988.884 // 3オクターブ上のミの音 (329.628 * 3.0 Hz)
#define BUZZER_DURATION 50  // 鳴動時間 50ms
#define LEDC_RESOLUTION 8   // 8ビット分解能
#define LEDC_DUTY 128       // デューティ比50% (8ビットで128)

// オンボードLED設定
#define LED_PIN 2 // ESP32 DevKit V1のオンボードLED (GPIO2)

// ブザー鳴動管理用
unsigned long buzzerStartTime = 0; // ブザー鳴動開始時刻
bool buzzerActive = false;         // ブザーが鳴っているかどうか

const uint32_t triggerNega = 4;
const uint32_t cbufSize = 8;
uint32_t cbufIndex=0;
int32_t cbuf2[cbufSize] = {};
int32_t checkForBeatValuePrev = 0;
uint32_t countPosi = 0;
uint32_t countNega = 0;
bool checkForBeat2(int32_t sample){
    cbuf2[cbufIndex] = sample;
    cbufIndex = (cbufIndex + 1) % cbufSize;
    int32_t total = 0;
    for (int i=0; i<cbufSize; i++) {
        total += cbuf2[i];
    }
    int32_t valueCurr = total / cbufSize;
    if (valueCurr <= checkForBeatValuePrev) {
        countPosi=0;
        countNega++;
    } else if (checkForBeatValuePrev < valueCurr) {
        countPosi++;
        countNega = 0;
    }
    checkForBeatValuePrev = valueCurr;
    return((triggerNega == countNega));
}

// MAX30102ポーリングタスク
void max30102Task(void *pvParameters) {
  while (1) {
    // IR値を取得
    irValue = particleSensor.getIR();

    // 指が離れている場合は心拍数をリセット
    if (irValue < 50000) {
      beatAvg = 0;
      vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms待機
      continue;
    }

    // 心拍検出
    if (checkForBeat2(irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0); // BPMを計算

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute; // 心拍数を保存
        rateSpot %= RATE_SIZE; // 配列の循環

        // 平均心拍数を計算
        int tempAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          tempAvg += rates[x];
        }
        tempAvg /= RATE_SIZE;
        beatAvg = tempAvg; // グローバル変数に保存
      }
      ledcWrite(BUZZER_PIN, LEDC_DUTY); // ブザーPWM出力開始
      digitalWrite(LED_PIN, HIGH);      // LEDをON
      buzzerStartTime = millis();       // 鳴動開始時刻を記録
      buzzerActive = true;
    } else {
      long delta = millis() - lastBeat;
      if (delta > 2000) {
        beatAvg = 0; // 2秒以上心拍がなければリセット
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms待機（100Hzサンプリング相当）
  }
}

void setup() {
  Serial.begin(115200);

  // MAX30102の初期化
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Please check wiring.");
    while (1);
  }

  // MAX30102センサーの設定
  byte ledBrightness = 0xFF; // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;    // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;          // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200;      // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;      // Options: 69, 118, 215, 411
  int adcRange = 16384;      // Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  // ブザーのLEDC設定
  pinMode(DRAIN_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(DRAIN_PIN, LOW);
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, LEDC_RESOLUTION);

  // オンボードLEDの設定
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 初期状態はOFF

  // MAX30102ポーリングタスクを作成（コア1に割り当て）
  xTaskCreatePinnedToCore(
    max30102Task,     // タスク関数
    "MAX30102 Task",  // タスク名
    4096,             // スタックサイズ
    NULL,             // パラメータ
    1,                // 優先度
    NULL,             // タスクハンドル
    1                 // コア1
  );
}

void loop() {
  // ブザー鳴動時間のチェック
  if (buzzerActive && (millis() - buzzerStartTime >= BUZZER_DURATION)) {
    ledcWrite(BUZZER_PIN, 0); // ブザー停止
    digitalWrite(LED_PIN, LOW); // LEDをOFF
    buzzerActive = false;
  }

  // 1秒ごとに心拍数を出力
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    if (beatAvg > 0) {
      Serial.print("bpm ");
      Serial.println(beatAvg);
    } else if (irValue < 50000) {
      Serial.println("No finger detected.");
    } else {
      Serial.println("No valid heart rate detected.");
    }
    lastPrint = millis();
  }
  delay(10);
}