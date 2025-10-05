#include <Wire.h>
#include "MAX30105.h" // SparkFun MAX3010xライブラリ
#include "heartRate.h" // 心拍数計算用

MAX30105 particleSensor;

const byte RATE_SIZE = 8; // 心拍数の平均を取るサンプル数（増やして安定化）
byte rates[RATE_SIZE]; // 心拍数の配列
byte rateSpot = 0;
long lastBeat = 0; // 最後の心拍検出時刻
float beatsPerMinute; // 計算された心拍数
int beatAvg; // 平均心拍数

// ブザー設定
#define BUZZER_PIN 15 // ブザーを接続するピン
#define BUZZER_FREQ 988.884 // 3オクターブ上のミの音 (329.628 * 3.0 Hz)
#define BUZZER_DURATION 50 // 鳴動時間 50ms
#define LEDC_RESOLUTION 8 // 8ビット分解能
#define LEDC_DUTY 128 // デューティ比50% (8ビットで128)

// オンボードLED設定
#define LED_PIN 2 // ESP32 DevKit V1のオンボードLED (GPIO2)

// ブザー鳴動管理用
unsigned long buzzerStartTime = 0; // ブザー鳴動開始時刻
bool buzzerActive = false; // ブザーが鳴っているかどうか

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // MAX30102の初期化
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Please check wiring.");
    while (1);
  }

  // センサー設定（高サンプルレートで精度向上）
  //particleSensor.setup(0, 4, 2, 400, 411, 4096); // サンプルレート400Hz、ADC範囲4096
  //particleSensor.setPulseAmplitudeRed(70); // LED強度（低め）
  //particleSensor.setPulseAmplitudeGreen(0); // 緑LEDオフ
  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode       = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate     = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth     = 411; //Options: 69, 118, 215, 411
  int adcRange       = 16384; //Options: 2048, 4096, 8192, 16384
  
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  
  // ブザーのLEDC設定（Core 3系）
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, LEDC_RESOLUTION);

  // オンボードLEDの設定
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 初期状態はOFF
}

void loop() {
  // ブザー鳴動時間のチェック
  if (buzzerActive && (millis() - buzzerStartTime >= BUZZER_DURATION)) {
    ledcWrite(BUZZER_PIN, 0); // ブザー停止
    digitalWrite(LED_PIN, LOW); // LEDをOFF
    buzzerActive = false;
  }

  long irValue = particleSensor.getIR(); // IR値を取得

  // 指が離れている場合を検出（IR値が低すぎる）
  if (irValue < 50000) {
    Serial.println("No finger detected.");
    beatAvg = 0; // 心拍数をリセット
    return;
  }

  if (checkForBeat(irValue) == true) {
    // 心拍を検出
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0); // BPMを計算

    // デバッグ: 心拍間隔とBPMを出力
    Serial.print("Beat detected, delta: ");
    Serial.print(delta);
    Serial.print("ms, BPM: ");
    Serial.println(beatsPerMinute);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; // 心拍数を保存
      rateSpot %= RATE_SIZE; // 配列の循環

      // 平均心拍数を計算
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;

      // ブザーとLEDを鳴らす/点灯（既に鳴っていない場合）
      if (!buzzerActive) {
        ledcWrite(BUZZER_PIN, LEDC_DUTY); // ブザーPWM出力開始
        digitalWrite(LED_PIN, HIGH); // LEDをON
        buzzerStartTime = millis(); // 鳴動開始時刻を記録
        buzzerActive = true;
      }
    }
  }

  // 1秒ごとに心拍数を出力
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    if (beatAvg > 0) {
      Serial.print("Heart Rate: ");
      Serial.print(beatAvg);
      Serial.println(" BPM");
    } else {
      Serial.println("No valid heart rate detected.");
    }
    lastPrint = millis();
  }
}