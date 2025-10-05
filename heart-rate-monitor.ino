#include <Wire.h>
#include "MAX30105.h" // SparkFun MAX3010xライブラリ
#include "heartRate.h" // 心拍数計算用

MAX30105 particleSensor;

// グローバル変数（タスク間で共有）
struct SensorData {
  volatile long irValue = 0;   // IR値
  volatile int beatAvg = 0;    // 平均心拍数
} sensorData;

struct HeartbeatConfig {
  static const byte RATE_SIZE = 8;      // 心拍数の平均を取るサンプル数
  static const uint32_t TRIGGER_NEGA = 4; // 単調減少判定閾値
  static const uint32_t CBUF_SIZE = 8;   // 移動平均バッファサイズ
} config;

struct HeartbeatState {
  byte rates[config.RATE_SIZE] = {};     // 心拍数の配列
  byte rateSpot = 0;                     // 現在のインデックス
  long lastBeat = 0;                     // 最後の心拍検出時刻
  float beatsPerMinute = 0;              // 計算された心拍数
  uint32_t cbufIndex = 0;                // 移動平均バッファインデックス
  int32_t cbuf[config.CBUF_SIZE] = {};   // 移動平均バッファ
  int32_t prevValue = 0;                 // 前の平均値
  uint32_t countPosi = 0;                // 単調増加カウンタ
  uint32_t countNega = 0;                // 単調減少カウンタ
} state;

// ハードウェア設定
struct HardwareConfig {
  static const uint8_t BUZZER_PIN = 15;    // ブザーピン
  static const uint8_t DRAIN_PIN = 4;      // ブザーGND
  static constexpr  float BUZZER_FREQ = 988.884; // 3オクターブ上のミ (329.628 * 3.0 Hz)
  static const uint32_t BUZZER_DURATION = 50; // 鳴動時間 (ms)
  static const uint8_t LEDC_RESOLUTION = 8; // PWM分解能
  static const uint8_t LEDC_DUTY = 128;     // デューティ比 50%
  static const uint8_t LED_PIN = 2;         // オンボードLEDピン
} hwConfig;

// ブザー/LED状態
struct BuzzerState {
  unsigned long buzzerStartTime = 0; // ブザー鳴動開始時刻
  bool buzzerActive = false;         // ブザーが鳴っているか
} buzzerState;

// 移動平均を計算する関数
int32_t calculateMovingAverage() {
  int32_t total = 0;
  for (int i = 0; i < config.CBUF_SIZE; i++) {
    total += state.cbuf[i];
  }
  return total / config.CBUF_SIZE;
}

// 心拍検出関数
bool checkForBeat2(int32_t sample) {
  state.cbuf[state.cbufIndex] = sample;
  state.cbufIndex = (state.cbufIndex + 1) % config.CBUF_SIZE;

  int32_t valueCurr = calculateMovingAverage();
  if (valueCurr <= state.prevValue) {
    state.countPosi = 0;
    state.countNega++;
  } else if (state.prevValue < valueCurr) {
    state.countPosi++;
    state.countNega = 0;
  }
  state.prevValue = valueCurr;

  return (config.TRIGGER_NEGA == state.countNega);
}

// MAX30102ポーリングタスク
void max30102Task(void *pvParameters) {
  while (1) {
    // IR値の取得
    sensorData.irValue = particleSensor.getIR();

    // 指が離れている場合
    if (sensorData.irValue < 50000) {
      sensorData.beatAvg = 0;
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    // 心拍検出
    if (checkForBeat2(sensorData.irValue)) {
      long delta = millis() - state.lastBeat;
      state.lastBeat = millis();

      state.beatsPerMinute = 60 / (delta / 1000.0);
      if (state.beatsPerMinute < 255 && state.beatsPerMinute > 20) {
        state.rates[state.rateSpot++] = (byte)state.beatsPerMinute;
        state.rateSpot %= config.RATE_SIZE;

        int tempAvg = 0;
        for (byte x = 0; x < config.RATE_SIZE; x++) {
          tempAvg += state.rates[x];
        }
        sensorData.beatAvg = tempAvg / config.RATE_SIZE;
      }

      // ブザーとLEDを動作
      ledcWrite(hwConfig.BUZZER_PIN, hwConfig.LEDC_DUTY);
      digitalWrite(hwConfig.LED_PIN, HIGH);
      buzzerState.buzzerStartTime = millis();
      buzzerState.buzzerActive = true;
    } else {
      long delta = millis() - state.lastBeat;
      if (delta > 2000) {
        sensorData.beatAvg = 0;
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hzサンプリング
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
  particleSensor.setup(0xFF, 4, 2, 200, 411, 16384);

  // ハードウェア初期化
  pinMode(hwConfig.DRAIN_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(hwConfig.DRAIN_PIN, LOW);
  ledcAttach(hwConfig.BUZZER_PIN, hwConfig.BUZZER_FREQ, hwConfig.LEDC_RESOLUTION);

  pinMode(hwConfig.LED_PIN, OUTPUT);
  digitalWrite(hwConfig.LED_PIN, LOW);

  // MAX30102ポーリングタスクを作成（コア1）
  xTaskCreatePinnedToCore(
    max30102Task, "MAX30102 Task", 4096, NULL, 1, NULL, 1
  );
}

void loop() {
  // ブザー鳴動時間のチェック
  if (buzzerState.buzzerActive && (millis() - buzzerState.buzzerStartTime >= hwConfig.BUZZER_DURATION)) {
    ledcWrite(hwConfig.BUZZER_PIN, 0);
    digitalWrite(hwConfig.LED_PIN, LOW);
    buzzerState.buzzerActive = false;
  }

  // 1秒ごとに心拍数を出力
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    if (sensorData.beatAvg > 0) {
      Serial.print("bpm ");
      Serial.println(sensorData.beatAvg);
    } else if (sensorData.irValue < 50000) {
      Serial.println("No finger detected.");
    } else {
      Serial.println("No valid heart rate detected.");
    }
    lastPrint = millis();
  }
}