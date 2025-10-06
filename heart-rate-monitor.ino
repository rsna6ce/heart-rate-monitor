#include <Wire.h>
#include <U8g2lib.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

// グローバル変数（タスク間で共有）
struct SensorData {
  volatile long irValue = 0;   // IR値
  volatile int beatAvg = 0;    // 平均心拍数
} sensorData;

struct HeartbeatConfig {
  static const byte RATE_SIZE = 4;      // 心拍数の平均を取るサンプル数
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

struct HardwareConfig {
  static const uint8_t BUZZER_PIN = 15;    // ブザーピン
  static const uint8_t DRAIN_PIN = 4;      // ブザーGND
  static constexpr float BUZZER_FREQ = 988.884; // 3オクターブ上のミ (329.628 * 3.0 Hz)
  static const uint32_t BUZZER_DURATION = 100; // 鳴動時間 (ms)
  static const uint8_t LEDC_RESOLUTION = 8; // PWM分解能
  static const uint8_t LEDC_DUTY = 128;     // デューティ比 50%
  static const uint8_t LED_PIN = 2;         // オンボードLEDピン
} hwConfig;

struct BuzzerState {
  unsigned long buzzerStartTime = 0; // ブザー鳴動開始時刻
  bool buzzerActive = false;         // ブザーが鳴っているか
} buzzerState;

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // Wireを使用
#define SCREEN_LINE32(n) ((32 * (n)) + 31)

// 32x32ハートビットマップ (generated image2cpp)
const unsigned char epd_bitmap_heart[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0xc0, 0x3f, 0xfc, 0x03, 0xe0, 0x7f, 0xfe, 0x07,
  0xf0, 0xff, 0xff, 0x0f, 0xf8, 0xff, 0xff, 0x1f, 0xf8, 0xff, 0xff, 0x1f, 0xf8, 0xff, 0xff, 0x3f,
  0xfc, 0xff, 0xff, 0x3f, 0xfc, 0xff, 0xff, 0x3f, 0xfc, 0xff, 0xff, 0x3f, 0xfc, 0xff, 0xff, 0x3f,
  0xfc, 0xff, 0xff, 0x3f, 0xfc, 0xff, 0xff, 0x3f, 0xf8, 0xff, 0xff, 0x1f, 0xf8, 0xff, 0xff, 0x1f,
  0xf8, 0xff, 0xff, 0x1f, 0xf8, 0xff, 0xff, 0x1f, 0xf0, 0xff, 0xff, 0x0f, 0xf0, 0xff, 0xff, 0x0f,
  0xe0, 0xff, 0xff, 0x07, 0xe0, 0xff, 0xff, 0x07, 0xc0, 0xff, 0xff, 0x03, 0x80, 0xff, 0xff, 0x01,
  0x00, 0xff, 0xff, 0x01, 0x00, 0xff, 0xff, 0x00, 0x00, 0xfe, 0x7f, 0x00, 0x00, 0xfc, 0x3f, 0x00,
  0x00, 0xf0, 0x1f, 0x00, 0x00, 0xe0, 0x07, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char epd_bitmap_heart2 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0xc0, 0x3f, 0xfc, 0x03, 0xc0, 0xff, 0xff, 0x03, 
	0xc0, 0xff, 0xff, 0x07, 0xe0, 0xff, 0xff, 0x07, 0xe0, 0xff, 0xff, 0x07, 0xe0, 0xff, 0xff, 0x07, 
	0xe0, 0xff, 0xff, 0x07, 0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0x03, 
	0x80, 0xff, 0xff, 0x03, 0x80, 0xff, 0xff, 0x01, 0x80, 0xff, 0xff, 0x01, 0x00, 0xff, 0xff, 0x00, 
	0x00, 0xff, 0xff, 0x00, 0x00, 0xfe, 0x7f, 0x00, 0x00, 0xfc, 0x7f, 0x00, 0x00, 0xf8, 0x3f, 0x00, 
	0x00, 0xf0, 0x1f, 0x00, 0x00, 0xe0, 0x0f, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

int32_t calculateMovingAverage() {
  int32_t total = 0;
  for (int i = 0; i < config.CBUF_SIZE; i++) {
    total += state.cbuf[i];
  }
  return total / config.CBUF_SIZE;
}

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

void screen_write_line(const char* msg, const unsigned char* bitmap) {
  u8g2.clearBuffer();
  String smsg = String(msg) + "     "; // 5文字に収まるよう調整
  u8g2.setFont(u8g2_font_inb30_mr); // 指定されたフォント
  u8g2.drawStr(0, SCREEN_LINE32(0), smsg.substring(0, 5).c_str()); // 5文字に制限
  u8g2.drawXBM(90, 0, 32, 32, bitmap); // 右端に32x32ビットマップを描画
  u8g2.sendBuffer();
}

// MAX30102ポーリングタスク
void max30102Task(void *pvParameters) {
  static int lastBeatAvg = -1; // 前回のbeatAvgを追跡
  while (1) {
    sensorData.irValue = particleSensor.getIR();

    if (sensorData.irValue < 50000) {
      sensorData.beatAvg = 0;
      if (lastBeatAvg != sensorData.beatAvg) {
        screen_write_line("???", epd_bitmap_heart2); // ハートをクリア
        lastBeatAvg = sensorData.beatAvg;
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

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

        // beatAvgが更新された場合に表示
        if (lastBeatAvg != sensorData.beatAvg) {
          String displayText = String(sensorData.beatAvg);
          lastBeatAvg = sensorData.beatAvg;
        }
      }

      // ブザー/LEDとハートを同期
      ledcWrite(hwConfig.BUZZER_PIN, hwConfig.LEDC_DUTY);
      digitalWrite(hwConfig.LED_PIN, HIGH);
      buzzerState.buzzerStartTime = millis();
      buzzerState.buzzerActive = true;
      screen_write_line(String(lastBeatAvg).c_str(), epd_bitmap_heart); // ハートを表示
    } else {
      long delta = millis() - state.lastBeat;
      if (delta > 2000) {
        sensorData.beatAvg = 0;
        if (lastBeatAvg != sensorData.beatAvg) {
          screen_write_line("???", epd_bitmap_heart2); // ハートをクリア
          lastBeatAvg = sensorData.beatAvg;
        }
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Please check wiring.");
    while (1);
  }

  particleSensor.setup(0xFF, 4, 2, 200, 411, 16384);

  pinMode(hwConfig.DRAIN_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(hwConfig.DRAIN_PIN, LOW);
  ledcAttach(hwConfig.BUZZER_PIN, hwConfig.BUZZER_FREQ, hwConfig.LEDC_RESOLUTION);

  pinMode(hwConfig.LED_PIN, OUTPUT);
  digitalWrite(hwConfig.LED_PIN, LOW);

  Wire.begin(); // MAX30102とSSD1306用
  if (!u8g2.begin()) {
    Serial.println("SSD1306 initialization failed!");
    while (1);
  }
  u8g2.setFlipMode(1);
  u8g2.setContrast(128);
  screen_write_line("Init", epd_bitmap_heart2);

  xTaskCreatePinnedToCore(max30102Task, "MAX30102 Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
  if (buzzerState.buzzerActive && (millis() - buzzerState.buzzerStartTime >= hwConfig.BUZZER_DURATION)) {
    ledcWrite(hwConfig.BUZZER_PIN, 0);
    digitalWrite(hwConfig.LED_PIN, LOW);
    buzzerState.buzzerActive = false;
    screen_write_line(String(sensorData.beatAvg).c_str(), epd_bitmap_heart2); // ハートをクリア
  }
}