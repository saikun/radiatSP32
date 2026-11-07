// 放射線検出ファームウェア (ビルド: pio run -e radiation)
//
// レンズを外して遮光した OV2640 を半導体検出器として使い、β線/γ線による輝点を検出する。
// 仕様は memo.md。PC 側 GUI は tools/radiat_viewer.py。
//
//   Core 0  Task_CaptureAndDetect    : フレーム取得 → ダークフレーム減算 → 閾値判定 → Queue
//   Core 1  Task_DisplayAndTelemetry : Queue → シリアル / TFT、1 秒ごとの統計、コマンド受付
//
// シリアル出力 (1 行 1 メッセージ。tools/radiat_viewer.py の正規表現と一致させること):
//   [HIT] X=123, Y=45, Energy=67, Sum=210, T=123456
//   [STAT] CPS=3, CPM=180, Total=1234, FPS=25.0, Drop=0, Cal=0
//   [CFG] W=400, H=296, TH=25, GAIN=10, EXP=300
//   [CAL] start / [CAL] done ...
// コマンド (改行区切り):
//   t=30   閾値 (1-255)
//   g=15   ゲイン (0-30)        ※変更後は自動で再キャリブレーション
//   e=300  露光時間 (0-1200)    ※変更後は自動で再キャリブレーション
//   c      再キャリブレーション
//   r      カウンタとヒットマップのリセット
//   info   設定の再送

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>

#include <atomic>

#include "camera_common.h"

#ifndef RADIATION_USE_TFT
#define RADIATION_USE_TFT 1
#endif

#if RADIATION_USE_TFT
#include <LovyanGFX.hpp>
#endif

namespace {

// CIF (400x296) はセンサーの CIF モードそのままの解像度で、DSP の縮小処理を通らない。
// VGA は SVGA モードからの縮小になり、1 画素の輝点が周囲と平均されて薄まる。
constexpr framesize_t kFrameSize = FRAMESIZE_CIF;
constexpr int kCalibrationFrames = 30;
constexpr int kSettleFrames = 3;  // 設定変更直後の不安定なフレームを捨てる数
constexpr int kQueueLength = 64;
constexpr uint32_t kLedPulseMs = 20;
constexpr uint32_t kStatPeriodMs = 1000;
constexpr int kCpmWindow = 60;

constexpr int kGainMax = 30;       // OV2640 ドライバの set_agc_gain の範囲
constexpr int kExposureMax = 1200;  // OV2640 ドライバの set_aec_value の範囲

struct RadiationEvent {
  uint16_t x;
  uint16_t y;
  uint8_t energy;  // ピーク画素の (現フレーム - ダークフレーム) = 輝度波高
  uint16_t sum;    // 3x3 の正の差分の合計。電荷が隣へ広がった分も含めたエネルギーの目安
  uint32_t timestamp_ms;
};

QueueHandle_t g_eventQueue = nullptr;
uint16_t g_width = 0;
uint16_t g_height = 0;
uint8_t *g_darkFrame = nullptr;
bool g_cameraReady = false;

// Core 1 (コマンド) → Core 0 (検出) への設定。センサーへの書き込みは Core 0 だけが行う
std::atomic<int> g_threshold{25};
std::atomic<int> g_gain{10};
std::atomic<int> g_exposure{300};
std::atomic<bool> g_sensorDirty{false};
std::atomic<bool> g_calibrationRequested{true};  // 起動直後に 1 回実行する
std::atomic<bool> g_calibrating{false};

// Core 0 → Core 1 の統計
std::atomic<uint32_t> g_frameCount{0};
std::atomic<uint32_t> g_hitCount{0};
std::atomic<uint32_t> g_dropCount{0};  // Queue 満杯で送れなかったイベント数

// ============================================================================
// カメラ
// ============================================================================

// ISP の補正は放射線の輝点をノイズとして消したり、エネルギーの線形性を崩すため全て切る
void applySensorSettings(sensor_t *s) {
  s->set_bpc(s, 0);       // Black Pixel Cancellation 無効
  s->set_wpc(s, 0);       // White Pixel Cancellation (ホットピクセル除去) 無効
  s->set_raw_gma(s, 0);   // ガンマ補正無効
  s->set_lenc(s, 0);      // レンズシェーディング補正無効
  s->set_whitebal(s, 0);  // AWB 無効 (輝度が勝手に変わらないように)
  s->set_awb_gain(s, 0);
  s->set_exposure_ctrl(s, 0);  // 自動露光無効 (memo.md の set_aec はこの関数名)
  s->set_aec2(s, 0);           // DSP 側の自動露光も無効
  s->set_aec_value(s, g_exposure.load());
  s->set_gain_ctrl(s, 0);  // 自動ゲイン無効
  s->set_agc_gain(s, g_gain.load());
}

bool startCamera() {
  const camera_config_t config = makeCameraConfig(PIXFORMAT_GRAYSCALE, kFrameSize);
  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[ERR] esp_camera_init failed: 0x%x %s\n", err, esp_err_to_name(err));
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID != OV2640_PID) {
    Serial.printf("[WARN] unexpected sensor PID=0x%02X\n", s->id.PID);
  }
  applySensorSettings(s);

  g_width = resolution[kFrameSize].width;
  g_height = resolution[kFrameSize].height;
  g_darkFrame = static_cast<uint8_t *>(
      heap_caps_malloc(static_cast<size_t>(g_width) * g_height, MALLOC_CAP_SPIRAM));
  if (g_darkFrame == nullptr) {
    Serial.println("[ERR] dark frame allocation failed (PSRAM)");
    return false;
  }
  Serial.println("[BOOT] Camera init succeeded");
  return true;
}

// 期待どおりのグレースケール画像でないフレームは使わない
bool isValidFrame(const camera_fb_t *fb) {
  return fb->width == g_width && fb->height == g_height &&
         fb->len == static_cast<size_t>(g_width) * g_height;
}

camera_fb_t *grabFrame() {
  static uint32_t lastErrorMs = 0;
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    if (millis() - lastErrorMs > 1000) {
      lastErrorMs = millis();
      Serial.println("[ERR] esp_camera_fb_get timeout");
    }
    return nullptr;
  }
  if (!isValidFrame(fb)) {
    if (millis() - lastErrorMs > 1000) {
      lastErrorMs = millis();
      Serial.printf("[ERR] unexpected frame %ux%u len=%u\n", fb->width, fb->height,
                    static_cast<unsigned>(fb->len));
    }
    esp_camera_fb_return(fb);
    return nullptr;
  }
  return fb;
}

void discardFrames(int count) {
  for (int i = 0; i < count; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb != nullptr) esp_camera_fb_return(fb);
  }
}

// ============================================================================
// Core 0: キャリブレーションと検出
// ============================================================================

// 各画素の最大値 (暗電流 + ホットピクセル) を dark frame とする
void runCalibration() {
  g_calibrating = true;
  Serial.printf("[CAL] start frames=%d GAIN=%d EXP=%d\n", kCalibrationFrames, g_gain.load(),
                g_exposure.load());
  const size_t pixels = static_cast<size_t>(g_width) * g_height;
  discardFrames(kSettleFrames);
  memset(g_darkFrame, 0, pixels);

  int frames = 0;
  while (frames < kCalibrationFrames) {
    camera_fb_t *fb = grabFrame();
    if (fb == nullptr) continue;
    const uint8_t *src = fb->buf;
    for (size_t i = 0; i < pixels; i++) {
      if (src[i] > g_darkFrame[i]) g_darkFrame[i] = src[i];
    }
    esp_camera_fb_return(fb);
    frames++;
    g_frameCount++;
    vTaskDelay(1);
  }

  uint64_t total = 0;
  uint8_t maxValue = 0;
  for (size_t i = 0; i < pixels; i++) {
    total += g_darkFrame[i];
    if (g_darkFrame[i] > maxValue) maxValue = g_darkFrame[i];
  }
  const float mean = static_cast<float>(total) / pixels;
  // 平均より閾値以上明るい画素 = ダークフレームが無ければ誤検出していたはずの画素
  const int hotLevel = static_cast<int>(mean) + g_threshold.load();
  uint32_t hot = 0;
  for (size_t i = 0; i < pixels; i++) {
    if (g_darkFrame[i] > hotLevel) hot++;
  }
  Serial.printf("[CAL] done mean=%.2f max=%u hot=%lu\n", mean, maxValue,
                static_cast<unsigned long>(hot));
  g_calibrating = false;
}

// (x, y) が 3x3 近傍で差分のピークなら true を返し、近傍の正の差分の合計を sum に入れる。
// 同じ値が並んだ場合に 1 つだけ選ばれるよう、走査順で前の画素とは同値を許し、後の画素とは許さない
bool examineCluster(const uint8_t *frame, int x, int y, int diff, uint16_t *sum) {
  int total = diff;
  for (int dy = -1; dy <= 1; dy++) {
    const int ny = y + dy;
    if (ny < 0 || ny >= g_height) continue;
    for (int dx = -1; dx <= 1; dx++) {
      const int nx = x + dx;
      if ((dx == 0 && dy == 0) || nx < 0 || nx >= g_width) continue;
      const int index = ny * g_width + nx;
      const int neighbor = static_cast<int>(frame[index]) - g_darkFrame[index];
      const bool before = dy < 0 || (dy == 0 && dx < 0);
      if (before ? neighbor > diff : neighbor >= diff) return false;
      if (neighbor > 0) total += neighbor;
    }
  }
  *sum = static_cast<uint16_t>(total);
  return true;
}

// 1 フレーム分のイベントを Queue に送り、検出数を返す
uint32_t detectHits(const uint8_t *frame, uint32_t timestampMs) {
  const int threshold = g_threshold.load();
  uint32_t hits = 0;
  for (int y = 0; y < g_height; y++) {
    const uint8_t *row = frame + y * g_width;
    const uint8_t *darkRow = g_darkFrame + y * g_width;
    for (int x = 0; x < g_width; x++) {
      const int diff = static_cast<int>(row[x]) - darkRow[x];
      if (diff <= threshold) continue;
      RadiationEvent event;
      if (!examineCluster(frame, x, y, diff, &event.sum)) continue;
      event.x = x;
      event.y = y;
      event.energy = static_cast<uint8_t>(diff);
      event.timestamp_ms = timestampMs;
      hits++;
      if (xQueueSend(g_eventQueue, &event, 0) != pdTRUE) g_dropCount++;
    }
  }
  return hits;
}

void Task_CaptureAndDetect(void *) {
  uint32_t ledOffMs = 0;
  bool ledOn = false;

  for (;;) {
    if (g_sensorDirty.exchange(false)) {
      applySensorSettings(esp_camera_sensor_get());
      g_calibrationRequested = true;  // 暗電流はゲイン・露光で変わる
    }
    if (g_calibrationRequested.exchange(false)) {
      runCalibration();
    }

    camera_fb_t *fb = grabFrame();
    if (fb != nullptr) {
      const uint32_t now = millis();
      const uint32_t hits = detectHits(fb->buf, now);
      esp_camera_fb_return(fb);
      g_frameCount++;
      if (hits > 0) {
        g_hitCount += hits;
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        ledOn = true;
        ledOffMs = now + kLedPulseMs;
      }
    }

    if (ledOn && static_cast<int32_t>(millis() - ledOffMs) >= 0) {
      digitalWrite(ONBOARD_LED_PIN, LOW);
      ledOn = false;
    }
    // 処理がフレーム周期を超えても IDLE タスク (タスク WDT) が動けるようにする
    vTaskDelay(1);
  }
}

// ============================================================================
// Core 1: TFT 表示
// ============================================================================

constexpr int kScreenWidth = 240;
constexpr int kMapHeight = 200;
constexpr int kStatusY = kMapHeight;

// エネルギー 0-255 を 青→緑→黄→赤→白 に対応させる (tools/radiat_viewer.py と同じ)
void energyColor(uint8_t energy, uint8_t *r, uint8_t *g, uint8_t *b) {
  static const uint8_t kStops[5][3] = {
      {0, 0, 255}, {0, 255, 0}, {255, 255, 0}, {255, 0, 0}, {255, 255, 255}};
  const int scaled = energy * 4;  // 0..1020
  const int segment = scaled >= 1020 ? 3 : scaled / 255;
  const int t = scaled - segment * 255;  // 0..255
  const uint8_t *from = kStops[segment];
  const uint8_t *to = kStops[segment + 1];
  *r = from[0] + (to[0] - from[0]) * t / 255;
  *g = from[1] + (to[1] - from[1]) * t / 255;
  *b = from[2] + (to[2] - from[2]) * t / 255;
}

#if RADIATION_USE_TFT

class Display : public lgfx::LGFX_Device {
 public:
  Display() {
    auto bus = bus_.config();
    bus.spi_host = SPI2_HOST;
    bus.spi_mode = 0;
    bus.freq_write = 40000000;
    bus.freq_read = 16000000;
    bus.spi_3wire = true;
    bus.use_lock = true;
    bus.dma_channel = SPI_DMA_CH_AUTO;
    bus.pin_sclk = TFT_SCLK;
    bus.pin_mosi = TFT_MOSI;
    bus.pin_miso = TFT_MISO;
    bus.pin_dc = TFT_DC;
    bus_.config(bus);
    panel_.setBus(&bus_);

    auto panel = panel_.config();
    panel.pin_cs = TFT_CS;
    panel.pin_rst = TFT_RST;
    panel.pin_busy = -1;
    panel.panel_width = 240;
    panel.panel_height = 240;
    panel.memory_width = 240;
    panel.memory_height = 320;
    panel.offset_x = 0;
    panel.offset_y = 0;
    panel.offset_rotation = 0;
    panel.readable = false;  // 書き込み専用 (未接続でもブロックしない)
    panel.invert = true;     // 240x240 の ST7789 モジュールは反転が必要なものが多い
    panel.rgb_order = false;
    panel.dlen_16bit = false;
    panel.bus_shared = false;
    panel_.config(panel);
    setPanel(&panel_);
  }

 private:
  lgfx::Panel_ST7789 panel_;
  lgfx::Bus_SPI bus_;
};

Display g_tft;

void displayBegin() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  g_tft.init();
  g_tft.setRotation(0);
  g_tft.fillScreen(TFT_BLACK);
  g_tft.drawFastHLine(0, kStatusY, kScreenWidth, TFT_DARKGREY);
}

void displayClearMap() { g_tft.fillRect(0, 0, kScreenWidth, kMapHeight, TFT_BLACK); }

void displayPlot(const RadiationEvent &event) {
  const int px = event.x * kScreenWidth / g_width;
  const int py = event.y * kMapHeight / g_height;
  uint8_t r, g, b;
  energyColor(event.energy, &r, &g, &b);
  g_tft.fillCircle(px, py, 2, g_tft.color888(r, g, b));
  g_tft.drawFastHLine(0, kStatusY, kScreenWidth, TFT_DARKGREY);  // 円がはみ出した分を戻す
}

void displayStatus(uint32_t cps, uint32_t cpm, uint32_t total, float fps, bool calibrating) {
  g_tft.fillRect(0, kStatusY + 1, kScreenWidth, 240 - kStatusY - 1, TFT_BLACK);
  g_tft.setTextSize(2);
  g_tft.setTextColor(TFT_WHITE, TFT_BLACK);
  g_tft.setCursor(2, kStatusY + 4);
  g_tft.printf("CPS%4lu CPM%6lu", static_cast<unsigned long>(cps),
               static_cast<unsigned long>(cpm));
  g_tft.setCursor(2, kStatusY + 22);
  if (calibrating) {
    g_tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    g_tft.print("CALIBRATING...");
  } else {
    g_tft.printf("T%8lu F%4.1f", static_cast<unsigned long>(total), fps);
  }
}

#else

void displayBegin() {}
void displayClearMap() {}
void displayPlot(const RadiationEvent &) {}
void displayStatus(uint32_t, uint32_t, uint32_t, float, bool) {}

#endif

// ============================================================================
// Core 1: 統計とコマンド
// ============================================================================

class Statistics {
 public:
  void reset() {
    lastMs_ = millis();
    lastHits_ = g_hitCount.load();
    lastFrames_ = g_frameCount.load();
    totalOffset_ = lastHits_;
    count_ = 0;
    index_ = 0;
  }

  // 1 秒ごとに呼ぶ
  void update() {
    const uint32_t now = millis();
    const uint32_t hits = g_hitCount.load();
    const uint32_t frames = g_frameCount.load();
    const uint32_t elapsed = now - lastMs_;
    const bool calibrating = g_calibrating.load();

    const uint32_t cps = hits - lastHits_;
    fps_ = elapsed > 0 ? (frames - lastFrames_) * 1000.0f / elapsed : 0.0f;
    lastHits_ = hits;
    lastFrames_ = frames;
    lastMs_ = now;

    // キャリブレーション中は検出していないので CPM の平均に入れない
    if (!calibrating) {
      cpsHistory_[index_] = cps;
      index_ = (index_ + 1) % kCpmWindow;
      if (count_ < kCpmWindow) count_++;
    }
    uint32_t sum = 0;
    for (int i = 0; i < count_; i++) sum += cpsHistory_[i];
    // 60 秒たまるまでは平均からの推計値
    const uint32_t cpm = count_ == 0 ? 0 : sum * kCpmWindow / count_;
    const uint32_t total = hits - totalOffset_;

    Serial.printf("[STAT] CPS=%lu, CPM=%lu, Total=%lu, FPS=%.1f, Drop=%lu, Cal=%d\n",
                  static_cast<unsigned long>(cps), static_cast<unsigned long>(cpm),
                  static_cast<unsigned long>(total), fps_,
                  static_cast<unsigned long>(g_dropCount.load()), calibrating ? 1 : 0);
    displayStatus(cps, cpm, total, fps_, calibrating);
  }

 private:
  uint32_t lastMs_ = 0;
  uint32_t lastHits_ = 0;
  uint32_t lastFrames_ = 0;
  uint32_t totalOffset_ = 0;
  float fps_ = 0.0f;
  uint32_t cpsHistory_[kCpmWindow] = {};
  int count_ = 0;
  int index_ = 0;
};

Statistics g_stats;

void printConfig() {
  Serial.printf("[CFG] W=%u, H=%u, TH=%d, GAIN=%d, EXP=%d\n", g_width, g_height,
                g_threshold.load(), g_gain.load(), g_exposure.load());
}

void printInfo() {
  Serial.printf("[INFO] chip=%s rev=%d cpu=%luMHz\n", ESP.getChipModel(), ESP.getChipRevision(),
                static_cast<unsigned long>(ESP.getCpuFreqMHz()));
  Serial.printf("[INFO] flash=%luMB psram=%luMB (free %luKB)\n",
                static_cast<unsigned long>(ESP.getFlashChipSize() / (1024 * 1024)),
                static_cast<unsigned long>(ESP.getPsramSize() / (1024 * 1024)),
                static_cast<unsigned long>(ESP.getFreePsram() / 1024));
  if (!psramFound()) Serial.println("[ERR] PSRAM not found");
  if (!g_cameraReady) Serial.println("[ERR] camera not initialized");
  printConfig();
}

// "123" のような 0 以上の整数だけを受け付ける
bool parseValue(const String &text, int minValue, int maxValue, int *value) {
  if (text.length() == 0 || text.length() > 5) return false;
  for (size_t i = 0; i < text.length(); i++) {
    if (!isDigit(text[i])) return false;
  }
  const int parsed = text.toInt();
  if (parsed < minValue || parsed > maxValue) return false;
  *value = parsed;
  return true;
}

void handleCommand(const String &command) {
  if (command.length() == 0) return;
  if (command == "info") {
    printInfo();
    return;
  }
  if (!g_cameraReady) {
    Serial.println("[ERR] camera not initialized");
    return;
  }
  if (command == "c") {
    g_calibrationRequested = true;
    Serial.println("[OK] calibration requested");
    return;
  }
  if (command == "r") {
    g_stats.reset();
    displayClearMap();
    Serial.println("[OK] reset");
    return;
  }

  int value = 0;
  const String argument = command.substring(2);
  if (command.startsWith("t=") && parseValue(argument, 1, 255, &value)) {
    g_threshold = value;
  } else if (command.startsWith("g=") && parseValue(argument, 0, kGainMax, &value)) {
    g_gain = value;
    g_sensorDirty = true;
  } else if (command.startsWith("e=") && parseValue(argument, 0, kExposureMax, &value)) {
    g_exposure = value;
    g_sensorDirty = true;
  } else {
    Serial.printf("[ERR] bad command: %s (t=1-255, g=0-%d, e=0-%d, c, r, info)\n",
                  command.c_str(), kGainMax, kExposureMax);
    return;
  }
  printConfig();
}

void pollCommands() {
  static String line;
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      line.trim();
      handleCommand(line);
      line = "";
    } else if (line.length() < 32) {
      line += c;
    }
  }
}

void Task_DisplayAndTelemetry(void *) {
  displayBegin();
  g_stats.reset();
  uint32_t nextStatMs = millis() + kStatPeriodMs;

  for (;;) {
    RadiationEvent event;
    if (xQueueReceive(g_eventQueue, &event, pdMS_TO_TICKS(10)) == pdTRUE) {
      Serial.printf("[HIT] X=%u, Y=%u, Energy=%u, Sum=%u, T=%lu\n", event.x, event.y,
                    event.energy, event.sum, static_cast<unsigned long>(event.timestamp_ms));
      displayPlot(event);
    }
    pollCommands();
    if (static_cast<int32_t>(millis() - nextStatMs) >= 0) {
      nextStatMs += kStatPeriodMs;
      g_stats.update();
    }
  }
}

}  // namespace

void setup() {
  Serial.setTxBufferSize(8192);  // [HIT] の連続出力で printf が待たされにくくする
  Serial.begin(STREAM_BAUD);
  delay(200);
  Serial.println("[BOOT] radiation detector firmware");

  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW);

  g_eventQueue = xQueueCreate(kQueueLength, sizeof(RadiationEvent));
  g_cameraReady = startCamera();
  printInfo();

  if (g_cameraReady) {
    xTaskCreatePinnedToCore(Task_CaptureAndDetect, "CaptureDetect", 4096, nullptr, 5, nullptr,
                            0);
  }
  xTaskCreatePinnedToCore(Task_DisplayAndTelemetry, "DisplayTelemetry", 8192, nullptr, 3,
                          nullptr, 1);
}

void loop() {
  // 処理は全て上の 2 タスクで行う
  vTaskDelete(nullptr);
}
