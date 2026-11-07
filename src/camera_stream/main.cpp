// カメラ動作確認用ファームウェア (ビルド: pio run -e camera_stream)
//
// OV2640 の映像を UART0 (基板の UART ブリッジ側 USB ポート) へ送る。
// 受信側は tools/camera_viewer.py。
//
// 通信はテキストとバイナリの混在で、バイナリ部は下記 FrameHeader で区切る:
//   PC -> ESP : 改行区切りのコマンド (handleCommand を参照)
//   ESP -> PC : 改行区切りのログ + "start" 後はフレーム (FrameHeader + 画像データ)

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_rom_crc.h>

#include "camera_common.h"

namespace {

enum WireFormat : uint8_t { WIRE_GRAY = 0, WIRE_JPEG = 1 };

// tools/camera_viewer.py の HEADER と一致させること (リトルエンディアン)
struct __attribute__((packed)) FrameHeader {
  char magic[4];  // "RFRM"
  uint8_t format;  // WireFormat
  uint8_t reserved;
  uint16_t width;
  uint16_t height;
  uint32_t seq;
  uint32_t payload_len;
  uint32_t payload_crc;  // 画像データの CRC32 (zlib 互換)
  uint32_t header_crc;   // ここより前のヘッダの CRC32
};

struct FrameSizeName {
  const char *name;
  framesize_t size;
};

const FrameSizeName kFrameSizes[] = {
    {"qqvga", FRAMESIZE_QQVGA},  // 160x120
    {"qvga", FRAMESIZE_QVGA},    // 320x240
    {"cif", FRAMESIZE_CIF},      // 400x296
    {"vga", FRAMESIZE_VGA},      // 640x480
    {"svga", FRAMESIZE_SVGA},    // 800x600
    {"uxga", FRAMESIZE_UXGA},    // 1600x1200
};

pixformat_t g_format = PIXFORMAT_JPEG;
framesize_t g_size = FRAMESIZE_QVGA;
bool g_cameraReady = false;
bool g_streaming = false;
uint32_t g_seq = 0;
String g_line;

const char *frameSizeName(framesize_t size) {
  for (const auto &entry : kFrameSizes) {
    if (entry.size == size) return entry.name;
  }
  return "?";
}

void printInfo() {
  Serial.printf("[INFO] chip=%s rev=%d cpu=%luMHz\n", ESP.getChipModel(), ESP.getChipRevision(),
                static_cast<unsigned long>(ESP.getCpuFreqMHz()));
  Serial.printf("[INFO] flash=%luMB psram=%luMB (free %luKB)\n",
                static_cast<unsigned long>(ESP.getFlashChipSize() / (1024 * 1024)),
                static_cast<unsigned long>(ESP.getPsramSize() / (1024 * 1024)),
                static_cast<unsigned long>(ESP.getFreePsram() / 1024));
  if (!psramFound()) {
    Serial.println("[ERR] PSRAM not found (check board_build.arduino.memory_type)");
  }

  sensor_t *sensor = g_cameraReady ? esp_camera_sensor_get() : nullptr;
  if (sensor == nullptr) {
    Serial.println("[ERR] camera not initialized");
    return;
  }
  Serial.printf("[INFO] sensor PID=0x%02X VER=0x%02X (%s) fmt=%s size=%s\n", sensor->id.PID,
                sensor->id.VER, sensor->id.PID == OV2640_PID ? "OV2640" : "unexpected",
                g_format == PIXFORMAT_JPEG ? "jpeg" : "gray", frameSizeName(g_size));
}

// フォーマットとサイズでフレームバッファの大きさが変わるため、変更時は作り直す
bool startCamera() {
  if (g_cameraReady) {
    esp_camera_deinit();
    g_cameraReady = false;
  }

  const camera_config_t config = makeCameraConfig(g_format, g_size);
  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[ERR] esp_camera_init failed: 0x%x %s\n", err, esp_err_to_name(err));
    return false;
  }
  g_cameraReady = true;
  return true;
}

void sendFrame(const camera_fb_t *fb) {
  FrameHeader header = {};
  memcpy(header.magic, "RFRM", sizeof(header.magic));
  header.format = fb->format == PIXFORMAT_JPEG ? WIRE_JPEG : WIRE_GRAY;
  header.width = fb->width;
  header.height = fb->height;
  header.seq = g_seq++;
  header.payload_len = fb->len;
  header.payload_crc = esp_rom_crc32_le(0, fb->buf, fb->len);
  header.header_crc = esp_rom_crc32_le(0, reinterpret_cast<const uint8_t *>(&header),
                                       offsetof(FrameHeader, header_crc));
  Serial.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header));
  Serial.write(fb->buf, fb->len);
}

// UART の送信時間を含まない、カメラ自体のフレームレートを測る
void measureCaptureFps() {
  constexpr uint32_t kDurationMs = 2000;
  // 測定前に溜まっているフレームを捨てる
  for (int i = 0; i < 2; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb != nullptr) esp_camera_fb_return(fb);
  }

  uint32_t frames = 0;
  size_t lastLen = 0;
  const uint32_t startMs = millis();
  while (millis() - startMs < kDurationMs) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr) {
      Serial.println("[ERR] esp_camera_fb_get timeout");
      return;
    }
    lastLen = fb->len;
    esp_camera_fb_return(fb);
    frames++;
  }
  const float fps = frames * 1000.0f / (millis() - startMs);
  Serial.printf("[FPS] capture=%.1f fps (%s %s, last frame %u bytes)\n", fps,
                g_format == PIXFORMAT_JPEG ? "jpeg" : "gray", frameSizeName(g_size),
                static_cast<unsigned>(lastLen));
}

void setCameraMode(pixformat_t format, framesize_t size) {
  if (g_cameraReady && format == g_format && size == g_size) {
    Serial.println("[OK] unchanged");
    return;
  }
  g_format = format;
  g_size = size;
  if (startCamera()) {
    Serial.printf("[OK] fmt=%s size=%s\n", format == PIXFORMAT_JPEG ? "jpeg" : "gray",
                  frameSizeName(size));
  }
}

void handleCommand(const String &command) {
  if (command == "start") {
    g_streaming = g_cameraReady;
    Serial.println(g_streaming ? "[OK] streaming" : "[ERR] camera not initialized");
  } else if (command == "stop") {
    g_streaming = false;
    Serial.println("[OK] stopped");
  } else if (command == "info") {
    printInfo();
  } else if (command == "fps") {
    if (g_cameraReady) {
      measureCaptureFps();
    } else {
      Serial.println("[ERR] camera not initialized");
    }
  } else if (command == "fmt=jpeg") {
    setCameraMode(PIXFORMAT_JPEG, g_size);
  } else if (command == "fmt=gray") {
    setCameraMode(PIXFORMAT_GRAYSCALE, g_size);
  } else if (command.startsWith("size=")) {
    const String name = command.substring(5);
    for (const auto &entry : kFrameSizes) {
      if (name == entry.name) {
        setCameraMode(g_format, entry.size);
        return;
      }
    }
    Serial.printf("[ERR] unknown size: %s\n", name.c_str());
  } else if (command.length() > 0) {
    Serial.printf("[ERR] unknown command: %s\n", command.c_str());
  }
}

void pollCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      g_line.trim();
      handleCommand(g_line);
      g_line = "";
    } else if (g_line.length() < 32) {
      g_line += c;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(STREAM_BAUD);
  delay(200);
  Serial.println("[BOOT] camera test firmware");
  startCamera();
  printInfo();
}

void loop() {
  pollCommands();

  if (!g_streaming) {
    delay(5);
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    Serial.println("[ERR] esp_camera_fb_get timeout");
    return;
  }
  sendFrame(fb);
  esp_camera_fb_return(fb);
}
