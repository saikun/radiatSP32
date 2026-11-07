#pragma once

// ============================================================================
// 1. OV2640 カメラピン定義 (ESP32-S3-WROOM CAM 標準回路・左列下線ピン)
// ============================================================================
#define CAM_PIN_PWDN    -1  // 基板上でGND固定
#define CAM_PIN_RESET   -1  // 基板上でハードウェアリセット共有
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD     4  // I2C SDA
#define CAM_PIN_SIOC     5  // I2C SCL

#define CAM_PIN_D7      16  // Y9
#define CAM_PIN_D6      17  // Y8
#define CAM_PIN_D5      18  // Y7
#define CAM_PIN_D4      12  // Y6
#define CAM_PIN_D3      10  // Y5
#define CAM_PIN_D2       8  // Y4
#define CAM_PIN_D1       9  // Y3
#define CAM_PIN_D0      11  // Y2

#define CAM_PIN_VSYNC    6
#define CAM_PIN_HREF     7
#define CAM_PIN_PCLK    13

// ============================================================================
// 2. ST7789 TFT液晶ピン定義 (右列の安全な空きGPIOへ割り当て)
// ※ カメラおよびOctal PSRAM (GPIO 35,36,37) と一切競合しない配置
// ============================================================================
#define TFT_MISO        -1  // ST7789は読み出し不要
#define TFT_MOSI        40  // SPI データ出力 (SDA / MOSI)
#define TFT_SCLK        39  // SPI クロック (SCL / SCK)
#define TFT_CS          38  // チップセレクト
#define TFT_DC          41  // データ/コマンド切り替え
#define TFT_RST         42  // リセット
#define TFT_BL           1  // バックライト制御 (HIGHで点灯。3V3直結でも可)

// ============================================================================
// 3. オンボード周辺機器 (デバッグ・インジケータ用)
// ============================================================================
#define ONBOARD_LED_PIN  2  // シルク「IO2」の単色LED (放射線ヒット時のパルス出力)
#define NEOPIXEL_PIN    48  // シルク「48」のRGB LED (放射線ヒット時のフラッシュ演出等に使用可能)