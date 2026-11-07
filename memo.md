実機のハードウェア仕様（`ESP32-S3-N16R8`）および確定したピン配置をすべて統合し、コーディングエージェントへそのまま入力可能な「完全版の実装仕様書（引き継ぎ用Markdown）」を提示します。

前のバージョンでは未確定だったメモリ構成（16MB Quad Flash / 8MB Octal PSRAM）やGPIOの物理的制約（GPIO 35〜37の使用禁止ルール等）をすべて明文化して組み込んでいます。エージェントが推測で誤ったピンやメモリ設定を出力する余地を排除した設計です。

以下のコードブロック全体をコピーし、コーディングエージェントへ入力してください。

```markdown
# ESP32-S3 (N16R8) + OV2640 リアルタイム放射線（β線/γ線）2次元トラッカー 実装仕様書（確定版）

## 1. プロジェクト概要
レンズを除去し完全遮光処置を施したCMOSカメラモジュール（OV2640）を半導体放射線検出器として使用する。シリコン空乏層を貫通した放射線（主にβ線およびγ線二次電子）による単一〜数ピクセルの局所的な電荷生成（輝点）をリアルタイムに検出し、抽出したイベント座標 $(X, Y)$ およびエネルギー（輝度波高）をSPI接続のTFT液晶ディスプレイ（ST7789）にプロットおよび統計表示するファームウェアを構築せよ。

## 2. ターゲットハードウェア・開発環境（確定仕様）
- **ビルド環境**: PlatformIO (Framework: Arduino / ESP-IDF ベース)
- **MCUボード**: ESP32-S3-WROOM-1-N16R8 カメラ統合開発ボード
  - **Flash**: 16MB (Quad SPI / `qio`)
  - **PSRAM**: 8MB (Octal SPI / `opi`) ※GPIO 35, 36, 37は内部バス専有のため外部使用厳禁
- **センサー**: OmniVision OV2640 (24-pin FPC DVP接続)
- **ディスプレイ**: ST7789 (解像度 240x240, SPI接続)
- **依存ライブラリ**: 
  - `esp32-camera` (標準ドライバ)
  - `lovyan03/LovyanGFX` (高速SPI/DMA描画用)

---

## 3. 設定ファイル (`platformio.ini`) の厳密定義
N16R8のOctal PSRAMを確実に認識させ、ピクセル走査ループを最大速度で実行するため、以下の設定をそのまま出力すること。左側のUARTブリッジ側USBポートを使用するため、`USB_CDC_ON_BOOT` は無効（0）とする。

```ini
[env:esp32-s3-n16r8]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

; N16R8 (16MB Quad Flash + 8MB Octal PSRAM) 専用メモリ定義
board_build.arduino.memory_type = qio_opi
board_build.f_flash = 80000000L
board_build.flash_mode = qio
board_upload.flash_size = 16MB
board_build.partitions = default_16MB.csv

build_flags = 
    -DBOARD_HAS_PSRAM
    -O3
    -DARDUINO_USB_CDC_ON_BOOT=0

monitor_speed = 115200
monitor_filters = esp32_exception_decoder

lib_deps = 
    lovyan03/LovyanGFX@^1.1.16

```

---

## 4. ハードウェアピン定義 (`include/pin_config.h`)

実機基板の回路パターンに基づく確定ピンアサインである。変更せずそのまま使用すること。

```cpp
#pragma once

// --- 1. OV2640 カメラピン定義 (基板内部配線・左列下線ピン) ---
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
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

// --- 2. ST7789 TFT液晶ピン定義 (右列の安全な空きGPIO) ---
// ※Octal PSRAM専有ピン(35,36,37)およびストラッピングピンを回避済み
#define TFT_MISO        -1
#define TFT_MOSI        40  // SPI SDA / MOSI
#define TFT_SCLK        39  // SPI SCL / SCK
#define TFT_CS          38  // Chip Select
#define TFT_DC          41  // Data / Command
#define TFT_RST         42  // Reset
#define TFT_BL           1  // Backlight (HIGHで点灯)

// --- 3. オンボードインジケータ定義 ---
#define ONBOARD_LED_PIN  2  // 放射線ヒット時のパルス出力用 (IO2)
#define NEOPIXEL_PIN    48  // オンボードRGB LED (オプション)

```

---

## 5. コアソフトウェア要件（厳守事項）

### 5.1. カメラ初期化とISPノイズ除去（ホットピクセル補正）の完全バイパス

放射線による1〜数ピクセルの鋭い輝点は、カメラ内蔵ISPによって「欠陥画素ノイズ（ホットピクセル）」と誤認され消去される。これを防ぐため以下の設定を必須とする。

* `pixel_format`: `PIXFORMAT_GRAYSCALE` (1 pixel = 1 byte, 0〜255)
* `frame_size`: `FRAMESIZE_CIF` (400x296) または `FRAMESIZE_VGA` (640x480)
* `fb_location`: `CAMERA_FB_IN_PSRAM`
* `fb_count`: `2` (DMAダブルバッファリングによる不感時間最小化)
* `grab_mode`: `CAMERA_GRAB_LATEST`
* **ISP制御レジスタの強制無効化**: `esp_camera_sensor_get()` で取得した `sensor_t *s` に対し、初期化直後に以下を実行せよ。
```cpp
s->set_bpc(s, 0);       // Black Pixel Cancellation 無効化
s->set_wpc(s, 0);       // White Pixel Cancellation (ホットピクセル除去) 絶対無効化
s->set_raw_gma(s, 0);   // ガンマ補正無効化（エネルギーの線形性維持）
s->set_lenc(s, 0);      // レンズシェーディング補正無効化
s->set_aec(s, 0);       // 自動露光無効化
s->set_aec_value(s, 300); // 固定露光時間（初期値、シリアルから変更可能にせよ）
s->set_gain_ctrl(s, 0); // 自動ゲイン無効化
s->set_agc_gain(s, 10); // マニュアルゲイン固定（初期値、シリアルから変更可能にせよ）

```



### 5.2. ダークフレーム・キャリブレーション（固定パターンノイズ除去）

1. 起動時、`heap_caps_malloc(frame_size, MALLOC_CAP_SPIRAM)` を用いてPSRAM上に `uint8_t* dark_frame` を確保せよ。
2. 起動直後の30フレームをキャリブレーション期間とし、各ピクセルにおける最大輝度値（暗電流のベースライン＋初期不良ホットピクセルの値）を `dark_frame[i]` に記録せよ。

### 5.3. ゼロサプレッション（イベント抽出と簡易クラスタリング）

カメラ画像をそのまま縮小してTFTへ転送する処理は、微小な放射線輝点を消失させるため**厳禁**とする。

1. 毎フレーム全ピクセルを走査し、`int diff = (int)current_frame[i] - (int)dark_frame[i]` を計算する。
2. `diff > THRESHOLD`（初期値: `25`、シリアルコマンドで変更可能）を満たすピクセルを検出する。
3. 斜め入射等により隣接ピクセル（上下左右 $3 \times 3$ 領域）が同時に閾値を超えた場合、二重カウントを防ぐため、ピーク値を持つ中心ピクセル1つに統合（または周辺をマスク）せよ。
4. 抽出したイベントを構造体 `RadiationEvent { uint16_t x; uint16_t y; uint8_t energy; uint32_t timestamp_ms; }` としてFreeRTOS Queue（サイズ64程度）へ送信する。

### 5.4. FreeRTOS マルチコア・パイプライン設計

* **Core 0 (`Task_CaptureAndDetect`)**:
* カメラからのフレーム取得、ダークフレーム減算、閾値判定、Queue送信のみを専任で高速ループ実行する。
* ヒット検出時、`ONBOARD_LED_PIN (GPIO 2)` を短時間トグル（またはワンショット点灯）させよ。


* **Core 1 (`Task_DisplayAndTelemetry`)**:
* LovyanGFXを用いてST7789（240x240）を制御する。TFTが未接続の状態でも動作がブロックされない設計とすること。
* **画面レイアウト**:
* 上部 `240x200` 領域：2次元ヒットマップ。カメラ座標 $(X_{cam}, Y_{cam})$ を `240x200` にスケーリングし、視認性を高めるため半径2〜3pxの円（または十字）でプロットする。`energy` の高さに応じて色を変化（青→緑→黄→赤→白）させること。
* 下部 `240x40` 領域：ステータスバー。現在の `CPS`（1秒あたりカウント）、`CPM`（直近60秒の移動平均または推計値）、`Total Counts`、およびCore 0の処理 `FPS` を1秒周期でテキスト描画する。


* **シリアルテレメトリ＆コマンド制御**:
* ヒット発生時、即座に `[HIT] X=..., Y=..., Energy=...` をシリアル出力せよ。
* 1秒ごとに `[STAT] CPS=..., CPM=..., Total=..., FPS=...` をシリアル出力せよ。
* シリアル入力から `t=30` (Threshold変更)、`g=15` (Gain変更)、`c` (再キャリブレーション実行) を受け付ける簡易コマンドパーサーを実装せよ。





## 6. 出力成果物の形式

省略（`// ...` 等）を一切含まない、そのままビルドして実機に書き込める以下の3ファイルを完全な形で出力せよ。

1. `platformio.ini`
2. `include/pin_config.h`
3. `src/main.cpp`

```

---

### 実装コード受け取り後の検証プロトコル

コーディングエージェントからコードが出力されたら、以下の順序で実機検証を進めてください。

1. **ビルドと書き込み（左側のUSBポートを使用）:**
   基板の左下にあるUSBポート（`TX`/`RX` のシルクがある側）にケーブルを接続して書き込みます[cite: 1]。書き込み時にエラーが出る場合は、基板上の `BOOT` ボタンを押しながら `EN/RST` ボタンを1回押し、`BOOT` を離すことで強制ダウンロードモードに入ります[cite: 1]。
2. **レンズを外す前の「明室・暗室テスト」:**
   まだレンズを外さず、TFT液晶も繋がない状態で起動し、シリアルモニタ（115200bps）を開きます。
   * カメラの初期化（`Camera init succeeded`）とPSRAMの確保が成功し、`FPS=25〜30` 前後が出ていることを確認します。
   * レンズを指や黒い布で完全に覆った状態で `c` コマンド（再キャリブレーション）を送り、`CPS=0` に落ち着くこと、少し隙間を開けて光を入れると大量の `[HIT]` が出ることを確認します。これでソフトウェアとカメラの疎通が100%証明されます。
3. **レンズ除去・完全遮光と線源（鉱物）テスト:**
   疎通が確認できたら電源を切り、レンズを慎重に取り外してアルミテープ＋黒ビニールテープで遮光窓を作ります。再度起動してキャリブレーションを行い、遮光漏れがないこと（室内の照明を当てても `CPS=0` であること）を確認した上で、30k CPMの鉱物をセンサーに密着させてください。

シリアルモニタ上に、放射線による単一イベントの座標とエネルギーがプロットされ始めたら、最後にTFT液晶を配線して可視化を完成させます。まずはコードの生成と初期ビルドを実行してください。

```