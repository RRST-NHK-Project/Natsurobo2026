// Natsurobo IR receiver (NEC) -> serial + status NeoPixel
// フィールドのIR-LED(38kHz変調)をESP32で受信し、上位へ1行ずつ送る。
// 上位は "IR addr cmd repeat" / "IR LOST" / "HB" の3種だけをパースすればよい。

#include <IRremote.hpp>
#include <Adafruit_NeoPixel.h>

constexpr uint8_t IR_PIN   = 15;   // 受信モジュールのOUT
constexpr uint8_t LED_PIN  = 4;    // 状態表示NeoPixel
constexpr uint8_t EXPECT_ADDR = 0x00;   // フィールド側の固定アドレスに合わせる

constexpr uint32_t SIGNAL_TIMEOUT_MS = 250;  // NECリピート ~110ms の約2回分
constexpr uint32_t HEARTBEAT_MS      = 500;  // 死活監視用

Adafruit_NeoPixel px(1, LED_PIN, NEO_GRB + NEO_KHZ800);
uint8_t  lastCmd  = 0xFF;
uint32_t lastRxMs = 0;
bool     active   = false;

// 状態表示の色は上位(ROS)が決める。ここは受け取った色を出すだけ。
// シリアルで "LED r g b" を受けたら適用する。
void applyLedCmd(const String &line) {
  int r, g, b;
  if (sscanf(line.c_str(), "LED %d %d %d", &r, &g, &b) == 3) {
    px.setPixelColor(0, (uint8_t)r, (uint8_t)g, (uint8_t)b);
    px.show();
  }
}

void setup() {
  Serial.begin(115200);
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  px.begin();
  px.show();
}

void loop() {
  // 出口: ROS からの LED 色指示を適用
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    if (line.startsWith("LED")) applyLedCmd(line);
  }

  if (IrReceiver.decode()) {
    auto &d = IrReceiver.decodedIRData;
    bool isRepeat = d.flags & IRDATA_FLAGS_IS_REPEAT;
    if (d.protocol == NEC && (isRepeat || d.address == EXPECT_ADDR)) {
      if (!isRepeat) lastCmd = d.command;   // リピートは前回コマンドを維持
      lastRxMs = millis();
      if (!active || !isRepeat) {
        active = true;
        Serial.printf("IR %02X %02X %d\n", d.address, lastCmd, isRepeat ? 1 : 0);
      }
    }
    IrReceiver.resume();
  }

  // タイムアウト -> ロストを1回だけ通知(表示はラッチなので消さない)
  if (active && millis() - lastRxMs > SIGNAL_TIMEOUT_MS) {
    active = false;
    Serial.println("IR LOST");
  }

  // ハートビート(supervisorの死活監視用)
  static uint32_t hb = 0;
  if (millis() - hb > HEARTBEAT_MS) {
    hb = millis();
    Serial.println("HB");
  }
}
