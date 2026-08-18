// Natsurobo IR transmitter (NEC) -- 送信側テスト/ビーコン用
//
// ★配線が肝★ ESP32のGPIOでIR-LEDを直接光らせないこと。
//   GPIO --[1kΩ]-- 2N2222(base)
//   2N2222(collector) -- IR-LED(940nm) -- [抵抗] -- 5V
//   2N2222(emitter)   -- GND
//   受信側モジュールの搬送波(通常38kHz)に必ず合わせる。
//
// これでフィールド側の代わりに大漁/状態コードを送って、受信ESP32の
// 動作確認ができる(実機フィールドが無くてもデバッグできる)。

#include <IRremote.hpp>

constexpr uint8_t IR_SEND_PIN = 17;   // トランジスタのベースへ
constexpr uint8_t TX_ADDR     = 0x00;

constexpr uint8_t CMD_TAIRYO = 0x45;
constexpr uint8_t CMD_STATE1 = 0x46;
constexpr uint8_t CMD_STATE2 = 0x47;

uint8_t current = CMD_STATE1;

void setup() {
  Serial.begin(115200);
  IrSender.begin(IR_SEND_PIN);   // 既定38kHz。合わなければ IrSender.setSendPin 後に周波数指定
  Serial.println("tx ready: t=大漁 1=状態A 2=状態B");
}

void loop() {
  // シリアルから送信コードを切り替え(t/1/2)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') current = CMD_TAIRYO;
    else if (c == '1') current = CMD_STATE1;
    else if (c == '2') current = CMD_STATE2;
  }

  // フィールドのLEDテープを模して連続ブロードキャスト
  IrSender.sendNEC(TX_ADDR, current, 0);
  delay(110);   // NECの標準リピート間隔に合わせる
}
