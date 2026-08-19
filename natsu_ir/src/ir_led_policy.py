#!/usr/bin/env python3
"""Natsurobo IR LED ポリシーノード -- 「何色を出すか」の判断だけを担う。

ir_node.py はあえて判断を持たない素通しブリッジ。このノードがその上に乗り、
受信IRコードや他トピックから色を決めて serial_bridge 経由で LED 用マイコン
(DEVICE_ID=4) の WS2812B へ送る。

    受信IRコード / 他トピック --(このノードで判断)--> serial_tx_4.data[9] = RGB565
                                                        --(serial_bridge)--> ID=4 MCU --> WS2812B

今の表示仕様:
  通常時 (manual_control_natsu26 が走っている間)  白
  大漁時                                          青
  エラー時 (IRリンク断)                           赤のゆっくり点滅
  上記いずれでもない (manual 未起動など)          消灯

色の送り方 (RGB565, 1スロット):
  serial_tx は int16 の配列。フルカラー(R8G8B8=24bit)は1枠に入らないので、
  16bit の RGB565 (R5 G6 B5) に圧縮して data[9] に入れる。約6.5万色ぶん、ほぼ自由な色。
  RGB565 は最大 0xFFFF で符号付き int16 の範囲(-32768..32767)を超えるため、
  そのままだと負の数として格納されるが、シリアル上のバイト列は同じ。MCU 側で uint16 として
  読み、RGB888 に展開して WS2812B を光らせる。点灯/点滅の判断はすべてこのノードが持つ
  (MCU は受け取った色を出すだけ)。

出力先(重要):
  serial_tx_4 の data[9] は、標準 esp32_serial_bridge ファームでは Rx_16Data[9] = SERVO1 の枠。
  WS2812B を光らせるには ID=4 ファーム側で「Rx_16Data[9] を RGB565 色として読み LED を出す」
  処理が要る。この serial_tx_4 は IR 専用(他ノードは publish しない)前提なので、毎回24スロット送る。

購読:
  /manual/mode std_msgs/String   manual_control_natsu26 の生存信号として使う(内容は見ない)。
                                 50Hz で出続けるので、途切れたら「走っていない」と判定。
  ir/link_ok   std_msgs/Bool     false のときエラー(赤のゆっくり点滅)で最優先に上書き。ラッチ受信。
  ir/tairyo    std_msgs/Bool     true のとき大漁色で上書き。ラッチ受信。
  ir/state     std_msgs/String   受信コード(16進, 例 "19")。今は未使用(下の ir_color 層を参照)。
  (LAYERS に足せば)任意トピック  値の条件で色を上書き。

publish:
  serial_tx_4 std_msgs/Int16MultiArray   24スロット。data[9] に RGB565、他は 0。色変化時のみ送る。
"""

from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Int16MultiArray, String
# 任意トピックを重ねるときは、その型もここで import する(例: Int32, Float32 ...)


# ---------- 送信先 ----------
TX_TOPIC = "serial_tx_4"   # DEVICE_ID=4 の MCU へ
TX_SLOTS = 24              # serial_bridge の int16 スロット数
LED_SLOT = 9              # data[9] に RGB565 を入れる(0始まり)

RGB = Tuple[int, int, int]   # 各 0..255


# ---------- 色の定義 (r,g,b は 0..255) ----------
OFF = (0, 0, 0)
UNKNOWN = (40, 40, 40)     # 検出はしたが色未確定のコード(弱い白)
WHITE = (255, 255, 255)    # 通常時(manual_control が走っている)
TAIRYO = (0, 0, 255)       # 大漁の表示色(青)
ERROR = (255, 0, 0)        # エラーの表示色(赤)
RED = (255, 0, 0)

# エラー点滅の速さ[Hz]。点滅周期 = 1/ERROR_BLINK_HZ 秒。
# 0.7 なら約1.4秒周期(約0.7秒点灯 -> 約0.7秒消灯)の「長めの点滅」。
ERROR_BLINK_HZ = 0.7

# manual_control の生存タイムアウト[秒]。/manual/mode は 50Hz(20ms周期)で出るので、
# 0.5秒来なければ落ちている/起動前とみなす。
MANUAL_ALIVE_TIMEOUT_S = 0.5


# ---------- IRコード -> 色 (16進大文字・0xなし) ----------
# 今わかっているのは 0x19 = 赤 のみ。他コードの色は判明し次第ここに足す。
#   例) COLOR_TABLE["16"] = (0, 0, 255)   # 0x16 = 青、など。好きなRGBでよい。
# ※現仕様(通常=白)では下の ir_color 層を無効にしているため、この表は今は使われない。
COLOR_TABLE = {
    "19": RED,     # 0x19 = 赤(確定)
}


# ---------- スタイル(点滅対応の色) ----------
class Style:
    """出す見た目。solid か、2色を blink_hz でトグルする点滅。"""

    def __init__(self, color: RGB, blink_hz: float = 0.0, color2: RGB = OFF):
        self.color = color
        self.blink_hz = blink_hz
        self.color2 = color2

    def resolve(self, t: float) -> RGB:
        if self.blink_hz > 0.0:
            on = int(t * self.blink_hz * 2) % 2 == 0
            return self.color if on else self.color2
        return self.color


# ---------- 層(layer) ----------
@dataclass
class Layer:
    name: str
    topic: str
    msg_type: type
    # 最新メッセージ -> 出すスタイル。None なら「この層は今アクティブでない」
    style_of: Callable[[object], Optional[Style]]
    priority: int          # 大きいほど優先
    # 最後の受信からこの秒数を過ぎたら層を無効にする。0 なら期限なし(ラッチ向け)。
    # 「そのトピックが今も流れているか」を条件にしたい層で使う。
    timeout: float = 0.0


def _ir_style(msg: String) -> Optional[Style]:
    color = COLOR_TABLE.get(msg.data.strip().upper())
    return Style(color if color is not None else UNKNOWN)


# priority で比較するので並び順は自由。同じトピックを複数層で使ってもよい。
LAYERS = [
    # エラー -- 最優先。IRリンク断のあいだ赤のゆっくり点滅。
    Layer("link_lost", "ir/link_ok", Bool,
          lambda m: Style(ERROR, blink_hz=ERROR_BLINK_HZ) if not m.data else None,
          priority=100),

    # 大漁 -- 中優先。青。
    # ir/tairyo はラッチなので、一度 true になると次の確定コードが来るまで青のまま。
    # 「大漁を数秒だけ光らせる」にしたければ timeout=3.0 などを足す。
    Layer("tairyo", "ir/tairyo", Bool,
          lambda m: Style(TAIRYO) if m.data else None,
          priority=50),

    # 通常 -- ベース。manual_control_natsu26 が走っているあいだ白。
    # /manual/mode は mc_2026 が 20ms 周期で出し続けるので、これを生存信号に使う。
    # 中身(DRIVE/GET_EEL/SHOOT)は見ない。途切れたら timeout で層ごと無効 -> 消灯。
    Layer("manual_alive", "/manual/mode", String,
          lambda m: Style(WHITE),
          priority=0, timeout=MANUAL_ALIVE_TIMEOUT_S),

    # --- IR受信コードを色に変換する層(現仕様では無効。使うならコメントを外す) ---
    # Layer("ir_color", "ir/state", String, _ir_style, priority=10),

    # --- 任意トピックを重ねる例(必要になったらコメントを外し、上の import に型を足す) ---
    # Layer("auto_mode", "drive/mode", String,
    #       lambda m: Style((0, 0, 255)) if m.data == "auto" else None,
    #       priority=30),
]


def to_rgb565(color: RGB) -> int:
    """(r,g,b) 0..255 -> RGB565 の 16bit値。int16 に収まるよう符号付きに畳む。"""
    r, g, b = color
    v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)   # 0..65535
    return v - 0x10000 if v >= 0x8000 else v               # -> int16 範囲


def latched_qos():
    # ir_node 側のラッチ publish を取りこぼさないよう TRANSIENT_LOCAL で購読。
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class IrLedPolicy(Node):
    def __init__(self):
        super().__init__("ir_led_policy")

        self.pub = self.create_publisher(Int16MultiArray, TX_TOPIC, 10)
        self.latest = {}          # topic -> (最新メッセージ, 受信時刻[秒])
        self.last_val = None      # 直近に送った data[9] の値。重複送信を抑制

        # 層が使うトピックを重複なく購読する(同じトピックを複数層が使ってもOK)。
        seen = set()
        for lyr in LAYERS:
            if lyr.topic in seen:
                continue
            seen.add(lyr.topic)
            qos = latched_qos() if lyr.topic.startswith("ir/") else 10
            self.create_subscription(
                lyr.msg_type, lyr.topic,
                lambda m, t=lyr.topic: self.on_msg(t, m),
                qos)

        # 20Hz で再評価。点滅もここで進む。色(RGB565値)が変わった時だけ送信。
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"IR LED policy up -> {TX_TOPIC}.data[{LED_SLOT}] (RGB565)")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_msg(self, topic: str, msg):
        # 受信時刻も残す。timeout 付きの層(生存監視)がこれを見る。
        self.latest[topic] = (msg, self.now_s())

    def tick(self):
        now = self.now_s()
        style = self.decide(now)
        val = to_rgb565(style.resolve(now))
        if val == self.last_val:
            return
        self.last_val = val
        arr = Int16MultiArray()
        arr.data = [0] * TX_SLOTS
        arr.data[LED_SLOT] = val
        self.pub.publish(arr)

    def decide(self, now: float) -> Style:
        """アクティブな層のうち優先度が最大のスタイルを返す。無ければ消灯。"""
        best = Style(OFF)
        best_pri = None
        for lyr in LAYERS:
            entry = self.latest.get(lyr.topic)
            if entry is None:
                continue
            msg, t_recv = entry
            # timeout 付きの層は、最後の受信から時間が経ちすぎていたら無効。
            if lyr.timeout > 0.0 and now - t_recv > lyr.timeout:
                continue
            style = lyr.style_of(msg)
            if style is None:
                continue
            if best_pri is None or lyr.priority > best_pri:
                best, best_pri = style, lyr.priority
        return best


def main():
    rclpy.init()
    node = IrLedPolicy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
