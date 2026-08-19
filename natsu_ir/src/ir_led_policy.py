#!/usr/bin/env python3
"""Natsurobo IR LED ポリシーノード -- 「何色を出すか」の判断だけを担う。

ir_node.py はあえて判断を持たない素通しブリッジ。このノードがその上に乗り、
受信IRコードや他トピックから色を決めて serial_bridge 経由で LED 用マイコン
(DEVICE_ID=4) の WS2812B へ送る。

    受信IRコード / 他トピック --(このノードで判断)--> ir/led_color (RGB565)
        --(manual_charge/charge_node が data_[9] に載せる)--> serial_tx_4 --> ID=4 MCU --> WS2812B

今の表示仕様:
  操作モード表示 (manual_control_natsu26 が走っている間) /manual/mode の中身で色を変える
      DRIVE   走行モード   青   (#58a6ff)
      GET_EEL 捕獲モード   緑   (#3fb950)
      SHOOT   射出モード   橙   (#f0883e)
    色は natsu_console_2026 の PS4ControllerPanel のモードバッジと同じ値に揃えてある。
    片方を変えたらもう片方も変えること。
  大漁時                                          虹色(色相が2.5秒で一周)
  エラー時 (IRリンク断)                           赤のゆっくり点滅
  上記いずれでもない (manual 未起動など)          消灯

色の送り方 (RGB565, int16 1個):
  serial_tx は int16 の配列。フルカラー(R8G8B8=24bit)は1枠に入らないので、
  16bit の RGB565 (R5 G6 B5) に圧縮して1スロットに収める。約6.5万色ぶん、ほぼ自由な色。
  RGB565 は最大 0xFFFF で符号付き int16 の範囲(-32768..32767)を超えるため、
  そのままだと負の数として格納されるが、シリアル上のバイト列は同じ。MCU 側で uint16 として
  読み、RGB888 に展開して WS2812B を光らせる。点灯/点滅の判断はすべてこのノードが持つ
  (MCU は受け取った色を出すだけ)。

出力先(重要):
  このノードは serial_tx_4 に直接 publish しない。serial_bridge は publish のたびに
  24スロット全部をマイコンへ送る(部分更新できない)ため、serial_tx_4 に複数ノードが
  publish すると互いのスロットを潰し合う。ID=4 は manual_charge/charge_node が
  50Hz で publish しているので、そちらに色だけ渡し、data_[9] に載せてもらう。
  実際に光らせるには、ID=4 ファーム側に「Rx_16Data[9] を RGB565 色として読み LED を出す」
  処理が必要(Rx_16Data[9] は標準 esp32_serial_bridge ファームでは SERVO1 の枠)。

購読:
  /manual/mode std_msgs/String   操作モード("DRIVE"/"GET_EEL"/"SHOOT")。色に対応させる。
                                 同時に生存信号でもあり、50Hz で出続けるので途切れたら
                                 「走っていない」と判定して消灯する。
  ir/link_ok   std_msgs/Bool     false のときエラー(赤のゆっくり点滅)で最優先に上書き。ラッチ受信。
  ir/tairyo    std_msgs/Bool     true のとき大漁色(虹色)で上書き。ラッチ受信。
  ir/state     std_msgs/String   受信コード(16進, 例 "19")。今は未使用(下の ir_color 層を参照)。
  (LAYERS に足せば)任意トピック  値の条件で色を上書き。

publish:
  ir/led_color std_msgs/Int16   RGB565 を int16 に畳んだ値。色が変わった時だけ送る。
                                charge_node がこれを serial_tx_4.data[9] に載せる。
"""

import colorsys
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Int16, String
# 任意トピックを重ねるときは、その型もここで import する(例: Int32, Float32 ...)


# ---------- 送信先 ----------
# 色だけを publish し、serial_tx_4 への実際の送信は charge_node に任せる。
# (serial_tx_4 に publish するノードを1つに保たないとスロットを潰し合うため)
TX_TOPIC = "ir/led_color"

RGB = Tuple[int, int, int]   # 各 0..255


# ---------- 色の定義 (r,g,b は 0..255) ----------
OFF = (0, 0, 0)
UNKNOWN = (40, 40, 40)     # 検出はしたが色未確定のコード(弱い白)
WHITE = (255, 255, 255)    # モード不明(未知の文字列が来たとき)のフォールバック
TAIRYO = (0, 0, 255)       # 大漁の表示色(今は虹色を使うので未使用。単色に戻すとき用に残す)
ERROR = (255, 0, 0)        # エラーの表示色(赤)
RED = (255, 0, 0)

# ---------- 操作モード -> 色 ----------
# mc_2026 が /manual/mode に流す文字列(SHARE で DRIVE -> GET_EEL -> SHOOT と巡回)に対応。
# 値は natsu_console_2026/src/components/PS4ControllerPanel.js のモードバッジと同じ。
# GUI と機体の LED で色がずれると操縦者が混乱するので、片方だけ変えないこと。
MODE_COLORS = {
    "DRIVE":   (0x58, 0xa6, 0xff),   # #58a6ff 走行モード(青)
    "GET_EEL": (0x3f, 0xb9, 0x50),   # #3fb950 捕獲モード(緑)
    "SHOOT":   (0xf0, 0x88, 0x3e),   # #f0883e 射出モード(橙)
}


# エラー点滅の速さ[Hz]。点滅周期 = 1/ERROR_BLINK_HZ 秒。
# 0.7 なら約1.4秒周期(約0.7秒点灯 -> 約0.7秒消灯)の「長めの点滅」。
ERROR_BLINK_HZ = 0.7

# 大漁の虹色が一周する速さ[Hz]。0.4 なら 2.5 秒で赤->緑->青->赤 と一巡する。
# 評価は 20Hz なので、この値を上げすぎる(2.0以上)とカクついて見える。
TAIRYO_RAINBOW_HZ = 0.4

# manual_control の生存タイムアウト[秒]。/manual/mode は 50Hz(20ms周期)で出るので、
# 0.5秒来なければ落ちている/起動前とみなす。
MANUAL_ALIVE_TIMEOUT_S = 0.5


# ---------- IRコード -> 色 (16進大文字・0xなし) ----------
# 今わかっているのは 0x19 = 赤 のみ。他コードの色は判明し次第ここに足す。
#   例) COLOR_TABLE["16"] = (0, 0, 255)   # 0x16 = 青、など。好きなRGBでよい。
# ※現仕様(通常=操作モード色)では下の ir_color 層を無効にしているため、この表は今は使われない。
COLOR_TABLE = {
    "19": RED,     # 0x19 = 赤(確定)
}


def hue_rgb(hue: float, sat: float = 1.0, val: float = 1.0) -> RGB:
    """色相(0.0〜1.0の一周) -> RGB。虹色はこれを時間で回して作る。"""
    r, g, b = colorsys.hsv_to_rgb(hue % 1.0, sat, val)
    return (int(r * 255), int(g * 255), int(b * 255))


# ---------- スタイル(点滅・虹色対応の色) ----------
class Style:
    """出す見た目。solid か、2色を blink_hz でトグルする点滅か、虹色(rainbow_hz)。"""

    def __init__(self, color: RGB = OFF, blink_hz: float = 0.0, color2: RGB = OFF,
                 rainbow_hz: float = 0.0):
        self.color = color
        self.blink_hz = blink_hz
        self.color2 = color2
        # >0 なら色相を時間で回す虹色。1秒間に何周するか。color/blink_hz より優先。
        self.rainbow_hz = rainbow_hz

    def resolve(self, t: float) -> RGB:
        if self.rainbow_hz > 0.0:
            return hue_rgb((t * self.rainbow_hz) % 1.0)
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


def _mode_style(msg: String) -> Optional[Style]:
    """/manual/mode の中身をモード色にする。未知の文字列でも消さずに白で光らせる
    (このトピックが来ている = manual_control は生きている、という表示は残したいため)。"""
    return Style(MODE_COLORS.get(msg.data.strip().upper(), WHITE))


def _ir_style(msg: String) -> Optional[Style]:
    color = COLOR_TABLE.get(msg.data.strip().upper())
    return Style(color if color is not None else UNKNOWN)


# priority で比較するので並び順は自由。同じトピックを複数層で使ってもよい。
LAYERS = [
    # エラー -- 最優先。IRリンク断のあいだ赤のゆっくり点滅。
    Layer("link_lost", "ir/link_ok", Bool,
          lambda m: Style(ERROR, blink_hz=ERROR_BLINK_HZ) if not m.data else None,
          priority=100),

    # 大漁 -- 中優先。虹色(色相が回る)。走行モードの青と紛れないよう単色にはしない。
    # ir/tairyo はラッチなので、一度 true になると次の確定コードが来るまで虹色のまま。
    # 「大漁を数秒だけ光らせる」にしたければ timeout=3.0 などを足す。
    # 単色(青)に戻すなら Style(TAIRYO) にする。
    Layer("tairyo", "ir/tairyo", Bool,
          lambda m: Style(rainbow_hz=TAIRYO_RAINBOW_HZ) if m.data else None,
          priority=50),

    # 通常 -- ベース。manual_control_natsu26 が走っているあいだ、操作モードの色。
    # /manual/mode は mc_2026 が 20ms 周期で出し続けるので、生存信号も兼ねる。
    # 途切れたら timeout で層ごと無効 -> 消灯。
    Layer("manual_mode", "/manual/mode", String,
          _mode_style,
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

        self.pub = self.create_publisher(Int16, TX_TOPIC, 10)
        self.latest = {}          # topic -> (最新メッセージ, 受信時刻[秒])
        self.last_val = None      # 直近に送った RGB565 値。重複送信を抑制

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
        self.get_logger().info(f"IR LED policy up -> {TX_TOPIC} (RGB565)")

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
        self.pub.publish(Int16(data=val))

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
