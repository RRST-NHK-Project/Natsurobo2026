#!/usr/bin/env python3
"""Natsurobo IR LED ポリシーノード -- 「何色を出すか」の判断だけを担う。

ir_node.py はあえて判断を持たない素通しブリッジ。このノードがその上に乗り、
操作モードや大漁フラグなどの各トピックから色を決めて serial_bridge 経由で
LED 用マイコン (DEVICE_ID=4) の WS2812B へ送る。

    /manual/mode, ir/tairyo, /manual/tairyo, ... --(このノードで判断)--> ir/led_color (RGB565)
        --(charge_node が data_[9]〜data_[12] に載せる)--> serial_tx_4 --> ID=4 MCU --> WS2812B

今の表示仕様:
  操作モード表示 (manual_control_natsu26 が走っている間) /manual/mode の中身で色を変える
      DRIVE   走行モード   青   (#58a6ff)
      GET_EEL 捕獲モード   緑   (#3fb950)
      SHOOT   射出モード   橙   (#f0883e)
    色は natsu_console_2026 の PS4ControllerPanel のモードバッジと同じ値に揃えてある。
    片方を変えたらもう片方も変えること。
  大漁時                                          虹色(色相が2.5秒で一周)
  エラー時 (IRリンク断)                           赤のゆっくり点滅
    大漁とエラーが重なった時は大漁(虹色)を優先する。理由は下の LAYERS のコメント参照。
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
  50Hz で publish しているので、そちらに色だけ渡し、data_[9]〜data_[12] に載せてもらう。
  LED は4本あり、4本とも同じ色なので同じ値が4スロットに入る。
  実際に光らせるには、ID=4 ファーム側に「Rx_16Data[9]〜[12] を RGB565 色として読み LED を出す」
  処理が必要(Rx_16Data[9]〜[12] は標準 esp32_serial_bridge ファームでは SERVO1〜SERVO4 の枠)。

購読:
  /manual/mode std_msgs/String   操作モード("DRIVE"/"GET_EEL"/"SHOOT")。色に対応させる。
                                 同時に生存信号でもあり、50Hz で出続けるので途切れたら
                                 「走っていない」と判定して消灯する。
  ir/link_ok   std_msgs/Bool     false のときエラー(赤のゆっくり点滅)で上書き。ラッチ受信。
                                 大漁中は大漁が優先される(ir_node は状態変化時しか publish
                                 しないので、一度 false になると永久に残り続けるため)。
  ir/tairyo    std_msgs/Bool     true のとき大漁色(虹色)で上書き。ラッチ受信。
  /manual/tairyo std_msgs/Bool   操縦者がPS4のL3+R3同時押しで出す大漁宣言。同じく虹色。
                                 ラッチ受信。mc_2026 が publish する。
  (LAYERS に足せば)任意トピック  値の条件で色を上書きできる。ir/state(受信IRコード)を
                                 色にしたい場合も LAYERS に1行足すだけ(下の例を参照)。

publish:
  ir/led_color std_msgs/Int16   RGB565 を int16 に畳んだ値。色が変わった時だけ送る。
                                charge_node がこれを serial_tx_4.data[9]〜[12] に載せる。
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


# =====================================================================
# 色の設定 -- 表示色を変えたいときはこのブロックだけ書き換える
#
# 色は (R, G, B) の3つの数字。それぞれ 0〜255 で、明るさもここで決まる。
#   (255,   0,   0) = 赤      (  0, 255,   0) = 緑      (  0,   0, 255) = 青
#   (255, 255, 255) = 白      (255, 128,   0) = 橙      (  0,   0,   0) = 消灯
#   (128, 128, 128) = 白の半分の明るさ (全成分を等倍で下げると暗くなる)
# カラーピッカーの #RRGGBB をそのまま使いたいときは hex_rgb("#58a6ff") と書ける。
# =====================================================================

OFF = (0, 0, 0)            # 消灯
WHITE = (255, 255, 255)    # モード名が未知の文字列だった時のフォールバック


def hex_rgb(code: str) -> "RGB":
    """"#58a6ff" や "58a6ff" -> (88, 166, 255)。GUIの色をそのまま貼れるようにする。"""
    c = code.lstrip("#")
    return (int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16))


# 操作モード(/manual/mode の中身) -> 色。
# mc_2026 が SHARE で DRIVE -> GET_EEL -> SHOOT と巡回させて流してくる。
# 値は natsu_console_2026/src/components/PS4ControllerPanel.js のモードバッジと同じ。
# GUI と機体の LED で色がずれると操縦者が混乱するので、片方だけ変えないこと。
# モードを増やしたら、ここに1行足すだけで色が付く(コードの変更は不要)。
MODE_COLORS = {
    "DRIVE":   (88, 166, 255),   # #58a6ff 走行モード(青)
    "GET_EEL": (63, 185, 80),    # #3fb950 捕獲モード(緑)
    "SHOOT":   (225, 225, 0),   # #f0883e 射出モード(黃)
}

# IRリンク断のエラー色(赤)。点滅の速さは ERROR_BLINK_HZ。
ERROR = (255, 0, 0)

# 大漁を単色にしたいとき用。今は虹色(TAIRYO_RAINBOW_HZ)なので使っていない。
# 戻すときは LAYERS の tairyo 層を Style(TAIRYO) にする。
TAIRYO = (0, 0, 255)


# エラー点滅の速さ[Hz]。点滅周期 = 1/ERROR_BLINK_HZ 秒。
# 0.7 なら約1.4秒周期(約0.7秒点灯 -> 約0.7秒消灯)の「長めの点滅」。
ERROR_BLINK_HZ = 0.7

# 大漁の虹色が一周する速さ[Hz]。0.4 なら 2.5 秒で赤->緑->青->赤 と一巡する。
# 評価は 20Hz なので、この値を上げすぎる(2.0以上)とカクついて見える。
TAIRYO_RAINBOW_HZ = 0.4

# manual_control の生存タイムアウト[秒]。/manual/mode は 50Hz(20ms周期)で出るので、
# 0.5秒来なければ落ちている/起動前とみなす。
MANUAL_ALIVE_TIMEOUT_S = 0.5



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
    # 購読QoSをラッチ(TRANSIENT_LOCAL)にするか。publish側がラッチで流していて、
    # かつ「状態が変わった時しか publish しない」トピックは必ず True にする。
    # False(既定)だと、このノードが後から起動した時に最後の値を取りこぼし、
    # 次に状態が変わるまでその層が永久に非アクティブのままになる。
    latched: bool = False


def _mode_style(msg: String) -> Optional[Style]:
    """/manual/mode の中身をモード色にする。未知の文字列でも消さずに白で光らせる
    (このトピックが来ている = manual_control は生きている、という表示は残したいため)。"""
    return Style(MODE_COLORS.get(msg.data.strip().upper(), WHITE))


# priority で比較するので並び順は自由。同じトピックを複数層で使ってもよい。
#
# 優先度の考え方(上ほど強い):
#   120 大漁(手動宣言)  審判に見せる合図。試合を決めるので何にも潰されてはいけない
#   110 大漁(IR受信)    同上
#   100 エラー(IRリンク断)  大漁中は隠れる。IRリンクの状態は端末ログとGUIでも見えるため
#     0 操作モード色     ベース
#
# 大漁をエラーより上に置いてあるのは意図的。ir/link_ok は「状態が変わった時だけ」
# publish されるラッチなので、IR基板のHBが一度途切れると false が永久に残り続ける。
# エラーを最優先のままにすると、その後どれだけ大漁を宣言しても赤点滅に潰される。
LAYERS = [
    # エラー -- IRリンク断のあいだ赤のゆっくり点滅。大漁中だけは大漁が優先される。
    Layer("link_lost", "ir/link_ok", Bool,
          lambda m: Style(ERROR, blink_hz=ERROR_BLINK_HZ) if not m.data else None,
          priority=100, latched=True),

    # 大漁(IR受信) -- 虹色(色相が回る)。走行モードの青と紛れないよう単色にはしない。
    # ir/tairyo はラッチなので、一度 true になると次の確定コードが来るまで虹色のまま。
    # 「大漁を数秒だけ光らせる」にしたければ timeout=3.0 などを足す。
    # 単色(青)に戻すなら Style(TAIRYO) にする。
    Layer("tairyo", "ir/tairyo", Bool,
          lambda m: Style(rainbow_hz=TAIRYO_RAINBOW_HZ) if m.data else None,
          priority=110, latched=True),

    # 大漁(手動宣言) -- 操縦者が PS4 の L3+R3 同時押しで出す合図。mc_2026 が
    # /manual/tairyo にラッチで流す。IR受信由来の ir/tairyo と見た目は同じ虹色だが、
    # トピックを分けてある(同じトピックに2ノードが publish すると状態が食い違うため)。
    # もう一度 L3+R3 を押せば false が来て、下のモード色に戻る。
    Layer("tairyo_manual", "/manual/tairyo", Bool,
          lambda m: Style(rainbow_hz=TAIRYO_RAINBOW_HZ) if m.data else None,
          priority=120, latched=True),

    # 通常 -- ベース。manual_control_natsu26 が走っているあいだ、操作モードの色。
    # /manual/mode は mc_2026 が 20ms 周期で出し続けるので、生存信号も兼ねる。
    # 途切れたら timeout で層ごと無効 -> 消灯。
    Layer("manual_mode", "/manual/mode", String,
          _mode_style,
          priority=0, timeout=MANUAL_ALIVE_TIMEOUT_S),

    # --- 層を足す例(必要になったらコメントを外し、上の import に型を足す) ---
    # 条件に合えば Style を、合わなければ None を返す関数を書くだけで層が増える。
    # 下は「受信IRコードが 0x19 なら紫」の例。ir/state は "19" のような16進文字列。
    # Layer("ir_code", "ir/state", String,
    #       lambda m: Style((160, 0, 255)) if m.data.strip().upper() == "19" else None,
    #       priority=10),
]


def to_rgb565(color: RGB) -> int:
    """(r,g,b) 0..255 -> RGB565 の 16bit値。int16 に収まるよう符号付きに畳む。"""
    r, g, b = color
    v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)   # 0..65535
    return v - 0x10000 if v >= 0x8000 else v               # -> int16 範囲


def latched_qos():
    # publish側のラッチ(TRANSIENT_LOCAL)を取りこぼさないための購読QoS。
    # ir_node の ir/* と mc_2026 の /manual/tairyo が該当する。
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class IrLedPolicy(Node):
    def __init__(self):
        super().__init__("ir_led_policy")

        self.pub = self.create_publisher(Int16, TX_TOPIC, 10)
        self.latest = {}          # topic -> (最新メッセージ, 受信時刻[秒])
        self.last_val = None      # 直近に送った RGB565 値。重複送信を抑制

        # 層が使うトピックを重複なく購読する(同じトピックを複数層が使ってもOK)。
        # ラッチで購読するかは Layer.latched で決める。同じトピックを複数の層が使い、
        # 片方だけ latched=True の場合はラッチ側に寄せる(取りこぼす方が害が大きい)。
        latched_topics = {lyr.topic for lyr in LAYERS if lyr.latched}
        seen = set()
        for lyr in LAYERS:
            if lyr.topic in seen:
                continue
            seen.add(lyr.topic)
            qos = latched_qos() if lyr.topic in latched_topics else 10
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
