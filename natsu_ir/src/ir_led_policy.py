#!/usr/bin/env python3
"""Natsurobo IR LED ポリシーノード -- 「何色を出すか」の判断だけを担う。

ir_node.py はあえて判断を持たない素通しブリッジ(ir/led_cmd に来た色を ESP32 へ流すだけ)。
このノードがその上に乗り、最終的な表示色を決めて ir/led_cmd に publish する。

    受信IRコード / 他トピック  --(このノードで判断)-->  ir/led_cmd  --(ir_node)-->  ESP32 NeoPixel

色は「層(layer)」を優先度つきで重ねて決める:
  - ベース層 : ir/state の受信コードを COLOR_TABLE で色に変換(優先度 低)。
  - 上書き層 : link_ok / tairyo / 任意トピック。値が条件を満たす層のうち
               優先度が最大のものが色を上書きする(優先度 高)。

購読:
  ir/state    std_msgs/String   受信コード(16進, 例 "19")。COLOR_TABLE で色へ。ラッチ受信。
  ir/link_ok  std_msgs/Bool     false のとき警告(赤点滅)で最優先に上書き。
  ir/tairyo   std_msgs/Bool     true のとき大漁色で上書き。
  (LAYERS に足せば)任意トピック  値の条件で色を上書き(下の drive/mode 例を参照)。

publish:
  ir/led_cmd  std_msgs/ColorRGBA  最終色 (r,g,b は 0..1, a=1)。色が変わった時だけ送る。

色を増やすとき:
  - IRコードに色を割り当てる      -> COLOR_TABLE に "コード": rgba(...) を足す。
  - 別トピックで色を上書きしたい  -> LAYERS に Layer(...) を1行足す(msg_type も import する)。
"""

from dataclasses import dataclass
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, ColorRGBA, String
# 任意トピックを重ねるときは、その型もここで import する(例: Int32, Float32 ...)


# ---------- 色の定義 (r,g,b は 0..1) ----------
def rgba(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


OFF = rgba(0, 0, 0)
UNKNOWN = rgba(0.15, 0.15, 0.15)   # 検出はしたが色未確定のコード
RED = rgba(1, 0, 0)
TAIRYO = rgba(0, 1, 0)             # 大漁の表示色(暫定・緑)
WARN = rgba(1, 0.25, 0)            # link断の警告(橙)


# ---------- IRコード -> 色 (16進大文字・0xなし) ----------
# 今わかっているのは 0x19 = 赤 のみ。他コードの色は判明し次第ここに足す。
#   例) COLOR_TABLE["16"] = rgba(0, 0, 1)   # 0x16 = 青、など
COLOR_TABLE = {
    "19": RED,     # 0x19 = 赤(確定)
}


# ---------- スタイル(点滅対応の色) ----------
class Style:
    """出す見た目。solid か、2色 blink_hz でトグルする点滅。"""

    def __init__(self, color: ColorRGBA, blink_hz: float = 0.0,
                 color2: ColorRGBA = OFF):
        self.color = color
        self.blink_hz = blink_hz
        self.color2 = color2

    def resolve(self, t: float) -> ColorRGBA:
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


def _ir_style(msg: String) -> Optional[Style]:
    color = COLOR_TABLE.get(msg.data.strip().upper())
    return Style(color if color is not None else UNKNOWN)


# 上から優先度順に並べる必要はない(priority で比較する)。同じトピックを複数層で使ってもよい。
LAYERS = [
    # 通信断の警告 -- 最優先。link_ok=false のあいだ赤点滅。
    Layer("link_lost", "ir/link_ok", Bool,
          lambda m: Style(WARN, blink_hz=3.0) if not m.data else None,
          priority=100),

    # 大漁 -- 中優先。
    Layer("tairyo", "ir/tairyo", Bool,
          lambda m: Style(TAIRYO) if m.data else None,
          priority=50),

    # ベース -- IR受信コードを色に変換。常にアクティブ(未知コードは UNKNOWN)。
    Layer("ir_color", "ir/state", String, _ir_style, priority=0),

    # --- 任意トピックを重ねる例(必要になったらコメントを外し、上の import に型を足す) ---
    # from std_msgs.msg import String  # drive/mode が String の場合
    # Layer("auto_mode", "drive/mode", String,
    #       lambda m: Style(rgba(0, 0, 1)) if m.data == "auto" else None,
    #       priority=30),
]


def latched_qos():
    # ir_node 側のラッチ publish を取りこぼさないよう TRANSIENT_LOCAL で購読。
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class IrLedPolicy(Node):
    def __init__(self):
        super().__init__("ir_led_policy")

        self.pub = self.create_publisher(ColorRGBA, "ir/led_cmd", 10)
        self.latest = {}          # topic -> 最新メッセージ
        self.last_rgb = None      # 直近に送った (r,g,b) 0..255。重複送信を抑制

        # 層が使うトピックを重複なく購読する(同じトピックを複数層が使ってもOK)。
        seen = {}
        for lyr in LAYERS:
            key = (lyr.topic, lyr.msg_type)
            if key in seen:
                continue
            seen[key] = True
            qos = latched_qos() if lyr.topic.startswith("ir/") else 10
            self.create_subscription(
                lyr.msg_type, lyr.topic,
                lambda m, t=lyr.topic: self.latest.__setitem__(t, m),
                qos)

        # 20Hz で再評価。点滅もここで進む。
        self.create_timer(0.05, self.tick)
        self.get_logger().info("IR LED policy up")

    def tick(self):
        style = self.decide()
        if style is None:
            return
        color = style.resolve(self.get_clock().now().nanoseconds / 1e9)
        self.publish_if_changed(color)

    def decide(self) -> Optional[Style]:
        """アクティブな層のうち優先度が最大のスタイルを返す。無ければ None。"""
        best = None
        best_pri = None
        for lyr in LAYERS:
            msg = self.latest.get(lyr.topic)
            if msg is None:
                continue
            style = lyr.style_of(msg)
            if style is None:
                continue
            if best_pri is None or lyr.priority > best_pri:
                best, best_pri = style, lyr.priority
        return best

    def publish_if_changed(self, color: ColorRGBA):
        rgb = (max(0, min(255, int(color.r * 255))),
               max(0, min(255, int(color.g * 255))),
               max(0, min(255, int(color.b * 255))))
        if rgb == self.last_rgb:
            return
        self.last_rgb = rgb
        self.pub.publish(color)


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
