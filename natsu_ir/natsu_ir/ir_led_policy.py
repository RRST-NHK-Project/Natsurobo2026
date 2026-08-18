#!/usr/bin/env python3
"""Natsurobo IR LED ポリシーノード -- 「何色を出すか」の判断だけを担う。

ir_node.py はあえて判断を持たない素通しブリッジ。このノードがその上に乗り、
受信IRコードや他トピックから「色ID」を決めて serial_bridge 経由で LED 用マイコン
(DEVICE_ID=4) へ送る。

    受信IRコード / 他トピック --(このノードで判断)--> serial_tx_4.data[9] = 色ID
                                                        --(serial_bridge)--> ID=4 MCU --> WS2812B

なぜ「色ID」か:
  serial_tx は int16 の配列。1スロット(16bit)ではフルカラー(R8G8B8=24bit)は入らないので、
  ここでは小さな整数(色ID)だけ送り、色ID->RGB の対応(パレット)と WS2812B の点灯/点滅は
  受信側マイコンのファームが持つ。ROS 側は「今どの状態か」を番号で伝えるだけ。

出力先(重要):
  serial_tx_4 の data[9] は、標準 esp32_serial_bridge ファームでは Rx_16Data[9] = SERVO1 の枠。
  WS2812B を光らせるには ID=4 ファーム側で「Rx_16Data[9] を色IDとして読み LED を出す」処理が要る。
  この serial_tx_4 は IR 専用(他ノードは publish しない)前提なので、毎回24スロットを送ってよい。

購読:
  ir/state    std_msgs/String   受信コード(16進, 例 "19")。CODE_ID で色IDへ。ラッチ受信。
  ir/link_ok  std_msgs/Bool     false のとき警告ID(最優先)。MCU側で橙点滅させる想定。
  ir/tairyo   std_msgs/Bool     true のとき大漁ID。
  (LAYERS に足せば)任意トピック  値の条件で色IDを上書き。

publish:
  serial_tx_4 std_msgs/Int16MultiArray   24スロット。data[9] に色ID、他は 0。ID変化時のみ送る。
"""

from dataclasses import dataclass
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Int16MultiArray, String
# 任意トピックを重ねるときは、その型もここで import する(例: Int32, Float32 ...)


# ---------- 送信先 ----------
TX_TOPIC = "serial_tx_4"   # DEVICE_ID=4 の MCU へ
TX_SLOTS = 24              # serial_bridge の int16 スロット数
LED_SLOT = 9              # data[9] に色IDを入れる(0始まり)


# ---------- 色ID ----------
# 0 は消灯。色は 1 から。状態(大漁/通信断)は衝突しないよう大きい番号にしておく。
ID_OFF = 0
ID_UNKNOWN = 90     # IRは受信したが色が未確定のコード(MCU側で弱く点灯など)
ID_TAIRYO = 91      # 大漁
ID_LINK_LOST = 99   # IR受信リンク断(MCU側で橙点滅させる想定)

# IRコード(16進大文字・0xなし) -> 色ID
# 今わかっているのは 0x19 = 赤 のみ。色が判明したらここに足す。
#   例) CODE_ID["16"] = 2   # 0x16 = 青、など。RGBの実体は MCU のパレットで定義。
CODE_ID = {
    "19": 1,     # 0x19 = 赤 = 1
}


# ---------- 層(layer) ----------
@dataclass
class Layer:
    name: str
    topic: str
    msg_type: type
    # 最新メッセージ -> 出す色ID。None なら「この層は今アクティブでない」
    id_of: Callable[[object], Optional[int]]
    priority: int          # 大きいほど優先


def _ir_id(msg: String) -> Optional[int]:
    return CODE_ID.get(msg.data.strip().upper(), ID_UNKNOWN)


# priority で比較するので並び順は自由。同じトピックを複数層で使ってもよい。
LAYERS = [
    # 通信断の警告 -- 最優先。link_ok=false のあいだ警告ID。
    Layer("link_lost", "ir/link_ok", Bool,
          lambda m: ID_LINK_LOST if not m.data else None,
          priority=100),

    # 大漁 -- 中優先。
    Layer("tairyo", "ir/tairyo", Bool,
          lambda m: ID_TAIRYO if m.data else None,
          priority=50),

    # ベース -- IR受信コードを色IDに変換。常にアクティブ(未知コードは ID_UNKNOWN)。
    Layer("ir_color", "ir/state", String, _ir_id, priority=0),

    # --- 任意トピックを重ねる例(必要になったらコメントを外し、上の import に型を足す) ---
    # Layer("auto_mode", "drive/mode", String,
    #       lambda m: 2 if m.data == "auto" else None,   # 例: 走行中は色ID=2
    #       priority=30),
]


def latched_qos():
    # ir_node 側のラッチ publish を取りこぼさないよう TRANSIENT_LOCAL で購読。
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class IrLedPolicy(Node):
    def __init__(self):
        super().__init__("ir_led_policy")

        self.pub = self.create_publisher(Int16MultiArray, TX_TOPIC, 10)
        self.latest = {}          # topic -> 最新メッセージ
        self.last_id = None       # 直近に送った色ID。重複送信を抑制

        # 層が使うトピックを重複なく購読する(同じトピックを複数層が使ってもOK)。
        seen = set()
        for lyr in LAYERS:
            if lyr.topic in seen:
                continue
            seen.add(lyr.topic)
            qos = latched_qos() if lyr.topic.startswith("ir/") else 10
            self.create_subscription(
                lyr.msg_type, lyr.topic,
                lambda m, t=lyr.topic: self.latest.__setitem__(t, m),
                qos)

        # 10Hz で再評価。ID が変わった時だけ送信。
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f"IR LED policy up -> {TX_TOPIC}.data[{LED_SLOT}]")

    def tick(self):
        led_id = self.decide()
        if led_id == self.last_id:
            return
        self.last_id = led_id
        arr = Int16MultiArray()
        arr.data = [0] * TX_SLOTS
        arr.data[LED_SLOT] = int(led_id)
        self.pub.publish(arr)
        self.get_logger().info(f"led id -> {led_id}")

    def decide(self) -> int:
        """アクティブな層のうち優先度が最大の色IDを返す。無ければ消灯。"""
        best_id = ID_OFF
        best_pri = None
        for lyr in LAYERS:
            msg = self.latest.get(lyr.topic)
            if msg is None:
                continue
            led_id = lyr.id_of(msg)
            if led_id is None:
                continue
            if best_pri is None or lyr.priority > best_pri:
                best_id, best_pri = led_id, lyr.priority
        return best_id


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
