#!/usr/bin/env python3
"""
joy_mux: 実機PS4コントローラ(/joy_ps4)とGUIの仮想コントローラ(/joy_gui)を
1本の /joy に合成して配る。

なぜ必要か:
  mc_2026 / zakiomni / 101mm / charge はどれも "joy" を購読していて、
  mc_2026 は `if (L1 && !last_L1)` 方式のエッジ検出でシリンダやモードをトグルする。
  ここに2つのpublisherが同じトピックへ流し込むと、PS4のフレームとGUIのフレームが
  交互に届き、「PS4で押しっぱなし」の間にGUIの全ボタン0フレームが挟まって
  離した→押した の誤エッジが出る(＝シリンダやSHAREが勝手に連打される)。
  なので一旦ここで1つの一貫した状態に畳んでから、固定レートで配り直す。

合成ルール:
  ボタン : 両者のOR (どちらで押しても効く)
  トリガ軸(L2/R2, 未押下=+1.0): より深く押されている方 = min
  それ以外の軸: PS4が中立でなければPS4、中立ならGUI
  gui_blocked_axes に入れた軸はGUIの値を一切見ない(既定でLS/RS/R2=走行系)。
  → 足回りは実機PS4限定。GUIから機体が走り出すことはない。

安全:
  各入力にウォッチドッグ。timeout秒 受信が途切れた入力は中立値として扱う。
  GUIはWi-Fi経由なので、これが無いと通信断でボタンが押しっぱなしになる。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# 中立値が +1.0 の軸(L2/R2)。未押下=+1.0, 全押し=-1.0。
DEFAULT_TRIGGER_AXES = [2, 5]

# GUIからの入力を無視する軸。走行はPS4限定にするため。
#   0,1 = LS(移動), 3,4 = RS(旋回), 5 = R2(zakiomni のスロットル)
# L2(2)は 101mm の昇降なので通してある。
DEFAULT_GUI_BLOCKED_AXES = [0, 1, 3, 4, 5]

EPS = 1e-3


class JoyMux(Node):
    def __init__(self):
        super().__init__("joy_mux")

        self.ps4_topic = self.declare_parameter("ps4_topic", "joy_ps4").value
        self.gui_topic = self.declare_parameter("gui_topic", "joy_gui").value
        self.out_topic = self.declare_parameter("out_topic", "joy").value
        rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)
        self.ps4_timeout = float(self.declare_parameter("ps4_timeout", 0.5).value)
        # GUIは10Hz publish なので、PS4より少し長めに見る
        self.gui_timeout = float(self.declare_parameter("gui_timeout", 0.6).value)
        self.min_axes = int(self.declare_parameter("min_axes", 8).value)
        self.min_buttons = int(self.declare_parameter("min_buttons", 14).value)
        self.trigger_axes = set(
            self.declare_parameter("trigger_axes", DEFAULT_TRIGGER_AXES).value)
        self.gui_blocked_axes = set(
            self.declare_parameter("gui_blocked_axes", DEFAULT_GUI_BLOCKED_AXES).value)
        # 既定では全ボタンをGUIに開放する。塞ぎたいものが出たらここに番号を足す。
        self.gui_blocked_buttons = set(
            self.declare_parameter("gui_blocked_buttons", []).value)

        self.ps4_msg = None
        self.ps4_time = None
        self.gui_msg = None
        self.gui_time = None
        self.last_status = None

        self.create_subscription(Joy, self.ps4_topic, self._on_ps4, 10)
        self.create_subscription(Joy, self.gui_topic, self._on_gui, 10)
        self.pub = self.create_publisher(Joy, self.out_topic, 10)

        # どちらの入力が生きているかをGUI/操縦者へ通知する。
        # 遅れて繋いだGUIにも最後の状態が届くようラッチしておく。
        self.status_pub = self.create_publisher(
            String, "/joy_mux/status",
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"joy_mux: {self.ps4_topic}(PS4) + {self.gui_topic}(GUI) -> "
            f"{self.out_topic} @ {rate_hz:.0f}Hz")
        self.get_logger().info(
            f"GUIから無視する軸(走行系): {sorted(self.gui_blocked_axes)}")

    def _on_ps4(self, msg):
        self.ps4_msg = msg
        self.ps4_time = self.get_clock().now()

    def _on_gui(self, msg):
        self.gui_msg = msg
        self.gui_time = self.get_clock().now()

    def _alive(self, stamp, timeout):
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9 < timeout

    def _neutral_axis(self, i):
        return 1.0 if i in self.trigger_axes else 0.0

    def _tick(self):
        ps4_ok = self._alive(self.ps4_time, self.ps4_timeout)
        gui_ok = self._alive(self.gui_time, self.gui_timeout)

        ps4_axes = list(self.ps4_msg.axes) if (ps4_ok and self.ps4_msg) else []
        ps4_btns = list(self.ps4_msg.buttons) if (ps4_ok and self.ps4_msg) else []
        gui_axes = list(self.gui_msg.axes) if (gui_ok and self.gui_msg) else []
        gui_btns = list(self.gui_msg.buttons) if (gui_ok and self.gui_msg) else []

        n_axes = max(self.min_axes, len(ps4_axes), len(gui_axes))
        n_btns = max(self.min_buttons, len(ps4_btns), len(gui_btns))

        out = Joy()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "joy_mux"

        axes = []
        for i in range(n_axes):
            neutral = self._neutral_axis(i)
            p = ps4_axes[i] if i < len(ps4_axes) else neutral
            if i in self.gui_blocked_axes:
                g = neutral
            else:
                g = gui_axes[i] if i < len(gui_axes) else neutral

            if i in self.trigger_axes:
                # 未押下=+1.0 なので、深く押した方(小さい方)を採用
                axes.append(min(p, g))
            elif abs(p) > EPS:
                axes.append(p)
            else:
                axes.append(g)
        out.axes = axes

        buttons = []
        for i in range(n_btns):
            p = ps4_btns[i] if i < len(ps4_btns) else 0
            g = 0 if i in self.gui_blocked_buttons else (
                gui_btns[i] if i < len(gui_btns) else 0)
            buttons.append(1 if (p or g) else 0)
        out.buttons = buttons

        self.pub.publish(out)
        self._publish_status(ps4_ok, gui_ok)

    def _publish_status(self, ps4_ok, gui_ok):
        status = f"ps4={'alive' if ps4_ok else 'lost'} gui={'alive' if gui_ok else 'lost'}"
        if status == self.last_status:
            return
        self.last_status = status
        self.status_pub.publish(String(data=status))
        # 片方が落ちたのは操縦中に気づきたいので警告で出す
        if not ps4_ok or not gui_ok:
            self.get_logger().warn(f"joy_mux: {status}")
        else:
            self.get_logger().info(f"joy_mux: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = JoyMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
