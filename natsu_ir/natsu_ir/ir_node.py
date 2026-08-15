#!/usr/bin/env python3
"""Natsurobo IR node -- ROS2 との出入り口だけを持つ薄いブリッジ。

入口 (ESP32 -> ROS):
  ir/state    std_msgs/String     確定した状態コード(16進)。ラッチ。
  ir/tairyo   std_msgs/Bool       大漁フラグ。ラッチ。
  ir/link_ok  std_msgs/Bool       ESP32リンク死活(HB監視)。ラッチ。

出口 (ROS -> ESP32):
  ir/led_cmd  std_msgs/ColorRGBA  状態表示LEDの色指示。ここに publish されたら
                                   "LED r g b\n" にして ESP32 へ素通しする。
                                   いつ/どの色にするかの判断はこのノードでは持たない。

ラッチは QoS(TRANSIENT_LOCAL)で実現。後から起動した購読側も最後の値を受け取れる。
"""

import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, ColorRGBA, String

SERIAL_PORT = "/dev/natsurobo_ir"   # udev symlink で固定
BAUD = 115200

CMD_TAIRYO = 0x45
DEBOUNCE_N = 3          # 連続同一コード数で確定
HB_TIMEOUT_S = 0.7      # これ以上HBが無ければリンク異常


def latched_qos():
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class IrNode(Node):
    def __init__(self):
        super().__init__("ir_node")
        self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)

        # --- 入口: publishers (ラッチ) ---
        self.pub_state = self.create_publisher(String, "ir/state", latched_qos())
        self.pub_tairyo = self.create_publisher(Bool, "ir/tairyo", latched_qos())
        self.pub_link = self.create_publisher(Bool, "ir/link_ok", latched_qos())

        # --- 出口: subscriber (LED表示コマンドを素通し) ---
        self.create_subscription(ColorRGBA, "ir/led_cmd", self.on_led_cmd, 10)

        self.last_hb = time.time()
        self.link_ok = None      # None から始めて初回で必ず publish
        self.buf = []

        self.create_timer(0.02, self.spin_serial)
        self.create_timer(0.20, self.watchdog)
        self.get_logger().info(f"IR node up on {SERIAL_PORT}")

    # ========== 入口: ESP32 -> ROS ==========
    def spin_serial(self):
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
        except serial.SerialException as e:
            self.get_logger().error(f"serial read failed: {e}")
            return
        if not line:
            return

        if line == "HB":
            self.last_hb = time.time()
            self._set_link(True)
            return

        if line == "IR LOST":
            # 状態はラッチ運用なので ir/state は保持したまま。信号途絶だけを扱う。
            self.buf.clear()
            return

        if line.startswith("IR"):
            self.last_hb = time.time()
            self._set_link(True)
            try:
                _, _addr, cmd, _rpt = line.split()
                self.debounce(int(cmd, 16))
            except ValueError:
                self.get_logger().warn(f"unparsable line: {line!r}")

    def debounce(self, cmd):
        self.buf.append(cmd)
        self.buf = self.buf[-DEBOUNCE_N:]
        if len(self.buf) == DEBOUNCE_N and len(set(self.buf)) == 1:
            # ラッチ: 確定値をそのまま publish(TRANSIENT_LOCAL で保持される)
            self.pub_state.publish(String(data=f"{cmd:02X}"))
            self.pub_tairyo.publish(Bool(data=(cmd == CMD_TAIRYO)))

    def watchdog(self):
        if time.time() - self.last_hb > HB_TIMEOUT_S:
            self._set_link(False)

    def _set_link(self, ok):
        if ok != self.link_ok:
            self.link_ok = ok
            self.pub_link.publish(Bool(data=ok))
            if not ok:
                self.get_logger().warn("IR link lost (HB timeout)")

    # ========== 出口: ROS -> ESP32 ==========
    def on_led_cmd(self, msg: ColorRGBA):
        r = max(0, min(255, int(msg.r * 255)))
        g = max(0, min(255, int(msg.g * 255)))
        b = max(0, min(255, int(msg.b * 255)))
        try:
            self.ser.write(f"LED {r} {g} {b}\n".encode())
        except serial.SerialException as e:
            self.get_logger().error(f"serial write failed: {e}")


def main():
    rclpy.init()
    node = IrNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
