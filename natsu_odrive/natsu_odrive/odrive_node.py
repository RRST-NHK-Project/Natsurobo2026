#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odrive_node.py
ODrive v3.6 (firmware 0.5.6) を USB 経由で制御する ROS2 ノード。
DualMD: 1台の ODrive で axis0 / axis1 の 2 モータを制御する。

購読 (subscribe):
    <ns>/axis0/velocity_cmd  [std_msgs/Float64]  axis0 目標速度 [turn/s]
    <ns>/axis1/velocity_cmd  [std_msgs/Float64]  axis1 目標速度 [turn/s]

配信 (publish):
    <ns>/axis0/velocity, <ns>/axis0/position
    <ns>/axis1/velocity, <ns>/axis1/position
    <ns>/bus_voltage

2台構成(4モータ)は、このノードを2つ起動し、
それぞれ serial_number と namespace(例: odrv_a / odrv_b)を分けて使う。
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import odrive
from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
    INPUT_MODE_PASSTHROUGH,
)
from odrive.utils import dump_errors


class OdriveNode(Node):
    def __init__(self):
        super().__init__("odrive_node")

        self.declare_parameter("serial_number", "")   # 2台使う時は必ず指定
        self.declare_parameter("vel_limit", 20.0)      # [turn/s]
        self.declare_parameter("current_limit", 20.0)  # [A]
        self.declare_parameter("vel_ramp_rate", 10.0)  # [turn/s^2] ramp時の傾き
        # 入力モード: "ramp"（滑らかに追従）/ "passthrough"（即座に目標を与える=キレる）
        self.declare_parameter("input_mode", "ramp")
        # 速度PIDゲイン: 0.0 のときはボード保存値をそのまま使う（上書きしない）
        self.declare_parameter("vel_gain", 0.0)
        self.declare_parameter("vel_integrator_gain", 0.0)
        self.declare_parameter("cmd_timeout", 0.5)     # [s] 指令途絶で停止
        self.declare_parameter("publish_rate", 20.0)   # [Hz]
        self.declare_parameter("use_axis0", True)
        self.declare_parameter("use_axis1", True)

        self.serial_number = self.get_parameter("serial_number").value
        self.vel_limit = float(self.get_parameter("vel_limit").value)
        self.current_limit = float(self.get_parameter("current_limit").value)
        self.vel_ramp_rate = float(self.get_parameter("vel_ramp_rate").value)
        self.input_mode = str(self.get_parameter("input_mode").value).lower()
        self.vel_gain = float(self.get_parameter("vel_gain").value)
        self.vel_integrator_gain = float(self.get_parameter("vel_integrator_gain").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.use = {
            0: bool(self.get_parameter("use_axis0").value),
            1: bool(self.get_parameter("use_axis1").value),
        }

        self._connect()

        self.axes = {}
        if self.use[0]:
            self.axes[0] = self.odrv.axis0
        if self.use[1]:
            self.axes[1] = self.odrv.axis1

        for idx, ax in self.axes.items():
            self._prepare_axis(idx, ax)

        # ROS I/F を軸ごとに作る
        self._subs = {}
        self._pub_vel = {}
        self._pub_pos = {}
        self._last_cmd = {}
        for idx in self.axes:
            self._subs[idx] = self.create_subscription(
                Float64, f"axis{idx}/velocity_cmd",
                self._make_cmd_cb(idx), 10)
            self._pub_vel[idx] = self.create_publisher(Float64, f"axis{idx}/velocity", 10)
            self._pub_pos[idx] = self.create_publisher(Float64, f"axis{idx}/position", 10)
            self._last_cmd[idx] = self.get_clock().now()

        self.pub_vbus = self.create_publisher(Float64, "bus_voltage", 10)
        self.create_timer(1.0 / publish_rate, self._on_timer)
        self.get_logger().info(
            f"odrive_node ready. axes={list(self.axes.keys())}, "
            f"serial={self.serial_number or 'auto'}")

    # -------------------------------------------------------------------------
    def _connect(self):
        if self.serial_number:
            self.get_logger().info(f"Searching ODrive serial={self.serial_number} ...")
            self.odrv = odrive.find_any(serial_number=self.serial_number)
        else:
            self.get_logger().info("Searching any ODrive over USB ...")
            self.odrv = odrive.find_any()
        self.get_logger().info(
            f"Connected. fw={self.odrv.fw_version_major}.{self.odrv.fw_version_minor}"
            f".{self.odrv.fw_version_revision}, Vbus={self.odrv.vbus_voltage:.1f}V")
        try:
            self.odrv.clear_errors()
        except Exception:
            pass

    def _prepare_axis(self, idx, ax):
        ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        # 入力モードの選択
        if self.input_mode == "passthrough":
            # ランプ無し。押した瞬間に目標速度を与える（一番キレる）
            ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        else:
            # 目標速度を vel_ramp_rate の傾きで滑らかに動かす
            ax.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            ax.controller.config.vel_ramp_rate = self.vel_ramp_rate

        ax.controller.config.vel_limit = self.vel_limit
        ax.motor.config.current_lim = self.current_limit

        # 速度PIDゲイン（0より大きいときだけ上書き。追従の“食いつき”を調整）
        if self.vel_gain > 0.0:
            ax.controller.config.vel_gain = self.vel_gain
        if self.vel_integrator_gain > 0.0:
            ax.controller.config.vel_integrator_gain = self.vel_integrator_gain

        # 起動時インデックス探索の完了（エンコーダ準備）を待つ
        t0 = time.time()
        while not ax.encoder.is_ready:
            if time.time() - t0 > 10.0:
                self.get_logger().error(
                    f"axis{idx}: エンコーダが ready になりません。"
                    "校正(configure_odrive.py)とZ相配線を確認してください。")
                dump_errors(self.odrv)
                raise RuntimeError(f"axis{idx} encoder not ready")
            time.sleep(0.1)

        ax.controller.input_vel = 0.0
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.2)
        if ax.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.get_logger().error(f"axis{idx}: CLOSED_LOOP 失敗。errors:")
            dump_errors(self.odrv)
            raise RuntimeError(
                f"axis{idx} がクローズドループに入りません。configure_odrive.py で校正済みか確認。")
        self.get_logger().info(f"axis{idx}: CLOSED_LOOP_CONTROL OK")

    # -------------------------------------------------------------------------
    def _make_cmd_cb(self, idx):
        def cb(msg: Float64):
            target = max(-self.vel_limit, min(self.vel_limit, float(msg.data)))
            self.axes[idx].controller.input_vel = target
            self._last_cmd[idx] = self.get_clock().now()
        return cb

    def _on_timer(self):
        now = self.get_clock().now()
        for idx, ax in self.axes.items():
            dt = (now - self._last_cmd[idx]).nanoseconds * 1e-9
            if dt > self.cmd_timeout:
                ax.controller.input_vel = 0.0
            try:
                self._pub_vel[idx].publish(Float64(data=float(ax.encoder.vel_estimate)))
                self._pub_pos[idx].publish(Float64(data=float(ax.encoder.pos_estimate)))
            except Exception as e:
                self.get_logger().warn(f"axis{idx} read failed: {e}")
        try:
            self.pub_vbus.publish(Float64(data=float(self.odrv.vbus_voltage)))
        except Exception:
            pass

    def shutdown(self):
        for idx, ax in self.axes.items():
            try:
                ax.controller.input_vel = 0.0
                ax.requested_state = AXIS_STATE_IDLE
            except Exception:
                pass
        self.get_logger().info("All axes set to IDLE.")


def main(args=None):
    rclpy.init(args=args)
    node = OdriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
