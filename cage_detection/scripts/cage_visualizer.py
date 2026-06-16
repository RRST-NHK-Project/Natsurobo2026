#!/usr/bin/env python3
"""
cage_visualizer.py

/cage_detection/cages → visualization_msgs/MarkerArray に変換して RViz2 で表示する。

subscribe:
  /cage_detection/cages   [cage_detection/msg/CageArray]

publish:
  /cage_detection/markers [visualization_msgs/MarkerArray]
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from visualization_msgs.msg import Marker, MarkerArray
from cage_detection.msg import CageArray


# RGBA
_GREEN  = (0.1, 0.9, 0.1, 0.9)
_BLUE   = (0.2, 0.5, 1.0, 0.9)
_TARGET = (1.0, 0.8, 0.0, 1.0)  # 最優先ターゲットは金色

_SPHERE_R = 0.12   # カゴ球の半径 [m]
_LIFETIME = 0.5    # マーカーの有効時間 [s]


class CageVisualizer(Node):
    def __init__(self):
        super().__init__('cage_visualizer')

        self.sub_ = self.create_subscription(
            CageArray, '/cage_detection/cages', self._cb, 10)
        self.pub_ = self.create_publisher(
            MarkerArray, '/cage_detection/markers', 10)

        self.get_logger().info('cage_visualizer started')

    def _cb(self, msg: CageArray):
        array = MarkerArray()

        # 前フレームの残存マーカーを全消去
        clr = Marker()
        clr.header = msg.header
        clr.action = Marker.DELETEALL
        array.markers.append(clr)

        lifetime = Duration(seconds=_LIFETIME).to_msg()

        for i, cage in enumerate(msg.cages):
            is_target = (i == 0)

            if is_target:
                r, g, b, a = _TARGET
            elif cage.color == 0:
                r, g, b, a = _GREEN
            else:
                r, g, b, a = _BLUE

            # ── 球マーカー ──────────────────────────────────
            sphere = Marker()
            sphere.header   = msg.header
            sphere.ns       = 'cages'
            sphere.id       = i * 3
            sphere.type     = Marker.SPHERE
            sphere.action   = Marker.ADD
            sphere.lifetime = lifetime
            sphere.pose.position    = cage.position
            sphere.pose.orientation.w = 1.0
            d = _SPHERE_R * (1.6 if is_target else 1.0)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = d
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, a
            array.markers.append(sphere)

            # ── ターゲットには矢印を追加 ────────────────────
            if is_target:
                arrow = Marker()
                arrow.header   = msg.header
                arrow.ns       = 'cages'
                arrow.id       = i * 3 + 1
                arrow.type     = Marker.ARROW
                arrow.action   = Marker.ADD
                arrow.lifetime = lifetime
                arrow.pose.position.x = cage.position.x
                arrow.pose.position.y = cage.position.y - _SPHERE_R * 2.5
                arrow.pose.position.z = cage.position.z
                # 下向き矢印 (Y軸 180°回転)
                arrow.pose.orientation.x = 1.0
                arrow.pose.orientation.w = 0.0
                arrow.scale.x = 0.05   # shaft diameter
                arrow.scale.y = 0.10   # head diameter
                arrow.scale.z = _SPHERE_R * 2.0  # length
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, a
                array.markers.append(arrow)

            # ── テキストラベル ──────────────────────────────
            text = Marker()
            text.header   = msg.header
            text.ns       = 'cage_labels'
            text.id       = i * 3 + 2
            text.type     = Marker.TEXT_VIEW_FACING
            text.action   = Marker.ADD
            text.lifetime = lifetime
            text.pose.position.x = cage.position.x
            text.pose.position.y = cage.position.y + _SPHERE_R * 1.8
            text.pose.position.z = cage.position.z
            text.pose.orientation.w = 1.0
            text.scale.z  = 0.07  # 文字高さ [m]
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            color_name = 'GREEN' if cage.color == 0 else 'BLUE'
            star = '★ ' if is_target else ''
            text.text = f'{star}{color_name}\n{cage.distance:.2f} m  (prio {cage.priority})'
            array.markers.append(text)

        self.pub_.publish(array)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CageVisualizer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
