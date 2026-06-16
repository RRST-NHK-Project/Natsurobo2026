#!/usr/bin/env python3
"""
sensor_visualizer.py  ─ センサテスト用 RViz2 可視化ノード

subscribe:
  /wall_detection/angle    [std_msgs/Float64]  壁偏角 [rad]
  /wall_detection/distance [std_msgs/Float64]  壁距離 [m]
  /imu/data                [sensor_msgs/Imu]   WT901C

publish:
  /sensor_viz/markers      [visualization_msgs/MarkerArray]
    ns='wall'       : 壁ライン + 法線アロー (ldlidar_link frame)
    ns='wall_label' : 角度/距離テキスト    (ldlidar_link frame)
    ns='imu'        : Roll/Pitch/Yaw テキスト + 重力アロー (imu_link / base_link frame)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


LIDAR_FRAME    = 'ldlidar_link'
BASE_FRAME     = 'base_link'
IMU_FRAME      = 'imu_link'
WALL_HALF_LEN  = 2.0   # 壁ラインの描画半長 [m]
MARKER_LIFE_S  = 0.3   # マーカー有効時間 [s]


class SensorVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_visualizer')

        self._wall_angle = 0.0
        self._wall_dist  = 0.0
        self._roll = self._pitch = self._yaw = 0.0
        self._ax = self._ay = self._az = 0.0
        self._imu_valid = False
        self._wall_valid = False

        self.create_subscription(Float64, '/wall_detection/angle',
            self._angle_cb, 10)
        self.create_subscription(Float64, '/wall_detection/distance',
            self._dist_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)

        self._pub = self.create_publisher(MarkerArray, '/sensor_viz/markers', 10)
        self.create_timer(0.1, self._publish)

        self.get_logger().info('sensor_visualizer started')

    def _angle_cb(self, msg: Float64):
        self._wall_angle = msg.data
        self._wall_valid = True

    def _dist_cb(self, msg: Float64):
        self._wall_dist = msg.data

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        # クォータニオン → ZYX オイラー角
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x**2 + q.y**2)
        self._roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self._pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self._yaw = math.atan2(siny, cosy)

        self._ax = msg.linear_acceleration.x
        self._ay = msg.linear_acceleration.y
        self._az = msg.linear_acceleration.z
        self._imu_valid = True

    @staticmethod
    def _tilt_rgba(tilt_deg: float):
        """傾き [deg] → RGBA (緑/黄/赤)"""
        if tilt_deg < 2.0:
            return (0.1, 0.9, 0.1, 1.0)
        elif tilt_deg < 5.0:
            return (1.0, 0.8, 0.0, 1.0)
        else:
            return (1.0, 0.2, 0.1, 1.0)

    def _base_marker(self, ns, mid, mtype, frame, stamp, lifetime):
        m = Marker()
        m.header.stamp    = stamp
        m.header.frame_id = frame
        m.ns       = ns
        m.id       = mid
        m.type     = mtype
        m.action   = Marker.ADD
        m.lifetime = lifetime
        m.pose.orientation.w = 1.0
        return m

    def _publish(self):
        stamp    = self.get_clock().now().to_msg()
        lifetime = Duration(seconds=MARKER_LIFE_S).to_msg()
        array    = MarkerArray()

        # ── 壁ライン（ldlidar_link フレーム） ────────────
        if self._wall_valid:
            a  = self._wall_angle
            d  = self._wall_dist
            cx = math.cos(a) * d   # 壁最近傍点 X
            cy = math.sin(a) * d   # 壁最近傍点 Y
            tx = -math.sin(a)      # 壁接線方向 X
            ty =  math.cos(a)      # 壁接線方向 Y

            # 壁ライン
            p1, p2 = Point(), Point()
            p1.x = cx + WALL_HALF_LEN * tx;  p1.y = cy + WALL_HALF_LEN * ty
            p2.x = cx - WALL_HALF_LEN * tx;  p2.y = cy - WALL_HALF_LEN * ty

            wl = self._base_marker('wall', 0, Marker.LINE_STRIP, LIDAR_FRAME, stamp, lifetime)
            wl.scale.x = 0.04
            wl.color.r = 0.2; wl.color.g = 0.8; wl.color.b = 1.0; wl.color.a = 1.0
            wl.points  = [p1, p2]
            array.markers.append(wl)

            # 法線アロー（origin → 壁最近傍点）
            tip = Point(); tip.x = cx; tip.y = cy
            na = self._base_marker('wall', 1, Marker.ARROW, LIDAR_FRAME, stamp, lifetime)
            na.scale.x = 0.025   # shaft 径
            na.scale.y = 0.06    # head  径
            na.scale.z = 0.0
            na.points  = [Point(), tip]
            na.color.r = 1.0; na.color.g = 1.0; na.color.b = 0.0; na.color.a = 1.0
            array.markers.append(na)

            # テキスト（距離・角度）
            wt = self._base_marker('wall_label', 2, Marker.TEXT_VIEW_FACING, LIDAR_FRAME, stamp, lifetime)
            wt.pose.position.x = cx
            wt.pose.position.y = cy + 0.18
            wt.scale.z = 0.13
            wt.color.r = wt.color.g = wt.color.b = wt.color.a = 1.0
            wt.text = f'dist : {d:.3f} m\nangle: {math.degrees(a):+.1f} deg'
            array.markers.append(wt)

        # ── IMU 傾き（base_link / imu_link フレーム） ─────
        if self._imu_valid:
            roll_d  = math.degrees(self._roll)
            pitch_d = math.degrees(self._pitch)
            yaw_d   = math.degrees(self._yaw)
            tilt_d  = math.hypot(roll_d, pitch_d)
            r, g, b, alpha = self._tilt_rgba(tilt_d)

            # Roll/Pitch/Yaw テキスト（base_link 上方）
            it = self._base_marker('imu', 10, Marker.TEXT_VIEW_FACING, BASE_FRAME, stamp, lifetime)
            it.pose.position.z = 0.45
            it.scale.z = 0.10
            it.color.r = r; it.color.g = g; it.color.b = b; it.color.a = alpha
            it.text = (
                f'Roll : {roll_d:+6.1f} deg\n'
                f'Pitch: {pitch_d:+6.1f} deg\n'
                f'Yaw  : {yaw_d:+6.1f} deg'
            )
            array.markers.append(it)

            # 重力方向アロー（imu_link フレーム）
            # WT901C の加速度 = 実加速度 + 重力。静止時は重力方向を指す
            norm = math.sqrt(self._ax**2 + self._ay**2 + self._az**2)
            if norm > 0.5:
                scale = 0.5 / norm
                gp = Point()
                gp.x = self._ax * scale
                gp.y = self._ay * scale
                gp.z = self._az * scale

                ga = self._base_marker('imu', 11, Marker.ARROW, IMU_FRAME, stamp, lifetime)
                ga.scale.x = 0.03; ga.scale.y = 0.06; ga.scale.z = 0.0
                ga.points  = [Point(), gp]
                ga.color.r = r; ga.color.g = g; ga.color.b = b; ga.color.a = 1.0
                array.markers.append(ga)

        self._pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SensorVisualizer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
