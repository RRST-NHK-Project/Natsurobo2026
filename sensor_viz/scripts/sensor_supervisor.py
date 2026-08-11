#!/usr/bin/env python3
"""
sensor_supervisor.py ─ LiDAR / IMU の自動検知つき起動監督ノード

やること:
  1. 起動時に LiDAR / IMU のシリアルポートを *データの中身* で自動検知
     (sensor_port_detect)。ttyUSB 番号のズレや udev の取り違えに強い。
  2. 検知したポートを引数に渡して sensor_test.launch.py を子プロセス
     (独立セッション) として起動。
  3. /scan と /imu の実データ流通を監視。一定時間データが来ない or ドライバ
     プロセスが死んだら、ポートを再検知して sensor 一式を再起動(自動復旧)。
  4. 状態を /diagnostics (diagnostic_msgs/DiagnosticArray) に publish。

背景: ldlidar ドライバは configure 失敗で finalized に落ちて再configure 不可、
かつ「active なのに data timeout」でも自力復帰しない。よって確実な復旧手段は
「正しいポートでプロセスを起動し直す」ことであり、それをこのノードが担う。

使い方:
    ros2 run sensor_viz sensor_supervisor
    ros2 run sensor_viz sensor_supervisor --ros-args -p use_imu:=true -p stale_sec:=4.0
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu, LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# 同じ lib/sensor_viz に入る検知モジュールを import できるようにする
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import sensor_port_detect as spd   # noqa: E402


class SensorSupervisor(Node):
    def __init__(self):
        super().__init__('sensor_supervisor')

        # ── パラメータ ─────────────────────────────
        self.use_imu    = self.declare_parameter('use_imu', True).value
        # RViz は「センサ再起動サイクル」とは切り離して1回だけ常駐起動する
        # (復旧のたびに RViz が閉じ/開きしないように)
        self.use_rviz   = self.declare_parameter('use_rviz', False).value
        self.rviz_config = self.declare_parameter('rviz_config', '').value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.imu_topic  = self.declare_parameter('imu_topic', '/imu').value
        # データが何秒来なければ「停止」とみなすか
        self.stale_sec  = float(self.declare_parameter('stale_sec', 4.0).value)
        # 起動直後、lidar の configure/spinup を待つ猶予
        self.grace_sec  = float(self.declare_parameter('startup_grace_sec', 15.0).value)
        # 再起動の連発を防ぐクールダウン
        self.cooldown_sec = float(self.declare_parameter('restart_cooldown_sec', 10.0).value)
        # デバイスが「検知できない(物理不在)」時、再起動しても直らないので待つ時間
        self.absent_backoff_sec = float(
            self.declare_parameter('absent_backoff_sec', 30.0).value)
        # フォールバックのポート(検知に失敗した時に使う udev シンボリックリンク)
        self.fallback_lidar = self.declare_parameter('fallback_lidar_port', '/dev/ldlidar').value
        self.fallback_imu   = self.declare_parameter('fallback_imu_port', '/dev/wt901').value

        # ── 状態 ───────────────────────────────────
        self._proc = None
        self._rviz_proc = None
        self._launch_mono = 0.0            # 直近 (再)起動時刻(monotonic)
        self._last_restart_mono = -1e9
        self._absent_until = 0.0           # この時刻まで再起動を控える(物理不在バックオフ)
        self._last_scan_mono = 0.0
        self._last_imu_mono = 0.0
        self._scan_count = 0
        self._imu_count = 0
        self._scan_hz = 0.0
        self._imu_hz = 0.0
        self._ports = {'lidar': None, 'imu': None}
        self._restart_total = 0

        # ── I/O ────────────────────────────────────
        self.create_subscription(LaserScan, self.scan_topic,
                                 self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic,
                                 self._imu_cb, qos_profile_sensor_data)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # 初回起動
        self._start_sensors(reason='initial start')
        if self.use_rviz:
            self._start_rviz()

        # 監視/診断ループ (1Hz)
        self.create_timer(1.0, self._tick)

    # ── コールバック ───────────────────────────────
    def _scan_cb(self, _msg):
        self._last_scan_mono = time.monotonic()
        self._scan_count += 1

    def _imu_cb(self, _msg):
        self._last_imu_mono = time.monotonic()
        self._imu_count += 1

    # ── 起動 / 停止 ────────────────────────────────
    def _detect_ports(self):
        self.get_logger().info('シリアルポートを自動検知中...')
        res = spd.detect_all()
        lidar = res['lidar'] or self.fallback_lidar
        imu   = res['imu'] or self.fallback_imu
        if not res['lidar']:
            self.get_logger().warn(
                f'LiDAR を検知できず。フォールバック {lidar} を使用 '
                '(未給電/未接続/回転停止の可能性)')
        else:
            self.get_logger().info(f'LiDAR 検知: {lidar}')
        if self.use_imu:
            if not res['imu']:
                self.get_logger().warn(
                    f'IMU を検知できず。フォールバック {imu} を使用')
            else:
                self.get_logger().info(f'IMU 検知: {imu}')
        self._ports = {'lidar': lidar, 'imu': imu}
        return lidar, imu

    def _start_sensors(self, reason=''):
        # 前世代/他プロセスの残骸がポートを掴んでいると検知・起動に失敗するため
        # 起動直前に必ず掃除してから検知する
        self._kill_sensor_orphans()
        time.sleep(0.5)
        lidar, imu = self._detect_ports()
        cmd = [
            'ros2', 'launch', 'sensor_viz', 'sensor_test.launch.py',
            # RViz は別管理なので、監督下の launch では必ず false
            'use_rviz:=false',
            f'use_imu:={"true" if self.use_imu else "false"}',
            f'lidar_port:={lidar}',
            f'imu_port:={imu}',
        ]
        self.get_logger().info(f'センサ起動 ({reason}): {" ".join(cmd)}')
        # 独立セッションにして、あとで killpg で丸ごと落とせるようにする
        self._proc = subprocess.Popen(cmd, start_new_session=True, env=os.environ.copy())
        now = time.monotonic()
        self._launch_mono = now
        self._last_restart_mono = now
        # 猶予期間中は stale 判定しないよう last_* を現在時刻で初期化
        self._last_scan_mono = now
        self._last_imu_mono = now

    def _stop_sensors(self):
        if not self._proc:
            return
        pid = self._proc.pid
        try:
            pgid = os.getpgid(pid)
        except ProcessLookupError:
            self._proc = None
            return
        self.get_logger().info(f'センサ停止中 (pgid={pgid})...')
        # まず SIGINT で ros2 launch に正常終了させる
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            pass
        # 最大 6 秒待って、残っていれば SIGKILL
        t0 = time.monotonic()
        while time.monotonic() - t0 < 6.0:
            if self._proc.poll() is not None:
                break
            time.sleep(0.2)
        if self._proc.poll() is None:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                self._proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                pass
        self._proc = None
        # ros2 launch を落としても composable container 等が孤児化して残り、
        # シリアルポートを掴んだまま次世代の起動を失敗させる(=復旧ループ)ことがある。
        # sensor_test.launch 由来のノードを node 名指定で確実に掃除する。
        self._kill_sensor_orphans()
        time.sleep(1.5)   # ポート解放を待つ

    # sensor_test.launch が起動するノード(=このスタック専用の名前)
    _SENSOR_NODE_NAMES = (
        'ldlidar_container', 'wt901c_publisher', 'wall_detection_node',
        'scan_relay', 'tf_lidar', 'tf_imu', 'tf_odom',
        'sensor_visualizer', 'lidar_lifecycle_manager',
    )

    def _kill_sensor_orphans(self):
        for name in self._SENSOR_NODE_NAMES:
            # __node:=<name> は sensor_test.launch のノードだけに一致するので
            # 無関係な TF publisher 等を巻き込まない
            subprocess.run(['pkill', '-9', '-f', f'__node:={name}'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _start_rviz(self):
        cfg = self.rviz_config
        if not cfg:
            try:
                from ament_index_python.packages import get_package_share_directory
                cfg = os.path.join(get_package_share_directory('sensor_viz'),
                                   'rviz', 'sensor_test.rviz')
            except Exception:
                cfg = ''
        cmd = ['rviz2'] + (['-d', cfg] if cfg else [])
        self.get_logger().info(f'RViz起動: {" ".join(cmd)}')
        self._rviz_proc = subprocess.Popen(cmd, start_new_session=True, env=os.environ.copy())

    def _stop_rviz(self):
        if not self._rviz_proc:
            return
        try:
            os.killpg(os.getpgid(self._rviz_proc.pid), signal.SIGINT)
            self._rviz_proc.wait(timeout=4.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(self._rviz_proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        self._rviz_proc = None

    def _restart(self, reason):
        self._restart_total += 1
        self.get_logger().error(f'自動復旧 #{self._restart_total}: {reason} → 再起動')
        self._stop_sensors()
        self._start_sensors(reason=reason)

    # ── 監視ループ ─────────────────────────────────
    def _tick(self):
        now = time.monotonic()
        # 実レート(直近1秒)
        self._scan_hz, self._scan_count = float(self._scan_count), 0
        self._imu_hz, self._imu_count = float(self._imu_count), 0

        scan_age = now - self._last_scan_mono
        imu_age = now - self._last_imu_mono
        in_grace = (now - self._launch_mono) < self.grace_sec
        cooled = (now - self._last_restart_mono) > self.cooldown_sec

        scan_ok = scan_age < self.stale_sec
        imu_ok = (imu_age < self.stale_sec) if self.use_imu else True

        self._publish_diag(scan_ok, imu_ok, scan_age, imu_age, in_grace)

        # プロセスが死んでいたら即再起動(クールダウンは尊重)
        if self._proc is not None and self._proc.poll() is not None:
            if cooled:
                self._restart('ドライバプロセスが終了')
            return

        # 猶予中/クールダウン中/物理不在バックオフ中は静観
        if in_grace or not cooled or now < self._absent_until:
            return

        if not scan_ok or not imu_ok:
            self._handle_stale(scan_ok, imu_ok, scan_age, imu_age)

    def _handle_stale(self, scan_ok, imu_ok, scan_age, imu_age):
        """データ停止時の対応。

        再起動する前に「そのデバイスが今バス上に居るか」を検知して判断する。
          - 検知できる  → ポート番号ズレ/掴み直しで直る見込み → 再起動して復旧
          - 検知できない → 物理的に外れている。再起動しても直らず、健全な方の
                           センサまで巻き込むだけなので再起動せず待機(バックオフ)
        """
        found = spd.detect_all()
        recoverable = False
        reasons = []
        if not scan_ok:
            if found['lidar']:
                recoverable = True
                reasons.append(f'/scan停止→LiDARを{found["lidar"]}に検出')
            else:
                reasons.append(f'/scan停止({scan_age:.0f}s)&LiDAR未検出')
        if self.use_imu and not imu_ok:
            if found['imu']:
                recoverable = True
                reasons.append(f'/imu停止→IMUを{found["imu"]}に検出')
            else:
                reasons.append(f'/imu停止({imu_age:.0f}s)&IMU未検出')
        msg = ' / '.join(reasons)

        if recoverable:
            self._restart(msg)
        else:
            # 物理不在: 再起動しても直らない。健全な方は動かしたまま待機する。
            self.get_logger().error(
                f'{msg} → デバイス未検出のため再起動を抑止({self.absent_backoff_sec:.0f}s待機)。'
                'ハード/配線/給電/USB接続を確認してください')
            self._absent_until = time.monotonic() + self.absent_backoff_sec

    def _publish_diag(self, scan_ok, imu_ok, scan_age, imu_age, in_grace):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        def mk(name, ok, age, hz, port):
            st = DiagnosticStatus()
            st.name = name
            st.hardware_id = port or 'unknown'
            if in_grace:
                st.level = DiagnosticStatus.WARN
                st.message = '起動中(猶予期間)'
            elif ok:
                st.level = DiagnosticStatus.OK
                st.message = f'OK {hz:.1f} Hz'
            else:
                st.level = DiagnosticStatus.ERROR
                st.message = f'データ停止 {age:.1f}s'
            st.values = [
                KeyValue(key='port', value=str(port)),
                KeyValue(key='rate_hz', value=f'{hz:.1f}'),
                KeyValue(key='age_sec', value=f'{age:.1f}'),
            ]
            return st

        arr.status.append(
            mk('sensor_supervisor: LiDAR (/scan)', scan_ok, scan_age,
               self._scan_hz, self._ports.get('lidar')))
        if self.use_imu:
            arr.status.append(
                mk('sensor_supervisor: IMU (/imu)', imu_ok, imu_age,
                   self._imu_hz, self._ports.get('imu')))
        self._diag_pub.publish(arr)

    def destroy_node(self):
        self._stop_rviz()
        self._stop_sensors()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
