#!/usr/bin/env python3
"""
sensor_port_detect.py ─ LiDAR / IMU のシリアルポート自動検知

「/dev/ttyUSB* の番号は抜き差しで変わる」「udev シンボリックリンクが別デバイスを
指すことがある」問題に対し、*実際に流れているデータの中身* で各デバイスを見分ける。

判定ロジック（実機で検証済み）:
  - LD19 LiDAR : Silicon Labs CP210x(VID 10c4)。230400bps で連続大量配信し、
                 パケット先頭 0x54 0x2C が周期的に出る。モータ基板等は無言/低レート。
  - WT901C IMU : CH340(VID 1a86)。WIT プロトコルのフレーム先頭 0x55 + 型(0x50-0x5A)。

ライブラリとしても CLI としても使える:
    ros2 run sensor_viz detect_ports          # 検知結果を表示
    python3 sensor_port_detect.py --json       # JSON で出力
"""

import glob
import os
import sys
import termios
import time

# VID(ベンダID) → デバイス種別のヒント（/dev/serial/by-id 名で判定）
_LIDAR_NAME_HINTS = ("CP210", "Silicon_Labs")   # CP2102 / CP2102N
_IMU_NAME_HINTS   = ("1a86", "CH340", "QinHeng")

_LD19_BAUD = termios.B230400
_WIT_BAUD  = termios.B9600     # sensor_test.launch.py の wt901c baud_rate と一致させる


def list_candidates():
    """(実体パス, by-id名) のリストを返す。by-id が無ければ ttyUSB* を素で列挙。"""
    cands = []
    byid = sorted(glob.glob("/dev/serial/by-id/*"))
    if byid:
        for link in byid:
            cands.append((os.path.realpath(link), os.path.basename(link)))
    else:
        for dev in sorted(glob.glob("/dev/ttyUSB*")):
            cands.append((dev, os.path.basename(dev)))
    return cands


def _name_matches(name, hints):
    return any(h.lower() in name.lower() for h in hints)


def probe(port, baud_const, duration=1.2):
    """port を baud で開いて duration 秒読み、生バイト列を返す(read-only)。

    キャリア検出待ちでブロックしないよう O_NONBLOCK + CLOCAL で開く。
    ポートが掴めない/読めない時は空 bytes。
    """
    try:
        fd = os.open(port, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    except OSError:
        return b""
    try:
        attr = termios.tcgetattr(fd)
        attr[0] = 0                                   # iflag
        attr[1] = 0                                   # oflag
        attr[3] = 0                                   # lflag (raw)
        attr[2] = (attr[2] | termios.CLOCAL | termios.CREAD) & ~termios.CSIZE
        attr[2] |= termios.CS8
        attr[4] = baud_const                          # ispeed
        attr[5] = baud_const                          # ospeed
        termios.tcsetattr(fd, termios.TCSANOW, attr)
        try:
            termios.tcflush(fd, termios.TCIFLUSH)
        except termios.error:
            pass
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < duration:
            try:
                d = os.read(fd, 4096)
                if d:
                    buf.extend(d)
            except BlockingIOError:
                time.sleep(0.01)
            except OSError:
                break
        return bytes(buf)
    finally:
        try:
            os.close(fd)
        except OSError:
            pass


def _count_ld19_frames(data):
    """LD19 パケット先頭 0x54 0x2C の出現回数。"""
    n = 0
    for i in range(len(data) - 1):
        if data[i] == 0x54 and data[i + 1] == 0x2C:
            n += 1
    return n


def _count_wit_frames(data):
    """WIT(WT901) フレーム先頭 0x55 + 型(0x50..0x5A) の出現回数。"""
    n = 0
    for i in range(len(data) - 1):
        if data[i] == 0x55 and 0x50 <= data[i + 1] <= 0x5A:
            n += 1
    return n


def detect_lidar_port(duration=1.2, min_bytes=1500, min_frames=3, skip=()):
    """LD19 とみられるポートを返す。見つからなければ None。

    複数候補が流れていれば LD19 フレーム数が最多のものを採用。
    """
    best, best_score = None, 0
    for real, name in list_candidates():
        if real in skip:
            continue
        if not _name_matches(name, _LIDAR_NAME_HINTS):
            continue
        data = probe(real, _LD19_BAUD, duration)
        frames = _count_ld19_frames(data)
        if len(data) >= min_bytes and frames >= min_frames and frames > best_score:
            best, best_score = real, frames
    return best


def detect_imu_port(duration=1.5, min_frames=2, skip=()):
    """WT901C とみられるポートを返す。見つからなければ None。

    CH340 候補が1つだけならデータ検証に失敗しても最有力として返す(保険)。
    """
    ch340 = [(r, n) for r, n in list_candidates()
             if _name_matches(n, _IMU_NAME_HINTS) and r not in skip]
    verified = None
    for real, _name in ch340:
        data = probe(real, _WIT_BAUD, duration)
        if _count_wit_frames(data) >= min_frames:
            verified = real
            break
    if verified:
        return verified
    if len(ch340) == 1:            # CH340 が一意なら保険で採用
        return ch340[0][0]
    return None


def detect_all(skip=()):
    lidar = detect_lidar_port(skip=skip)
    imu = detect_imu_port(skip=skip)
    return {"lidar": lidar, "imu": imu}


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    as_json = "--json" in argv
    res = detect_all()
    if as_json:
        import json
        print(json.dumps(res))
    else:
        print("=== sensor port detection ===")
        print(f"  LiDAR (LD19)  : {res['lidar'] or '見つからず (未給電/未接続の可能性)'}")
        print(f"  IMU   (WT901C): {res['imu'] or '見つからず (未給電/未接続の可能性)'}")
    # 両方見つかれば 0、片方でも欠ければ 1
    return 0 if (res["lidar"] and res["imu"]) else 1


if __name__ == "__main__":
    sys.exit(main())
