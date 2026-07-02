#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
configure_odrive.py
ODrive v3.6 (firmware 0.5.6) を NEEBRC D4250-600KV 用に設定・校正する単体スクリプト。

DualMD 構成: ODrive 1台で axis0 / axis1 の 2 モータ。ODrive 2台で計 4 モータ。
このスクリプトは「1台ぶん（2軸）」を設定する。2台目は SERIAL_NUMBER を変えてもう一度実行する。

使い方:
    source ~/odrive_venv/bin/activate
    python3 configure_odrive.py                 # 最初に見つかった1台
    python3 configure_odrive.py 208F31834D4D    # シリアル番号で特定の1台を指定

★ FEEDBACK_MODE を必ず自分の構成に合わせること（下記参照）。
"""

import sys
import time

import odrive
from odrive.enums import (
    MOTOR_TYPE_HIGH_CURRENT,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
)
from odrive.utils import dump_errors

# ============================ フィードバック方式の選択 ============================
# "encoder"    : 各モータにエンコーダ(ABI等)を増設した場合。低速からトルクが出る。推奨。
# "sensorless" : エンコーダ無し。ある程度の回転数以上でのみ安定。停止/微速は苦手。
FEEDBACK_MODE = "encoder"   # ← "encoder" か "sensorless" を選ぶ

# ============================ D4250-600KV のパラメータ ============================
MOTOR = {
    "kv": 600.0,
    "pole_pairs": 6,               # 確認済み: 磁石12個 = 6極対
    "motor_type": MOTOR_TYPE_HIGH_CURRENT,
    "calibration_current": 10.0,   # 校正時電流 [A]（大きめモータなので10A程度）
    "current_lim": 20.0,           # 運転時電流上限 [A]（まず控えめに。必要なら上げる）
    "resistance_calib_max_voltage": 2.0,
}

# --- エンコーダ設定（FEEDBACK_MODE="encoder" のときだけ使う）---
ENCODER = {
    # ABI インクリメンタルの CPR = 1回転パルス数 × 4。
    #   推奨: CUI AMT102-V（Z相=インデックス付き, 既定 2048P/R → 8192 CPR）
    "cpr": 8192,                   # ★実際のエンコーダに合わせる
    # インデックス(Z相)を配線したか。True 推奨（起動ごとの再校正が要らなくなる）。
    "use_index": True,             # ★Z相を配線しないなら False
}

# --- 電源系（24V LiPo = 6S 想定, 満充電 25.2V）---
POWER = {
    "dc_bus_overvoltage_trip_level": 26.0,   # 満充電 25.2V より少し上
    "dc_bus_undervoltage_trip_level": 18.0,  # 6S の下限目安 (3.0V/cell)
    "dc_max_positive_current": 30.0,   # 電源から引く最大電流 [A]
    "dc_max_negative_current": -3.0,   # 回生で戻す最大 [A]（ブレーキ抵抗無しなら小さく）
    "enable_brake_resistor": True,     # ブレーキ抵抗を付けているなら True
    "brake_resistance": 2.0,           # ★実際の抵抗値 [Ω]。無いなら enable_brake_resistor=False
}

# --- 運転パラメータ ---
RUN = {
    "vel_limit": 20.0,       # 速度上限 [turn/s]
    "vel_ramp_rate": 10.0,   # 速度ランプ [turn/s^2]
}
# ==============================================================================


def configure_axis(odrv, ax, name):
    """1軸ぶんの設定・校正を行う。"""
    print(f"\n===== {name} を設定 =====")

    # モータ
    ax.motor.config.motor_type = MOTOR["motor_type"]
    ax.motor.config.pole_pairs = MOTOR["pole_pairs"]
    ax.motor.config.torque_constant = 8.27 / MOTOR["kv"]
    ax.motor.config.calibration_current = MOTOR["calibration_current"]
    ax.motor.config.current_lim = MOTOR["current_lim"]
    ax.motor.config.resistance_calib_max_voltage = MOTOR["resistance_calib_max_voltage"]

    # コントローラ（速度制御）
    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    ax.controller.config.vel_ramp_rate = RUN["vel_ramp_rate"]
    ax.controller.config.vel_limit = RUN["vel_limit"]

    if FEEDBACK_MODE == "encoder":
        ax.encoder.config.cpr = ENCODER["cpr"]
        ax.config.enable_sensorless_mode = False
        if ENCODER["use_index"]:
            # Z相あり: 起動時にインデックス探索して即クローズドループ可能にする
            ax.encoder.config.use_index = True
            ax.config.startup_encoder_index_search = True
            ax.config.startup_encoder_offset_calibration = False
        else:
            # Z相なし: 起動ごとにオフセット校正（起動時にモータが自由に少し回る必要あり）
            ax.encoder.config.use_index = False
            ax.config.startup_encoder_index_search = False
            ax.config.startup_encoder_offset_calibration = True
    elif FEEDBACK_MODE == "sensorless":
        # センサレス推定に必要な磁束鎖交数。ODrive の推奨式。
        ax.sensorless_estimator.config.pm_flux_linkage = \
            5.51 / (MOTOR["pole_pairs"] * MOTOR["kv"])
        ax.config.enable_sensorless_mode = True
        # センサレス起動ランプ（必要に応じて調整）
        ax.config.sensorless_ramp.current = MOTOR["calibration_current"]
    else:
        raise ValueError("FEEDBACK_MODE は 'encoder' か 'sensorless'")


def calibrate_axis(odrv, ax, name):
    print(f"\n===== {name} を校正 =====")
    odrv.clear_errors()
    if FEEDBACK_MODE == "encoder":
        ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    else:
        # センサレスはエンコーダオフセット校正が不要。モータ校正のみ。
        ax.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(1.0)
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.2)
    dump_errors(odrv)

    if FEEDBACK_MODE == "encoder":
        ok = ax.motor.is_calibrated and ax.encoder.is_ready
        if ok:
            ax.motor.config.pre_calibrated = True
            ax.encoder.config.pre_calibrated = True
    else:
        ok = ax.motor.is_calibrated
        if ok:
            ax.motor.config.pre_calibrated = True
    print(f"{name}: calibrated = {ok}")
    return ok


def test_spin(odrv, ax, name):
    print(f"\n===== {name} をテスト回転 (1.0 turn/s, 3s) =====")
    odrv.clear_errors()
    ax.controller.input_vel = 0.0
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)
    if ax.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print(f"{name}: クローズドループに入れませんでした。")
        dump_errors(odrv)
        return
    ax.controller.input_vel = 1.0
    time.sleep(3.0)
    ax.controller.input_vel = 0.0
    time.sleep(0.5)
    ax.requested_state = AXIS_STATE_IDLE


def main():
    serial = sys.argv[1] if len(sys.argv) > 1 else None
    if serial:
        print(f"Searching ODrive serial={serial} ...")
        odrv = odrive.find_any(serial_number=serial)
    else:
        print("Searching any ODrive over USB ...")
        odrv = odrive.find_any()

    print(f"Connected. fw={odrv.fw_version_major}.{odrv.fw_version_minor}."
          f"{odrv.fw_version_revision}, Vbus={odrv.vbus_voltage:.1f}V, "
          f"serial={format(odrv.serial_number, 'x').upper()}")
    print(f"FEEDBACK_MODE = {FEEDBACK_MODE}")

    odrv.clear_errors()

    # 電源系（ボード全体で1つ）
    odrv.config.dc_bus_overvoltage_trip_level = POWER["dc_bus_overvoltage_trip_level"]
    odrv.config.dc_bus_undervoltage_trip_level = POWER["dc_bus_undervoltage_trip_level"]
    odrv.config.dc_max_positive_current = POWER["dc_max_positive_current"]
    odrv.config.dc_max_negative_current = POWER["dc_max_negative_current"]
    odrv.config.enable_brake_resistor = POWER["enable_brake_resistor"]
    odrv.config.brake_resistance = POWER["brake_resistance"]

    # 2軸ぶん設定
    configure_axis(odrv, odrv.axis0, "axis0")
    configure_axis(odrv, odrv.axis1, "axis1")

    print("\nSaving configuration ...")
    try:
        odrv.save_configuration()
    except Exception:
        pass
    time.sleep(2.0)

    # save で再起動が入るので再接続
    print("Reconnecting after save ...")
    odrv = odrive.find_any(serial_number=serial) if serial else odrive.find_any()

    ok0 = calibrate_axis(odrv, odrv.axis0, "axis0")
    ok1 = calibrate_axis(odrv, odrv.axis1, "axis1")

    if ok0 or ok1:
        print("\nSaving pre_calibrated flags ...")
        try:
            odrv.save_configuration()
        except Exception:
            pass
        time.sleep(2.0)
        odrv = odrive.find_any(serial_number=serial) if serial else odrive.find_any()

    if ok0:
        test_spin(odrv, odrv.axis0, "axis0")
    if ok1:
        test_spin(odrv, odrv.axis1, "axis1")

    print("\n完了。両軸が回れば、この1台のハード準備は完了です。")
    print("2台目は SERIAL_NUMBER を指定してもう一度実行してください。")


if __name__ == "__main__":
    main()
