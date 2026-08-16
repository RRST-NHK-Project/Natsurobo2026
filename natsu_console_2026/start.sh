#!/bin/bash

# R2 Console - 起動スクリプト
# rosbridge と GUI を起動します

# FastDDSのSHM transport崩壊(open_and_lock_file failed)を避け、UDP loopbackに統一する。
# これがないと古い端末から起動した時にSHMになり、UDPの他ノードと分断する。
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

set -e

# スクリプトの場所を取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONSOLE_DIR="${SCRIPT_DIR}"

cd "${CONSOLE_DIR}"

BRIDGE_PORT="${BRIDGE_PORT:-9090}"
BRIDGE_PID=""
CONSOLE_BACKEND_PORT="${CONSOLE_BACKEND_PORT:-3031}"
BACKEND_PID=""

# センサを sensor_supervisor 経由でGUIと一緒に起動する。
# supervisor が lidar/IMU のポートを自動検知し、データ停止時は自動復旧する。
# START_SENSORS=0 で無効化、SENSOR_USE_IMU=false でIMUを起動しない、
# SENSOR_USE_RVIZ=false で RViz(点群表示)を出さない。
# LiDARとIMUは基本的に常時接続しているのでIMUもデフォルトで起動する。
# IMU未接続で試すときだけ SENSOR_USE_IMU=false を渡す。
START_SENSORS="${START_SENSORS:-1}"
SENSOR_USE_IMU="${SENSOR_USE_IMU:-true}"
SENSOR_USE_RVIZ="${SENSOR_USE_RVIZ:-true}"
SENSOR_PID=""

# ros2 をシステムPython環境で(仮想環境を避けて)バックグラウンド起動する共通ヘルパ。
# 使い方: run_ros2_bg <logfile> <ros2 引数...>
run_ros2_bg() {
  local logfile="$1"; shift
  if [ -n "${VIRTUAL_ENV:-}" ]; then
    local CLEAN_PATH="${PATH}"
    CLEAN_PATH="${CLEAN_PATH#"${VIRTUAL_ENV}/bin:"}"
    (
      unset VIRTUAL_ENV
      export PATH="${CLEAN_PATH}"
      exec ros2 "$@"
    ) >"${logfile}" 2>&1 &
  else
    ros2 "$@" >"${logfile}" 2>&1 &
  fi
}

start_rosbridge() {
  if [ -n "${VIRTUAL_ENV:-}" ]; then
    echo "Python仮想環境を検出: ${VIRTUAL_ENV}"
    echo "rosbridge はシステムPython環境で起動します"
  fi
  run_ros2_bg /tmp/r2_console_rosbridge.log \
    launch rosbridge_server rosbridge_websocket_launch.xml port:=${BRIDGE_PORT}
  BRIDGE_PID=$!
}

start_sensors() {
  run_ros2_bg /tmp/r2_console_sensors.log \
    run sensor_viz sensor_supervisor --ros-args \
      -p use_imu:=${SENSOR_USE_IMU} -p use_rviz:=${SENSOR_USE_RVIZ}
  SENSOR_PID=$!
}

cleanup() {
  if [ -n "${BACKEND_PID}" ] && kill -0 "${BACKEND_PID}" >/dev/null 2>&1; then
    echo ""
    echo "console backend を停止しています..."
    kill "${BACKEND_PID}" >/dev/null 2>&1 || true
    wait "${BACKEND_PID}" 2>/dev/null || true
  fi

  if [ -n "${SENSOR_PID}" ] && kill -0 "${SENSOR_PID}" >/dev/null 2>&1; then
    echo ""
    echo "センサノードを停止しています..."
    kill "${SENSOR_PID}" >/dev/null 2>&1 || true
    wait "${SENSOR_PID}" 2>/dev/null || true
  fi

  if [ -n "${BRIDGE_PID}" ] && kill -0 "${BRIDGE_PID}" >/dev/null 2>&1; then
    echo ""
    echo "rosbridge を停止しています..."
    kill "${BRIDGE_PID}" >/dev/null 2>&1 || true
    wait "${BRIDGE_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if command -v ros2 >/dev/null 2>&1; then
  if command -v ss >/dev/null 2>&1 && ss -ltn "sport = :${BRIDGE_PORT}" | grep -q ":${BRIDGE_PORT}"; then
    echo "rosbridge は既にポート ${BRIDGE_PORT} で起動中です"
  else
    echo "rosbridge を起動中... (port: ${BRIDGE_PORT})"
    start_rosbridge
    sleep 1

    if ! kill -0 "${BRIDGE_PID}" >/dev/null 2>&1; then
      echo "rosbridge の起動に失敗しました。ログ: /tmp/r2_console_rosbridge.log"
      exit 1
    fi
  fi
else
  echo "ros2 コマンドが見つかりません。先に ROS2 環境を有効化してください。"
  exit 1
fi

if [ "${START_SENSORS}" = "1" ]; then
  echo "センサを起動中... (sensor_supervisor: ポート自動検知/自動復旧, use_imu:=${SENSOR_USE_IMU}, use_rviz:=${SENSOR_USE_RVIZ})"
  start_sensors
  sleep 1

  if ! kill -0 "${SENSOR_PID}" >/dev/null 2>&1; then
    # センサ(LiDAR等)が繋がっていない環境でも GUI は動かしたいので、
    # ここで exit せず警告だけ出して続行する。
    echo "⚠ センサノードの起動に失敗しました。ログ: /tmp/r2_console_sensors.log"
    echo "  センサ無しのままGUIを起動します (センサ表示は出ません)。"
    echo "  最初からセンサを起動しない場合は START_SENSORS=0 ./start.sh"
    SENSOR_PID=""
  fi
else
  echo "センサノードの起動はスキップします (START_SENSORS=0)"
fi

if command -v node >/dev/null 2>&1; then
  if command -v ss >/dev/null 2>&1 && ss -ltn "sport = :${CONSOLE_BACKEND_PORT}" | grep -q ":${CONSOLE_BACKEND_PORT}"; then
    echo "console backend は既にポート ${CONSOLE_BACKEND_PORT} で起動中です"
  else
    echo "console backend を起動中... (port: ${CONSOLE_BACKEND_PORT})"
    CONSOLE_BACKEND_PORT="${CONSOLE_BACKEND_PORT}" node ./tools/console_backend.js >/tmp/r2_console_backend.log 2>&1 &
    BACKEND_PID=$!
    sleep 1

    if ! kill -0 "${BACKEND_PID}" >/dev/null 2>&1; then
      echo "console backend の起動に失敗しました。ログ: /tmp/r2_console_backend.log"
      exit 1
    fi
  fi
else
  echo "node コマンドが見つかりません。console backend を起動できません。"
fi

# 開発サーバーを起動
echo "=================================="
echo "R2 Console を起動しています..."
echo "=================================="
echo ""
echo "rosbridge: ws://localhost:${BRIDGE_PORT}"
echo "console backend: http://localhost:${CONSOLE_BACKEND_PORT}"
if [ -n "${SENSOR_PID}" ]; then
  echo "センサノード: 起動済み (ログ: /tmp/r2_console_sensors.log)"
elif [ "${START_SENSORS}" = "1" ]; then
  echo "センサノード: 起動できず (センサ無しで続行中)"
fi
echo "ブラウザで http://localhost:3000 を開いてください"
echo "終了するには Ctrl+C を押してください (センサノードも一緒に停止します)"
echo ""

npm start
