#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
YOLO_PYTHON="${YOLO_PYTHON:-$PROJECT_ROOT/.venv-yolo/bin/python}"

if [ ! -x "$YOLO_PYTHON" ]; then
    echo "错误：找不到 YOLO Python 环境：$YOLO_PYTHON" >&2
    exit 1
fi

export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-$PROJECT_ROOT/.ultralytics}"

typhoon_h480_num=6
vehicle_num=0

while(( $vehicle_num< typhoon_h480_num))
do
    "$YOLO_PYTHON" "$SCRIPT_DIR/yolo11n.py" typhoon_h480 "$vehicle_num" &
    let "vehicle_num++"
done
