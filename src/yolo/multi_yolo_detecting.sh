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
child_pids=()
cleanup_started=0

stop_and_wait_for_children() {
    if [ "$cleanup_started" -eq 1 ]; then
        return
    fi
    cleanup_started=1
    trap - TERM INT HUP

    local pid
    for pid in "${child_pids[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done

    (
        sleep "${YOLO_SHUTDOWN_GRACE_SECONDS:-5}"
        for pid in "${child_pids[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                kill -KILL "$pid" 2>/dev/null || true
            fi
        done
    ) &
    local watchdog_pid=$!

    for pid in "${child_pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done

    kill -TERM "$watchdog_pid" 2>/dev/null || true
    wait "$watchdog_pid" 2>/dev/null || true
}

handle_signal() {
    local returncode="$1"
    stop_and_wait_for_children
    exit "$returncode"
}

trap 'handle_signal 143' TERM
trap 'handle_signal 130' INT
trap 'handle_signal 129' HUP

while(( $vehicle_num< typhoon_h480_num))
do
    "$YOLO_PYTHON" "$SCRIPT_DIR/yolo11n.py" typhoon_h480 "$vehicle_num" &
    child_pids+=("$!")
    let "vehicle_num++"
done

wait -n "${child_pids[@]}"
returncode=$?
stop_and_wait_for_children
exit "$returncode"
