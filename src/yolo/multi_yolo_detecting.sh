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
child_start_times=()
cleanup_started=0
pending_signal_status=0
startup_complete=0

read_start_time() {
    local pid="$1"
    local proc_root="${YOLO_TEST_PROC_ROOT:-/proc}"
    local stat_line
    local stat_fields
    local stat_parts=()
    if [ ! -r "$proc_root/$pid/stat" ]; then
        return 1
    fi
    IFS= read -r stat_line < "$proc_root/$pid/stat" || return 1
    stat_fields="${stat_line##*) }"
    if [ "$stat_fields" = "$stat_line" ]; then
        return 1
    fi
    read -r -a stat_parts <<< "$stat_fields"
    if [ "${#stat_parts[@]}" -lt 20 ] || [[ ! "${stat_parts[19]}" =~ ^[0-9]+$ ]]; then
        return 1
    fi
    printf '%s\n' "${stat_parts[19]}"
}

worker_identity_matches() {
    local index="$1"
    local expected="${child_start_times[$index]}"
    local current
    if [ -z "$expected" ]; then
        return 1
    fi
    current="$(read_start_time "${child_pids[$index]}")" || return 1
    [ "$current" = "$expected" ]
}

record_signal() {
    if [ "$pending_signal_status" -eq 0 ]; then
        pending_signal_status="$1"
    fi
}

handle_signal() {
    local status="$1"
    if [ "$startup_complete" -eq 0 ]; then
        record_signal "$status"
        return
    fi
    trap '' TERM INT HUP
    stop_and_wait_for_children
    exit "$status"
}

stop_and_wait_for_children() {
    if [ "$cleanup_started" -eq 1 ]; then
        return
    fi
    cleanup_started=1
    trap '' TERM INT HUP

    local index
    local pid
    for index in "${!child_pids[@]}"; do
        pid="${child_pids[$index]}"
        if worker_identity_matches "$index"; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done

    (
        sleep "${YOLO_SHUTDOWN_GRACE_SECONDS:-5}"
        for index in "${!child_pids[@]}"; do
            pid="${child_pids[$index]}"
            if worker_identity_matches "$index"; then
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

trap 'handle_signal 143' TERM
trap 'handle_signal 130' INT
trap 'handle_signal 129' HUP

while(( $vehicle_num< typhoon_h480_num))
do
    "$YOLO_PYTHON" "$SCRIPT_DIR/yolo11n.py" typhoon_h480 "$vehicle_num" &
    worker_pid=$!
    if [ -n "${YOLO_TEST_AFTER_WORKER_START_HOOK:-}" ]; then
        "$YOLO_TEST_AFTER_WORKER_START_HOOK" "$$" "$worker_pid" "$vehicle_num"
    fi
    child_pids+=("$worker_pid")
    worker_start_time="$(read_start_time "$worker_pid")" || worker_start_time=""
    child_start_times+=("$worker_start_time")
    let "vehicle_num++"
    if [ "$pending_signal_status" -ne 0 ]; then
        stop_and_wait_for_children
        exit "$pending_signal_status"
    fi
done

startup_complete=1
if [ "$pending_signal_status" -ne 0 ]; then
    stop_and_wait_for_children
    exit "$pending_signal_status"
fi

wait -n "${child_pids[@]}"
returncode=$?
if [ "$pending_signal_status" -ne 0 ]; then
    returncode="$pending_signal_status"
fi
stop_and_wait_for_children
exit "$returncode"
