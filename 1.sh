#!/bin/bash

# ===================================================================
# ROS / PX4 六机仿真一键启动脚本 - down_resume 版本
# 用法: bash 1.sh [无人机数量] [任务文件名]
# 常规任务: bash 1.sh 6 mission_down.json
# BP 备份路线: bash 1.sh 6 mission_bp.json
# 本脚本替代旧 bp.sh 的节点功能，但不创建 tmux 多窗格。
# ===================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
PROJECT_ROOT="$(cd "$WORKSPACE_DIR/.." && pwd)"
PX4_DIR="${PX4_DIR:-$PROJECT_ROOT/PX4_Firmware}"
XTDRONE_DIR="${XTDRONE_DIR:-$PROJECT_ROOT/XTDrone}"
GAZEBO_MODELS_DIR="${GAZEBO_MODELS_DIR:-$PROJECT_ROOT/gazebo_models}"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
SIMULATION_LAUNCH="$WORKSPACE_DIR/robocup_zzufly.launch"
ROS_SETUP_FILE="${ROS_SETUP_FILE:-/opt/ros/noetic/setup.bash}"
XTDRONE_PYTHON="${XTDRONE_PYTHON:-/usr/bin/python3}"
XTDRONE_PYTHONPATH="${XTDRONE_PYTHONPATH:-$PROJECT_ROOT/.xtdrone-python}"
COMPLIANCE_PACKAGE_DIR="$WORKSPACE_DIR/src/competition_compliance"
COMPLIANCE_PYTHON="${COMPLIANCE_PYTHON:-/usr/bin/python3}"
PREPARE_MODEL="$COMPLIANCE_PACKAGE_DIR/scripts/prepare_model.py"
OFFICIAL_MANIFEST="$COMPLIANCE_PACKAGE_DIR/config/official_manifest.json"
SENSOR_MOUNT_CONFIG="$COMPLIANCE_PACKAGE_DIR/config/sensor_mount.yaml"
PROCESS_SUPERVISOR="$WORKSPACE_DIR/scripts/process_supervisor.py"
SUPERVISOR_PYTHON="${SUPERVISOR_PYTHON:-/usr/bin/python3}"
RUN_TMP_DIR=""
GENERATED_MODEL=""
READY_TIMEOUT_SECONDS="${READY_TIMEOUT_SECONDS:-180}"
COMMUNICATION_TIMEOUT_SECONDS="${COMMUNICATION_TIMEOUT_SECONDS:-20}"
CAMERA_TIMEOUT_SECONDS="${CAMERA_TIMEOUT_SECONDS:-60}"
CLEANUP_GRACE_SECONDS="${CLEANUP_GRACE_SECONDS:-5}"
HELPER_SURVIVAL_SECONDS="${HELPER_SURVIVAL_SECONDS:-2}"
OFFBOARD_WARMUP_SECONDS="${OFFBOARD_WARMUP_SECONDS:-2}"
LOG_DIR="$WORKSPACE_DIR/logs/competition-clean"
RUN_LOG=""
TEE_BIN="${TEE_BIN:-tee}"

NUM_DRONES=${1:-6}
MISSION_FILE=${2:-mission_down.json}
SIMULATION_PID=""
COMMUNICATION_PID=""
MISSION_PID=""
PENDING_PID=""
PENDING_START_TIME=""
HELPER_PIDS=()
OWNED_PIDS=()
OWNED_GATE_DIRS=()
DIRECT_FALLBACK_PIDS=()
declare -A DIRECT_FALLBACK_START_TIMES=()
declare -A OWNED_NAMES=()
declare -A OWNED_START_TIMES=()
declare -A OWNED_REAPED=()
declare -A OWNED_STATUSES=()
declare -A OWNED_GROUP_ESTABLISHED=()
LAST_STARTED_PID=""
LAST_PROCESS_STATUS=""
CLEANUP_DONE=false
CLEANUP_STATUS=0

LOGGER_DIR=""
LOGGER_FIFO=""
LOGGER_PID=""
LOGGER_FDS_SAVED=false
LOGGER_REDIRECTED=false
LOGGER_STARTED=false
LOGGER_REAPED=false
LOGGER_STATUS=0
OFFICIAL_SANDBOX_CHILD_PID=""
OFFICIAL_SANDBOX_PENDING_SIGNAL=""

forward_official_sandbox_signal() {
    local signal_name="$1"

    OFFICIAL_SANDBOX_PENDING_SIGNAL="$signal_name"
    if [ -n "$OFFICIAL_SANDBOX_CHILD_PID" ]; then
        if [ "$signal_name" = INT ]; then
            # Async commands inherit SIGINT ignored; TERM reaches the inner cleanup trap.
            kill -TERM "$OFFICIAL_SANDBOX_CHILD_PID" 2>/dev/null || true
        else
            kill -s "$signal_name" "$OFFICIAL_SANDBOX_CHILD_PID" 2>/dev/null || true
        fi
    fi
}

official_root_is_readonly_mount() {
    local root="$1"
    local mount_options

    if [ ! -x /usr/bin/findmnt ]; then
        return 1
    fi
    if ! mount_options="$(
        /usr/bin/findmnt --noheadings --output OPTIONS --mountpoint "$root"
    )"; then
        return 1
    fi
    mount_options="${mount_options//[[:space:]]/}"
    case ",$mount_options," in
    *,ro,*) return 0 ;;
    *) return 1 ;;
    esac
}

ensure_official_readonly_sandbox() {
    local bwrap_path resolved_px4 resolved_xtdrone
    local resolved_gazebo_models resolved_xtdrone_pythonpath
    local status_dir status_fifo status_fd status_line status_deadline
    local bwrap_pid sandbox_status

    if [ "${ROBOCUP_OFFICIAL_ROOTS_READONLY:-}" = 1 ]; then
        if ! official_root_is_readonly_mount "$PX4_DIR"; then
            echo "错误：sandbox 标记无效，PX4_DIR 并非独立只读挂载：$PX4_DIR" >&2
            return 1
        fi
        if ! official_root_is_readonly_mount "$XTDRONE_DIR"; then
            echo "错误：sandbox 标记无效，XTDRONE_DIR 并非独立只读挂载：$XTDRONE_DIR" >&2
            return 1
        fi
        if ! official_root_is_readonly_mount "$GAZEBO_MODELS_DIR"; then
            echo "错误：sandbox 标记无效，GAZEBO_MODELS_DIR 并非独立只读挂载：$GAZEBO_MODELS_DIR" >&2
            return 1
        fi
        if ! official_root_is_readonly_mount "$XTDRONE_PYTHONPATH"; then
            echo "错误：sandbox 标记无效，XTDRONE_PYTHONPATH 并非独立只读挂载：$XTDRONE_PYTHONPATH" >&2
            return 1
        fi
        return 0
    fi
    bwrap_path="$(command -v bwrap 2>/dev/null)"
    if [ -z "$bwrap_path" ]; then
        echo "错误：缺少 bubblewrap，无法保护 PX4/XTDrone 官方目录。请运行 sudo apt install bubblewrap 后重试。" >&2
        return 1
    fi
    if [ ! -x /usr/bin/setsid ]; then
        echo "错误：缺少 /usr/bin/setsid，无法安全转发 sandbox 信号。" >&2
        return 1
    fi
    if [ ! -d "$PX4_DIR" ] || [ -L "$PX4_DIR" ]; then
        echo "错误：PX4_DIR 必须是存在的普通目录且最终组件不能是符号链接：$PX4_DIR" >&2
        return 1
    fi
    if [ ! -d "$XTDRONE_DIR" ] || [ -L "$XTDRONE_DIR" ]; then
        echo "错误：XTDRONE_DIR 必须是存在的普通目录且最终组件不能是符号链接：$XTDRONE_DIR" >&2
        return 1
    fi
    if [ ! -d "$GAZEBO_MODELS_DIR" ] || [ -L "$GAZEBO_MODELS_DIR" ]; then
        echo "错误：GAZEBO_MODELS_DIR 必须是存在的普通目录且最终组件不能是符号链接：$GAZEBO_MODELS_DIR" >&2
        return 1
    fi
    if [ ! -d "$XTDRONE_PYTHONPATH" ] || [ -L "$XTDRONE_PYTHONPATH" ]; then
        echo "错误：XTDRONE_PYTHONPATH 必须是存在的普通目录且最终组件不能是符号链接：$XTDRONE_PYTHONPATH" >&2
        return 1
    fi
    if ! resolved_px4="$(cd "$PX4_DIR" && pwd -P)"; then
        echo "错误：无法解析 PX4_DIR：$PX4_DIR" >&2
        return 1
    fi
    if ! resolved_xtdrone="$(cd "$XTDRONE_DIR" && pwd -P)"; then
        echo "错误：无法解析 XTDRONE_DIR：$XTDRONE_DIR" >&2
        return 1
    fi
    if ! resolved_gazebo_models="$(cd "$GAZEBO_MODELS_DIR" && pwd -P)"; then
        echo "错误：无法解析 GAZEBO_MODELS_DIR：$GAZEBO_MODELS_DIR" >&2
        return 1
    fi
    if ! resolved_xtdrone_pythonpath="$(cd "$XTDRONE_PYTHONPATH" && pwd -P)"; then
        echo "错误：无法解析 XTDRONE_PYTHONPATH：$XTDRONE_PYTHONPATH" >&2
        return 1
    fi
    PX4_DIR="$resolved_px4"
    XTDRONE_DIR="$resolved_xtdrone"
    GAZEBO_MODELS_DIR="$resolved_gazebo_models"
    XTDRONE_PYTHONPATH="$resolved_xtdrone_pythonpath"
    PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
    export PX4_DIR XTDRONE_DIR GAZEBO_MODELS_DIR XTDRONE_PYTHONPATH PX4_BUILD_DIR
    export ROBOCUP_OFFICIAL_ROOTS_READONLY=1

    if ! status_dir="$(mktemp -d /tmp/robocup-fly-bwrap.XXXXXX)"; then
        echo "错误：无法创建 bubblewrap 状态目录。" >&2
        return 1
    fi
    status_fifo="$status_dir/status.fifo"
    if ! mkfifo "$status_fifo"; then
        rmdir "$status_dir" 2>/dev/null || true
        echo "错误：无法创建 bubblewrap 状态管道。" >&2
        return 1
    fi
    if ! exec {status_fd}<>"$status_fifo"; then
        rm -f -- "$status_fifo"
        rmdir "$status_dir" 2>/dev/null || true
        echo "错误：无法打开 bubblewrap 状态管道。" >&2
        return 1
    fi
    rm -f -- "$status_fifo"
    rmdir "$status_dir" 2>/dev/null || true

    trap 'forward_official_sandbox_signal HUP' HUP
    trap 'forward_official_sandbox_signal INT' INT
    trap 'forward_official_sandbox_signal TERM' TERM
    /usr/bin/setsid "$bwrap_path" \
        --die-with-parent \
        --json-status-fd "$status_fd" \
        --dev-bind / / \
        --ro-bind "$PX4_DIR" "$PX4_DIR" \
        --ro-bind "$XTDRONE_DIR" "$XTDRONE_DIR" \
        --ro-bind "$GAZEBO_MODELS_DIR" "$GAZEBO_MODELS_DIR" \
        --ro-bind "$XTDRONE_PYTHONPATH" "$XTDRONE_PYTHONPATH" \
        "$SCRIPT_DIR/1.sh" "$@" &
    bwrap_pid=$!

    status_line=""
    status_deadline=$((SECONDS + 5))
    while [ -z "$status_line" ] && [ "$SECONDS" -lt "$status_deadline" ]; do
        IFS= read -r -t 1 status_line <&"$status_fd" || true
        if ! kill -0 "$bwrap_pid" 2>/dev/null; then
            break
        fi
    done
    if [[ "$status_line" =~ \"child-pid\"[[:space:]]*:[[:space:]]*([0-9]+) ]]; then
        OFFICIAL_SANDBOX_CHILD_PID="${BASH_REMATCH[1]}"
    else
        kill -TERM "$bwrap_pid" 2>/dev/null || true
        wait "$bwrap_pid" 2>/dev/null || true
        exec {status_fd}>&-
        trap - HUP INT TERM
        echo "错误：无法取得 bubblewrap 内层启动器 PID。" >&2
        return 1
    fi
    if [ -n "$OFFICIAL_SANDBOX_PENDING_SIGNAL" ]; then
        forward_official_sandbox_signal "$OFFICIAL_SANDBOX_PENDING_SIGNAL"
    fi

    while :; do
        if wait "$bwrap_pid"; then
            sandbox_status=0
        else
            sandbox_status=$?
        fi
        if ! kill -0 "$bwrap_pid" 2>/dev/null; then
            break
        fi
    done
    case "$OFFICIAL_SANDBOX_PENDING_SIGNAL" in
    HUP) sandbox_status=129 ;;
    INT) sandbox_status=130 ;;
    TERM) sandbox_status=143 ;;
    esac
    exec {status_fd}>&-
    trap - HUP INT TERM
    OFFICIAL_SANDBOX_CHILD_PID=""
    OFFICIAL_SANDBOX_PENDING_SIGNAL=""
    exit "$sandbox_status"
}

current_milliseconds() {
    date +%s%3N
}

sleep_milliseconds() {
    local milliseconds="$1"
    local duration

    if [ "$milliseconds" -le 0 ]; then
        return
    fi
    printf -v duration '%d.%03d' "$((milliseconds / 1000))" "$((milliseconds % 1000))"
    sleep "$duration"
}

sleep_until_deadline() {
    local deadline="$1"
    local maximum_milliseconds="$2"
    local remaining

    remaining=$((deadline - $(current_milliseconds)))
    if [ "$remaining" -le 0 ]; then
        return
    fi
    if [ "$remaining" -gt "$maximum_milliseconds" ]; then
        remaining="$maximum_milliseconds"
    fi
    sleep_milliseconds "$remaining"
}

validate_positive_integer() {
    local name="$1"
    local value="$2"

    if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
        echo "错误：$name 必须是正整数，当前值：$value" >&2
        return 1
    fi
}

validate_nonnegative_integer() {
    local name="$1"
    local value="$2"

    if [[ ! "$value" =~ ^(0|[1-9][0-9]*)$ ]]; then
        echo "错误：$name 必须是非负整数，当前值：$value" >&2
        return 1
    fi
}

validate_configuration() {
    validate_positive_integer READY_TIMEOUT_SECONDS "$READY_TIMEOUT_SECONDS" || return 1
    validate_positive_integer COMMUNICATION_TIMEOUT_SECONDS "$COMMUNICATION_TIMEOUT_SECONDS" || return 1
    validate_positive_integer CAMERA_TIMEOUT_SECONDS "$CAMERA_TIMEOUT_SECONDS" || return 1
    validate_positive_integer CLEANUP_GRACE_SECONDS "$CLEANUP_GRACE_SECONDS" || return 1
    validate_positive_integer HELPER_SURVIVAL_SECONDS "$HELPER_SURVIVAL_SECONDS" || return 1
    validate_nonnegative_integer OFFBOARD_WARMUP_SECONDS "$OFFBOARD_WARMUP_SECONDS" || return 1
}

process_is_running() {
    local pid="$1"
    local stat_line state

    kill -0 "$pid" 2>/dev/null || return 1
    IFS= read -r stat_line 2>/dev/null <"/proc/$pid/stat" || return 1
    stat_line="${stat_line##*) }"
    state="${stat_line%% *}"
    [ "$state" != Z ]
}

process_start_time() {
    local pid="$1"
    local stat_line state ppid pgrp session tty_nr tpgid flags
    local minflt cminflt majflt cmajflt utime stime cutime cstime
    local priority nice num_threads itrealvalue start_time remainder

    IFS= read -r stat_line 2>/dev/null <"/proc/$pid/stat" || return 1
    stat_line="${stat_line##*) }"
    read -r state ppid pgrp session tty_nr tpgid flags \
        minflt cminflt majflt cmajflt utime stime cutime cstime \
        priority nice num_threads itrealvalue start_time remainder <<< "$stat_line"
    [ -n "$start_time" ] || return 1
    printf '%s\n' "$start_time"
}

cleanup_logger_files() {
    if [ -n "$LOGGER_FIFO" ]; then
        case "$LOGGER_FIFO" in
        "$LOGGER_DIR"/*)
            rm -f -- "$LOGGER_FIFO"
            ;;
        *)
            echo "拒绝清理非本次启动创建的日志 FIFO：$LOGGER_FIFO" >&2
            ;;
        esac
        LOGGER_FIFO=""
    fi

    if [ -n "$LOGGER_DIR" ]; then
        case "$LOGGER_DIR" in
        "$LOG_DIR"/.logger-*)
            rmdir -- "$LOGGER_DIR" 2>/dev/null || true
            ;;
        *)
            echo "拒绝清理非本次启动创建的 logger 目录：$LOGGER_DIR" >&2
            ;;
        esac
        LOGGER_DIR=""
    fi
}

stop_logger_bounded() {
    local pid="$1"
    local deadline

    if ! process_is_running "$pid"; then
        return
    fi
    kill -TERM "$pid" 2>/dev/null || true
    deadline=$(( $(current_milliseconds) + 500 ))
    while process_is_running "$pid" && [ "$(current_milliseconds)" -lt "$deadline" ]; do
        sleep_milliseconds 50
    done
    if process_is_running "$pid"; then
        kill -KILL "$pid" 2>/dev/null || true
        deadline=$(( $(current_milliseconds) + 500 ))
        while process_is_running "$pid" && [ "$(current_milliseconds)" -lt "$deadline" ]; do
            sleep_milliseconds 50
        done
    fi
    if process_is_running "$pid"; then
        return 1
    fi
    return 0
}

reap_logger_if_exited() {
    local status

    if ! "$LOGGER_STARTED"; then
        return 0
    fi
    if "$LOGGER_REAPED"; then
        return 0
    fi
    if process_is_running "$LOGGER_PID"; then
        return 1
    fi
    if wait "$LOGGER_PID"; then
        status=0
    else
        status=$?
    fi
    LOGGER_REAPED=true
    LOGGER_STATUS="$status"
    return 0
}

check_logger_health() {
    if ! "$LOGGER_STARTED"; then
        return 1
    fi
    if ! reap_logger_if_exited; then
        return 0
    fi
    if "$LOGGER_FDS_SAVED"; then
        echo "错误：启动日志进程提前退出（状态 $LOGGER_STATUS）。" >&4
    else
        echo "错误：启动日志进程提前退出（状态 $LOGGER_STATUS）。" >&2
    fi
    return 1
}

setup_logging() {
    local tee_path

    if ! mkdir -p "$LOG_DIR"; then
        echo "错误：无法创建启动日志目录：$LOG_DIR" >&2
        return 1
    fi
    if [ ! -d "$LOG_DIR" ] || [ ! -w "$LOG_DIR" ]; then
        echo "错误：启动日志目录不可写：$LOG_DIR" >&2
        return 1
    fi
    if ! RUN_LOG="$(mktemp "$LOG_DIR/launch-$(date +%Y%m%d-%H%M%S)-XXXXXX.log")"; then
        echo "错误：无法创建本次启动日志文件。" >&2
        return 1
    fi
    if ! LOGGER_DIR="$(mktemp -d "$LOG_DIR/.logger-XXXXXX")"; then
        echo "错误：无法创建 logger 临时目录。" >&2
        return 1
    fi
    LOGGER_FIFO="$LOGGER_DIR/output.fifo"
    if ! mkfifo "$LOGGER_FIFO"; then
        echo "错误：无法创建启动日志 FIFO。" >&2
        cleanup_logger_files
        return 1
    fi
    tee_path="$(command -v "$TEE_BIN" 2>/dev/null)"
    if [ -z "$tee_path" ] || [ ! -x "$tee_path" ]; then
        echo "错误：找不到可执行的 tee：$TEE_BIN" >&2
        cleanup_logger_files
        return 1
    fi
    if ! "$TEE_BIN" -a "$RUN_LOG" </dev/null >/dev/null 2>&1; then
        echo "错误：tee 无法写入启动日志：$RUN_LOG" >&2
        cleanup_logger_files
        return 1
    fi

    exec 3>&1 4>&2
    LOGGER_FDS_SAVED=true
    "$TEE_BIN" -a "$RUN_LOG" <"$LOGGER_FIFO" >&3 2>&4 &
    LOGGER_PID=$!
    LOGGER_STARTED=true
    LOGGER_REAPED=false
    LOGGER_STATUS=0
    trap '' PIPE
    if ! exec 1>"$LOGGER_FIFO" 2>&1; then
        exec 1>&3 2>&4
        stop_logger_bounded "$LOGGER_PID"
        reap_logger_if_exited || true
        LOGGER_STARTED=false
        trap - PIPE
        cleanup_logger_files
        return 1
    fi
    LOGGER_REDIRECTED=true
    if ! process_is_running "$LOGGER_PID"; then
        exec 1>&3 2>&4
        LOGGER_REDIRECTED=false
        reap_logger_if_exited || true
        LOGGER_STARTED=false
        trap - PIPE
        echo "错误：启动日志进程在初始化期间退出。" >&2
        cleanup_logger_files
        return 1
    fi

    echo "本次完整启动日志：$RUN_LOG"
}

finish_logging() {
    local primary_status="$1"
    local logger_status=0
    local deadline
    local logger_forced=false

    if "$LOGGER_REDIRECTED"; then
        exec 1>&3 2>&4
        LOGGER_REDIRECTED=false
    fi

    if "$LOGGER_STARTED" && ! "$LOGGER_REAPED"; then
        deadline=$(( $(current_milliseconds) + CLEANUP_GRACE_SECONDS * 1000 ))
        while process_is_running "$LOGGER_PID" && [ "$(current_milliseconds)" -lt "$deadline" ]; do
            sleep_milliseconds 50
        done
        if process_is_running "$LOGGER_PID"; then
            echo "错误：启动日志进程未在期限内结束，正在强制停止。" >&2
            logger_forced=true
            stop_logger_bounded "$LOGGER_PID" || logger_status=1
        fi
        if ! reap_logger_if_exited; then
            logger_status=1
        fi
    fi
    if "$LOGGER_STARTED"; then
        if [ "$logger_status" -eq 0 ]; then
            logger_status="$LOGGER_STATUS"
        fi
        if "$logger_forced" && [ "$logger_status" -eq 0 ]; then
            logger_status=1
        fi
        LOGGER_STARTED=false
    fi

    cleanup_logger_files
    if "$LOGGER_FDS_SAVED"; then
        exec 3>&- 4>&-
        LOGGER_FDS_SAVED=false
    fi
    trap - PIPE

    if [ "$logger_status" -ne 0 ]; then
        echo "错误：启动日志进程异常退出（状态 $logger_status）。" >&2
        if [ "$primary_status" -eq 0 ]; then
            return 1
        fi
    fi
    return "$primary_status"
}

register_owned_process() {
    local pid="$1"
    local name="$2"
    local start_time

    if ! start_time="$(process_start_time "$pid")"; then
        echo "错误：无法记录${name}的进程身份（PID $pid）。" >&2
        return 1
    fi

    OWNED_PIDS+=("$pid")
    OWNED_NAMES["$pid"]="$name"
    OWNED_START_TIMES["$pid"]="$start_time"
    OWNED_REAPED["$pid"]=false
    OWNED_GROUP_ESTABLISHED["$pid"]=false
    LAST_STARTED_PID="$pid"
}

owned_identity_matches() {
    local pid="$1"
    local current_start_time expected_start_time

    expected_start_time="${OWNED_START_TIMES[$pid]:-}"
    [ -n "$expected_start_time" ] || return 1
    current_start_time="$(process_start_time "$pid")" || return 1
    [ "$current_start_time" = "$expected_start_time" ]
}

owned_group_wrapper() {
    local owner_pid="$1"
    local ready_file="$2"
    local release_file="$3"
    local acknowledged_file="$4"
    local group_ready_file="$5"
    local start_file="$6"
    local command_started_file="$7"
    local gate_dir="${ready_file%/*}"
    shift 7

    touch "$ready_file" || return 125
    while [ ! -e "$release_file" ]; do
        if ! kill -0 "$owner_pid" 2>/dev/null || [ ! -d "$gate_dir" ]; then
            return 130
        fi
        sleep_milliseconds 10
    done
    touch "$acknowledged_file" || return 125
    exec setsid bash -c '
        group_ready_file="$1"
        start_file="$2"
        command_started_file="$3"
        supervisor_python="$4"
        process_supervisor="$5"
        cleanup_grace_seconds="$6"
        shift 6
        touch "$group_ready_file" || exit 125
        while [ ! -e "$start_file" ]; do
            [ -d "${start_file%/*}" ] || exit 130
            sleep 0.01
        done
        touch "$command_started_file" || exit 125
        exec "$supervisor_python" "$process_supervisor" \
            --grace-seconds "$cleanup_grace_seconds" -- "$@"
    ' -- "$group_ready_file" "$start_file" "$command_started_file" \
        "$SUPERVISOR_PYTHON" "$PROCESS_SUPERVISOR" "$CLEANUP_GRACE_SECONDS" "$@"
}

cleanup_owned_gate_dir() {
    local gate_dir="$1"

    if [ -z "$RUN_TMP_DIR" ]; then
        return 1
    fi
    case "$gate_dir" in
    "$RUN_TMP_DIR"/owned-gate.*)
        rm -f -- \
            "$gate_dir/ready" \
            "$gate_dir/release" \
            "$gate_dir/acknowledged" \
            "$gate_dir/group-ready" \
            "$gate_dir/start" \
            "$gate_dir/command-started"
        rmdir -- "$gate_dir" 2>/dev/null || true
        ;;
    *)
        echo "拒绝清理非本次启动创建的进程门控目录：$gate_dir" >&2
        return 1
        ;;
    esac
}

cleanup_owned_gate_files() {
    local gate_dir

    for gate_dir in "${OWNED_GATE_DIRS[@]}"; do
        cleanup_owned_gate_dir "$gate_dir" || CLEANUP_STATUS=1
    done
}

start_owned_group() {
    local name="$1"
    local gate_dir ready_file release_file acknowledged_file
    local group_ready_file start_file command_started_file
    local deadline pgid pid
    shift

    if [ -z "$RUN_TMP_DIR" ]; then
        echo "错误：无法在临时目录建立${name}进程门控。" >&2
        return 1
    fi
    if ! gate_dir="$(mktemp -d "$RUN_TMP_DIR/owned-gate.XXXXXX")"; then
        echo "错误：无法为${name}创建进程门控目录。" >&2
        return 1
    fi
    OWNED_GATE_DIRS+=("$gate_dir")
    ready_file="$gate_dir/ready"
    release_file="$gate_dir/release"
    acknowledged_file="$gate_dir/acknowledged"
    group_ready_file="$gate_dir/group-ready"
    start_file="$gate_dir/start"
    command_started_file="$gate_dir/command-started"

    owned_group_wrapper \
        "$$" \
        "$ready_file" "$release_file" "$acknowledged_file" \
        "$group_ready_file" "$start_file" "$command_started_file" "$@" &
    PENDING_PID=$!
    pid="$PENDING_PID"
    if ! PENDING_START_TIME="$(process_start_time "$PENDING_PID")"; then
        echo "错误：无法记录${name}进程门控的身份（PID $PENDING_PID）。" >&2
        return 1
    fi
    deadline=$(( $(current_milliseconds) + 5000 ))

    while [ ! -e "$ready_file" ]; do
        if ! process_is_running "$PENDING_PID"; then
            wait "$PENDING_PID" 2>/dev/null || true
            PENDING_PID=""
            PENDING_START_TIME=""
            echo "错误：${name}进程门控在登记前退出。" >&2
            return 1
        fi
        check_logger_health || return 1
        if [ "$(current_milliseconds)" -ge "$deadline" ]; then
            echo "错误：等待${name}进程门控就绪超时。" >&2
            return 1
        fi
        sleep_milliseconds 10
    done

    register_owned_process "$PENDING_PID" "$name" || return 1
    touch "$release_file" || return 1

    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        if ! process_is_running "$pid"; then
            reap_if_exited "$pid" || true
            echo "错误：${name}在进程组建立前退出（状态 $LAST_PROCESS_STATUS）。" >&2
            return 1
        fi
        check_logger_health || return 1
        pgid="$(ps -o pgid= -p "$pid" 2>/dev/null)"
        pgid="${pgid//[[:space:]]/}"
        if [ -e "$acknowledged_file" ] && [ -e "$group_ready_file" ] && [ "$pgid" = "$pid" ]; then
            OWNED_GROUP_ESTABLISHED["$pid"]=true
            PENDING_PID=""
            PENDING_START_TIME=""
            touch "$start_file" || return 1
            while [ ! -e "$command_started_file" ]; do
                if ! process_is_running "$pid"; then
                    reap_if_exited "$pid" || true
                    echo "错误：${name}未能执行启动命令（状态 $LAST_PROCESS_STATUS）。" >&2
                    return 1
                fi
                check_logger_health || return 1
                if [ "$(current_milliseconds)" -ge "$deadline" ]; then
                    echo "错误：等待${name}启动命令执行超时。" >&2
                    return 1
                fi
                sleep_milliseconds 10
            done
            cleanup_owned_gate_dir "$gate_dir" || return 1
            return 0
        fi
        sleep_milliseconds 10
    done

    echo "错误：${name}未能在期限内建立独立进程组。" >&2
    return 1
}

reap_if_exited() {
    local pid="$1"
    local status

    LAST_PROCESS_STATUS=""
    if [ "${OWNED_REAPED[$pid]:-false}" = true ]; then
        LAST_PROCESS_STATUS="${OWNED_STATUSES[$pid]}"
        return 0
    fi
    if owned_identity_matches "$pid" && process_is_running "$pid"; then
        return 1
    fi
    if wait "$pid"; then
        status=0
    else
        status=$?
    fi
    OWNED_REAPED["$pid"]=true
    OWNED_STATUSES["$pid"]="$status"
    LAST_PROCESS_STATUS="$status"
    return 0
}

wait_for_owned_process() {
    local pid="$1"
    local status

    if [ "${OWNED_REAPED[$pid]:-false}" = true ]; then
        LAST_PROCESS_STATUS="${OWNED_STATUSES[$pid]}"
        return
    fi
    if wait "$pid"; then
        status=0
    else
        status=$?
    fi
    OWNED_REAPED["$pid"]=true
    OWNED_STATUSES["$pid"]="$status"
    LAST_PROCESS_STATUS="$status"
}

report_early_exit() {
    local pid="$1"
    local context="$2"

    if ! reap_if_exited "$pid"; then
        return 1
    fi
    echo "错误：${context}进程提前退出（状态 $LAST_PROCESS_STATUS）。" >&2
    return 0
}

check_owned_dependency() {
    local pid="$1"
    local context="$2"

    if ! reap_if_exited "$pid"; then
        return 0
    fi
    echo "错误：${OWNED_NAMES[$pid]}在${context}期间退出（状态 $LAST_PROCESS_STATUS）。" >&2
    return 1
}

check_required_processes() {
    local context="$1"
    local pid

    check_logger_health || return 1
    if [ -n "$SIMULATION_PID" ]; then
        check_owned_dependency "$SIMULATION_PID" "$context" || return 1
    fi
    if [ -n "$COMMUNICATION_PID" ]; then
        check_owned_dependency "$COMMUNICATION_PID" "$context" || return 1
    fi
    for pid in "${HELPER_PIDS[@]}"; do
        check_owned_dependency "$pid" "$context" || return 1
    done
    return 0
}

owned_target_is_running() {
    local pid="$1"

    owned_identity_matches "$pid" || return 1
    process_is_running "$pid"
}

collect_direct_process_tree() {
    local root_pid="$1"
    local deadline="$2"
    local child expected_start_time now remaining snapshot start_time timeout_value

    expected_start_time="${DIRECT_FALLBACK_START_TIMES[$root_pid]:-}"
    if [ -z "$expected_start_time" ]; then
        expected_start_time="${OWNED_START_TIMES[$root_pid]:-}"
    fi
    if [ -z "$expected_start_time" ] \
        && [ "$root_pid" = "$PENDING_PID" ]; then
        expected_start_time="$PENDING_START_TIME"
    fi
    [ -n "$expected_start_time" ] || return
    start_time="$(process_start_time "$root_pid")" || return
    [ "$start_time" = "$expected_start_time" ] || return
    if [ -z "${DIRECT_FALLBACK_START_TIMES[$root_pid]+tracked}" ]; then
        DIRECT_FALLBACK_START_TIMES["$root_pid"]="$expected_start_time"
        DIRECT_FALLBACK_PIDS+=("$root_pid")
    fi

    now="$(current_milliseconds)"
    remaining=$((deadline - now))
    [ "$remaining" -gt 0 ] || return
    printf -v timeout_value '%d.%03ds' \
        "$((remaining / 1000))" "$((remaining % 1000))"
    snapshot="$(timeout "$timeout_value" ps -eo pid=,ppid= 2>/dev/null)" || return
    for child in $(awk -v root="$root_pid" '
        { parent[$1] = $2 }
        END {
            owned[root] = 1
            changed = 1
            while (changed) {
                changed = 0
                for (pid in parent) {
                    if (!owned[pid] && owned[parent[pid]]) {
                        owned[pid] = 1
                        changed = 1
                    }
                }
            }
            for (pid in owned) {
                if (pid != root && owned[pid]) print pid
            }
        }
    ' <<< "$snapshot"); do
        [ "$(current_milliseconds)" -lt "$deadline" ] || return
        start_time="$(process_start_time "$child")" || continue
        if [ -z "${DIRECT_FALLBACK_START_TIMES[$child]+tracked}" ]; then
            DIRECT_FALLBACK_START_TIMES["$child"]="$start_time"
            DIRECT_FALLBACK_PIDS+=("$child")
        fi
    done
}

direct_fallback_is_running() {
    local pid="$1"
    local current_start_time expected_start_time

    expected_start_time="${DIRECT_FALLBACK_START_TIMES[$pid]:-}"
    [ -n "$expected_start_time" ] || return 1
    process_is_running "$pid" || return 1
    current_start_time="$(process_start_time "$pid")" || return 1
    [ "$current_start_time" = "$expected_start_time" ]
}

signal_direct_process_tree() {
    local signal="$1"
    local root_pid="$2"
    local deadline="$3"
    local pid

    collect_direct_process_tree "$root_pid" "$deadline"
    for pid in "${DIRECT_FALLBACK_PIDS[@]}"; do
        [ "$(current_milliseconds)" -lt "$deadline" ] || return
        if direct_fallback_is_running "$pid"; then
            kill "-$signal" "$pid" 2>/dev/null || true
        fi
    done
}

signal_owned_target() {
    local signal="$1"
    local pid="$2"
    local deadline="${3:-$(( $(current_milliseconds) + CLEANUP_GRACE_SECONDS * 1000 ))}"

    owned_identity_matches "$pid" || return 0

    if [ "${OWNED_GROUP_ESTABLISHED[$pid]:-false}" = true ]; then
        if process_is_running "$pid"; then
            if [ "$signal" = TERM ]; then
                kill -TERM -- "-$pid" 2>/dev/null || true
            else
                kill -KILL -- "-$pid" 2>/dev/null || true
            fi
        fi
    elif process_is_running "$pid"; then
        signal_direct_process_tree "$signal" "$pid" "$deadline"
    fi
}

cleanup() {
    local index pid deadline active pending_unregistered

    if "$CLEANUP_DONE"; then
        return "$CLEANUP_STATUS"
    fi
    CLEANUP_DONE=true
    CLEANUP_STATUS=0

    echo
    echo "正在停止本脚本启动的节点..."
    deadline=$(( $(current_milliseconds) + CLEANUP_GRACE_SECONDS * 1000 ))

    pending_unregistered=""
    if [ -n "$PENDING_PID" ] && [ -z "${OWNED_NAMES[$PENDING_PID]+registered}" ]; then
        pending_unregistered="$PENDING_PID"
        if process_is_running "$pending_unregistered" \
            && [ -n "$PENDING_START_TIME" ]; then
            DIRECT_FALLBACK_START_TIMES["$pending_unregistered"]="$PENDING_START_TIME"
            DIRECT_FALLBACK_PIDS+=("$pending_unregistered")
            signal_direct_process_tree TERM "$pending_unregistered" "$deadline"
        fi
    fi

    for ((index=${#OWNED_PIDS[@]} - 1; index >= 0; index--)); do
        pid="${OWNED_PIDS[$index]}"
        signal_owned_target TERM "$pid" "$deadline"
    done

    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        active=false
        if [ -n "$pending_unregistered" ] \
            && direct_fallback_is_running "$pending_unregistered"; then
            active=true
        fi
        for pid in "${DIRECT_FALLBACK_PIDS[@]}"; do
            if direct_fallback_is_running "$pid"; then
                active=true
                break
            fi
        done
        for pid in "${OWNED_PIDS[@]}"; do
            if owned_target_is_running "$pid"; then
                active=true
                break
            fi
        done
        if ! "$active"; then
            break
        fi
        sleep_milliseconds 50
    done

    for pid in "${DIRECT_FALLBACK_PIDS[@]}"; do
        if direct_fallback_is_running "$pid"; then
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
    for pid in "${OWNED_PIDS[@]}"; do
        if owned_target_is_running "$pid"; then
            echo "进程组 ${OWNED_NAMES[$pid]} 未响应 TERM，发送 KILL。" >&2
            signal_owned_target KILL "$pid" "$deadline"
        fi
    done

    deadline=$(( $(current_milliseconds) + 1000 ))
    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        active=false
        if [ -n "$pending_unregistered" ] \
            && direct_fallback_is_running "$pending_unregistered"; then
            active=true
        fi
        for pid in "${DIRECT_FALLBACK_PIDS[@]}"; do
            if direct_fallback_is_running "$pid"; then
                active=true
                break
            fi
        done
        for pid in "${OWNED_PIDS[@]}"; do
            if owned_target_is_running "$pid"; then
                active=true
                break
            fi
        done
        if ! "$active"; then
            break
        fi
        sleep_milliseconds 50
    done

    for pid in "${DIRECT_FALLBACK_PIDS[@]}"; do
        if direct_fallback_is_running "$pid"; then
            echo "错误：直接清理的进程未在期限内退出（PID $pid）。" >&2
            CLEANUP_STATUS=1
        fi
    done
    for pid in "${OWNED_PIDS[@]}"; do
        if ! reap_if_exited "$pid"; then
            echo "错误：无法在清理期限内回收 ${OWNED_NAMES[$pid]}（PID $pid）。" >&2
            CLEANUP_STATUS=1
        fi
    done

    if [ -n "$pending_unregistered" ]; then
        if direct_fallback_is_running "$pending_unregistered"; then
            echo "错误：无法在清理期限内回收未登记进程（PID $pending_unregistered）。" >&2
            CLEANUP_STATUS=1
        else
            wait "$pending_unregistered" 2>/dev/null || true
        fi
    fi
    PENDING_PID=""
    PENDING_START_TIME=""
    cleanup_owned_gate_files

    if [ -n "$RUN_TMP_DIR" ]; then
        case "$RUN_TMP_DIR" in
        /tmp/robocup-fly-competition-clean.*)
            rm -rf -- "$RUN_TMP_DIR"
            ;;
        *)
            echo "拒绝清理非 competition-clean 临时目录：$RUN_TMP_DIR" >&2
            CLEANUP_STATUS=1
            ;;
        esac
    fi
    return "$CLEANUP_STATUS"
}

handle_exit() {
    local primary_status="$1"
    local cleanup_status logger_result final_status

    trap - EXIT HUP INT TERM
    cleanup
    cleanup_status=$?
    final_status="$primary_status"
    if [ "$final_status" -eq 0 ] && [ "$cleanup_status" -ne 0 ]; then
        final_status="$cleanup_status"
    fi
    if finish_logging "$final_status"; then
        logger_result=0
    else
        logger_result=$?
    fi
    if [ "$primary_status" -ne 0 ]; then
        final_status="$primary_status"
    elif [ "$logger_result" -ne 0 ]; then
        final_status="$logger_result"
    fi
    exit "$final_status"
}

on_interrupt() {
    exit "$1"
}

require_file() {
    if [ ! -e "$1" ]; then
        echo "错误：找不到 $2：$1" >&2
        exit 1
    fi
}

require_ros_package() {
    if ! rospack find "$1" >/dev/null 2>&1; then
        echo "错误：未找到 ROS 软件包 '$1'。请确认环境和依赖已正确安装。" >&2
        exit 1
    fi
}

project_simulator_running() {
    rosnode list 2>/dev/null | grep -Eq '^/typhoon_h480_[0-5]/mavros$'
}

all_vehicles_ready() {
    local deadline="$1"
    local topic_list
    local id state_topic pose_topic now remaining probe_milliseconds probe_timeout

    check_required_processes "仿真就绪检查" || return 1
    rosservice list 2>/dev/null | grep -qx '/gazebo/get_world_properties' || return 1
    topic_list="$(rostopic list 2>/dev/null)" || return 1

    for id in $(seq 0 5); do
        check_required_processes "仿真就绪检查" || return 1
        state_topic="/typhoon_h480_${id}/mavros/state"
        pose_topic="/typhoon_h480_${id}/mavros/local_position/pose"

        grep -qx "$pose_topic" <<< "$topic_list" || return 1
        now="$(current_milliseconds)"
        remaining=$((deadline - now))
        if [ "$remaining" -le 0 ]; then
            return 1
        fi
        probe_milliseconds=2000
        if [ "$remaining" -lt "$probe_milliseconds" ]; then
            probe_milliseconds="$remaining"
        fi
        printf -v probe_timeout '%d.%03ds' \
            "$((probe_milliseconds / 1000))" "$((probe_milliseconds % 1000))"
        timeout "$probe_timeout" rostopic echo -n 1 "$state_topic" 2>/dev/null \
            | grep -q 'connected: True' || return 1
    done
}

wait_for_simulator() {
    local deadline missing
    deadline=$(( $(current_milliseconds) + READY_TIMEOUT_SECONDS * 1000 ))

    echo "等待 Gazebo、PX4 和六个 MAVROS 实例连接（最长 ${READY_TIMEOUT_SECONDS} 秒）..."
    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        check_required_processes "仿真就绪检查" || return 1
        if all_vehicles_ready "$deadline"; then
            check_required_processes "仿真就绪检查" || return 1
            echo "六架无人机均已连接，开始启动任务节点。"
            return 0
        fi
        sleep_until_deadline "$deadline" 200
    done

    echo "错误：仿真未在 ${READY_TIMEOUT_SECONDS} 秒内就绪。" >&2
    echo "请检查 Gazebo/PX4/MAVROS 的终端输出，并确认以下话题均可用：" >&2
    for missing in $(seq 0 5); do
        echo "  /typhoon_h480_${missing}/mavros/state" >&2
        echo "  /typhoon_h480_${missing}/mavros/local_position/pose" >&2
    done
    return 1
}

all_communication_nodes_ready() {
    local id node_list
    node_list="$(rosnode list 2>/dev/null)" || return 1

    for id in $(seq 0 5); do
        grep -qx "/typhoon_h480_${id}_communication" <<< "$node_list" || return 1
    done
}

wait_for_communication() {
    local deadline warmup_deadline
    deadline=$(( $(current_milliseconds) + COMMUNICATION_TIMEOUT_SECONDS * 1000 ))

    echo "等待六个 XTDrone 通信节点就绪（最长 ${COMMUNICATION_TIMEOUT_SECONDS} 秒）..."
    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        check_required_processes "通信节点就绪检查" || return 1
        if all_communication_nodes_ready; then
            echo "六个 XTDrone 通信节点均已就绪。"
            warmup_deadline=$(( $(current_milliseconds) + OFFBOARD_WARMUP_SECONDS * 1000 ))
            while [ "$(current_milliseconds)" -lt "$warmup_deadline" ]; do
                check_required_processes "通信节点预热" || return 1
                sleep_until_deadline "$warmup_deadline" 100
            done
            check_required_processes "通信节点预热" || return 1
            return 0
        fi
        sleep_until_deadline "$deadline" 200
    done

    echo "错误：XTDrone 通信节点未在 ${COMMUNICATION_TIMEOUT_SECONDS} 秒内就绪。" >&2
    return 1
}

all_cameras_ready() {
    local deadline="$1"
    local id topic now remaining probe_milliseconds probe_timeout

    for id in $(seq 0 5); do
        for topic in \
            "/typhoon_h480_${id}/realsense/depth_camera/color/image_raw" \
            "/typhoon_h480_${id}/realsense/depth_camera/depth/image_raw" \
            "/typhoon_h480_${id}/realsense/depth_camera/color/camera_info"; do
            check_required_processes "Realsense 就绪检查" || return 1
            now="$(current_milliseconds)"
            remaining=$((deadline - now))
            if [ "$remaining" -le 0 ]; then
                return 1
            fi
            probe_milliseconds=3000
            if [ "$remaining" -lt "$probe_milliseconds" ]; then
                probe_milliseconds="$remaining"
            fi
            printf -v probe_timeout '%d.%03ds' \
                "$((probe_milliseconds / 1000))" "$((probe_milliseconds % 1000))"
            timeout "$probe_timeout" rostopic echo -n 1 "$topic" >/dev/null 2>&1 || return 1
        done
    done
}

wait_for_cameras() {
    local deadline
    deadline=$(( $(current_milliseconds) + CAMERA_TIMEOUT_SECONDS * 1000 ))

    echo "等待六组 Realsense 彩色图、深度图和 CameraInfo（最长 ${CAMERA_TIMEOUT_SECONDS} 秒）..."
    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        check_required_processes "Realsense 就绪检查" || return 1
        if all_cameras_ready "$deadline"; then
            check_required_processes "Realsense 就绪检查" || return 1
            echo "六组 Realsense 话题均已就绪。"
            return 0
        fi
        sleep_until_deadline "$deadline" 1000
    done

    echo "错误：Realsense 话题未在 ${CAMERA_TIMEOUT_SECONDS} 秒内全部就绪。" >&2
    return 1
}

start_communication() {
    local python_bin="$1"
    local bridge_script="$2"

    echo "启动六机 XTDrone 通信桥..."
    start_owned_group "XTDrone 通信桥" bash -c '
        pids=()
        for id in $(seq 0 5); do
            "$1" "$2" typhoon_h480 "$id" &
            pids+=("$!")
        done
        wait -n "${pids[@]}"
        status=$?
        for child in "${pids[@]}"; do
            kill -TERM "$child" 2>/dev/null || true
        done
        for child in "${pids[@]}"; do
            wait "$child" 2>/dev/null || true
        done
        if [ "$status" -eq 0 ]; then
            status=1
        fi
        exit "$status"
    ' -- "$python_bin" "$bridge_script" || return 1
    COMMUNICATION_PID="$LAST_STARTED_PID"
}

start_helper() {
    local directory="$1"
    local script_name="$2"
    local title="$3"

    echo "启动${title}..."
    start_owned_group "$title" bash -c 'cd "$1" && exec bash "./$2"' \
        -- "$directory" "$script_name" || return 1
    HELPER_PIDS+=("$LAST_STARTED_PID")
}

wait_for_helpers_survival() {
    local deadline
    deadline=$(( $(current_milliseconds) + HELPER_SURVIVAL_SECONDS * 1000 ))

    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        check_required_processes "辅助节点存活检查" || return 1
        sleep_until_deadline "$deadline" 100
    done
    check_required_processes "辅助节点存活检查" || return 1
    return 0
}

supervise_mission() {
    local mission_status

    while :; do
        check_required_processes "任务运行" || return 1
        if reap_if_exited "$MISSION_PID"; then
            wait_for_owned_process "$MISSION_PID"
            mission_status="$LAST_PROCESS_STATUS"
            check_required_processes "任务退出确认" || return 1
            LAST_PROCESS_STATUS="$mission_status"
            return 0
        fi
        sleep_milliseconds 100
    done
}

main() {
    local mission_status

    validate_configuration || return 1
    setup_logging || return 1

    if [ "$NUM_DRONES" != "6" ]; then
        echo "错误：robocup_zzufly.launch 固定启动 6 架无人机，当前仅支持 'bash 1.sh 6 [任务文件名]'。" >&2
        return 1
    fi

    # --- ROS、PX4 与 Gazebo 环境设置 ---
    require_file "$ROS_SETUP_FILE" "ROS Noetic 环境脚本"
    require_file "$PX4_DIR/Tools/setup_gazebo.bash" "PX4 Gazebo 环境脚本"
    require_file "$PX4_BUILD_DIR/bin/px4" "PX4 SITL 编译产物"
    require_file "$WORKSPACE_DIR/devel/setup.bash" "Catkin 工作空间环境脚本"
    require_file "$WORKSPACE_DIR/scripts/graphics_environment.sh" "Gazebo 图形环境脚本"
    require_file "$SIMULATION_LAUNCH" "六机仿真 launch 文件"
    require_file "$PX4_DIR/Tools/sitl_gazebo/worlds/robocup.world" "RoboCup Gazebo 世界"
    require_file "$XTDRONE_DIR/sitl_config/models/walker/walk_0.dae" "XTDrone 行人模型"
    require_file "$XTDRONE_DIR/communication/multirotor_communication.py" "XTDrone 多旋翼通信脚本"
    require_file "$XTDRONE_PYTHON" "XTDrone Python 环境"
    require_file "$XTDRONE_PYTHONPATH/pyquaternion/__init__.py" "XTDrone Python 依赖"
    require_file "$GAZEBO_MODELS_DIR/cessna/model.sdf" "Gazebo 官方场景模型"
    require_file "$COMPLIANCE_PYTHON" "合规自检 Python 环境"
    require_file "$SUPERVISOR_PYTHON" "进程监督 Python 环境"
    require_file "$PROCESS_SUPERVISOR" "本项目进程监督器"
    require_file "$PREPARE_MODEL" "合规模型生成器"
    require_file "$OFFICIAL_MANIFEST" "官方依赖校验清单"
    require_file "$SENSOR_MOUNT_CONFIG" "Realsense 安装配置"
    require_file "$XTDRONE_DIR/sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf" "XTDrone 官方 Realsense 机型"
    require_file "$XTDRONE_DIR/sitl_config/models/realsense_camera/realsense_camera.sdf" "XTDrone 官方 Realsense 传感器"
    require_file "$WORKSPACE_DIR/src/yolo/multi_yolo_detecting.sh" "YOLO 检测脚本"
    require_file "$WORKSPACE_DIR/src/yolo/multi_solving.sh" "坐标计算脚本"
    require_file "$WORKSPACE_DIR/devel/lib/libActorCollisionsPlugin.so" "Gazebo 行人碰撞插件"
    require_file "$WORKSPACE_DIR/devel/lib/libros_actor_cmd_pose_plugin.so" "Gazebo 行人 ROS 控制插件"

    source "$ROS_SETUP_FILE"
    source "$WORKSPACE_DIR/devel/setup.bash"
    source "$PX4_DIR/Tools/setup_gazebo.bash" "$PX4_DIR" "$PX4_BUILD_DIR"
    source "$WORKSPACE_DIR/scripts/graphics_environment.sh"
    export PYTHONPATH="$XTDRONE_PYTHONPATH${PYTHONPATH:+:$PYTHONPATH}"
    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+${ROS_PACKAGE_PATH}:}$PX4_DIR:$PX4_DIR/Tools/sitl_gazebo"
    export GAZEBO_MODEL_PATH="$XTDRONE_DIR/sitl_config/models:$PX4_DIR/Tools/sitl_gazebo/models:$GAZEBO_MODELS_DIR${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
    export GAZEBO_PLUGIN_PATH="$WORKSPACE_DIR/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
    ensure_graphics_environment || return 1
    echo "Gazebo 图形显示：$DISPLAY"

    if ! RUN_TMP_DIR="$(mktemp -d /tmp/robocup-fly-competition-clean.XXXXXX)"; then
        echo "错误：无法创建 competition-clean 临时目录。" >&2
        return 1
    fi
    GENERATED_MODEL="$RUN_TMP_DIR/typhoon_h480_realsense.sdf"
    echo "执行快速合规自检并生成临时模型..."
    if ! "$COMPLIANCE_PYTHON" "$PREPARE_MODEL" \
        --px4-dir "$PX4_DIR" \
        --xtdrone-dir "$XTDRONE_DIR" \
        --gazebo-models-dir "$GAZEBO_MODELS_DIR" \
        --xtdrone-pythonpath "$XTDRONE_PYTHONPATH" \
        --manifest "$OFFICIAL_MANIFEST" \
        --mount-config "$SENSOR_MOUNT_CONFIG" \
        --output "$GENERATED_MODEL" >/dev/null; then
        echo "错误：快速合规自检或临时模型生成失败。" >&2
        return 1
    fi

    for package in px4 mavlink_sitl_gazebo mavros gazebo_ros look_up; do
        require_ros_package "$package"
    done

    if project_simulator_running; then
        echo "错误：检测到已有 typhoon_h480 MAVROS 仿真正在运行。" >&2
        echo "请先在原来的终端按 Ctrl-C 停止它，再运行本脚本。" >&2
        return 1
    fi

    echo "============================================"
    echo "  启动 PX4/Gazebo 六机仿真和 down_resume 任务"
    echo "  无人机数量: $NUM_DRONES"
    echo "  任务文件:   $MISSION_FILE"
    echo "============================================"

    echo "启动 Gazebo、PX4 SITL 和 MAVROS..."
    start_owned_group "六机仿真" roslaunch "$SIMULATION_LAUNCH" model_file:="$GENERATED_MODEL" || return 1
    SIMULATION_PID="$LAST_STARTED_PID"

    wait_for_simulator || return 1

    start_communication "$XTDRONE_PYTHON" \
        "$XTDRONE_DIR/communication/multirotor_communication.py" || return 1
    wait_for_communication || return 1

    wait_for_cameras || return 1

    start_helper "$WORKSPACE_DIR/src/yolo" "multi_yolo_detecting.sh" "YOLO 检测" || return 1
    start_helper "$WORKSPACE_DIR/src/yolo" "multi_solving.sh" "坐标计算" || return 1
    wait_for_helpers_survival || return 1
    check_required_processes "任务启动前检查" || return 1

    echo "启动 down_resume 任务节点..."
    start_owned_group "任务节点" roslaunch look_up down_resume.launch \
        num_drones:="$NUM_DRONES" mission_filename:="$MISSION_FILE" || return 1
    MISSION_PID="$LAST_STARTED_PID"
    supervise_mission || return 1
    mission_status="$LAST_PROCESS_STATUS"
    if [ "$mission_status" -ne 0 ]; then
        echo "错误：down_resume 任务节点退出（状态 $mission_status）。" >&2
        return "$mission_status"
    fi
    return 0
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    ensure_official_readonly_sandbox "$@" || exit 1
    trap 'handle_exit $?' EXIT
    trap 'on_interrupt 129' HUP
    trap 'on_interrupt 130' INT
    trap 'on_interrupt 143' TERM
    main "$@"
    exit $?
fi
