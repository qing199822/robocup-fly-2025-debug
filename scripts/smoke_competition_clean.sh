#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd -P)"
ROS_SETUP_FILE="${ROS_SETUP_FILE:-/opt/ros/noetic/setup.bash}"
WORKSPACE_SETUP_FILE="${WORKSPACE_SETUP_FILE:-$WORKSPACE_DIR/devel/setup.bash}"
LOG_DIR="${LOG_DIR:-$WORKSPACE_DIR/logs/competition-clean}"
SMOKE_TIMEOUT_SECONDS="${SMOKE_TIMEOUT_SECONDS:-5}"
REPORT=""

fail() {
    echo "错误：$*" >&2
    exit 1
}

require_command() {
    local command_name="$1"
    command -v "$command_name" >/dev/null 2>&1 \
        || fail "缺少命令 $command_name，请先安装完整 ROS Noetic 运行环境。"
}

log_line() {
    printf '%s\n' "$*" | tee -a "$REPORT"
}

check_message() {
    local topic="$1"

    if ! timeout "${SMOKE_TIMEOUT_SECONDS}s" \
        rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
        log_line "FAIL topic $topic" >&2
        return 1
    fi
    log_line "PASS topic $topic"
}

check_node() {
    local node="$1"

    if ! timeout "${SMOKE_TIMEOUT_SECONDS}s" rosnode list 2>/dev/null \
        | grep -Fx -- "$node" >/dev/null; then
        log_line "FAIL node $node" >&2
        return 1
    fi
    log_line "PASS node $node"
}

check_true_boolean() {
    local topic="$1" output

    if ! output="$(
        timeout "${SMOKE_TIMEOUT_SECONDS}s" \
            rostopic echo -n 1 "$topic" 2>/dev/null
    )" || ! grep -Eq '^data:[[:space:]]+true$' <<<"$output"; then
        log_line "FAIL takeoff gate $topic" >&2
        return 1
    fi
    log_line "PASS takeoff gate $topic"
}

check_sensor_tf() {
    local output status

    set +e
    output="$(
        timeout "${SMOKE_TIMEOUT_SECONDS}s" \
            rosrun tf tf_echo base_link depth_camera_base 2>&1
    )"
    status=$?
    set -e
    if { [ "$status" -ne 0 ] && [ "$status" -ne 124 ]; } \
        || ! grep -Eq '^[[:space:]]*(-[[:space:]]*)?Translation[[:space:]]*:' \
            <<<"$output"; then
        log_line "FAIL TF base_link -> depth_camera_base" >&2
        return 1
    fi
    log_line "PASS TF base_link -> depth_camera_base"
}

[[ "$SMOKE_TIMEOUT_SECONDS" =~ ^[1-9][0-9]*$ ]] \
    || fail "SMOKE_TIMEOUT_SECONDS 必须是正整数，当前值：$SMOKE_TIMEOUT_SECONDS"
[ -r "$ROS_SETUP_FILE" ] \
    || fail "找不到可读取的 ROS 环境脚本：$ROS_SETUP_FILE"
[ -r "$WORKSPACE_SETUP_FILE" ] \
    || fail "找不到工作区环境：$WORKSPACE_SETUP_FILE。请先运行完整验证构建。"

# shellcheck disable=SC1090
set +u
if ! source "$ROS_SETUP_FILE"; then
    set -u
    fail "加载 ROS 环境失败：$ROS_SETUP_FILE"
fi
# shellcheck disable=SC1090
if ! source "$WORKSPACE_SETUP_FILE"; then
    set -u
    fail "加载工作区环境失败：$WORKSPACE_SETUP_FILE"
fi
set -u
for command_name in python3 timeout rostopic rosnode rosrun grep tee mktemp realpath; do
    require_command "$command_name"
done

LOG_DIR_LEXICAL="$(realpath -ms -- "$LOG_DIR")"
LOG_DIR_PHYSICAL="$(realpath -m -- "$LOG_DIR")"
if [ "$LOG_DIR_LEXICAL" != "$LOG_DIR_PHYSICAL" ]; then
    fail "smoke 日志路径包含符号链接，拒绝写入：$LOG_DIR"
fi
LOG_DIR="$LOG_DIR_LEXICAL"
if [ -L "$LOG_DIR" ]; then
    fail "smoke 日志目录不能是符号链接：$LOG_DIR"
fi
if [ -e "$LOG_DIR" ] && [ ! -d "$LOG_DIR" ]; then
    fail "smoke 日志路径不是目录：$LOG_DIR"
fi
mkdir -p -- "$LOG_DIR"
REPORT="$(mktemp -- "$LOG_DIR/smoke-$(date +%Y%m%d-%H%M%S).XXXXXX.log")"
chmod 0644 "$REPORT"

echo "smoke 报告：$REPORT"
for id in $(seq 0 5); do
    check_message "/typhoon_h480_${id}/mavros/state"
    check_message "/typhoon_h480_${id}/mavros/local_position/pose"
    check_message "/typhoon_h480_${id}/realsense/depth_camera/color/image_raw"
    check_message "/typhoon_h480_${id}/realsense/depth_camera/depth/image_raw"
    check_message "/typhoon_h480_${id}/realsense/depth_camera/color/camera_info"
    check_message "/typhoon_h480_${id}/safety/status"
    check_node "/typhoon_h480_${id}_communication"
    check_node "/typhoon_h480_${id}/safety_filter"
done

check_true_boolean "/swarm/takeoff_complete"

if ! python3 "$WORKSPACE_DIR/scripts/check_final_control_publishers.py" \
    --count 6 --vehicle-type typhoon_h480 | tee -a "$REPORT"; then
    log_line "FAIL final control publisher ownership" >&2
    exit 1
fi

# The clean launcher publishes one global fixed sensor mount transform.
check_sensor_tf
log_line "PASS competition-clean six-vehicle smoke"
