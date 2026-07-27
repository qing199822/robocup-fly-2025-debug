#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd -P)"
PROJECT_ROOT="$(cd "$WORKSPACE_DIR/.." && pwd -P)"
PX4_DIR="${PX4_DIR:-$PROJECT_ROOT/PX4_Firmware}"
XTDRONE_DIR="${XTDRONE_DIR:-$PROJECT_ROOT/XTDrone}"
GAZEBO_MODELS_DIR="${GAZEBO_MODELS_DIR:-$PROJECT_ROOT/gazebo_models}"
XTDRONE_PYTHONPATH="${XTDRONE_PYTHONPATH:-$PROJECT_ROOT/.xtdrone-python}"
ROS_SETUP_FILE="${ROS_SETUP_FILE:-/opt/ros/noetic/setup.bash}"
PACKAGE_DIR="$WORKSPACE_DIR/src/competition_compliance"
MANIFEST="$PACKAGE_DIR/config/official_manifest.json"
OWNERSHIP="$PACKAGE_DIR/config/ownership.json"
ARTIFACT_DIR="$WORKSPACE_DIR/competition-artifacts"
STATIC_EVIDENCE="$ARTIFACT_DIR/static-compliance.json"
POST_BUILD_EVIDENCE="$ARTIFACT_DIR/post-build-compliance.json"
ROS_LOG_DIR="$WORKSPACE_DIR/logs/verification"
CATKIN_TOPLEVEL="${CATKIN_TOPLEVEL:-/opt/ros/noetic/share/catkin/cmake/toplevel.cmake}"
XTDRONE_STATUS_BEFORE=""

fail() {
    echo "错误：$*" >&2
    exit 1
}

require_command() {
    local command_name="$1"
    local hint="$2"

    command -v "$command_name" >/dev/null 2>&1 \
        || fail "缺少命令 $command_name。$hint"
}

canonical_runtime_directory() {
    local variable_name="$1"
    local directory="$2"

    [ ! -L "$directory" ] \
        || fail "$variable_name 不能是符号链接：$directory"
    [ -d "$directory" ] \
        || fail "找不到 $variable_name 目录：$directory。请按 docs/ENVIRONMENT.md 配置依赖。"
    (cd "$directory" && pwd -P) \
        || fail "无法解析 $variable_name 目录：$directory"
}

prepare_generated_evidence() {
    local evidence="$1"

    if [ -L "$ARTIFACT_DIR" ]; then
        fail "证据目录不能是符号链接：$ARTIFACT_DIR"
    fi
    if [ -e "$ARTIFACT_DIR" ] && [ ! -d "$ARTIFACT_DIR" ]; then
        fail "证据路径不是目录：$ARTIFACT_DIR"
    fi
    if [ ! -e "$ARTIFACT_DIR" ]; then
        mkdir -- "$ARTIFACT_DIR"
    fi
    [ "$(cd "$ARTIFACT_DIR" && pwd -P)" = "$WORKSPACE_DIR/competition-artifacts" ] \
        || fail "证据目录超出团队工作区：$ARTIFACT_DIR"
    if [ -L "$evidence" ]; then
        fail "拒绝清理符号链接证据：$evidence"
    fi
    if [ -e "$evidence" ]; then
        [ -f "$evidence" ] || fail "自产证据路径不是普通文件：$evidence"
        rm -f -- "$evidence"
    fi
}

prepare_ros_log_directory() {
    local directory expected

    for directory in "$WORKSPACE_DIR/logs" "$ROS_LOG_DIR"; do
        if [ -L "$directory" ]; then
            fail "ROS 日志目录不能是符号链接：$directory"
        fi
        if [ -e "$directory" ] && [ ! -d "$directory" ]; then
            fail "ROS 日志路径不是目录：$directory"
        fi
        if [ ! -e "$directory" ]; then
            mkdir -- "$directory"
        fi
    done
    expected="$WORKSPACE_DIR/logs/verification"
    [ "$(cd "$ROS_LOG_DIR" && pwd -P)" = "$expected" ] \
        || fail "ROS 日志目录超出团队工作区：$ROS_LOG_DIR"
}

cleanup_generated_catkin_toplevel_link() {
    local link="$WORKSPACE_DIR/src/CMakeLists.txt"
    local resolved_link resolved_expected

    if [ ! -L "$link" ]; then
        return
    fi
    [ -f "$CATKIN_TOPLEVEL" ] \
        || fail "找不到 Catkin 标准顶层文件：$CATKIN_TOPLEVEL"
    resolved_link="$(realpath -e -- "$link")" \
        || fail "无法解析 Catkin 生成链接：$link"
    resolved_expected="$(realpath -e -- "$CATKIN_TOPLEVEL")" \
        || fail "无法解析 Catkin 标准顶层文件：$CATKIN_TOPLEVEL"
    [ "$resolved_link" = "$resolved_expected" ] \
        || fail "src/CMakeLists.txt 不是 Catkin 标准生成链接，拒绝删除：$link"
    rm -f -- "$link"
}

run_compliance_verifier() {
    local evidence="$1"

    python3 "$PACKAGE_DIR/scripts/verify_full.py" \
        --root "$WORKSPACE_DIR" \
        --px4-dir "$PX4_DIR" \
        --xtdrone-dir "$XTDRONE_DIR" \
        --gazebo-models-dir "$GAZEBO_MODELS_DIR" \
        --xtdrone-pythonpath "$XTDRONE_PYTHONPATH" \
        --manifest "$MANIFEST" \
        --ownership "$OWNERSHIP" \
        --evidence "$evidence"
}

snapshot_xtdrone_status() {
    if command -v git >/dev/null 2>&1 \
        && git -C "$XTDRONE_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        git -C "$XTDRONE_DIR" status --porcelain=v1 --untracked-files=all
    fi
}

[ -r "$ROS_SETUP_FILE" ] \
    || fail "找不到可读取的 ROS 环境脚本：$ROS_SETUP_FILE。请安装 ROS Noetic 或设置 ROS_SETUP_FILE。"
[ -f "$PACKAGE_DIR/scripts/verify_full.py" ] \
    || fail "找不到规范合规验证器：$PACKAGE_DIR/scripts/verify_full.py"
[ -f "$MANIFEST" ] || fail "找不到规范官方清单：$MANIFEST"
[ -f "$OWNERSHIP" ] || fail "找不到规范所有权清单：$OWNERSHIP"

# shellcheck disable=SC1090
set +u
if ! source "$ROS_SETUP_FILE"; then
    set -u
    fail "加载 ROS 环境失败：$ROS_SETUP_FILE"
fi
set -u
require_command python3 "请安装 Python 3。"
require_command catkin_make "请安装 ROS Noetic catkin。"
require_command catkin_test_results "请安装 ROS Noetic catkin 测试工具。"
require_command realpath "请安装 coreutils。"

PX4_DIR="$(canonical_runtime_directory PX4_DIR "$PX4_DIR")"
XTDRONE_DIR="$(canonical_runtime_directory XTDRONE_DIR "$XTDRONE_DIR")"
GAZEBO_MODELS_DIR="$(canonical_runtime_directory GAZEBO_MODELS_DIR "$GAZEBO_MODELS_DIR")"
XTDRONE_PYTHONPATH="$(canonical_runtime_directory XTDRONE_PYTHONPATH "$XTDRONE_PYTHONPATH")"
export PX4_DIR XTDRONE_DIR GAZEBO_MODELS_DIR XTDRONE_PYTHONPATH

export PYTHONPATH="$PACKAGE_DIR/src:$XTDRONE_PYTHONPATH${PYTHONPATH:+:$PYTHONPATH}"
cd "$WORKSPACE_DIR"

prepare_generated_evidence "$STATIC_EVIDENCE"
prepare_generated_evidence "$POST_BUILD_EVIDENCE"
cleanup_generated_catkin_toplevel_link
XTDRONE_STATUS_BEFORE="$(snapshot_xtdrone_status)"

run_compliance_verifier "$STATIC_EVIDENCE"
prepare_ros_log_directory
export ROS_LOG_DIR
python3 -m unittest discover -s "$WORKSPACE_DIR/tests" -p 'test_*.py'
catkin_make -DCMAKE_BUILD_TYPE=Release
bash "$SCRIPT_DIR/build_xtdrone_actor_collisions.sh"
catkin_make run_tests
catkin_test_results
cleanup_generated_catkin_toplevel_link
run_compliance_verifier "$POST_BUILD_EVIDENCE"

XTDRONE_STATUS_AFTER="$(snapshot_xtdrone_status)"
if [ "$XTDRONE_STATUS_BEFORE" != "$XTDRONE_STATUS_AFTER" ]; then
    echo "错误：验证期间 XTDrone 外部目录状态发生变化。" >&2
    echo "验证前：" >&2
    printf '%s\n' "$XTDRONE_STATUS_BEFORE" >&2
    echo "验证后：" >&2
    printf '%s\n' "$XTDRONE_STATUS_AFTER" >&2
    exit 1
fi

echo "完整验证通过：静态与构建后合规证据均已生成。"
