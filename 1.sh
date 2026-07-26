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
XTDRONE_PYTHON="${XTDRONE_PYTHON:-/usr/bin/python3}"
XTDRONE_PYTHONPATH="${XTDRONE_PYTHONPATH:-$PROJECT_ROOT/.xtdrone-python}"
COMPLIANCE_PACKAGE_DIR="$WORKSPACE_DIR/src/competition_compliance"
COMPLIANCE_PYTHON="${COMPLIANCE_PYTHON:-/usr/bin/python3}"
PREPARE_MODEL="$COMPLIANCE_PACKAGE_DIR/scripts/prepare_model.py"
OFFICIAL_MANIFEST="$COMPLIANCE_PACKAGE_DIR/config/official_manifest.json"
SENSOR_MOUNT_CONFIG="$COMPLIANCE_PACKAGE_DIR/config/sensor_mount.yaml"
RUN_TMP_DIR=""
GENERATED_MODEL=""
READY_TIMEOUT_SECONDS="${READY_TIMEOUT_SECONDS:-180}"
COMMUNICATION_TIMEOUT_SECONDS=20
CAMERA_TIMEOUT_SECONDS="${CAMERA_TIMEOUT_SECONDS:-60}"
LOG_DIR="$WORKSPACE_DIR/logs/competition-clean"
RUN_LOG="$LOG_DIR/launch-$(date +%Y%m%d-%H%M%S).log"

mkdir -p "$LOG_DIR"
exec > >(tee -a "$RUN_LOG") 2>&1
echo "本次完整启动日志：$RUN_LOG"

NUM_DRONES=${1:-6}
MISSION_FILE=${2:-mission_down.json}
SIMULATION_PID=""
MISSION_PID=""
HELPER_PIDS=()
CLEANUP_DONE=false

cleanup() {
    if "$CLEANUP_DONE"; then
        return
    fi
    CLEANUP_DONE=true

    echo
    echo "正在停止本脚本启动的节点..."

    if [ -n "$MISSION_PID" ] && kill -0 "$MISSION_PID" 2>/dev/null; then
        kill -TERM "$MISSION_PID" 2>/dev/null || true
    fi

    for pid in "${HELPER_PIDS[@]}"; do
        # Helper scripts spawn several Python processes then exit. They remain
        # in the setsid-created process group whose ID is the saved PID.
        kill -TERM -- "-$pid" 2>/dev/null || true
    done

    if [ -n "$SIMULATION_PID" ] && kill -0 "$SIMULATION_PID" 2>/dev/null; then
        kill -TERM "$SIMULATION_PID" 2>/dev/null || true
    fi

    for pid in "$MISSION_PID" "${HELPER_PIDS[@]}" "$SIMULATION_PID"; do
        if [ -n "$pid" ]; then
            wait "$pid" 2>/dev/null || true
        fi
    done

    if [ -n "$RUN_TMP_DIR" ]; then
        case "$RUN_TMP_DIR" in
        /tmp/robocup-fly-competition-clean.*)
            rm -rf -- "$RUN_TMP_DIR"
            ;;
        *)
            echo "拒绝清理非 competition-clean 临时目录：$RUN_TMP_DIR" >&2
            ;;
        esac
    fi
}

on_interrupt() {
    cleanup
    exit 130
}

trap cleanup EXIT
trap on_interrupt INT TERM

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
    local topic_list
    local id state_topic pose_topic

    rosservice list 2>/dev/null | grep -qx '/gazebo/get_world_properties' || return 1
    topic_list="$(rostopic list 2>/dev/null)" || return 1

    for id in $(seq 0 5); do
        state_topic="/typhoon_h480_${id}/mavros/state"
        pose_topic="/typhoon_h480_${id}/mavros/local_position/pose"

        grep -qx "$pose_topic" <<< "$topic_list" || return 1
        timeout 2s rostopic echo -n 1 "$state_topic" 2>/dev/null | grep -q 'connected: True' || return 1
    done
}

wait_for_simulator() {
    local deadline missing
    deadline=$(( $(date +%s) + READY_TIMEOUT_SECONDS ))

    echo "等待 Gazebo、PX4 和六个 MAVROS 实例连接（最长 ${READY_TIMEOUT_SECONDS} 秒）..."
    while [ "$(date +%s)" -lt "$deadline" ]; do
        if all_vehicles_ready; then
            echo "六架无人机均已连接，开始启动任务节点。"
            return 0
        fi
        sleep 2
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
    local deadline
    deadline=$(( $(date +%s) + COMMUNICATION_TIMEOUT_SECONDS ))

    echo "等待六个 XTDrone 通信节点就绪（最长 ${COMMUNICATION_TIMEOUT_SECONDS} 秒）..."
    while [ "$(date +%s)" -lt "$deadline" ]; do
        if all_communication_nodes_ready; then
            echo "六个 XTDrone 通信节点均已就绪。"
            # PX4 requires a setpoint stream before accepting OFFBOARD mode.
            sleep 2
            return 0
        fi
        sleep 1
    done

    echo "错误：XTDrone 通信节点未在 ${COMMUNICATION_TIMEOUT_SECONDS} 秒内就绪。" >&2
    return 1
}

all_cameras_ready() {
    local id topic

    for id in $(seq 0 5); do
        for topic in \
            "/typhoon_h480_${id}/realsense/depth_camera/color/image_raw" \
            "/typhoon_h480_${id}/realsense/depth_camera/depth/image_raw" \
            "/typhoon_h480_${id}/realsense/depth_camera/color/camera_info"; do
            timeout 3s rostopic echo -n 1 "$topic" >/dev/null 2>&1 || return 1
        done
    done
}

wait_for_cameras() {
    local deadline
    deadline=$(( $(date +%s) + CAMERA_TIMEOUT_SECONDS ))

    echo "等待六组 Realsense 彩色图、深度图和 CameraInfo（最长 ${CAMERA_TIMEOUT_SECONDS} 秒）..."
    while [ "$(date +%s)" -lt "$deadline" ]; do
        if all_cameras_ready; then
            echo "六组 Realsense 话题均已就绪。"
            return 0
        fi
        sleep 1
    done

    echo "错误：Realsense 话题未在 ${CAMERA_TIMEOUT_SECONDS} 秒内全部就绪。" >&2
    return 1
}

start_communication() {
    local python_bin="$1"
    local bridge_script="$2"

    echo "启动六机 XTDrone 通信桥..."
    setsid bash -c '
        for id in $(seq 0 5); do
            "$1" "$2" typhoon_h480 "$id" &
        done
        wait
    ' -- "$python_bin" "$bridge_script" &
    HELPER_PIDS+=("$!")
}

start_helper() {
    local directory="$1"
    local script_name="$2"
    local title="$3"

    echo "启动${title}..."
    setsid bash -c 'cd "$1" && exec bash "./$2"' -- "$directory" "$script_name" &
    HELPER_PIDS+=("$!")
}

if [ "$NUM_DRONES" != "6" ]; then
    echo "错误：robocup_zzufly.launch 固定启动 6 架无人机，当前仅支持 'bash 1.sh 6 [任务文件名]'。" >&2
    exit 1
fi

# --- ROS、PX4 与 Gazebo 环境设置 ---
require_file /opt/ros/noetic/setup.bash "ROS Noetic 环境脚本"
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
require_file "$PREPARE_MODEL" "合规模型生成器"
require_file "$OFFICIAL_MANIFEST" "官方依赖校验清单"
require_file "$SENSOR_MOUNT_CONFIG" "Realsense 安装配置"
require_file "$XTDRONE_DIR/sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf" "XTDrone 官方 Realsense 机型"
require_file "$XTDRONE_DIR/sitl_config/models/realsense_camera/realsense_camera.sdf" "XTDrone 官方 Realsense 传感器"
require_file "$WORKSPACE_DIR/src/yolo/multi_yolo_detecting.sh" "YOLO 检测脚本"
require_file "$WORKSPACE_DIR/src/yolo/multi_solving.sh" "坐标计算脚本"
require_file "$WORKSPACE_DIR/devel/lib/libActorCollisionsPlugin.so" "Gazebo 行人碰撞插件"
require_file "$WORKSPACE_DIR/devel/lib/libros_actor_cmd_pose_plugin.so" "Gazebo 行人 ROS 控制插件"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
source "$PX4_DIR/Tools/setup_gazebo.bash" "$PX4_DIR" "$PX4_BUILD_DIR"
source "$WORKSPACE_DIR/scripts/graphics_environment.sh"
export PYTHONPATH="$XTDRONE_PYTHONPATH${PYTHONPATH:+:$PYTHONPATH}"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+${ROS_PACKAGE_PATH}:}$PX4_DIR:$PX4_DIR/Tools/sitl_gazebo"
export GAZEBO_MODEL_PATH="$PX4_DIR/Tools/sitl_gazebo/models:$XTDRONE_DIR/sitl_config/models:$GAZEBO_MODELS_DIR${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$WORKSPACE_DIR/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
ensure_graphics_environment || exit 1
echo "Gazebo 图形显示：$DISPLAY"

if ! RUN_TMP_DIR="$(mktemp -d /tmp/robocup-fly-competition-clean.XXXXXX)"; then
    echo "错误：无法创建 competition-clean 临时目录。" >&2
    exit 1
fi
GENERATED_MODEL="$RUN_TMP_DIR/typhoon_h480_realsense.sdf"
echo "执行快速合规自检并生成临时模型..."
if ! "$COMPLIANCE_PYTHON" "$PREPARE_MODEL" \
    --px4-dir "$PX4_DIR" \
    --xtdrone-dir "$XTDRONE_DIR" \
    --manifest "$OFFICIAL_MANIFEST" \
    --mount-config "$SENSOR_MOUNT_CONFIG" \
    --output "$GENERATED_MODEL" >/dev/null; then
    echo "错误：快速合规自检或临时模型生成失败。" >&2
    exit 1
fi

for package in px4 mavlink_sitl_gazebo mavros gazebo_ros look_up; do
    require_ros_package "$package"
done

if project_simulator_running; then
    echo "错误：检测到已有 typhoon_h480 MAVROS 仿真正在运行。" >&2
    echo "请先在原来的终端按 Ctrl-C 停止它，再运行本脚本。" >&2
    exit 1
fi

echo "============================================"
echo "  启动 PX4/Gazebo 六机仿真和 down_resume 任务"
echo "  无人机数量: $NUM_DRONES"
echo "  任务文件:   $MISSION_FILE"
echo "============================================"

echo "启动 Gazebo、PX4 SITL 和 MAVROS..."
roslaunch "$SIMULATION_LAUNCH" model_file:="$GENERATED_MODEL" &
SIMULATION_PID=$!

if ! wait_for_simulator; then
    exit 1
fi

start_communication "$XTDRONE_PYTHON" "$XTDRONE_DIR/communication/multirotor_communication.py"
if ! wait_for_communication; then
    exit 1
fi

if ! wait_for_cameras; then
    exit 1
fi

start_helper "$WORKSPACE_DIR/src/yolo" "multi_yolo_detecting.sh" "YOLO 检测"
start_helper "$WORKSPACE_DIR/src/yolo" "multi_solving.sh" "坐标计算"

echo "启动 down_resume 任务节点..."
roslaunch look_up down_resume.launch num_drones:="$NUM_DRONES" mission_filename:="$MISSION_FILE" &
MISSION_PID=$!
wait "$MISSION_PID"
