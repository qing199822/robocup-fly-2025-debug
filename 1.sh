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
READY_TIMEOUT_SECONDS=90
COMMUNICATION_TIMEOUT_SECONDS=20

NUM_DRONES=${1:-6}
MISSION_FILE=${2:-mission_down.json}
SIMULATION_PID=""
HELPER_PIDS=()
CLEANUP_DONE=false

cleanup() {
    if "$CLEANUP_DONE"; then
        return
    fi
    CLEANUP_DONE=true

    echo
    echo "正在停止本脚本启动的节点..."

    for pid in "${HELPER_PIDS[@]}"; do
        # Helper scripts spawn several Python processes then exit. They remain
        # in the setsid-created process group whose ID is the saved PID.
        kill -INT -- "-$pid" 2>/dev/null || true
    done

    if [ -n "$SIMULATION_PID" ] && kill -0 "$SIMULATION_PID" 2>/dev/null; then
        kill -INT "$SIMULATION_PID" 2>/dev/null || true
    fi

    if [ "${MODEL_LINK_CREATED:-false}" = true ]; then
        rm -f "$MODEL_LINK"
    fi

    for pid in "${HELPER_PIDS[@]}" "$SIMULATION_PID"; do
        if [ -n "$pid" ]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
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
require_file "$PX4_DIR/launch/single_vehicle_spawn_xtd.launch" "XTDrone 单机启动文件"
require_file "$XTDRONE_DIR/sitl_config/models/walker/walk_0.dae" "XTDrone 行人模型"
require_file "$XTDRONE_DIR/communication/multirotor_communication.py" "XTDrone 多旋翼通信脚本"
require_file "$XTDRONE_PYTHON" "XTDrone Python 环境"
require_file "$XTDRONE_PYTHONPATH/pyquaternion/__init__.py" "XTDrone Python 依赖"
require_file "$GAZEBO_MODELS_DIR/cessna/model.sdf" "Gazebo 官方场景模型"
require_file "$WORKSPACE_DIR/typhoon_h480_zzufly/typhoon_h480_zzufly.sdf" "自定义 Typhoon 模型文件"
require_file "$WORKSPACE_DIR/src/gimbal/multi_gimbal_control.sh" "云台控制脚本"
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
# single_vehicle_spawn_xtd.launch 从 PX4 的模型目录读取 SDF；本项目的自定义
# Typhoon 模型保存在工作空间根目录，因此为本次启动临时链接到该目录。
MODEL_LINK="$PX4_DIR/Tools/sitl_gazebo/models/typhoon_h480_zzufly"
MODEL_LINK_CREATED=false
if [ -L "$MODEL_LINK" ] && [ ! -e "$MODEL_LINK" ]; then
    echo "错误：PX4 自定义模型路径是损坏的符号链接：$MODEL_LINK" >&2
    exit 1
elif [ ! -e "$MODEL_LINK" ]; then
    if [ ! -w "$(dirname "$MODEL_LINK")" ]; then
        echo "错误：无权创建 PX4 自定义模型链接：$(dirname "$MODEL_LINK")" >&2
        exit 1
    fi
    ln -s "$WORKSPACE_DIR/typhoon_h480_zzufly" "$MODEL_LINK"
    MODEL_LINK_CREATED=true
elif [ ! -d "$MODEL_LINK" ] || [ ! -f "$MODEL_LINK/typhoon_h480_zzufly.sdf" ]; then
    echo "错误：PX4 自定义模型路径无效或缺少 typhoon_h480_zzufly.sdf：$MODEL_LINK" >&2
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
roslaunch "$SIMULATION_LAUNCH" &
SIMULATION_PID=$!

if ! wait_for_simulator; then
    exit 1
fi

start_communication "$XTDRONE_PYTHON" "$XTDRONE_DIR/communication/multirotor_communication.py"
if ! wait_for_communication; then
    exit 1
fi

start_helper "$WORKSPACE_DIR/src/gimbal" "multi_gimbal_control.sh" "云台控制"
start_helper "$WORKSPACE_DIR/src/yolo" "multi_yolo_detecting.sh" "YOLO 检测"
start_helper "$WORKSPACE_DIR/src/yolo" "multi_solving.sh" "坐标计算"

echo "启动 down_resume 任务节点..."
roslaunch look_up down_resume.launch num_drones:="$NUM_DRONES" mission_filename:="$MISSION_FILE"
