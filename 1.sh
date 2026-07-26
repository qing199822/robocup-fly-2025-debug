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
HELPER_PIDS=()
OWNED_PIDS=()
OWNED_PGIDS=()
declare -A OWNED_NAMES=()
declare -A OWNED_REAPED=()
declare -A OWNED_STATUSES=()
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
    local state

    kill -0 "$pid" 2>/dev/null || return 1
    state="$(ps -o stat= -p "$pid" 2>/dev/null)" || return 1
    [[ "$state" != Z* ]]
}

group_is_running() {
    local pgid="$1"

    ps -eo pgid=,stat= 2>/dev/null | awk -v target="$pgid" '
        $1 == target && $2 !~ /^Z/ { found = 1 }
        END { exit(found ? 0 : 1) }
    '
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
    fi
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
    if ! exec 1>"$LOGGER_FIFO" 2>&1; then
        exec 1>&3 2>&4
        stop_logger_bounded "$LOGGER_PID"
        wait "$LOGGER_PID" 2>/dev/null || true
        LOGGER_STARTED=false
        cleanup_logger_files
        return 1
    fi
    LOGGER_REDIRECTED=true
    if ! process_is_running "$LOGGER_PID"; then
        exec 1>&3 2>&4
        LOGGER_REDIRECTED=false
        wait "$LOGGER_PID" 2>/dev/null || true
        LOGGER_STARTED=false
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

    if "$LOGGER_REDIRECTED"; then
        exec 1>&3 2>&4
        LOGGER_REDIRECTED=false
    fi

    if "$LOGGER_STARTED"; then
        deadline=$(( $(current_milliseconds) + CLEANUP_GRACE_SECONDS * 1000 ))
        while process_is_running "$LOGGER_PID" && [ "$(current_milliseconds)" -lt "$deadline" ]; do
            sleep_milliseconds 50
        done
        if process_is_running "$LOGGER_PID"; then
            echo "错误：启动日志进程未在期限内结束，正在强制停止。" >&2
            stop_logger_bounded "$LOGGER_PID"
        fi
        if wait "$LOGGER_PID"; then
            logger_status=0
        else
            logger_status=$?
        fi
        LOGGER_STARTED=false
    fi

    cleanup_logger_files
    if "$LOGGER_FDS_SAVED"; then
        exec 3>&- 4>&-
        LOGGER_FDS_SAVED=false
    fi

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

    OWNED_PIDS+=("$pid")
    OWNED_PGIDS+=("$pid")
    OWNED_NAMES["$pid"]="$name"
    OWNED_REAPED["$pid"]=false
    LAST_STARTED_PID="$pid"
}

start_owned_group() {
    local name="$1"
    shift

    setsid "$@" &
    register_owned_process "$!" "$name"
}

reap_if_exited() {
    local pid="$1"
    local status

    LAST_PROCESS_STATUS=""
    if [ "${OWNED_REAPED[$pid]:-false}" = true ]; then
        LAST_PROCESS_STATUS="${OWNED_STATUSES[$pid]}"
        return 0
    fi
    if process_is_running "$pid"; then
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

cleanup() {
    local index pid deadline active

    if "$CLEANUP_DONE"; then
        return "$CLEANUP_STATUS"
    fi
    CLEANUP_DONE=true
    CLEANUP_STATUS=0

    echo
    echo "正在停止本脚本启动的节点..."

    for ((index=${#OWNED_PGIDS[@]} - 1; index >= 0; index--)); do
        pid="${OWNED_PGIDS[$index]}"
        if group_is_running "$pid"; then
            kill -TERM -- "-$pid" 2>/dev/null || true
        fi
    done

    deadline=$(( $(current_milliseconds) + CLEANUP_GRACE_SECONDS * 1000 ))
    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        active=false
        for pid in "${OWNED_PGIDS[@]}"; do
            if group_is_running "$pid"; then
                active=true
                break
            fi
        done
        if ! "$active"; then
            break
        fi
        sleep_milliseconds 50
    done

    for pid in "${OWNED_PGIDS[@]}"; do
        if group_is_running "$pid"; then
            echo "进程组 ${OWNED_NAMES[$pid]} 未响应 TERM，发送 KILL。" >&2
            kill -KILL -- "-$pid" 2>/dev/null || true
        fi
    done

    deadline=$(( $(current_milliseconds) + 1000 ))
    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        active=false
        for pid in "${OWNED_PGIDS[@]}"; do
            if group_is_running "$pid"; then
                active=true
                break
            fi
        done
        if ! "$active"; then
            break
        fi
        sleep_milliseconds 50
    done

    for pid in "${OWNED_PIDS[@]}"; do
        if ! reap_if_exited "$pid"; then
            echo "错误：无法在清理期限内回收 ${OWNED_NAMES[$pid]}（PID $pid）。" >&2
            CLEANUP_STATUS=1
        fi
    done

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

    trap - EXIT INT TERM
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
    exit 130
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
    deadline=$(( $(current_milliseconds) + READY_TIMEOUT_SECONDS * 1000 ))

    echo "等待 Gazebo、PX4 和六个 MAVROS 实例连接（最长 ${READY_TIMEOUT_SECONDS} 秒）..."
    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        if report_early_exit "$SIMULATION_PID" "六机仿真"; then
            return 1
        fi
        if all_vehicles_ready; then
            if report_early_exit "$SIMULATION_PID" "六机仿真"; then
                return 1
            fi
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
        if report_early_exit "$COMMUNICATION_PID" "XTDrone 通信桥"; then
            return 1
        fi
        if all_communication_nodes_ready; then
            echo "六个 XTDrone 通信节点均已就绪。"
            warmup_deadline=$(( $(current_milliseconds) + OFFBOARD_WARMUP_SECONDS * 1000 ))
            while [ "$(current_milliseconds)" -lt "$warmup_deadline" ]; do
                if report_early_exit "$COMMUNICATION_PID" "XTDrone 通信桥"; then
                    return 1
                fi
                sleep_until_deadline "$warmup_deadline" 100
            done
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
        if all_cameras_ready "$deadline"; then
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
    setsid bash -c '
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
    ' -- "$python_bin" "$bridge_script" &
    register_owned_process "$!" "XTDrone 通信桥"
    COMMUNICATION_PID="$LAST_STARTED_PID"
    HELPER_PIDS+=("$COMMUNICATION_PID")
}

start_helper() {
    local directory="$1"
    local script_name="$2"
    local title="$3"

    echo "启动${title}..."
    start_owned_group "$title" bash -c 'cd "$1" && exec bash "./$2"' -- "$directory" "$script_name"
    HELPER_PIDS+=("$LAST_STARTED_PID")
}

wait_for_helpers_survival() {
    local deadline pid status
    deadline=$(( $(current_milliseconds) + HELPER_SURVIVAL_SECONDS * 1000 ))

    while [ "$(current_milliseconds)" -lt "$deadline" ]; do
        if report_early_exit "$COMMUNICATION_PID" "XTDrone 通信桥"; then
            return 1
        fi
        for pid in "${HELPER_PIDS[@]}"; do
            if [ "$pid" = "$COMMUNICATION_PID" ]; then
                continue
            fi
            if reap_if_exited "$pid"; then
                status="$LAST_PROCESS_STATUS"
                if [ "$status" -ne 0 ] || ! group_is_running "$pid"; then
                    echo "错误：${OWNED_NAMES[$pid]}在任务启动前退出（状态 $status）。" >&2
                    return 1
                fi
            fi
        done
        sleep_until_deadline "$deadline" 100
    done
    return 0
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
    export GAZEBO_MODEL_PATH="$PX4_DIR/Tools/sitl_gazebo/models:$XTDRONE_DIR/sitl_config/models:$GAZEBO_MODELS_DIR${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
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
    start_owned_group "六机仿真" roslaunch "$SIMULATION_LAUNCH" model_file:="$GENERATED_MODEL"
    SIMULATION_PID="$LAST_STARTED_PID"

    wait_for_simulator || return 1

    start_communication "$XTDRONE_PYTHON" "$XTDRONE_DIR/communication/multirotor_communication.py"
    wait_for_communication || return 1

    wait_for_cameras || return 1

    start_helper "$WORKSPACE_DIR/src/yolo" "multi_yolo_detecting.sh" "YOLO 检测"
    start_helper "$WORKSPACE_DIR/src/yolo" "multi_solving.sh" "坐标计算"
    wait_for_helpers_survival || return 1

    echo "启动 down_resume 任务节点..."
    start_owned_group "任务节点" roslaunch look_up down_resume.launch \
        num_drones:="$NUM_DRONES" mission_filename:="$MISSION_FILE"
    MISSION_PID="$LAST_STARTED_PID"
    wait_for_owned_process "$MISSION_PID"
    mission_status="$LAST_PROCESS_STATUS"
    if [ "$mission_status" -ne 0 ]; then
        echo "错误：down_resume 任务节点退出（状态 $mission_status）。" >&2
        return "$mission_status"
    fi
    return 0
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    trap 'handle_exit $?' EXIT
    trap on_interrupt INT TERM
    main "$@"
    exit $?
fi
