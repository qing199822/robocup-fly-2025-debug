# 2025 RoboCup 多旋翼无人机集群协同搜索仿真

## 分支用途

`competition-clean` 是参赛候选分支。PX4 1.11、XTDrone、Gazebo 11 及其官方模型是外部只读依赖；本仓库只包含队伍上层 ROS 算法、允许的 launch/world 配置、合规检查工具，以及经审计的 Realsense 安装位姿。第三方消息和 Actor 插件的来源、版本和许可证见 [docs/THIRD_PARTY.md](docs/THIRD_PARTY.md)。

无人机基线是 XTDrone 的 `typhoon_h480_realsense`。传感器成像、量程和关节定义保持官方原值；只有 [src/competition_compliance/config/sensor_mount.yaml](src/competition_compliance/config/sensor_mount.yaml) 中的安装位置和角度可调整，最终物理合理性仍须队伍和裁判检查。合规边界和哈希证据见 [docs/COMPLIANCE.md](docs/COMPLIANCE.md)。

## 环境

已验证组合：Ubuntu 20.04.6、ROS Noetic 1.5.0、Gazebo 11.15.1、PX4 `v1.11.0-beta1`、MAVROS 1.20.1、Python 3.8.10、NVIDIA 535.230.02、RTX 3060 12GB、PyTorch 2.1.2（CUDA 12.1 wheel）、Ultralytics 8.3.40。PX4 1.13 不在此分支支持范围内。安装顺序和外部目录见 [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)。

## 准备与构建

默认目录：

```text
~/robocup_fly/
|-- 2025_ZZU_FLY/       # 本仓库
|-- PX4_Firmware/       # 教程提供的 PX4 1.11 完整归档
|-- XTDrone/            # XTDrone，提交 8e88116
|-- gazebo_models/      # Gazebo 模型库
|-- .venv-yolo/         # YOLO Python 环境
`-- .xtdrone-python/    # XTDrone Python 依赖
```

```bash
cd ~/robocup_fly/2025_ZZU_FLY
export PX4_DIR=${PX4_DIR:-$HOME/robocup_fly/PX4_Firmware}
export XTDRONE_DIR=${XTDRONE_DIR:-$HOME/robocup_fly/XTDrone}
export GAZEBO_MODELS_DIR=${GAZEBO_MODELS_DIR:-$HOME/robocup_fly/gazebo_models}
export XTDRONE_PYTHONPATH=${XTDRONE_PYTHONPATH:-$HOME/robocup_fly/.xtdrone-python}
export YOLO_PYTHON=${YOLO_PYTHON:-$HOME/robocup_fly/.venv-yolo/bin/python}
export YOLO_CONFIG_DIR=${YOLO_CONFIG_DIR:-$HOME/robocup_fly/.ultralytics}
source /opt/ros/noetic/setup.bash
catkin_init_workspace src
catkin_make -DCMAKE_BUILD_TYPE=Release
bash scripts/build_xtdrone_actor_collisions.sh
```

PX4 必须先由教程提供的完整包构建：`cd "$PX4_DIR" && make px4_sitl_default gazebo`。不要用 `git clone` 或新版分支替代约 1.2GB 归档，也不要修改外部官方目录。

## 快速启动

在有桌面图形会话的终端执行：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
bash 1.sh 6 mission_down.json
```

脚本会做不超过约 2 秒的快速合规预检，生成临时 Realsense 模型，启动 Gazebo、PX4、MAVROS、XTDrone 通信及队伍算法，并等待六路相机。只在同一终端按 `Ctrl-C` 停止。

## 验证

静态、构建和 Catkin 验证：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
bash scripts/verify_competition_clean.sh
```

完整六机检查需两个终端。终端 A 启动上面的命令；终端 B 在相机就绪后运行：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
bash scripts/smoke_competition_clean.sh
```

报告位于 `logs/competition-clean/`，最后一行应为 `PASS competition-clean six-vehicle smoke`。停止后确认没有本次运行的 PX4、Gazebo、MAVROS、ROS、YOLO 进程，临时目录已清理，且 `git -C "$XTDRONE_DIR" status --short` 无输出。日志查看：

```bash
ls -lt logs/competition-clean/
tail -n 80 logs/competition-clean/launch-*.log
tail -n 80 logs/competition-clean/smoke-*.log
```

常见故障和逐步诊断见 [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)。

## 仓库边界

算法包包括任务管理、导航、起飞、位姿转换、目标检测/坐标解算、跟踪和合规检查。生成的 `build/`、`devel/`、日志、外部 PX4/XTDrone/Gazebo 源码不会提交。提交问题时附上版本、启动命令、首次错误和 smoke 报告，不要上传完整固件或 Python 环境。
