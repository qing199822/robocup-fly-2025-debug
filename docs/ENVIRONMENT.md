# 仿真环境安装与版本说明

## 版本矩阵

| 组件 | 固定版本 |
| --- | --- |
| Ubuntu | 20.04.6 LTS |
| ROS | Noetic 1.5.0 |
| Gazebo Classic | 11.15.1 |
| PX4 | `v1.11.0-beta1` |
| MAVROS | 1.20.1 |
| Python / GCC / CMake | 3.8.10 / 9.4.0 / 3.16.3 |
| NVIDIA / GPU | 535.230.02 / RTX 3060 12GB |
| PyTorch / Ultralytics | 2.1.2 + CUDA 12.1 wheel / 8.3.40 |

PX4 1.13 会导致本项目相机链路缺图，不能替换。驱动显示 CUDA 12.2 只表示驱动上限，不要求另装完整 Toolkit。

## 安装顺序

先按 [XTDrone 基础配置](https://www.yuque.com/xtdrone/manual_cn/basic_config) 和 [基础配置补充](https://www.yuque.com/xtdrone/manual_cn/basic_config_13) 完成 PX4 配置前步骤。Gazebo 页面不可用时参考 [备用安装说明](https://www.cnblogs.com/iwehdio/p/12756241.html)。顺序为：Ubuntu/ROS、Gazebo 11、MAVROS 与 GeographicLib、XTDrone 和模型库、教程提供的 PX4 归档、YOLO 环境、本仓库构建。

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full gazebo11 libgazebo11-dev \
  ros-noetic-mavros ros-noetic-mavros-extras geographiclib-tools \
  python3-rosdep python3-catkin-tools
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

PX4 下载使用教程给出的约 1.2GB 完整包，不使用 `git clone`。确认 `git -C "$PX4_DIR" describe --tags --always` 为 `v1.11.0-beta1` 后，才执行 `make px4_sitl_default gazebo`。

## 外部目录与变量

```bash
export PX4_DIR=${PX4_DIR:-$HOME/robocup_fly/PX4_Firmware}
export XTDRONE_DIR=${XTDRONE_DIR:-$HOME/robocup_fly/XTDrone}
export GAZEBO_MODELS_DIR=${GAZEBO_MODELS_DIR:-$HOME/robocup_fly/gazebo_models}
export XTDRONE_PYTHONPATH=${XTDRONE_PYTHONPATH:-$HOME/robocup_fly/.xtdrone-python}
export YOLO_PYTHON=${YOLO_PYTHON:-$HOME/robocup_fly/.venv-yolo/bin/python}
export YOLO_CONFIG_DIR=${YOLO_CONFIG_DIR:-$HOME/robocup_fly/.ultralytics}
```

官方目录仅作为只读输入；Actor collision 插件使用本仓库的 `build/`、`devel/` 构建，不会写回 XTDrone。版本和每个输入文件的哈希见 [COMPLIANCE.md](COMPLIANCE.md)。

## YOLO 环境

```bash
cd ~/robocup_fly
python3 -m venv --system-site-packages .venv-yolo
source .venv-yolo/bin/activate
python -m pip install --upgrade pip
python -m pip install -r 2025_ZZU_FLY/requirements-yolo.txt
python -c 'import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))'
```

权重文件为 `2025_ZZU_FLY/src/yolo/yolo11n_942.pt`；再分发许可未在仓库资料中声明，见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 本仓库构建

```bash
cd ~/robocup_fly/2025_ZZU_FLY
source /opt/ros/noetic/setup.bash
catkin_init_workspace src
catkin_make -DCMAKE_BUILD_TYPE=Release
bash scripts/build_xtdrone_actor_collisions.sh
```

不要提交构建产物，也不要通过修改官方源码解决构建错误。
