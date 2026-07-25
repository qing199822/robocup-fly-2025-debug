# 调试与常见问题

## 已知问题：3 号机可能失稳坠落

六机运行、MAVROS 连接和 12 路 RGB/深度图像已经验证，但当前 `mission_down.json` 联调仍可能出现 `typhoon_h480_3` 侧翻并坠落。2026-07-25 的一次复现中，3 号机在采样开始时已经接近侧翻，随后从约 1.15 米跌至 0.78 米以下；其他五架保持飞行。

已确认并修复过一个独立问题：跟踪节点不应直接发布到 `/xtdrone/typhoon_h480_N/cmd_vel_flu`，否则会和 MUX 同时发布。当前正确关系应为：

```text
/typhoon_h480_N/tracking_node
  -> /typhoon_h480_N/mux_inputs/external/pose_cmd
  -> /typhoon_h480_N/pose_cmd_mux
  -> /xtdrone/typhoon_h480_N/cmd_vel_flu
```

用下面命令确认最终速度话题只有 MUX 一个发布者：

```bash
rostopic info /xtdrone/typhoon_h480_3/cmd_vel_flu
rostopic info /typhoon_h480_3/mux_inputs/external/pose_cmd
```

如果仍然坠落，请在 issue 中附上首次异常前后的仿真时间、Gazebo 位姿、MUX 选中话题和跟踪/任务节点日志。不要把 ROS 日志、PX4 构建目录或完整固件包直接提交到 Git；请压缩后作为 issue 附件或只粘贴相关片段。

## 六路相机没有图像

先检查 Gazebo 日志中是否有：

```text
Can't open display
Unable to create DepthCameraSensor. Rendering is disabled.
```

这表示 Gazebo 没有继承桌面图形会话，不是 YOLO 订阅话题写错。`1.sh` 会在启动 Gazebo 前调用 `scripts/graphics_environment.sh` 自动补齐环境变量，并打印：

```text
Gazebo 图形显示：:1
```

如果仍失败：

```bash
echo "$DISPLAY"
test -r "$XAUTHORITY" && echo readable
pgrep -a gnome-shell
```

不要使用 PX4 1.13；本项目固定使用 `v1.11.0-beta1`。

## 确认 RGB 和深度消息

仿真启动后：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
for id in 0 1 2 3 4 5; do
  timeout 5s rostopic echo -n 1 "/typhoon_h480_${id}/realsense/depth_camera/color/image_raw/header"
  timeout 5s rostopic echo -n 1 "/typhoon_h480_${id}/realsense/depth_camera/depth/image_raw/header"
done
```

只看到话题名不等于相机正常，必须实际收到消息。

## 六机节点检查

```bash
rosnode list | sort
```

应至少包含每架无人机对应的：

- `/typhoon_h480_N/mavros`
- `/typhoon_h480_N_communication`
- `/yolo11n_pedestrian_detector_typhoon_h480_N`
- `/coordinate_estimator_node_typhoon_h480_N`
- `/typhoon_h480_N/tracking_node`
- `/gimbal_control_typhoon_h480_N`
- `/waypoint_navigator_typhoon_h480_N`

## MAVROS 连接失败

```bash
for id in 0 1 2 3 4 5; do
  rostopic echo -n 1 "/typhoon_h480_${id}/mavros/state"
done
```

每架都应有 `connected: True`。如果只有部分实例连接，先停止整个启动脚本，确认没有残留 PX4、MAVROS、Gazebo 或 roslaunch 进程，再重新启动。不要单独重复启动失败编号，因为端口分配是按六机整体配置的。

## 起飞后不进入 OFFBOARD

PX4 接受 OFFBOARD 前必须先持续收到控制设定值。`1.sh` 会先启动六个 XTDrone 通信桥并等待节点出现，然后额外等待两秒，再启动任务节点。手工启动时也必须保持同样顺序。

## 无人机撞建筑或路灯

任务高度必须低于 6 米，因此不能通过提高到建筑上方来规避。修改航点后先运行：

```bash
python3 src/mix_nav/task_manager/test/test_mission_clearance.py
```

测试按无人机尺寸和控制偏差保留 2 米水平余量。它只覆盖已经登记的静态障碍，新增地图或路线仍需要在 Gazebo 中做完整飞行回归。

## 退出后仍有进程

正常退出只需在 `1.sh` 所在终端按 `Ctrl-C`。检查残留：

```bash
pgrep -af 'roslaunch|gzserver|gzclient|px4|mavros|yolo11n|tracking_node'
```

不要在脚本仍运行时直接关闭终端。若桌面或终端异常退出，应先定位确切 PID/进程组，再停止对应仿真，避免误伤其他 ROS 项目。

## 测试说明

项目自身的导航、任务、启动和图形环境测试应通过。`src/gazebo_ros_pkgs` 带有完整上游 Gazebo 测试套件；其中部分测试依赖额外权限、独占端口或专用场景，不适合作为本项目的单次验收标准。提交修改时至少运行 README 中列出的项目测试，并注明未运行的集成测试。
