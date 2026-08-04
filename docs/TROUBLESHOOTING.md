# 调试与常见问题

## 已修复：3 号机撞加油站后失稳

Gazebo 接触流确认，原 `mission_down.json` 航线会让 `typhoon_h480_3::base_link` 撞到 `gas_station_73::link`。一次捕获中接触发生在 `x=53.20, y=-6.96, z=3.87`，随后俯仰角从 23.6 度升到 34.7 度；另一次运行中相同航段导致持续抬头并坠落。

加油站碰撞网格范围约为 `x=51.59..72.17`、`y=-36.63..-6.64`，最高约 8.98 米，不能从上方绕过。当前固定巡逻基线不再让多架飞机共用 `y=0` 通道，而是让六机分别进入西南、南中、东南、西北、北中和东北责任区后在区内闭环。

旧航线修复后的 180 秒六机回归中，3 号机没有接触加油站或路灯，最大倾角为 43.2 度且保持飞行。六机最高高度分别为 5.45、4.95、4.48、3.93、4.45、5.49 米。这组数字是旧任务的历史证据，不是当前 3.5 米固定巡逻基线的验收结果。

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

如果仍然坠落，请在 issue 中附上首次异常前后的仿真时间、Gazebo 位姿、Gazebo 接触对、MUX 选中话题和跟踪/任务节点日志。不要把 ROS 日志、PX4 构建目录或完整固件包直接提交到 Git；请压缩后作为 issue 附件或只粘贴相关片段。

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

## 固定巡逻高度或路线异常

当前高度合同分三层：起飞目标为 3.0 米，固定巡逻的进入点和闭环点统一为 3.5 米，`safety_filter` 默认最高边界为 4.0 米。到达 4.0 米时，最终速度过滤器必须阻止继续向上的速度。起飞器仍必须在每架飞机分别到高后停止该机爬升，不能等待最慢的一架。

运行时唯一权威任务文件是：

```text
src/mix_nav/task_manager/launch/mission_down.json
```

根目录的 `waypoint/mission_down.json` 只是指向它的可见相对链接。每架飞机先执行一次 `entry_waypoints`，随后只循环 `waypoints`；最后一个巡逻点会隐式连接回第一个巡逻点。不要把进入航点复制进巡逻数组，否则会让飞机每轮重新穿越公共起飞区。

## 无人机撞建筑或路灯

固定任务不能通过提高到建筑上方来规避。修改航点后先运行：

```bash
python3 src/mix_nav/task_manager/test/test_mission_clearance.py
```

测试会检查出生点、一次性进入路线、巡逻闭环末段、已登记静态障碍、3.5 米任务高度，以及起飞区外不同飞机路线至少 5 米的中心线净空。静态障碍仍按无人机尺寸和控制偏差保留 2 米水平余量。

这 5 米只保证当前固定任务的静态中心线。目标跟踪会离开固定路线，未来 EGO 也会动态改路，因此两者仍必须提供运行时动态避碰；不能拿这项静态测试代替真实 Gazebo 全航程、接触流和实际机间距离验收。

## 退出后仍有进程

正常退出只需在 `1.sh` 所在终端按 `Ctrl-C`。检查残留：

```bash
pgrep -af 'roslaunch|gzserver|gzclient|px4|mavros|yolo11n|tracking_node'
```

不要在脚本仍运行时直接关闭终端。若桌面或终端异常退出，应先定位确切 PID/进程组，再停止对应仿真，避免误伤其他 ROS 项目。

`1.sh` 会保存任务、辅助节点和底层仿真的 PID，并用可终止后台 Bash 的信号清理。任务节点启动后的自动退出回归耗时约 16 秒。

## competition-clean 完整验证与六机冒烟检查

不打开 Gazebo 前，先在仓库根目录运行完整验证：

```bash
bash scripts/verify_competition_clean.sh
```

它会依次完成静态合规检查、仓库单元测试、Release 构建、官方 Actor collision 插件的工作区外置构建、Catkin 测试及构建后的第二次合规检查。重复运行是安全的；脚本只会更新自己生成的
`competition-artifacts/static-compliance.json` 和
`competition-artifacts/post-build-compliance.json`，不会删除该目录中的其他证据。

需要检查真实六机运行状态时，打开两个终端，并确保两个终端都位于仓库根目录。

终端 A：

```bash
bash 1.sh 6 mission_down.json
```

等待终端 A 显示六路相机就绪并开始任务后，在终端 B 运行：

```bash
bash scripts/smoke_competition_clean.sh
```

报告写入 `logs/competition-clean/smoke-日期-时间.随机字符.log`。只有六架飞机各自的 MAVROS 状态、位置、RGB、深度、彩色相机参数和精确通信节点都通过，并且全局固定 TF
`base_link -> depth_camera_base` 可查询时，报告最后一行才会是：

```text
PASS competition-clean six-vehicle smoke
```

脚本遇到首个失败会立即返回非零，不会把后续车辆的结果伪装成成功：

- `FAIL topic ...`：该编号飞机在 5 秒内没有收到对应消息；只有话题名存在还不够。
- `FAIL node ..._communication`：缺少该编号的精确通信节点，不能由另一架飞机的节点代替。
- `FAIL TF base_link -> depth_camera_base`：比赛启动流程发布的全局 Realsense 固定变换不可查询。

检查完成后，在终端 A 按 `Ctrl-C`，等待启动脚本完成清理，再运行：

```bash
pgrep -af 'px4|gzserver|gzclient|multirotor_communication.py|yolo11n.py|bbox2coord_node.py'
find /tmp -maxdepth 1 -type d -name 'robocup-fly-competition-clean.*' -print
git -C "${XTDRONE_DIR:-../XTDrone}" status --short
```

预期三条命令都没有输出。若第一条列出进程，先依据终端 A 的本次运行日志核对 PID 和进程组；只停止这次 `1.sh` 启动的会话，不要使用宽泛的 `pkill`，以免停止其他 ROS 项目。若第二条列出临时目录或第三条显示 XTDrone 有变化，请保留终端 A 日志和 smoke 报告并按相应错误继续排查，不要手工修改官方目录来绕过检查。

## 测试说明

项目自身的导航、任务、启动和图形环境测试应通过。完整验收以 `scripts/verify_competition_clean.sh` 为准；其中包含官方 Actor collision 插件的工作区外构建、静态边界检查和构建后复核。提交问题时附上命令、版本、退出码和相关日志，不要用修改官方目录的方式绕过失败。
