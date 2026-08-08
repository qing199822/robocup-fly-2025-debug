# AI Agent 项目交接手册

这份文件写给第一次接触本仓库的 AI Agent。开始工作前先完整阅读本文件，再按任务涉及的模块阅读对应源码、测试和包内 `README.md`。不要仅凭聊天摘要或错误日志直接改代码。

## 一分钟了解项目

本项目是 2025 中国机器人大赛多旋翼无人机集群协同搜索仿真项目。运行环境是 ROS Noetic、Gazebo 11、PX4 1.11 和 XTDrone，比赛任务固定使用 6 架 `typhoon_h480_realsense`。

`competition-clean` 是参赛候选分支。核心原则是：

- PX4、XTDrone、Gazebo、官方模型和外部 Python 环境是只读输入。
- 本仓库负责队伍的感知、坐标解算、起飞、任务管理、导航、目标分配、跟踪控制、TF 和合规检查。
- 无人机基线模型不变。唯一允许生成的模型差异是 Realsense include 的安装位姿。
- 启动器在运行前校验官方文件，在本次运行的临时目录生成模型，并在退出时只清理由本次启动登记的进程。
- 发现底层兼容问题时，应修正队伍包或队伍脚本，不得通过改外部官方目录来适配。

首先阅读这些权威文件：

1. [README.md](../README.md)：项目入口和日常命令。
2. [ENVIRONMENT.md](ENVIRONMENT.md)：固定版本、安装顺序和目录布局。
3. [COMPLIANCE.md](COMPLIANCE.md)：比赛边界和官方文件哈希。
4. [THIRD_PARTY.md](THIRD_PARTY.md)：第三方代码和权重许可状态。
5. [TROUBLESHOOTING.md](TROUBLESHOOTING.md)：已知问题和诊断方法。
6. [`ownership.json`](../src/competition_compliance/config/ownership.json)：仓库文件所有权分类。

## 当前可信状态

截至 2026-07-27，提交 `91e6e6f7dea64f966a401e1faadeefb47e8d3a72` 已完成以下验证：

- 真实六机启动完成，6 个 MAVROS、6 个 XTDrone 通信节点、6 组 RGB/深度/CameraInfo 均就绪。
- YOLO 检测、坐标解算和 `down_resume` 任务节点正常启动。
- `scripts/smoke_competition_clean.sh` 最后一行为 `PASS competition-clean six-vehicle smoke`。
- 启动终端按 `Ctrl-C` 后返回 `130`，本次 PX4、Gazebo、ROS、YOLO 和监督进程无残留。
- 最终完整验证中，仓库 Python 测试 118 项通过；Catkin 汇总 116 项，0 error、0 failure。
- PX4、XTDrone 和 Gazebo 官方输入通过静态及构建后哈希检查。

公开仓库和分支：

```text
git@github.com:qing199822/robocup-fly-2025-debug.git
branch: competition-clean
```

后续文档提交可能位于上述运行验证提交之后。新 Agent 必须用下面命令检查实际状态，不能假设工作树、远端或 HEAD 未变化：

```bash
git status --short --branch
git log --oneline --decorate -12
git remote -v
```

`public/main` 是社区调试分支，不得因为维护 `competition-clean` 而移动或覆盖。

## 不可突破的修改边界

### 外部只读输入

以下目录不属于队伍维护范围：

```text
~/robocup_fly/PX4_Firmware
~/robocup_fly/XTDrone
~/robocup_fly/gazebo_models
~/robocup_fly/external/ego-planner-swarm
~/robocup_fly/.xtdrone-python
~/robocup_fly/.venv-yolo
```

禁止改变这些内容来让队伍程序通过：

- PX4 飞控、估计器、传感器驱动和 SITL 核心实现。
- XTDrone 的通信脚本、官方模型和基础框架。
- Gazebo 官方包、物理实现和系统模型库。
- `typhoon_h480_realsense` 的机体几何结构。
- Realsense 的光学、成像、量程、噪声和关节参数。

完整官方哈希只维护在 [COMPLIANCE.md](COMPLIANCE.md) 和 [`official_manifest.json`](../src/competition_compliance/config/official_manifest.json)。不要在新文档或脚本中复制第二份哈希表。

### 允许维护的内容

可以修改：

- `src/` 下的队伍 ROS 包和队伍自有 YOLO 代码。
- 根目录的队伍 launch、任务配置及 `1.sh`。
- `scripts/` 下的队伍构建、验证、图形环境和进程监督工具。
- `waypoint/` 和队伍包内的任务 JSON，但必须保持比赛限高和障碍余量。
- [`sensor_mount.yaml`](../src/competition_compliance/config/sensor_mount.yaml) 中的 6 个安装位姿数值。
- 测试和项目文档。

`sensor_mount.yaml` 只允许改变 Realsense 安装位置和角度。生成模型中必须仍只有一个 `model://realsense_camera` include，固定关节 parent 必须仍为 `base_link`。

### 仓库内第三方文件

`src/gazebo_ros_actor_plugin` 和 `src/darknet_ros_msgs` 包含有来源记录的第三方文件，不等同于队伍原创代码。修改前先查 [THIRD_PARTY.md](THIRD_PARTY.md) 和 `ownership.json`。需要适配时优先在队伍包边界增加调用代码，不要随意改逐字节核验的副本。

## 仓库结构与模块职责

| 路径 | 责任 |
| --- | --- |
| `1.sh` | 一键预检、启动顺序、存活检查、任务监督和有界清理 |
| `robocup_zzufly.launch` | 六机 Gazebo、PX4 SITL 和 MAVROS 启动配置 |
| `scripts/process_supervisor.py` | Linux subreaper；监督脱离会话的后代并按身份清理 |
| `scripts/verify_competition_clean.sh` | 静态合规、单元测试、构建、Catkin 测试和构建后复核 |
| `scripts/smoke_competition_clean.sh` | 运行中的六机 topic、node 和 Realsense TF 检查 |
| `src/competition_compliance` | 官方 manifest、ownership、临时模型生成、固定 TF 和完整合规验证 |
| `src/pose_init` | 将各飞机位姿整理到队伍使用的坐标链路 |
| `src/mix_nav/fly` | 六机起飞逻辑和高度约束 |
| `src/mix_nav/task_manager` | 多机任务和航点调度 |
| `src/mix_nav/simple_navigator` | 航点导航、速度连续性和转向控制 |
| `src/look_up` | 目标查询服务和 `down_resume` 总任务 launch |
| `src/tracking` | 目标跟踪状态机、滤波、平滑和控制命令 |
| `src/ego_fusion_search/safety_filter` | 最终速度看门狗、限幅和发布者唯一性边界 |
| `src/ego_fusion_search/search_msgs` | 队伍局部地图健康与净空度消息 |
| `src/ego_fusion_search/local_mapping` | 0 号机深度同步、人物语义过滤、局部体素地图、健康、净空度、前沿候选和轨迹扫掠验证；EGO 模式唯一占据事实来源 |
| `src/ego_fusion_search/search_coordinator` | 只为 0 号机选择已知空闲局部目标并发布任务 generation，不发布飞行控制命令 |
| `src/ego_fusion_search/ego_adapter` | 校验外部 EGO 轨迹与速度命令，只发布既有 navigator MUX 输入 |
| `src/transform_tree` | 队伍需要的动态 TF 发布 |
| `src/yolo` | 六路检测、深度与 CameraInfo 匹配、三维坐标解算 |
| `waypoint` | 可见任务入口、地图和路线辅助数据；`mission_down.json` 是指向包内权威文件的链接 |
| `tests` | 启动器、边界、图形环境、验证脚本、相机几何和生命周期回归 |

修改某个包前，先阅读该包的 `README.md`、`package.xml`、`CMakeLists.txt` 和测试。不要仅根据文件名推测接口。

## 运行时数据流

一键启动顺序：

```text
1.sh
  -> 快速合规预检和本次运行的私有临时模型
  -> Gazebo + 6 个 PX4 SITL + 6 个 MAVROS
  -> 6 个 XTDrone 通信节点
  -> 6 组 Realsense RGB / depth / CameraInfo 就绪检查
  -> 6 个 YOLO worker + 6 个坐标解算节点
  -> down_resume.launch
  -> 起飞、任务管理、导航、目标服务和跟踪节点
```

感知链路概念图：

```text
Realsense RGB
  -> yolo11n.py
  -> darknet_ros_msgs/BoundingBoxes
  -> bbox2coord_node.py + depth + CameraInfo + TF
  -> actor_msgs/ActorInfo（裁判话题）
  -> CoordinateBroadcastHeartbeat（队伍内部确认）
  -> tracking / look_up 完成状态
```

控制命令必须依次经过 MUX 和后置安全过滤：

```text
fly_takeoff -> takeoff input -------------------------------+
static_patrol: simple_navigator -> navigator input ---------+-> pose_cmd_mux -> raw_cmd_vel
ego: EGO -> ego_adapter -> navigator input -----------------+                     |
tracking -> external input ---------------------------------+                safety_filter
                                                                                  |
                                                               XTDrone cmd_vel_flu (唯一发布者)
```

起飞节点先选择 takeoff；六机全部成功并发送零速度、HOVER 后才选择 navigator。任何起飞未完成或部分交权失败都保持或回滚到零速度 takeoff。跟踪通过现有 MUX 服务在 external 与 navigator 之间切换。

`/swarm/takeoff_complete` 是全机门控，类型为锁存的 `std_msgs/Bool`。`confident_takeoff_node` 启动时发布 `false`，只有六机全部到高且 navigator 交权全部成功后才发布 `true`。tracking 默认按 `false` 处理：不能锁目标、发布 `PAUSE` 或选择 external；运行中回退为 `false` 时只释放一次目标并清空状态，不主动切换 MUX。成功后起飞节点保持空闲存活只是为了保存 ROS 1 锁存值，不再发送任何飞行命令。

人物完成条件以成功发布裁判 `ActorInfo` 后的本机结构化心跳为准，不以检测框可见时间为准。tracking 要求同一架无人机对当前锁定人物连续广播 15 秒，相邻心跳默认不能超过 0.5 秒。确认后继续跟踪并等待裁判移除；单次会话最多 20 秒。人物消失或会话到期后进入显式 `RETURNING`：先持续发布零 external 命令，成功切回 navigator MUX，再完成或释放中央锁，最后只发送一次 `RESUME`。MUX 失败时不得提前恢复任务。未满 15 秒便达到 20 秒上限的目标只释放，并对本机设置 5 秒重试冷却；`COMPLETED` 目标不能再次申请。

固定巡逻的高度合同是起飞 3.0 米、任务航点 3.5 米、`safety_filter` 默认上限 4.0 米。任务管理器先完整校验六机 JSON，再创建线程；每机的 `entry_waypoints` 只执行一次，`waypoints` 在最后一点到第一点之间隐式闭环。跟踪暂停会保存当前处于进入还是巡逻阶段，返航后从原阶段和原索引继续。

`src/mix_nav/task_manager/launch/mission_down.json` 是运行时唯一权威文件。根目录 `waypoint/mission_down.json` 只是指向它的相对符号链接。当前六机分别负责西南、南中、东南、西北、北中、东北区域；自动化几何契约保证固定任务在起飞区外的跨机中心线净空不小于 5 米。该保证不覆盖跟踪和未来 EGO 动态改路，它们仍需动态避碰。

当前 EGO 接线只覆盖 `typhoon_h480_0`。EGO-Planner-Swarm 固定在仓库外，按 GPL-3.0 作为只读运行依赖；必须由 `scripts/check_ego_external.py` 核验固定提交和干净工作树，不能把其源码复制进本仓库或为兼容本项目而修改它。`search_coordinator` 只发布局部目标和 generation，`local_mapping` 是唯一占据事实来源，`ego_adapter` 只能替换 `simple_navigator` 并发布既有 navigator MUX 输入。它不得新增第四路 MUX 输入，也不得直接发布最终 XTDrone 速度话题。

`navigation_mode:=static_patrol` 保留原六机固定巡逻；`navigation_mode:=ego` 只启动 0 号机的地图、协调器和适配器。`layered_2d` 仍未实现，启动器会明确拒绝，不能静默降级。当前不得声称双机或六机 EGO 已跑通，也不得把自动化接线测试描述成 Gazebo 绕障验收。最终 `/xtdrone/typhoon_h480_N/cmd_vel_flu` 的队伍发布者必须始终且仅为 `/typhoon_h480_N/safety_filter`；可用 `python3 scripts/check_final_control_publishers.py` 检查。

清理链路：

```text
Ctrl-C
  -> 1.sh 只处理已登记的 PID / PGID 和启动时间
  -> process_supervisor 接管孤儿后代
  -> TERM 宽限期
  -> 对身份仍匹配的残留发送 KILL
  -> 回收进程、删除本次临时模型和门控文件
```

## 环境、目录与变量

固定环境组合：

| 组件 | 已验证版本 |
| --- | --- |
| Ubuntu | 20.04.6 LTS |
| ROS | Noetic 1.5.0 |
| Gazebo Classic | 11.15.1 |
| PX4 | `v1.11.0-beta1` |
| MAVROS | 1.20.1 |
| Python | 3.8.10 |
| NVIDIA 驱动 | 535.230.02 |
| PyTorch / Ultralytics | 2.1.2 CUDA 12.1 wheel / 8.3.40 |

不要替换为 PX4 1.13；该版本在本项目中出现过相机无图问题。PX4 使用教程提供的约 1.2GB 完整归档，不用其他分支代替。

默认目录：

```text
~/robocup_fly/
|-- 2025_ZZU_FLY/       # 仓库主工作区或来源仓库
|-- PX4_Firmware/       # 外部 PX4 1.11
|-- XTDrone/            # 外部 XTDrone 8e88116
|-- gazebo_models/      # 外部 Gazebo 模型
|-- .venv-yolo/         # YOLO Python 环境
`-- .xtdrone-python/    # XTDrone Python 依赖
```

如果使用 Git worktree，当前目录可能类似：

```text
~/robocup_fly/2025_ZZU_FLY/.worktrees/competition-clean
```

不要根据 `../` 猜外部依赖位置；显式设置环境变量：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
export PX4_DIR=${PX4_DIR:-$HOME/robocup_fly/PX4_Firmware}
export XTDRONE_DIR=${XTDRONE_DIR:-$HOME/robocup_fly/XTDrone}
export GAZEBO_MODELS_DIR=${GAZEBO_MODELS_DIR:-$HOME/robocup_fly/gazebo_models}
export XTDRONE_PYTHONPATH=${XTDRONE_PYTHONPATH:-$HOME/robocup_fly/.xtdrone-python}
export YOLO_PYTHON=${YOLO_PYTHON:-$HOME/robocup_fly/.venv-yolo/bin/python}
export YOLO_CONFIG_DIR=${YOLO_CONFIG_DIR:-$HOME/robocup_fly/.ultralytics}
source /opt/ros/noetic/setup.bash
```

在 worktree 中，将第一条 `cd` 换成实际 worktree 的绝对路径，其余外部变量保持指向 `~/robocup_fly/` 下的只读依赖。

## 程序使用命令

### 首次或干净构建

先确认 PX4 本身已经构建：

```bash
cd "$PX4_DIR"
make px4_sitl_default gazebo
```

然后构建队伍工作区：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
source /opt/ros/noetic/setup.bash
catkin_init_workspace src
catkin_make -DCMAKE_BUILD_TYPE=Release
bash scripts/build_xtdrone_actor_collisions.sh
```

Actor collision 插件从只读 XTDrone 输入构建到本工作区，不应在 XTDrone 目录产生修改。

### 日常启动

在图形桌面终端 A：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
bash 1.sh 6 mission_down.json
```

启动器会依次等待六机连接、通信节点、相机和队伍辅助节点。不要在等待过程中另开一套相同仿真。

可见任务入口位于 `waypoint/`，但 `1.sh` 的参数只传文件名。例如：

```bash
bash 1.sh 6 mission_middle.json
bash 1.sh 6 mission_up.json
```

`mission_down.json` 的真实内容维护在 `src/mix_nav/task_manager/launch/mission_down.json`，不要把符号链接改回第二份 JSON。修改任务前先确认比赛规则和航点测试仍适用。

### 运行态六机检查

终端 A 显示六路相机就绪并启动任务后，在终端 B：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
bash scripts/smoke_competition_clean.sh
```

成功报告写入 `logs/competition-clean/`，最后一行必须是：

```text
PASS takeoff gate /swarm/takeoff_complete
PASS competition-clean six-vehicle smoke
```

该检查要求每架飞机都有 MAVROS 状态、位置、RGB、深度、CameraInfo 和精确通信节点，并能查询 `base_link -> depth_camera_base`。

### 停止程序

回到终端 A，按一次 `Ctrl-C`，等待脚本完成清理并返回 shell。预期退出码是 `130`。

不要直接关闭终端，不要按名称批量结束系统中的 ROS 进程。机器可能同时运行其他项目，只能处理本次启动器登记的会话。

退出后检查：

```bash
pgrep -af 'px4|gzserver|gzclient|multirotor_communication.py|yolo11n.py|bbox2coord_node.py'
find /tmp -maxdepth 1 -type d -name 'robocup-fly-competition-clean.*' -print
git -C "$XTDRONE_DIR" status --short
```

解释：

- 第一条不应列出本次运行的进程。注意 `pgrep` 可能显示检查命令自身，必须核对完整参数和 PID。
- 第二条不应比启动前增加目录；历史目录不能未经归属核对就删除。
- 第三条应为空，表示外部 XTDrone 没有被写入。

### 查看日志

```bash
ls -lt logs/competition-clean/
tail -n 120 logs/competition-clean/launch-*.log
tail -n 120 logs/competition-clean/smoke-*.log
ls -lt logs/verification/
```

优先保存最早出现的错误、命令、退出码和对应日志路径。长日志不要整份提交到 Git。

## 修改和验证工作流

### 开始修改前

```bash
git status --short --branch
git diff --check
python3 -m json.tool src/competition_compliance/config/ownership.json >/dev/null
```

然后：

1. 在 `ownership.json` 查目标文件的类别。
2. 阅读最接近的包 README、launch、接口和现有测试。
3. 用日志或最小复现确认根因，不根据现象猜修复。
4. 检查工作树中是否已有用户改动；不得覆盖或回退无关修改。

### 行为修改

先写一个能稳定复现问题的失败测试，确认它因目标缺陷失败；再做最小修改并看测试转为通过。重点测试真实行为，不要只检查源码是否包含某个字符串。

常用聚焦测试示例：

```bash
python3 -m unittest tests.test_camera_geometry -v
python3 -m unittest tests.test_one_click_launch -v
python3 -m unittest tests.test_yolo_helper_lifecycle -v
python3 -m unittest tests.test_verification_scripts -v
python3 src/mix_nav/task_manager/test/test_mission_clearance.py
```

全仓 Python 回归：

```bash
python3 -m unittest discover -s tests -v
```

Shell 或 Python 脚本变化还应执行：

```bash
bash -n 1.sh scripts/*.sh src/yolo/*.sh
python3 -m py_compile scripts/process_supervisor.py
git diff --check
```

### 完整验证

环境、模型、launch、启动器、跨包接口或发布前变化必须运行：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
bash scripts/verify_competition_clean.sh
```

该脚本执行：

1. 静态官方文件和所有权检查。
2. 仓库 Python 单元测试。
3. Release Catkin 构建。
4. Actor collision 插件工作区外置构建。
5. Catkin 测试和结果汇总。
6. 构建后的第二次官方文件检查。

最后应看到：

```text
完整验证通过：静态与构建后合规证据均已生成。
```

合规证据在：

```text
competition-artifacts/static-compliance.json
competition-artifacts/post-build-compliance.json
```

完整验证通过不代替六机运行态 smoke；涉及运行流程的变化必须两者都做。

## 常见故障排查

### Gazebo 或相机没有画面

先查启动日志是否有 `Can't open display` 或 `Rendering is disabled`。检查：

```bash
echo "$DISPLAY"
echo "$XAUTHORITY"
test -r "$XAUTHORITY" && echo readable
pgrep -a gnome-shell
```

`1.sh` 会通过 `scripts/graphics_environment.sh` 导入桌面图形环境。不要把渲染失败误判为 YOLO 订阅错误，也不要更换 PX4 大版本绕过。

确认相机必须实际收到消息：

```bash
for id in 0 1 2 3 4 5; do
  timeout 5s rostopic echo -n 1 "/typhoon_h480_${id}/realsense/depth_camera/color/image_raw/header"
  timeout 5s rostopic echo -n 1 "/typhoon_h480_${id}/realsense/depth_camera/depth/image_raw/header"
done
```

只有 topic 名存在不代表传感器正常。

### MAVROS 连接不完整

```bash
for id in 0 1 2 3 4 5; do
  rostopic echo -n 1 "/typhoon_h480_${id}/mavros/state"
done
```

每架都应显示 `connected: True`。部分连接时先完整停止本次六机启动、检查端口和残留，再整体重启；不要单独补启动某个编号。

### YOLO 或坐标节点提前退出

```bash
rosnode list | sort | grep -E 'yolo11n|coordinate_estimator'
tail -n 160 logs/competition-clean/launch-*.log
```

`src/yolo/multi_yolo_detecting.sh` 必须保持为 6 个 worker 的父监督脚本。修改它时要保留 PID 启动时间校验、启动阶段信号窗口处理、异常退出码传播和有界 TERM/KILL 清理。

### 起飞后不进入 OFFBOARD

PX4 接受 OFFBOARD 前必须持续收到 setpoint。正确顺序是六机连接、XTDrone 通信节点就绪、控制命令持续发布，然后启动任务。不要缩短 `1.sh` 的依赖存活检查来掩盖初始化错误。

### 无人机超高或撞障碍

固定任务应保持 3.5 米，最终安全过滤上限为 4.0 米。路线变化先运行航点净空测试，再进行真实 Gazebo 全航程回归。测试覆盖完整进入段、巡逻闭环、已登记静态障碍和起飞区外 5 米跨机中心线净空；跟踪和未来 EGO 动态路线仍需单独验证动态避碰。

### Ctrl-C 后仍有进程

先保留启动日志并核对本次启动登记的 PID、PGID 和启动时间。`1.sh` 与 `process_supervisor.py` 的安全合同包括：父进程提前退出、独立会话后代、PID 复用、单调时钟截止时间、TERM 后 KILL 和无关会话隔离。

不要为了方便退回到按名称清理，也不要只在测试结束后宽泛扫系统进程。新增清理场景必须同时验证“本次进程被清掉”和“无关独立会话仍存活”。

## Git 与交付规则

### 提交前

```bash
git status --short
git diff --check
git diff --stat
git diff
```

确认：

- 只有任务要求的文件变化。
- 没有构建产物、虚拟环境、日志、PX4 固件或官方模型。
- `.vscode/settings.json` 和用户无关改动得到保留。
- 相关测试和完整验证结果有明确退出码，不依赖“应该通过”的推测。

### 提交

```bash
git add <本次明确修改的文件>
git commit -m "<类型>: <简短说明>"
git status --short --branch
```

不要使用会把工作树中所有文件一并加入的模糊操作。提交后再次核对 commit 内容。

### 推送 competition-clean

只有用户要求发布时才执行：

```bash
git push -u public competition-clean
git ls-remote --heads public competition-clean
```

要求：

- 不使用强制推送。
- 不移动 `public/main`。
- 本地 HEAD 与远端 `refs/heads/competition-clean` SHA 必须一致。
- 推送后保留 worktree，方便继续处理审查意见。

## 已知风险和维护重点

- Gazebo Classic 11 已结束上游生命周期，但比赛环境固定使用它；不要在参赛分支进行新 Gazebo 迁移。
- PX4 版本非常旧，必须按 manifest 固定；升级属于新的环境项目，不是普通修复。
- `src/yolo/yolo11n_942.pt` 的再分发授权在现有资料中为 `NOASSERTION`，公开发布前需单独确认权利。
- Realsense 位姿在软件上受约束，但物理合理性仍需队伍和裁判确认。
- 进程身份检查存在 Linux PID 与 `/proc` 读取之间不可完全消除的微小竞态；不要降低现有 start-time 防护。
- 启动器清理失败使用保留状态 `125`，受监督命令自然返回同一状态时日志可能表现为监督清理失败，但最终仍是非零失败。
- 六机仿真资源消耗高。不能用减少实例、跳过相机或缩短就绪检查来让验收看起来更快。
- `smoke` 证明消息和节点存在，不证明完整比赛路线安全；任务修改仍需全航程观察、限高和碰撞验证。
- 坐标广播完成与恢复巡逻已在真实六机运行中走通，但同次运行发生撞房和异常高度；该功能现场通过不等于完整任务安全或比赛就绪。

## 新 Agent 首次接手清单

按顺序完成：

- [ ] 确认当前目录和分支是预期 worktree，而不是误在 `main`。
- [ ] 运行 `git status --short --branch`，记录并保留用户现有改动。
- [ ] 阅读本文件、README、COMPLIANCE、THIRD_PARTY 和目标包 README。
- [ ] 在 `ownership.json` 中确认目标文件归属。
- [ ] 确认 6 个外部环境变量指向实际只读依赖。
- [ ] 运行与任务相关的最小现有测试，建立修改前基线。
- [ ] 用日志、测试或最小复现确定根因。
- [ ] 行为变化先写失败回归，再做最小修复。
- [ ] 运行聚焦测试、全仓 Python 测试和静态检查。
- [ ] 高风险或发布变化运行完整 verifier 和真实六机 smoke。
- [ ] Ctrl-C 后检查进程、临时目录和 XTDrone 状态。
- [ ] 复核 diff，只提交任务内文件。
- [ ] 用户明确要求后才推送；不得改变公开主分支。

## 交接结果模板

后续 Agent 完成一轮维护时，用下面格式留下结果：

```markdown
### 目标

一句话说明本轮要解决的问题。

### 根因与边界

- 根因：说明证据，不只描述现象。
- 修改范围：列出队伍文件。
- 官方输入：说明 PX4、XTDrone、Gazebo 和模型是否保持不变。

### 修改

- 文件路径：行为变化。
- 文件路径：新增或更新的测试。

### 验证

- 命令：退出码和通过数量。
- 完整 verifier：是否通过。
- 六机 smoke：报告路径和最后一行。
- Ctrl-C：退出码、进程残留、临时目录变化、XTDrone 状态。

### Git

- 分支：competition-clean
- 提交：完整 SHA 和标题
- 远端：是否推送；远端 SHA

### 剩余风险

只列已经确认、尚未解决且会影响下一位维护者的事项。
```

如果某项没有执行，必须明确写“未执行”及原因，不能省略后让下一位 Agent 误以为已经验证。

## 2026-08-03 起飞门控与自动图形环境验收

### 根因与修改边界

- `1.sh` 原先进入 bubblewrap 只读隔离后才读取桌面进程环境；隔离内无法读取宿主桌面的 `/proc/<pid>/environ`，因此缺少 `DISPLAY` 时会在 Gazebo 启动前退出。
- 启动器现在在进入只读隔离前调用本项目的图形环境探测，再把导出的变量自然传给内层进程；没有硬编码显示编号。
- ROS Noetic 在本机将 `std_msgs/Bool` 输出为 `data: True`，smoke 原先只接受小写 `true`；判断现仅兼容 `true` 和 `True` 两种输出拼写，其他值仍拒绝。
- 本轮只修改队伍启动脚本、smoke、测试和本文档。PX4、XTDrone、Gazebo、EGO、第三方插件与官方模型均未修改；验收后 `XTDrone` 工作树为空。

### 自动化验证

- 聚焦图形环境和 smoke 测试：22 项通过。
- 完整 verifier：仓库 Python 134 项通过；Catkin 146 项，0 errors、0 failures；静态与构建后合规检查通过。
- Codex 隔离内的首次 verifier 因 `netifaces.interfaces()` 被禁止读取网络接口而失败；在本机环境用相同 verifier 重跑后全部通过。这是工具隔离限制，不是项目失败。

### 真实六机验收

- 启动时显式清空 `DISPLAY`、`XAUTHORITY`、`XDG_RUNTIME_DIR` 和 `WAYLAND_DISPLAY`，启动器仍自动找到 `DISPLAY=:1` 并进入 Gazebo。完整日志：`logs/competition-clean/launch-20260803-220852-2puSP5.log`。
- 日志记录六个 tracking 节点均收到起飞门控打开；无人机 0、2、5 随后锁定人物并成功把各自 MUX 切换到 external 输入，证明门控后跟踪接管链可运行。
- smoke 报告：`logs/competition-clean/smoke-20260803-221012.8bXHsd.log`，最后一行为 `PASS competition-clean six-vehicle smoke`；其中起飞门控、最终发布者唯一性和传感器 TF 均通过。
- 使用 Ctrl-C 正常停止，外层退出码 130；项目相关 Gazebo、PX4、roslaunch、XTDrone 通信和起飞节点无残留，competition-clean 临时目录无残留。

## 2026-08-04 固定巡逻碰撞修复与真实复验

### 目标

修复首次固定巡逻验收中已经闭合的两个根因：4 号机路线穿越 `house_1_146_clone`，以及 tracking 在任务管理器仍为 `IDLE` 时提前发送 `PAUSE`。本轮没有根据坠落后的异常坐标猜测修改 PX4、XTDrone、Gazebo、EGO-Planner-Swarm 或官方无人机模型。

### 首次失败证据

- 主日志：`logs/competition-clean/launch-20260804-172326-kQZ6IP.log`。
- smoke：`logs/competition-clean/smoke-20260804-172926.AUMa07.log`。smoke 通过，但完整飞行验收失败，证明 smoke 不能代替全航程验证。
- 首次 bag：`/tmp/static-patrol-validation.bag`，约 8 MB；首次接触流：`/tmp/static-patrol-contacts.log`，约 3.0 GB。两份文件仍保留。
- 0、4 号机异常最高高度分别为 490.143 米和 489.329 米；5 号机最高仅约 2.277 米。
- 任务阶段最小机距为 2.384 米，发生在 4、5 号机之间。
- Gazebo 接触流确认 4 号机旧路线持续接触 `house_1_146_clone`。官方 world 和碰撞网格计算出的房屋世界边界原先未登记到几何测试中。
- 日志确认 5 号机在起飞门控打开后发送 `PAUSE`，但任务管理器当时仍为 `STATE_IDLE`，该命令被忽略。
- 0 号机首次事故发生在旧接触采集开始前，根因没有闭合，本轮不猜修。

### 修改与自动化验证

- `test_mission_clearance.py` 登记 `house_1_146` 和 `house_1_146_clone` 的碰撞边界；旧 4 号机路线先稳定出现三个 `intersects house_1_146_clone` 失败，再改为南侧进入 `(0,7) -> (25,7)` 和北中巡逻矩形 `(25,12) -> (68,12) -> (68,41) -> (25,41)`。
- `MissionManager` 新增锁存话题 `/typhoon_h480_N/mission/active`：初始化为 `false`，完成位姿等待和默认 10 秒倒计时、进入任务阶段后为 `true`；`PAUSED` 和 `RESUMING` 保持已激活。
- tracking 必须同时收到 `/swarm/takeoff_complete=true` 和本机 `/mission/active=true` 才能请求目标、发送 `PAUSE` 或切换 external；任一门控关闭时释放目标并复位。
- 新增 task_manager rostest，先确认当前实现因缺少初始 `false` 失败，再验证锁存状态先假后真。tracking rostest 先确认仅起飞门控为真时错误请求 `green0`，再覆盖两个门控的四种组合和两种关闭复位。
- 聚焦几何测试 9 项通过；task_manager、tracking、safety_filter 聚焦构建通过，工作区 Catkin 汇总 178 项、0 errors、0 failures。
- fresh 完整 verifier 在本机环境通过：仓库 Python 134 项通过，Catkin 178 项、0 errors、0 failures，静态和构建后合规证据均生成。工具隔离内不能枚举网络接口，因此 ROS 测试使用本机权限执行。
- PX4、XTDrone、Gazebo、EGO 和官方模型均未修改；完整验证和真实运行结束后 `git -C "$XTDRONE_DIR" status --short` 为空。

### 2026-08-04 真实六机复验结果

本次结果为失败，不能作为比赛可用基线。

- 主日志：`logs/competition-clean/launch-20260804-181217-A05PZ5.log`。
- smoke：`logs/competition-clean/smoke-20260804-181340.rZy76C.log`，最后一行为 `PASS competition-clean six-vehicle smoke`。
- 恢复索引后的 bag：`/tmp/static-patrol-revalidation.bag.active`，持续 406 秒、195030 条消息、约 66 MB。原始副本保留为 `.active.raw` 和 `.orig.active`，不要擅自删除。
- 接触流：`/tmp/static-patrol-revalidation-contacts.log`，约 5.1 GB。采集在两个门控打开前开始。
- 起飞门控事件为 `1785.544 false`、`1793.816 true`。六个任务激活话题先在约 1785.5 发布 `false`，再于 `1795.524–1795.700` 发布 `true`。
- 5 号机首次 `PAUSE` 在 1814.168，晚于本机 mission active 1795.608；本轮没有再出现“在 `IDLE` 时被 tracking 抢占”的启动竞态。
- 4 号机完成两个进入点并到达巡逻点 1、2，运行位置经过约 `(66.6,13.3,3.3)`；接触流中没有 `house_1_146_clone`，说明确定的绕房路线解决了原碰撞。
- 任务进度并未全机闭环：0 号机没有航点到达事件；1 号机完成两个进入点和巡逻点 1–4；2 号机仅完成前两个进入点；3 号机完成进入并运行多轮；4 号机完成进入和巡逻点 1、2；5 号机完成六个进入点和巡逻点 1、2。
- 全程和任务阶段最小三维机距均为 0.671 米，发生在 4、5 号机，时刻 1798.064。接触流确认 1797.740–1798.240 期间两机的 `base_link`、保险杠和旋翼发生直接接触；事故位置约为 4 号机 `(-13.581,2.408,3.593)`、5 号机 `(-12.688,2.020,3.600)`。
- 5 号机从 1952.288 起接触 `house_3_68`，当时位置约 `(101.197,13.755,3.504)`；随后接触地面，任务阶段最低 -165.266 米、最高 488.934 米，首次超过 10 米在 1982.328。
- 2 号机从 2009.640 起接触 `house_3_156`，当时位置约 `(-36.058,-20.513,2.963)`；随后持续接触地面，任务阶段最低 -9.018 米。
- 0、1、3、4 号机任务阶段最高高度分别为 3.425、3.599、3.627、3.597 米；2 号机最高 3.600 米。0 号机本轮没有出现约 490 米的坐标发散，但长期没有完成航点。
- 2、5 号机安全状态历史包含 `ALTITUDE_LIMIT`，停止前读取均为 `OK`，MUX 均选择 navigator；这不代表事故期间安全边界有效。
- 主启动器按一次 Ctrl-C 后退出码 130；rosbag 和接触采集退出码均为 0。本次 Gazebo、PX4、roslaunch、XTDrone 通信、YOLO 和采集进程无残留，competition-clean 临时目录无残留，XTDrone 工作树为空。

### Git

- 分支：`competition-clean`。
- 本轮主要提交：`90dd5cd fix: route north-central patrol around houses`、`d312006 feat: publish active mission state`、`d94a775 fix: gate tracking on active missions`。
- 计划和执行命令修正另有文档提交。未经用户再次明确要求，本轮未推送公开仓库。

### 剩余风险与下一步

- 4、5 号机在起飞区内只相距约 3 米，现有几何测试会裁掉起飞区内部线段，因此没有发现两机进入初段的动态碰撞。下一轮应先单独设计起飞区解耦或进入时序，而不是缩小 5 米净空合同。
- 5 号机撞 `house_3_68`、2 号机撞 `house_3_156` 的完整控制链原因尚未闭合。结合暂停/恢复日志，必须区分静态任务段、tracking 外部控制和断点返航段，再决定是补障碍、修返航还是调整跟踪退出；不要直接改官方 world 或飞控。
- 0 号机本轮未坠毁但始终没有航点到达，需从 bag 中结合目标锁定、MUX 和任务状态单独分析。现有 bag 没有记录 MUX selected 和 mission control 话题，下一轮证据采集应补上这两类话题。
- 当前真实复验失败，禁止把 `competition-clean` 描述为全航程安全或比赛就绪。

## 2026-08-04 人物坐标广播完成与恢复巡逻

### 规则与实现

- 规则要求连续 15 秒正确广播人物 ID 和坐标；本实现只在 `ActorInfo` 真正发布成功后生成结构化心跳。
- `look_up` 将人物状态扩展为 `AVAILABLE`、`TRACKED`、`COMPLETED`，完成状态幂等且普通 release 不能重新开放。
- tracking 使用独立 `BroadcastProgress` 计算 15 秒连续广播和 20 秒会话上限；错误无人机、错误人物、未来、倒退、零和断续时间戳不会错误累计。
- 返回巡逻通过 `RETURNING` 状态保证零命令、navigator MUX、complete/release、单次 `RESUME` 的顺序。navigator 切换失败会重试；完成调用最多 3 次，失败后只按普通 release 处理。
- 本轮只修改队伍自有 `look_up`、`yolo`、`tracking`、测试和文档。PX4、XTDrone、Gazebo、EGO-Planner-Swarm、第三方 actor 插件和官方无人机模型均未修改。

### 自动化证据

- TDD 的 RED 证据是旧状态机在 navigator MUX 第一次失败后仍立即发送 `RESUME`，且人物消失后没有调用 `complete(green0)`。
- Task 4 聚焦结果：tracking 汇总 14 项，0 errors、0 failures；其中完成流程验证 `mux_fail -> mux_success -> complete(green0) -> RESUME`，未确认的 `blue1` 在 20 秒等比例加速上限后只 release，并遵守本机冷却。
- fresh 完整 verifier 在本机权限下通过：仓库 Python 137 项、Catkin 192 项，均为 0 errors、0 failures；Release 构建、actor collision 外置构建、静态与构建后官方文件检查均通过。隔离环境不能枚举网络接口，因此 ROS 测试按既定方式在本机权限下执行。
- verifier 后 `git -C "$XTDRONE_DIR" status --short` 为空。PX4、XTDrone、Gazebo、EGO-Planner-Swarm、第三方 actor 插件源码和官方模型均未被本轮修改。

### 真实六机证据

- 主日志：`logs/competition-clean/launch-20260804-192440-LNaSc5.log`。
- smoke：`logs/competition-clean/smoke-20260804-192600.v6wx5b.log`，最后一行为 `PASS competition-clean six-vehicle smoke`。
- 1 号机在仿真时刻 1922.156 将 MUX 切至 external 并进入 `TRACKING`，在 1923.592 记录 `brown2` 的首个有效坐标广播，在 1938.656 确认连续广播满 15 秒。此前 1873.892 的同目标首个心跳属于已释放的另一会话，没有跨会话错误累计。
- 在 1942.156，1 号机依次记录 `Returning control for target 'brown2'`、navigator MUX 切换成功、`Completed target 'brown2'`、单次 `RESUME` 和 `RETURNING -> IDLE`。这证明“确认完成并恢复任务”的现场主路径已走通。
- `brown2` 完成后再次申请均被拒绝，说明 `COMPLETED` 状态没有重新开放。日志仍沿用通用文案 `already locked by another drone`，该文案不能用来区分“已完成”和“正被锁定”。
- 补充 bag：`/tmp/tracking-report-validation.bag`，持续 76.936 秒、36988 条消息、约 13 MB。窗口内 0–5 号机最高高度依次为 3.557、3.583、3.575、3.591、484.632、488.832 米；4、5 号机仍存在严重异常高度。
- 该补充窗口内最小三维机距为 18.860 米，发生在 1、3 号机。采集在事故后才开始，不能用该数值证明完整全航程机间安全。
- 接触流：`/tmp/tracking-report-validation-contacts.log`，约 895 MB。其内容明确记录 5 号机的 `base_link`、保险杠等部件接触 `house_3_68`，随后旋翼等部件接触 `ground_plane`。
- 主启动器按一次 Ctrl-C 后退出码 130；bag 和接触采集均以 0 退出。停止后没有本次 PX4、Gazebo、roslaunch、XTDrone 通信、YOLO 或 tracking 残留，没有新增 competition-clean 临时目录，XTDrone 工作树为空。

### 当前安全结论

人物连续广播 15 秒、完成目标并恢复巡逻的功能已有自动化和真实六机证据，但本次整体运行仍为失败：4、5 号机出现约 485–489 米异常高度，5 号机撞 `house_3_68` 并接触地面；上一轮确认的 4/5 号机动态碰撞、2 号机撞 `house_3_156` 和 0 号机无航点进度也尚未全部闭合。禁止把 `competition-clean` 描述为全航程安全或比赛就绪。

## 2026-08-07 局部地图基础链真实 Gazebo 检查

### 范围与运行方式

- 验收时代码基线为 `d0b640ed0665a84fee56b00046f10bb3ddb7f8d4`（`test: add single-drone mapping contract checker`）。
- 这是“六机比赛场景中只运行 0 号机局部地图”的检查，不是独立纯单机 Gazebo 场景，也不是连续 5 次完整单机验收。

在图形桌面终端 A 启动六机主环境：

```bash
cd ~/robocup_fly/2025_ZZU_FLY-competition-clean
source /opt/ros/noetic/setup.bash
source devel/setup.bash
bash 1.sh 6 mission_down.json
```

等待终端 A 报告六路相机和任务链就绪后，在终端 B 只启动 `typhoon_h480_0` 的 `local_mapping` 节点：

```bash
cd ~/robocup_fly/2025_ZZU_FLY-competition-clean
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch local_mapping local_mapping_single.launch \
  vehicle_type:=typhoon_h480 drone_id:=0
```

其他五架无人机不启动局部地图节点。本次六机主环境的完整日志为 `logs/competition-clean/launch-20260807-202341-BwUXbC.log`。

### 30 秒合同检查结果

终端 B 的局部地图节点就绪后，在终端 C 运行 30 秒合同检查：

```bash
cd ~/robocup_fly/2025_ZZU_FLY-competition-clean
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 scripts/check_local_mapping_single.py \
  --vehicle typhoon_h480_0 --duration 30
```

本次检查的总结论为 **FAIL**，不能记为第一阶段通过：

- `health`、`static_cloud`、`dynamic_cloud` 的墙钟接收速率约为 3.658–3.659 Hz，低于合同要求的 5.0 Hz。
- 三类输出的墙钟最大消息间隔约为 0.382–0.384 秒，超过 0.250 秒上限。
- ROS 仿真时间下的话题频率稳定为 5.000 Hz，Gazebo 实时因子约为 0.63–0.73。这能解释为什么按 ROS 时间的 5 Hz 定时器在墙钟观测中只有约 3.66 Hz，但不改变合同检查失败的结论。

基础功能链同时给出了可用证据：

- 四项健康标志均为 `true`，`fault_code` 为 `OK`。
- 深度图和 `CameraInfo` 来自 Gazebo，`global_odom` 来自 `multi_drone_pose_transformer`，人物检测框来自 `yolo11n`，四类输入均已连通。
- `planner_depth` 有唯一发布者，实际消息坐标系为传感器坐标系 `depth_camera_base`；`health`、`static_cloud`、`dynamic_cloud`、`clearance` 的输出坐标系为 `map`。
- `local_mapping` 未发布控制话题。

因此，当前只能说“六机环境中 0 号机局部地图基础功能链健康，但墙钟性能验收失败”。EGO 轨迹接入、静态障碍绕行、故障注入和《单机局部建图与障碍导航设计》第一阶段完整验收都尚未完成。

`LOCAL_MAPPING_NAVIGATION_DESIGN.md` 页首的“尚未实现”是第一阶段整体状态，且成文早于本次局部地图实现。后续 Agent 判断实现进度时以本节为准：局部地图基础已实现并取得上述六机环境证据，但该设计文档定义的完整第一阶段仍未通过。

### 停止与清理

- 单独启动的 `local_mapping` 正常退出；主启动器 `1.sh` 按 `Ctrl-C` 后退出码为 130。
- 停止后，本次 Gazebo、PX4、`roslaunch`、MAVROS 和 XTDrone 通信相关进程均无残留，本次临时目录无残留，XTDrone 工作树状态为空。
- 原始 `~/.ros` 日志只作本机诊断证据，不得提交到仓库。

### 官方输入保护

本轮 Task 7B 未修改 PX4、XTDrone、Gazebo、EGO-Planner-Swarm、官方无人机模型或官方 World。本次 `1.sh` 真实运行在进入主启动流程前，已由启动器校验 PX4、XTDrone、Gazebo 模型和外部 Python 环境根目录均为独立只读挂载；官方 World 位于受保护的 PX4 树中。XTDrone 在运行前后的 `git status --short` 均为空。

针对当前 HEAD，主 Agent 已在系统环境完整运行 `bash scripts/verify_competition_clean.sh`，退出码为 0：仓库 Python 137 项测试通过；Catkin 最终汇总 373 项，0 errors、0 failures、0 skipped。构建前静态合规证据为 `competition-artifacts/static-compliance.json`，构建后合规证据为 `competition-artifacts/post-build-compliance.json`，两个证据文件的 SHA-256 均为 `9166b6474cc93f3b08fd61d29a15b0a545aef45e2805456973ba2d595b904cdf`。完整验证后 XTDrone 的 `git status --short` 仍为空。
