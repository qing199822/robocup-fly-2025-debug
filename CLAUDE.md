# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

本仓库文档与注释均为中文，回复和新增文档请保持中文。

## 项目定位

2025 中国机器人大赛多旋翼无人机集群协同搜索仿真，固定 6 架 `typhoon_h480_realsense`。运行栈是 ROS Noetic + Gazebo Classic 11 + PX4 `v1.11.0-beta1` + XTDrone。当前分支 `competition-clean` 是参赛候选分支。

新会话开始前，先完整阅读 `docs/AI_AGENT_HANDOFF.md`（权威交接手册，含边界、验证流程和交接模板），再按任务读对应包的 `README.md`。其他权威文档：`docs/ENVIRONMENT.md`（版本与安装顺序）、`docs/COMPLIANCE.md`（合规边界与官方哈希）、`docs/THIRD_PARTY.md`、`docs/TROUBLESHOOTING.md`。

## 硬约束（改代码前必须知道）

- 外部只读输入：`~/robocup_fly/{PX4_Firmware,XTDrone,gazebo_models,.xtdrone-python,.venv-yolo}`。遇到兼容问题只能改队伍包或队伍脚本，禁止改这些目录来"适配"。
- 官方文件哈希只维护在 `src/competition_compliance/config/official_manifest.json`（+ `docs/COMPLIANCE.md`），不要在别处复制第二份哈希表。
- 文件归属查 `src/competition_compliance/config/ownership.json`。`src/darknet_ros_msgs` 和 `src/gazebo_ros_actor_plugin/*` 标记为 `third-party`，逐字节核验，不要随手改；需要适配时在队伍包边界加代码。
- 无人机基线模型不变。唯一允许的模型差异是 `src/competition_compliance/config/sensor_mount.yaml` 里的 6 元素 Realsense 安装位姿（x y z r p y）；生成模型必须仍只含一个 `model://realsense_camera` include，固定关节 parent 仍为 `base_link`。
- 所有任务航点高度必须低于 6 米。
- PX4 不要升到 1.13（该版本在本项目出现相机无图）。

## 环境变量与构建

外部依赖位置不要靠 `../` 推断，显式导出（脚本默认值取 `<workspace>/..`，worktree 下会指错）：

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
export PX4_DIR=${PX4_DIR:-$HOME/robocup_fly/PX4_Firmware}
export XTDRONE_DIR=${XTDRONE_DIR:-$HOME/robocup_fly/XTDrone}
export GAZEBO_MODELS_DIR=${GAZEBO_MODELS_DIR:-$HOME/robocup_fly/gazebo_models}
export XTDRONE_PYTHONPATH=${XTDRONE_PYTHONPATH:-$HOME/robocup_fly/.xtdrone-python}
export YOLO_PYTHON=${YOLO_PYTHON:-$HOME/robocup_fly/.venv-yolo/bin/python}
export YOLO_CONFIG_DIR=${YOLO_CONFIG_DIR:-$HOME/robocup_fly/.ultralytics}
source /opt/ros/noetic/setup.bash
```

构建（PX4 需先自行 `make px4_sitl_default gazebo`）：

```bash
catkin_init_workspace src
catkin_make -DCMAKE_BUILD_TYPE=Release
bash scripts/build_xtdrone_actor_collisions.sh   # 从只读 XTDrone 输入构建到本工作区
```

`build/` 和 `devel/` 记录绝对路径。目录搬迁后先 `rm -rf build devel` 再重新完整验证。

## 常用命令

运行（需要真实桌面图形会话，终端 A）：

```bash
bash 1.sh 6 mission_down.json    # 任务文件只传文件名，候选见 waypoint/*.json
```

只按一次 `Ctrl-C` 停止，预期退出码 `130`；不要关终端、不要按名字批量杀 ROS 进程（同机可能跑着别的项目）。

运行态六机检查（终端 B，相机就绪后）：

```bash
bash scripts/smoke_competition_clean.sh   # 最后一行须为 PASS competition-clean six-vehicle smoke
```

完整验证（静态合规 → Python 单测 → Release catkin_make → 外置插件构建 → catkin run_tests → 构建后再次合规）：

```bash
bash scripts/verify_competition_clean.sh  # 结尾：完整验证通过：静态与构建后合规证据均已生成。
```

证据落在 `competition-artifacts/{static-compliance,post-build-compliance}.json`，日志在 `logs/competition-clean/` 和 `logs/verification/`。

单项测试：

```bash
python3 -m unittest discover -s tests -p 'test_*.py'          # 全仓 Python 回归
python3 -m unittest tests.test_one_click_launch -v            # 单个测试模块
python3 -m unittest tests.test_camera_geometry.TestX.test_y   # 单个用例
python3 src/mix_nav/task_manager/test/test_mission_clearance.py
python3 src/mix_nav/fly/test/test_fly_launch.py
rostest simple_navigator velocity_continuity.test             # 需先 source devel/setup.bash
rostest pose_init pose_namespace.test
```

包内 Python 测试（`src/*/test/test_*.py`）由 `catkin_make run_tests` 驱动，`catkin_test_results` 汇总。

脚本改动附加检查：

```bash
bash -n 1.sh scripts/*.sh src/yolo/*.sh
python3 -m py_compile scripts/process_supervisor.py
git diff --check
```

完整验证不能代替六机 smoke；涉及运行流程的改动两者都要跑。

## 架构要点

启动编排集中在 `1.sh`（约 1500 行，是真正的一键入口，`2.sh`/`3.sh`/`bp.sh` 是旧 tmux/docker 版本）。顺序：

```
快速合规预检 + prepare_model.py 在本次运行的 /tmp/robocup-fly-competition-clean.* 生成 Realsense 模型
  -> robocup_zzufly.launch: Gazebo + 6×PX4 SITL + 6×MAVROS
  -> 6× XTDrone multirotor_communication.py
  -> 6 组 RGB/depth/CameraInfo 就绪检查
  -> src/yolo/multi_yolo_detecting.sh（6 worker）+ multi_solving.sh
  -> roslaunch look_up down_resume.launch（队伍算法总启动，见该文件的 12 步顺序）
```

关键设计：

- **进程生命周期**：`1.sh` 登记每个子进程的 PID/PGID/启动时间，`scripts/process_supervisor.py` 作为 Linux subreaper 接管脱离会话的后代，TERM 宽限后再 KILL，且只处理身份仍匹配的进程。清理失败保留状态 `125`。改动清理逻辑必须同时验证"本次进程被清掉"和"无关会话仍存活"，不要退回按名称杀进程。
- **图形环境**：`scripts/graphics_environment.sh` 从 `gnome-shell`/`plasmashell` 的 `/proc/<pid>/environ` 导入 `DISPLAY`/`XAUTHORITY`。相机无图先查这里，不要误判为 YOLO 订阅问题。
- **控制命令必须走 MUX**：`tracking` 发到 `/typhoon_h480_N/mux_inputs/external/pose_cmd`，`simple_navigator` 发到 `.../mux_inputs/navigator/cmd_vel`，由 `pose_cmd_mux`（`look_up/launch/spawn_mux_swarm.launch`）择一转发到 `/xtdrone/typhoon_h480_N/cmd_vel_flu`。跟踪节点不得直接成为 XTDrone 速度话题的第二个发布者；`tracking/src/service_manager.cpp` 通过 `pose_cmd_mux/select` 切换。
- **感知链路**：`src/yolo/yolo11n.py` → `actor_msgs/ActorInfo` → `bbox2coord_node.py`（融合 depth + CameraInfo + TF）→ 目标三维位置 → `look_up` / `tracking`。`tracking` 会发布状态与当前目标，坐标节点据此过滤，避免误发布到裁判系统。
- **定位**：比赛不能订阅位姿真值，`pose_init` 由 `/mavros/local_position/{pose,odom}` 加已知初始位置解算出 `/global_pose`、`/global_odom`。禁止占用 `/mavros/vision_*` 名称（那是发给飞控的外部视觉输入，会污染 PX4 估计器）。
- **目标互斥**：`look_up/src/target_lookup_service.cpp` 提供锁定查询服务，一个目标同一时间只能被一架飞机锁定；两个相同 `red` 走"先请求 red4 再 red5"的顺序。
- **任务与断点返航**：`task_manager` 发布航点，跟踪期间每 3 秒压栈记录位置，目标消失后逐点弹栈返回断点，实现返航避障。任务 JSON 在 `waypoint/` 和包内 launch 目录，launch 用相对路径读取。
- **TF**：静态相机外参由 `competition_compliance/launch/sensor_tf.launch` + `simple_navigator/launch/static_tf.launch` 发布，动态部分在 `transform_tree`。smoke 检查 `base_link -> depth_camera_base` 可查询。
- **合规包**：`src/competition_compliance/scripts/verify_full.py` 是唯一权威验证器（manifest 哈希 + ownership + 生成模型契约），`prepare_model.py` 负责临时模型生成，两者被 `1.sh` 和 verifier 复用。

## 工作流约定

- 行为改动先写能复现缺陷的失败测试，再做最小修复；测真实行为，不要断言源码含某个字符串。
- 改包前读该包的 `README.md`、`package.xml`、`CMakeLists.txt` 和现有测试，不要凭文件名推接口。
- 只提交任务内文件（`git add <明确文件>`，不用通配全加）；不提交 `build/`、`devel/`、日志、虚拟环境、PX4/官方模型。
- 只在用户明确要求时推送：`git push -u public competition-clean`，不强推，不移动 `public/main`。
- 退出后按 `docs/AI_AGENT_HANDOFF.md` 的清单核验进程残留、`/tmp` 临时目录、`git -C "$XTDRONE_DIR" status --short` 为空。
- 交接结果按手册末尾模板书写；未执行的验证必须写明"未执行"及原因。
