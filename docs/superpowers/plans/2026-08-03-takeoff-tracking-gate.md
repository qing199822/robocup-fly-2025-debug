# 起飞完成与人物跟踪门控 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立失效默认关闭的 `/swarm/takeoff_complete` 契约，保证六机起飞并完整交权前 tracking 不能锁定人物或切换 MUX。

**Architecture:** `fly_takeoff` 是全机起飞状态的唯一事实来源，启动时锁存发布 `false`，仅在六机全部到高且 navigator 交权全部成功后发布 `true` 并保持节点存活。每个 tracking 节点订阅该全局话题，将门控放在状态机更新入口；关闭时释放目标并清空跟踪状态，但不切换 MUX。既有三路 MUX、后置 `safety_filter` 和最终发布者所有权保持不变。

**Tech Stack:** Ubuntu 20.04、ROS Noetic、Catkin、C++14、`roscpp`、`std_msgs/Bool`、`topic_tools/MuxSelect`、Python 3 `unittest`、Rostest、Bash。

---

## 范围与完成定义

本计划只实现 [起飞完成与人物跟踪门控设计](../specs/2026-08-03-takeoff-tracking-gate-design.md)。不接入 EGO、地图、前沿、任务分配，不修改 PX4、XTDrone、Gazebo、第三方插件或官方模型。

完成时必须满足：

1. `/swarm/takeoff_complete` 的唯一发布者是 `/confident_takeoff_node`；
2. 起飞节点首先锁存发布 `false`；
3. 仅在全机到高且 navigator 交权全部成功后发布 `true`；
4. 成功后起飞节点不再发送命令，但保持存活以保存锁存状态；
5. tracking 在没有收到 `true` 时不申请目标、不暂停任务、不切换 MUX；
6. 门控从开变关时只释放一次目标并清空状态，不主动切换 MUX；
7. 最终速度话题仍只有对应 `safety_filter` 一个队伍发布者；
8. 自动测试、完整验证和真实六机 smoke 全部通过。

## 文件结构

### 新建

```text
src/mix_nav/fly/test/takeoff_gate_success.test
src/mix_nav/fly/test/takeoff_gate_timeout.test
src/mix_nav/fly/test/takeoff_gate_handoff_failure.test
src/mix_nav/fly/test/test_takeoff_gate.py
src/tracking/test/takeoff_gate.test
src/tracking/test/test_takeoff_gate.py
```

### 修改

```text
src/mix_nav/fly/include/fly/fly_takeoff.h
src/mix_nav/fly/src/fly_takeoff.cpp
src/mix_nav/fly/CMakeLists.txt
src/mix_nav/fly/package.xml
src/tracking/include/tracking/state_machine.h
src/tracking/src/state_machine.cpp
src/tracking/src/tracking_node.cpp
src/tracking/CMakeLists.txt
src/tracking/package.xml
tests/test_verification_scripts.py
scripts/smoke_competition_clean.sh
docs/AI_AGENT_HANDOFF.md
docs/COMPLIANCE.md
src/ego_fusion_search/safety_filter/README.md
```

不创建第四路 MUX 输入，不新增最终速度发布者。

### Task 1: 用 Rostest 固化起飞状态发布契约

**Files:**
- Create: `src/mix_nav/fly/test/test_takeoff_gate.py`
- Create: `src/mix_nav/fly/test/takeoff_gate_success.test`
- Create: `src/mix_nav/fly/test/takeoff_gate_timeout.test`
- Create: `src/mix_nav/fly/test/takeoff_gate_handoff_failure.test`
- Modify: `src/mix_nav/fly/CMakeLists.txt`
- Modify: `src/mix_nav/fly/package.xml`
- Test: `src/mix_nav/fly/test/test_takeoff_gate.py`

- [ ] **Step 1: 写可复用的 ROS 行为测试夹具**

`test_takeoff_gate.py` 必须创建两架测试无人机的 MUX mock 服务，持续发布测试位姿，并记录 `/swarm/takeoff_complete`。核心回调固定为：

```python
def _mux_callback(self, request, drone_id):
    self.selected_topics.append((drone_id, request.topic))
    if self.fail_handoff_id == drone_id and request.topic.endswith(
        "/mux_inputs/navigator/cmd_vel"
    ):
        raise rospy.ServiceException("planned navigator handoff failure")
    return MuxSelectResponse(prev_topic="")

def _status_callback(self, message):
    self.status_history.append(message.data)
```

成功用例必须先观察到 `false`，再发布两架高度 `3.1` 米的 `PoseStamped`，等待 `true`，随后新建一个晚到订阅者并断言其立即收到锁存的 `true`。超时用例不发布位姿，等待节点按测试参数退出，并断言历史中没有 `true`。交权失败用例发布到高位姿，让 1 号机 navigator MUX mock 抛出服务异常，断言没有 `true` 且两架都出现回滚到 takeoff 的选择记录。

- [ ] **Step 2: 写三个最小 Rostest Launch**

三个 `.test` 都启动：

```xml
<node pkg="fly" type="fly_takeoff" name="confident_takeoff_node"
      args="test_drone 2 3.0" output="screen">
  <param name="startup_delay" value="0.0"/>
  <param name="takeoff_timeout" value="0.3"/>
</node>
```

并分别向测试节点设置 `mode=success`、`mode=timeout`、`mode=handoff_failure`。每个测试的 `time-limit` 为 10 秒。

- [ ] **Step 3: 接入 Catkin 测试依赖**

在 `CMakeLists.txt` 的测试块加入：

```cmake
find_package(rostest REQUIRED)
add_rostest(test/takeoff_gate_success.test)
add_rostest(test/takeoff_gate_timeout.test)
add_rostest(test/takeoff_gate_handoff_failure.test)
```

在 `package.xml` 加入：

```xml
<test_depend>rospy</test_depend>
<test_depend>rostest</test_depend>
```

- [ ] **Step 4: 构建测试并确认失败原因正确**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_fly
catkin_test_results build/test_results/fly
```

Expected: 新增 Rostest 失败，因为 `/swarm/takeoff_complete` 尚无发布者；既有 fly 测试仍通过。

### Task 2: 实现起飞完成状态的唯一发布者

**Files:**
- Modify: `src/mix_nav/fly/include/fly/fly_takeoff.h`
- Modify: `src/mix_nav/fly/src/fly_takeoff.cpp`
- Test: `src/mix_nav/fly/test/test_takeoff_gate.py`

- [ ] **Step 1: 声明发布器、参数与唯一发布方法**

在 `ConfidentTakeoff` 中增加：

```cpp
#include <std_msgs/Bool.h>

void publishTakeoffComplete(bool complete);

ros::Publisher takeoff_complete_pub_;
double startup_delay_ = 2.0;
double takeoff_timeout_ = 15.0;
```

删除原来的常量 `timeout`，所有超时判断改用 `takeoff_timeout_`。

- [ ] **Step 2: 在构造阶段发布失效关闭状态**

构造函数加载测试可覆盖、生产默认不变的参数：

```cpp
ros::NodeHandle private_nh("~");
private_nh.param("startup_delay", startup_delay_, 2.0);
private_nh.param("takeoff_timeout", takeoff_timeout_, 15.0);
takeoff_complete_pub_ =
    nh_.advertise<std_msgs::Bool>("/swarm/takeoff_complete", 1, true);
publishTakeoffComplete(false);
```

节点内 `<param>` 位于 private namespace，因此参数必须通过 `private_nh` 读取；不能用当前全局 `nh_` 读取。实现发布方法：

```cpp
void ConfidentTakeoff::publishTakeoffComplete(bool complete) {
    std_msgs::Bool message;
    message.data = complete;
    takeoff_complete_pub_.publish(message);
}
```

将固定两秒等待替换为 `ros::Duration(startup_delay_).sleep()`。

- [ ] **Step 3: 只在完整交权后开放并保持锁存发布者存活**

保留所有现有失败 `return`。仅在最后一架 navigator MUX 切换成功后执行：

```cpp
publishTakeoffComplete(true);
ROS_INFO("Cluster mission completed! Control has been handed over.");
ROS_INFO("Takeoff gate is open; keeping latched status publisher alive.");
ros::spin();
```

`ros::spin()` 期间不得再次发布速度、OFFBOARD、ARM 或 HOVER 命令。

- [ ] **Step 4: 运行 fly 测试并确认转绿**

Run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_fly
catkin_test_results build/test_results/fly
```

Expected: success、timeout、handoff_failure 和既有测试全部通过，`0 errors, 0 failures`。

- [ ] **Step 5: 提交起飞门控发布端**

```bash
git add src/mix_nav/fly/CMakeLists.txt \
  src/mix_nav/fly/package.xml \
  src/mix_nav/fly/include/fly/fly_takeoff.h \
  src/mix_nav/fly/src/fly_takeoff.cpp \
  src/mix_nav/fly/test/test_takeoff_gate.py \
  src/mix_nav/fly/test/takeoff_gate_success.test \
  src/mix_nav/fly/test/takeoff_gate_timeout.test \
  src/mix_nav/fly/test/takeoff_gate_handoff_failure.test
git commit -m "feat: publish swarm takeoff readiness"
```

### Task 3: 用 Rostest 固化 tracking 的失效关闭行为

**Files:**
- Create: `src/tracking/test/test_takeoff_gate.py`
- Create: `src/tracking/test/takeoff_gate.test`
- Modify: `src/tracking/CMakeLists.txt`
- Modify: `src/tracking/package.xml`
- Test: `src/tracking/test/test_takeoff_gate.py`

- [ ] **Step 1: 建立 tracking 所需的全部 mock 服务**

测试节点必须在 tracking 启动时提供：

```text
/test_drone_0/pose_cmd_mux/select
/lookup/request_{green0,blue1,brown2,white3,red4,person}
/lookup/release_{green0,blue1,brown2,white3,red4,person}
```

request、release 和 MUX 回调分别记录调用次数与参数并返回成功。测试持续发布 `/test_drone_0/mavros/local_position/pose`、`velocity_body` 和带 `Class="green0"` 的 bounding box。

- [ ] **Step 2: 写完整门控行为序列**

同一个测试严格执行：

```python
self.gate_pub.publish(Bool(data=False))
self._publish_inputs_for(0.5)
self.assertEqual([], self.requested_targets)
self.assertEqual([], self.selected_topics)
self.assertEqual([], self.mission_commands)

self.gate_pub.publish(Bool(data=True))
self._wait_for(lambda: self.requested_targets == ["green0"])
self._wait_for(lambda: any(topic.endswith("/external/pose_cmd")
                           for topic in self.selected_topics))

switch_count = len(self.selected_topics)
self.gate_pub.publish(Bool(data=False))
self._wait_for(lambda: self.released_targets == ["green0"])
self.gate_pub.publish(Bool(data=False))
self._publish_inputs_for(0.5)
self.assertEqual(["green0"], self.released_targets)
self.assertEqual(switch_count, len(self.selected_topics))
self.assertEqual(["green0"], self.requested_targets)
```

Launch 将 `state_machine/confirmation_duration` 设为 `0.0`，避免测试依赖实际 0.25 秒延迟。

- [ ] **Step 3: 接入 Rostest 并确认先失败**

在 tracking `CMakeLists.txt` 测试块加入 `find_package(rostest REQUIRED)` 和 `add_rostest(test/takeoff_gate.test)`；在 `package.xml` 加 `rospy`、`rostest` 测试依赖。

Run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_tracking
catkin_test_results build/test_results/tracking
```

Expected: 测试失败，表现为门控为 `false` 时仍调用 `/lookup/request_green0`。

### Task 4: 在 tracking 状态机入口实现门控

**Files:**
- Modify: `src/tracking/include/tracking/state_machine.h`
- Modify: `src/tracking/src/state_machine.cpp`
- Modify: `src/tracking/src/tracking_node.cpp`
- Test: `src/tracking/test/test_takeoff_gate.py`

- [ ] **Step 1: 增加状态机公开入口和默认关闭状态**

在 `TrackingStateMachine` 公共接口增加：

```cpp
void setTakeoffComplete(bool complete);
```

私有区增加：

```cpp
void resetForClosedTakeoffGate();
bool takeoff_complete_ = false;
```

- [ ] **Step 2: 在 update 最前面失效关闭**

`update()` 在时间、目标和状态处理前执行：

```cpp
if (!takeoff_complete_) {
    return;
}
```

因此关闭状态下不会进入 `handleIdleState()`，也不会请求人物目标。

- [ ] **Step 3: 实现幂等状态切换和关闭清理**

```cpp
void TrackingStateMachine::setTakeoffComplete(bool complete) {
    if (takeoff_complete_ == complete) {
        return;
    }
    takeoff_complete_ = complete;
    if (!takeoff_complete_) {
        resetForClosedTakeoffGate();
    }
}
```

`resetForClosedTakeoffGate()` 在目标 ID 非空时调用一次 `releaseTarget()`，然后清空目标 ID、Kalman filter、计时器、丢失帧计数、高度降低标志并设置 `current_state_ = State::IDLE`。该方法不得调用 `switchControl()`、`pauseMission()`、发布 `RESUME` 或发布速度命令。

- [ ] **Step 4: 在 tracking 节点接入全局 Bool 话题**

在 `tracking_node.cpp` 引入 `std_msgs/Bool.h`，新增 subscriber 和回调：

```cpp
takeoff_complete_sub_ = nh_.subscribe<std_msgs::Bool>(
    "/swarm/takeoff_complete", 1,
    &TrackingNode::takeoffCompleteCallback, this);

void takeoffCompleteCallback(const std_msgs::Bool::ConstPtr& msg) {
    state_machine_->setTakeoffComplete(msg->data);
}
```

订阅使用绝对话题，避免 private NodeHandle 将其解析成每机不同的话题。

- [ ] **Step 5: 运行 tracking 门控测试并确认转绿**

Run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_tracking
catkin_test_results build/test_results/tracking
```

Expected: 门控关闭、开放、回退和重复 `false` 的行为序列全部通过，`0 errors, 0 failures`。

- [ ] **Step 6: 提交 tracking 门控订阅端**

```bash
git add src/tracking/CMakeLists.txt src/tracking/package.xml \
  src/tracking/include/tracking/state_machine.h \
  src/tracking/src/state_machine.cpp src/tracking/src/tracking_node.cpp \
  src/tracking/test/test_takeoff_gate.py \
  src/tracking/test/takeoff_gate.test
git commit -m "feat: gate tracking until swarm takeoff"
```

### Task 5: 将门控加入运行态 smoke

**Files:**
- Modify: `tests/test_verification_scripts.py`
- Modify: `scripts/smoke_competition_clean.sh`
- Test: `tests/test_verification_scripts.py`

- [ ] **Step 1: 先写 smoke 的失败行为测试**

扩展 fake `rostopic`：对 `/swarm/takeoff_complete` 输出 `data: true`；当 `FAIL_KIND=takeoff_gate` 时输出 `data: false`。新增断言：正常 smoke 报告包含 `PASS takeoff gate /swarm/takeoff_complete`，false 时非零退出且不得输出最终总 PASS。

- [ ] **Step 2: 运行单测并确认失败**

Run:

```bash
python3 -m unittest \
  tests.test_verification_scripts.VerificationScriptsBehaviorTest -v
```

Expected: 新增起飞门控 smoke 测试失败，因为脚本尚未读取 Bool 值。

- [ ] **Step 3: 实现精确 Bool 检查**

在 smoke 中新增：

```bash
check_true_boolean() {
    local topic="$1" output
    if ! output="$(timeout "${SMOKE_TIMEOUT_SECONDS}s" \
        rostopic echo -n 1 "$topic" 2>/dev/null)" \
        || ! grep -Eq '^data:[[:space:]]+true$' <<<"$output"; then
        log_line "FAIL takeoff gate $topic" >&2
        return 1
    fi
    log_line "PASS takeoff gate $topic"
}
```

六机基础话题检查完成后调用：

```bash
check_true_boolean "/swarm/takeoff_complete"
```

- [ ] **Step 4: 运行 smoke 单测和脚本语法检查**

Run:

```bash
python3 -m unittest tests.test_verification_scripts -v
bash -n scripts/smoke_competition_clean.sh
```

Expected: 全部通过，脚本语法检查退出 0。

- [ ] **Step 5: 提交运行态守卫**

```bash
git add tests/test_verification_scripts.py scripts/smoke_competition_clean.sh
git commit -m "test: require open takeoff gate in smoke"
```

### Task 6: 更新交接和合规文档

**Files:**
- Modify: `docs/AI_AGENT_HANDOFF.md`
- Modify: `docs/COMPLIANCE.md`
- Modify: `src/ego_fusion_search/safety_filter/README.md`

- [ ] **Step 1: 补充控制链说明**

在三份现有未提交文档中统一加入：

```text
/swarm/takeoff_complete 默认 false；只有六机到高且全部交权给 navigator 后才 true。
tracking 在 false 时不能锁目标或选择 external；false 回退只释放目标，不切换 MUX。
```

明确 `confident_takeoff_node` 成功后保持空闲存活是为了保存 ROS 1 锁存状态，不代表它继续控制无人机。

- [ ] **Step 2: 检查文档和改动边界**

Run:

```bash
git diff --check
git diff --name-only public/competition-clean
```

Expected: 没有空白错误；修改只位于队伍 ROS 包、队伍脚本、测试和文档，不出现 PX4、XTDrone、Gazebo、EGO 或官方模型路径。

- [ ] **Step 3: 提交此前保留的 Task 7 文档**

```bash
git add docs/AI_AGENT_HANDOFF.md docs/COMPLIANCE.md \
  src/ego_fusion_search/safety_filter/README.md
git commit -m "docs: explain gated swarm control flow"
```

### Task 7: 完整自动验证

**Files:**
- Verify only; no source edits expected.

- [ ] **Step 1: 运行快速 Python 回归**

Run:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 -m unittest discover -s src/competition_compliance/test -p 'test_*.py'
```

Expected: 全部通过；测试总数不得少于改动前的仓库 127 和 ownership 39。

- [ ] **Step 2: 运行完整 competition-clean 验证**

Run:

```bash
export PX4_DIR=/home/wangtao/robocup_fly/PX4_Firmware
export XTDRONE_DIR=/home/wangtao/robocup_fly/XTDrone
export GAZEBO_MODELS_DIR=/home/wangtao/robocup_fly/gazebo_models
export XTDRONE_PYTHONPATH=/home/wangtao/robocup_fly/.xtdrone-python
export YOLO_PYTHON=/home/wangtao/robocup_fly/.venv-yolo/bin/python
export YOLO_CONFIG_DIR=/home/wangtao/robocup_fly/.ultralytics
bash scripts/verify_competition_clean.sh
```

Expected: Release 构建成功、Catkin 测试 `0 errors, 0 failures`，结尾为 `完整验证通过：静态与构建后合规证据均已生成。`

- [ ] **Step 3: 核对官方输入没有变化**

Run:

```bash
git -C /home/wangtao/robocup_fly/XTDrone status --short
git diff --name-only public/competition-clean
```

Expected: XTDrone 状态为空；仓库差异不包含外部官方目录和逐字节核验的第三方包。

### Task 8: 真实六机验收与最终记录

**Files:**
- Modify only if evidence requires correction: `docs/AI_AGENT_HANDOFF.md`

- [ ] **Step 1: 启动真实六机任务**

Run in terminal A:

```bash
bash 1.sh 6 mission_down.json
```

Expected: 六机、相机、识别、队伍算法全部启动；门控日志先为关闭。

- [ ] **Step 2: 核对起飞期间没有跟踪抢权**

检查启动日志和 ROS 状态，必须同时满足：

```text
六架无人机均报告到达目标高度
起飞完成前没有 tracking 的目标锁定成功日志
起飞完成前没有 tracking 切换 external 的日志
/swarm/takeoff_complete 在全机 navigator 交权后才变为 true
```

- [ ] **Step 3: 运行正式 smoke**

Run in terminal B:

```bash
bash scripts/smoke_competition_clean.sh
```

Expected: 包含 `PASS takeoff gate /swarm/takeoff_complete`、最终发布者唯一性 PASS，最后一行为 `PASS competition-clean six-vehicle smoke`。

- [ ] **Step 4: 确认门控开放后 tracking 能正常工作**

在人物进入视野后确认 tracking 能申请目标并把对应无人机切到 external；目标丢失后仍能按原逻辑返回 navigator。不得通过修改人物、相机、官方 world 或 PX4 参数制造通过结果。

- [ ] **Step 5: 正常停止并检查残留**

在终端 A 只按一次 `Ctrl-C`。Expected: 启动器退出码 130；本次登记的进程被清理，无项目临时目录残留，不按进程名影响其他 ROS 会话。

- [ ] **Step 6: 记录真实验收结果**

若验收全部通过，在 `docs/AI_AGENT_HANDOFF.md` 记录日志文件名、六机到高、门控开放时序、tracking 接管和 smoke 结果；若失败，只记录真实失败证据并回到对应任务修复，不得宣称完成。

- [ ] **Step 7: 提交验收记录并检查分支**

```bash
git add docs/AI_AGENT_HANDOFF.md
git commit -m "docs: record gated six-drone acceptance"
git status --short --branch
git log --oneline --decorate -10
```

Expected: 工作树干净；`competition-clean` 只领先公开远端本轮经过验证的提交。未经用户明确要求不推送。
