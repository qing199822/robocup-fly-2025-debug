# 固定巡逻真实飞行失败修复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复六机固定巡逻中 4 号机撞房屋和 5 号机在任务启动前被跟踪抢占的两个已闭合根因，并用自动化测试与真实六机证据验证修复。

**Architecture:** 静态路线继续由 `task_manager` 的任务 JSON 定义，几何测试补齐官方世界中的两栋 `house_1` 碰撞边界并约束 4 号机绕行。`MissionManager` 发布车辆级锁存状态 `/车辆/mission/active`，`tracking` 将它与现有 `/swarm/takeoff_complete` 组成双门控；只有两者均为真时才允许请求目标、发送 `PAUSE` 和切换 external 控制。

**Tech Stack:** Ubuntu 20.04、ROS Noetic、C++14/roscpp、Python 3/rospy、rostest、Catkin、Gazebo Classic 11、JSON、unittest

---

## 文件结构与边界

- `src/mix_nav/task_manager/test/test_mission_clearance.py`：保存比赛世界静态障碍边界，验证完整进入段、巡逻闭环和跨机中心线净空。
- `src/mix_nav/task_manager/launch/mission_down.json`：固定六机任务的运行时唯一权威文件；本轮只调整 4 号机路线。
- `src/mix_nav/task_manager/include/task_manager/mission_manager.h`、`src/mix_nav/task_manager/src/mission_manager.cpp`：发布任务是否已经越过启动等待、进入可被跟踪暂停的活动阶段。
- `src/mix_nav/task_manager/test/mission_active.json`、`mission_active.test`、`test_mission_active.py`：用单机 rostest 验证锁存状态先假后真。
- `src/mix_nav/task_manager/CMakeLists.txt`、`package.xml`：登记 rostest 及测试依赖。
- `src/tracking/include/tracking/state_machine.h`、`src/tracking/src/state_machine.cpp`、`src/tracking/src/tracking_node.cpp`：订阅任务状态并执行双门控复位。
- `src/tracking/test/test_takeoff_gate.py`：扩展现有运行态测试，覆盖两个布尔门控的四种组合和关闭复位。
- `docs/AI_AGENT_HANDOFF.md`：保留首次失败证据，并记录本轮复验结果和未解决风险。

所有修改均属于队伍自有包、任务配置、测试和文档。不得修改 `/home/wangtao/robocup_fly/PX4_Firmware`、`XTDrone`、`gazebo_models`、EGO-Planner-Swarm、第三方 actor 插件或官方无人机模型。不得提交 `.superpowers/`、构建产物、日志、rosbag 或接触流；本计划不包含推送远端。

### Task 1: 补齐房屋障碍并调整 4 号机路线

**Files:**
- Modify: `src/mix_nav/task_manager/test/test_mission_clearance.py`
- Modify: `src/mix_nav/task_manager/launch/mission_down.json`

- [ ] **Step 1: 将两栋 `house_1` 的保守碰撞边界加入测试**

在 `STATIC_OBSTACLES` 中加入由官方 `robocup.world` 位姿和 `house_1.dae` 碰撞网格计算并向外取整的边界：

```python
STATIC_OBSTACLES = (
    # 现有障碍保持不变。
    ("house_1_146", -34.34, -21.40, 17.49, 33.99, 7.69),
    ("house_1_146_clone", 6.15, 19.09, 11.19, 27.70, 7.69),
    # 其余现有障碍保持不变。
)
```

不缩小现有 `HORIZONTAL_CLEARANCE = 2.0`，也不改变房屋高度来放行旧路线。

- [ ] **Step 2: 运行几何测试并确认旧路线按目标原因失败**

Run:

```bash
python3 src/mix_nav/task_manager/test/test_mission_clearance.py -v
```

Expected: 非零退出，`test_all_complete_segments_clear_known_static_obstacles` 失败，输出包含：

```text
typhoon_h480_4 ... intersects house_1_146_clone
```

保存这次失败输出作为 TDD 红灯证据；若因其他原因失败，先修正测试本身，不修改任务路线掩盖错误。

- [ ] **Step 3: 把 4 号机改为已验证的南侧进入路线和北中巡逻矩形**

将 `typhoon_h480_4` 的任务完整替换为：

```json
{
  "vehicle_id": "typhoon_h480_4",
  "entry_waypoints": [
    {"x": 0.0, "y": 7.0, "z": 3.5},
    {"x": 25.0, "y": 7.0, "z": 3.5}
  ],
  "waypoints": [
    {"x": 25.0, "y": 12.0, "z": 3.5},
    {"x": 68.0, "y": 12.0, "z": 3.5},
    {"x": 68.0, "y": 41.0, "z": 3.5},
    {"x": 25.0, "y": 41.0, "z": 3.5}
  ]
}
```

- [ ] **Step 4: 验证障碍、限高、闭环和跨机 5 米净空全部通过**

Run:

```bash
python3 src/mix_nav/task_manager/test/test_mission_clearance.py -v
python3 -m json.tool src/mix_nav/task_manager/launch/mission_down.json >/dev/null
```

Expected: 所有几何测试通过，JSON 校验退出码为 0；不出现静态障碍碰撞或跨机净空违规。

- [ ] **Step 5: 检查并提交路线修复**

Run:

```bash
git diff --check
git diff -- src/mix_nav/task_manager/test/test_mission_clearance.py src/mix_nav/task_manager/launch/mission_down.json
git add src/mix_nav/task_manager/test/test_mission_clearance.py src/mix_nav/task_manager/launch/mission_down.json
git commit -m "fix: route north-central patrol around houses"
```

Expected: 提交只包含上述两个文件，`.superpowers/` 未被跟踪。

### Task 2: MissionManager 发布锁存的任务激活状态

**Files:**
- Create: `src/mix_nav/task_manager/test/mission_active.json`
- Create: `src/mix_nav/task_manager/test/mission_active.test`
- Create: `src/mix_nav/task_manager/test/test_mission_active.py`
- Modify: `src/mix_nav/task_manager/include/task_manager/mission_manager.h`
- Modify: `src/mix_nav/task_manager/src/mission_manager.cpp`
- Modify: `src/mix_nav/task_manager/CMakeLists.txt`
- Modify: `src/mix_nav/task_manager/package.xml`

- [ ] **Step 1: 创建最小单机任务夹具**

`mission_active.json` 使用一个进入点和三个巡逻点，满足现有任务定义校验：

```json
[
  {
    "vehicle_id": "test_drone_0",
    "entry_waypoints": [
      {"x": 1.0, "y": 0.0, "z": 3.5}
    ],
    "waypoints": [
      {"x": 2.0, "y": 0.0, "z": 3.5},
      {"x": 2.0, "y": 2.0, "z": 3.5},
      {"x": 0.0, "y": 2.0, "z": 3.5}
    ]
  }
]
```

- [ ] **Step 2: 编写先假后真的 rostest**

`mission_active.test` 启动真实 `multi_mission_launcher`，将生产默认 10 秒倒计时在测试中设为 0 秒：

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="task_manager" type="multi_mission_launcher"
        name="mission_manager_under_test"
        args="$(find task_manager)/test/mission_active.json test_drone_0"
        output="screen">
    <param name="startup_countdown_seconds" value="0.0"/>
  </node>
  <test test-name="mission_active" pkg="task_manager"
        type="test_mission_active.py" time-limit="15"/>
</launch>
```

`test_mission_active.py` 必须订阅锁存话题、先观察 `false`，再持续发布位姿直到观察到 `true`：

```python
#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class MissionActiveTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.states = []
        self.state_sub = rospy.Subscriber(
            "/test_drone_0/mission/active", Bool, self._state_callback,
            queue_size=10,
        )
        self.pose_pub = rospy.Publisher(
            "/test_drone_0/global_pose", PoseStamped, queue_size=1,
        )

    def _state_callback(self, message):
        with self.lock:
            self.states.append(message.data)

    def _snapshot(self):
        with self.lock:
            return list(self.states)

    def _wait_for(self, predicate, message, timeout=5.0, publish_pose=False):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if publish_pose:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)
            if predicate():
                return
            rate.sleep()
        self.fail(message)

    def test_active_is_latched_false_until_mission_starts(self):
        self._wait_for(
            lambda: False in self._snapshot(),
            "mission manager did not publish the initial false state",
        )
        self.assertNotIn(True, self._snapshot())
        self._wait_for(
            lambda: True in self._snapshot(),
            "mission manager did not publish true after startup",
            publish_pose=True,
        )


if __name__ == "__main__":
    rospy.init_node("mission_active_test")
    rostest.rosrun("task_manager", "mission_active_test", MissionActiveTest)
```

给脚本执行权限。在 `CMakeLists.txt` 的测试块中加入：

```cmake
find_package(rostest REQUIRED)
add_rostest(test/mission_active.test)
```

在 `package.xml` 中加入：

```xml
<test_depend>rospy</test_depend>
<test_depend>rostest</test_depend>
```

- [ ] **Step 3: 构建并运行测试，确认因缺少 active 话题而失败**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg task_manager
source devel/setup.bash
rostest task_manager mission_active.test
```

Expected: rostest 非零退出，失败信息为没有收到初始 `false`；不是编译、任务 JSON 或 ROS master 配置错误。

- [ ] **Step 4: 在 MissionManager 中加入最小发布接口和可测试倒计时参数**

头文件增加消息依赖、发布者、辅助函数和参数：

```cpp
#include <std_msgs/Bool.h>

void publishMissionActive(bool active);

ros::Publisher active_pub_;
double startup_countdown_seconds_ = 10.0;
```

构造函数中创建车辆级锁存发布者并立即发布 `false`：

```cpp
std::string active_topic = "/" + vehicle_id_ + "/mission/active";
active_pub_ = nh_.advertise<std_msgs::Bool>(active_topic, 1, true);
private_nh.param("startup_countdown_seconds",
                 startup_countdown_seconds_, 10.0);
publishMissionActive(false);
```

辅助函数只负责发布明确状态：

```cpp
void MissionManager::publishMissionActive(bool active) {
    std_msgs::Bool message;
    message.data = active;
    active_pub_.publish(message);
}
```

把固定 10 秒倒计时改为参数化的 ROS 时间等待；生产默认仍为 10 秒。完成位姿等待和倒计时后先进入活动状态，再发布 `true`：

```cpp
ROS_INFO("[%s] 系統就緒，%.1fs後啟動任務。",
         vehicle_id_.c_str(), startup_countdown_seconds_);
const ros::Time start_time = ros::Time::now();
while (ros::ok() &&
       (ros::Time::now() - start_time).toSec() < startup_countdown_seconds_) {
    ros::spinOnce();
    rate.sleep();
}

if (!ros::ok()) return;
state_ = activeState();
publishMissionActive(true);
```

`PAUSED` 和 `RESUMING` 不发布 `false`；进程退出也不伪造状态。不得把 `RESUME` 或回调顺序当成任务激活依据。

- [ ] **Step 5: 重建并验证 active 合同通过**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg task_manager
source devel/setup.bash
rostest task_manager mission_active.test
catkin_make run_tests_task_manager
catkin_test_results build/test_results/task_manager
```

Expected: `mission_active` 先收到 `false` 后收到 `true`，task_manager 全部测试 0 error、0 failure。

- [ ] **Step 6: 检查并提交任务激活接口**

Run:

```bash
git diff --check
git diff -- src/mix_nav/task_manager
git add src/mix_nav/task_manager/include/task_manager/mission_manager.h \
  src/mix_nav/task_manager/src/mission_manager.cpp \
  src/mix_nav/task_manager/test/mission_active.json \
  src/mix_nav/task_manager/test/mission_active.test \
  src/mix_nav/task_manager/test/test_mission_active.py \
  src/mix_nav/task_manager/CMakeLists.txt \
  src/mix_nav/task_manager/package.xml
git commit -m "feat: publish active mission state"
```

Expected: 提交不包含路线文件、构建目录或 `.superpowers/`。

### Task 3: tracking 使用起飞与任务双门控

**Files:**
- Modify: `src/tracking/test/test_takeoff_gate.py`
- Modify: `src/tracking/include/tracking/state_machine.h`
- Modify: `src/tracking/src/state_machine.cpp`
- Modify: `src/tracking/src/tracking_node.cpp`

- [ ] **Step 1: 将现有 rostest 扩展为四种门控组合**

在 `setUp()` 新增锁存发布者并把两个门控初始化为假：

```python
self.takeoff_gate_pub = rospy.Publisher(
    "/swarm/takeoff_complete", Bool, queue_size=1, latch=True
)
self.mission_gate_pub = rospy.Publisher(
    "/test_drone_0/mission/active", Bool, queue_size=1, latch=True
)
self.takeoff_gate_pub.publish(Bool(data=False))
self.mission_gate_pub.publish(Bool(data=False))
```

测试按顺序证明：

```python
self._publish_inputs_for(0.5)  # false / false
self.assertEqual([], self._snapshot(self.requested_targets))

self.takeoff_gate_pub.publish(Bool(data=True))  # true / false
self._publish_inputs_for(0.5)
self.assertEqual([], self._snapshot(self.requested_targets))

self.takeoff_gate_pub.publish(Bool(data=False))
self.mission_gate_pub.publish(Bool(data=True))  # false / true
self._publish_inputs_for(0.5)
self.assertEqual([], self._snapshot(self.requested_targets))

self.takeoff_gate_pub.publish(Bool(data=True))  # true / true
self._publish_until(
    lambda: self._snapshot(self.requested_targets) == ["green0"],
    "both gates open did not allow target request",
)
```

保留现有 external MUX 和 `PAUSE` 断言。随后分别关闭任务门控和起飞门控，验证每次锁定目标只释放一次、关闭期间不产生新请求、MUX 切换或任务命令；重新打开两个门控后允许再次锁定，以证明状态机可以恢复工作。

- [ ] **Step 2: 运行现有 tracking rostest，确认仅起飞为真仍错误放行**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg tracking
source devel/setup.bash
rostest tracking takeoff_gate.test
```

Expected: 非零退出，`true / false` 阶段已经请求 `green0`，证明当前实现缺少任务激活门控。

- [ ] **Step 3: 给状态机增加 mission active 输入并统一关闭复位**

头文件公开新 setter，并将只针对起飞的命名改为双门控命名：

```cpp
void setTakeoffComplete(bool complete);
void setMissionActive(bool active);

void updateControlGateState(const char* gate_name);
void resetForClosedControlGate();

bool takeoff_complete_ = false;
bool mission_active_ = false;
```

`update()` 开头必须同时检查两个条件：

```cpp
if (!takeoff_complete_ || !mission_active_) {
    return;
}
```

两个 setter 只在值变化时更新状态，再调用统一辅助函数。只有两个门控都打开时重置 `last_update_time_` 并记录开放；任一关闭时释放当前目标并完整复位：

```cpp
void TrackingStateMachine::setMissionActive(bool active) {
    if (mission_active_ == active) {
        return;
    }
    mission_active_ = active;
    updateControlGateState("mission active");
}

void TrackingStateMachine::updateControlGateState(const char* gate_name) {
    if (takeoff_complete_ && mission_active_) {
        last_update_time_ = ros::Time::now();
        ROS_INFO("[%s_%s Tracker] Control gates opened after %s update.",
                 vehicle_type_.c_str(), vehicle_id_.c_str(), gate_name);
        return;
    }
    ROS_WARN("[%s_%s Tracker] Control gate closed after %s update; resetting tracking state.",
             vehicle_type_.c_str(), vehicle_id_.c_str(), gate_name);
    resetForClosedControlGate();
}
```

`resetForClosedControlGate()` 复用现有 `resetForClosedTakeoffGate()` 的全部字段复位，不切换 MUX、不发送 `RESUME`、不发送 `PAUSE`。若当前目标为空，重复关闭不得调用 release 服务。

- [ ] **Step 4: 在 tracking 节点订阅车辆对应的任务状态**

在 `setupSubscribers()` 中使用现有 `ns` 订阅：

```cpp
mission_active_sub_ = nh_.subscribe<std_msgs::Bool>(
    ns + "/mission/active", 1,
    &TrackingNode::missionActiveCallback, this);
```

新增回调和成员：

```cpp
void missionActiveCallback(const std_msgs::Bool::ConstPtr& msg) {
    state_machine_->setMissionActive(msg->data);
}

ros::Subscriber mission_active_sub_;
```

话题必须是 `/typhoon_h480_N/mission/active`，不能放在 tracking 私有命名空间，也不能从超时推断为真。

- [ ] **Step 5: 重建并验证双门控及现有 tracking 测试**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg tracking
source devel/setup.bash
rostest tracking takeoff_gate.test
catkin_make run_tests_tracking
catkin_test_results build/test_results/tracking
```

Expected: 四种组合只有 `true / true` 请求目标并发送 `PAUSE`；任一关闭均只释放一次且不产生新控制动作；tracking 全部测试 0 error、0 failure。

- [ ] **Step 6: 检查并提交双门控修复**

Run:

```bash
git diff --check
git diff -- src/tracking
git add src/tracking/include/tracking/state_machine.h \
  src/tracking/src/state_machine.cpp \
  src/tracking/src/tracking_node.cpp \
  src/tracking/test/test_takeoff_gate.py
git commit -m "fix: gate tracking on active missions"
```

Expected: 提交只包含 tracking 队伍包内四个文件。

### Task 4: 运行自动化回归与完整 competition-clean 验证器

**Files:**
- Verify only; do not commit generated `build/`, `devel/`, `competition-artifacts/` or logs

- [ ] **Step 1: 运行聚焦几何和三个相关包测试**

Run:

```bash
python3 src/mix_nav/task_manager/test/test_mission_clearance.py -v
source /opt/ros/noetic/setup.bash
catkin_make --pkg task_manager tracking safety_filter
catkin_make run_tests_task_manager run_tests_tracking run_tests_safety_filter
catkin_test_results build/test_results
```

Expected: 几何测试全过；三个包汇总 0 error、0 failure。

- [ ] **Step 2: 显式指定只读外部输入并运行完整 verifier**

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

Expected: 结尾为 `完整验证通过：静态与构建后合规证据均已生成。`。若工具隔离禁止读取网络接口，应按权限流程在本机环境重跑同一命令，不能把隔离失败写成项目通过。

- [ ] **Step 3: 检查自动化验证没有污染官方输入或提交范围**

Run:

```bash
git -C "$XTDRONE_DIR" status --short
git status --short --branch
git diff --check
```

Expected: XTDrone 输出为空；仓库只保留未跟踪 `.superpowers/` 和预期生成物，不把生成物加入 Git。

### Task 5: 重新执行真实六机全航程验收

**Files:**
- Runtime evidence only under `/tmp` and `logs/competition-clean/`; do not commit raw evidence

- [ ] **Step 1: 启动前记录基线并准备证据采集命令**

记录项目相关进程和临时目录基线。rosbag 必须覆盖：

```text
/swarm/takeoff_complete
/typhoon_h480_0..5/mission/active
/typhoon_h480_0..5/global_pose
/typhoon_h480_0..5/global_odom
/typhoon_h480_0..5/safety/status
```

在主启动进入任务节点阶段、起飞门控尚未打开时启动采集；将范围展开为六架飞机的明确话题：

```bash
rosbag record -O /tmp/static-patrol-revalidation.bag \
  /swarm/takeoff_complete \
  /typhoon_h480_0/mission/active /typhoon_h480_1/mission/active \
  /typhoon_h480_2/mission/active /typhoon_h480_3/mission/active \
  /typhoon_h480_4/mission/active /typhoon_h480_5/mission/active \
  /typhoon_h480_0/global_pose /typhoon_h480_1/global_pose \
  /typhoon_h480_2/global_pose /typhoon_h480_3/global_pose \
  /typhoon_h480_4/global_pose /typhoon_h480_5/global_pose \
  /typhoon_h480_0/global_odom /typhoon_h480_1/global_odom \
  /typhoon_h480_2/global_odom /typhoon_h480_3/global_odom \
  /typhoon_h480_4/global_odom /typhoon_h480_5/global_odom \
  /typhoon_h480_0/safety/status /typhoon_h480_1/safety/status \
  /typhoon_h480_2/safety/status /typhoon_h480_3/safety/status \
  /typhoon_h480_4/safety/status /typhoon_h480_5/safety/status
```

Gazebo 11 接触流使用正确语法：

```bash
gz topic -e /gazebo/default/physics/contacts \
  > /tmp/static-patrol-revalidation-contacts.log
```

不得使用无效的 `gz topic -e -t ...`。证据采集必须在 `/swarm/takeoff_complete` 打开前启动，避免再次漏掉 0 号机早期事故。

- [ ] **Step 2: 在真实图形桌面启动六机任务**

Run:

```bash
bash 1.sh 6 mission_down.json
```

Expected: 六机、六路相机和队伍任务节点全部就绪。运行期间另一个终端执行：

```bash
bash scripts/smoke_competition_clean.sh
```

Expected smoke ending: `PASS competition-clean six-vehicle smoke`。smoke 通过只证明链路存在，不等于全航程验收通过。

- [ ] **Step 3: 观察完整进入阶段和至少一轮四点巡逻闭环**

逐机记录：进入航点完成、巡逻点 1 到 4 完成并再次到达巡逻点 1。必须同时量化：

- 每架全程最高 `global_pose.z` 和双门控打开后的最高高度；
- 全程与任务阶段的最小机间距离、机号和时间；
- 所有接触双方完整名称；
- 4 号机与 `house_1_146_clone` 是否仍接触；
- 5 号机在 `/mission/active=true` 前是否出现目标请求、external 切换或 `PAUSE`；
- 自然发生的进入/巡逻跟踪打断和恢复，不能伪造目标；
- 0 号机是否再次发生早期事故。

若 0 号机再次事故，保存事故前位姿、控制源、安全状态和接触证据后停止本轮，不根据坠落后的约 490 米坐标发散猜修。

- [ ] **Step 4: 正常停止并核验清理边界**

在主启动终端按一次 `Ctrl-C`。Expected: 外层退出码 130。随后运行：

```bash
pgrep -af 'px4|gzserver|gzclient|multirotor_communication.py|yolo11n.py|bbox2coord_node.py'
find /tmp -maxdepth 1 -type d -name 'robocup-fly-competition-clean.*' -print
git -C "$XTDRONE_DIR" status --short
```

Expected: 没有本次项目进程残留、没有本次新增临时目录、XTDrone 工作树为空。不要删除 `/tmp/static-patrol-validation.bag` 或约 3 GB 的 `/tmp/static-patrol-contacts.log`，除非用户另行明确授权。

- [ ] **Step 5: 按验收合同判定结果**

只有六机均完成进入和至少一轮闭环、无无人机/建筑接触、无持续异常高度、无跨区或重复进入路线、双门控顺序正确，才写“真实六机复验通过”。任何一项失败都如实保留日志和量化结果，并把新根因留给下一轮单独设计。

### Task 6: 更新权威交接手册并提交复验记录

**Files:**
- Modify: `docs/AI_AGENT_HANDOFF.md`

- [ ] **Step 1: 记录首次失败证据，不被后续结果覆盖**

新增带日期的小节，至少保留：

```text
首次日志：logs/competition-clean/launch-20260804-172326-kQZ6IP.log
smoke：logs/competition-clean/smoke-20260804-172926.AUMa07.log（smoke 通过但全航程失败）
0/4 号机异常最高高度：490.143 m / 489.329 m
任务阶段最小机距：2.384 m（4、5 号机）
已闭合根因：4 号机路线穿越 house_1_146_clone；5 号机在 task_manager 为 IDLE 时提前 PAUSE
未闭合根因：0 号机首次事故
```

- [ ] **Step 2: 记录本轮自动化和真实六机结果**

使用手册的“目标、根因与边界、修改、验证、Git、剩余风险”模板。写入实际命令、退出码、通过数量、日志路径、最高高度、最小机距、接触双方、Ctrl-C 退出码、残留检查和 XTDrone 状态。任何未执行或失败项明确写“未执行”或“失败”及原因，不能省略。

- [ ] **Step 3: 检查文档和仓库差异**

Run:

```bash
git diff --check
git diff -- docs/AI_AGENT_HANDOFF.md
git status --short --branch
```

Expected: 文档数据与本轮保存的实际证据一致，不声称未验证的结果，`.superpowers/` 保持未跟踪。

- [ ] **Step 4: 提交复验记录**

Run:

```bash
git add docs/AI_AGENT_HANDOFF.md
git commit -m "docs: record patrol failure fixes and revalidation"
git status --short --branch
git log --oneline -8
```

Expected: 文档提交完成，工作树除 `.superpowers/` 和忽略的运行产物外无任务代码改动。未经用户再次明确要求，不执行 `git push`。
