# Tracking Report Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 按赛事规则连续广播人物 ID 和坐标满 15 秒，等待裁判移除，并在单次跟踪达到 20 秒时可靠释放控制、恢复巡逻且避免重复锁定已完成目标。

**Architecture:** `bbox2coord_node.py` 在真正发布 `ActorInfo` 后发布本机结构化心跳；tracking 使用独立、可单测的 `BroadcastProgress` 计算 15 秒连续广播与 20 秒会话上限；中央 `look_up` 服务把目标状态扩展为 `COMPLETED`。恢复控制通过显式返回阶段保证零命令、MUX 交权、目标状态更新和单次 `RESUME` 的顺序与幂等性。

**Tech Stack:** ROS Noetic、C++14、Python 3、Catkin、rostest、GoogleTest、unittest、Gazebo 11。

---

## 文件职责

新增文件：

- `src/look_up/msg/CoordinateBroadcastHeartbeat.msg`：队伍内部的有效坐标发布心跳。
- `src/look_up/srv/CompleteTarget.srv`：把中央目标锁永久转为本场 `COMPLETED`。
- `src/look_up/test/target_lookup_service.test`：中央服务集成测试入口。
- `src/look_up/test/test_target_lookup_service.py`：六目标申请、释放和完成行为测试。
- `src/yolo/coordinate_reporting.py`：保证先发布裁判消息、后发布本机心跳的微小可测函数。
- `tests/test_coordinate_reporting.py`：坐标消息与心跳顺序、失败短路测试。
- `src/tracking/include/tracking/broadcast_progress.h`：15/20 秒计时组件接口。
- `src/tracking/src/broadcast_progress.cpp`：纯时间状态逻辑，不访问 ROS 话题或服务。
- `src/tracking/test/broadcast_progress_test.cpp`：计时、间隔、无效时间和复位单测。
- `src/tracking/test/tracking_completion.test`：加速后的 tracking 完成流程测试入口。
- `src/tracking/test/test_tracking_completion.py`：真实 ROS 话题和服务级完成/失败/冷却/交权测试。

修改文件：

- `src/look_up/CMakeLists.txt`、`src/look_up/package.xml`：生成消息和服务并注册 rostest。
- `src/look_up/include/look_up/target_lookup_service.h`、`src/look_up/src/target_lookup_service.cpp`：增加 `COMPLETED` 状态及幂等完成服务。
- `src/yolo/bbox2coord_node.py`：创建心跳发布器，并在 `ActorInfo` 发布成功路径发送心跳。
- `tests/test_camera_geometry.py`：约束心跳只能位于有效坐标发布路径。
- `src/tracking/CMakeLists.txt`、`src/tracking/package.xml`：编译计时组件并注册单元/集成测试。
- `src/tracking/include/tracking/service_manager.h`、`src/tracking/src/service_manager.cpp`：增加完成客户端并补齐 `red5`。
- `src/tracking/include/tracking/state_machine.h`、`src/tracking/src/state_machine.cpp`：接收心跳、启动两个计时器、执行完成或失败退出和冷却。
- `src/tracking/src/tracking_node.cpp`：订阅本机心跳并交给状态机。
- `src/tracking/config/params.yaml`：加入四个规则计时参数。
- `src/tracking/test/test_takeoff_gate.py`：为新增完成服务和 `red5` 保持旧测试依赖完整。
- `src/tracking/README.md`、`src/look_up/README.md`、`src/yolo/README.md`、`docs/AI_AGENT_HANDOFF.md`：记录规则行为、接口和验证证据。

所有路径都属于队伍代码。不得修改官方或第三方目录。

### Task 1: 中央目标完成状态和结构化接口

**Files:**
- Create: `src/look_up/msg/CoordinateBroadcastHeartbeat.msg`
- Create: `src/look_up/srv/CompleteTarget.srv`
- Create: `src/look_up/test/target_lookup_service.test`
- Create: `src/look_up/test/test_target_lookup_service.py`
- Modify: `src/look_up/CMakeLists.txt`
- Modify: `src/look_up/package.xml`
- Modify: `src/look_up/include/look_up/target_lookup_service.h`
- Modify: `src/look_up/src/target_lookup_service.cpp`

- [ ] **Step 1: 写中央服务失败集成测试**

创建 rostest，启动 `target_lookup_service`，在单个测试方法中按确定顺序验证六个 ID，避免测试方法顺序共享服务状态：

```python
TARGET_IDS = ("green0", "blue1", "brown2", "white3", "red4", "red5")

def test_all_targets_release_and_completed_target_stays_unavailable(self):
    for target_id in TARGET_IDS:
        request = rospy.ServiceProxy("/lookup/request_" + target_id, RequestTarget)
        release = rospy.ServiceProxy("/lookup/release_" + target_id, ReleaseTarget)
        self.assertTrue(request(target_id).success)
        self.assertTrue(release(target_id).success)
        self.assertTrue(request(target_id).success)
        self.assertTrue(release(target_id).success)

    request_red5 = rospy.ServiceProxy("/lookup/request_red5", RequestTarget)
    complete = rospy.ServiceProxy("/lookup/complete_target", CompleteTarget)
    release_red5 = rospy.ServiceProxy("/lookup/release_red5", ReleaseTarget)

    self.assertTrue(request_red5("red5").success)
    self.assertTrue(complete("red5").success)
    self.assertTrue(complete("red5").success)
    self.assertTrue(release_red5("red5").success)
    self.assertFalse(request_red5("red5").success)
    self.assertFalse(complete("green0").success)
    self.assertFalse(complete("not-a-target").success)
```

在 `target_lookup_service.test` 中用 `respawn="false"` 启动服务，并给测试 20 秒上限。把 `add_rostest(test/target_lookup_service.test)` 注册到 `CMakeLists.txt`。

- [ ] **Step 2: 运行测试并确认 RED**

Run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_look_up
catkin_test_results build/test_results/look_up
```

Expected: FAIL；`look_up.msg.CoordinateBroadcastHeartbeat` 或 `look_up.srv.CompleteTarget` 尚不存在，或者 `/lookup/complete_target` 不可用。不能因为 Python 语法或 launch XML 错误而失败。

- [ ] **Step 3: 生成消息和完成服务**

消息与服务内容必须严格为：

```text
# CoordinateBroadcastHeartbeat.msg
std_msgs/Header header
string vehicle_name
string target_id
```

```text
# CompleteTarget.srv
string target_id
---
bool success
```

在 `CMakeLists.txt` 中增加：

```cmake
add_message_files(
  FILES
  CoordinateBroadcastHeartbeat.msg
)

add_service_files(
  FILES
  RequestTarget.srv
  ReleaseTarget.srv
  CompleteTarget.srv
)
```

保留 `generate_messages(DEPENDENCIES std_msgs)`，并在测试区注册 rostest。`package.xml` 保留 `message_generation`、`message_runtime`，增加 `rostest` 测试依赖。

- [ ] **Step 4: 实现 `COMPLETED` 状态**

在头文件中加入：

```cpp
#include <look_up/CompleteTarget.h>

const std::string STATE_COMPLETED = "COMPLETED";

ros::ServiceServer complete_service_;

bool handleCompleteTarget(look_up::CompleteTarget::Request& req,
                          look_up::CompleteTarget::Response& res);
bool isKnownTarget(const std::string& target_id) const;
```

构造函数只创建一个完成服务：

```cpp
complete_service_ = nh_.advertiseService(
    "/lookup/complete_target",
    &TargetLookupService::handleCompleteTarget,
    this);
```

完成回调实现确定状态转换：

```cpp
bool TargetLookupService::handleCompleteTarget(
    look_up::CompleteTarget::Request& req,
    look_up::CompleteTarget::Response& res)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isKnownTarget(req.target_id)) {
        ROS_ERROR_STREAM("收到未知目标的完成请求: '" << req.target_id << "'。");
        res.success = false;
        return true;
    }

    std::string& status = target_status_[req.target_id];
    if (status == STATE_COMPLETED) {
        res.success = true;
        return true;
    }
    if (status != STATE_TRACKED) {
        ROS_WARN_STREAM("目标 '" << req.target_id << "' 未被锁定，拒绝完成。");
        res.success = false;
        return true;
    }

    status = STATE_COMPLETED;
    ROS_INFO_STREAM("目标 '" << req.target_id << "' 已标记为 COMPLETED。");
    res.success = true;
    return true;
}
```

将 request、release、complete 共用 `isKnownTarget()`。release 遇到 `STATE_COMPLETED` 时返回成功但不改变状态；request 仍只批准 `STATE_AVAILABLE`。

- [ ] **Step 5: 运行聚焦测试并确认 GREEN**

Run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_look_up
catkin_test_results build/test_results/look_up
```

Expected: `look_up` 测试 0 errors、0 failures，`red5` 完成后普通释放也不能重新申请。

- [ ] **Step 6: 提交中央接口**

```bash
git add src/look_up/msg/CoordinateBroadcastHeartbeat.msg \
  src/look_up/srv/CompleteTarget.srv \
  src/look_up/test/target_lookup_service.test \
  src/look_up/test/test_target_lookup_service.py \
  src/look_up/CMakeLists.txt src/look_up/package.xml \
  src/look_up/include/look_up/target_lookup_service.h \
  src/look_up/src/target_lookup_service.cpp
git commit -m "feat: track completed search targets"
```

### Task 2: 坐标发布成功后发送本机心跳

**Files:**
- Create: `src/yolo/coordinate_reporting.py`
- Create: `tests/test_coordinate_reporting.py`
- Modify: `src/yolo/bbox2coord_node.py`
- Modify: `tests/test_camera_geometry.py`

- [ ] **Step 1: 写发布顺序和失败短路测试**

测试使用记录调用顺序的假发布器，不模拟 ROS 网络：

```python
class RecordingPublisher:
    def __init__(self, name, calls, error=None):
        self.name = name
        self.calls = calls
        self.error = error

    def publish(self, message):
        self.calls.append((self.name, message))
        if self.error is not None:
            raise self.error


def test_actor_report_is_published_before_heartbeat(self):
    calls = []
    actor_message = object()
    heartbeat = object()
    publish_actor_info_with_heartbeat(
        RecordingPublisher("actor", calls), actor_message,
        RecordingPublisher("heartbeat", calls), heartbeat,
    )
    self.assertEqual([("actor", actor_message), ("heartbeat", heartbeat)], calls)


def test_actor_publish_failure_prevents_heartbeat(self):
    calls = []
    with self.assertRaises(RuntimeError):
        publish_actor_info_with_heartbeat(
            RecordingPublisher("actor", calls, RuntimeError("publish failed")),
            object(), RecordingPublisher("heartbeat", calls), object(),
        )
    self.assertEqual(["actor"], [name for name, _ in calls])
```

在 `tests/test_camera_geometry.py` 的 AST 合同测试中再断言：节点导入 `CoordinateBroadcastHeartbeat` 和 `publish_actor_info_with_heartbeat`，构造本机心跳发布器，并且 `_publish_actor_info` 只在已经建立合法 `ActorInfo` 后调用该函数。

- [ ] **Step 2: 运行测试并确认 RED**

Run:

```bash
python3 -m unittest tests.test_coordinate_reporting tests.test_camera_geometry -v
```

Expected: FAIL with `ModuleNotFoundError: coordinate_reporting`，或心跳导入/发布合同缺失。

- [ ] **Step 3: 添加最小发布函数**

`coordinate_reporting.py` 完整行为：

```python
def publish_actor_info_with_heartbeat(
    actor_publisher,
    actor_message,
    heartbeat_publisher,
    heartbeat_message,
):
    actor_publisher.publish(actor_message)
    heartbeat_publisher.publish(heartbeat_message)
```

- [ ] **Step 4: 在坐标节点接入结构化心跳**

新增导入：

```python
from look_up.msg import CoordinateBroadcastHeartbeat
from coordinate_reporting import publish_actor_info_with_heartbeat
```

在 publisher 初始化阶段创建：

```python
self.broadcast_heartbeat_pub = rospy.Publisher(
    f"/{self.robot_name}/coordinate_broadcast/heartbeat",
    CoordinateBroadcastHeartbeat,
    queue_size=10,
)
```

在 `_publish_actor_info()` 中保留现有 ID 映射和 `ActorInfo` 内容，只把最终发布替换为：

```python
heartbeat = CoordinateBroadcastHeartbeat()
heartbeat.header.stamp = rospy.Time.now()
heartbeat.vehicle_name = self.robot_name
heartbeat.target_id = class_name
publish_actor_info_with_heartbeat(
    publisher,
    actor_message,
    self.broadcast_heartbeat_pub,
    heartbeat,
)
```

该代码必须仍位于 actor ID 已验证、坐标转换成功的分支内。日志放在发布函数返回之后，不能在失败时声称已经发布。

- [ ] **Step 5: 运行聚焦测试并确认 GREEN**

Run:

```bash
python3 -m unittest tests.test_coordinate_reporting tests.test_camera_geometry -v
python3 -m py_compile src/yolo/coordinate_reporting.py src/yolo/bbox2coord_node.py
```

Expected: 所有测试 PASS，两个 Python 文件编译退出码 0。

- [ ] **Step 6: 提交坐标心跳**

```bash
git add src/yolo/coordinate_reporting.py src/yolo/bbox2coord_node.py \
  tests/test_coordinate_reporting.py tests/test_camera_geometry.py
git commit -m "feat: report valid coordinate broadcasts"
```

### Task 3: 独立实现 15/20 秒计时组件

**Files:**
- Create: `src/tracking/include/tracking/broadcast_progress.h`
- Create: `src/tracking/src/broadcast_progress.cpp`
- Create: `src/tracking/test/broadcast_progress_test.cpp`
- Modify: `src/tracking/CMakeLists.txt`

- [ ] **Step 1: 写纯 C++ 失败测试**

测试固定使用秒数，不等待墙钟：

```cpp
TEST(BroadcastProgressTest, RequiresContinuousHeartbeatsForConfirmation) {
  BroadcastProgress progress(15.0, 0.5, 20.0);
  progress.start(100.0);
  for (int tenth = 1; tenth <= 149; ++tenth) {
    const double stamp = 100.0 + tenth / 10.0;
    EXPECT_TRUE(progress.recordHeartbeat(stamp, stamp));
  }
  EXPECT_FALSE(progress.broadcastConfirmed());
  EXPECT_TRUE(progress.recordHeartbeat(115.2, 115.2));
  EXPECT_TRUE(progress.broadcastConfirmed());
}

TEST(BroadcastProgressTest, GapRestartsConfirmationWindow) {
  BroadcastProgress progress(15.0, 0.5, 20.0);
  progress.start(10.0);
  EXPECT_TRUE(progress.recordHeartbeat(10.1, 10.1));
  EXPECT_TRUE(progress.recordHeartbeat(10.7, 10.7));  // 0.6s gap resets.
  for (int tenth = 8; tenth <= 156; ++tenth) {
    const double stamp = 10.0 + tenth / 10.0;
    EXPECT_TRUE(progress.recordHeartbeat(stamp, stamp));
  }
  EXPECT_FALSE(progress.broadcastConfirmed());
}

TEST(BroadcastProgressTest, RejectsInvalidTimestampsAndTimesOutSession) {
  BroadcastProgress progress(15.0, 0.5, 20.0);
  progress.start(50.0);
  EXPECT_FALSE(progress.recordHeartbeat(0.0, 50.1));
  EXPECT_FALSE(progress.recordHeartbeat(50.2, 50.1));
  EXPECT_TRUE(progress.recordHeartbeat(50.1, 50.1));
  EXPECT_FALSE(progress.recordHeartbeat(50.0, 50.2));
  EXPECT_FALSE(progress.sessionTimedOut(69.999));
  EXPECT_TRUE(progress.sessionTimedOut(70.0));
}

TEST(BroadcastProgressTest, ResetClearsAllSessionState) {
  BroadcastProgress progress(0.19, 0.11, 1.0);
  progress.start(1.0);
  progress.recordHeartbeat(1.1, 1.1);
  progress.recordHeartbeat(1.2, 1.2);
  progress.recordHeartbeat(1.3, 1.3);
  ASSERT_TRUE(progress.broadcastConfirmed());
  progress.reset();
  EXPECT_FALSE(progress.active());
  EXPECT_FALSE(progress.broadcastConfirmed());
  EXPECT_FALSE(progress.sessionTimedOut(100.0));
}
```

另加参数测试：非有限值、非正数、确认时间不小于会话上限、心跳超时不小于确认时间均抛 `std::invalid_argument`。

- [ ] **Step 2: 运行测试并确认 RED**

先在 `CMakeLists.txt` 注册计时组件源文件和测试：

```cmake
add_executable(tracking_node
  src/tracking_node.cpp
  src/controller.cpp
  src/state_machine.cpp
  src/broadcast_progress.cpp
  src/kalman_filter.cpp
  src/kalman_filter_base.cpp
  src/service_manager.cpp
  src/smoothing.cpp
)

catkin_add_gtest(broadcast_progress_test
  test/broadcast_progress_test.cpp
  src/broadcast_progress.cpp
)
if(TARGET broadcast_progress_test)
  target_link_libraries(broadcast_progress_test ${catkin_LIBRARIES})
endif()
```

再运行：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_tracking_gtest_broadcast_progress_test
catkin_test_results build/test_results/tracking
```

Expected: FAIL because `tracking/broadcast_progress.h` 或实现尚不存在；失败不能来自测试语法错误。

- [ ] **Step 3: 实现最小计时类**

接口固定为：

```cpp
class BroadcastProgress {
public:
  BroadcastProgress(double confirmation_duration,
                    double heartbeat_timeout,
                    double session_timeout);
  void start(double now);
  bool recordHeartbeat(double stamp, double now);
  bool active() const;
  bool broadcastConfirmed() const;
  bool sessionTimedOut(double now) const;
  void reset();

private:
  double confirmation_duration_;
  double heartbeat_timeout_;
  double session_timeout_;
  double session_start_ = 0.0;
  double streak_start_ = 0.0;
  double last_heartbeat_ = 0.0;
  bool active_ = false;
  bool broadcast_confirmed_ = false;
};
```

实现规则：

```cpp
bool BroadcastProgress::recordHeartbeat(double stamp, double now) {
  if (!active_ || !std::isfinite(stamp) || !std::isfinite(now) ||
      stamp <= 0.0 || stamp > now ||
      (last_heartbeat_ > 0.0 && stamp < last_heartbeat_)) {
    return false;
  }
  if (last_heartbeat_ <= 0.0 || stamp - last_heartbeat_ > heartbeat_timeout_) {
    streak_start_ = stamp;
  }
  last_heartbeat_ = stamp;
  if (stamp - streak_start_ >= confirmation_duration_) {
    broadcast_confirmed_ = true;
  }
  return true;
}
```

`sessionTimedOut(now)` 只在 active、时间有限且 `now - session_start_ >= session_timeout_` 时为真。`reset()` 把三个时间、active 和确认标志全部恢复初始值。

- [ ] **Step 4: 运行测试并确认 GREEN**

Run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_tracking_gtest_broadcast_progress_test
catkin_test_results build/test_results/tracking
```

Expected: `broadcast_progress_test` 0 failures。

- [ ] **Step 5: 提交计时组件**

```bash
git add src/tracking/include/tracking/broadcast_progress.h \
  src/tracking/src/broadcast_progress.cpp \
  src/tracking/test/broadcast_progress_test.cpp src/tracking/CMakeLists.txt
git commit -m "feat: track coordinate broadcast progress"
```

### Task 4: 将完成条件接入 tracking 控制交权

**Files:**
- Create: `src/tracking/test/tracking_completion.test`
- Create: `src/tracking/test/test_tracking_completion.py`
- Modify: `src/tracking/include/tracking/service_manager.h`
- Modify: `src/tracking/src/service_manager.cpp`
- Modify: `src/tracking/include/tracking/state_machine.h`
- Modify: `src/tracking/src/state_machine.cpp`
- Modify: `src/tracking/src/tracking_node.cpp`
- Modify: `src/tracking/config/params.yaml`
- Modify: `src/tracking/test/test_takeoff_gate.py`
- Modify: `src/tracking/CMakeLists.txt`
- Modify: `src/tracking/package.xml`

- [ ] **Step 1: 扩展旧门控测试的依赖桩**

把两个测试中的目标列表统一为：

```python
TARGET_IDS = ("green0", "blue1", "brown2", "white3", "red4", "red5", "person")
```

旧 `test_takeoff_gate.py` 增加 `/lookup/complete_target` 服务桩，记录完成请求；原测试仍断言门控关闭只释放目标且不完成目标。

- [ ] **Step 2: 写 tracking 完成流程失败 rostest**

`tracking_completion.test` 使用加速参数：

```xml
<param name="state_machine/confirmation_duration" value="0.0"/>
<param name="state_machine/lost_timeout" value="0.1"/>
<param name="state_machine/broadcast_confirmation_duration" value="0.3"/>
<param name="state_machine/broadcast_heartbeat_timeout" value="0.1"/>
<param name="state_machine/tracking_session_timeout" value="0.8"/>
<param name="state_machine/retry_cooldown" value="0.4"/>
```

测试按以下顺序执行并记录服务调用和消息时间：

```python
# 1. green0: 连续发送本机 green0 心跳超过 0.3 秒。
# 2. 停止人物框，让 LOST 超时。
# 3. 第一次 navigator MUX 请求由服务桩抛 ServiceException。
# 4. 第二次 MUX 请求成功，确认事件顺序为 mux_fail、mux_success、
#    complete(green0)、RESUME，且 complete 和 RESUME 各一次。
# 5. blue1: 保持可见但不发心跳，超过 0.8 秒。
# 6. 确认 release(blue1) 一次、无 complete(blue1)、RESUME 再增加一次。
# 7. 在 0.4 秒冷却内持续发布 blue1，确认没有第二次 request。
# 8. 冷却结束后继续发布，确认允许再次 request(blue1)。
```

心跳必须使用生成消息：

```python
heartbeat = CoordinateBroadcastHeartbeat()
heartbeat.header.stamp = rospy.Time.now()
heartbeat.vehicle_name = "test_drone_0"
heartbeat.target_id = target_id
self.heartbeat_pub.publish(heartbeat)
```

另发送错误无人机和错误人物心跳，确认它们不能使 blue1 完成。将 rostest 注册到 `CMakeLists.txt`。

- [ ] **Step 3: 运行测试并确认 RED**

Run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_tracking
catkin_test_results build/test_results/tracking
```

Expected: FAIL；tracking 尚不订阅心跳、不会调用完成服务，也没有 20 秒出口或冷却。

- [ ] **Step 4: 扩展 ServiceManager**

头文件加入 `look_up/CompleteTarget.h`、单一 `complete_client_` 和：

```cpp
bool completeTarget(const std::string& target_id);
```

初始化时等待并连接 `/lookup/complete_target`。`TARGET_IDS_` 必须包含 `red5`。调用实现只在 ROS 服务调用成功且 `response.success` 为真时返回 true；错误日志必须包含无人机名和 target ID。

- [ ] **Step 5: 加载并验证四个规则参数**

在 `params.yaml` 增加：

```yaml
  broadcast_confirmation_duration: 15.0
  broadcast_heartbeat_timeout: 0.5
  tracking_session_timeout: 20.0
  retry_cooldown: 5.0
```

状态机构造函数读取四项，交给 `BroadcastProgress` 构造函数验证前三项，并单独验证 `retry_cooldown` 为有限正数；不合法时抛 `std::invalid_argument` 使节点启动失败。

- [ ] **Step 6: 订阅并筛选本机心跳**

`tracking_node.cpp` 订阅：

```cpp
heartbeat_sub_ = nh_.subscribe<look_up::CoordinateBroadcastHeartbeat>(
    "/" + vehicle_type_ + "_" + vehicle_id_ +
        "/coordinate_broadcast/heartbeat",
    10,
    &TrackingNode::heartbeatCallback,
    this);
```

回调把 `vehicle_name`、`target_id` 和 `header.stamp` 传给：

```cpp
void TrackingStateMachine::recordCoordinateBroadcast(
    const std::string& vehicle_name,
    const std::string& target_id,
    const ros::Time& stamp);
```

状态机只在 `DASH` 或 `TRACKING`、两个控制门控均打开、vehicle name 等于 `<vehicle_type>_<vehicle_id>`、target ID 等于当前锁定 ID 时调用 `broadcast_progress_->recordHeartbeat()`。第一次有效心跳和首次达到 15 秒分别记录一次 INFO 日志。

- [ ] **Step 7: 启动会话、执行 20 秒判定和本机冷却**

`enterDashState()` 和直接 `enterTrackingState()` 在成功接管的同一会话中调用一次：

```cpp
broadcast_progress_->start(ros::Time::now().toSec());
```

`enterTrackingFromDash()` 不重启计时。每次 update 在执行 DASH、TRACKING 或 LOST handler 前检查 session timeout：

```cpp
if (broadcast_progress_->sessionTimedOut(now.toSec())) {
    beginReturnToMission(
        broadcast_progress_->broadcastConfirmed()
            ? ReturnOutcome::COMPLETE
            : ReturnOutcome::RELEASE_WITH_COOLDOWN);
}
```

冷却使用 `std::map<std::string, ros::Time> cooldown_until_`。IDLE 申请目标前删除已过期项并跳过未过期 ID。`RELEASE_WITH_COOLDOWN` 以当前 ROS 时间加 `retry_cooldown_` 写入当前人物；普通丢失不建立冷却。

- [ ] **Step 8: 增加幂等返回阶段**

增加 `State::RETURNING`、`navigator_selected_` 标志和：

```cpp
enum class ReturnOutcome { RELEASE, COMPLETE, RELEASE_WITH_COOLDOWN };

void beginReturnToMission(ReturnOutcome outcome);
void handleReturningState(double dt);
void finalizeReturnToMission();
```

`beginReturnToMission()` 只在尚未 RETURNING 时保存 outcome、清零完成尝试次数、把 `navigator_selected_` 设为 false 并切换状态。`handleReturningState()` 每轮先发布零 external 命令；仅当 `navigator_selected_` 为 false 时请求 MUX 切 navigator，失败时立即返回且不发布 `RESUME`，成功时把该标志设为 true。完成服务的后续重试不得再次调用 MUX。

MUX 成功后：

```cpp
if (return_outcome_ == ReturnOutcome::COMPLETE) {
    if (!services_.completeTarget(currently_tracked_target_id_)) {
        ++complete_attempts_;
        if (complete_attempts_ < 3) {
            return;
        }
        services_.releaseTarget(currently_tracked_target_id_);
    }
} else {
    services_.releaseTarget(currently_tracked_target_id_);
}
```

完成或回退释放后只调用一次 `finalizeReturnToMission()`：根据 outcome 写冷却，发送一个 `RESUME`，清空目标、滤波会话、计时、`navigator_selected_`、返回 outcome、完成尝试和丢失计数并进入 IDLE。原 `returnControlToMission()` 替换为该流程。

LOST 超时时依据确认标志选择：

```cpp
beginReturnToMission(
    broadcast_progress_->broadcastConfirmed()
        ? ReturnOutcome::COMPLETE
        : ReturnOutcome::RELEASE);
```

门控关闭沿用现有“不主动切 MUX”合同，但必须 reset 计时器、返回阶段、完成尝试和冷却中的当前会话临时状态。已建立的其他目标冷却保留到自然过期。

- [ ] **Step 9: 运行 tracking 测试并确认 GREEN**

Run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_tracking
catkin_test_results build/test_results/tracking
```

Expected: `broadcast_progress_test`、`tracking_takeoff_gate`、`tracking_completion` 全部 0 errors、0 failures；日志可见一次规则确认、一次 MUX 重试和每次会话一个 `RESUME`。

- [ ] **Step 10: 提交 tracking 集成**

```bash
git add src/tracking/include/tracking/service_manager.h \
  src/tracking/src/service_manager.cpp \
  src/tracking/include/tracking/state_machine.h \
  src/tracking/src/state_machine.cpp src/tracking/src/tracking_node.cpp \
  src/tracking/config/params.yaml \
  src/tracking/test/test_takeoff_gate.py \
  src/tracking/test/tracking_completion.test \
  src/tracking/test/test_tracking_completion.py \
  src/tracking/CMakeLists.txt src/tracking/package.xml
git commit -m "fix: resume patrol after target reporting"
```

### Task 5: 文档、全量验证和真实六机证据

**Files:**
- Modify: `src/tracking/README.md`
- Modify: `src/look_up/README.md`
- Modify: `src/yolo/README.md`
- Modify: `docs/AI_AGENT_HANDOFF.md`

- [ ] **Step 1: 更新使用和维护文档**

记录以下确定内容：

- 规则要求的是 15 秒连续有效坐标广播，不是 15 秒视觉可见。
- 心跳话题、消息字段、0.5 秒间隔、20 秒上限和 5 秒失败冷却。
- `AVAILABLE`、`TRACKED`、`COMPLETED` 的含义。
- 15 秒后人物消失与 20 秒两种出口。
- 未满 15 秒超时只释放，不标记完成。
- 自动化验证结果和真实 Gazebo 结果必须分开陈述。

- [ ] **Step 2: 运行所有聚焦静态与单元测试**

Run:

```bash
python3 -m unittest tests.test_coordinate_reporting tests.test_camera_geometry -v
python3 -m py_compile src/yolo/coordinate_reporting.py src/yolo/bbox2coord_node.py
python3 -m unittest discover -s tests -v
git diff --check
```

Expected: 所有命令退出码 0；Python 测试总数不得少于修改前 134 项加本轮新增测试数。

- [ ] **Step 3: 运行完整 competition-clean verifier**

Run from repository root:

```bash
bash scripts/verify_competition_clean.sh
```

Expected: Python、Release Catkin 构建、actor collision 外置构建、Catkin tests、测试结果汇总和两次官方文件校验全部 PASS。若隔离环境因网络接口枚举权限失败，只能在获准的本机环境用同一脚本重跑并记录两次结果，不能删减 verifier。

- [ ] **Step 4: 检查官方目录和任务 diff**

```bash
git diff --check
git status --short
git diff --stat
git -C "$XTDRONE_DIR" status --short
```

Expected: 项目 diff 只包含本计划列出的队伍文件；XTDrone 输出为空；`.superpowers/` 仍未跟踪且不加入提交。

- [ ] **Step 5: 提交文档和自动化结果**

```bash
git add src/tracking/README.md src/look_up/README.md src/yolo/README.md \
  docs/AI_AGENT_HANDOFF.md
git commit -m "docs: document target report completion"
```

- [ ] **Step 6: 启动真实六机验证**

确认桌面图形会话可用后，在仓库根目录运行：

```bash
bash 1.sh 6 mission_down.json
```

另一个终端运行现有 smoke：

```bash
bash scripts/smoke_competition_clean.sh
```

观察至少一个目标完整经历：锁定、`PAUSE`、有效广播起点、15 秒确认、人物移除或 20 秒出口、中央 `COMPLETED`/release、MUX navigator、单次 `RESUME`。同时记录六机高度、最小机距、碰撞和航点进度；smoke PASS 不能替代全航程结果。

- [ ] **Step 7: 正常停止并检查残留**

在 `1.sh` 终端按一次 Ctrl-C，然后运行：

```bash
pgrep -af 'px4|gzserver|gzclient|roslaunch|multirotor_communication.py|yolo11n.py|bbox2coord_node.py|tracking_node'
find /tmp -maxdepth 1 -type d -name 'competition-clean.*' -print
git -C "$XTDRONE_DIR" status --short
```

Expected: 没有本次会话相关进程，没有新增 competition-clean 临时目录，XTDrone 工作树为空。不得用宽泛 `pkill` 清理。

- [ ] **Step 8: 把真实结果写入交接文档并提交**

在 `docs/AI_AGENT_HANDOFF.md` 新增一节，记录：主日志、smoke 日志、目标 ID、首条有效心跳时间、15 秒确认时间、退出原因、完成/释放调用数、MUX 与 `RESUME` 次数、碰撞与全航程结论、Ctrl-C 退出码和残留检查。若真实运行失败，明确写“失败”和剩余根因，禁止描述为比赛就绪。

```bash
git add docs/AI_AGENT_HANDOFF.md
git commit -m "docs: record tracking completion validation"
git status --short --branch
```

Expected: 仅 `.superpowers/` 保持未跟踪；本地分支领先远端，但不推送。只有用户再次明确要求时才能执行 `git push`。
