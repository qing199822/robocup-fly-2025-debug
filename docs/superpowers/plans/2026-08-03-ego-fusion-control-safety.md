# EGO-Fusion 控制安全骨架 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不修改 PX4、XTDrone、Gazebo 或 EGO 的前提下，为 6 架无人机建立 takeoff、navigator、external 三路 MUX 和后置安全过滤器，使每个 XTDrone 最终速度话题始终只有一个队伍发布者。

**Architecture:** 每架无人机的三个队伍控制源只发布各自 MUX 输入，`topic_tools/mux` 输出到队伍内部 `raw_cmd_vel`，新建的 `safety_filter` 检查指令新鲜度、位姿新鲜度、有限数值、高度、速度及加速度后，独占发布 `/xtdrone/typhoon_h480_N/cmd_vel_flu`。本阶段不接入 EGO；navigator 输入仍由 `simple_navigator` 使用，后续 `ego_adapter` 复用该入口。

**Tech Stack:** Ubuntu 20.04、ROS Noetic、Catkin、C++14、`roscpp`、`geometry_msgs`、`std_msgs`、`topic_tools`、Python 3 `unittest`、Catkin GTest/Rostest。

---

## 范围与完成定义

本计划只实现 [EGO-Fusion Search 详细设计](../../EGO_FUSION_SEARCH_DESIGN.md) 的“阶段 1：控制安全骨架”。地图、前沿、任务分配、EGO `PositionCommand` 适配和人物 TTL 均不在本计划中。

完成时必须满足：

1. `fly_takeoff` 不再发布 XTDrone 最终速度话题；
2. MUX 有 takeoff、navigator、external 三个输入，输出只到 `raw_cmd_vel`；
3. 每机一个 `safety_filter`，它是最终速度话题唯一发布者；
4. 原始指令或 odometry 超时、出现 NaN/Inf、越过高度边界时输出零速度或阻止危险轴向；
5. 起飞前选择 takeoff 输入，起飞完成归零后选择 navigator；
6. tracking 仍通过现有服务选择 external/navigator；
7. smoke 从 ROS Master 验证每个最终话题恰好一个发布者且节点名正确；
8. PX4、XTDrone、Gazebo 外部目录未改变；
9. 完整验证、Catkin 测试和六机 smoke 通过。

## 文件结构

### 新建

```text
src/ego_fusion_search/safety_filter/CMakeLists.txt
src/ego_fusion_search/safety_filter/package.xml
src/ego_fusion_search/safety_filter/README.md
src/ego_fusion_search/safety_filter/config/default.yaml
src/ego_fusion_search/safety_filter/include/safety_filter/safety_policy.h
src/ego_fusion_search/safety_filter/src/safety_policy.cpp
src/ego_fusion_search/safety_filter/src/safety_filter_node.cpp
src/ego_fusion_search/safety_filter/launch/safety_filter_swarm.launch
src/ego_fusion_search/safety_filter/test/safety_policy_test.cpp
src/ego_fusion_search/safety_filter/test/safety_filter_node.test
src/ego_fusion_search/safety_filter/test/test_safety_filter_node.py
tests/test_control_safety_wiring.py
scripts/check_final_control_publishers.py
tests/test_final_control_publishers.py
```

### 修改

```text
src/look_up/launch/spawn_mux_swarm.launch
src/look_up/launch/down_resume.launch
src/mix_nav/fly/include/fly/fly_takeoff.h
src/mix_nav/fly/src/fly_takeoff.cpp
src/mix_nav/fly/CMakeLists.txt
src/mix_nav/fly/package.xml
src/mix_nav/fly/test/test_fly_launch.py
src/competition_compliance/config/ownership.json
src/competition_compliance/test/test_ownership.py
scripts/smoke_competition_clean.sh
docs/AI_AGENT_HANDOFF.md
docs/COMPLIANCE.md
```

`safety_policy` 只做可测试的数值检查和限幅；`safety_filter_node` 只处理 ROS 订阅、时间、参数和发布。发布者守卫的纯函数放在脚本中，允许脱离 ROS Master 单测。

## 固定话题契约

对 `N = 0..5`：

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| fly -> MUX | `/typhoon_h480_N/mux_inputs/takeoff/cmd_vel` | `geometry_msgs/Twist` |
| navigator -> MUX | `/typhoon_h480_N/mux_inputs/navigator/cmd_vel` | `geometry_msgs/Twist` |
| tracking -> MUX | `/typhoon_h480_N/mux_inputs/external/pose_cmd` | `geometry_msgs/Twist` |
| MUX -> safety | `/typhoon_h480_N/control/raw_cmd_vel` | `geometry_msgs/Twist` |
| pose_init -> safety | `/typhoon_h480_N/global_odom` | `nav_msgs/Odometry` |
| safety -> XTDrone | `/xtdrone/typhoon_h480_N/cmd_vel_flu` | `geometry_msgs/Twist` |
| safety diagnostics | `/typhoon_h480_N/safety/status` | `std_msgs/String` |

节点名固定为 `/typhoon_h480_N/safety_filter`，供发布者守卫核对。

### Task 1: 固化已确认文档和基线

**Files:**
- Add: `docs/EGO_FUSION_SEARCH_DESIGN.md`
- Add: `docs/superpowers/plans/2026-08-03-ego-fusion-control-safety.md`

- [ ] **Step 1: 确认公开备份仍是设计前基线**

Run:

```bash
git status --short --branch
git rev-parse HEAD
git rev-parse public/competition-clean
```

Expected: `HEAD` 与 `public/competition-clean` 均为 `c2d234af98040ea51e30b860abbc606056d68676`；只有两份新文档尚未跟踪。

- [ ] **Step 2: 运行基线 ownership 测试**

Run:

```bash
python3 -m unittest discover -s src/competition_compliance/test -p 'test_ownership.py'
```

Expected: `Ran 39 tests` 和 `OK`。

- [ ] **Step 3: 提交已批准规格和计划**

```bash
git add docs/EGO_FUSION_SEARCH_DESIGN.md docs/superpowers/plans/2026-08-03-ego-fusion-control-safety.md
git commit -m "docs: design EGO fusion control safety"
```

Expected: 一个仅包含两份 Markdown 的提交；不推送，先继续本地 TDD。

### Task 2: 先建立失败的控制拓扑契约

**Files:**
- Create: `tests/test_control_safety_wiring.py`
- Test: `tests/test_control_safety_wiring.py`

- [ ] **Step 1: 写静态拓扑测试**

Create `tests/test_control_safety_wiring.py`:

```python
#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).parents[1]
MUX_LAUNCH = ROOT / "src/look_up/launch/spawn_mux_swarm.launch"
MISSION_LAUNCH = ROOT / "src/look_up/launch/down_resume.launch"
FLY_SOURCE = ROOT / "src/mix_nav/fly/src/fly_takeoff.cpp"
SAFETY_LAUNCH = (
    ROOT
    / "src/ego_fusion_search/safety_filter/launch/safety_filter_swarm.launch"
)


class ControlSafetyWiringTest(unittest.TestCase):
    def test_mux_has_three_inputs_and_internal_output(self):
        root = ET.parse(str(MUX_LAUNCH)).getroot()
        node = root.find(".//node[@pkg='topic_tools'][@type='mux']")
        self.assertIsNotNone(node)
        args = node.get("args", "")
        self.assertIn("$(arg input_takeoff)", args)
        self.assertIn("$(arg input_navigator)", args)
        self.assertIn("$(arg input_external)", args)
        text = MUX_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            "/$(arg vehicle_type)_$(arg drone_id)/control/raw_cmd_vel", text
        )
        self.assertNotIn(
            '<arg name="mux_output" value="/xtdrone/', text
        )

    def test_takeoff_never_publishes_final_velocity(self):
        source = FLY_SOURCE.read_text(encoding="utf-8")
        self.assertIn("/mux_inputs/takeoff/cmd_vel", source)
        self.assertNotIn('"/cmd_vel_flu"', source)
        self.assertIn("topic_tools::MuxSelect", source)

    def test_mission_launch_starts_safety_before_mux_and_takeoff(self):
        root = ET.parse(str(MISSION_LAUNCH)).getroot()
        includes = [item.get("file", "") for item in root.findall("./include")]
        safety = "$(find safety_filter)/launch/safety_filter_swarm.launch"
        mux = "$(find look_up)/launch/spawn_mux_swarm.launch"
        fly = "$(find fly)/launch/fly.launch"
        self.assertLess(includes.index(safety), includes.index(mux))
        self.assertLess(includes.index(mux), includes.index(fly))

    def test_safety_launch_names_all_per_vehicle_interfaces(self):
        text = SAFETY_LAUNCH.read_text(encoding="utf-8")
        self.assertIn("/control/raw_cmd_vel", text)
        self.assertIn("/global_odom", text)
        self.assertIn("/xtdrone/$(arg vehicle_type)_$(arg drone_id)/cmd_vel_flu", text)
        self.assertIn("name=\"safety_filter\"", text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行测试并确认它因功能尚不存在而失败**

Run:

```bash
python3 tests/test_control_safety_wiring.py -v
```

Expected: 4 个测试中至少 3 个失败或报错，原因包括缺少 `safety_filter_swarm.launch`、MUX 没有 takeoff 输入、fly 仍含最终速度话题。

- [ ] **Step 3: 暂不提交失败测试**

保持工作树中的测试未提交，直接进入 Task 3。测试转绿后与实现一起提交。

### Task 3: 用 TDD 实现纯 C++ 安全策略

**Files:**
- Create: `src/ego_fusion_search/safety_filter/package.xml`
- Create: `src/ego_fusion_search/safety_filter/CMakeLists.txt`
- Create: `src/ego_fusion_search/safety_filter/include/safety_filter/safety_policy.h`
- Create: `src/ego_fusion_search/safety_filter/src/safety_policy.cpp`
- Create: `src/ego_fusion_search/safety_filter/test/safety_policy_test.cpp`
- Modify: `src/competition_compliance/config/ownership.json`
- Modify: `src/competition_compliance/test/test_ownership.py`

- [ ] **Step 1: 建立包清单和构建文件**

Create `package.xml` with package name/version/license `safety_filter`/`0.1.0`/`LicenseRef-Team-Code`, and dependencies:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>safety_filter</name>
  <version>0.1.0</version>
  <description>Final command safety gate for the competition drones.</description>
  <maintainer email="qing199822@users.noreply.github.com">ZZU FLY Team</maintainer>
  <license>LicenseRef-Team-Code</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <test_depend>rostest</test_depend>
  <test_depend>rospy</test_depend>
</package>
```

Create `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(safety_filter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES safety_policy
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(safety_policy src/safety_policy.cpp)
target_link_libraries(safety_policy ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(safety_policy_test test/safety_policy_test.cpp)
  if(TARGET safety_policy_test)
    target_link_libraries(safety_policy_test safety_policy ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS safety_policy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
```

- [ ] **Step 2: 在 ownership 中声明队伍包**

在 `ownership.json` 的 team entries 中加入：

```json
{"path": "src/ego_fusion_search/safety_filter", "kind": "team", "source": "this repository", "version": "0.1.0", "license": "LicenseRef-Team-Code"}
```

不要修改或重排第三方 hash 项。

同时在 `test_ownership.py` 的 `TEAM_ENTRIES` 中加入：

```python
"src/ego_fusion_search/safety_filter": ("0.1.0", "LicenseRef-Team-Code"),
```

这两个清单必须同步；测试不能为了接受任意未登记包而放宽。

- [ ] **Step 3: 写失败的数值策略测试**

Create `test/safety_policy_test.cpp`:

```cpp
#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "safety_filter/safety_policy.h"

using safety_filter::Fault;
using safety_filter::Limits;
using safety_filter::SafetyPolicy;

TEST(SafetyPolicy, RejectsNonFiniteCommand) {
  SafetyPolicy policy(Limits{});
  geometry_msgs::Twist input;
  input.linear.x = std::numeric_limits<double>::quiet_NaN();
  const auto result = policy.apply(input, 3.0, 0.05);
  EXPECT_EQ(Fault::NON_FINITE_COMMAND, result.fault);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.x);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.z);
}

TEST(SafetyPolicy, ClampsHorizontalVectorAndYawRate) {
  Limits limits;
  limits.max_xy_speed = 3.0;
  limits.max_yaw_rate = 1.0;
  limits.max_xy_acceleration = 100.0;
  SafetyPolicy policy(limits);
  geometry_msgs::Twist input;
  input.linear.x = 3.0;
  input.linear.y = 4.0;
  input.angular.z = 2.0;
  const auto result = policy.apply(input, 3.0, 1.0);
  EXPECT_EQ(Fault::NONE, result.fault);
  EXPECT_NEAR(3.0, std::hypot(result.command.linear.x, result.command.linear.y), 1e-9);
  EXPECT_DOUBLE_EQ(1.0, result.command.angular.z);
}

TEST(SafetyPolicy, BlocksOnlyVelocityThatCrossesAltitudeBoundary) {
  Limits limits;
  limits.min_altitude = 0.5;
  limits.max_altitude = 5.5;
  limits.max_z_acceleration = 100.0;
  SafetyPolicy high_policy(limits);
  geometry_msgs::Twist climb;
  climb.linear.z = 0.8;
  auto result = high_policy.apply(climb, 5.4, 1.0);
  EXPECT_DOUBLE_EQ(0.8, result.command.linear.z);
  result = high_policy.apply(climb, 5.5, 1.0);
  EXPECT_EQ(Fault::ALTITUDE_LIMIT, result.fault);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.z);

  SafetyPolicy low_policy(limits);
  geometry_msgs::Twist descend;
  descend.linear.z = -0.8;
  result = low_policy.apply(descend, 0.5, 1.0);
  EXPECT_EQ(Fault::ALTITUDE_LIMIT, result.fault);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.z);
}

TEST(SafetyPolicy, LimitsAccelerationFromPreviousOutput) {
  Limits limits;
  limits.max_xy_speed = 10.0;
  limits.max_xy_acceleration = 2.0;
  SafetyPolicy policy(limits);
  geometry_msgs::Twist input;
  input.linear.x = 8.0;
  const auto result = policy.apply(input, 3.0, 0.05);
  EXPECT_NEAR(0.1, result.command.linear.x, 1e-9);
}
```

- [ ] **Step 4: 运行 GTest 并确认链接或符号失败**

Run:

```bash
catkin_make safety_policy_test
```

Expected: FAIL，因为 `safety_policy.h/.cpp` 尚不存在。

- [ ] **Step 5: 实现最小安全策略接口**

Create `include/safety_filter/safety_policy.h`:

```cpp
#pragma once

#include <geometry_msgs/Twist.h>

namespace safety_filter {

enum class Fault { NONE, NON_FINITE_COMMAND, INVALID_DT, ALTITUDE_LIMIT };

struct Limits {
  double max_xy_speed{3.0};
  double max_z_speed{1.0};
  double max_yaw_rate{1.0};
  double max_xy_acceleration{2.0};
  double max_z_acceleration{1.0};
  double min_altitude{0.5};
  double max_altitude{5.5};
};

struct Result {
  geometry_msgs::Twist command;
  Fault fault{Fault::NONE};
};

class SafetyPolicy {
 public:
  explicit SafetyPolicy(const Limits& limits);
  Result apply(const geometry_msgs::Twist& requested, double altitude, double dt);
  void reset();

 private:
  Limits limits_;
  geometry_msgs::Twist previous_;
};

const char* faultCode(Fault fault);
geometry_msgs::Twist zeroCommand();

}  // namespace safety_filter
```

Implement `src/safety_policy.cpp` with these exact rules:

```cpp
#include "safety_filter/safety_policy.h"

#include <algorithm>
#include <cmath>

namespace safety_filter {
namespace {

double clamp(double value, double limit) {
  return std::max(-limit, std::min(limit, value));
}

bool finite(const geometry_msgs::Twist& value) {
  return std::isfinite(value.linear.x) && std::isfinite(value.linear.y) &&
         std::isfinite(value.linear.z) && std::isfinite(value.angular.x) &&
         std::isfinite(value.angular.y) && std::isfinite(value.angular.z);
}

}  // namespace

SafetyPolicy::SafetyPolicy(const Limits& limits) : limits_(limits) {}

geometry_msgs::Twist zeroCommand() { return geometry_msgs::Twist{}; }

void SafetyPolicy::reset() { previous_ = zeroCommand(); }

Result SafetyPolicy::apply(const geometry_msgs::Twist& requested,
                           double altitude, double dt) {
  if (!finite(requested) || !std::isfinite(altitude)) {
    reset();
    return {zeroCommand(), Fault::NON_FINITE_COMMAND};
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    reset();
    return {zeroCommand(), Fault::INVALID_DT};
  }

  Result result;
  result.command = requested;
  const double horizontal = std::hypot(result.command.linear.x,
                                       result.command.linear.y);
  if (horizontal > limits_.max_xy_speed && horizontal > 0.0) {
    const double scale = limits_.max_xy_speed / horizontal;
    result.command.linear.x *= scale;
    result.command.linear.y *= scale;
  }
  result.command.linear.z = clamp(result.command.linear.z, limits_.max_z_speed);
  result.command.angular.x = 0.0;
  result.command.angular.y = 0.0;
  result.command.angular.z = clamp(result.command.angular.z, limits_.max_yaw_rate);

  const bool altitude_limited =
      (altitude >= limits_.max_altitude && result.command.linear.z > 0.0) ||
      (altitude <= limits_.min_altitude && result.command.linear.z < 0.0);

  const double max_xy_step = limits_.max_xy_acceleration * dt;
  const double delta_x = result.command.linear.x - previous_.linear.x;
  const double delta_y = result.command.linear.y - previous_.linear.y;
  const double delta_xy = std::hypot(delta_x, delta_y);
  if (delta_xy > max_xy_step && delta_xy > 0.0) {
    const double scale = max_xy_step / delta_xy;
    result.command.linear.x = previous_.linear.x + delta_x * scale;
    result.command.linear.y = previous_.linear.y + delta_y * scale;
  }
  const double max_z_step = limits_.max_z_acceleration * dt;
  result.command.linear.z = previous_.linear.z +
      clamp(result.command.linear.z - previous_.linear.z, max_z_step);
  // Apply the hard altitude gate after acceleration limiting. Otherwise a
  // previous upward velocity could be reintroduced at the upper boundary.
  if (altitude_limited) {
    result.command.linear.z = 0.0;
    result.fault = Fault::ALTITUDE_LIMIT;
  }
  previous_ = result.command;
  return result;
}

const char* faultCode(Fault fault) {
  switch (fault) {
    case Fault::NONE: return "OK";
    case Fault::NON_FINITE_COMMAND: return "NON_FINITE_COMMAND";
    case Fault::INVALID_DT: return "INVALID_DT";
    case Fault::ALTITUDE_LIMIT: return "ALTITUDE_LIMIT";
  }
  return "UNKNOWN_FAULT";
}

}  // namespace safety_filter
```

- [ ] **Step 6: 构建并运行策略测试**

Run:

```bash
catkin_make safety_policy_test
devel/lib/safety_filter/safety_policy_test
```

Expected: 4 个测试全部 PASS。

- [ ] **Step 7: 提交纯策略和包骨架**

```bash
git add src/ego_fusion_search/safety_filter src/competition_compliance/config/ownership.json src/competition_compliance/test/test_ownership.py
git commit -m "feat: add final command safety policy"
```

### Task 4: 实现 ROS 看门狗节点和六机 Launch

**Files:**
- Modify: `src/ego_fusion_search/safety_filter/CMakeLists.txt`
- Create: `src/ego_fusion_search/safety_filter/src/safety_filter_node.cpp`
- Create: `src/ego_fusion_search/safety_filter/config/default.yaml`
- Create: `src/ego_fusion_search/safety_filter/launch/safety_filter_swarm.launch`
- Create: `src/ego_fusion_search/safety_filter/README.md`
- Create: `src/ego_fusion_search/safety_filter/test/safety_filter_node.test`
- Create: `src/ego_fusion_search/safety_filter/test/test_safety_filter_node.py`
- Modify: `src/look_up/launch/down_resume.launch`

- [ ] **Step 1: 写节点行为测试要求**

在 `safety_policy_test.cpp` 追加 reset 测试，确保看门狗故障后不会从旧非零命令继续加速：

```cpp
TEST(SafetyPolicy, ResetClearsPreviousOutput) {
  Limits limits;
  limits.max_xy_speed = 10.0;
  limits.max_xy_acceleration = 2.0;
  SafetyPolicy policy(limits);
  geometry_msgs::Twist input;
  input.linear.x = 8.0;
  EXPECT_NEAR(0.2, policy.apply(input, 3.0, 0.1).command.linear.x, 1e-9);
  policy.reset();
  EXPECT_NEAR(0.2, policy.apply(input, 3.0, 0.1).command.linear.x, 1e-9);
}
```

Run `catkin_make safety_policy_test && devel/lib/safety_filter/safety_policy_test` and expect 5 tests PASS；该步骤保护后续节点的 timeout reset 语义。

Create `test/safety_filter_node.test` before implementing the node:

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="safety_filter" type="safety_filter_node"
        name="safety_filter_under_test" required="true">
    <param name="raw_command_topic" value="/test/raw_cmd"/>
    <param name="odom_topic" value="/test/odom"/>
    <param name="final_command_topic" value="/test/final_cmd"/>
    <param name="status_topic" value="/test/status"/>
    <param name="publish_rate" value="50.0"/>
    <param name="command_timeout" value="0.15"/>
    <param name="odom_timeout" value="0.15"/>
    <param name="max_xy_acceleration" value="100.0"/>
    <param name="max_z_acceleration" value="100.0"/>
  </node>
  <test test-name="safety_filter_node_test" pkg="safety_filter"
        type="test_safety_filter_node.py" time-limit="15"/>
</launch>
```

Create executable `test/test_safety_filter_node.py`:

```python
#!/usr/bin/env python3

import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class SafetyFilterNodeTest(unittest.TestCase):
    def setUp(self):
        self.command = None
        self.status = None
        self.raw_pub = rospy.Publisher("/test/raw_cmd", Twist, queue_size=1)
        self.odom_pub = rospy.Publisher("/test/odom", Odometry, queue_size=1)
        rospy.Subscriber("/test/final_cmd", Twist, self._command_callback)
        rospy.Subscriber("/test/status", String, self._status_callback)

    def _command_callback(self, message):
        self.command = message

    def _status_callback(self, message):
        self.status = message.data

    def _wait_for(self, predicate, timeout=2.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        self.fail("timed out waiting for safety_filter state")

    def _publish_for(self, duration, publish_raw, publish_odom):
        deadline = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if publish_raw:
                command = Twist()
                command.linear.x = 1.0
                self.raw_pub.publish(command)
            if publish_odom:
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.pose.pose.position.z = 3.0
                self.odom_pub.publish(odom)
            rate.sleep()

    def test_watchdogs_are_fail_closed_and_recover(self):
        self._wait_for(
            lambda: self.status == "ODOM_TIMEOUT"
            and self.command is not None
            and self.command.linear.x == 0.0
        )
        self.assertAlmostEqual(0.0, self.command.linear.x)

        self._publish_for(0.20, publish_raw=False, publish_odom=True)
        self._wait_for(
            lambda: self.status == "COMMAND_TIMEOUT"
            and self.command.linear.x == 0.0
        )
        self.assertAlmostEqual(0.0, self.command.linear.x)

        self._publish_for(0.20, publish_raw=True, publish_odom=True)
        self._wait_for(
            lambda: self.status == "OK" and self.command.linear.x > 0.0
        )

        self._publish_for(0.30, publish_raw=False, publish_odom=True)
        self._wait_for(
            lambda: self.status == "COMMAND_TIMEOUT"
            and self.command.linear.x == 0.0
        )
        self.assertAlmostEqual(0.0, self.command.linear.x)

        self._publish_for(0.30, publish_raw=True, publish_odom=False)
        self._wait_for(
            lambda: self.status == "ODOM_TIMEOUT"
            and self.command.linear.x == 0.0
        )
        self.assertAlmostEqual(0.0, self.command.linear.x)


if __name__ == "__main__":
    rospy.init_node("safety_filter_node_test")
    rostest.rosrun("safety_filter", "safety_filter_node_test", SafetyFilterNodeTest)
```

Add to the existing `if(CATKIN_ENABLE_TESTING)` block:

```cmake
find_package(rostest REQUIRED)
add_rostest(test/safety_filter_node.test)
```

Run `catkin_make run_tests_safety_filter` and expect FAIL because `safety_filter_node` does not exist yet. This is the red step for command and odometry watchdog behavior.

- [ ] **Step 2: 实现 ROS 节点**

Create complete `src/safety_filter_node.cpp`:

```cpp
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "safety_filter/safety_policy.h"

class SafetyFilterNode {
 public:
  SafetyFilterNode() : nh_(), private_nh_("~") {
    std::string raw_topic;
    std::string odom_topic;
    std::string final_topic;
    std::string status_topic;
    double publish_rate = 20.0;
    safety_filter::Limits limits;

    private_nh_.param("raw_command_topic", raw_topic,
                      std::string("control/raw_cmd_vel"));
    private_nh_.param("odom_topic", odom_topic,
                      std::string("global_odom"));
    private_nh_.param("final_command_topic", final_topic,
                      std::string("final_cmd_vel"));
    private_nh_.param("status_topic", status_topic,
                      std::string("safety/status"));
    private_nh_.param("publish_rate", publish_rate, 20.0);
    private_nh_.param("command_timeout", command_timeout_, 0.25);
    private_nh_.param("odom_timeout", odom_timeout_, 0.25);
    private_nh_.param("max_xy_speed", limits.max_xy_speed, 3.0);
    private_nh_.param("max_z_speed", limits.max_z_speed, 1.0);
    private_nh_.param("max_yaw_rate", limits.max_yaw_rate, 1.0);
    private_nh_.param("max_xy_acceleration", limits.max_xy_acceleration, 2.0);
    private_nh_.param("max_z_acceleration", limits.max_z_acceleration, 1.0);
    private_nh_.param("min_altitude", limits.min_altitude, 0.5);
    private_nh_.param("max_altitude", limits.max_altitude, 5.5);

    if (!std::isfinite(publish_rate) || publish_rate <= 0.0 ||
        !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
        !std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0 ||
        !std::isfinite(limits.max_xy_speed) || limits.max_xy_speed <= 0.0 ||
        !std::isfinite(limits.max_z_speed) || limits.max_z_speed <= 0.0 ||
        !std::isfinite(limits.max_yaw_rate) || limits.max_yaw_rate <= 0.0 ||
        !std::isfinite(limits.max_xy_acceleration) ||
        limits.max_xy_acceleration <= 0.0 ||
        !std::isfinite(limits.max_z_acceleration) ||
        limits.max_z_acceleration <= 0.0 ||
        !std::isfinite(limits.min_altitude) ||
        !std::isfinite(limits.max_altitude) ||
        limits.min_altitude >= limits.max_altitude) {
      throw std::invalid_argument("invalid safety_filter timing or altitude parameters");
    }

    policy_.reset(new safety_filter::SafetyPolicy(limits));
    raw_command_sub_ = nh_.subscribe(
        raw_topic, 1, &SafetyFilterNode::rawCommandCallback, this);
    odom_sub_ = nh_.subscribe(
        odom_topic, 1, &SafetyFilterNode::odomCallback, this);
    final_command_pub_ = nh_.advertise<geometry_msgs::Twist>(final_topic, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic, 1, true);
    publish_timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_rate), &SafetyFilterNode::tick, this);
  }

 private:
  void rawCommandCallback(const geometry_msgs::Twist::ConstPtr& message) {
    latest_raw_command_ = *message;
    raw_command_received_at_ = ros::Time::now();
    has_raw_command_ = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    latest_altitude_ = message->pose.pose.position.z;
    odom_received_at_ = ros::Time::now();
    has_odom_ = true;
  }

  void tick(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    std::string status;
    geometry_msgs::Twist output;
    if (!has_odom_ || (now - odom_received_at_).toSec() > odom_timeout_) {
      policy_->reset();
      output = safety_filter::zeroCommand();
      status = "ODOM_TIMEOUT";
    } else if (!has_raw_command_ ||
               (now - raw_command_received_at_).toSec() > command_timeout_) {
      policy_->reset();
      output = safety_filter::zeroCommand();
      status = "COMMAND_TIMEOUT";
    } else {
      const double dt = previous_tick_.isZero()
                            ? 0.05
                            : (now - previous_tick_).toSec();
      const auto result = policy_->apply(
          latest_raw_command_, latest_altitude_, dt);
      output = result.command;
      status = safety_filter::faultCode(result.fault);
    }
    previous_tick_ = now;
    final_command_pub_.publish(output);
    std_msgs::String message;
    message.data = status;
    status_pub_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber raw_command_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher final_command_pub_;
  ros::Publisher status_pub_;
  ros::Timer publish_timer_;
  geometry_msgs::Twist latest_raw_command_;
  double latest_altitude_{0.0};
  ros::Time raw_command_received_at_;
  ros::Time odom_received_at_;
  ros::Time previous_tick_;
  bool has_raw_command_{false};
  bool has_odom_{false};
  double command_timeout_{0.25};
  double odom_timeout_{0.25};
  std::unique_ptr<safety_filter::SafetyPolicy> policy_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "safety_filter");
  try {
    SafetyFilterNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("safety_filter initialization failed: %s", error.what());
    return 1;
  }
  return 0;
}
```

Then add the runtime target and runtime resources to `CMakeLists.txt`:

```cmake
add_executable(safety_filter_node src/safety_filter_node.cpp)
target_link_libraries(safety_filter_node safety_policy ${catkin_LIBRARIES})

install(TARGETS safety_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

- [ ] **Step 3: 添加集中参数**

Create `config/default.yaml`:

```yaml
publish_rate: 20.0
command_timeout: 0.25
odom_timeout: 0.25
max_xy_speed: 3.0
max_z_speed: 1.0
max_yaw_rate: 1.0
max_xy_acceleration: 2.0
max_z_acceleration: 1.0
min_altitude: 0.5
max_altitude: 5.5
```

Do not hard-code a second parameter set in Launch.

- [ ] **Step 4: 添加递归六机 Launch**

Create complete `launch/safety_filter_swarm.launch`:

```xml
<?xml version="1.0"?>
<launch>
  <arg name="num_drones" default="6"/>
  <arg name="vehicle_type" default="typhoon_h480"/>
  <arg name="drone_id" default="0"/>

  <group if="$(eval drone_id &lt; num_drones)">
    <group ns="$(arg vehicle_type)_$(arg drone_id)">
      <node pkg="safety_filter" type="safety_filter_node"
            name="safety_filter" output="screen" required="true">
        <rosparam command="load"
                  file="$(find safety_filter)/config/default.yaml"/>
        <param name="raw_command_topic"
               value="/$(arg vehicle_type)_$(arg drone_id)/control/raw_cmd_vel"/>
        <param name="odom_topic"
               value="/$(arg vehicle_type)_$(arg drone_id)/global_odom"/>
        <param name="final_command_topic"
               value="/xtdrone/$(arg vehicle_type)_$(arg drone_id)/cmd_vel_flu"/>
        <param name="status_topic"
               value="/$(arg vehicle_type)_$(arg drone_id)/safety/status"/>
      </node>
    </group>

    <include file="$(find safety_filter)/launch/safety_filter_swarm.launch">
      <arg name="num_drones" value="$(arg num_drones)"/>
      <arg name="vehicle_type" value="$(arg vehicle_type)"/>
      <arg name="drone_id" value="$(eval drone_id + 1)"/>
    </include>
  </group>
</launch>
```

- [ ] **Step 5: 先把 safety filter 加到主 Launch，尚不改 MUX**

In `down_resume.launch`, include `safety_filter_swarm.launch` immediately before `spawn_mux_swarm.launch` and pass `num_drones`.

At this intermediate point, do not run live simulation: both the old MUX and safety filter would publish the final topic. Only build and run unit/static tests until Task 5 atomically rewires MUX and fly.

- [ ] **Step 6: 写包 README**

Document inputs, outputs, default limits, timeout behavior, and the rule that no other team node may publish the final XTDrone topic. Explicitly state that depth stop-distance filtering is a later phase and is not claimed here.

- [ ] **Step 7: 构建，不提交中间双发布者状态**

Run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_safety_filter
catkin_test_results build/safety_filter
python3 tests/test_control_safety_wiring.py -v
```

Expected: build PASS；5 个 policy tests 和 watchdog rostest PASS；wiring test仍因 MUX/fly 未改而失败。保持 Task 4 变更未提交，直接进入 Task 5。

### Task 5: 原子切换三路 MUX 和 takeoff 控制权

**Files:**
- Modify: `src/look_up/launch/spawn_mux_swarm.launch`
- Modify: `src/mix_nav/fly/include/fly/fly_takeoff.h`
- Modify: `src/mix_nav/fly/src/fly_takeoff.cpp`
- Modify: `src/mix_nav/fly/CMakeLists.txt`
- Modify: `src/mix_nav/fly/package.xml`
- Modify: `src/mix_nav/fly/test/test_fly_launch.py`
- Test: `tests/test_control_safety_wiring.py`

- [ ] **Step 1: 扩展 fly 失败测试**

Replace `test_completed_aircraft_do_not_receive_more_climb_commands` with assertions that preserve the old rule and add MUX handoff:

```python
def test_completed_aircraft_do_not_receive_more_climb_commands(self):
    source_file = pathlib.Path(__file__).parents[1] / "src" / "fly_takeoff.cpp"
    compacted_source = "".join(source_file.read_text().split())
    self.assertIn(
        "if(!mission_done_flags_[i]){vel_pubs_[i].publish(climb_twist);}",
        compacted_source,
    )

def test_takeoff_uses_mux_input_and_hands_off_to_navigator(self):
    source_file = pathlib.Path(__file__).parents[1] / "src" / "fly_takeoff.cpp"
    source = source_file.read_text()
    compacted = "".join(source.split())
    self.assertIn("/mux_inputs/takeoff/cmd_vel", source)
    self.assertNotIn('"/cmd_vel_flu"', source)
    self.assertIn("selectControl(i,takeoff_topic)", compacted)
    self.assertIn("selectControl(i,navigator_topic)", compacted)
    failure_guard = source.rindex("if (!allMissionDone())")
    navigator_handoff = source.rindex("selectControl(i, navigator_topic)")
    self.assertLess(failure_guard, navigator_handoff)
```

Run both Python test modules and expect failure before implementation.

- [ ] **Step 2: 修改 MUX 拓扑**

In `spawn_mux_swarm.launch` define:

```xml
<arg name="mux_output" value="/$(arg vehicle_type)_$(arg drone_id)/control/raw_cmd_vel"/>
<arg name="input_takeoff" value="/$(arg vehicle_type)_$(arg drone_id)/mux_inputs/takeoff/cmd_vel"/>
<arg name="input_navigator" value="/$(arg vehicle_type)_$(arg drone_id)/mux_inputs/navigator/cmd_vel"/>
<arg name="input_external" value="/$(arg vehicle_type)_$(arg drone_id)/mux_inputs/external/pose_cmd"/>
```

Set node args and initial topic:

```xml
args="$(arg mux_output) $(arg input_takeoff) $(arg input_navigator) $(arg input_external) mux:=$(arg mux_name)"
<param name="initial_topic" value="$(arg input_takeoff)"/>
```

- [ ] **Step 3: 给 fly 添加 MUX 客户端**

Add `topic_tools` to `find_package`, `CATKIN_DEPENDS`, `package.xml` build/exec dependencies, and include `topic_tools/MuxSelect.h` in the header.

Add members:

```cpp
std::vector<ros::ServiceClient> mux_select_clients_;
bool selectControl(int drone_id, const std::string& topic_name);
void publishZeroVelocity();
```

For every drone constructor iteration, create client:

```cpp
const std::string vehicle = drone_name_ + "_" + std::to_string(i);
const std::string vel_topic = "/" + vehicle + "/mux_inputs/takeoff/cmd_vel";
vel_pubs_.push_back(nh_.advertise<geometry_msgs::Twist>(vel_topic, 1));
const std::string service = "/" + vehicle + "/pose_cmd_mux/select";
mux_select_clients_.push_back(nh_.serviceClient<topic_tools::MuxSelect>(service));
```

Implement:

```cpp
bool ConfidentTakeoff::selectControl(int drone_id, const std::string& topic_name) {
  topic_tools::MuxSelect request;
  request.request.topic = topic_name;
  if (!mux_select_clients_.at(drone_id).waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR("MUX service unavailable for drone %d", drone_id);
    return false;
  }
  if (!mux_select_clients_.at(drone_id).call(request)) {
    ROS_ERROR("Failed to select MUX input for drone %d", drone_id);
    return false;
  }
  return true;
}

void ConfidentTakeoff::publishZeroVelocity() {
  const geometry_msgs::Twist zero;
  for (auto& publisher : vel_pubs_) publisher.publish(zero);
}
```

- [ ] **Step 4: 使起飞切换顺序 fail-closed**

Before sending `OFFBOARD`/`ARM`, loop over all drones and call:

```cpp
const std::string vehicle = drone_name_ + "_" + std::to_string(i);
const std::string takeoff_topic = "/" + vehicle + "/mux_inputs/takeoff/cmd_vel";
if (!selectControl(i, takeoff_topic)) {
  publishZeroVelocity();
  ROS_ERROR("Takeoff aborted because MUX ownership was not confirmed.");
  return;
}
```

After the climb loop, replace the old handoff tail with this fail-closed order:

```cpp
publishZeroVelocity();
ros::Duration(0.25).sleep();

cmd_msg.data = "HOVER";
for (auto& publisher : cmd_pubs_) {
  for (int repeat = 0; repeat < 5; ++repeat) {
    publisher.publish(cmd_msg);
    rate_.sleep();
  }
}

if (!allMissionDone()) {
  ROS_ERROR("Takeoff incomplete; keeping zero takeoff input selected.");
  return;
}

for (int i = 0; i < drone_quantity_; ++i) {
  const std::string vehicle = drone_name_ + "_" + std::to_string(i);
  const std::string navigator_topic =
      "/" + vehicle + "/mux_inputs/navigator/cmd_vel";
  if (!selectControl(i, navigator_topic)) {
    ROS_ERROR("Navigator handoff failed for drone %d; takeoff input stays zero.", i);
  }
}
```

Zero must pass through MUX and safety filter before `HOVER`. A timeout or partial takeoff must keep the zero takeoff input selected; it must not hand control to the already-running navigator. Preserve the existing independent altitude stop condition.

- [ ] **Step 5: 运行静态、fly 和策略测试**

Run:

```bash
python3 tests/test_control_safety_wiring.py -v
python3 src/mix_nav/fly/test/test_fly_launch.py -v
catkin_make safety_policy_test
devel/lib/safety_filter/safety_policy_test
```

Expected: all static tests, 3 fly tests, and 5 policy tests PASS。

- [ ] **Step 6: 构建完整工作区**

Run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Expected: `safety_filter_node` and `fly_takeoff` build successfully; no external tree changes.

- [ ] **Step 7: 提交原子控制链切换**

```bash
git add tests/test_control_safety_wiring.py src/look_up/launch/spawn_mux_swarm.launch src/look_up/launch/down_resume.launch src/mix_nav/fly src/ego_fusion_search/safety_filter
git commit -m "feat: route flight commands through safety filter"
```

### Task 6: 添加最终发布者守卫

**Files:**
- Create: `scripts/check_final_control_publishers.py`
- Create: `tests/test_final_control_publishers.py`
- Modify: `scripts/smoke_competition_clean.sh`

- [ ] **Step 1: 写纯函数失败测试**

Create `tests/test_final_control_publishers.py`:

```python
#!/usr/bin/env python3

import importlib.util
import pathlib
import unittest

ROOT = pathlib.Path(__file__).parents[1]
SCRIPT = ROOT / "scripts/check_final_control_publishers.py"


def load_module():
    spec = importlib.util.spec_from_file_location("publisher_guard", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FinalControlPublisherTest(unittest.TestCase):
    def test_accepts_exactly_one_expected_publisher_per_vehicle(self):
        module = load_module()
        publishers = []
        for drone_id in range(6):
            publishers.append((
                f"/xtdrone/typhoon_h480_{drone_id}/cmd_vel_flu",
                [f"/typhoon_h480_{drone_id}/safety_filter"],
            ))
        self.assertEqual([], module.validate_publishers(publishers, 6, "typhoon_h480"))

    def test_rejects_missing_unexpected_and_duplicate_publishers(self):
        module = load_module()
        publishers = [
            ("/xtdrone/typhoon_h480_0/cmd_vel_flu", []),
            ("/xtdrone/typhoon_h480_1/cmd_vel_flu", [
                "/typhoon_h480_1/safety_filter", "/confident_takeoff_node"
            ]),
        ]
        errors = module.validate_publishers(publishers, 2, "typhoon_h480")
        self.assertEqual(2, len(errors))
        self.assertIn("vehicle 0", errors[0])
        self.assertIn("vehicle 1", errors[1])


if __name__ == "__main__":
    unittest.main()
```

Run `python3 tests/test_final_control_publishers.py -v` and expect import failure because the script does not exist.

- [ ] **Step 2: 实现守卫脚本**

Create executable `scripts/check_final_control_publishers.py`:

```python
#!/usr/bin/env python3

import argparse
import sys


def validate_publishers(publishers, count, vehicle_type):
    by_topic = dict(publishers)
    errors = []
    for drone_id in range(count):
        topic = f"/xtdrone/{vehicle_type}_{drone_id}/cmd_vel_flu"
        expected = [f"/{vehicle_type}_{drone_id}/safety_filter"]
        actual = sorted(by_topic.get(topic, []))
        if actual != expected:
            errors.append(
                f"vehicle {drone_id}: expected {expected}, got {actual} on {topic}"
            )
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=6)
    parser.add_argument("--vehicle-type", default="typhoon_h480")
    args = parser.parse_args()
    import rosgraph
    master = rosgraph.Master("/competition_clean_final_publisher_guard")
    publishers, _, _ = master.getSystemState()
    errors = validate_publishers(publishers, args.count, args.vehicle_type)
    if errors:
        for error in errors:
            print(f"FAIL {error}", file=sys.stderr)
        return 1
    print("PASS final control topics have one safety_filter publisher each")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

Set executable mode and rerun the unit test; expect 2 tests PASS.

- [ ] **Step 3: 集成 smoke**

In `scripts/smoke_competition_clean.sh`:

1. add `python3` to required commands;
2. after the six-vehicle topic/node loop, run:

```bash
if ! python3 "$WORKSPACE_DIR/scripts/check_final_control_publishers.py" \
    --count 6 --vehicle-type typhoon_h480 | tee -a "$REPORT"; then
    log_line "FAIL final control publisher ownership" >&2
    exit 1
fi
```

3. For each vehicle also call `check_node "/typhoon_h480_${id}/safety_filter"` and `check_message "/typhoon_h480_${id}/safety/status"`.

- [ ] **Step 4: 运行守卫和脚本测试**

Run:

```bash
python3 tests/test_final_control_publishers.py -v
python3 tests/test_verification_scripts.py -v
bash -n scripts/smoke_competition_clean.sh
```

Expected: unit tests PASS and shell syntax check exits 0。

- [ ] **Step 5: 提交发布者守卫**

```bash
git add scripts/check_final_control_publishers.py scripts/smoke_competition_clean.sh tests/test_final_control_publishers.py
git commit -m "test: enforce final command publisher ownership"
```

### Task 7: 文档、完整验证与真实六机验收

**Files:**
- Modify: `docs/AI_AGENT_HANDOFF.md`
- Modify: `docs/COMPLIANCE.md`
- Modify: `src/ego_fusion_search/safety_filter/README.md`
- Test: full repository and live simulation

- [ ] **Step 1: 更新维护文档**

In `AI_AGENT_HANDOFF.md`, replace the old two-input/direct-output diagram with:

```text
fly_takeoff -> takeoff input -----+
simple_navigator -> navigator ----+-> pose_cmd_mux -> raw_cmd_vel
tracking -> external -------------+                     |
                                                   safety_filter
                                                        |
                                     XTDrone cmd_vel_flu (唯一发布者)
```

State clearly that future EGO adapter replaces `simple_navigator` as navigator-input publisher, never adds another MUX input or final publisher.

In `COMPLIANCE.md`, add a “控制发布边界” section saying all control-source changes remain team code and external official trees stay read-only. Do not add safety files to `official_manifest.json`, because they are team files, not official inputs.

- [ ] **Step 2: 运行快速 Python 和 ownership 回归**

Run:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 -m unittest discover -s src/competition_compliance/test -p 'test_ownership.py'
```

Expected: all repository Python tests PASS；ownership test仍显示 39 tests 或因新增明确测试而增加，但 failures/errors 必须为 0。

- [ ] **Step 3: 运行完整 competition-clean 验证**

Run:

```bash
bash scripts/verify_competition_clean.sh
```

Expected final line: `完整验证通过：静态与构建后合规证据均已生成。` Catkin test results show 0 errors and 0 failures.

- [ ] **Step 4: 启动真实六机仿真**

From a graphical desktop terminal:

```bash
bash 1.sh 6 mission_down.json
```

Acceptance evidence during takeoff:

```bash
rosservice call /typhoon_h480_0/pose_cmd_mux/select "/typhoon_h480_0/mux_inputs/takeoff/cmd_vel"
python3 scripts/check_final_control_publishers.py --count 6 --vehicle-type typhoon_h480
```

Do not repeatedly call the service during normal flight; this single manual call is only a controlled verification if automatic selection evidence is unclear.

Expected:

- 6 架分别停止爬升，不超过目标高度和 5.5m 软件上限；
- 起飞完成后 MUX 自动切到 navigator；
- publisher guard prints PASS；
- safety status is `OK` during fresh healthy commands；
- stop `simple_navigator` or block `raw_cmd_vel` and status changes to `COMMAND_TIMEOUT`, final command becomes zero within 0.25s；
- stop odom input in a controlled single-vehicle test and status changes to `ODOM_TIMEOUT`；
- tracking 切 external 后仍只有 safety filter 发布最终话题。

- [ ] **Step 5: 运行正式 smoke**

In a second terminal while simulation is healthy:

```bash
bash scripts/smoke_competition_clean.sh
```

Expected final line: `PASS competition-clean six-vehicle smoke`，并包含 final publisher ownership PASS。

- [ ] **Step 6: 验证退出和官方目录未改变**

Press `Ctrl-C` in the launcher terminal, then run:

```bash
pgrep -af 'px4|gzserver|gzclient|yolo11n|multirotor_communication|safety_filter_node'
git -C /home/wangtao/robocup_fly/PX4_Firmware status --short
git -C /home/wangtao/robocup_fly/XTDrone status --short
git status --short
```

Expected: no project processes remain; official trees show no new changes; repository only has intended documentation or generated ignored artifacts.

- [ ] **Step 7: 提交文档和验收更新**

```bash
git add docs/AI_AGENT_HANDOFF.md docs/COMPLIANCE.md src/ego_fusion_search/safety_filter/README.md
git commit -m "docs: document final control safety boundary"
```

- [ ] **Step 8: 最终审查提交范围**

Run:

```bash
git status --short --branch
git log --oneline --decorate -8
git diff public/competition-clean...HEAD --stat
```

Expected: 工作树干净；提交只涉及队伍 ROS 包、队伍 Launch、测试、脚本和文档；无 PX4、XTDrone、Gazebo、EGO 或基础模型文件。

## 明确延后到下一份计划的内容

本计划通过后，下一份计划是“单机 EGO 适配”，只处理：

- 固定并验证外部只读 EGO-Planner-Swarm 提交；
- 单机 odom/depth/goal/PositionCommand 接口；
- `ego_adapter` 的 ENU 到 FLU 转换；
- 让 `ego_adapter` 替换 `simple_navigator` 成为 navigator 输入唯一发布者；
- 单机绕障、指令过期和轨迹失效测试。

不得在控制安全骨架未通过真实六机验收前开始上述内容。
