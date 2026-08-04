# 固定巡逻静态安全基线 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将六机固定巡逻改成一次性安全进入 `3 x 2` 独立责任区后各自闭环，并把巡逻高度、任务校验和跨机路线净空固化为可自动验证的安全基线。

**Architecture:** `task_manager` 先把 JSON 完整解析为经过校验的 `MissionDefinition`，再为每机运行一个带 `ENTERING/PATROLLING` 活动阶段的任务管理器；跟踪暂停只暂存活动阶段，返航后恢复原阶段。Python 几何测试把出生点、进入路线、闭环路线、静态障碍和跨机 5 米净空视为一个完整任务图；`safety_filter` 仍是唯一最终速度发布者，只将高度上限收紧到 4.0 米。

**Tech Stack:** Ubuntu 20.04、ROS Noetic、C++14、JsonCpp、Gazebo Classic 11、PX4 SITL 1.11、Python 3.8 `unittest`、Catkin/GTest/Rostest、Bash。

---

## 文件结构与职责

本计划只触及队伍文件：

- 新建 `src/mix_nav/task_manager/include/task_manager/mission_definition.h`：定义 `Waypoint`、`MissionDefinition` 和任务文件解析接口。
- 新建 `src/mix_nav/task_manager/src/mission_definition.cpp`：集中执行 JSON 结构、有限数值、唯一编号和请求完整性校验。
- 新建 `src/mix_nav/task_manager/include/task_manager/mission_progress.h`：定义与 ROS 无关的进入/巡逻进度状态机。
- 新建 `src/mix_nav/task_manager/src/mission_progress.cpp`：实现一次性进入、循环索引、暂停和恢复。
- 新建 `src/mix_nav/task_manager/test/mission_definition_test.cpp`：测试任务文件整体校验和旧格式兼容。
- 新建 `src/mix_nav/task_manager/test/mission_progress_test.cpp`：测试进入只执行一次及暂停后恢复原阶段。
- 修改 `src/mix_nav/task_manager/src/multi_mission_launcher.cpp`：先完整校验，再创建全部任务线程。
- 修改 `src/mix_nav/task_manager/include/task_manager/mission_manager.h` 和 `src/mix_nav/task_manager/src/mission_manager.cpp`：接入进入阶段和进度状态机。
- 修改 `src/mix_nav/task_manager/test/test_mission_clearance.py`：成为固定任务几何契约的唯一测试入口。
- 修改 `src/mix_nav/task_manager/launch/mission_down.json`：写入六机 3.5 米进入路线和闭环路线。
- 将 `waypoint/mission_down.json` 改为相对符号链接：保留可见入口但消除第二份内容。
- 修改 `src/ego_fusion_search/safety_filter/config/default.yaml`、`include/safety_filter/safety_policy.h` 和 `src/safety_filter_node.cpp`：默认最高高度统一为 4.0 米。
- 修改 `src/ego_fusion_search/safety_filter/test/safety_policy_test.cpp`、`test/test_safety_filter_node.py` 和 `test/safety_filter_node.test`：固化默认配置和运行态上边界。
- 修改 `docs/TROUBLESHOOTING.md` 和 `docs/AI_AGENT_HANDOFF.md`：记录新任务格式、权威文件和验收证据。

外部 `PX4_Firmware`、`XTDrone`、Gazebo、EGO、第三方插件和官方模型全部只读。

---

### Task 1: 在创建线程前完整校验任务文件

**Files:**
- Create: `src/mix_nav/task_manager/include/task_manager/mission_definition.h`
- Create: `src/mix_nav/task_manager/src/mission_definition.cpp`
- Create: `src/mix_nav/task_manager/test/mission_definition_test.cpp`
- Modify: `src/mix_nav/task_manager/src/multi_mission_launcher.cpp`
- Modify: `src/mix_nav/task_manager/CMakeLists.txt`

- [ ] **Step 1: 写任务解析器的失败测试**

新增 `mission_definition_test.cpp`，使用 `std::istringstream` 构造数据，不读真实任务文件。至少包含以下测试：

```cpp
#include <sstream>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "task_manager/mission_definition.h"

TEST(MissionDefinition, ParsesOptionalEntryWaypoints) {
  std::istringstream input(R"json([
    {"vehicle_id":"typhoon_h480_0",
     "entry_waypoints":[{"x":1,"y":2,"z":3.5}],
     "waypoints":[{"x":4,"y":5,"z":3.5}]}
  ])json");
  const auto missions = task_manager::loadMissionDefinitions(
      input, {"typhoon_h480_0"});
  ASSERT_EQ(1u, missions.size());
  ASSERT_EQ(1u, missions[0].entry_waypoints.size());
  EXPECT_DOUBLE_EQ(1.0, missions[0].entry_waypoints[0].x);
  EXPECT_DOUBLE_EQ(4.0, missions[0].patrol_waypoints[0].x);
}

TEST(MissionDefinition, KeepsLegacyMissionWithoutEntryCompatible) {
  std::istringstream input(R"json([
    {"vehicle_id":"typhoon_h480_0",
     "waypoints":[{"x":4,"y":5,"z":3.5}]}
  ])json");
  const auto missions = task_manager::loadMissionDefinitions(
      input, {"typhoon_h480_0"});
  EXPECT_TRUE(missions[0].entry_waypoints.empty());
}

TEST(MissionDefinition, RejectsDuplicateVehicleIds) {
  std::istringstream input(R"json([
    {"vehicle_id":"typhoon_h480_0","waypoints":[{"x":1,"y":2,"z":3}]},
    {"vehicle_id":"typhoon_h480_0","waypoints":[{"x":4,"y":5,"z":3}]}
  ])json");
  EXPECT_THROW(task_manager::loadMissionDefinitions(
                   input, {"typhoon_h480_0"}),
               std::runtime_error);
}

TEST(MissionDefinition, RejectsMissingRequestedVehicleBeforeAnyThreadStarts) {
  std::istringstream input(R"json([
    {"vehicle_id":"typhoon_h480_0","waypoints":[{"x":1,"y":2,"z":3}]}
  ])json");
  EXPECT_THROW(task_manager::loadMissionDefinitions(
                   input, {"typhoon_h480_0", "typhoon_h480_1"}),
               std::runtime_error);
}

TEST(MissionDefinition, RejectsMalformedWaypointsAndEmptyArrays) {
  std::istringstream bad_coordinate(R"json([
    {"vehicle_id":"typhoon_h480_0","waypoints":[{"x":"bad","y":2,"z":3}]}
  ])json");
  EXPECT_THROW(task_manager::loadMissionDefinitions(
                   bad_coordinate, {"typhoon_h480_0"}),
               std::runtime_error);

  std::istringstream empty_entry(R"json([
    {"vehicle_id":"typhoon_h480_0","entry_waypoints":[],
     "waypoints":[{"x":1,"y":2,"z":3}]}
  ])json");
  EXPECT_THROW(task_manager::loadMissionDefinitions(
                   empty_entry, {"typhoon_h480_0"}),
               std::runtime_error);
}
```

- [ ] **Step 2: 注册 GTest 并确认测试因接口不存在而失败**

在 `CMakeLists.txt` 的测试区加入：

```cmake
catkin_add_gtest(mission_definition_test test/mission_definition_test.cpp)
if(TARGET mission_definition_test)
  target_link_libraries(mission_definition_test
    mission_definition_lib ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})
endif()
```

运行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg task_manager
```

预期：编译失败，明确指出 `task_manager/mission_definition.h` 或 `mission_definition_lib` 尚不存在。

- [ ] **Step 3: 实现任务定义和完整校验**

`mission_definition.h` 使用以下公开契约：

```cpp
#pragma once

#include <istream>
#include <string>
#include <vector>

namespace task_manager {

struct Waypoint {
  double x;
  double y;
  double z;
};

struct MissionDefinition {
  std::string vehicle_id;
  std::vector<Waypoint> entry_waypoints;
  std::vector<Waypoint> patrol_waypoints;
};

std::vector<MissionDefinition> loadMissionDefinitions(
    std::istream& input,
    const std::vector<std::string>& requested_vehicle_ids);

}  // namespace task_manager
```

`mission_definition.cpp` 按以下顺序处理，任何失败均抛出带 `vehicle_id` 和字段名的 `std::runtime_error`：

```cpp
#include "task_manager/mission_definition.h"

#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

#include <json/json.h>

namespace task_manager {
namespace {

Waypoint parseWaypoint(const Json::Value& value,
                       const std::string& context) {
  if (!value.isObject() || !value.isMember("x") ||
      !value.isMember("y") || !value.isMember("z") ||
      !value["x"].isNumeric() || !value["y"].isNumeric() ||
      !value["z"].isNumeric()) {
    throw std::runtime_error(context + " must contain numeric x/y/z");
  }
  Waypoint point{value["x"].asDouble(), value["y"].asDouble(),
                 value["z"].asDouble()};
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z)) {
    throw std::runtime_error(context + " contains a non-finite value");
  }
  return point;
}

std::vector<Waypoint> parseWaypointArray(const Json::Value& value,
                                         const std::string& context,
                                         bool require_non_empty) {
  if (!value.isArray() || (require_non_empty && value.empty())) {
    throw std::runtime_error(context + " must be a non-empty array");
  }
  std::vector<Waypoint> points;
  for (Json::ArrayIndex index = 0; index < value.size(); ++index) {
    points.push_back(parseWaypoint(
        value[index], context + "[" + std::to_string(index) + "]"));
  }
  return points;
}

}  // namespace

std::vector<MissionDefinition> loadMissionDefinitions(
    std::istream& input,
    const std::vector<std::string>& requested_vehicle_ids) {
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  if (!Json::parseFromStream(builder, input, &root, &errors) ||
      !root.isArray()) {
    throw std::runtime_error("mission root must be a JSON array: " + errors);
  }

  std::map<std::string, MissionDefinition> by_id;
  for (const auto& item : root) {
    if (!item.isObject() || !item["vehicle_id"].isString() ||
        item["vehicle_id"].asString().empty()) {
      throw std::runtime_error("mission vehicle_id must be a non-empty string");
    }
    MissionDefinition mission;
    mission.vehicle_id = item["vehicle_id"].asString();
    if (by_id.count(mission.vehicle_id) != 0) {
      throw std::runtime_error("duplicate vehicle_id: " + mission.vehicle_id);
    }
    if (item.isMember("entry_waypoints")) {
      mission.entry_waypoints = parseWaypointArray(
          item["entry_waypoints"], mission.vehicle_id + ".entry_waypoints",
          true);
    }
    if (!item.isMember("waypoints")) {
      throw std::runtime_error(mission.vehicle_id + ".waypoints is missing");
    }
    mission.patrol_waypoints = parseWaypointArray(
        item["waypoints"], mission.vehicle_id + ".waypoints", true);
    by_id.emplace(mission.vehicle_id, mission);
  }

  std::set<std::string> requested_once;
  std::vector<MissionDefinition> result;
  for (const auto& id : requested_vehicle_ids) {
    if (!requested_once.insert(id).second) {
      throw std::runtime_error("requested vehicle_id is duplicated: " + id);
    }
    const auto found = by_id.find(id);
    if (found == by_id.end()) {
      throw std::runtime_error("requested vehicle_id is missing: " + id);
    }
    result.push_back(found->second);
  }
  return result;
}

}  // namespace task_manager
```

在 CMake 中新建并链接 `mission_definition_lib`。从 `mission_manager.h` 删除旧的全局 `Waypoint`，改为包含新头文件。

- [ ] **Step 4: 让启动器先完整解析，再创建线程**

`multi_mission_launcher.cpp` 打开文件后先调用：

```cpp
std::vector<task_manager::MissionDefinition> missions;
try {
  missions = task_manager::loadMissionDefinitions(
      mission_file, target_vehicle_ids);
} catch (const std::exception& error) {
  ROS_FATAL("[Launcher] mission validation failed: %s", error.what());
  return 1;
}

std::vector<std::thread> threads;
for (const auto& mission : missions) {
  auto manager = std::make_shared<MissionManager>(mission);
  threads.emplace_back(&MissionManager::run_mission, manager);
}
```

删除旧的“边遍历 JSON 边启动线程”和“缺编号只警告跳过”路径，保证解析失败时线程数为零。

- [ ] **Step 5: 运行任务定义测试并提交**

运行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg task_manager
catkin_make run_tests_task_manager
catkin_test_results build/task_manager
```

预期：`mission_definition_test` 全部通过，Catkin 汇总 0 errors、0 failures。

提交：

```bash
git add src/mix_nav/task_manager/include/task_manager/mission_definition.h \
  src/mix_nav/task_manager/src/mission_definition.cpp \
  src/mix_nav/task_manager/test/mission_definition_test.cpp \
  src/mix_nav/task_manager/include/task_manager/mission_manager.h \
  src/mix_nav/task_manager/src/multi_mission_launcher.cpp \
  src/mix_nav/task_manager/CMakeLists.txt
git commit -m "feat: validate complete swarm missions before launch"
```

---

### Task 2: 分离一次性进入与区域循环进度

**Files:**
- Create: `src/mix_nav/task_manager/include/task_manager/mission_progress.h`
- Create: `src/mix_nav/task_manager/src/mission_progress.cpp`
- Create: `src/mix_nav/task_manager/test/mission_progress_test.cpp`
- Modify: `src/mix_nav/task_manager/include/task_manager/mission_manager.h`
- Modify: `src/mix_nav/task_manager/src/mission_manager.cpp`
- Modify: `src/mix_nav/task_manager/CMakeLists.txt`

- [ ] **Step 1: 写进入只执行一次和暂停恢复的失败测试**

```cpp
#include <gtest/gtest.h>

#include "task_manager/mission_progress.h"

using task_manager::MissionPhase;
using task_manager::MissionProgress;

TEST(MissionProgress, RunsEntryOnceThenLoopsPatrolOnly) {
  MissionProgress progress(2, 3);
  EXPECT_EQ(MissionPhase::ENTERING, progress.phase());
  EXPECT_EQ(0u, progress.index());
  progress.advance();
  EXPECT_EQ(MissionPhase::ENTERING, progress.phase());
  EXPECT_EQ(1u, progress.index());
  progress.advance();
  EXPECT_EQ(MissionPhase::PATROLLING, progress.phase());
  EXPECT_EQ(0u, progress.index());
  progress.advance();
  progress.advance();
  progress.advance();
  EXPECT_EQ(MissionPhase::PATROLLING, progress.phase());
  EXPECT_EQ(0u, progress.index());
}

TEST(MissionProgress, LegacyMissionStartsInPatrol) {
  MissionProgress progress(0, 3);
  EXPECT_EQ(MissionPhase::PATROLLING, progress.phase());
  EXPECT_EQ(0u, progress.index());
}

TEST(MissionProgress, ResumesEntryAtSameIndex) {
  MissionProgress progress(2, 3);
  progress.advance();
  progress.pause();
  EXPECT_TRUE(progress.paused());
  progress.resume();
  EXPECT_FALSE(progress.paused());
  EXPECT_EQ(MissionPhase::ENTERING, progress.phase());
  EXPECT_EQ(1u, progress.index());
}

TEST(MissionProgress, ResumesPatrolAtSameIndex) {
  MissionProgress progress(1, 3);
  progress.advance();
  progress.advance();
  progress.pause();
  progress.resume();
  EXPECT_EQ(MissionPhase::PATROLLING, progress.phase());
  EXPECT_EQ(1u, progress.index());
}
```

- [ ] **Step 2: 注册测试并确认因状态机不存在而失败**

运行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg task_manager
```

预期：缺少 `mission_progress.h` 或 `mission_progress_lib`，测试不能编译。

- [ ] **Step 3: 实现纯进度状态机**

公开接口固定为：

```cpp
#pragma once

#include <cstddef>
#include <stdexcept>

namespace task_manager {

enum class MissionPhase { ENTERING, PATROLLING };

class MissionProgress {
 public:
  MissionProgress(std::size_t entry_count, std::size_t patrol_count);
  MissionPhase phase() const;
  std::size_t index() const;
  bool paused() const;
  void advance();
  void pause();
  void resume();

 private:
  std::size_t entry_count_;
  std::size_t patrol_count_;
  std::size_t entry_index_{0};
  std::size_t patrol_index_{0};
  MissionPhase phase_;
  MissionPhase phase_before_pause_;
  bool paused_{false};
};

}  // namespace task_manager
```

实现规则：构造时 `patrol_count == 0` 抛异常；`entry_count > 0` 从 `ENTERING` 开始，否则从 `PATROLLING` 开始；进入索引到末尾后永久切换为巡逻索引 0；巡逻索引按 `patrol_count` 取模；暂停不改变阶段和索引，恢复还原暂停前阶段；重复暂停或未暂停时恢复均抛出 `std::logic_error`，避免静默损坏状态。

- [ ] **Step 4: 将 MissionManager 接到新进度状态机**

构造函数改为：

```cpp
explicit MissionManager(const task_manager::MissionDefinition& mission);
```

成员改为：

```cpp
std::string vehicle_id_;
std::vector<task_manager::Waypoint> entry_waypoints_;
std::vector<task_manager::Waypoint> patrol_waypoints_;
task_manager::MissionProgress progress_;
```

状态枚举增加 `STATE_ENTERING`，并新增两个私有帮助函数：

```cpp
State activeState() const;
const task_manager::Waypoint& activeWaypoint() const;
```

其行为为：

```cpp
MissionManager::State MissionManager::activeState() const {
  return progress_.phase() == task_manager::MissionPhase::ENTERING
             ? STATE_ENTERING
             : STATE_PATROLLING;
}

const task_manager::Waypoint& MissionManager::activeWaypoint() const {
  const auto& points =
      progress_.phase() == task_manager::MissionPhase::ENTERING
          ? entry_waypoints_
          : patrol_waypoints_;
  return points.at(progress_.index());
}
```

在运行循环中让 `STATE_ENTERING` 和 `STATE_PATROLLING` 共用现有目标发布逻辑；到达时只调用 `progress_.advance()` 并同步 `state_ = activeState()`。删除原 `current_waypoint_index_` 和“整组 waypoints 从零重启”逻辑。

`PAUSE` 同时接受 `STATE_ENTERING` 与 `STATE_PATROLLING`，先调用 `progress_.pause()` 再进入 `STATE_PAUSED`。返航到中断点后调用 `progress_.resume()`，再用 `activeState()` 恢复。`RESUME` 从 `STATE_IDLE` 首次启动时也使用 `activeState()`。

- [ ] **Step 5: 运行进度和任务定义测试并提交**

运行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg task_manager
catkin_make run_tests_task_manager
catkin_test_results build/task_manager
```

预期：任务包全部测试通过，0 errors、0 failures。

提交：

```bash
git add src/mix_nav/task_manager/include/task_manager/mission_progress.h \
  src/mix_nav/task_manager/src/mission_progress.cpp \
  src/mix_nav/task_manager/test/mission_progress_test.cpp \
  src/mix_nav/task_manager/include/task_manager/mission_manager.h \
  src/mix_nav/task_manager/src/mission_manager.cpp \
  src/mix_nav/task_manager/CMakeLists.txt
git commit -m "feat: separate mission entry from patrol loops"
```

---

### Task 3: 用几何测试驱动六机安全路线

**Files:**
- Modify: `src/mix_nav/task_manager/test/test_mission_clearance.py`
- Modify: `src/mix_nav/task_manager/launch/mission_down.json`
- Replace with symlink: `waypoint/mission_down.json`

- [ ] **Step 1: 为完整任务图增加失败测试**

在测试文件加入固定起点和起飞区：

```python
INITIAL_POSITIONS = {
    "typhoon_h480_0": (-17.0, -3.0, 3.0),
    "typhoon_h480_1": (-14.0, -3.0, 3.0),
    "typhoon_h480_2": (-17.0, 0.0, 3.0),
    "typhoon_h480_3": (-14.0, 0.0, 3.0),
    "typhoon_h480_4": (-17.0, 3.0, 3.0),
    "typhoon_h480_5": (-14.0, 3.0, 3.0),
}
LAUNCH_BOX = (-25.0, 0.0, -12.0, 12.0)
INTER_VEHICLE_CLEARANCE = 5.0
PATROL_ALTITUDE = 3.5
```

增加结构化帮助函数：

```python
def mission_segments(mission):
    vehicle_id = mission["vehicle_id"]
    sx, sy, sz = INITIAL_POSITIONS[vehicle_id]
    start = {"x": sx, "y": sy, "z": sz}
    entry = mission.get("entry_waypoints", [])
    patrol = mission["waypoints"]
    route = [start] + entry + patrol[:1]
    segments = list(zip(route, route[1:]))
    segments.extend(zip(patrol, patrol[1:]))
    segments.append((patrol[-1], patrol[0]))
    return segments

def point_to_segment_distance(point, start, end):
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return ((point["x"] - start["x"]) ** 2 +
                (point["y"] - start["y"]) ** 2) ** 0.5
    projection = ((point["x"] - start["x"]) * dx +
                  (point["y"] - start["y"]) * dy) / denominator
    projection = max(0.0, min(1.0, projection))
    nearest_x = start["x"] + projection * dx
    nearest_y = start["y"] + projection * dy
    return ((point["x"] - nearest_x) ** 2 +
            (point["y"] - nearest_y) ** 2) ** 0.5

def segment_distance(first_start, first_end, second_start, second_end):
    if segments_intersect(first_start, first_end, second_start, second_end):
        return 0.0
    return min(
        point_to_segment_distance(first_start, second_start, second_end),
        point_to_segment_distance(first_end, second_start, second_end),
        point_to_segment_distance(second_start, first_start, first_end),
        point_to_segment_distance(second_end, first_start, first_end),
    )
```

`segments_intersect` 使用叉积方向和在线段边界判断，必须把共线重叠判为相交。`clip_segment_outside_box` 使用 Liang-Barsky 参数区间求线段在 `LAUNCH_BOX` 内的部分，并返回 0 至 2 条区外线段；不能按“整条线段只要碰到起飞区就全部豁免”。

新增测试方法：

```python
def test_static_baseline_has_complete_six_vehicle_schema(self):
    missions = load_missions(MISSION_FILE)
    self.assertEqual(set(INITIAL_POSITIONS),
                     {mission["vehicle_id"] for mission in missions})
    for mission in missions:
        self.assertGreaterEqual(len(mission["entry_waypoints"]), 1)
        self.assertGreaterEqual(len(mission["waypoints"]), 3)
        for waypoint in mission["entry_waypoints"] + mission["waypoints"]:
            self.assertEqual(PATROL_ALTITUDE, waypoint["z"])

def test_all_complete_segments_clear_known_static_obstacles(self):
    collisions = collect_obstacle_collisions(load_missions(MISSION_FILE))
    self.assertEqual([], collisions, "\n" + "\n".join(collisions))

def test_different_aircraft_routes_clear_five_metres_outside_launch_box(self):
    violations = collect_inter_vehicle_violations(load_missions(MISSION_FILE))
    self.assertEqual([], violations, "\n" + "\n".join(violations))

def test_visible_waypoint_entry_resolves_to_runtime_mission(self):
    visible = Path(__file__).resolve().parents[4] / "waypoint" / "mission_down.json"
    self.assertTrue(visible.is_symlink())
    self.assertEqual(MISSION_FILE.resolve(), visible.resolve())
```

添加纯几何单元用例，分别断言相交为 0、共线重叠为 0、平行 4.99 米失败、平行恰好 5.0 米通过，以及穿过起飞区后仍保留区外片段。

- [ ] **Step 2: 运行测试，确认当前任务因目标缺陷失败**

运行：

```bash
python3 src/mix_nav/task_manager/test/test_mission_clearance.py -v
```

预期失败必须同时包含：缺少 `entry_waypoints`、1/2/3 号机 `y=0` 共线重叠、现有根目录文件不是符号链接。若失败来自几何函数自身，先修正测试帮助函数，再重新确认目标缺陷。

- [ ] **Step 3: 写入已经静态预演通过的六机路线**

将权威 `mission_down.json` 改为以下坐标，所有点 `z` 均为 `3.5`：

```json
[
  {
    "vehicle_id": "typhoon_h480_0",
    "entry_waypoints": [
      {"x": -25.0, "y": -8.0, "z": 3.5}
    ],
    "waypoints": [
      {"x": -41.0, "y": -8.0, "z": 3.5},
      {"x": -41.0, "y": -41.0, "z": 3.5},
      {"x": -10.0, "y": -41.0, "z": 3.5},
      {"x": -10.0, "y": -8.0, "z": 3.5}
    ]
  },
  {
    "vehicle_id": "typhoon_h480_1",
    "entry_waypoints": [
      {"x": -8.0, "y": 0.0, "z": 3.5},
      {"x": 0.0, "y": 0.0, "z": 3.5}
    ],
    "waypoints": [
      {"x": 10.0, "y": 0.0, "z": 3.5},
      {"x": 45.0, "y": 0.0, "z": 3.5},
      {"x": 45.0, "y": -41.0, "z": 3.5},
      {"x": 10.0, "y": -41.0, "z": 3.5}
    ]
  },
  {
    "vehicle_id": "typhoon_h480_2",
    "entry_waypoints": [
      {"x": -25.0, "y": 0.0, "z": 3.5},
      {"x": -47.0, "y": 0.0, "z": 3.5},
      {"x": -47.0, "y": -47.0, "z": 3.5},
      {"x": 127.0, "y": -47.0, "z": 3.5},
      {"x": 127.0, "y": -41.0, "z": 3.5}
    ],
    "waypoints": [
      {"x": 122.0, "y": -41.0, "z": 3.5},
      {"x": 122.0, "y": 0.0, "z": 3.5},
      {"x": 78.0, "y": 0.0, "z": 3.5},
      {"x": 78.0, "y": -41.0, "z": 3.5}
    ]
  },
  {
    "vehicle_id": "typhoon_h480_3",
    "entry_waypoints": [
      {"x": -20.0, "y": 12.0, "z": 3.5}
    ],
    "waypoints": [
      {"x": -41.0, "y": 12.0, "z": 3.5},
      {"x": -41.0, "y": 41.0, "z": 3.5},
      {"x": -10.0, "y": 41.0, "z": 3.5},
      {"x": -10.0, "y": 12.0, "z": 3.5}
    ]
  },
  {
    "vehicle_id": "typhoon_h480_4",
    "entry_waypoints": [
      {"x": 0.0, "y": 12.0, "z": 3.5}
    ],
    "waypoints": [
      {"x": 10.0, "y": 12.0, "z": 3.5},
      {"x": 68.0, "y": 12.0, "z": 3.5},
      {"x": 68.0, "y": 41.0, "z": 3.5},
      {"x": 10.0, "y": 41.0, "z": 3.5}
    ]
  },
  {
    "vehicle_id": "typhoon_h480_5",
    "entry_waypoints": [
      {"x": -20.0, "y": 7.0, "z": 3.5},
      {"x": -29.0, "y": 7.0, "z": 3.5},
      {"x": -47.0, "y": 7.0, "z": 3.5},
      {"x": -47.0, "y": 47.0, "z": 3.5},
      {"x": 127.0, "y": 47.0, "z": 3.5},
      {"x": 127.0, "y": 41.0, "z": 3.5}
    ],
    "waypoints": [
      {"x": 122.0, "y": 41.0, "z": 3.5},
      {"x": 122.0, "y": 12.0, "z": 3.5},
      {"x": 78.0, "y": 12.0, "z": 3.5},
      {"x": 78.0, "y": 41.0, "z": 3.5}
    ]
  }
]
```

这组坐标的静态预演结果为：已登记障碍碰撞数 0；起飞区外跨机线段最小中心线距离 5.0 米。实际实现仍必须由仓库测试重新计算，不能直接相信计划文本。

- [ ] **Step 4: 消除第二份任务文件**

将根目录文件替换成相对符号链接：

```bash
git rm waypoint/mission_down.json
ln -s ../src/mix_nav/task_manager/launch/mission_down.json waypoint/mission_down.json
git add waypoint/mission_down.json
```

运行 `readlink waypoint/mission_down.json`，预期输出：

```text
../src/mix_nav/task_manager/launch/mission_down.json
```

- [ ] **Step 5: 运行几何测试确认转绿并提交**

运行：

```bash
python3 src/mix_nav/task_manager/test/test_mission_clearance.py -v
python3 -m json.tool src/mix_nav/task_manager/launch/mission_down.json >/dev/null
git diff --check
```

预期：全部任务几何测试通过；JSON 解析成功；无空白错误。

提交：

```bash
git add src/mix_nav/task_manager/test/test_mission_clearance.py \
  src/mix_nav/task_manager/launch/mission_down.json waypoint/mission_down.json
git commit -m "feat: partition static patrol into safe zones"
```

---

### Task 4: 将最终高度安全边界收紧到 4 米

**Files:**
- Modify: `src/ego_fusion_search/safety_filter/config/default.yaml`
- Modify: `src/ego_fusion_search/safety_filter/include/safety_filter/safety_policy.h`
- Modify: `src/ego_fusion_search/safety_filter/src/safety_filter_node.cpp`
- Modify: `src/ego_fusion_search/safety_filter/test/safety_policy_test.cpp`
- Modify: `src/ego_fusion_search/safety_filter/test/test_safety_filter_node.py`
- Modify: `src/ego_fusion_search/safety_filter/test/safety_filter_node.test`

- [ ] **Step 1: 写默认 4 米和节点运行态的失败测试**

在 `safety_policy_test.cpp` 增加：

```cpp
TEST(SafetyPolicy, DefaultMaximumAltitudeIsFourMetres) {
  EXPECT_DOUBLE_EQ(4.0, Limits{}.max_altitude);
}
```

让 `safety_filter_node.test` 在节点内加载真实默认配置：

```xml
<rosparam command="load"
          file="$(find safety_filter)/config/default.yaml"/>
```

扩展 Python 测试的发布帮助函数，使其接受 `altitude` 和 `vertical_speed`，增加：

```python
def test_default_four_metre_ceiling_blocks_climb(self):
    self._publish_until_command(
        altitude=4.0,
        vertical_speed=0.5,
        predicate=lambda: self.status == "ALTITUDE_LIMIT"
        and self.command is not None
        and self.command.linear.z == 0.0,
    )
```

- [ ] **Step 2: 运行安全过滤测试并确认旧默认值导致失败**

运行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg safety_filter
catkin_make run_tests_safety_filter
catkin_test_results build/safety_filter
```

预期：默认上限断言或 4 米节点测试失败，现有 5.5 米逻辑仍允许向上速度。

- [ ] **Step 3: 将三个默认来源统一改为 4.0**

修改：

```yaml
max_altitude: 4.0
```

```cpp
double max_altitude{4.0};
```

```cpp
private_nh_.param("max_altitude", limits.max_altitude, 4.0);
```

不要改变 `min_altitude`、速度、加速度、超时和故障码。

- [ ] **Step 4: 运行安全过滤测试确认通过并提交**

运行 Task 4 Step 2 的三个命令，预期 0 errors、0 failures。

提交：

```bash
git add src/ego_fusion_search/safety_filter/config/default.yaml \
  src/ego_fusion_search/safety_filter/include/safety_filter/safety_policy.h \
  src/ego_fusion_search/safety_filter/src/safety_filter_node.cpp \
  src/ego_fusion_search/safety_filter/test/safety_policy_test.cpp \
  src/ego_fusion_search/safety_filter/test/test_safety_filter_node.py \
  src/ego_fusion_search/safety_filter/test/safety_filter_node.test
git commit -m "fix: cap static patrol altitude at four metres"
```

---

### Task 5: 更新项目说明并完成自动化回归

**Files:**
- Modify: `docs/TROUBLESHOOTING.md`
- Modify: `docs/AI_AGENT_HANDOFF.md`
- Verify: all files changed by Tasks 1-4

- [ ] **Step 1: 更新故障排查和交接手册**

文档必须明确写出：

- 起飞仍为 3.0 米，固定巡逻为 3.5 米，安全过滤上限为 4.0 米；
- `entry_waypoints` 只执行一次，`waypoints` 隐式闭环；
- 运行时权威任务是 `src/mix_nav/task_manager/launch/mission_down.json`；
- `waypoint/mission_down.json` 只是指向权威文件的可见链接；
- 六个责任区和 5 米中心线净空只保证固定任务，跟踪和未来 EGO 仍需动态避碰；
- PX4、XTDrone、Gazebo、EGO 和官方模型保持原版。

- [ ] **Step 2: 运行聚焦测试**

```bash
python3 src/mix_nav/task_manager/test/test_mission_clearance.py -v
source /opt/ros/noetic/setup.bash
catkin_make --pkg task_manager safety_filter
catkin_make run_tests_task_manager run_tests_safety_filter
catkin_test_results build/task_manager build/safety_filter
```

预期：所有聚焦测试通过，Catkin 0 errors、0 failures。

- [ ] **Step 3: 运行全仓 Python 和静态检查**

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
bash -n 1.sh scripts/*.sh src/yolo/*.sh
python3 -m py_compile scripts/process_supervisor.py
git diff --check
```

预期：Python 全部通过，Shell/Python 语法无错误，Git 无空白错误。

- [ ] **Step 4: 运行完整 competition-clean verifier**

显式设置外部只读目录后运行：

```bash
export PX4_DIR=/home/wangtao/robocup_fly/PX4_Firmware
export XTDRONE_DIR=/home/wangtao/robocup_fly/XTDrone
export GAZEBO_MODELS_DIR=/home/wangtao/robocup_fly/gazebo_models
export XTDRONE_PYTHONPATH=/home/wangtao/robocup_fly/.xtdrone-python
export YOLO_PYTHON=/home/wangtao/robocup_fly/.venv-yolo/bin/python
export YOLO_CONFIG_DIR=/home/wangtao/robocup_fly/.ultralytics
bash scripts/verify_competition_clean.sh
```

预期结尾：

```text
完整验证通过：静态与构建后合规证据均已生成。
```

若 Codex 隔离阻止网络接口或图形相关系统调用，使用同一命令在本机执行环境重跑，并分别记录隔离失败证据和本机结果；不得把隔离限制写成项目通过。

- [ ] **Step 5: 提交文档和自动化验证状态**

在文档中先记录自动化结果，不提前写真实六机通过。

```bash
git add docs/TROUBLESHOOTING.md docs/AI_AGENT_HANDOFF.md
git commit -m "docs: explain static patrol safety baseline"
```

---

### Task 6: 真实六机全航程验收

**Files:**
- Runtime evidence: `logs/competition-clean/`（不提交）
- Runtime evidence: `competition-artifacts/`（按仓库既有规则处理）
- Modify after evidence: `docs/AI_AGENT_HANDOFF.md`

- [ ] **Step 1: 启动前核对只读输入和残留**

```bash
git -C /home/wangtao/robocup_fly/XTDrone status --short
pgrep -af 'px4|gzserver|gzclient|multirotor_communication.py|yolo11n.py|bbox2coord_node.py'
find /tmp -maxdepth 1 -type d -name 'robocup-fly-competition-clean.*' -print
```

预期：XTDrone 工作树为空；没有属于本项目上一轮的进程；记录已有临时目录，不能误删来源不明的目录。

- [ ] **Step 2: 启动六机固定任务**

在真实图形桌面终端运行：

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
bash 1.sh 6 mission_down.json
```

等待六机 MAVROS、通信、相机、YOLO、任务节点和起飞门控全部就绪。不得跳过相机或减少飞机数量。

- [ ] **Step 3: 运行 smoke 并记录任务阶段**

在第二个终端运行：

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
bash scripts/smoke_competition_clean.sh
```

预期最后两项包括：

```text
PASS takeoff gate /swarm/takeoff_complete
PASS competition-clean six-vehicle smoke
```

同时从任务日志确认每架先报告进入航点，随后只循环本区域巡逻航点；任何飞机完成进入后再次回到进入航点均视为失败。

在独立终端启动位姿和接触证据采集：

```bash
rosbag record -O /tmp/static-patrol-validation.bag \
  /typhoon_h480_0/global_pose /typhoon_h480_1/global_pose \
  /typhoon_h480_2/global_pose /typhoon_h480_3/global_pose \
  /typhoon_h480_4/global_pose /typhoon_h480_5/global_pose \
  /swarm/takeoff_complete
```

另开终端运行：

```bash
gz topic -e -t /gazebo/default/physics/contacts \
  > /tmp/static-patrol-contacts.log
```

两项采集都持续到六机完成进入和至少一轮闭环；结束采集时只停止对应终端，不停止主仿真。

- [ ] **Step 4: 观察完整进入和至少一轮区域循环**

验收记录必须包含：

- 六机各自最高 `global_pose.z`；
- 固定任务阶段的最小机间实际距离；
- 六机各自进入阶段完成时间；
- 每架至少完成一轮四航点闭环；
- Gazebo 中是否存在无人机与无人机、无人机与静态建筑的接触；
- 是否出现 `ALTITUDE_LIMIT`，若出现则记录飞机编号、时间和此前垂直命令；
- 进入阶段和巡逻阶段各至少一次人物跟踪打断；返回后分别继续剩余进入航点或原巡逻索引。

停止 rosbag 后，用下面的只读分析计算最高高度和异步位姿流中的最小机间距离：

```bash
python3 - /tmp/static-patrol-validation.bag <<'PY'
import itertools
import math
import sys

import rosbag

bag_path = sys.argv[1]
vehicle_ids = [f"typhoon_h480_{index}" for index in range(6)]
topic_to_vehicle = {
    f"/{vehicle_id}/global_pose": vehicle_id for vehicle_id in vehicle_ids
}
latest = {}
maximum_altitude = {vehicle_id: float("-inf") for vehicle_id in vehicle_ids}
task_maximum_altitude = {
    vehicle_id: float("-inf") for vehicle_id in vehicle_ids
}
minimum_distance = float("inf")
minimum_pair = None
minimum_time = None
task_minimum_distance = float("inf")
task_minimum_pair = None
task_minimum_time = None
takeoff_complete = False

with rosbag.Bag(bag_path) as bag:
    topics = list(topic_to_vehicle) + ["/swarm/takeoff_complete"]
    for topic, message, stamp in bag.read_messages(topics=topics):
        if topic == "/swarm/takeoff_complete":
            takeoff_complete = takeoff_complete or bool(message.data)
            continue
        vehicle_id = topic_to_vehicle[topic]
        position = message.pose.position
        latest[vehicle_id] = (position.x, position.y, position.z)
        maximum_altitude[vehicle_id] = max(
            maximum_altitude[vehicle_id], position.z
        )
        if takeoff_complete:
            task_maximum_altitude[vehicle_id] = max(
                task_maximum_altitude[vehicle_id], position.z
            )
        if len(latest) != len(vehicle_ids):
            continue
        for first, second in itertools.combinations(vehicle_ids, 2):
            distance = math.dist(latest[first], latest[second])
            if distance < minimum_distance:
                minimum_distance = distance
                minimum_pair = (first, second)
                minimum_time = stamp.to_sec()
            if takeoff_complete and distance < task_minimum_distance:
                task_minimum_distance = distance
                task_minimum_pair = (first, second)
                task_minimum_time = stamp.to_sec()

for vehicle_id in vehicle_ids:
    print(f"MAX_ALTITUDE {vehicle_id} {maximum_altitude[vehicle_id]:.3f}")
    print(
        f"TASK_MAX_ALTITUDE {vehicle_id} "
        f"{task_maximum_altitude[vehicle_id]:.3f}"
    )
print(
    f"MIN_DISTANCE {minimum_distance:.3f} "
    f"PAIR {minimum_pair[0]} {minimum_pair[1]} TIME {minimum_time:.3f}"
)
print(
    f"TASK_MIN_DISTANCE {task_minimum_distance:.3f} "
    f"PAIR {task_minimum_pair[0]} {task_minimum_pair[1]} "
    f"TIME {task_minimum_time:.3f}"
)
PY
```

保存输出到交接记录。由于六机出生中心本来只有约 3 米，必须同时报告全程值和 `/swarm/takeoff_complete` 首次为真之后的任务阶段值，不得删掉不利样本。若 `TASK_` 值仍为无穷或目标对为空，说明 bag 没有录到打开的门控，本次采集无效，必须重新运行。

检查接触流：

```bash
rg -n -C 4 'typhoon_h480|gas_station|fast_food|house_|lamp_post' \
  /tmp/static-patrol-contacts.log
```

预期：没有任何包含 `typhoon_h480_N` 的无人机互撞或静态建筑接触对。人物接触或其他 Gazebo 内部接触不能混写成无人机碰撞，需要核对双方 collision 完整名称。

任何实际机间接触、建筑接触、持续超过 4.0 米、跨区巡逻或错误重跑进入路线均视为失败。先保存最早异常证据，再回到对应队伍文件排查；不得修改官方组件来使验收通过。

- [ ] **Step 5: 正常停止并检查清理**

回到启动终端按一次 `Ctrl-C`，预期外层退出码 130。随后运行：

```bash
pgrep -af 'px4|gzserver|gzclient|multirotor_communication.py|yolo11n.py|bbox2coord_node.py|confident_takeoff_node'
find /tmp -maxdepth 1 -type d -name 'robocup-fly-competition-clean.*' -print
git -C /home/wangtao/robocup_fly/XTDrone status --short
```

预期：本次项目进程无残留；临时目录没有新增残留；XTDrone 工作树仍为空。

- [ ] **Step 6: 写入真实证据并提交**

在 `docs/AI_AGENT_HANDOFF.md` 追加本次日期、提交 SHA、完整 verifier 结果、smoke 日志路径、最高高度、实际最小机间距离、闭环完成情况、接触结果、Ctrl-C 退出码和只读输入状态。未通过或未执行的项目必须原样写明，不能省略。

```bash
git add docs/AI_AGENT_HANDOFF.md
git commit -m "docs: record static patrol flight validation"
git status --short --branch
```

预期：除 `.superpowers/` 可视化临时目录外，任务文件均已提交；不包含日志、构建产物、虚拟环境、PX4、XTDrone 或官方模型。只有用户再次明确要求时才推送 `competition-clean`。
