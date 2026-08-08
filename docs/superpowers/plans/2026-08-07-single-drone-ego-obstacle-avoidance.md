# 0号机 EGO 障碍绕行闭环 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `typhoon_h480_0` 在未知地图中使用现有深度与位姿、EGO-Planner-Swarm 和队伍安全适配器，自主绕过静态障碍到达短距离目标，并在感知、规划、控制权或 tracking 状态异常时安全悬停。

**Architecture:** 外部 EGO 固定为仓库外只读依赖，只通过队伍 Launch remap 接入。`local_mapping` 保持唯一体素地图事实来源并新增扫掠体验证服务；队伍自有 `search_coordinator` 选择最多8米的已知空闲局部目标，`ego_adapter` 校验 EGO 轨迹与命令后复用既有 navigator MUX 输入，最终仍由 `safety_filter` 独占 XTDrone 控制话题。

**Tech Stack:** Ubuntu 20.04、ROS 1 Noetic、Catkin、C++14、Python 3、GoogleTest、rostest、EGO-Planner-Swarm、Eigen、Gazebo 11

---

## 开始条件和不可修改边界

执行前先完成 `2026-08-07-static-route-collision-containment.md`。外部 EGO 当前尚未下载、构建或运行验证；计划中的提交号只是上游审计基线，Task 1 必须重新联网核验。

禁止修改或提交：

- `/home/wangtao/robocup_fly/PX4_Firmware`
- `/home/wangtao/robocup_fly/XTDrone`
- Gazebo 系统包和官方 World
- EGO-Planner-Swarm 核心源码
- 官方无人机模型和传感器模型
- `src/gazebo_ros_actor_plugin`、`src/darknet_ros_msgs`
- `.superpowers/`、日志、bag、构建产物

只允许新增或修改队伍自有 ROS 包、Launch、配置、测试和文档。任何 EGO 接口差异都在 `ego_adapter` 或 Launch remap 中处理。

开始执行时先运行：

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
source /opt/ros/noetic/setup.bash
test -f devel/setup.bash && source devel/setup.bash || true
EGO_PLAN_BASE_SHA="$(git rev-parse HEAD)"
```

Task 1 构建外部 EGO 后，每个构建或运行 shell 都必须按以下顺序 source，随后回到比赛仓库根目录：

```bash
source /opt/ros/noetic/setup.bash
source /home/wangtao/robocup_fly/external/ego-planner-swarm/devel/setup.bash
source /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean/devel/setup.bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
```

## 文件结构

新增：

- `scripts/check_ego_external.py`：核验外部 EGO 路径、提交、工作树和消息接口。
- `src/ego_fusion_search/search_msgs/srv/ValidateTrajectory.srv`：统一扫掠体验证服务。
- `src/ego_fusion_search/ego_adapter/`：EGO 输出到现有 FLU navigator 输入的失效关闭适配器。
- `src/ego_fusion_search/search_coordinator/`：高层目标、局部目标、观察和任务代次状态机。
- `src/ego_fusion_search/ego_adapter/launch/ego_single.launch`：外部 EGO 与队伍适配器的单机接线。
- `src/ego_fusion_search/search_coordinator/launch/search_single.launch`：0号机协调器接线。
- `src/look_up/launch/navigation_single.launch`：`ego|layered_2d|static_patrol` 启动互斥入口。
- `tests/test_ego_navigation_wiring.py`：静态发布者、模式和边界检查。
- `scripts/record_ego_acceptance.sh`：验收话题记录和结果目录骨架。
- `scripts/ego_acceptance_scenario.py`：运行时临时障碍、故障和 tracking 场景驱动。
- `scripts/analyze_ego_acceptance.py`：contacts、轨迹、时限、RTF 和重规划延迟验收。

修改：

- `src/ego_fusion_search/search_msgs/{CMakeLists.txt,package.xml}`
- `src/ego_fusion_search/local_mapping/{include/local_mapping/voxel_map.h,src/voxel_map.cpp,src/local_mapping_node.cpp,CMakeLists.txt,package.xml}`
- `src/ego_fusion_search/local_mapping/{config/single_drone.yaml,launch/local_mapping_single.launch,test/voxel_map_test.cpp,test/local_mapping_node.test,test/test_local_mapping_node.py}`
- `src/tracking/{include/tracking/state_machine.h,src/state_machine.cpp,test/tracking_phase.test,test/test_tracking_phase.py,CMakeLists.txt}`
- `src/ego_fusion_search/safety_filter/{include,src,config,launch,test}` 中的队伍最终安全策略与测试
- `src/look_up/launch/down_resume.launch`
- `1.sh`
- `src/competition_compliance/config/ownership.json`
- `src/competition_compliance/test/test_ownership.py`
- `docs/THIRD_PARTY.md`、`docs/AI_AGENT_HANDOFF.md`

### Task 1: 固定并核验外部 EGO 依赖

**Files:**
- Create: `scripts/check_ego_external.py`
- Modify: `docs/THIRD_PARTY.md`
- Test: `tests/test_ego_navigation_wiring.py`

- [ ] **Step 1: 写外部依赖检查的失败测试**

在 `tests/test_ego_navigation_wiring.py` 创建以下测试骨架：

```python
#!/usr/bin/env python3
import pathlib
import subprocess
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
CHECK = ROOT / "scripts/check_ego_external.py"


class EgoExternalContractTest(unittest.TestCase):
    def test_checker_exists_and_has_pinned_revision(self):
        self.assertTrue(CHECK.is_file())
        text = CHECK.read_text(encoding="utf-8")
        self.assertIn("92fe9f7227b2da819133eb8e0e8c7fc000f6ae20", text)
        self.assertIn("src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg", text)
        self.assertIn("src/planner/traj_utils/msg/Bspline.msg", text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行测试确认失败**

Run:

```bash
python3 -m unittest tests.test_ego_navigation_wiring.EgoExternalContractTest -v
```

Expected: `FAIL`，提示 `scripts/check_ego_external.py` 不存在。

- [ ] **Step 3: 下载固定版本到仓库外目录**

Run:

```bash
mkdir -p /home/wangtao/robocup_fly/external
git clone https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git \
  /home/wangtao/robocup_fly/external/ego-planner-swarm
git -C /home/wangtao/robocup_fly/external/ego-planner-swarm \
  checkout --detach 92fe9f7227b2da819133eb8e0e8c7fc000f6ae20
```

Expected: `HEAD is now at 92fe9f7...`。若下载失败，只重试下载；不得以另一个提交静默替代，也不得在比赛仓库内复制源码。

- [ ] **Step 4: 创建严格只读检查脚本**

`scripts/check_ego_external.py` 使用以下完整行为：

```python
#!/usr/bin/env python3
import argparse
import pathlib
import subprocess
import sys

PIN = "92fe9f7227b2da819133eb8e0e8c7fc000f6ae20"
REQUIRED = (
    "src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg",
    "src/planner/traj_utils/msg/Bspline.msg",
    "src/planner/plan_manage/launch/single_run_in_sim.launch",
    "src/planner/plan_manage/launch/advanced_param.xml",
)

REQUIRED_FIELDS = {
    "src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg": (
        "Header header",
        "geometry_msgs/Point position",
        "geometry_msgs/Vector3 velocity",
        "geometry_msgs/Vector3 acceleration",
        "float64 yaw",
        "float64 yaw_dot",
    ),
    "src/planner/traj_utils/msg/Bspline.msg": (
        "int32 order",
        "time start_time",
        "int64 traj_id",
        "geometry_msgs/Point[] pos_pts",
        "float64[] knots",
    ),
    "src/planner/plan_manage/launch/advanced_param.xml": (
        "grid_map/fx",
        "grid_map/fy",
        "grid_map/cx",
        "grid_map/cy",
    ),
}


def git(repo, *args):
    return subprocess.check_output(
        ["git", "-C", str(repo), *args], text=True
    ).strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ego-dir",
        default="/home/wangtao/robocup_fly/external/ego-planner-swarm",
    )
    args = parser.parse_args()
    root = pathlib.Path(args.ego_dir).resolve()
    failures = []
    if not (root / ".git").is_dir():
        failures.append(f"not a git checkout: {root}")
    else:
        if git(root, "rev-parse", "HEAD") != PIN:
            failures.append("EGO revision differs from pinned revision")
        if git(root, "status", "--porcelain"):
            failures.append("EGO working tree is modified")
    for relative in REQUIRED:
        if not (root / relative).is_file():
            failures.append(f"missing interface: {relative}")
    for relative, fields in REQUIRED_FIELDS.items():
        path = root / relative
        if not path.is_file():
            continue
        source = path.read_text(encoding="utf-8")
        for field in fields:
            if field not in source:
                failures.append(f"missing field in {relative}: {field}")
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    print(f"EGO_EXTERNAL_OK {PIN}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 5: 核对上游消息和 Launch 的实际字段**

Run:

```bash
python3 scripts/check_ego_external.py
sed -n '1,200p' /home/wangtao/robocup_fly/external/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg
sed -n '1,200p' /home/wangtao/robocup_fly/external/ego-planner-swarm/src/planner/traj_utils/msg/Bspline.msg
rg -n "odom_world|grid_map/depth|grid_map/pose|move_base_simple/goal|position_cmd|broadcast_bspline" \
  /home/wangtao/robocup_fly/external/ego-planner-swarm/src
```

Expected: 第一条输出 `EGO_EXTERNAL_OK 92fe9f...`；其余输出逐项确认上述消息字段和六个 EGO 话题。固定提交与契约不一致时 Task 1 失败并停止执行，不能修改 EGO、猜字段或换提交继续。

- [ ] **Step 6: 在外部工作空间构建上游依赖**

Run:

```bash
cd /home/wangtao/robocup_fly/external/ego-planner-swarm
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
rospack find ego_planner
rosmsg show quadrotor_msgs/PositionCommand
rosmsg show traj_utils/Bspline
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
```

Expected: 构建成功，三个查询均退出码为0。若上游原版在当前 Noetic 环境不能构建，保留完整日志并停止，不允许打补丁到上游源码。

- [ ] **Step 7: 记录来源但不把外部包加入 ownership**

在 `docs/THIRD_PARTY.md` 增加一行：

```markdown
| `EGO-Planner-Swarm`（外部运行依赖，不进入本仓库） | `ZJU-FAST-Lab/ego-planner-swarm`，提交 `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20` | GPL-3.0；`scripts/check_ego_external.py` 核验提交、干净工作树和接口文件 |
```

不要在 `ownership.json` 增加 EGO，因为它不在比赛仓库内。

- [ ] **Step 8: 运行测试并提交**

```bash
python3 -m unittest tests.test_ego_navigation_wiring.EgoExternalContractTest -v
git add scripts/check_ego_external.py tests/test_ego_navigation_wiring.py docs/THIRD_PARTY.md
git commit -m "build: pin external EGO planner contract"
```

Expected: 测试通过，提交不包含外部 EGO 文件。

### Task 2: 定义轨迹验证服务

**Files:**
- Create: `src/ego_fusion_search/search_msgs/srv/ValidateTrajectory.srv`
- Modify: `src/ego_fusion_search/search_msgs/CMakeLists.txt`
- Modify: `src/ego_fusion_search/search_msgs/package.xml`

- [ ] **Step 1: 先写服务生成契约测试**

在 `tests/test_ego_navigation_wiring.py` 加入：

```python
SEARCH_MSGS = ROOT / "src/ego_fusion_search/search_msgs"


class ValidateTrajectoryContractTest(unittest.TestCase):
    def test_service_contract_is_generated(self):
        service = (SEARCH_MSGS / "srv/ValidateTrajectory.srv").read_text()
        self.assertIn("uint64 task_generation", service)
        self.assertIn("geometry_msgs/Point[] samples", service)
        self.assertIn("time map_stamp", service)
        cmake = (SEARCH_MSGS / "CMakeLists.txt").read_text()
        self.assertIn("add_service_files", cmake)
        self.assertIn("ValidateTrajectory.srv", cmake)
        self.assertIn("geometry_msgs", cmake)
```

- [ ] **Step 2: 运行测试确认失败**

Run: `python3 -m unittest tests.test_ego_navigation_wiring.ValidateTrajectoryContractTest -v`

Expected: `ERROR`，服务文件不存在。

- [ ] **Step 3: 创建精确服务定义**

`ValidateTrajectory.srv`：

```text
std_msgs/Header header
uint64 task_generation
geometry_msgs/Point[] samples
---
bool valid
uint64 task_generation
time map_stamp
float32 min_clearance_m
string fault_code
```

- [ ] **Step 4: 登记消息依赖**

在 `search_msgs/CMakeLists.txt` 中把 `geometry_msgs` 加入 `find_package` 和 `generate_messages(DEPENDENCIES ...)`，并加入：

```cmake
add_service_files(
  FILES
  ValidateTrajectory.srv
)
```

在 `package.xml` 加入：

```xml
<depend>geometry_msgs</depend>
```

- [ ] **Step 5: 构建并提交**

```bash
python3 -m unittest tests.test_ego_navigation_wiring.ValidateTrajectoryContractTest -v
catkin_make --pkg search_msgs
source devel/setup.bash
rossrv show search_msgs/ValidateTrajectory
git add src/ego_fusion_search/search_msgs tests/test_ego_navigation_wiring.py
git commit -m "feat: define trajectory validation service"
```

Expected: 测试通过，`rossrv` 输出与上述字段完全一致。

### Task 3: 在统一 VoxelMap 中实现扫掠体验证

**Files:**
- Modify: `src/ego_fusion_search/local_mapping/include/local_mapping/voxel_map.h`
- Modify: `src/ego_fusion_search/local_mapping/src/voxel_map.cpp`
- Modify: `src/ego_fusion_search/local_mapping/test/voxel_map_test.cpp`

- [ ] **Step 1: 写失败的扫掠体单元测试**

在 `voxel_map_test.cpp` 加入已知空闲、unknown、occupied 和动态点四类测试，核心断言如下：

```cpp
TEST(VoxelMapTest, SweptVolumeRequiresEveryVoxelToBeKnownAndFree) {
  VoxelMap map(0.2, 1, 1, 1.0);
  for (double y = -0.6; y <= 0.6; y += 0.2) {
    for (double z = 2.6; z <= 3.4; z += 0.2) {
      map.integrateStaticRay({0.0, y, z}, {3.0, y, z});
    }
  }
  const std::vector<Vec3> samples{{0.5, 0.0, 3.0}, {2.5, 0.0, 3.0}};
  const auto result = map.validateSweptVolume(samples, 0.35, 0.25, 0.10, 0.0);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(SweepFault::NONE, result.fault);
  EXPECT_GT(result.min_clearance_m, 0.0);
}

TEST(VoxelMapTest, SweptVolumeRejectsUnknownAndOccupiedCells) {
  VoxelMap unknown(0.2, 1, 1, 1.0);
  auto unknown_result = unknown.validateSweptVolume(
      {{0.0, 0.0, 3.0}}, 0.35, 0.25, 0.10, 0.0);
  EXPECT_FALSE(unknown_result.valid);
  EXPECT_EQ(SweepFault::UNKNOWN, unknown_result.fault);

  VoxelMap occupied(0.2, 1, 1, 1.0);
  occupied.integrateDynamicPoint({1.0, 0.0, 3.0}, 5.0);
  auto occupied_result = occupied.validateSweptVolume(
      {{1.0, 0.0, 3.0}}, 0.35, 0.25, 0.10, 5.1);
  EXPECT_FALSE(occupied_result.valid);
  EXPECT_EQ(SweepFault::OCCUPIED, occupied_result.fault);
}
```

- [ ] **Step 2: 运行测试确认编译失败**

Run:

```bash
catkin_make run_tests_local_mapping_gtest_voxel_map_test
```

Expected: 编译失败，提示 `validateSweptVolume`、`SweepFault` 或 `SweepResult` 未定义。

- [ ] **Step 3: 增加公开类型和方法**

在 `voxel_map.h` 中 `Clearance` 后增加：

```cpp
enum class SweepFault { NONE, EMPTY, NON_FINITE, UNKNOWN, OCCUPIED };

struct SweepResult {
  bool valid;
  SweepFault fault;
  double min_clearance_m;
};
```

在 `VoxelMap` 公共方法中增加：

```cpp
SweepResult validateSweptVolume(const std::vector<Vec3>& samples,
                                double horizontal_radius,
                                double vertical_radius,
                                double safety_margin,
                                double now) const;
```

- [ ] **Step 4: 实现保守体素覆盖**

实现必须：拒绝空数组、非有限输入、负半径或负安全余量；按相邻轨迹点之间不大于 `resolution_/2` 的距离插值。对每个中心点先构造膨胀圆柱的世界坐标 AABB，再枚举 AABB 覆盖的所有 voxel key；用“体素 AABB 到圆柱中心轴的最近点距离”判断真实相交，不能只检查按 resolution 偏移得到的探针中心，否则中心靠近体素边界时会漏掉相邻体素。

关键判断复用 `stateForKey`，不创建第二张地图：

```cpp
const double radius = horizontal_radius + safety_margin;
const double half_height = vertical_radius + safety_margin;
for (const Vec3& centre : interpolated_samples) {
  const Key lower = keyFor(
      {centre.x - radius, centre.y - radius, centre.z - half_height});
  const Key upper = keyFor(
      {centre.x + radius, centre.y + radius, centre.z + half_height});
  for (std::int64_t x = lower.x; x <= upper.x; ++x) {
    for (std::int64_t y = lower.y; y <= upper.y; ++y) {
      for (std::int64_t z = lower.z; z <= upper.z; ++z) {
        const Key key{x, y, z};
        const Vec3 voxel_centre = centreFor(key);
        const double half_voxel = resolution_ / 2.0;
        const double dx = std::max(0.0,
            std::abs(voxel_centre.x - centre.x) - half_voxel);
        const double dy = std::max(0.0,
            std::abs(voxel_centre.y - centre.y) - half_voxel);
        const double dz = std::max(0.0,
            std::abs(voxel_centre.z - centre.z) - half_voxel);
        if (std::hypot(dx, dy) > radius || dz > half_height) {
          continue;
        }
        const CellState state = stateForKey(key, now);
        if (state != CellState::FREE) {
          return {false,
                  state == CellState::UNKNOWN ? SweepFault::UNKNOWN
                                              : SweepFault::OCCUPIED,
                  0.0};
        }
      }
    }
  }
}
return {true, SweepFault::NONE, safety_margin};
```

通过时 `min_clearance_m=safety_margin` 表示由膨胀圆柱证明的保守下界，不冒充最近障碍的精确距离；失败时为0。单元测试额外放置一个“仅体素盒角与圆柱相交、体素中心在圆柱外”的 occupied voxel，旧探针算法必须失败、新算法必须拒绝轨迹。

- [ ] **Step 5: 补充输入边界测试并运行**

增加空 samples、NaN、负半径、过大插值计数的异常测试。所有无效参数统一抛 `std::invalid_argument`，不得返回“安全”。

Run:

```bash
catkin_make run_tests_local_mapping_gtest_voxel_map_test
catkin_test_results build/test_results/local_mapping
```

Expected: `0 tests failed`。

- [ ] **Step 6: 提交**

```bash
git add src/ego_fusion_search/local_mapping
git commit -m "feat: validate conservative voxel swept volumes"
```

### Task 4: 由 local_mapping 暴露同图轨迹校验服务

**Files:**
- Modify: `src/ego_fusion_search/local_mapping/src/local_mapping_node.cpp`
- Modify: `src/ego_fusion_search/local_mapping/config/single_drone.yaml`
- Modify: `src/ego_fusion_search/local_mapping/launch/local_mapping_single.launch`
- Modify: `src/ego_fusion_search/local_mapping/test/local_mapping_node.test`
- Modify: `src/ego_fusion_search/local_mapping/test/test_local_mapping_node.py`

- [ ] **Step 1: 写服务级失败测试**

在 test Launch 加入：

```xml
<param name="validate_trajectory_service"
       value="/test_drone_0/local_mapping/validate_trajectory"/>
<param name="task_generation_topic"
       value="/test_drone_0/navigation/task_generation"/>
<param name="vehicle_horizontal_radius" value="0.35"/>
<param name="trajectory_safety_margin" value="0.10"/>
<param name="planner_pose_topic"
       value="/test_drone_0/local_mapping/planner_pose"/>
```

在 Python rostest 中发布 latched `std_msgs/UInt64` generation，并调用 `search_msgs/ValidateTrajectory`，断言：健康地图上的已知空闲 samples 通过；unknown、occupied、空 samples、旧 generation、错误 frame、过期/过度未来 stamp、首点偏离当前 odom、任一点超出 `[2.0,4.0]m`、相邻样本跳距过大和地图不健康分别返回结构化故障码。另断言 `planner_pose` 的 stamp 与输出 `planner_depth` 完全相同，frame=`map`，位姿是 `map_T_base * base_T_camera`，而不是直接复制机体 odom。

- [ ] **Step 2: 运行 rostest 确认失败**

Run: `rostest local_mapping local_mapping_node.test`

Expected: `ERROR`，服务尚不存在。

- [ ] **Step 3: 在节点中登记参数和同图服务**

`NodeConfig` 增加：

```cpp
std::string validate_trajectory_service;
std::string task_generation_topic;
std::string planner_pose_topic;
double vehicle_horizontal_radius;
double trajectory_safety_margin;
double trajectory_max_age;
double trajectory_max_future;
double trajectory_start_tolerance;
double trajectory_max_sample_gap;
double min_search_altitude;
double max_search_altitude;
```

节点增加 `ros::ServiceServer trajectory_server_`、`ros::Subscriber generation_subscriber_`、`std::uint64_t task_generation_{0}`。generation 回调只接受单调递增值。服务回调顺序固定为：

```cpp
response.task_generation = request.task_generation;
response.map_stamp = last_map_stamp_;
if (!map_healthy_) return reject("MAP_UNHEALTHY");
if (request.header.frame_id != "map") return reject("WRONG_FRAME");
if ((now - request.header.stamp).toSec() > config_.trajectory_max_age)
  return reject("STALE_TRAJECTORY");
if ((request.header.stamp - now).toSec() > config_.trajectory_max_future)
  return reject("FUTURE_TRAJECTORY");
if (request.task_generation != task_generation_)
  return reject("STALE_GENERATION");
if (request.samples.size() < 2u) return reject("EMPTY_TRAJECTORY");
if (distance(request.samples.front(), current_odom_position_) >
    config_.trajectory_start_tolerance)
  return reject("START_DEVIATION");
for (const auto& sample : request.samples)
  if (sample.z < config_.min_search_altitude ||
      sample.z > config_.max_search_altitude)
    return reject("HEIGHT_LIMIT");
for (std::size_t i = 1; i < request.samples.size(); ++i)
  if (distance(request.samples[i - 1], request.samples[i]) >
      config_.trajectory_max_sample_gap)
    return reject("SAMPLE_GAP");
const auto result = voxel_map_->validateSweptVolume(
    convert(request.samples), config_.vehicle_horizontal_radius,
    config_.vehicle_vertical_radius, config_.trajectory_safety_margin,
    ros::Time::now().toSec());
```

`SweepFault::UNKNOWN` 映射为 `UNKNOWN`，`OCCUPIED` 映射为 `OCCUPIED`，参数异常映射为 `INVALID_REQUEST`。任何拒绝都设置 `valid=false` 和 `min_clearance_m=0.0`。

- [ ] **Step 4: 配置真实0号机接口**

在 `single_drone.yaml` 加入：

```yaml
vehicle_horizontal_radius: 0.35
trajectory_safety_margin: 0.10
trajectory_max_age: 0.50
trajectory_max_future: 0.20
trajectory_start_tolerance: 0.75
trajectory_max_sample_gap: 0.20
min_search_altitude: 2.00
max_search_altitude: 4.00
validate_trajectory_service: /typhoon_h480_0/local_mapping/validate_trajectory
task_generation_topic: /typhoon_h480_0/navigation/task_generation
planner_pose_topic: /typhoon_h480_0/local_mapping/planner_pose
```

在现有同步深度回调内，使用同一个 depth stamp、已查到的 `base_T_camera` 与同步 odom 计算 `map_T_camera`，发布 `planner_pose`。TF 不可用时不发布 `planner_depth` 或 `planner_pose`，并把 health 置为失败，保证 EGO 永远不会拿到不同步的深度和相机位姿。

- [ ] **Step 5: 运行节点和全部局部地图测试**

```bash
catkin_make --pkg search_msgs local_mapping
rostest local_mapping local_mapping_node.test
catkin_make run_tests_local_mapping
catkin_test_results build/test_results/local_mapping
```

Expected: 所有服务故障码及既有深度、动态人物、健康、净空和前沿测试通过。

- [ ] **Step 6: 提交**

```bash
git add src/ego_fusion_search/local_mapping
git commit -m "feat: expose current-map trajectory validation"
```

### Task 5: 建立 ego_adapter 纯策略核心

**Files:**
- Create: `src/ego_fusion_search/ego_adapter/package.xml`
- Create: `src/ego_fusion_search/ego_adapter/CMakeLists.txt`
- Create: `src/ego_fusion_search/ego_adapter/include/ego_adapter/command_policy.h`
- Create: `src/ego_fusion_search/ego_adapter/src/command_policy.cpp`
- Create: `src/ego_fusion_search/ego_adapter/test/command_policy_test.cpp`

- [ ] **Step 1: 写策略失败测试**

测试覆盖：消息过期、时间倒退、NaN、generation 不匹配、地图不健康、MUX 非 navigator、搜索高度高于4米、yaw 误差大于30度、前方小于制动距离、急停距离、侧飞/倒飞限制。策略输入和输出固定为：

```cpp
struct AxisClearance {
  bool known;
  double metres;
};

struct DirectionalClearance {
  AxisClearance forward;
  AxisClearance backward;
  AxisClearance left;
  AxisClearance right;
  AxisClearance up;
  AxisClearance down;
};

struct PolicyInput {
  double now;
  double command_stamp;
  std::uint64_t bound_generation;
  std::uint64_t active_generation;
  bool map_healthy;
  bool mux_is_navigator;
  bool trajectory_valid;
  Vec3 current_position;
  Vec3 desired_position;
  double current_yaw;
  double desired_yaw;
  double desired_yaw_rate;
  Vec3 world_velocity;
  DirectionalClearance clearance;
};

struct PolicyOutput {
  bool accepted;
  std::string fault_code;
  double forward;
  double left;
  double up;
  double yaw_rate;
};
```

关键断言：

```cpp
EXPECT_EQ("STALE_COMMAND", policy.evaluate(stale).fault_code);
EXPECT_EQ("WRONG_GENERATION", policy.evaluate(old_generation).fault_code);
EXPECT_EQ("HEIGHT_LIMIT", policy.evaluate(above_four).fault_code);
EXPECT_DOUBLE_EQ(0.0, policy.evaluate(yaw_misaligned).forward);
EXPECT_DOUBLE_EQ(0.0, policy.evaluate(emergency_clearance).forward);
```

- [ ] **Step 2: 运行测试确认失败**

Run: `catkin_make run_tests_ego_adapter_gtest_command_policy_test`

Expected: 包或头文件不存在，构建失败。

- [ ] **Step 3: 创建 Catkin 包与策略实现**

依赖固定为 `geometry_msgs nav_msgs roscpp search_msgs std_msgs tf2 topic_tools quadrotor_msgs traj_utils`。策略参数：

```cpp
struct PolicyLimits {
  double command_timeout = 0.20;
  double max_search_altitude = 4.0;
  double position_gain = 0.60;
  double yaw_align_threshold = 0.5235987756;
  double max_forward_speed = 1.5;
  double max_lateral_speed = 0.25;
  double max_reverse_speed = 0.10;
  double max_vertical_speed = 0.50;
  double max_yaw_rate = 0.80;
  double braking_clearance = 1.50;
  double emergency_clearance = 0.80;
};
```

`DirectionalClearance` 包含 forward/backward/left/right/up/down 六组 `known + metres`，字段与 `LocalClearance.msg` 一一对应。期望世界速度使用：

```cpp
corrected_world_velocity = world_velocity +
    limits.position_gain * (desired_position - current_position);
```

再用当前 odom yaw 旋转到机体 FLU 并限幅。所有拒绝路径输出全零。大角度 yaw 只允许 `yaw_rate`；每个机体系分量都按自身方向的 known/clearance 门控，unknown 方向速度必须为0，急停距离内禁止继续接近，制动距离内线性压低。这样侧飞、倒飞和升降不会只依赖 forward 净空。

- [ ] **Step 4: 运行策略测试并提交**

```bash
catkin_make --pkg ego_adapter
catkin_make run_tests_ego_adapter_gtest_command_policy_test
catkin_test_results build/test_results/ego_adapter
git add src/ego_fusion_search/ego_adapter
git commit -m "feat: add fail-closed EGO command policy"
```

Expected: `0 tests failed`。

### Task 6: 接入 PositionCommand、Bspline、地图与 MUX

**Files:**
- Create: `src/ego_fusion_search/ego_adapter/src/ego_adapter_node.cpp`
- Create: `src/ego_fusion_search/ego_adapter/config/single_drone.yaml`
- Create: `src/ego_fusion_search/ego_adapter/test/ego_adapter_node.test`
- Create: `src/ego_fusion_search/ego_adapter/test/test_ego_adapter_node.py`
- Modify: `src/ego_fusion_search/ego_adapter/CMakeLists.txt`

- [ ] **Step 1: 写节点失败关闭 rostest**

测试节点发布 odom、`PerceptionHealth`、`LocalClearance`、generation、MUX selected、EGO `PositionCommand` 和 `Bspline`；提供假的 `ValidateTrajectory` 服务。依次断言：

```text
无健康数据 -> navigator cmd_vel 全零
轨迹服务拒绝 -> 全零且 status=TRAJECTORY_REJECTED
健康且轨迹通过 -> 有限、限幅后的前向命令
旧 generation -> 全零
地图 stamp 更新且重验尚未返回 -> 全零
最新地图加入障碍后重验拒绝 -> 持续全零
tracking 导致 MUX selected=external -> 全零
切回 navigator 但未产生新 generation/新轨迹 -> 仍全零
深度或 odom 超时 -> 0.20秒内归零
```

- [ ] **Step 2: 运行测试确认失败**

Run: `rostest ego_adapter ego_adapter_node.test`

Expected: `ERROR`，可执行文件不存在。

- [ ] **Step 3: 实现轨迹缓存和验证顺序**

节点订阅：

```text
/typhoon_h480_0/ego/position_cmd
/typhoon_h480_0/ego/broadcast_bspline
/typhoon_h480_0/global_odom
/typhoon_h480_0/local_mapping/health
/typhoon_h480_0/local_mapping/clearance
/typhoon_h480_0/navigation/task_generation
/typhoon_h480_0/pose_cmd_mux/selected
```

发布：

```text
/typhoon_h480_0/mux_inputs/navigator/cmd_vel
/typhoon_h480_0/ego_adapter/status
```

Bspline 到达后按上游 `order`、`knots`、`pos_pts` 和 `start_time` 用标准 De Boor 递推求值，从当前时刻到轨迹末端按 `0.10s` 采样，并调用 `ValidateTrajectory`。先用夹持均匀二次 B-spline 的直线控制点写纯 C++ 单元测试，断言起点、中点、终点和越界时间；再把已核对的上游消息字段转换为该纯采样器输入。

任务代次握手不能假设 EGO 消息带队伍 generation。上游 `traj_id` 是有符号 `int64 traj_id`；adapter 内部比较和缓存必须保持有符号64位，禁止转为无符号后比较，以防负值绕回。每次 generation 变化时 adapter 必须：

1. 记录 `generation_started_at_` 和变化前见过的最大上游 `traj_id`（有符号64位）；
2. 清除全部命令、曲线和验证结果并立即输出零；
3. 只接受按有符号64位比较时 `traj_id` 更大且 `start_time >= generation_started_at_ - 0.05s` 的新 Bspline；
4. 验证通过后把 `(active_generation, int64 traj_id)` 绑定为唯一执行对；
5. 只接受 header stamp 晚于 Bspline 接收时刻，且 position/velocity 与绑定曲线在该 stamp 的采样误差分别不超过 `0.30m/0.50m/s` 的 PositionCommand。

这使 tracking 前缓存的旧曲线和仍在发布的旧 PositionCommand 无法伪装成新代次。rostest 必须注入“按有符号64位比较更旧的 traj_id”“新 traj_id 但旧 start_time”“新 stamp 但位置不在绑定曲线上”三类消息并断言全部归零。

轨迹不能只校验一次。adapter 以10Hz重采样“当前 odom 到曲线终点”的剩余段；只要 `health.header.stamp` 晚于上一次 response `map_stamp`、动态人物层 TTL 变化、当前位姿偏离曲线超过0.50米或达到0.10秒周期，就先令 `trajectory_valid_=false` 再调用服务。只有新 response generation 匹配、map_stamp 不旧于请求时观察到的 health stamp 且 valid 才恢复输出；新增障碍导致拒绝时保持零并发布 `TRAJECTORY_REJECTED`。

- [ ] **Step 4: 实现20Hz失效关闭输出**

每次 timer 回调先构造零 `Twist`，依次检查 odom、health、clearance、MUX、Bspline 校验、PositionCommand 新鲜度和 generation。全部通过后才调用 `CommandPolicy`，并始终发布一个结果，禁止沿用上一帧非零命令。

配置固定为：

```yaml
control_rate: 20.0
command_timeout: 0.20
health_timeout: 0.50
odom_timeout: 0.50
map_response_timeout: 0.20
trajectory_revalidate_period: 0.10
trajectory_position_match_tolerance: 0.30
trajectory_velocity_match_tolerance: 0.50
trajectory_deviation_tolerance: 0.50
max_search_altitude: 4.0
yaw_align_threshold: 0.5235987756
max_forward_speed: 1.5
max_lateral_speed: 0.25
max_reverse_speed: 0.10
max_vertical_speed: 0.50
max_yaw_rate: 0.80
braking_clearance: 1.50
emergency_clearance: 0.80
trajectory_sample_dt: 0.10
```

- [ ] **Step 5: 运行节点测试并提交**

```bash
catkin_make --pkg ego_adapter
rostest ego_adapter ego_adapter_node.test
catkin_test_results build/test_results/ego_adapter
git add src/ego_fusion_search/ego_adapter
git commit -m "feat: adapt validated EGO commands to navigator input"
```

Expected: 全部拒绝路径在时限内归零，只有健康且已验证的新任务输出非零。

### Task 7: 建立 search_coordinator 纯状态机

**Files:**
- Create: `src/ego_fusion_search/search_coordinator/package.xml`
- Create: `src/ego_fusion_search/search_coordinator/CMakeLists.txt`
- Create: `src/ego_fusion_search/search_coordinator/include/search_coordinator/coordinator.h`
- Create: `src/ego_fusion_search/search_coordinator/src/coordinator.cpp`
- Create: `src/ego_fusion_search/search_coordinator/test/coordinator_test.cpp`

- [ ] **Step 1: 写状态机失败测试**

状态固定为：

```cpp
enum class State {
  WAIT_READY,
  OBSERVING,
  PLANNING,
  EXECUTING,
  HOLD,
  CANDIDATE_HOLD,
  TRACKING_EXTERNAL,
  REJOINING,
};
```

测试至少断言：未起飞不发目标；高层目标截到最多8米；10Hz重复的等价高层目标不递增 generation；直达验证失败时只选经过二次扫掠体验证的新鲜 frontier；无安全 frontier 时 HOLD；1.0秒没有新 Bspline 状态进入 `HOLD:PLANNING_TIMEOUT`；adapter 报 `TRAJECTORY_REJECTED` 时回到 OBSERVING；到局部目标1.0米内重新评估原高层目标；首次人物候选进入 `CANDIDATE_HOLD`；external 接管递增 generation；回到 navigator 进入 `REJOINING`；只有当前位置生成的新目标才能恢复执行；迟到服务响应和旧 generation 永不复用。

- [ ] **Step 2: 运行测试确认失败**

Run: `catkin_make run_tests_search_coordinator_gtest_coordinator_test`

Expected: 包或目标不存在。

- [ ] **Step 3: 实现局部目标规则**

`CoordinatorInput` 包含 now、ready、mission_active、tracking_candidate、tracking_active、map_healthy、mux_selected、odom、high_level_goal、frontier_goal、adapter_status 和验证结果。`CoordinatorOutput` 包含 state、generation、publish_generation、publish_ego_goal、goal 和 fault_code。配置固定 `goal_position_epsilon=0.20m`、`goal_altitude_epsilon=0.10m`、`planning_timeout=1.0s`、`local_arrival_tolerance=1.0m`；在 epsilon 内的重复高层目标只刷新接收时间，不生成新任务。

局部目标算法：

```cpp
const Vec3 delta = high_level_goal - current_position;
const double horizontal = std::hypot(delta.x, delta.y);
const double scale = horizontal > 8.0 ? 8.0 / horizontal : 1.0;
candidate.x = current_position.x + delta.x * scale;
candidate.y = current_position.y + delta.y * scale;
candidate.z = std::min(4.0, std::max(2.0, high_level_goal.z));
```

先验证从当前位置到 candidate 的扫掠样本。失败时仅把 frame=`map`、距离不超过8米、z 在 `[2.0,4.0]` 且时间新鲜的 `frontier_goal` 当候选；必须再次调用同一个 `ValidateTrajectory` 验证“当前 odom 到 frontier”的完整扫掠体，通过后才能发布。两者都失败则 HOLD 并要求原地观察，不得把前沿点本身空闲误当成整段路径安全，也不得把 unknown 当可通行。

状态转换固定为：发送 goal 后进入 PLANNING；绑定新 Bspline 且 adapter status=`EXECUTING:generation:traj_id` 后进入 EXECUTING；超时、无解或轨迹拒绝进入 HOLD 并递增 generation 取消旧任务；到局部目标后回 OBSERVING；地图更新由 adapter 先停并重验，拒绝则 coordinator 重新选目标。服务回调必须携带发起时的 generation，迟到 response 与当前 generation 不同则丢弃。

- [ ] **Step 4: 运行状态机测试并提交**

```bash
catkin_make --pkg search_coordinator
catkin_make run_tests_search_coordinator_gtest_coordinator_test
catkin_test_results build/test_results/search_coordinator
git add src/ego_fusion_search/search_coordinator
git commit -m "feat: coordinate known-free EGO search goals"
```

### Task 8: 接入任务、tracking 和代次失效

**Files:**
- Create: `src/ego_fusion_search/search_coordinator/src/search_coordinator_node.cpp`
- Create: `src/ego_fusion_search/search_coordinator/config/single_drone.yaml`
- Create: `src/ego_fusion_search/search_coordinator/test/search_coordinator_node.test`
- Create: `src/ego_fusion_search/search_coordinator/test/test_search_coordinator_node.py`
- Modify: `src/ego_fusion_search/search_coordinator/CMakeLists.txt`
- Modify: `src/tracking/include/tracking/state_machine.h`
- Modify: `src/tracking/src/state_machine.cpp`
- Create: `src/tracking/test/tracking_phase.test`
- Create: `src/tracking/test/test_tracking_phase.py`
- Modify: `src/tracking/CMakeLists.txt`

- [ ] **Step 1: 先写 tracking phase 接口的失败 rostest**

`tracking_phase.test` 启动0号 tracking 和既有 fake 服务；`test_tracking_phase.py` 订阅 latched `/typhoon_h480_0/tracking/phase`，依次断言 `WAIT_READY`、`IDLE`、首次锁定目标后的 `DETECTING`、确认后的 `DASH` 或 `TRACKING`、结束时 `RETURNING`，消息格式固定为：

```text
STATE:target_id:ros_time_seconds
```

Run: `rostest tracking tracking_phase.test`

Expected: `FAIL`，新 phase 话题尚不存在。

- [ ] **Step 2: 在 tracking 增加只读状态出口**

保留现有 `yolo_human_tracking_*` 话题，避免破坏既有消费者；另加：

```cpp
tracking_phase_pub_ = nh_.advertise<std_msgs::String>(
    "/" + vehicle_type_ + "_" + vehicle_id_ + "/tracking/phase", 1, true);
```

在每次 `update()` 状态处理后发布 `stateToString(current_state_) + ":" + currently_tracked_target_id_ + ":" + now`。控制门未打开时发布 `WAIT_READY::time`。这个出口只报告已有状态，不改变目标选择、0.25秒确认、15秒连续坐标广播、MUX 或控制算法。

Run: `rostest tracking tracking_phase.test`

Expected: phase 序列断言通过，既有 tracking 测试保持通过。

- [ ] **Step 3: 写接管/恢复 rostest**

测试发布高层 goal、odom、map health、frontier、`mission/active`、`swarm/takeoff_complete`、tracking status 和 MUX selected，提供假的轨迹验证服务。断言顺序：

```text
ready -> generation=1 -> 发布不超过8米的 ego goal
DETECTING -> generation=2 -> CANDIDATE_HOLD 且不再发新 goal
TRACKING/DASH 或 MUX external -> 保持 generation=2 且旧 goal 失效
tracking 退出但仍 external -> 保持 TRACKING_EXTERNAL
MUX navigator -> REJOINING -> generation=3
新 odom、新地图、新验证通过 -> 从当前位置发布新 goal
```

若 `DETECTING` 已产生 generation=2，确认后的 TRACKING/DASH 不再次递增；只有新的接管会话才递增一次。假检测从 `DETECTING` 返回 `IDLE` 时进入 `REJOINING` 并生成 generation=3。

- [ ] **Step 4: 运行测试确认失败**

Run: `rostest search_coordinator search_coordinator_node.test`

Expected: `ERROR`，节点不存在。

- [ ] **Step 5: 实现 ROS 接线**

输入话题：

```text
/typhoon_h480_0/move_base_simple/goal
/typhoon_h480_0/global_odom
/typhoon_h480_0/local_mapping/health
/typhoon_h480_0/local_mapping/frontier_goal
/typhoon_h480_0/mission/active
/swarm/takeoff_complete
/typhoon_h480_0/tracking/phase
/typhoon_h480_0/pose_cmd_mux/selected
```

输出话题：

```text
/typhoon_h480_0/ego/goal
/typhoon_h480_0/navigation/task_generation
/typhoon_h480_0/search_coordinator/status
```

generation 使用 latched `std_msgs/UInt64`。tracking phase 前缀 `DETECTING:` 立即递增 generation 并进入候选保持，`DASH:`/`TRACKING:`/`LOST:` 视为 tracking 活跃；MUX selected 才是控制权最终事实。`RESUME` 后禁止重发旧局部目标。

- [ ] **Step 6: 实现观察故障策略**

没有已知空闲目标时 coordinator 不发布平移目标，只发布状态 `OBSERVING:NO_KNOWN_FREE_GOAL`。本阶段原地观察由 adapter 的零平移和最多 `0.25 rad/s` 小角度 yaw 承担；tracking candidate 一出现立即把 yaw 也归零，避免丢失人物视野。

- [ ] **Step 7: 运行测试并提交**

```bash
catkin_make --pkg tracking search_coordinator
rostest tracking tracking_phase.test
rostest search_coordinator search_coordinator_node.test
catkin_test_results build/test_results/search_coordinator
git add src/tracking src/ego_fusion_search/search_coordinator
git commit -m "feat: invalidate EGO tasks across tracking takeover"
```

### Task 9: 创建 EGO 单机 Launch 和导航模式互斥

**Files:**
- Create: `src/ego_fusion_search/ego_adapter/launch/ego_single.launch`
- Create: `src/ego_fusion_search/ego_adapter/scripts/ego_external_guard.py`
- Create: `src/ego_fusion_search/ego_adapter/scripts/ego_camera_bootstrap.py`
- Create: `src/ego_fusion_search/search_coordinator/launch/search_single.launch`
- Create: `src/look_up/launch/navigation_single.launch`
- Modify: `src/look_up/launch/down_resume.launch`
- Modify: `1.sh`
- Modify: `tests/test_one_click_launch.py`
- Modify: `scripts/verify_competition_clean.sh`
- Modify: `tests/test_verification_scripts.py`
- Modify: `tests/test_ego_navigation_wiring.py`

- [ ] **Step 1: 写静态接线失败测试**

在 `tests/test_ego_navigation_wiring.py` 解析 XML 并断言：

```python
class NavigationModeWiringTest(unittest.TestCase):
    def test_modes_are_mutually_exclusive(self):
        text = (ROOT / "src/look_up/launch/navigation_single.launch").read_text()
        self.assertIn("navigation_mode", text)
        self.assertIn("navigation_mode == 'ego'", text)
        self.assertIn("navigation_mode == 'static_patrol'", text)
        self.assertIn("navigation_mode is not implemented", text)
        self.assertNotIn("cmd_vel_flu", text)

    def test_ego_mode_does_not_start_simple_navigator(self):
        text = (ROOT / "src/look_up/launch/navigation_single.launch").read_text()
        ego_group = text.split("navigation_mode == 'ego'", 1)[1].split("</group>", 1)[0]
        self.assertNotIn("simple_navigator", ego_group)
        self.assertIn("ego_single.launch", ego_group)
        self.assertIn("search_single.launch", ego_group)

    def test_down_resume_replaces_unconditional_navigator_include(self):
        text = (ROOT / "src/look_up/launch/down_resume.launch").read_text()
        self.assertIn("navigation_mode", text)
        self.assertIn("active_num_drones", text)
        self.assertIn("navigation_single.launch", text)
        self.assertNotIn(
            '<include file="$(find simple_navigator)/launch/nav.launch">',
            text.split("active_num_drones", 1)[0],
        )
```

- [ ] **Step 2: 运行测试确认失败**

Run: `python3 -m unittest tests.test_ego_navigation_wiring.NavigationModeWiringTest -v`

Expected: `ERROR`，Launch 不存在。

- [ ] **Step 3: 创建只支持0号机的 EGO Launch**

`ego_single.launch` 必须拒绝非 `typhoon_h480_0`。执行前必须 source 外部 EGO 的 `devel/setup.bash`，并运行 `scripts/check_ego_external.py`。Launch 不直接并行启动 EGO；`ego_camera_bootstrap.py` 先等待一组与 `planner_depth` 同尺寸、K 矩阵有限且焦距为正的真实 `CameraInfo`，把 `fx=K[0]`、`fy=K[4]`、`cx=K[2]`、`cy=K[5]` 作为参数启动 Task 1 固定的上游 `single_run_in_sim.launch`。这样相机内参来自实际传感器，不从模型文本猜测。bootstrap 监督子 roslaunch，子进程退出时自身非0退出并触发 required shutdown。

bootstrap 启动的 remap 固定为：

```text
~odom_world            <- /typhoon_h480_0/global_odom
~grid_map/depth        <- /typhoon_h480_0/local_mapping/planner_depth
~grid_map/pose         <- /typhoon_h480_0/local_mapping/planner_pose
/move_base_simple/goal <- /typhoon_h480_0/ego/goal
/position_cmd          -> /typhoon_h480_0/ego/position_cmd
/broadcast_bspline     -> /typhoon_h480_0/ego/broadcast_bspline
```

新增队伍自有 `ego_external_guard.py`：读取必填参数 `ego_dir`，执行与 `check_ego_external.py` 相同的提交、干净工作树和接口检查，成功后以1Hz存活；失败则返回非0。该节点设 `required="true"`，通过 `CMakeLists.txt` 的 `catkin_install_python` 安装。路径错误或核验失败时整套单机启动退出，不能启动静态巡逻替代。

Task 1 的接口检查同时确认上游 launch 暴露相机内参参数和深度编码路径。bootstrap 只接受 `16UC1`（毫米）或 `32FC1`（米），把实际 encoding 传给上游对应参数；其他 encoding 立即失败。rostest 用两组 CameraInfo 证明传入值来自 K，并用尺寸变化、NaN K、未知 encoding 证明 EGO 不会启动。

- [ ] **Step 4: 创建互斥入口**

`navigation_single.launch` 固定只接受已实现的 `ego` 和 `static_patrol`。`layered_2d` 是已选的后续人工备用方案，但本阶段选择它时必须在解析期报错，不能静默降级：

```xml
<arg name="navigation_mode" default="ego"/>
<arg name="vehicle_type" default="typhoon_h480"/>
<arg name="drone_id" default="0"/>
<arg name="_validated_mode"
     value="$(eval navigation_mode if navigation_mode in ['ego', 'static_patrol'] else int('navigation_mode is not implemented in the single-drone EGO phase'))"/>
<group if="$(eval navigation_mode == 'ego')">
  <include file="$(find local_mapping)/launch/local_mapping_single.launch"/>
  <include file="$(find ego_adapter)/launch/ego_single.launch"/>
  <include file="$(find search_coordinator)/launch/search_single.launch"/>
</group>
<group if="$(eval navigation_mode == 'static_patrol')">
  <include file="$(find simple_navigator)/launch/nav.launch">
    <arg name="num_drones" value="1"/>
  </include>
</group>
```

在 `down_resume.launch` 增加 `navigation_mode`，默认 `static_patrol` 以保持现有六机启动行为，并定义 `active_num_drones=$(eval 1 if navigation_mode == 'ego' else int(num_drones))`。定位可以继续看见六个仿真模型，但 safety、MUX、起飞、任务、tracking、TF 和导航 include 全部使用 `active_num_drones`；`ego` 因而只给0号机建立控制链和起飞任务。static 模式继续 include 现有六机 `simple_navigator/nav.launch`，ego 模式只 include `navigation_single.launch`。六机复制 EGO 节点的 Launch 结构不存在。

修改 `1.sh` 接受第三个参数 `navigation_mode` 和第四个参数 `goal_source`：前者只允许 `static_patrol|ego|layered_2d`，默认 `static_patrol`；后者只允许 `mission|manual`，默认 `mission`。两者传给 `down_resume.launch`；`manual` 时不启动 task_manager，只接受人工/验收 goal。ego 模式还必须 `require_file` 并 source 外部 EGO `devel/setup.bash`，调用 `scripts/check_ego_external.py` 通过后才启动。`layered_2d` 在本阶段由 `down_resume` 解析期明确拒绝。`tests/test_one_click_launch.py` 增加 `bash 1.sh 6 mission_down.json ego manual` 的参数转发测试、manual 分支不启动 task_manager 的 Launch 测试和缺失 EGO 环境的失败测试。

`scripts/verify_competition_clean.sh` 在 Catkin 构建前运行 `check_ego_external.py` 并 source 固定外部 EGO `devel/setup.bash`；目录缺失、提交错误或工作树有修改时给出明确错误并退出，不能跳过新包。`tests/test_verification_scripts.py` 用临时 fake EGO 目录验证检查顺序位于 `catkin_make` 之前，并验证失败不会继续构建。

- [ ] **Step 5: 增加启动前唯一发布者检查**

静态测试必须解析 `down_resume.launch`，证明 ego 条件分支不 include `simple_navigator`。运行中 adapter 每秒通过 ROS master `getSystemState` 检查 `/typhoon_h480_0/mux_inputs/navigator/cmd_vel` 的发布节点，集合必须恰为本 adapter；数量不是1或出现 `waypoint_navigator` 时状态为 `NAVIGATOR_PUBLISHER_CONFLICT` 并持续发零。

- [ ] **Step 6: 运行接线测试并提交**

```bash
python3 -m unittest tests.test_ego_navigation_wiring -v
python3 -m unittest tests.test_one_click_launch -v
python3 -m unittest tests.test_verification_scripts -v
roslaunch --nodes look_up navigation_single.launch navigation_mode:=ego
roslaunch --nodes look_up navigation_single.launch navigation_mode:=static_patrol
git add src/ego_fusion_search/ego_adapter/launch \
  src/ego_fusion_search/ego_adapter/scripts \
  src/ego_fusion_search/search_coordinator/launch \
  src/look_up/launch 1.sh tests/test_ego_navigation_wiring.py \
  tests/test_one_click_launch.py scripts/verify_competition_clean.sh \
  tests/test_verification_scripts.py
git commit -m "feat: add mutually exclusive single-drone navigation modes"
```

Expected: ego 节点列表不含 `waypoint_navigator`；static 列表不含 EGO adapter/coordinator；两种模式均不直接发布 `cmd_vel_flu`。

### Task 10: 把深度 P0 保护放到 tracking 也经过的最终安全层

**Files:**
- Modify: `src/ego_fusion_search/safety_filter/include/safety_filter/safety_policy.h`
- Modify: `src/ego_fusion_search/safety_filter/src/safety_policy.cpp`
- Modify: `src/ego_fusion_search/safety_filter/src/safety_filter_node.cpp`
- Modify: `src/ego_fusion_search/safety_filter/config/default.yaml`
- Modify: `src/ego_fusion_search/safety_filter/launch/safety_filter_swarm.launch`
- Modify: `src/ego_fusion_search/safety_filter/test/safety_policy_test.cpp`
- Create: `src/ego_fusion_search/safety_filter/test/perception_guard.test`
- Create: `src/ego_fusion_search/safety_filter/test/test_perception_guard.py`

- [ ] **Step 1: 写最终层失败测试**

纯策略测试固定三种控制源高度：takeoff 和 navigator 的上限4.0米，external tracking 的上限6.0米；unknown MUX 一律拒绝。rostest 启动真实 MUX 与 safety_filter，依次选择 navigator/external 并发布 raw command、health 和六方向 clearance，断言：

```text
navigator z=4.01 且继续上升 -> final z=0, HEIGHT_LIMIT
external z=4.01 且低于6.0 -> 仍可在净空允许时跟踪
external z=6.01 且继续上升 -> final z=0, HEIGHT_LIMIT
depth/map health 超时 -> final 六自由度全零, PERCEPTION_TIMEOUT
forward unknown/过近 + raw x>0 -> final x=0
left unknown/过近 + raw y>0 -> final y=0
up unknown/过近 + raw z>0 -> final z=0
MUX selected 不明 -> final 全零, MUX_UNKNOWN
```

Run:

```bash
catkin_make run_tests_safety_filter
rostest safety_filter perception_guard.test
```

Expected: 新断言失败，因为当前最终层只读 odom、raw command 和固定4米上限。

- [ ] **Step 2: 实现模式化高度和六方向净空策略**

`SafetyPolicy::apply` 增加本帧 `max_altitude` 参数，不把全局限制永久改成6米。节点根据 MUX selected 精确映射：

```cpp
if (selected == takeoff_topic_ || selected == navigator_topic_) {
  mode_max_altitude = 4.0;
} else if (selected == external_topic_) {
  mode_max_altitude = 6.0;
} else {
  return publishZero("MUX_UNKNOWN");
}
```

新增 `PerceptionGuard` 把 `LocalClearance` 六方向映射到 FLU command 符号；每个非零平移分量必须对应 `known=true`，并按 `emergency_clearance=0.80m` 禁止、`braking_clearance=1.50m` 线性缩放。任何 NaN、负距离或不匹配 frame 归零。

- [ ] **Step 3: 只在 ego 模式开启感知保护**

为 Launch 增加参数：

```yaml
perception_guard_enabled: false
perception_timeout: 0.50
navigator_max_altitude: 4.0
external_max_altitude: 6.0
braking_clearance: 1.50
emergency_clearance: 0.80
```

static_patrol 默认 `false`，保持现有六机基线；`down_resume navigation_mode:=ego` 显式传 `true`。启用后 safety_filter 订阅：

```text
/typhoon_h480_0/local_mapping/health
/typhoon_h480_0/local_mapping/clearance
/typhoon_h480_0/pose_cmd_mux/selected
```

health 必须 depth/odom/map 全健康且不超过0.50秒；否则最终输出全零。该检查位于 MUX 之后，因此 navigator 与 external tracking 都不能绕过 P0。takeoff 在 local_mapping 尚未恢复1秒前也保持零，避免无感知起飞；启动测试必须证明 local_mapping 健康后起飞门才可继续。

- [ ] **Step 4: 运行测试并提交**

```bash
catkin_make --pkg safety_filter
catkin_make run_tests_safety_filter
rostest safety_filter perception_guard.test
catkin_test_results build/test_results/safety_filter
git add src/ego_fusion_search/safety_filter src/look_up/launch/down_resume.launch
git commit -m "feat: guard tracking with final depth safety"
```

Expected: navigator 始终受4米限制，tracking 可在4至6米间工作但仍受深度、可靠 odom、六方向净空和 MUX 事实约束。

### Task 11: 登记队伍所有权和维护文档

**Files:**
- Modify: `src/competition_compliance/config/ownership.json`
- Modify: `src/competition_compliance/test/test_ownership.py`
- Modify: `docs/AI_AGENT_HANDOFF.md`
- Modify: `docs/THIRD_PARTY.md`

- [ ] **Step 1: 先让所有权测试准确失败**

在 `TEAM_ENTRIES` 增加：

```python
"src/ego_fusion_search/ego_adapter": ("0.1.0", "LicenseRef-Team-Code"),
"src/ego_fusion_search/search_coordinator": ("0.1.0", "LicenseRef-Team-Code"),
```

Run: `python3 -m unittest src.competition_compliance.test.test_ownership -v`

Expected: `FAIL`，ownership 缺少两个队伍包。

- [ ] **Step 2: 登记两个队伍包**

在 `ownership.json` 的 team entries 加入：

```json
{"path":"src/ego_fusion_search/ego_adapter","kind":"team","source":"this repository","version":"0.1.0","license":"LicenseRef-Team-Code"},
{"path":"src/ego_fusion_search/search_coordinator","kind":"team","source":"this repository","version":"0.1.0","license":"LicenseRef-Team-Code"}
```

- [ ] **Step 3: 更新交接手册**

写清：0号机是唯一已接入目标；EGO 是仓库外 GPL-3.0 只读运行依赖；`ego_adapter` 只能发布 navigator MUX 输入；`search_coordinator` 只发局部目标与 generation；`local_mapping` 是唯一占据事实来源；`layered_2d` 尚未实现；不得声称双机或六机 EGO 已跑通。

- [ ] **Step 4: 运行合规测试并提交**

```bash
python3 -m unittest src.competition_compliance.test.test_ownership -v
python3 -m json.tool src/competition_compliance/config/ownership.json >/dev/null
git add src/competition_compliance docs/AI_AGENT_HANDOFF.md docs/THIRD_PARTY.md
git commit -m "docs: register team-owned EGO navigation packages"
```

Expected: 全部通过；EGO 外部源码不在 ownership 中。

### Task 12: 自动化故障注入与控制链回归

**Files:**
- Create: `src/ego_fusion_search/ego_adapter/test/test_ego_single_faults.py`
- Create: `src/ego_fusion_search/ego_adapter/test/ego_single_faults.test`
- Modify: `src/ego_fusion_search/ego_adapter/CMakeLists.txt`

- [ ] **Step 1: 建立故障注入 rostest**

用假的 EGO publisher 和真实 `local_mapping`、coordinator、adapter、MUX、safety_filter 组成无 PX4 的闭环，覆盖：

```text
正常轨迹 -> navigator raw 和 final 都有限
停止 depth -> 0.50秒内 adapter 归零
停止 odom -> 0.50秒内 adapter 归零
EGO PositionCommand 含 NaN -> 立即归零
Bspline 穿 unknown -> 服务拒绝且归零
目标 z=4.01 -> HEIGHT_LIMIT 且归零
前方 clearance=0.79 -> 禁止继续接近
MUX selected 不明确 -> 归零
EGO publisher 退出 -> 0.30秒内 adapter raw 和 safety final 都归零
tracking external 接管 -> generation 增加，旧轨迹永久失效
tracking external 前进且深度超时 -> safety final 全零
tracking external 在 z=4.5 且净空健康 -> 可执行
tracking external 在 z=6.01 且继续上升 -> safety final z=0
```

- [ ] **Step 2: 运行故障测试确认暴露缺口**

Run: `rostest ego_adapter ego_single_faults.test`

Expected: 首次运行至少一个未实现故障路径失败；不得通过放宽 timeout 或删除断言规避。

- [ ] **Step 3: 逐项修到失效关闭**

只修改 `local_mapping`、`ego_adapter`、`search_coordinator`、`safety_filter` 和队伍 Launch。每修一个故障立即重跑对应测试；所有回调异常、服务超时和 ROS 时钟倒退都必须产生显式 fault_code 并输出零。

- [ ] **Step 4: 运行全部自动测试并提交**

```bash
python3 -m unittest tests.test_ego_navigation_wiring -v
catkin_make run_tests_search_msgs run_tests_local_mapping \
  run_tests_ego_adapter run_tests_search_coordinator run_tests_tracking \
  run_tests_safety_filter
catkin_test_results build/test_results
git add tests src/ego_fusion_search src/look_up/launch src/tracking
git commit -m "test: verify fail-closed single-drone EGO control"
```

Expected: `0 tests failed`；既有 tracking、task_manager、safety_filter 测试也保持通过。0.30秒由 adapter 的0.20秒命令时限加20Hz发布调度余量构成，不能放宽到 safety_filter 单独的0.25秒超时之后继续沿用旧 raw command。

### Task 13: 单机 Gazebo 分场景验收

**Files:**
- Create: `scripts/record_ego_acceptance.sh`
- Create: `scripts/ego_acceptance_scenario.py`
- Create: `scripts/analyze_ego_acceptance.py`
- Create: `tests/test_ego_acceptance_tools.py`
- Create: `src/ego_fusion_search/ego_adapter/scripts/acceptance_proxy.py`
- Create: `src/ego_fusion_search/ego_adapter/launch/acceptance_proxy.launch`
- Modify: `src/ego_fusion_search/ego_adapter/CMakeLists.txt`
- Modify: `src/look_up/launch/down_resume.launch`
- Create: `docs/verification/ego-single/README.md`

- [ ] **Step 1: 先写验收工具失败测试**

`tests/test_ego_acceptance_tools.py` 导入 scenario/analyzer 的纯函数，用合成 odom、goal、status、command 和 contacts 文本断言：0接触通过；任意包含 `typhoon_h480_0` 的 collision1/2 接触失败；搜索 z>4失败；tracking z>6失败；非零最终命令没有100ms内成功轨迹验证失败；generation 倒退或 tracking 后复用旧 generation 失败；故障后超过0.30秒仍非零失败；RTF<0.8或 replan P95>200ms失败。

Run: `python3 -m unittest tests.test_ego_acceptance_tools -v`

Expected: `ERROR`，三个工具尚不存在。

- [ ] **Step 2: 创建只在 manual 验收模式启用的输入代理**

`acceptance_proxy.py` 透明转发 depth、odom、EGO PositionCommand 和 Bspline，各自用 `std_srvs/SetBool` 服务开关：

```text
/ego_acceptance/depth_enabled
/ego_acceptance/odom_enabled
/ego_acceptance/position_cmd_enabled
/ego_acceptance/bspline_enabled
```

`down_resume goal_source:=manual` include 该代理，并把 local_mapping、EGO、coordinator、adapter 输入 remap 到代理输出；`goal_source:=mission` 完全不启动或经过该代理。每个开关默认 true，关闭时停止发布而不是发布伪造零数据，恢复时只转发新消息。manual 模式没有 task_manager，因此代理还订阅 latched `/swarm/takeoff_complete`，把同一 Bool 转发为 latched `/typhoon_h480_0/mission/active`；只有起飞门打开后 coordinator 与真实 tracking 才能运行，关闭时立即复位。

- [ ] **Step 3: 创建确定性场景脚本**

`ego_acceptance_scenario.py` 提供：

```text
prepare --kind clear|wall|corner|unknown
goal --distance METRES --lateral METRES --height METRES
fault --source depth|odom|position_cmd|bspline --enabled true|false
tracking --phase detecting|external|resume
cleanup
```

`wall` 和 `corner` 通过 `/gazebo/spawn_sdf_model` 在0号机当前前方4米生成名为 `team_acceptance_*` 的简单 box SDF；`cleanup` 通过 `/gazebo/delete_model` 删除，仅改变本次运行状态，不写官方 World 或模型。`unknown` 在当前前视8米外发目标，不生成真值导航提示。`tracking` 使用现有 actor `cmd_motion` 把一个人物移到相机正前方并等待真实 YOLO phase；只有超时诊断时才允许显式报告“人物未被检测”，不得伪造检测成功。

- [ ] **Step 4: 创建记录脚本和分析器**

脚本接收 `run_id`，记录到 `logs/competition-clean/ego-single/<run_id>/`，至少包含：

```text
/clock
/gazebo/model_states
/typhoon_h480_0/global_odom
/typhoon_h480_0/local_mapping/health
/typhoon_h480_0/local_mapping/clearance
/typhoon_h480_0/local_mapping/static_cloud
/typhoon_h480_0/local_mapping/dynamic_cloud
/typhoon_h480_0/navigation/task_generation
/typhoon_h480_0/search_coordinator/status
/typhoon_h480_0/ego_adapter/status
/typhoon_h480_0/tracking/phase
/typhoon_h480_0/mission/active
/typhoon_h480_0/pose_cmd_mux/selected
/typhoon_h480_0/control/raw_cmd_vel
/xtdrone/typhoon_h480_0/cmd_vel_flu
```

ROS 话题写入 `run.bag`。contacts 不使用不存在的 `/gazebo/contact_states`，而是并行运行：

```bash
gz topic -e -t /gazebo/default/physics/contacts > contacts.log
```

脚本用 trap 结束 rosbag 和 `gz topic` 并保存退出码。`analyze_ego_acceptance.py --bag run.bag --contacts contacts.log --out result.json` 计算最大搜索/tracking 高度、contacts、目标误差、故障归零延迟、generation 顺序、非零命令最近验证 age、基于 `/clock` 与 bag wall stamp 的 RTF、coordinator `REPLAN_MS:` 状态的 P95；任一门槛失败返回1。日志目录不得提交 Git。

- [ ] **Step 5: 运行工具测试并保存官方输入基线**

Run:

```bash
python3 -m unittest tests.test_ego_acceptance_tools -v
bash scripts/verify_competition_clean.sh
python3 scripts/check_ego_external.py
git -C /home/wangtao/robocup_fly/PX4_Firmware status --short
git -C /home/wangtao/robocup_fly/XTDrone status --short
```

Expected: 工具测试和标准 competition-clean 验证入口通过；EGO 是固定提交且干净；官方输入没有本任务产生的修改。

- [ ] **Step 6: 按冷启动执行七个单机场景**

每个场景打开三个终端。终端A：

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
source /opt/ros/noetic/setup.bash
source /home/wangtao/robocup_fly/external/ego-planner-swarm/devel/setup.bash
source devel/setup.bash
bash 1.sh 6 mission_down.json ego manual
```

该命令仍生成官方六机仿真模型，但控制、起飞、任务、tracking 和 EGO 节点只为0号机启动，其余五机保持地面无控制。终端B运行 `bash scripts/record_ego_acceptance.sh <run_id>`；终端C按场景运行：

```bash
python3 scripts/ego_acceptance_scenario.py prepare --kind clear
python3 scripts/ego_acceptance_scenario.py goal --distance 5 --lateral 0 --height 3

python3 scripts/ego_acceptance_scenario.py prepare --kind wall
python3 scripts/ego_acceptance_scenario.py goal --distance 8 --lateral 0 --height 3

python3 scripts/ego_acceptance_scenario.py prepare --kind corner
python3 scripts/ego_acceptance_scenario.py goal --distance 8 --lateral 2 --height 3

python3 scripts/ego_acceptance_scenario.py prepare --kind unknown
python3 scripts/ego_acceptance_scenario.py goal --distance 8 --lateral 0 --height 3

python3 scripts/ego_acceptance_scenario.py fault --source depth --enabled false
python3 scripts/ego_acceptance_scenario.py fault --source depth --enabled true
python3 scripts/ego_acceptance_scenario.py fault --source odom --enabled false
python3 scripts/ego_acceptance_scenario.py fault --source odom --enabled true
python3 scripts/ego_acceptance_scenario.py fault --source position_cmd --enabled false
python3 scripts/ego_acceptance_scenario.py fault --source position_cmd --enabled true

python3 scripts/ego_acceptance_scenario.py tracking --phase detecting
python3 scripts/ego_acceptance_scenario.py tracking --phase external
python3 scripts/ego_acceptance_scenario.py tracking --phase resume
```

每个 prepare 前先 `cleanup`，每场结束运行 analyzer。clear 必须直达；wall/corner 必须绕行到达；unknown 必须先保持/观察再推进；每个 fault 必须在门槛内归零并只在连续健康1秒后恢复；tracking 必须看到真实 `DETECTING -> DASH/TRACKING -> RETURNING/IDLE`，候选阶段导航归零、external 期间最终 P0 有效、resume 后 generation 递增且旧轨迹不恢复。

- [ ] **Step 7: 连续五轮完整验收**

至少连续5次运行场景2、4、5、6、7，全部满足：

```text
建筑/地面/模型接触 = 0
绕障后到达目标 = true
搜索高度最大值 <= 4.0m，tracking 高度 <= 6.0m
Gazebo 实时因子约 >= 0.8
EGO 重规划 P95 约 <= 200ms
故障发生后输出归零满足配置时限
tracking 接管与恢复 generation 严格递增
```

任何一轮失败都归零连续计数，修复后重新从第1轮开始。

- [ ] **Step 8: 重跑边界核验**

```bash
bash scripts/verify_competition_clean.sh
python3 scripts/check_ego_external.py
git diff --check
git status --short
```

Expected: PX4、XTDrone、Gazebo、EGO、第三方包和官方模型保持原版；Git 只出现队伍代码、配置、测试和验收文档，`.superpowers/`、日志与 bag 未被跟踪。

- [ ] **Step 9: 提交验收工具和证据索引**

```bash
git add scripts/record_ego_acceptance.sh scripts/ego_acceptance_scenario.py \
  scripts/analyze_ego_acceptance.py tests/test_ego_acceptance_tools.py \
  src/ego_fusion_search/ego_adapter/scripts/acceptance_proxy.py \
  src/ego_fusion_search/ego_adapter/launch/acceptance_proxy.launch \
  src/ego_fusion_search/ego_adapter/CMakeLists.txt \
  src/look_up/launch/down_resume.launch docs/verification/ego-single/README.md
git commit -m "test: document single-drone EGO acceptance"
```

## 完成门槛

只有 Task 1 至 Task 13 全部通过并连续5轮 Gazebo 无接触，才能把“0号机 EGO 单机绕障已跑通”写入交接手册。此后另写双机轨迹冲突与避碰设计；不得直接复制0号机 Launch 到六机并宣称完成集群规划。
