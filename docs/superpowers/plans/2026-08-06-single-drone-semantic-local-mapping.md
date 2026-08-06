# Single-Drone Semantic Local Mapping Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 `typhoon_h480_0` 建立可独立验收的深度健康检查、人物语义过滤、稀疏三维占据地图、方向净空和安全前沿输出，为下一阶段 EGO 接入提供稳定输入。

**Architecture:** 新增队伍自有 `search_msgs` 和 `local_mapping` 两个 ROS 1 Catkin 包。`local_mapping` 同步 Realsense 深度、`CameraInfo`、`global_odom` 和 YOLO 检测框，分别输出给 EGO 使用的即时障碍深度、排除人物的持久静态地图、带 TTL 的人物动态层、结构化健康状态和已知空闲侧前沿；本阶段不发布飞行控制命令。

**Tech Stack:** Ubuntu 20.04、ROS Noetic、C++14、roscpp、message_filters、cv_bridge、OpenCV、Eigen、sensor_msgs、nav_msgs、darknet_ros_msgs、GoogleTest、rostest、Catkin、Gazebo Classic 11。

---

## 范围边界

本计划是 [LOCAL_MAPPING_NAVIGATION_DESIGN.md](../../LOCAL_MAPPING_NAVIGATION_DESIGN.md) 的第一个独立交付阶段，只完成感知和地图基础。以下内容分别留给后续计划：

- EGO-Planner-Swarm 构建、Launch 和 `PositionCommand -> Twist` 适配；
- `safety_filter` 深度制动、tracking 6米上限和退出高度恢复；
- `move_base + costmap_2d` 多高度备用模式；
- 双机轨迹避碰和六机任务分配。

EGO 上游审计基线为 `ZJU-FAST-Lab/ego-planner-swarm` 提交 `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20`，许可证 GPL-3.0。本计划不复制、不修改、不提交该外部源码。

## 文件结构

```text
src/ego_fusion_search/search_msgs/
|-- CMakeLists.txt
|-- package.xml
`-- msg/
    |-- PerceptionHealth.msg
    `-- LocalClearance.msg

src/ego_fusion_search/local_mapping/
|-- CMakeLists.txt
|-- package.xml
|-- include/local_mapping/
|   |-- health_monitor.h
|   |-- semantic_depth_filter.h
|   |-- voxel_map.h
|   `-- frontier_selector.h
|-- src/
|   |-- health_monitor.cpp
|   |-- semantic_depth_filter.cpp
|   |-- voxel_map.cpp
|   |-- frontier_selector.cpp
|   `-- local_mapping_node.cpp
|-- config/single_drone.yaml
|-- launch/local_mapping_single.launch
`-- test/
    |-- health_monitor_test.cpp
    |-- semantic_depth_filter_test.cpp
    |-- voxel_map_test.cpp
    |-- frontier_selector_test.cpp
    |-- local_mapping_node.test
    `-- test_local_mapping_node.py
```

修改：

- `src/competition_compliance/config/ownership.json`
- `src/competition_compliance/test/test_ownership.py`
- `docs/THIRD_PARTY.md`
- `docs/AI_AGENT_HANDOFF.md`

## 统一消息和话题

`PerceptionHealth.msg`：

```text
std_msgs/Header header
bool depth_healthy
bool odom_healthy
bool synchronized
bool map_healthy
float32 valid_depth_ratio
uint32 dropped_frames
string fault_code
```

`LocalClearance.msg`：

```text
std_msgs/Header header
bool forward_known
bool backward_known
bool left_known
bool right_known
bool upward_known
bool downward_known
float32 forward_m
float32 backward_m
float32 left_m
float32 right_m
float32 upward_m
float32 downward_m
```

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 输入 | `/typhoon_h480_0/realsense/depth_camera/depth/image_raw` | `sensor_msgs/Image` |
| 输入 | `/typhoon_h480_0/realsense/depth_camera/depth/camera_info` | `sensor_msgs/CameraInfo` |
| 输入 | `/typhoon_h480_0/global_odom` | `nav_msgs/Odometry` |
| 输入 | `/typhoon_h480_0/yolo11n/bounding_boxes` | `darknet_ros_msgs/BoundingBoxes` |
| 输出 | `/typhoon_h480_0/local_mapping/planner_depth` | `sensor_msgs/Image` |
| 输出 | `/typhoon_h480_0/local_mapping/static_cloud` | `sensor_msgs/PointCloud2` |
| 输出 | `/typhoon_h480_0/local_mapping/dynamic_cloud` | `sensor_msgs/PointCloud2` |
| 输出 | `/typhoon_h480_0/local_mapping/health` | `search_msgs/PerceptionHealth` |
| 输出 | `/typhoon_h480_0/local_mapping/clearance` | `search_msgs/LocalClearance` |
| 输出 | `/typhoon_h480_0/local_mapping/frontier_goal` | `geometry_msgs/PoseStamped` |

`planner_depth` 保留人物深度供 EGO 立即避让；持久地图使用人物掩膜后的静态深度。人物动态点只存在于 TTL 层。

### Task 1: 建立消息包和所有权契约

**Files:**
- Create: `src/ego_fusion_search/search_msgs/CMakeLists.txt`
- Create: `src/ego_fusion_search/search_msgs/package.xml`
- Create: `src/ego_fusion_search/search_msgs/msg/PerceptionHealth.msg`
- Create: `src/ego_fusion_search/search_msgs/msg/LocalClearance.msg`
- Create: `src/ego_fusion_search/local_mapping/package.xml`
- Create: `src/ego_fusion_search/local_mapping/CMakeLists.txt`
- Modify: `src/competition_compliance/test/test_ownership.py`
- Modify: `src/competition_compliance/config/ownership.json`

- [ ] **Step 1: 写缺包失败测试**

在 `TEAM_ENTRIES` 增加：

```python
"src/ego_fusion_search/search_msgs": ("0.1.0", "LicenseRef-Team-Code"),
"src/ego_fusion_search/local_mapping": ("0.1.0", "LicenseRef-Team-Code"),
```

并在 `OwnershipDocumentTest` 增加：

```python
def test_local_mapping_team_packages_exist(self):
    for relative in (
        "src/ego_fusion_search/search_msgs/package.xml",
        "src/ego_fusion_search/local_mapping/package.xml",
    ):
        self.assertTrue((ROOT / relative).is_file(), relative)
```

- [ ] **Step 2: 运行测试确认 RED**

```bash
python3 -m unittest src.competition_compliance.test.test_ownership.OwnershipDocumentTest.test_local_mapping_team_packages_exist
```

Expected: FAIL，明确缺少两个 `package.xml`。

- [ ] **Step 3: 创建包和消息**

`search_msgs/CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(search_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES PerceptionHealth.msg LocalClearance.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
```

`search_msgs/package.xml` 使用格式2、版本 `0.1.0`、许可证 `LicenseRef-Team-Code`，依赖 `message_generation`、`message_runtime`、`std_msgs`。

`local_mapping/package.xml` 使用相同版本和许可证，并声明 `roscpp`、`cv_bridge`、`darknet_ros_msgs`、`geometry_msgs`、`message_filters`、`nav_msgs`、`pcl_conversions`、`pcl_ros`、`search_msgs`、`sensor_msgs`、`tf2`、`tf2_geometry_msgs`，测试依赖 `rostest` 和 `rospy`。

本任务先创建可被 Catkin 发现的最小 `local_mapping/CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(local_mapping)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED)
catkin_package()
```

在 `ownership.json` 加两条匹配的 `kind: team` 记录。

- [ ] **Step 4: 构建并验证 GREEN**

```bash
catkin_make --pkg search_msgs local_mapping
python3 -m unittest src.competition_compliance.test.test_ownership
```

Expected: 两条命令退出码0，ownership 测试0失败。

- [ ] **Step 5: 提交**

```bash
git add src/ego_fusion_search/search_msgs src/ego_fusion_search/local_mapping/CMakeLists.txt src/ego_fusion_search/local_mapping/package.xml src/competition_compliance/config/ownership.json src/competition_compliance/test/test_ownership.py
git commit -m "feat: define local mapping health interfaces"
```

### Task 2: 实现时间同步和连续恢复门控

**Files:**
- Create: `src/ego_fusion_search/local_mapping/include/local_mapping/health_monitor.h`
- Create: `src/ego_fusion_search/local_mapping/src/health_monitor.cpp`
- Create: `src/ego_fusion_search/local_mapping/test/health_monitor_test.cpp`
- Modify: `src/ego_fusion_search/local_mapping/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

```cpp
TEST(HealthMonitor, RequiresContinuousRecoveryAfterTimeout) {
  local_mapping::HealthConfig config{0.15, 0.50, 0.50, 1.00, 0.20};
  local_mapping::HealthMonitor monitor(config);
  monitor.observeDepth(10.00, 0.80);
  monitor.observeOdom(10.05);
  EXPECT_FALSE(monitor.evaluate(10.05).healthy);
  monitor.observeDepth(11.00, 0.80);
  monitor.observeOdom(11.05);
  EXPECT_FALSE(monitor.evaluate(11.05).healthy);
  monitor.observeDepth(12.00, 0.80);
  monitor.observeOdom(12.05);
  EXPECT_TRUE(monitor.evaluate(12.05).healthy);
}

TEST(HealthMonitor, RejectsUnsynchronizedOrLowQualityDepth) {
  local_mapping::HealthMonitor monitor({0.15, 0.50, 0.50, 1.00, 0.20});
  monitor.observeDepth(5.00, 0.10);
  monitor.observeOdom(5.05);
  EXPECT_EQ("DEPTH_INVALID", monitor.evaluate(5.05).fault_code);
}
```

- [ ] **Step 2: 运行测试确认 RED**

```bash
catkin_make health_monitor_test
```

Expected: FAIL，缺少头文件或测试目标。

- [ ] **Step 3: 实现最小健康状态机**

```cpp
struct HealthConfig {
  double max_sync_delta, depth_timeout, odom_timeout;
  double recovery_window, min_valid_depth_ratio;
};
struct HealthResult {
  bool healthy{false}, depth_healthy{false}, odom_healthy{false};
  bool synchronized{false};
  double valid_depth_ratio{0.0};
  std::string fault_code{"NOT_READY"};
};
class HealthMonitor {
 public:
  explicit HealthMonitor(const HealthConfig& config);
  void observeDepth(double stamp, double valid_ratio);
  void observeOdom(double stamp);
  void noteDroppedFrame();
  HealthResult evaluate(double now);
  uint32_t droppedFrames() const;
};
```

`evaluate()` 依次检查有限值、新鲜度、同步差和有效率。首次全部成立时记录恢复起点，持续1秒后才返回健康；任何失败立即清除恢复计时。故障码使用 `DEPTH_TIMEOUT`、`ODOM_TIMEOUT`、`SYNC_ERROR`、`DEPTH_INVALID`。

把最小 CMake 扩展为查找 `roscpp`，创建 `local_mapping_health` 库，并用 `catkin_add_gtest(health_monitor_test test/health_monitor_test.cpp)` 链接该库和 `${catkin_LIBRARIES}`。

- [ ] **Step 4: 验证 GREEN 并提交**

```bash
catkin_make health_monitor_test
devel/lib/local_mapping/health_monitor_test
git add src/ego_fusion_search/local_mapping
git commit -m "feat: add perception health gate"
```

Expected: 全部测试 PASS，0失败。

### Task 3: 分离规划深度和人物静态掩膜

**Files:**
- Create: `src/ego_fusion_search/local_mapping/include/local_mapping/semantic_depth_filter.h`
- Create: `src/ego_fusion_search/local_mapping/src/semantic_depth_filter.cpp`
- Create: `src/ego_fusion_search/local_mapping/test/semantic_depth_filter_test.cpp`
- Modify: `src/ego_fusion_search/local_mapping/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

```cpp
TEST(SemanticDepthFilter, KeepsPlannerObstacleButMasksPersistentDepth) {
  cv::Mat depth(6, 8, CV_16UC1, cv::Scalar(4000));
  darknet_ros_msgs::BoundingBox person;
  person.Class = "green0";
  person.xmin = 2; person.xmax = 4;
  person.ymin = 2; person.ymax = 3;
  const auto result = local_mapping::SemanticDepthFilter(1).apply(depth, {person});
  EXPECT_EQ(4000, result.planner_depth.at<uint16_t>(2, 3));
  EXPECT_EQ(0, result.static_depth.at<uint16_t>(2, 3));
  EXPECT_EQ(255, result.person_mask.at<uint8_t>(1, 1));
  EXPECT_EQ(4000, result.static_depth.at<uint16_t>(0, 7));
}
```

- [ ] **Step 2: 运行测试确认 RED**

```bash
catkin_make semantic_depth_filter_test
```

Expected: FAIL，缺少过滤器实现。

- [ ] **Step 3: 实现深度分流**

```cpp
struct FilteredDepth {
  cv::Mat planner_depth;
  cv::Mat static_depth;
  cv::Mat person_mask;
};
class SemanticDepthFilter {
 public:
  explicit SemanticDepthFilter(int mask_margin_pixels);
  FilteredDepth apply(
      const cv::Mat& depth,
      const std::vector<darknet_ros_msgs::BoundingBox>& boxes) const;
};
```

只把六个比赛人物 ID 视为人物。`planner_depth` 深拷贝输入；`static_depth` 在膨胀并裁剪的掩膜内写零；支持 `16UC1`、`32FC1`，其他编码抛出 `std::invalid_argument`；禁止原地修改 ROS 输入图像。

- [ ] **Step 4: 验证 GREEN 并提交**

```bash
catkin_make semantic_depth_filter_test
devel/lib/local_mapping/semantic_depth_filter_test
git add src/ego_fusion_search/local_mapping
git commit -m "feat: separate person depth from static mapping"
```

Expected: 编码、边界裁剪、非人物框测试全部 PASS。

### Task 4: 实现稀疏三维占据、动态 TTL 和净空

**Files:**
- Create: `src/ego_fusion_search/local_mapping/include/local_mapping/voxel_map.h`
- Create: `src/ego_fusion_search/local_mapping/src/voxel_map.cpp`
- Create: `src/ego_fusion_search/local_mapping/test/voxel_map_test.cpp`
- Modify: `src/ego_fusion_search/local_mapping/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

```cpp
TEST(VoxelMap, RayAndDynamicTtlAreConservative) {
  local_mapping::VoxelMap map(1.0, 2, 2, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {3.1, 0.1, 0.1});
  map.integrateStaticRay({0.1, 0.1, 0.1}, {3.1, 0.1, 0.1});
  EXPECT_EQ(local_mapping::CellState::FREE, map.stateAt({1.1, 0.1, 0.1}, 0.0));
  EXPECT_EQ(local_mapping::CellState::OCCUPIED, map.stateAt({3.1, 0.1, 0.1}, 0.0));
  EXPECT_EQ(local_mapping::CellState::UNKNOWN, map.stateAt({8.1, 0.1, 0.1}, 0.0));
  map.integrateDynamicPoint({2.1, 0.1, 0.1}, 10.0);
  EXPECT_EQ(local_mapping::CellState::OCCUPIED, map.stateAt({2.1, 0.1, 0.1}, 10.5));
  EXPECT_EQ(local_mapping::CellState::FREE, map.stateAt({2.1, 0.1, 0.1}, 11.1));
}

TEST(VoxelMap, UnknownDirectionIsNotClear) {
  local_mapping::VoxelMap map(0.5, 2, 2, 1.0);
  const auto clearance = map.axisClearance(
      {0.0, 0.0, 2.0}, {1.0, 0.0, 0.0}, 4.0, 0.0);
  EXPECT_FALSE(clearance.known);
  EXPECT_DOUBLE_EQ(0.0, clearance.metres);
}
```

- [ ] **Step 2: 运行测试确认 RED**

```bash
catkin_make voxel_map_test
```

Expected: FAIL，缺少 `VoxelMap`。

- [ ] **Step 3: 实现保守地图接口**

```cpp
enum class CellState { UNKNOWN, FREE, OCCUPIED };
struct Vec3 { double x, y, z; };
struct Clearance { bool known; double metres; };
class VoxelMap {
 public:
  VoxelMap(double resolution, int occupied_hits, int free_hits,
           double dynamic_ttl);
  void integrateStaticRay(const Vec3& origin, const Vec3& endpoint);
  void integrateDynamicPoint(const Vec3& point, double stamp);
  CellState stateAt(const Vec3& point, double now) const;
  Clearance axisClearance(const Vec3& origin, const Vec3& unit_axis,
                          double max_distance, double now) const;
  std::vector<Vec3> staticOccupiedPoints(double now) const;
  std::vector<Vec3> dynamicOccupiedPoints(double now) const;
};
```

射线按不大于 `resolution/2` 采样并去重 voxel key。端点之前累计 free evidence，端点累计 occupied evidence，达到阈值才改变静态状态。动态点保存最后观测时间，1秒内覆盖为 occupied，过期后恢复静态证据。净空路径出现 UNKNOWN 立即返回 `known=false, metres=0`。

- [ ] **Step 4: 验证 GREEN 并提交**

```bash
catkin_make voxel_map_test
devel/lib/local_mapping/voxel_map_test
git add src/ego_fusion_search/local_mapping
git commit -m "feat: add conservative semantic voxel map"
```

Expected: 射线、重复观测、TTL、unknown 和六方向测试全部 PASS。

### Task 5: 只在已知空闲侧选择前沿

**Files:**
- Create: `src/ego_fusion_search/local_mapping/include/local_mapping/frontier_selector.h`
- Create: `src/ego_fusion_search/local_mapping/src/frontier_selector.cpp`
- Create: `src/ego_fusion_search/local_mapping/test/frontier_selector_test.cpp`
- Modify: `src/ego_fusion_search/local_mapping/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

```cpp
TEST(FrontierSelector, GoalIsFreeAndAdjacentToUnknown) {
  nav_msgs::OccupancyGrid grid;
  grid.info.width = 5; grid.info.height = 5; grid.info.resolution = 1.0;
  grid.data.assign(25, -1);
  for (int y = 1; y <= 3; ++y)
    for (int x = 1; x <= 2; ++x) grid.data[y * 5 + x] = 0;
  const auto goal = local_mapping::FrontierSelector(2, 8.0)
                        .select(grid, {1.5, 2.5, 2.0});
  ASSERT_TRUE(goal.valid);
  EXPECT_LE(goal.distance_from_robot, 8.0);
  EXPECT_EQ(0, grid.data[goal.cell_y * 5 + goal.cell_x]);
}
```

- [ ] **Step 2: 运行测试确认 RED**

```bash
catkin_make frontier_selector_test
```

Expected: FAIL，缺少选择器。

- [ ] **Step 3: 实现选择规则**

前沿格定义为值0且四邻域至少一个值-1。按八邻域聚类，丢弃小于 `min_cluster_cells` 的簇；目标必须是已知空闲格，距离不超过8米，z 使用当前搜索层。

```text
score = 1.0 * cluster_size
      - 0.5 * distance_from_robot
      - 0.2 * absolute_yaw_change
```

返回类型固定为 `FrontierGoal { bool valid; int cell_x; int cell_y; double x; double y; double z; double yaw; double distance_from_robot; }`。输出 frame 固定 `map`；无合格前沿返回 `valid=false`，不使用 C++17 `std::optional`。

- [ ] **Step 4: 验证 GREEN 并提交**

```bash
catkin_make frontier_selector_test
devel/lib/local_mapping/frontier_selector_test
git add src/ego_fusion_search/local_mapping
git commit -m "feat: select frontiers from known free space"
```

Expected: 空闲侧、最小簇、距离上限和确定性排序测试全部 PASS。

### Task 6: 集成为单机 ROS 节点

**Files:**
- Create: `src/ego_fusion_search/local_mapping/src/local_mapping_node.cpp`
- Create: `src/ego_fusion_search/local_mapping/config/single_drone.yaml`
- Create: `src/ego_fusion_search/local_mapping/launch/local_mapping_single.launch`
- Create: `src/ego_fusion_search/local_mapping/test/local_mapping_node.test`
- Create: `src/ego_fusion_search/local_mapping/test/test_local_mapping_node.py`
- Modify: `src/ego_fusion_search/local_mapping/CMakeLists.txt`

- [ ] **Step 1: 写失败的端到端 rostest**

```python
def test_semantic_mapping_outputs_are_consistent(self):
    self.publish_inputs_for(1.4)
    self.assertTrue(self.health.map_healthy)
    self.assertTrue(self.health.synchronized)
    self.assertEqual("map", self.static_cloud.header.frame_id)
    self.assertGreater(len(self.static_cloud.data), 0)
    self.assertGreater(len(self.dynamic_cloud.data), 0)
    self.assertEqual("map", self.frontier_goal.header.frame_id)

def test_stale_depth_fails_closed_and_dynamic_points_expire(self):
    self.publish_inputs_for(1.4)
    self.stop_depth_but_keep_odom_for(1.2)
    self.assertFalse(self.health.map_healthy)
    self.assertEqual("DEPTH_TIMEOUT", self.health.fault_code)
    self.assertEqual(0, len(self.dynamic_cloud.data))
```

测试 Launch 使用 `/test_drone_0/...`，将恢复窗口设0.20秒、动态 TTL 设0.30秒。

- [ ] **Step 2: 运行测试确认 RED**

```bash
catkin_make --pkg search_msgs local_mapping
rostest local_mapping local_mapping_node.test
```

Expected: FAIL，找不到 `local_mapping_node`。

- [ ] **Step 3: 实现节点**

节点必须：

1. 用 `ApproximateTime<Image, CameraInfo, Odometry>` 同步，队列5；
2. 缓存与深度时间差不超过0.15秒的 `BoundingBoxes`；
3. 使用实际 `CameraInfo.K` 反投影，不写死内参；
4. 每4个像素采样一次，只接受 `16UC1`、`32FC1`；
5. 有效深度0.20至8.00米；
6. 用 odom 和已验证相机 TF 转换到 `map`；
7. 即时发布 `planner_depth`，持久地图只融合 `static_depth`；
8. 人物有效深度写动态层；
9. 5Hz发布点云、净空、健康和前沿；
10. 数据失效时停止融合、不清 unknown、发布 `map_healthy=false`。

前沿使用内部二维投影：以3.0米搜索高度为中心，把完整无人机垂直扫掠范围相交的 occupied voxel 投为100，把扫掠范围全部已知 free 的列投为0，其余投为-1。首版投影分辨率0.25米；任何 voxel 为 unknown 时该列不能投成 free。

首版 YAML：

```yaml
max_sync_delta: 0.15
depth_timeout: 0.50
odom_timeout: 0.50
recovery_window: 1.00
min_valid_depth_ratio: 0.20
depth_min_m: 0.20
depth_max_m: 8.00
pixel_stride: 4
mask_margin_pixels: 4
voxel_resolution: 0.20
occupied_hits: 2
free_hits: 2
dynamic_ttl: 1.00
publish_rate: 5.0
search_altitude: 3.0
max_frontier_distance: 8.0
```

节点不得订阅或发布 MUX、`cmd_vel`、PX4 或 XTDrone 命令话题。检测积压时增加 `dropped_frames` 并丢弃旧帧。

Task 6结束时 `CMakeLists.txt` 的构建图必须等价于：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(local_mapping)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge darknet_ros_msgs geometry_msgs message_filters nav_msgs
  pcl_conversions pcl_ros roscpp search_msgs sensor_msgs
  tf2 tf2_geometry_msgs)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES local_mapping_core
  CATKIN_DEPENDS cv_bridge darknet_ros_msgs geometry_msgs message_filters
    nav_msgs pcl_conversions pcl_ros roscpp search_msgs sensor_msgs
    tf2 tf2_geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

add_library(local_mapping_core
  src/health_monitor.cpp
  src/semantic_depth_filter.cpp
  src/voxel_map.cpp
  src/frontier_selector.cpp)
add_dependencies(local_mapping_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_mapping_core ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(local_mapping_node src/local_mapping_node.cpp)
add_dependencies(local_mapping_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_mapping_node local_mapping_core
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  foreach(test_target health_monitor_test semantic_depth_filter_test
      voxel_map_test frontier_selector_test)
    catkin_add_gtest(${test_target} test/${test_target}.cpp)
    if(TARGET ${test_target})
      target_link_libraries(${test_target} local_mapping_core
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
    endif()
  endforeach()
  add_rostest(test/local_mapping_node.test)
endif()

install(TARGETS local_mapping_core local_mapping_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

- [ ] **Step 4: 验证 GREEN 并提交**

```bash
catkin_make --pkg search_msgs local_mapping
rostest local_mapping local_mapping_node.test
catkin_make run_tests_local_mapping
git add src/ego_fusion_search/local_mapping
git commit -m "feat: publish single-drone semantic local map"
```

Expected: 构建和测试退出码0，Catkin 汇总0 errors、0 failures。

### Task 7: 真实单机 Gazebo 验证和合规回归

**Files:**
- Create: `scripts/check_local_mapping_single.py`
- Create: `scripts/test_check_local_mapping_single.py`
- Modify: `src/ego_fusion_search/local_mapping/launch/local_mapping_single.launch`
- Modify: `docs/THIRD_PARTY.md`
- Modify: `docs/AI_AGENT_HANDOFF.md`

- [ ] **Step 1: 写检查器失败测试**

```python
def validate_sample(sample):
    errors = []
    if sample["health_rate_hz"] < 5.0:
        errors.append("health rate below 5Hz")
    if sample["max_depth_age_s"] > 0.50:
        errors.append("depth age exceeds 0.50s")
    if sample["planner_depth_publishers"] != 1:
        errors.append("planner_depth must have exactly one publisher")
    if sample["control_publishers"]:
        errors.append("local_mapping must not publish control topics")
    return errors
```

单元测试分别验证健康样本、4Hz、0.60秒年龄、两个发布者和意外控制发布者。

- [ ] **Step 2: 运行测试确认 RED**

```bash
python3 -m unittest scripts.test_check_local_mapping_single
```

Expected: FAIL，缺少检查器。

- [ ] **Step 3: 完成检查器和 Launch 守卫**

检查器运行30秒，验证四个输入均有消息、输出 frame 为 `map`、地图更新不低于5Hz、深度年龄不超过0.50秒、`planner_depth` 恰好一个发布者，并确认 `local_mapping_node` 不发布任何控制话题。

Launch 只接受 `vehicle_type:=typhoon_h480 drone_id:=0`；其他编号直接退出，防止第一阶段误启六机。

- [ ] **Step 4: 启动仿真并采集证据**

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
bash 1.sh 1 mission_down.json
```

终端B：

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch local_mapping local_mapping_single.launch vehicle_type:=typhoon_h480 drone_id:=0
```

终端C：

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 scripts/check_local_mapping_single.py --vehicle typhoon_h480_0 --duration 30
```

Expected: 打印 `PASS local mapping single-drone contract`；静态点云稳定；人物只进入动态点云并在离开1秒后清除；停止深度后0.50秒内变为 `DEPTH_TIMEOUT`。

- [ ] **Step 5: 完整回归并提交**

```bash
python3 -m unittest scripts.test_check_local_mapping_single
catkin_make run_tests_local_mapping run_tests_competition_compliance
bash scripts/verify_competition_clean.sh
git status --short
```

Expected: 0失败；完整合规 PASS；PX4、XTDrone、Gazebo 和官方模型哈希不变。`AI_AGENT_HANDOFF.md` 只能写“局部地图基础已验证”，不能写“EGO 已跑通”。

```bash
git add scripts/check_local_mapping_single.py scripts/test_check_local_mapping_single.py src/ego_fusion_search/local_mapping docs/THIRD_PARTY.md docs/AI_AGENT_HANDOFF.md
git commit -m "test: verify semantic mapping in single-drone simulation"
```

## 完成门槛

只有以下条件全部满足才进入 EGO 单机控制计划：

1. `search_msgs`、`local_mapping` 自动化测试0失败；
2. 真实深度、CameraInfo 和 `global_odom` 同步可用；
3. 静态障碍进入持久地图，人物只进入1秒 TTL 动态层；
4. unknown 不会错误变成 free；
5. 前沿目标位于已知空闲侧；
6. 地图更新不低于5Hz，深度不积压；
7. `local_mapping` 没有控制话题发布权；
8. competition-clean 完整验证通过，外部官方文件未修改。

任一项失败都在本阶段闭环，不能提前接入 EGO、tracking 或六机。
