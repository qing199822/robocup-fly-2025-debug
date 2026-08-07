# 4、5号固定路线建筑净空封堵 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 补全已知建筑和导航器提前切换航点的静态几何回归，并修改队伍任务路线，使4、5号机的 `static_patrol` 降级路线具有可验证的保守建筑净空。

**Architecture:** 继续使用 `task_manager` 现有二维线段与膨胀包围盒检查，不把 Gazebo 真值用于运行时导航。测试除2米建筑净空外，再为三栋邻近路线的房屋增加现有 `ARRIVAL_TOLERANCE=3.0m` 的拐点切换余量，从而覆盖“距航点3米就转向下一目标”产生的斜向捷径；测试先红，再与路线修改一起形成绿色提交。

**Tech Stack:** Python 3 `unittest`、JSON、ROS 1 Noetic Catkin、现有 `task_manager` 几何测试

---

## 已知事实和范围边界

- 5号机接触 `house_3_68` 后坠地已有 Gazebo contacts 证据。
- 4号机沿 `(68,12) -> (68,41)` 在仿真时刻约1857进入 `house_1_66` 边界，随后姿态失稳、被弹出并坠地；Gazebo 真值与控制时间线高度可信地确认该建筑碰撞链。
- `house_2_71` 是本轮几何复算发现的额外漏登建筑；它影响4、5号路线，但不是既有事故日志确认的首次接触对象。
- 4、5号机在起飞区内动态互撞已经确认，且不在本静态建筑路线计划中修复。完成本计划后仍禁止声称“两机坠机问题全部解决”或“六机全航程安全”。

只修改：

- `src/mix_nav/task_manager/test/test_mission_clearance.py`
- `src/mix_nav/task_manager/launch/mission_down.json`

禁止修改 PX4、XTDrone、Gazebo、EGO-Planner-Swarm、`robocup.world`、官方无人机模型和第三方插件。`waypoint/mission_down.json` 是运行任务文件的符号链接，不直接编辑。

开始执行时先运行：

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
source /opt/ros/noetic/setup.bash
test -f devel/setup.bash && source devel/setup.bash || true
ROUTE_BASE_SHA="$(git rev-parse HEAD)"
```

后续命令均在该 shell 和仓库根目录执行。

### Task 1: 用遗漏建筑和3米提前切换余量重现风险

**Files:**
- Modify: `src/mix_nav/task_manager/test/test_mission_clearance.py`
- Test: `src/mix_nav/task_manager/test/test_mission_clearance.py`

- [ ] **Step 1: 登记保守 AABB 和运行控制余量**

在常量区加入：

```python
WAYPOINT_SWITCH_ALLOWANCE = 3.0

# Conservative world-frame AABBs derived from robocup.world collision
# transforms and model collision meshes; values are rounded outward.
SWITCH_SENSITIVE_OBSTACLES = (
    ("house_1_66", 54.89, 67.83, 14.91, 31.41, 7.69),
    ("house_2_71", 70.34, 83.08, 8.50, 18.30, 7.20),
    ("house_3_68", 90.10, 102.76, 11.74, 16.54, 10.62),
)
```

把这些条目展开加入 `STATIC_OBSTACLES`。两位小数是向障碍外侧取整后的保守边界，不称为精确模型几何。

- [ ] **Step 2: 让碰撞收集器显式接收障碍和净空**

把函数签名改成：

```python
def collect_obstacle_collisions(
        missions,
        obstacles=STATIC_OBSTACLES,
        horizontal_clearance=HORIZONTAL_CLEARANCE):
```

函数内部循环使用 `obstacles`，包围盒膨胀使用 `horizontal_clearance`，其余逻辑不变。这样既有测试仍检查2米净空，新测试可单独检查2米机体净空加3米提前切换余量，而不会把旧地图全部灯杆无依据地扩大5米。

- [ ] **Step 3: 新增提前切换保守回归**

在 `MissionClearanceTest` 加入：

```python
def test_routes_clear_switch_sensitive_buildings_with_arrival_allowance(self):
    collisions = collect_obstacle_collisions(
        load_missions(MISSION_FILE),
        obstacles=SWITCH_SENSITIVE_OBSTACLES,
        horizontal_clearance=(
            HORIZONTAL_CLEARANCE + WAYPOINT_SWITCH_ALLOWANCE
        ),
    )
    self.assertEqual([], collisions, "\n" + "\n".join(collisions))
```

这不是精确动力学模拟，而是对 `MissionManager` 3米提前切换造成拐角捷径的保守包络；真实动力学仍需后续 Gazebo 全航程验收。

- [ ] **Step 4: 运行测试确认旧路线失败**

Run:

```bash
python3 -m unittest \
  src.mix_nav.task_manager.test.test_mission_clearance.MissionClearanceTest.test_all_complete_segments_clear_known_static_obstacles \
  src.mix_nav.task_manager.test.test_mission_clearance.MissionClearanceTest.test_routes_clear_switch_sensitive_buildings_with_arrival_allowance \
  -v
```

Expected: 两项均 `FAIL`。第一项至少检测到4号 `house_1_66` 和5号 `house_2_71`、`house_3_68`；第二项还会检测到4号南北相邻段对 `house_1_66`/`house_2_71` 的5米保守包络违规。当前基线的段编号必须与实际输出一致；若此前任务 JSON 已变化，停止并重新做几何审查。

保留该 RED 输出用于执行报告，但此时不提交，避免分支停留在故意失败的 HEAD。

### Task 2: 外移4、5号巡逻路线并形成绿色提交

**Files:**
- Modify: `src/mix_nav/task_manager/launch/mission_down.json`
- Modify: `src/mix_nav/task_manager/test/test_mission_clearance.py`
- Test: `src/mix_nav/task_manager/test/test_mission_clearance.py`

- [ ] **Step 1: 把4号机航点移到房屋保守包络外**

保持4号机 `entry_waypoints` 不变，把 `waypoints` 完整替换为：

```json
[
  {"x": 25.0, "y": 12.0, "z": 3.5},
  {"x": 48.0, "y": 12.0, "z": 3.5},
  {"x": 48.0, "y": 39.0, "z": 3.5},
  {"x": 72.0, "y": 39.0, "z": 3.5},
  {"x": 72.0, "y": 41.0, "z": 3.5},
  {"x": 25.0, "y": 41.0, "z": 3.5}
]
```

`x=48` 位于 `house_1_66` 的5米保守西边界外，`y=39` 位于其5米保守北边界外；提前切换产生的斜向捷径即使进入外层3米切换余量，也不会进入内层2米机体安全包络。

- [ ] **Step 2: 把5号机航点移到两栋房的保守北侧**

保持5号机 `entry_waypoints` 不变，把 `waypoints` 完整替换为：

```json
[
  {"x": 122.0, "y": 41.0, "z": 3.5},
  {"x": 122.0, "y": 25.0, "z": 3.5},
  {"x": 78.0, "y": 25.0, "z": 3.5},
  {"x": 78.0, "y": 41.0, "z": 3.5}
]
```

`y=25` 同时位于 `house_2_71` 和 `house_3_68` 的5米保守北边界外。

- [ ] **Step 3: 验证 JSON 和两个 RED 测试转绿**

```bash
python3 -m json.tool src/mix_nav/task_manager/launch/mission_down.json >/dev/null
python3 -m unittest \
  src.mix_nav.task_manager.test.test_mission_clearance.MissionClearanceTest.test_all_complete_segments_clear_known_static_obstacles \
  src.mix_nav.task_manager.test.test_mission_clearance.MissionClearanceTest.test_routes_clear_switch_sensitive_buildings_with_arrival_allowance \
  -v
```

Expected: JSON 解析成功，两项测试均 `OK`。

- [ ] **Step 4: 运行整个 task_manager Python 回归**

```bash
python3 -m unittest discover \
  -s src/mix_nav/task_manager/test \
  -p 'test_*.py' -v
```

Expected: schema、6米硬上限、已知建筑净空、3米切换余量、跨机5米中心线净空和可见符号链接全部通过。

- [ ] **Step 5: 运行 Catkin task_manager 测试**

```bash
catkin_make run_tests_task_manager
catkin_test_results build/test_results/task_manager
```

Expected: `0 tests failed`。

- [ ] **Step 6: 提交测试与路线的完整红绿闭环**

```bash
git add \
  src/mix_nav/task_manager/test/test_mission_clearance.py \
  src/mix_nav/task_manager/launch/mission_down.json
git commit -m "fix: keep patrol routes clear of eastern houses"
```

### Task 3: 合规边界与补丁范围验证

**Files:**
- Verify: `src/competition_compliance/config/ownership.json`
- Verify: `scripts/verify_competition_clean.sh`

- [ ] **Step 1: 运行完整 competition-clean verifier**

```bash
bash scripts/verify_competition_clean.sh
```

Expected: 仓库 Python、Catkin、静态合规和构建后官方文件核验均通过；若受 Codex 网络接口隔离限制失败，必须在本机桌面终端用同一命令重跑并记录两次结果，不能把隔离失败写成项目通过。

- [ ] **Step 2: 核对 ownership 未发生变化**

```bash
git diff "$ROUTE_BASE_SHA"..HEAD -- src/competition_compliance/config/ownership.json
```

Expected: 无输出，因为两个修改文件原本就是已登记的队伍内容，无需改变所有权分类。

- [ ] **Step 3: 检查补丁格式和精确范围**

```bash
git diff --check "$ROUTE_BASE_SHA"..HEAD
git diff --name-only "$ROUTE_BASE_SHA"..HEAD
```

Expected: 第一条无输出；第二条只列出：

```text
src/mix_nav/task_manager/launch/mission_down.json
src/mix_nav/task_manager/test/test_mission_clearance.py
```

## 完成边界

本计划只交付固定任务的静态建筑净空合同。它不修复4、5号起飞区动态互撞，不证明 tracking 控制段、定位误差或真实动力学安全，也不替代 EGO 在线绕障。只有后续独立解决起飞区冲突并完成 Gazebo 全航程 contacts 验收后，才能声称4、5号完整飞行风险已闭合。
