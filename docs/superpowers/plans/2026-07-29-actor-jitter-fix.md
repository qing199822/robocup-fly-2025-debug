# Gazebo 人物模型抖动修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 启动时从已校验的官方 `robocup.world` 生成仅禁用人物碰撞插件的临时场景，使 6 个人物在 Gazebo 11 中保持稳定，同时不修改任何官方或第三方输入。

**Architecture:** 在 `competition_compliance` 包中新增纯 XML 变换模块和薄命令行入口。`1.sh` 在官方 manifest 校验及临时无人机模型生成成功后生成同一私有运行目录下的临时 world，并通过已有 launch 的 `world` 参数启动 Gazebo；现有目录级清理负责同时删除两个临时产物。

**Tech Stack:** Python 3.8、`xml.etree.ElementTree`、ROS Noetic/Catkin、Bash、Gazebo Classic 11、`unittest`/nose。

---

## 文件结构

- 新建 `src/competition_compliance/src/competition_compliance/world.py`：验证官方场景结构，只移除 actor 直属的指定碰撞插件，并验证输出差异。
- 新建 `src/competition_compliance/scripts/prepare_world.py`：解析路径参数、禁止输出到官方目录、调用 world 生成核心并统一报告合规错误。
- 新建 `src/competition_compliance/test/test_world.py`：覆盖精确变换、输入不变、结构异常、误删防护和写入失败清理。
- 修改 `src/competition_compliance/CMakeLists.txt`：安装新脚本并注册新测试。
- 修改 `src/competition_compliance/test/test_launch_contract.py`：验证脚本安装和 launch 的 world 参数契约。
- 修改 `tests/test_one_click_launch.py`：验证一键启动生成、传递并清理临时 world。
- 修改 `1.sh`：声明 world 准备器和临时输出，执行生成器，并传入 launch。
- 修改 `src/competition_compliance/scripts/verify_full.py`：把新的只读输入引用和生成命令加入官方目录访问白名单。
- 保持 `robocup_zzufly.launch` 的现有 `world` 参数及 `world_name` 转发不变；只用测试固定该接口，不做无意义格式改动。

### Task 1: 严格的临时 world 生成核心

**Files:**
- Create: `src/competition_compliance/src/competition_compliance/world.py`
- Create: `src/competition_compliance/test/test_world.py`

- [ ] **Step 1: 写入能复现冲突配置的失败测试**

测试夹具构造 6 个 actor，每个 actor 各有一个碰撞插件和一个 ROS 控制插件，并在 world 直属位置放一个同名插件验证不会误删：

```python
def make_world(path, actor_count=6):
    root = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(root, "world", {"name": "default"})
    ET.SubElement(
        world, "plugin", {"filename": "libActorCollisionsPlugin.so"}
    )
    for index in range(actor_count):
        actor = ET.SubElement(world, "actor", {"name": "actor_{}".format(index)})
        ET.SubElement(
            actor, "plugin", {"filename": "libActorCollisionsPlugin.so"}
        )
        ros = ET.SubElement(
            actor, "plugin", {"filename": "libros_actor_cmd_pose_plugin.so"}
        )
        ET.SubElement(ros, "init_pose").text = "{} 0 1.25 1.57 0 0".format(index)
        ET.SubElement(actor, "skin").text = "walker"
        ET.SubElement(actor, "animation", {"name": "walking"})
    ET.ElementTree(root).write(str(path), encoding="utf-8", xml_declaration=True)
```

核心断言：

```python
def test_generation_removes_only_actor_collision_plugins(self):
    with tempfile.TemporaryDirectory() as directory:
        source = pathlib.Path(directory) / "official.world"
        output = pathlib.Path(directory) / "generated.world"
        make_world(source)
        before = source.read_bytes()

        generate_world(source, output)

        root = ET.parse(str(output)).getroot()
        self.assertEqual(before, source.read_bytes())
        self.assertEqual(0, len(root.findall(
            "./world/actor/plugin[@filename='libActorCollisionsPlugin.so']"
        )))
        self.assertEqual(1, len(root.findall(
            "./world/plugin[@filename='libActorCollisionsPlugin.so']"
        )))
        self.assertEqual(6, len(root.findall(
            "./world/actor/plugin[@filename='libros_actor_cmd_pose_plugin.so']"
        )))
```

另加独立用例验证 actor 数量不是 6、任一 actor 缺少或重复两类插件、输出已存在、XML 损坏时抛出 `ComplianceError` 且不留下新输出。

- [ ] **Step 2: 运行新测试并确认因模块不存在而失败**

Run:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/competition_compliance/test/test_world.py -v
```

Expected: FAIL，提示 `No module named 'competition_compliance.world'`。

- [ ] **Step 3: 实现最小的严格 XML 变换**

`world.py` 定义：

```python
ACTOR_COUNT = 6
COLLISION_PLUGIN = "libActorCollisionsPlugin.so"
ROS_PLUGIN = "libros_actor_cmd_pose_plugin.so"


def _parse_xml(path):
    try:
        return ET.parse(str(pathlib.Path(path)))
    except OSError as error:
        raise ComplianceError("无法读取 Gazebo world {}：{}".format(path, error)) from error
    except ET.ParseError as error:
        raise ComplianceError("Gazebo world XML 格式错误 {}：{}".format(path, error)) from error


def _actor_plugins(actor, filename):
    return [
        plugin for plugin in actor.findall("plugin")
        if plugin.get("filename") == filename
    ]


def _validated_actors(root):
    worlds = root.findall("./world")
    if len(worlds) != 1:
        raise ComplianceError("Gazebo world 必须恰好包含一个 world 元素")
    actors = worlds[0].findall("actor")
    if len(actors) != ACTOR_COUNT:
        raise ComplianceError("Gazebo world 必须恰好包含 6 个 actor")
    names = [actor.get("name") for actor in actors]
    if any(not name for name in names) or len(set(names)) != ACTOR_COUNT:
        raise ComplianceError("Gazebo actor 名称必须非空且唯一")
    for actor in actors:
        if len(_actor_plugins(actor, COLLISION_PLUGIN)) != 1:
            raise ComplianceError("每个 actor 必须恰好包含一个人物碰撞插件")
        if len(_actor_plugins(actor, ROS_PLUGIN)) != 1:
            raise ComplianceError("每个 actor 必须恰好包含一个人物 ROS 控制插件")
    return actors
```

`generate_world()` 必须拒绝覆盖输出，用 `xb` 写入；写后调用 `assert_only_actor_collisions_removed()`，该函数同时解析输入和输出，将输入树中 6 个目标插件移除后做规范化结构比较。任何写入后验证失败都删除新输出并重新抛出 `ComplianceError`。

- [ ] **Step 4: 运行核心测试并确认通过**

Run:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/competition_compliance/test/test_world.py -v
```

Expected: 所有 `WorldGenerationTest` 用例 PASS。

- [ ] **Step 5: 提交核心与测试**

```bash
git add \
  src/competition_compliance/src/competition_compliance/world.py \
  src/competition_compliance/test/test_world.py
git commit -m "fix: generate stable actor world copy"
```

### Task 2: 命令行入口与 Catkin 契约

**Files:**
- Create: `src/competition_compliance/scripts/prepare_world.py`
- Modify: `src/competition_compliance/CMakeLists.txt`
- Modify: `src/competition_compliance/test/test_launch_contract.py`

- [ ] **Step 1: 写入脚本安装契约的失败测试**

在 `test_launch_contract.py` 增加常量 `COMPLIANCE_CMAKE`，并增加：

```python
def test_prepare_world_is_installed_and_tested(self):
    cmake = COMPLIANCE_CMAKE.read_text(encoding="utf-8")
    self.assertIn("scripts/prepare_world.py", cmake)
    self.assertIn("catkin_add_nosetests(test/test_world.py)", cmake)
```

- [ ] **Step 2: 运行测试并确认缺少安装项而失败**

Run:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/competition_compliance/test/test_launch_contract.py -v
```

Expected: FAIL，指出 `scripts/prepare_world.py` 不在 CMake 内容中。

- [ ] **Step 3: 增加薄命令行入口和 CMake 条目**

`prepare_world.py` 使用以下接口：

```python
parser.add_argument("--px4-dir", required=True, type=pathlib.Path)
parser.add_argument("--xtdrone-dir", required=True, type=pathlib.Path)
parser.add_argument("--input", required=True, type=pathlib.Path)
parser.add_argument("--output", required=True, type=pathlib.Path)
```

入口先调用：

```python
output = validate_output_path(
    args.output,
    {"PX4_DIR": args.px4_dir, "XTDRONE_DIR": args.xtdrone_dir},
)
generate_world(args.input, output)
print(output)
```

捕获 `ComplianceError` 时沿用 `prepare_model.py` 的中文错误格式并返回 2。将该脚本加入 `catkin_install_python(PROGRAMS ...)`，将 `test/test_world.py` 加入 `CATKIN_ENABLE_TESTING` 块。

- [ ] **Step 4: 运行 world 与 launch 契约测试**

Run:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/competition_compliance/test/test_world.py -v
python3 src/competition_compliance/test/test_launch_contract.py -v
```

Expected: 两组测试 PASS。

- [ ] **Step 5: 提交命令行与构建接线**

```bash
git add \
  src/competition_compliance/scripts/prepare_world.py \
  src/competition_compliance/CMakeLists.txt \
  src/competition_compliance/test/test_launch_contract.py
git commit -m "feat: install stable world generator"
```

### Task 3: 一键启动生成并使用临时 world

**Files:**
- Modify: `tests/test_one_click_launch.py`
- Modify: `1.sh`
- Modify: `src/competition_compliance/scripts/verify_full.py`

- [ ] **Step 1: 写一键启动失败测试**

扩展 `LauncherHarness._create_required_files()`，创建空的 `src/competition_compliance/scripts/prepare_world.py`。让 `preflight-python` 继续从所有参数中读取 `--output`，因此模型与 world 两次调用都会生成各自文件。

将静态契约改为：

```python
def test_generated_world_is_private_run_output_and_passed_to_launch(self):
    script = self.script
    self.assertIn('GENERATED_WORLD=""', script)
    self.assertIn('GENERATED_WORLD="$RUN_TMP_DIR/robocup.world"', script)
    self.assertIn('"$COMPLIANCE_PYTHON" "$PREPARE_WORLD"', script)
    self.assertIn('--input "$OFFICIAL_WORLD"', script)
    self.assertIn('--output "$GENERATED_WORLD"', script)
    self.assertIn('world:="$GENERATED_WORLD"', script)
```

修改 roslaunch stub，把仿真启动参数写入 `$STATE_DIR/simulation_args`，增加行为测试确认成功启动时同时包含：

```text
model_file:=<run_tmp>/typhoon_h480_realsense.sdf
world:=<run_tmp>/robocup.world
```

并确认启动完成后 `run_tmp` 目录不存在，证明沿用现有清理。

- [ ] **Step 2: 运行一键启动测试并确认缺少变量和参数而失败**

Run:

```bash
python3 -m unittest tests.test_one_click_launch -v
```

Expected: 新增的临时 world 契约用例 FAIL；原有用例继续 PASS。

- [ ] **Step 3: 对 `1.sh` 做最小接线**

在现有变量旁增加：

```bash
PREPARE_WORLD="$COMPLIANCE_PACKAGE_DIR/scripts/prepare_world.py"
OFFICIAL_WORLD="$PX4_DIR/Tools/sitl_gazebo/worlds/robocup.world"
GENERATED_WORLD=""
```

在路径规范化后同步更新并导出 `OFFICIAL_WORLD`。配置检查使用 `require_file "$PREPARE_WORLD" "临时 Gazebo world 生成器"` 和 `require_file "$OFFICIAL_WORLD" "RoboCup Gazebo 世界"`。

创建 `RUN_TMP_DIR` 后增加：

```bash
GENERATED_WORLD="$RUN_TMP_DIR/robocup.world"
```

保持 `prepare_model.py` 作为唯一 manifest/版本校验入口；其成功后执行：

```bash
if ! "$COMPLIANCE_PYTHON" "$PREPARE_WORLD" \
    --px4-dir "$PX4_DIR" \
    --xtdrone-dir "$XTDRONE_DIR" \
    --input "$OFFICIAL_WORLD" \
    --output "$GENERATED_WORLD" >/dev/null; then
    echo "错误：临时 Gazebo world 生成失败。" >&2
    return 1
fi
```

仿真命令改为：

```bash
start_owned_group "六机仿真" roslaunch "$SIMULATION_LAUNCH" \
    model_file:="$GENERATED_MODEL" world:="$GENERATED_WORLD" || return 1
```

- [ ] **Step 4: 更新合规扫描器的精确白名单**

在 `_ALLOWED_OFFICIAL_SHELL_COMMANDS` 中增加 `OFFICIAL_WORLD` 的声明、规范化后的赋值、`require_file` 行及完整的 `prepare_world.py` 调用行。不得扩大 `_ALLOWED_OFFICIAL_SHELL_COMMANDS` 之外的命令集合，也不得移除 manifest 对原 world 哈希的校验。

- [ ] **Step 5: 运行一键启动与合规单测**

Run:

```bash
python3 -m unittest tests.test_one_click_launch -v
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/competition_compliance/test/test_manifest.py -v
```

Expected: 两组测试全部 PASS；测试结束后没有 `robocup-fly-competition-clean.*` 测试临时目录残留。

- [ ] **Step 6: 提交启动链路**

```bash
git add 1.sh tests/test_one_click_launch.py \
  src/competition_compliance/scripts/verify_full.py
git commit -m "fix: launch Gazebo with stable actor world"
```

### Task 4: 全量验证和 Gazebo 运行复测

**Files:**
- Modify only if verification exposes a task-scoped defect.

- [ ] **Step 1: 运行快速静态检查与 Python 回归**

Run:

```bash
bash -n 1.sh scripts/*.sh src/yolo/*.sh
python3 -m py_compile \
  src/competition_compliance/scripts/prepare_world.py \
  src/competition_compliance/src/competition_compliance/world.py
python3 -m unittest discover -s tests -p 'test_*.py'
git diff --check
```

Expected: Bash/Python 语法检查退出 0，仓库 Python 回归全部 PASS，`git diff --check` 无输出。

- [ ] **Step 2: 运行 competition-clean 完整验证**

Run:

```bash
export PX4_DIR=/home/wangtao/robocup_fly/PX4_Firmware
export XTDRONE_DIR=/home/wangtao/robocup_fly/XTDrone
export GAZEBO_MODELS_DIR=/home/wangtao/robocup_fly/gazebo_models
export XTDRONE_PYTHONPATH=/home/wangtao/robocup_fly/.xtdrone-python
bash scripts/verify_competition_clean.sh
```

Expected: 结尾为 `完整验证通过：静态与构建后合规证据均已生成。`，Catkin 测试 0 error、0 failure，官方 manifest 在构建后仍通过。

- [ ] **Step 3: 运行最小 Gazebo actor 稳定性验证**

用新脚本从官方 world 生成 `/tmp` 测试副本，启动 `gazebo_ros/empty_world.launch`（无无人机），确认 6 个 `/actor_N/cmd_motion` 均无发布者。对 `actor_1` 连续调用 `/gazebo/get_model_state` 至少 30 次。

Expected: 每次位置均为 `x=70.0, y=22.0`，不再出现 `x=0.0, y=0.0`；测试退出后删除这次明确创建的 `/tmp` 文件并确认 Gazebo/ROS 测试进程已退出。

- [ ] **Step 4: 运行真实六机 smoke**

终端 A：

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
bash 1.sh 6 mission_down.json
```

终端 B，在六路相机就绪后：

```bash
bash scripts/smoke_competition_clean.sh
```

Expected: 最后一行为 `PASS competition-clean six-vehicle smoke`；Gazebo 中人物不跳回原点；相机和目标识别仍能看到人物。回到终端 A 按一次 `Ctrl-C`，预期退出码 130。

- [ ] **Step 5: 核验外部目录与工作树**

Run:

```bash
git -C "$XTDRONE_DIR" status --short
sha256sum "$PX4_DIR/Tools/sitl_gazebo/worlds/robocup.world"
find /tmp -maxdepth 1 -type d -name 'robocup-fly-competition-clean.*' -print
git status --short --branch
```

Expected: XTDrone 状态为空；官方 world 哈希仍为 `b17daad2b9662760aba6defbd1637214e6d4832e3828ec13ca342f544c6e0b98`；本次运行没有新增临时目录；工作树只保留进入任务前已有的 `CLAUDE.md` 和 `src/competition_compliance/test/test_ownership.py` 改动。

- [ ] **Step 6: 确认任务提交完整**

Run:

```bash
git log --oneline --decorate -5
git status --short --branch
```

Expected: 计划中的三个实现提交均存在；没有未提交的任务文件，工作树仅保留进入任务前已有的 `CLAUDE.md` 和 `src/competition_compliance/test/test_ownership.py` 改动。不创建空提交。
