# AI Agent Handoff Guide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create one self-contained Chinese handoff guide that lets a new AI Agent safely understand, run, verify, modify, and maintain the `competition-clean` project without chat history.

**Architecture:** Add one documentation file, `docs/AI_AGENT_HANDOFF.md`, that summarizes the authoritative existing docs and executable scripts without duplicating the official hash table. Validate it with shell content checks and existing repository tests so commands and compliance boundaries cannot silently drift.

**Tech Stack:** Markdown, Bash command examples, ROS Noetic, Catkin, PX4 1.11, XTDrone, Gazebo 11, Python `unittest`, Git.

---

### Task 1: Establish the handoff content contract

**Files:**
- Create: `docs/AI_AGENT_HANDOFF.md`
- Reference: `README.md`
- Reference: `docs/ENVIRONMENT.md`
- Reference: `docs/COMPLIANCE.md`
- Reference: `docs/THIRD_PARTY.md`
- Reference: `docs/TROUBLESHOOTING.md`
- Reference: `src/competition_compliance/config/ownership.json`
- Reference: `1.sh`
- Reference: `scripts/verify_competition_clean.sh`
- Reference: `scripts/smoke_competition_clean.sh`

- [ ] **Step 1: Verify the target file does not already exist**

Run:

```bash
test ! -e docs/AI_AGENT_HANDOFF.md
```

Expected: exit `0`. If the file exists, inspect and preserve its useful content instead of overwriting it blindly.

- [ ] **Step 2: Record the required section contract**

The final file must use these exact top-level sections so another Agent can scan it reliably:

```text
# AI Agent 项目交接手册
## 一分钟了解项目
## 当前可信状态
## 不可突破的修改边界
## 仓库结构与模块职责
## 运行时数据流
## 环境、目录与变量
## 程序使用命令
## 修改和验证工作流
## 常见故障排查
## Git 与交付规则
## 已知风险和维护重点
## 新 Agent 首次接手清单
## 交接结果模板
```

Expected: every section has concrete content; none is an empty placeholder.

### Task 2: Write and validate the handoff guide

**Files:**
- Create: `docs/AI_AGENT_HANDOFF.md`
- Modify: `README.md`

- [ ] **Step 1: Write the project summary and compliance boundary**

Document these facts explicitly:

```text
- Branch: competition-clean; public/main must not be moved.
- External read-only inputs: PX4 1.11, XTDrone 8e88116, Gazebo 11, official models, external Python environments.
- Allowed code: team ROS packages, team launch/verification scripts, mission configuration, and sensor_mount.yaml pose.
- Forbidden adaptation: modifying official flight control, XTDrone communication, aircraft geometry, Realsense optical/range parameters, or official Gazebo code.
- Third-party byte-verified copies and LicenseRef-Team-Code remain governed by COMPLIANCE.md and THIRD_PARTY.md.
```

Explain that `docs/COMPLIANCE.md` and `official_manifest.json` are the sole official hash sources; do not copy the 28-row hash table.

- [ ] **Step 2: Document modules and runtime data flow**

Map the team packages to responsibilities: `competition_compliance`, `pose_init`, `mix_nav/fly`, `mix_nav/task_manager`, `mix_nav/simple_navigator`, `look_up`, `tracking`, `transform_tree`, and `yolo`. Include this runtime sequence:

```text
1.sh
  -> fast compliance preflight and private generated model
  -> Gazebo + six PX4 SITL + six MAVROS
  -> six XTDrone communication nodes
  -> six Realsense RGB/depth/CameraInfo readiness checks
  -> six YOLO workers + six coordinate estimators
  -> down_resume.launch and team mission/tracking nodes
```

Also record the command path from tracking through the mux:

```text
/typhoon_h480_N/tracking_node
  -> /typhoon_h480_N/mux_inputs/external/pose_cmd
  -> /typhoon_h480_N/pose_cmd_mux
  -> /xtdrone/typhoon_h480_N/cmd_vel_flu
```

- [ ] **Step 3: Document exact environment and usage commands**

Include complete copy-ready blocks for:

```bash
cd ~/robocup_fly/2025_ZZU_FLY
export PX4_DIR=${PX4_DIR:-$HOME/robocup_fly/PX4_Firmware}
export XTDRONE_DIR=${XTDRONE_DIR:-$HOME/robocup_fly/XTDrone}
export GAZEBO_MODELS_DIR=${GAZEBO_MODELS_DIR:-$HOME/robocup_fly/gazebo_models}
export XTDRONE_PYTHONPATH=${XTDRONE_PYTHONPATH:-$HOME/robocup_fly/.xtdrone-python}
export YOLO_PYTHON=${YOLO_PYTHON:-$HOME/robocup_fly/.venv-yolo/bin/python}
export YOLO_CONFIG_DIR=${YOLO_CONFIG_DIR:-$HOME/robocup_fly/.ultralytics}
source /opt/ros/noetic/setup.bash
```

```bash
catkin_init_workspace src
catkin_make -DCMAKE_BUILD_TYPE=Release
bash scripts/build_xtdrone_actor_collisions.sh
bash 1.sh 6 mission_down.json
bash scripts/smoke_competition_clean.sh
bash scripts/verify_competition_clean.sh
```

State that smoke runs in a second terminal after readiness, and the launcher must be stopped with `Ctrl-C` in its own terminal.

- [ ] **Step 4: Document maintenance, diagnostics, and Git rules**

Include:

```text
- Read ownership.json and the closest package README before editing.
- Diagnose before fixing; add a failing regression before behavioral changes.
- Run focused tests, then `python3 -m unittest discover -s tests -v`.
- Run the full verifier after environment, launch, model, or cross-package changes.
- Never use killall, broad pkill, process-name ownership, force-push, or edits to official trees.
- Preserve unrelated user changes and `.vscode/settings.json`.
- Commit only intended files; push competition-clean without moving public/main.
```

List diagnostic commands for ROS nodes/topics, camera messages, logs, residual processes, temporary directories, and XTDrone status. Explain expected output instead of presenting commands without interpretation.

- [ ] **Step 5: Link the handoff from README**

Add one sentence near the repository boundary or troubleshooting entry:

```markdown
交给新的 AI Agent 维护前，请先让它完整阅读 [docs/AI_AGENT_HANDOFF.md](docs/AI_AGENT_HANDOFF.md)。
```

- [ ] **Step 6: Validate required content and forbidden guidance**

Run:

```bash
for heading in \
  '一分钟了解项目' '当前可信状态' '不可突破的修改边界' \
  '仓库结构与模块职责' '运行时数据流' '环境、目录与变量' \
  '程序使用命令' '修改和验证工作流' '常见故障排查' \
  'Git 与交付规则' '已知风险和维护重点' '新 Agent 首次接手清单' \
  '交接结果模板'; do
  grep -F "## $heading" docs/AI_AGENT_HANDOFF.md >/dev/null || exit 1
done

rg -n 'bash 1\.sh 6 mission_down\.json|verify_competition_clean\.sh|smoke_competition_clean\.sh|sensor_mount\.yaml|ownership\.json' docs/AI_AGENT_HANDOFF.md
! rg -n 'killall|pkill -f|git push --force|修改 PX4|修改 XTDrone' docs/AI_AGENT_HANDOFF.md
! rg -n 'TBD|TODO|FIXME|待定' docs/AI_AGENT_HANDOFF.md
git diff --check
```

Expected: all headings exist; required command/boundary references match; forbidden commands and placeholders have no matches; diff check exits `0`.

- [ ] **Step 7: Run repository documentation-adjacent tests**

Run:

```bash
python3 -m unittest \
  tests.test_competition_boundary \
  tests.test_verification_scripts \
  tests.test_one_click_launch.OneClickLaunchTest -v
```

Expected: all selected tests pass with exit `0`.

- [ ] **Step 8: Commit the handoff guide**

```bash
git add README.md docs/AI_AGENT_HANDOFF.md
git commit -m "docs: add AI agent maintenance handoff"
git status --short --branch
```

Expected: commit succeeds and the worktree is clean. The branch is ahead of `public/competition-clean` until the user authorizes or requests another push.

