# AI Agent 交接手册设计

## 目标

新增 `docs/AI_AGENT_HANDOFF.md`，为首次接触本仓库的 AI Agent 提供一份可独立阅读的中文交接手册。阅读者应能在不依赖聊天历史的情况下理解项目边界、定位队伍代码、配置环境、执行验证、启动六机仿真、安全停止，并继续修改和维护项目。

## 事实来源

交接手册只汇总当前仓库和已验证环境中的事实，不另行发明版本、路径或规则。权威来源为：

- `README.md`：项目入口、构建和快速启动。
- `docs/ENVIRONMENT.md`：固定版本、外部目录和安装顺序。
- `docs/COMPLIANCE.md`：官方文件边界、传感器位姿和哈希证据。
- `docs/THIRD_PARTY.md`：第三方来源及许可证边界。
- `docs/TROUBLESHOOTING.md`：已知故障和运行诊断。
- `1.sh`、`scripts/verify_competition_clean.sh`、`scripts/smoke_competition_clean.sh`：实际启动、完整验证和六机冒烟合同。
- `src/competition_compliance/config/ownership.json`：文件所有权分类。

28 个官方文件哈希不在交接手册中重复维护。手册链接 `docs/COMPLIANCE.md` 和原始 manifest，避免多处副本发生漂移。

## 文档结构

`docs/AI_AGENT_HANDOFF.md` 按下面顺序组织：

1. 项目目标、当前分支、公开远端和最近验证状态。
2. 不可突破的比赛合规边界，以及允许修改的队伍代码范围。
3. 仓库目录、主要 ROS 包、节点和控制/感知数据流。
4. 固定软件版本、外部依赖目录和必需环境变量。
5. 首次构建、日常快速启动、双终端 smoke、Ctrl-C 停止和残留检查命令。
6. 快速测试、完整测试、Catkin/合规验证和提交前检查命令。
7. 相机、MAVROS、YOLO、Gazebo、任务路线和退出清理的排查顺序。
8. AI Agent 维护规则，包括先读文件、测试优先、修改范围、进程安全和 Git 操作。
9. 已知风险、日志/证据位置，以及向下一位维护者报告结果的模板。

## 命令约束

- 命令以仓库默认布局 `~/robocup_fly/2025_ZZU_FLY` 为主，并同时说明当前独立工作区路径可能不同。
- 环境变量使用仓库已经验证的六个路径变量，不在命令中写入新的依赖位置。
- 启动命令固定为 `bash 1.sh 6 mission_down.json`。
- 完整验证固定通过 `scripts/verify_competition_clean.sh`，运行态验证通过 `scripts/smoke_competition_clean.sh`。
- 停止方式固定为启动终端中的 `Ctrl-C`。不得建议 `killall`、宽泛 `pkill` 或按进程名批量终止。
- Git 发布只描述非强制推送 `competition-clean`；不得移动公开仓库的 `main`。

## 安全与维护规则

- PX4、XTDrone、Gazebo、官方模型和外部 Python 环境均视为只读输入。
- 允许修改队伍 ROS 包、队伍启动/验证脚本、任务配置和唯一允许的 Realsense 安装位姿。
- 无人机模型、传感器光学/量程参数、官方通信实现不得为适配队伍代码而修改。
- 修复行为问题时先添加失败回归测试，再做最小修改，并运行与风险相称的验证。
- 进程清理必须基于本次运行登记的 PID/进程组和进程身份；不得误伤其他 ROS 会话。
- 未经用户明确授权，不 force-push、不合并或移动 `main`，不删除用户改动。

## 验收条件

- 文件可脱离聊天历史独立使用，面向不了解项目的新 AI Agent。
- 所有命令、版本、路径变量和修改边界与当前仓库一致。
- 包含完整的构建、启动、smoke、停止、测试和 Git 检查流程。
- 清楚区分队伍代码、逐字节核验的第三方副本和外部官方只读依赖。
- 不包含 `TBD`、`TODO`、`FIXME`、过时自定义机型/兼容包引用或修改官方源码的建议。
- Markdown 路径有效，`git diff --check` 和交接文档内容检查通过。

