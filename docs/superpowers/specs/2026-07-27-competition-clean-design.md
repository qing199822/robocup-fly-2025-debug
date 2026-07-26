# Competition-Clean 参赛候选版本设计

日期：2026-07-27

## 1. 背景

当前 `main` 分支用于社区共同调试，包含队伍代码、完整自定义
`typhoon_h480_zzufly` 模型以及项目内置的 `gazebo_ros_pkgs`。这些内容便于复现
现有仿真，但不能直接视为符合比赛提交边界。

本设计建立独立的 `competition-clean` 参赛候选版本。该版本遵循以下原则：

- PX4、XTDrone、Gazebo 和 ROS/Gazebo 集成组件保持官方原版。
- 只在官方教程和赛事规则允许的集成层进行 Launch、World、路径和参数配置。
- 无人机本体模型不能修改。
- 传感器类型和内部参数不能修改。
- 传感器安装位置和安装角度可以调整，但必须符合实际物理规律。
- 队伍算法通过公开 ROS/MAVROS 接口工作，不修改底层通信或飞控。

该版本是按现有规则整理的参赛候选版本，不代替当年赛事组委会或裁判的最终判定。

## 2. 目标与非目标

### 2.1 目标

- 建立与社区调试版隔离的 `competition-clean` 分支和工作区。
- 保留队伍自己的集群导航、任务调度、视觉识别和目标定位算法。
- 使用 XTDrone 官方 Typhoon H480 Realsense 模型作为只读生成基线。
- 只允许通过单独配置修改 Realsense 的六自由度安装位姿。
- 启动和测试过程中不写入 PX4、XTDrone 或 Gazebo 官方目录。
- 自动证明生成模型除允许的传感器位姿外没有其他差异。
- 使用两级自检，在保持合规性的同时避免增加搜索任务延迟。
- 验证通过后将 `competition-clean` 推送到公开 GitHub 仓库。

### 2.2 非目标

- 不在 clean 分支修复或增强 PX4、XTDrone、Gazebo 核心源码。
- 不把当前 `typhoon_h480_zzufly` 整机模型清洗后继续使用。
- 不把 Realsense 改接到活动云台。
- 不修改相机视场角、分辨率、焦距、更新率、裁剪范围或点云范围。
- 不将完整自检放入每次搜索任务的实时路径。
- 不承诺自动判断安装方式是否完全符合现实物理规律；该项仍需人工和裁判确认。

## 3. 规则边界

赛事规则截图说明：相机视场角、分辨率、焦距，以及激光雷达扫描范围等参数必须与
XTDrone 平台自带传感器保持一致，不能修改；传感器安装角可以自行设定，但需符合
实际物理规律；每次尝试前裁判会检查传感器参数与安装位置。

据此，本项目采用以下边界：

### 3.1 禁止修改

- PX4 飞控、SITL、MAVLink 和 Gazebo 插件源码。
- XTDrone 通信脚本及其核心框架。
- Gazebo 物理引擎和系统 `gazebo_ros_pkgs`。
- Typhoon H480 的机身、旋翼、电机、惯性、碰撞体、关节、动力学和控制通道。
- Realsense 内部的相机、深度、IMU、插件和话题参数。
- Realsense 与机身的固定连接对象，保持为官方 `base_link`。

### 3.2 允许修改

- 队伍自己的 ROS 节点和算法。
- 队伍自己的 Launch 和启动脚本。
- 官方教程明确要求的 World、模型路径和运行参数配置。
- Realsense 相对于 `base_link` 的 `x y z roll pitch yaw` 六个安装值。

## 4. 已发现的不合规内容

当前 `typhoon_h480_zzufly.sdf` 不仅改变传感器安装，还改变了机体朝向、云台连杆、
云台关节、IMU 姿态、旋翼轴、电机编号、旋转方向、MAVLink 电机通道、声呐位置和
相机参数。其深度相机 `horizontal_fov` 为 `2`，XTDrone 自带 Realsense 深度相机为
`1.047198`。因此整个 `typhoon_h480_zzufly` 目录只保留在 `main` 社区调试版，不进入
clean 分支。

当前 `src/gazebo_ros_pkgs` 是 ROS/Gazebo 集成项目的第三方副本，不是 Gazebo 11
物理引擎，也不是赛事组委会原创代码。该副本包含 Gazebo 与 ROS 通信层改动。
clean 分支删除它并使用系统 ROS Noetic 提供的官方包。

## 5. 官方模型基线

clean 版本以以下外部文件为只读基线：

- `XTDrone/sitl_config/models/typhoon_h480/typhoon_h480.sdf`
- `XTDrone/sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf`
- `XTDrone/sitl_config/models/realsense_camera/realsense_camera.sdf`

当前安装中，XTDrone 的 `typhoon_h480_realsense` 相对其 `typhoon_h480` 只包含：

- 模型名称变为 `typhoon_h480_realsense`。
- 增加一个 `model://realsense_camera` 引用。
- 增加该传感器的安装 `<pose>`。
- 增加连接到 `base_link` 的固定关节。

这使得 Realsense 安装位姿可以成为严格、单一且可审计的修改点。

当前已观察到的 SHA-256 值如下。实现时将其记录到版本化清单中，并且只从经确认的
原版环境更新清单：

| 文件 | SHA-256 |
| --- | --- |
| PX4 `typhoon_h480.sdf` | `4f3ae25801c704e1f9e640eaf1717e6a06a688256ad8f6ad5a0872a2843c4680` |
| XTDrone `typhoon_h480.sdf` | `1346f71a33130e3f5634b1513cc5598d1dc2693fdf30d13c2cf9dda2ef2cd29e` |
| XTDrone `typhoon_h480_realsense.sdf` | `3b056f3676e8f47b90421c5357eca8154e6686304855eb14467aa82bf60ddd46` |
| XTDrone `realsense_camera.sdf` | `0745c705ac3a90cf16529a9b49729d34f49ce7b457998a4d3cc3f2fb6aab921c` |
| PX4 `single_vehicle_spawn_xtd.launch` | `05bb251d1bebf28890cc03191a7fbbe0e121a5e2929a18b8968eb3d9ac071e7e` |
| XTDrone `multirotor_communication.py` | `64c13f6ad6de9181208cf584ac1b796d49d4f153935369b41e64a4b893a74d27` |

## 6. 分支与仓库边界

- `main` 继续作为社区调试版本，不删除现有调试能力。
- 实现工作在独立 Git 工作区中的 `competition-clean` 分支完成。
- 本地未提交的 `.vscode/settings.json` 不提交、不覆盖、不还原。
- clean 分支不包含 PX4、XTDrone 或 Gazebo 官方源码副本。
- clean 分支删除 `src/gazebo_ros_pkgs` 和 `typhoon_h480_zzufly`。
- clean 分支不安装或启动当前用于旧 Realsense 云台假设的 `src/gimbal`。
- 构建必需的第三方消息包或 Actor 插件只能原样保留，并记录来源、版本、许可证和
  校验值；如果不能证明为原样副本，则替换为经核验的上游版本或外部依赖。
- 每个保留包都在所有权清单中标记为“队伍代码”或“原样第三方依赖”。

## 7. 组件设计

### 7.1 官方依赖清单

版本化清单记录 PX4 1.11、XTDrone、Gazebo 11、ROS Noetic，以及比赛实际使用的
关键官方文件校验值。PX4 来自压缩包且 Git 元数据不完整，因此不能只依赖
`git status`，必须使用文件校验清单。

### 7.2 传感器安装配置

单独配置文件只包含 Realsense 的：

```text
x y z roll pitch yaw
```

默认值为 XTDrone 官方位姿 `0.09 0 -0.04 0 0 0`。六个值必须是有限数字。
角度使用弧度，坐标使用米。每次启动打印最终数值，供队伍和裁判检查。

### 7.3 合规模型生成器

生成器使用 XML 解析器读取外部官方 `typhoon_h480_realsense.sdf`，定位 URI 为
`model://realsense_camera` 的唯一 `<include>`，只替换它的直接子元素 `<pose>`。

生成器必须拒绝以下情况：

- 官方文件校验值不匹配。
- 找不到 Realsense `<include>` 或找到多个候选项。
- `<pose>` 不是六个有限数字。
- 固定关节的父链接不是 `base_link`。
- 传感器内部模型校验值不匹配。

输出写入 `/tmp/robocup-fly-competition-clean/<run-id>/models/`，不进入 Git，也不写入
PX4、XTDrone 或 Gazebo 目录。退出时删除本次临时目录。

### 7.4 模型差异验证器

验证器分别解析官方源文件和生成文件，将唯一允许的 Realsense `<pose>` 归一化为
同一值，再逐元素比较完整 XML 树。元素名称、属性、文本、子元素数量、顺序或其他
位姿出现任何差异都视为失败。

Realsense 内部模型不生成副本，也不允许参数覆盖。视场角、分辨率、更新率、裁剪
范围、点云范围、插件名称和话题名称直接来自官方文件。

### 7.5 队伍 Spawn Launch

队伍自己的 Launch 直接读取临时 SDF 并生成六架无人机。它复用原版 PX4 SITL、
MAVROS 和 XTDrone 通信节点，但不调用会把模型路径硬编码到 PX4 模型目录的写入流程，
也不创建符号链接。

### 7.6 上层视觉适配

- YOLO 继续订阅官方 Realsense 彩色图话题。
- 目标定位同时订阅深度图和 `CameraInfo`。
- 像素到三维坐标的计算使用 `CameraInfo.K` 中的 `fx fy cx cy`，不保留
  `fx=205.47` 等旧自定义相机常量。
- 相机坐标系使用消息头中的官方 frame。机身到传感器的静态 TF 由同一安装配置和
  XTDrone 官方传感器坐标轴约定共同生成；只有六个安装值可变，官方坐标轴约定不可
  覆盖，防止模型位姿和算法坐标变换不一致。
- Realsense 固定在 `base_link`，clean 启动流程不控制或查询 CGO3 云台来推断
  Realsense 姿态。

## 8. 启动流程

一键启动按以下顺序执行：

1. 运行快速自检。
2. 读取传感器安装配置并生成临时模型。
3. 对临时模型执行 XML 白名单差异验证。
4. 使用队伍 Launch 启动官方 Gazebo、六个 PX4 SITL 和 MAVROS 实例。
5. 启动原版 XTDrone 多机通信节点。
6. 等待六架无人机连接，并等待彩色图、深度图和 `CameraInfo` 第一条有效消息。
7. 启动队伍的导航、任务调度、YOLO 和目标定位节点。
8. 搜索任务运行期间不重复计算文件校验值。
9. 停止时终止本次子进程、删除临时模型并保留诊断日志。

## 9. 两级自检与性能

### 9.1 快速自检

快速自检在每次仿真启动时强制执行，不提供跳过开关。它只检查比赛实际使用的关键
文件、XML 结构、安装配置和 Launch 写入边界。

当前机器上，六个关键文件合计约 `172 KB`，SHA-256 校验耗时显示为 `0.00` 秒；
两个 SDF 的 XML 解析耗时也显示为 `0.00` 秒。实现验收要求快速自检总耗时不超过
`2` 秒。

快速自检在 Gazebo 启动前结束。搜索任务开始后不运行哈希或 XML 比较，因此不会
增加 YOLO、路径规划或控制循环延迟。

### 9.2 完整自检

完整自检在以下时机手动执行：

- 首次建立环境后。
- PX4、XTDrone、ROS 或 Gazebo 发生更新后。
- 修改 clean 分支依赖或配置后。
- 比赛前和公开发布前。

完整自检检查全部版本清单、第三方所有权与许可证、官方文件清单、构建、单元测试和
六机冒烟测试。它可以耗时几十秒或更长，但不进入日常搜索启动路径。

## 10. 错误处理

- 快速自检失败时，在启动任何仿真进程前退出。
- 错误信息使用中文说明具体文件、期望值、实际值和人工恢复方法。
- 工具不会自动下载、修补、覆盖或重新编译官方源码。
- 运行期六机连接或相机就绪超时后，停止本次启动的进程并返回失败。
- 清理只作用于本次运行记录的 PID 和明确的临时目录，不清理系统或其他仿真进程。
- 完整日志写入队伍工作区的日志目录，便于社区复现和裁判前自查。

## 11. 测试策略

### 11.1 静态合规测试

- 校验官方依赖清单。
- 验证 clean 分支不包含 `src/gazebo_ros_pkgs`。
- 验证 clean 分支不包含 `typhoon_h480_zzufly`。
- 验证 Launch 和脚本不会写入官方目录。
- 验证保留包全部出现在所有权与许可证清单中。
- 验证不存在对 PX4、XTDrone 通信或 Gazebo 核心源码的补丁。

### 11.2 模型生成测试

- 官方默认位姿生成成功。
- 合法自定义六自由度位姿只改变目标 `<pose>`。
- 非数字、无穷、NaN、字段缺失和字段过多均被拒绝。
- Realsense 节点缺失、重复或固定关节父链接变化均被拒绝。
- 机体、旋翼、电机、云台、碰撞体或传感器参数的任何差异均被拒绝。

### 11.3 上层算法测试

- 使用不同有效 `CameraInfo` 验证坐标计算不依赖固定焦距。
- 验证安装配置与发布 TF 一致。
- 验证彩色图、深度图和检测框的时间及坐标系处理。
- 验证原有集群导航与任务高度安全测试继续通过。

### 11.4 六机冒烟测试

- 六个 PX4 SITL 进程启动。
- 六个 MAVROS 实例连接成功。
- 六个原版 XTDrone 通信节点就绪。
- 六组彩色图、深度图和 `CameraInfo` 收到有效消息。
- 相机 TF 可查询，目标定位节点不报告坐标系错误。
- 队伍任务节点启动，停止后没有遗留本次进程或临时模型。

## 12. 验收标准

以下条件全部满足后，才可称为 `competition-clean` 参赛候选版本：

- clean 分支在隔离工作区构建成功。
- 快速自检在当前机器上不超过 `2` 秒。
- 完整自检和所有静态、单元及六机冒烟测试通过。
- PX4、XTDrone 和关键 Gazebo/ROS 组件与清单一致。
- 生成模型只有 Realsense 安装 `<pose>` 与官方基线不同。
- 相机参数完全来自 XTDrone 官方 Realsense 模型。
- 上层坐标计算使用 `CameraInfo`，不依赖旧自定义内参。
- 启动和停止过程不写入官方目录。
- `main` 社区调试版和本地 `.vscode/settings.json` 不受影响。
- 文档明确记录启动命令、版本、第三方来源、合规边界和已知限制。

## 13. 发布方式

实现与验证完成后，将独立 `competition-clean` 分支推送到公开远端：

```text
git@github.com:qing199822/robocup-fly-2025-debug.git
```

公开 README 必须明确：

- `main` 是社区调试版。
- `competition-clean` 是按现有规则整理的参赛候选版。
- 官方环境是外部依赖，不属于队伍源码。
- 最终合规性以当年赛事规则和裁判检查为准。
