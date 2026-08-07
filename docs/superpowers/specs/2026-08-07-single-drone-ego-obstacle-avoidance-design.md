# 单机 EGO 障碍绕行闭环设计

更新日期：2026-08-07

状态：用户已确认，等待实施计划

## 1. 目标

在不修改 PX4、XTDrone、Gazebo、EGO-Planner-Swarm 核心源码和官方无人机模型的前提下，让 `typhoon_h480_0` 使用现有固定前视 Realsense 与 `global_odom`，完成以下单机闭环：

```text
接收高层目标 -> 在线感知障碍 -> 生成安全局部目标
-> EGO 三维规划 -> 队伍适配器输出控制 -> 绕障到达
```

本阶段只证明单机能够在独立场景中发现静态障碍、绕过障碍并安全到达目标。通过后才能设计双机轨迹避碰和六机正式接入。

## 2. 当前基线

已经存在并通过单机运行检查的队伍模块：

- `local_mapping`：同步深度、相机参数和 `global_odom`；
- 人物深度与静态地图分流；
- `FREE/OCCUPIED/UNKNOWN` 三态体素地图；
- 静态点云、人物短期动态点云和过滤后规划深度；
- 地图健康、前方净空和已知空闲侧前沿目标；
- 数据超时、同步失败和 ROS 时钟倒退时失效关闭。

当前仍然缺少：

- 固定版本的外部 EGO-Planner-Swarm 运行环境；
- 队伍自有 `ego_adapter`；
- 搜索导航状态机和局部目标桥接；
- EGO 输出轨迹的二次安全校验；
- EGO 导航模式的互斥 Launch；
- 单机 Gazebo 绕障验收。

外部 EGO 目录当前不存在，不能把文档设计误报为已经接入。

## 3. 方案选择

采用以下组合：

1. EGO-Planner-Swarm 作为唯一三维局部轨迹规划内核；
2. 现有 `local_mapping` 作为队伍统一感知和健康事实来源；
3. 队伍自有 `ego_adapter` 负责接口、坐标、轨迹和控制安全；
4. 前向深度制动作为独立硬保护，不替代 EGO 路径规划；
5. `layered_2d` 保留为以后人工选择的性能备用模式；
6. `static_patrol` 只作为紧急降级模式，不与 EGO 同时发布导航命令。

不采用只有“看到墙就转向”的反应式导航作为主方案，因为它不能可靠处理墙角、凹形障碍和死路。

## 4. 范围边界

### 4.1 本阶段包含

- 只运行 `typhoon_h480_0`；
- 使用现有前视深度、CameraInfo、人物框和 `global_odom`；
- 接收一个队伍高层目标；
- 只在已确认空闲空间内生成短距离局部目标；
- EGO 绕过静态障碍；
- EGO 无解、深度失效、位姿失效或控制权不明时悬停；
- tracking 接管时使旧 EGO 任务和轨迹失效；
- tracking 退出后从当前位置重新规划；
- 保持现有 MUX 和 `safety_filter` 最终控制链。

### 4.2 本阶段不包含

- 六机同时运行 EGO；
- 中央前沿竞价和六机区域动态分配；
- 多高度二维地图；
- 运行中自动切换导航模式；
- 使用 Gazebo 真值障碍坐标导航；
- 修改官方 World、人物、无人机模型或第三方规划器源码；
- 把当前固定巡逻描述为已经具备未知地图自主探索。

## 5. 总体架构

```text
高层巡逻目标 / 人工测试目标
              |
              v
      search_coordinator
       |             |
       |             +---- 状态、任务代次、取消与重规划
       v
已知空闲侧短距离目标
              |
              v
      EGO-Planner-Swarm <---- planner_depth + global_odom
              |
      PositionCommand / 轨迹状态
              |
              v
         ego_adapter <---- map health + local clearance + MUX selected
              |
  /typhoon_h480_0/mux_inputs/navigator/cmd_vel
              |
现有 tracking external ---- pose_cmd_mux
                              |
                        safety_filter
                              |
       /xtdrone/typhoon_h480_0/cmd_vel_flu
```

EGO 回答“怎样安全到达局部目标”；`search_coordinator` 回答“现在应该去哪个局部目标”；tracking 回答“锁定人物后怎样跟随”。三者不得争用职责。

### 5.1 ROS 接口契约

第一阶段固定以下队伍侧接口，外部 EGO 的实际话题通过 Launch remap 接入：

| 方向 | 话题或服务 | 类型 | 用途 |
| --- | --- | --- | --- |
| 输入 | `/typhoon_h480_0/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 现有任务管理器发布的高层目标 |
| 输入 | `/typhoon_h480_0/local_mapping/planner_depth` | `sensor_msgs/Image` | 过滤后实时障碍深度 |
| 输入 | `/typhoon_h480_0/global_odom` | `nav_msgs/Odometry` | EGO 和适配器共用位姿 |
| 输入 | `/typhoon_h480_0/local_mapping/health` | `search_msgs/PerceptionHealth` | 感知和地图健康门 |
| 输入 | `/typhoon_h480_0/local_mapping/clearance` | `search_msgs/LocalClearance` | 前向硬制动依据 |
| 输入 | `/typhoon_h480_0/local_mapping/frontier_goal` | `geometry_msgs/PoseStamped` | 直达目标未知时的已知空闲观察点 |
| 新服务 | `/typhoon_h480_0/local_mapping/validate_trajectory` | `search_msgs/ValidateTrajectory` | 用当前体素地图验证完整轨迹扫掠体 |
| 输出 | `/typhoon_h480_0/ego/goal` | `geometry_msgs/PoseStamped` | 发给本机 EGO 的短距离目标 |
| 输入 | `/typhoon_h480_0/ego/position_cmd` | `quadrotor_msgs/PositionCommand` | EGO 生成的局部运动命令 |
| 输出 | `/typhoon_h480_0/mux_inputs/navigator/cmd_vel` | `geometry_msgs/Twist` | 现有 MUX navigator 输入 |
| 输出 | `/typhoon_h480_0/ego_adapter/status` | `std_msgs/String` | 当前状态和失效原因 |

`ValidateTrajectory` 请求包含轨迹时间戳、任务代次和按时间排序的中心点采样；服务端使用统一配置的机体水平半径、垂直半径和安全余量检查完整扫掠体。响应至少包含 `valid`、当前地图时间、最小净空和结构化故障码。地图过期、任一采样为 unknown、任一采样占据或请求任务代次过期时都返回失败。

轨迹验证必须查询 `local_mapping` 当前持有的同一份 `VoxelMap`，不能让 `ego_adapter` 复制第二份语义不同的占据地图。

## 6. 外部依赖和源码边界

EGO-Planner-Swarm 固定到一个经过构建和运行验证的上游提交，安装在比赛仓库外的只读目录中。版本、来源、提交号和许可证写入 `docs/THIRD_PARTY.md`。

允许通过队伍 Launch 完成：

- include 外部 Launch；
- ROS namespace；
- topic remap；
- 参数覆盖；
- 模型和相机路径适配。

接口不匹配必须在队伍自己的 `ego_adapter` 或 Launch 中解决，禁止修改外部 EGO 核心源码来匹配本项目。

## 7. 导航模式互斥

启动参数固定为：

```text
navigation_mode = ego | layered_2d | static_patrol
```

本阶段只实现和验收 `ego`。在 `ego` 模式中：

- 不启动 `simple_navigator` 的 navigator 输入发布；
- 只有 `ego_adapter` 可以发布 navigator MUX 输入；
- tracking 继续发布 external 输入；
- 起飞节点继续发布 takeoff 输入；
- `safety_filter` 继续是 XTDrone 最终速度话题的唯一发布者。

启动预检发现同一 navigator 输入存在多个发布者时，任务必须拒绝开始。

## 8. 局部目标规则

高层目标可能位于当前相机视野之外，不能直接假定整段空间安全。`search_coordinator` 必须把它拆成连续局部目标：

1. 目标必须位于 `map` 坐标；
2. 搜索高度不高于 4.0 米；
3. 目标位于已知空闲单元或前沿的已知空闲侧；
4. 首版目标距离不超过 8 米，与现有局部地图范围一致；
5. 通往目标的无人机扫掠体不能经过 `UNKNOWN` 或 `OCCUPIED`；
6. 地图变化、轨迹过期、起点偏差或任务代次变化时，旧目标立即失效；
7. 暂时没有合格目标时悬停并转向补充观察，不能向未知区域盲飞。

绕过建筑时可以连续选择建筑侧面的已知空闲观察点，逐步看见并确认建筑后方，再继续趋近原高层目标。

首版不另建全局规划器。协调器先用 `ValidateTrajectory` 检查朝高层目标截取的最多 8 米局部段；验证失败时使用现有 `frontier_goal` 作为观察目标。每取得一段新地图后重新评估原高层目标，直到到达或被取消。

## 9. `ego_adapter` 职责

新增队伍包 `src/ego_fusion_search/ego_adapter`，只承担以下职责：

- 接收 EGO 的位置、速度、加速度、yaw 和时间信息；
- 验证消息有限、连续、新鲜且属于当前任务代次；
- 验证轨迹采样扫掠体不穿越未知、占据或高度边界；
- 根据当前姿态把世界系运动意图转换为现有 FLU 控制接口；
- 大角度改变方向时先减速转向，前向深度健康后再前进；
- 限制侧飞和倒飞，不让固定前视相机在不可见方向高速运动；
- 持续发布状态和拒绝原因；
- 任一校验失败时发布零速度，并使旧轨迹失效。

`ego_adapter` 不提取前沿、不分配多机任务、不修改人物锁，也不直接发布 XTDrone 最终命令。

## 10. 状态机

```text
WAIT_READY
    |
    v
OBSERVING -> PLANNING -> EXECUTING
    ^           |             |
    |           +--无解------>HOLD
    |                         |
    +------补充观察/新地图-----+

EXECUTING --发现人物--> CANDIDATE_HOLD --> TRACKING_EXTERNAL
                                                |
                                                v
                                            REJOINING
                                                |
                                                v
                                            OBSERVING
```

- `WAIT_READY`：起飞、任务、深度、位姿、EGO、MUX 任一未就绪都输出零；
- `OBSERVING`：悬停或原地小角度转向，扩展已知空间；
- `PLANNING`：发送新任务代次目标，等待合格 EGO 结果；
- `EXECUTING`：只执行当前有效轨迹，持续检查地图和制动距离；
- `HOLD`：规划失败或健康失效时归零，不执行最后一条旧命令；
- `CANDIDATE_HOLD`：首次发现人物立即冻结导航转向并平滑减速；
- `TRACKING_EXTERNAL`：tracking 按现有流程取得 MUX 控制权；
- `REJOINING`：清除旧目标和旧轨迹，从当前位置和最新地图重新规划。

## 11. tracking 优先级

控制优先级保持为：

```text
P0 硬安全保护
P1 已锁定人物 tracking
P2 人物候选保持
P3 EGO 搜索导航
P4 悬停
```

tracking 请求接管时：

1. 导航任务代次递增，旧 EGO 轨迹永久失效；
2. navigator 输出平滑归零；
3. 现有流程确认 MUX 切到 external；
4. tracking 才能发布非零控制；
5. 返回 navigator 后，从当前位姿重新建立局部目标；
6. 禁止恢复 tracking 前的旧 EGO 轨迹。

tracking 仍不能越过 P0。局部深度完全失效、位姿跳变、MUX 状态不明或即将碰撞时必须停止运动。

## 12. 硬安全和故障处理

| 故障 | 立即动作 | 恢复条件 |
| --- | --- | --- |
| 深度超过 0.50 秒未更新 | 轨迹失效、零速度 | 连续 1.0 秒健康深度 |
| odom 超过 0.50 秒未更新、倒退或跳变 | 停止融合和执行 | 连续 1.0 秒可靠位姿 |
| EGO 输出过期或含非有限值 | 拒绝并归零 | 新任务代次产生合格输出 |
| 轨迹经过 unknown/occupied | 拒绝并重新观察或规划 | 新轨迹扫掠体验证通过 |
| 前方障碍进入制动距离 | 平滑制动 | 新鲜深度确认净空恢复 |
| 障碍进入紧急距离 | 立即禁止继续接近 | 稳定悬停后重新规划 |
| EGO 连续无解 | 取消目标并选择新观察点 | 新目标规划成功 |
| MUX 选择不明确 | 所有新控制源保持零 | MUX 选择被明确验证 |
| planner/adapter 进程退出 | `safety_filter` 保持零输出 | 人工结束本轮并重新启动 |

恢复不能依赖单帧正常数据。不得在当前飞行中自动切换到 `layered_2d` 或 `static_patrol`。

## 13. 已知路线事故的前置封堵

EGO 正式六机接入前，必须先修复当前静态安全基线：

- 4 号机 `(68,12) -> (68,41)` 对 `house_1_66` 净空不足；
- 5 号机 `(122,12) -> (78,12)` 穿过 `house_3_68`；
- 静态障碍清单遗漏这两栋房，导致危险路线测试仍然通过。

先增加会失败的回归测试，再调整队伍路线和障碍清单。该修复用于保护 `static_patrol` 降级模式，不得用它代替在线避障。

## 14. 测试设计

### 14.1 自动化测试

至少覆盖：

1. 只有一个 navigator 输入发布者；
2. 过期、倒退、非有限和错误任务代次的 EGO 输出被拒绝；
3. 穿越 `UNKNOWN/OCCUPIED` 的轨迹被拒绝；
4. 高于 4.0 米的搜索轨迹被拒绝；
5. 大角度转向时先停止平移；
6. 深度和 odom 超时后在时限内归零；
7. 前向障碍进入制动距离时停止继续接近；
8. tracking 接管使旧目标和旧轨迹失效；
9. tracking 返回后只能从当前位置生成新任务；
10. EGO 退出后最终控制话题保持零输出；
11. 4、5号已知危险路线不能再次通过静态回归测试。

### 14.2 单机 Gazebo 验收

按顺序执行：

1. 无障碍直达，确认接口、坐标和速度方向；
2. 单墙阻挡，确认减速、绕行和到达；
3. 墙角和凹形障碍，确认不会反复撞墙；
4. 目标位于未观察区域，确认先观察再推进；
5. 飞行中切断深度、odom 和规划器，确认悬停；
6. 绕障途中触发人物接管，确认旧轨迹不恢复；
7. 每次记录 Gazebo contacts、真实轨迹、估计轨迹、MUX、原始和最终命令。

至少连续 5 次完整运行全部满足：

- 无建筑、地面或模型接触；
- 无穿越 unknown；
- 无失控上升或定位发散；
- 搜索高度不超过 4.0 米；
- 深度和位姿失效时不继续执行旧轨迹；
- 绕障后到达目标；
- tracking 接管与恢复顺序正确；
- Gazebo 实时因子持续约不低于 0.8；
- EGO 重规划 P95 延迟约不超过 200 毫秒；
- PX4、XTDrone、Gazebo、EGO 和官方模型哈希保持不变。

## 15. 实施阶段

1. 固化4、5号路线事故回归并修复静态降级路线；
2. 固定、登记并只读验证 EGO-Planner-Swarm 外部版本；
3. 建立单机 EGO 接口契约测试；
4. 实现 `ego_adapter` 的失效关闭和轨迹校验；
5. 实现 `search_coordinator` 的短目标、观察和重规划状态机；
6. 增加 `navigation_mode=ego` 互斥 Launch；
7. 完成单机 Gazebo 故障注入和连续5次验收；
8. 单机通过后另行设计双机和六机阶段。

## 16. 相关设计

- `docs/LOCAL_MAPPING_NAVIGATION_DESIGN.md`
- `docs/EGO_FUSION_SEARCH_DESIGN.md`
- `docs/UNKNOWN_MAP_AUTONOMOUS_SEARCH_OPTIONS.md`
- `docs/COMPLIANCE.md`
- `docs/THIRD_PARTY.md`
