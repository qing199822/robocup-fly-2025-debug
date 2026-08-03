# EGO-Fusion Search 详细设计

更新日期：2026-08-03

状态：已确认的方案设计，尚未实现

本文把 [未知地图自主搜索方案调研](UNKNOWN_MAP_AUTONOMOUS_SEARCH_OPTIONS.md) 中的推荐路线细化为可实施规格。目标是在 2025 中国机器人大赛多旋翼无人机集群协同搜索仿真中，让 6 架无人机面对随机障碍地图完成自主覆盖、补漏、人物识别和跟踪。

本文件描述未来要新增的队伍代码、ROS 接口、状态机、故障处理和验收顺序。它不表示 EGO-Planner-Swarm 或本文模块已经接入当前仓库。

## 1. 一句话结论

采用 **EGO-Fusion Search**：

- EGO-Planner-Swarm 是唯一局部轨迹规划内核；
- 队伍自行实现中央 2.5D 覆盖地图、FUEL 风格增量前沿和 RACER 风格任务分配；
- FIESTA 只提供动态占据插入、删除和衰减的设计参考；
- 保留现有 YOLO、目标锁、tracking 和 MUX；
- EGO 和 tracking 都不能直接发布 XTDrone 最终控制命令；
- PX4、XTDrone、Gazebo、EGO 及无人机基础模型保持原版，只在队伍 ROS 包和允许的 Launch、参数、传感器安装位姿范围内集成。

该组合不是把多个完整开源规划器叠在一起。它只保留一条局部规划链，避免地图、轨迹和控制权互相竞争。

## 2. 设计目标与非目标

### 2.1 设计目标

1. 随机障碍位置未知时，不能依赖固定障碍坐标或固定直线路径。
2. 6 架无人机共享同一全局坐标系，并在 10 分钟内尽量减少重复搜索。
3. 每架无人机使用 Realsense 在线感知近处三维障碍，由 EGO 动态绕障。
4. 中央节点记录已观察区域，先执行分区条带覆盖，再用前沿探索补漏。
5. 发现人物后只有一架无人机取得目标锁并跟踪，其他无人机继续搜索。
6. 跟踪结束后，从当前位置领取新任务，不恢复已经过时的 EGO 轨迹。
7. 任意时刻只有一个节点向每架飞机的 XTDrone 最终控制话题发布命令。
8. 深度、定位、规划或中央协调失效时，系统能悬停或降级，不把未知空间当作安全空间。
9. 所有第三方来源、版本和许可证可追踪，队伍算法与第三方源码边界清楚。

### 2.2 非目标

第一版不做以下内容：

- 不修改 PX4 飞控、估计器或参数实现；
- 不修改 XTDrone 通信脚本和核心框架；
- 不修改 Gazebo 物理引擎或人物运动插件；
- 不改变 `typhoon_h480_realsense` 的机体模型，只保留规则允许的 Realsense 安装位姿配置；
- 不把 FUEL、RACER、FIESTA、Fast-Planner 的完整源码同时并入；
- 不在有卫星导航的比赛条件下优先引入完整 VINS-Fusion 定位链；
- 不在第一版实现任意动态物体的复杂行为预测；
- 不承诺本文建议参数无需实测即可用于正式比赛。

## 3. 赛事约束

设计必须始终满足以下约束：

| 约束 | 设计响应 |
| --- | --- |
| 约 200m x 100m 城市场地 | 使用已知边界和中央 2.5D 覆盖地图 |
| 障碍位置每次随机 | 使用在线深度地图和局部重规划 |
| 6 架无人机 | 固定 6 个独立 EGO 实例和一个中央分配器 |
| 高度低于 6m | 正常搜索高度初值 3.0m；所有目标和指令在安全层限高 |
| 单次最长 10 分钟 | 分阶段搜索，后期停止低收益远距离任务 |
| 人物会移动、逃跑和瞬移 | 人物进入短时动态层，不写入永久静态地图 |
| 通信距离无限 | 第一版采用中央地图和中央任务分配 |
| 碰撞影响得分 | 深度超时、轨迹过期和坐标异常时优先悬停 |
| 传感器计价 | 首版复用现有 Realsense，不擅自增加雷达 |

规则解释、传感器成本和候选路线见 [UNKNOWN_MAP_AUTONOMOUS_SEARCH_OPTIONS.md](UNKNOWN_MAP_AUTONOMOUS_SEARCH_OPTIONS.md)。

## 4. 总体架构

```text
每架无人机 N（N = 0..5）

Realsense depth + CameraInfo       MAVROS local odom
             |                            |
             +----------+-----------------+
                        |
                 感知适配与语义过滤
                        |
         +--------------+----------------+
         |                               |
  本机 EGO 三维局部地图             中央 2.5D 覆盖地图
         |                               |
  本机 EGO 局部规划             增量前沿 + 中央任务分配
         ^                               |
         +--------- 每机独立目标 <--------+
                        |
             EGO PositionCommand
                        |
                  ego_adapter
                        |
              navigator MUX 输入
                        |
现有 tracking ----------+---------- pose_cmd_mux
       external MUX 输入             ^      |
现有 fly 起飞输入 -------------------+      |
                                      MUX 原始输出
                                            |
                                      safety_filter
                                            |
                       /xtdrone/typhoon_h480_N/cmd_vel_flu
                              （唯一发布者）
```

中央层负责回答“哪架飞机下一步去哪里”，EGO 负责回答“这架飞机怎样安全到达短距离目标”，tracking 负责回答“锁定人物后怎样跟随”。三个问题不能由同一个庞大节点同时处理。

## 5. 源码与所有权边界

### 5.1 外部只读依赖

以下内容是环境依赖，不在队伍代码中修改：

```text
/home/wangtao/robocup_fly/PX4_Firmware
/home/wangtao/robocup_fly/XTDrone
/home/wangtao/robocup_fly/gazebo_models
/home/wangtao/robocup_fly/ego-planner-swarm    # 建议的未来外部目录名
```

EGO-Planner-Swarm 必须固定到经过验证的提交。队伍 Launch 可以 include 和 remap 外部 EGO 节点，但不得为了适配本项目修改 EGO 核心源码。若发现接口不匹配，在队伍自己的 adapter 中解决。

### 5.2 建议新增的队伍包

未来实现建议放在一个清晰的功能目录下：

```text
src/ego_fusion_search/
|-- search_msgs/           # 队伍自有消息和服务
|-- perception_adapter/    # 深度、位姿、点云与语义掩膜
|-- coverage_map/          # 中央 2.5D 覆盖地图
|-- frontier_manager/      # 增量前沿提取、聚类和生命周期
|-- swarm_allocator/       # 6 机任务分配
|-- ego_adapter/           # EGO 目标、轨迹命令与 Twist 适配
|-- search_coordinator/    # 总状态机和搜索阶段切换
|-- safety_filter/         # MUX 后置安全门
`-- launch/                # 队伍集成 Launch 和参数
```

每个包只承担一种责任。不得在 `ego_adapter` 中偷偷实现前沿分配，也不得在 `swarm_allocator` 中发布飞机速度。

### 5.3 第三方思想与源码边界

| 来源 | 只采用的长处 | 明确不采用的内容 |
| --- | --- | --- |
| EGO-Planner-Swarm | B 样条局部轨迹、实时重规划、多机轨迹避碰 | 不修改核心源码，不绕过队伍控制链 |
| FUEL | 增量维护前沿、按信息增益选择探索目标 | 不接入第二套完整轨迹规划器 |
| RACER | 中央竞价、负载均衡、冲突惩罚的思想 | 许可证不明时不复制源码 |
| FIESTA | 障碍增量插入、删除和时间衰减思想 | 第一版不额外运行一套完整 ESDF |
| Fast-Planner | 动力学约束和 B 样条工程经验 | 不与 EGO 并行运行第二规划内核 |
| OctoMap/Voxblox | 后续高质量全局地图备选 | 第一版不重复维护完整三维全局地图 |

赛事存在代码相似度检查。前沿、任务分配、状态机和控制适配应由队伍根据本文接口自行实现，并记录参考论文，不复制来源不明的实现。

## 6. 坐标系约定

### 6.1 唯一全局坐标

所有中央数据使用 `map` 坐标系，方向约定为 ROS ENU：

- x：东或场地长边正方向；
- y：北或场地短边正方向；
- z：向上；
- 角度：弧度；
- 位置：米；
- 时间：ROS 仿真时间。

当前 `pose_init` 已把每架 MAVROS 局部位姿加已知起点偏移，发布：

```text
/typhoon_h480_N/global_pose  geometry_msgs/PoseStamped
/typhoon_h480_N/global_odom  nav_msgs/Odometry
```

第一阶段应继续以这两个话题作为队伍全局坐标输入。正式接入前必须实测 6 架飞机的相同世界点是否转换为相同 `map` 坐标，不能仅因话题名包含 `global` 就默认正确。

### 6.2 EGO 坐标输入

每个 EGO 实例的 `~odom_world` 重映射到对应的 `global_odom`。深度位姿必须通过 TF 与同一 `map` 坐标一致。严禁同时叠加：

- `pose_init` 起点偏移；
- EGO Launch 中的第二份起点偏移；
- VIO bias；
- Gazebo 世界坐标手工修正。

四者只能保留一套被验证的全局对齐逻辑。第一版保留当前 `pose_init`，EGO 和中央地图只消费其结果。

### 6.3 坐标健康检查

系统启动后必须执行以下检查：

1. `global_odom` 时间戳连续且不倒退；
2. 位置和姿态均为有限数值；
3. 速度不出现不合理跳变；
4. 6 架飞机的初始相对位置与 Launch 起点一致；
5. 深度点投影到 `map` 后，地面和建筑位置稳定；
6. 任一检查连续失败，禁止地图融合和跨机轨迹共享。

## 7. 每机感知与语义过滤

### 7.1 输入

每架无人机消费以下现有数据：

| 话题模板 | 消息类型 | 用途 |
| --- | --- | --- |
| `/typhoon_h480_N/realsense/depth_camera/depth/image_raw` | `sensor_msgs/Image` | 深度障碍输入 |
| `/typhoon_h480_N/realsense/depth_camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度投影内参；正式接入时核对实际话题名 |
| `/typhoon_h480_N/realsense/depth_camera/color/image_raw` | `sensor_msgs/Image` | YOLO 人物检测 |
| `/typhoon_h480_N/global_odom` | `nav_msgs/Odometry` | 深度位姿和速度 |
| 现有 YOLO bounding boxes | `darknet_ros_msgs/BoundingBoxes` | 人物像素区域 |
| 现有人物三维结果 | `actor_msgs/ActorInfo` | 动态目标位置 |

表中的深度 `CameraInfo` 名称需要在真实仿真中用 `rostopic list` 固定为测试契约；当前 smoke 只验证了彩色 `CameraInfo`。

### 7.2 输出分流

感知适配层把同一帧深度分成三类：

1. `static_obstacle`：建筑、墙、灯杆等，进入 EGO 局部障碍地图；
2. `dynamic_semantic`：人物检测框对应的空间点，只进入短时动态层；
3. `invalid_or_unknown`：无效深度、超量程或遮挡，保持 unknown。

人物掩膜应比检测框略膨胀，避免框边缘点被写入静态层。没有可靠 YOLO 对应关系时，宁可将近处物体短时视为障碍，也不能直接清除。

### 7.3 时间同步

深度图、CameraInfo、位姿和检测框按时间戳匹配：

- 深度与位姿最大差值初始建议 100ms；
- 检测框与深度最大差值初始建议 150ms；
- 超出窗口的检测框不能用于语义清除；
- 使用图像原始时间戳，不能在回调中统一改成“当前时间”；
- 仿真暂停或时间跳变后清空同步缓存。

### 7.4 动态人物 TTL

动态占据记录：

```text
位置 + 半径 + 首次时间 + 最近观测时间 + 目标 ID + 置信度
```

初始建议：

- 人物占据半径：0.8m；
- 正常动态 TTL：1.0s；
- 目标被确认瞬移：立即删除旧目标占据，再在新位置建立短时占据；
- 低置信度观测：只允许延长较短 TTL；
- 动态点不得写入永久静态占据或覆盖完成记录。

这些是起始调参值，不是已经验证的比赛参数。

## 8. EGO 局部规划层

### 8.1 一机一实例

6 架无人机分别运行独立 EGO 实例。每个实例只读取本机深度和本机 odometry，并通过 EGO 原生 swarm trajectory 机制交换轨迹。

不得让 6 个实例共用以下全局话题：

- `/move_base_simple/goal`；
- `/position_cmd`；
- 私有 odometry 或深度输入；
- 本应按无人机区分的局部地图输出。

### 8.2 EGO 原生接口与重映射

根据已核对的 EGO-Planner-Swarm ROS1 接口，集成 Launch 应使用以下映射。具体命名空间仍需在固定提交上做自动化契约测试。

| EGO 接口 | 类型 | 每机映射建议 |
| --- | --- | --- |
| `~odom_world` | `nav_msgs/Odometry` | `/typhoon_h480_N/global_odom` |
| `~grid_map/depth` | `sensor_msgs/Image` | 语义过滤后的本机深度 |
| `~grid_map/pose` | `geometry_msgs/PoseStamped` | 与深度同步的相机位姿 |
| `~grid_map/cloud` | `sensor_msgs/PointCloud2` | 点云路线时的备选输入，不与深度路线同时启用 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | `/typhoon_h480_N/ego/goal` |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | `/typhoon_h480_N/ego/position_cmd` |
| `/broadcast_bspline` | `traj_utils/Bspline` | 保留 EGO 的多机轨迹广播机制 |

第一版在“深度 + 位姿”和“点云”两种地图输入中只选择一种。默认优先深度路线，以减少额外点云复制。

### 8.3 目标发送规则

中央分配器不直接控制 EGO，而是由每机 `search_coordinator` 发送短距离目标：

- 目标位于 `map` 坐标系；
- z 在安全搜索高度范围内；
- 目标必须处于已知 free 或前沿的安全侧，不能放在 unknown 深处；
- 单段目标距离初始建议不超过 15m；
- 新目标带单调递增的 `task_id`；
- 只有处于 `SEARCHING` 或 `REJOINING` 的飞机可以向 EGO 发目标；
- 进入 tracking 前必须取消当前任务并使旧轨迹失效；
- 离开 tracking 后重新申请任务，禁止继续执行旧目标或旧 B 样条。

### 8.4 EGO 输出有效性

`ego_adapter` 接收 `PositionCommand` 后检查：

1. 消息时间未过期；
2. 位置、速度、加速度、yaw 和 yaw rate 均为有限值；
3. 当前 `task_id` 与协调器有效任务一致；
4. 规划器状态不是失败或停止；
5. 当前飞机仍拥有导航控制权；
6. 目标高度和预测轨迹低于比赛限高并留有余量。

任一检查失败时，adapter 输出零速度并上报故障，但最终悬停仍由后置 `safety_filter` 保证。

## 9. PositionCommand 到 XTDrone Twist 的适配

### 9.1 为什么需要 adapter

EGO 的 `quadrotor_msgs/PositionCommand` 包含世界坐标下的期望位置、速度、加速度和偏航；当前 XTDrone MUX 输入是机体系 FLU 的 `geometry_msgs/Twist`。两者消息类型、坐标系和控制语义不同，不能只改话题名直接连接。

### 9.2 第一版控制方法

adapter 使用期望速度加位置误差反馈：

```text
v_world_cmd = v_ego + Kp_position * (p_ego - p_current)
```

然后按当前 yaw 把世界 ENU 水平速度转换到机体 FLU：

```text
v_body_x =  cos(yaw) * v_world_x + sin(yaw) * v_world_y
v_body_y = -sin(yaw) * v_world_x + cos(yaw) * v_world_y
v_body_z =  v_world_z
yaw_rate  = clamp(Kp_yaw * angle_error + yaw_rate_ego)
```

输出：

```text
/typhoon_h480_N/mux_inputs/navigator/cmd_vel
geometry_msgs/Twist
```

这沿用当前 navigator MUX 输入，因此 MUX 的“导航”和“跟踪”两类控制权不变。接入 EGO 后，旧 `simple_navigator` 不得与 `ego_adapter` 同时发布该话题。

### 9.3 速度与连续性约束

初始限制建议：

| 参数 | 初始值 | 说明 |
| --- | ---: | --- |
| 水平速度 | 3.0m/s | 先保证随机障碍中的稳定性，再逐步提高 |
| 垂直速度 | 1.0m/s | 降低触碰限高或地面的风险 |
| yaw rate | 1.0rad/s | 与相机视野和跟踪切换兼容 |
| 水平加速度 | 2.0m/s² | 限制速度跳变 |
| 垂直加速度 | 1.0m/s² | 避免上下振荡 |
| 指令超时 | 0.25s | 超时立即进入零速度保护 |

adapter 不使用 EGO acceleration 直接控制 PX4 姿态，第一版只把它用于前馈或诊断。这样能保持 XTDrone 当前速度接口和 PX4 原版控制链。

## 10. 中央 2.5D 覆盖地图

### 10.1 目的

中央地图不替代 EGO 的三维局部地图。它只回答：

- 哪些地面区域已经被相机有效观察；
- 哪些区域在搜索高度可能通行；
- 哪些区域仍需分配；
- 哪架飞机当前负责哪个区域。

### 10.2 网格结构

建议每格存储：

```text
cell_state: UNKNOWN | FREE | OCCUPIED | INFLATED | OUTSIDE
observed: bool
last_observed_time: ros::Time
observation_count: uint16
height_min / height_max: float
assigned_vehicle: int8
assignment_expiry: ros::Time
frontier_cluster_id: int32
semantic_flags: bitset
```

初始分辨率建议 0.5m。200m x 100m 场地约 80,000 个网格，适合中央节点实时维护。若灯杆等细障碍在 0.5m 下丢失，它们仍由每机 EGO 三维地图处理；中央层只需避免生成明显穿过建筑的目标。

### 10.3 观察判定

不能因为无人机飞过某格上方就直接标记“已搜索”。覆盖完成至少需要：

1. 相机光轴和视场确实覆盖该地面格；
2. 深度有效或射线能够确认无遮挡可见；
3. 观测距离在可靠范围内；
4. 相机位姿时间与图像匹配；
5. 连续观测达到最低次数或置信度。

人物检测范围与避障深度范围可以不同。覆盖率计算必须基于“能够发现人物的有效视场”，不能只基于障碍点云。

### 10.4 六机融合

每机只向中央节点发送降采样后的 `MapObservation` 增量，不发送完整高分辨率 ESDF。中央节点按 `map` 坐标合并，并保留来源飞机和时间戳用于诊断。

若某架飞机坐标健康检查失败：

- 停止融合该机新数据；
- 不删除其他飞机已建立的覆盖记录；
- 取消该机跨区域任务；
- 让该机进入本地安全悬停或固定分区降级模式。

## 11. FUEL 风格增量前沿

### 11.1 前沿定义

候选前沿格满足：

- 当前格为已确认 FREE；
- 8 邻域中至少存在 UNKNOWN；
- 不在障碍膨胀区；
- 与边界、建筑和已知危险保持安全距离；
- 对人物发现具有可用视角。

前沿格通过 8 邻域连接聚类为 `FrontierCluster`。

### 11.2 前沿数据结构

```text
FrontierCluster
  id
  revision
  centroid
  viewpoint
  yaw
  cell_count
  expected_new_area
  distance_to_obstacle
  last_updated
  status: ACTIVE | ASSIGNED | VISITED | BLOCKED | STALE
  assigned_vehicle
  failure_count
  blacklist_until
```

`id` 在同一空间簇生命周期内保持稳定，`revision` 在簇形状变化时递增。分配结果必须引用二者，防止飞机执行已经消失的前沿。

### 11.3 增量更新

只有地图变化区域及其一圈邻域重新计算前沿，不在每帧扫描整张地图。更新流程：

1. 收集本周期变更格；
2. 删除不再满足条件的旧前沿格；
3. 增加新前沿格；
4. 对受影响簇局部重聚类；
5. 计算候选视点和预计信息增益；
6. 更新分配器的候选集合。

### 11.4 前沿视点

视点位于前沿的 free 一侧，而不是未知区域内部。候选视点需通过：

- 2.5D 障碍膨胀检查；
- EGO 可规划性预检查；
- 高度范围检查；
- 相机朝向和预计可见面积检查；
- 与其他飞机当前目标的最小间距检查。

连续 3 次规划失败的视点初始黑名单 10s，然后可因地图变化重新激活。数值需实测调整。

## 12. RACER 风格中央任务分配

### 12.1 为什么采用中央分配

比赛通信距离无限，且当前已有中央目标锁服务。中央分配比第一版完整分布式竞价更容易调试，也能减少 6 架飞机重复领取同一前沿。

### 12.2 可分配任务

任务分为：

```text
STRIP_WAYPOINT      分区条带覆盖点
FRONTIER_VIEWPOINT  前沿补漏视点
REJOIN_POINT        跟踪结束后的安全重入点
HOLD_POINT          临时悬停点
FINISH_POINT        结束或返航点
```

每个任务具有唯一 `task_id`、地图 `revision`、有效期、目标位姿、预计收益和失败计数。

### 12.3 分配代价

对飞机 i 和任务 j 计算：

```text
cost(i,j) = wd * travel_distance
          + wt * estimated_travel_time
          + wr * obstacle_risk
          + wc * inter_vehicle_conflict
          + wl * current_load
          + wf * repeated_failure
          - wg * expected_information_gain
          - wb * strip_balance_bonus
```

第一版不要追求复杂学习模型。各项归一化到相近量级，保留日志中的分项值，才能解释为什么某架飞机拿到某个任务。

硬约束先于代价：

- 非健康飞机不能分配；
- tracking 中的飞机不能分配；
- 高度或边界非法的任务不能分配；
- 已被有效租约占用的任务不能重复分配；
- EGO 明确不可达的任务不能立即重新分给同一飞机。

### 12.4 租约与心跳

分配不是永久占用：

- 飞机收到任务后返回 ACK；
- 飞行期间周期上报进度；
- 租约初始建议 5s，并由正常心跳续期；
- 飞机进入 tracking、失联、规划失败或租约过期，任务自动回收；
- 回收任务按最新地图重新评估，不能盲目交给下一架飞机。

### 12.5 分配周期

以下事件触发重分配：

- 新前沿产生或旧前沿消失；
- 飞机完成、拒绝或放弃任务；
- 飞机进入或退出 tracking；
- 租约过期；
- 地图风险显著变化；
- 搜索阶段切换。

无事件时每 1s 做一次保底检查，避免高频全局重算。

## 13. 搜索策略与时间预算

### 13.1 第一阶段：六区条带快速覆盖

场地沿长边分为 6 个主责任区，每架从自己的起点进入最近区域。条带间距由相机在搜索高度的有效地面视场决定，并扣除遮挡余量。

条带点不是必须直线到达的固定航点。它只是短距离探索目标，EGO 仍根据在线障碍地图绕行。若条带被建筑截断，前沿阶段负责补齐遮挡后的区域。

### 13.2 第二阶段：前沿补漏

当任一条件成立时进入前沿补漏：

- 初始条带任务大部分完成；
- 某区域因障碍连续跳过条带点；
- 全局新增覆盖率下降；
- 剩余未观察区域主要集中在遮挡后方。

空闲飞机领取高收益前沿，允许跨越初始分区，但代价中保留跨区和冲突惩罚。

### 13.3 第三阶段：收尾

剩余比赛时间低于可配置阈值时：

- 不再分配收益低且距离远的前沿；
- 优先处理已发现但未稳定跟踪的目标附近区域；
- 保留足够时间处理规划失败、悬停或结束动作；
- 禁止为了极小覆盖增益执行高碰撞风险穿越。

建议以仿真时间而非墙钟时间计算阶段，正式阈值通过完整 10 分钟回归确定。

## 14. 搜索、跟踪和重入状态机

### 14.1 每机状态

```text
BOOT
  -> WAIT_READY
  -> TAKEOFF
  -> SEARCHING
  -> TRACKING
  -> REJOINING
  -> SEARCHING

任意运行状态
  -> HOLD
  -> SEARCHING / REJOINING / FINISHED

任务结束
  -> FINISHED
```

### 14.2 状态含义

| 状态 | 控制源 | 行为 |
| --- | --- | --- |
| `BOOT` | 无 | 加载参数并检查接口 |
| `WAIT_READY` | 安全零速度 | 等待 odom、深度、EGO、MUX 和通信 |
| `TAKEOFF` | fly 的 takeoff MUX 输入 | 起飞到安全搜索高度 |
| `SEARCHING` | EGO adapter | 执行条带或前沿任务 |
| `TRACKING` | 现有 tracking | 锁定并跟踪人物 |
| `REJOINING` | EGO adapter | 从当前位置领取安全重入任务 |
| `HOLD` | safety filter | 零速度悬停并等待恢复条件 |
| `FINISHED` | 安全结束策略 | 悬停、返航或规则要求动作 |

### 14.3 从搜索切换到跟踪

顺序必须是：

1. tracking 向现有 `/lookup/request_<target>` 请求目标锁；
2. 锁成功后，协调器标记该飞机不可分配；
3. 回收该机当前搜索任务租约；
4. 使当前 EGO `task_id` 失效；
5. 请求 MUX 切到 `/typhoon_h480_N/mux_inputs/external/pose_cmd`；
6. 确认 MUX 选择成功；
7. tracking 才开始输出非零控制；
8. 其他 5 架飞机继续搜索并重新分配空缺区域。

若 MUX 切换失败，tracking 不得输出非零命令，飞机进入 `HOLD`。

### 14.4 从跟踪切回搜索

顺序必须是：

1. tracking 停止非零输出并发布短暂零速度；
2. 释放人物目标锁；
3. 协调器读取当前位姿和地图健康状态；
4. 申请最近的安全 `REJOIN_POINT` 或有效前沿；
5. 发送全新的 EGO 目标和 `task_id`；
6. 等待 EGO 生成新鲜有效的 `PositionCommand`；
7. 请求 MUX 切回 navigator 输入；
8. 确认切换成功后进入 `REJOINING`；
9. 到达重入点后进入 `SEARCHING`。

绝对禁止恢复 tracking 前的旧 B 样条、旧前沿租约或旧固定航点栈。未知地图会持续变化，旧路径可能已经不安全。

## 15. MUX 与统一安全过滤链

### 15.1 最终控制拓扑

未来接入后，现有 MUX 输出要从 XTDrone 最终话题改到队伍内部话题：

```text
/typhoon_h480_N/mux_inputs/takeoff/cmd_vel     geometry_msgs/Twist
/typhoon_h480_N/mux_inputs/navigator/cmd_vel   geometry_msgs/Twist
/typhoon_h480_N/mux_inputs/external/pose_cmd   geometry_msgs/Twist
                         |
                 topic_tools pose_cmd_mux
                         |
/typhoon_h480_N/control/raw_cmd_vel             geometry_msgs/Twist
                         |
              /typhoon_h480_N/safety_filter
                         |
/xtdrone/typhoon_h480_N/cmd_vel_flu             geometry_msgs/Twist
```

最终话题的唯一发布者必须是 `safety_filter`。PX4、EGO、tracking、`fly_takeoff`、旧 navigator、任务分配器都不得直接发布该话题。

当前 `src/mix_nav/fly/src/fly_takeoff.cpp` 会直接发布 XTDrone 最终速度话题。未来实施安全链时，必须修改这个队伍自有包，使其改发 `/typhoon_h480_N/mux_inputs/takeoff/cmd_vel`，并由起飞状态机选择该 MUX 输入。起飞完成后先归零，再确认切到 navigator。不能通过放宽“唯一发布者”检查来兼容旧行为。

### 15.2 安全过滤器输入

每机安全过滤器至少读取：

- MUX 原始 Twist；
- 当前 `global_odom`；
- 深度健康状态；
- EGO/跟踪状态和控制模式；
- 当前任务有效性；
- 地图边界和高度限制；
- 最近障碍距离；
- 其他无人机共享轨迹健康状态。

### 15.3 安全检查顺序

1. 输入消息类型、时间戳和发布频率有效；
2. takeoff、navigator、external 中只有当前选中的 MUX 输入在控制；
3. 位置、姿态和速度健康；
4. 当前高度和预测下一时刻高度合法；
5. 深度未超时；
6. 前向停止距离内无新障碍；
7. 指令速度、yaw rate、加速度和 jerk 在限制内；
8. 坐标转换结果为有限值；
9. 通过后发布限幅指令，否则发布零速度并上报原因。

### 15.4 停止距离

安全层按当前速度估算：

```text
stop_distance = reaction_time * speed
              + speed^2 / (2 * guaranteed_deceleration)
              + fixed_margin
```

若有效深度范围小于停止距离，不允许继续加速。具体减速度必须用当前 PX4 1.11 + XTDrone 机型实测，不直接沿用理论值。

### 15.5 发布者守卫

smoke 和比赛前验证必须检查：

```text
rostopic info /xtdrone/typhoon_h480_N/cmd_vel_flu
```

每架飞机恰好只有一个队伍最终发布者，且节点名属于对应 `safety_filter`。发现第二发布者时启动失败，而不是打印警告后继续飞行。

## 16. 队伍自有消息与服务建议

以下为未来 `search_msgs` 的逻辑字段，具体 `.msg/.srv` 在实施计划阶段确定。

### 16.1 `MapObservation`

```text
Header header
uint8 vehicle_id
uint64 sequence
CellUpdate[] cells
bool pose_healthy
```

### 16.2 `FrontierCluster`

```text
Header header
uint64 frontier_id
uint32 revision
Pose viewpoint
float32 expected_new_area
float32 obstacle_clearance
uint8 status
```

### 16.3 `SearchTask`

```text
Header header
uint64 task_id
uint8 task_type
uint8 vehicle_id
uint64 map_revision
PoseStamped goal
Duration lease
float32 expected_gain
```

### 16.4 `VehicleSearchStatus`

```text
Header header
uint8 vehicle_id
uint8 state
uint64 task_id
float32 task_progress
bool odom_healthy
bool depth_healthy
bool planner_healthy
string fault_code
```

### 16.5 服务语义

- `RequestSearchTask`：空闲或重入飞机申请任务；
- `AcknowledgeSearchTask`：接受或拒绝任务；
- `CompleteSearchTask`：完成、失败或取消任务；
- `InvalidateTrajectory`：控制权切换时使旧 EGO 任务失效；
- `SetSearchState`：只允许协调器请求状态转换，不直接传速度。

所有请求应幂等。重复 ACK、重复释放或网络延迟不能造成同一任务被永久占用。

## 17. 参数初值

下表用于第一轮仿真，不是比赛最终参数：

| 类别 | 参数 | 初值 |
| --- | --- | ---: |
| 覆盖地图 | 分辨率 | 0.5m |
| 局部感知 | 有效半径 | 8m |
| 静态障碍 | 水平膨胀 | 0.8m |
| 动态人物 | 占据半径 | 0.8m |
| 动态人物 | TTL | 1.0s |
| 搜索高度 | 正常目标 | 3.0m |
| 高度安全 | 软件上限 | 5.5m |
| EGO 目标 | 最大单段距离 | 15m |
| 前沿 | 最小簇面积 | 2.0m² |
| 前沿 | 连续失败黑名单 | 3 次 / 10s |
| 分配器 | 保底周期 | 1.0s |
| 任务租约 | 初始时长 | 5.0s |
| 控制 | PositionCommand 超时 | 0.25s |
| 控制 | 水平速度上限 | 3.0m/s |
| 控制 | 垂直速度上限 | 1.0m/s |
| 控制 | yaw rate 上限 | 1.0rad/s |

参数必须集中在队伍 YAML 中，不散落在源码和 Launch。每次调参记录地图种子、结果和碰撞情况。

## 18. 故障处理与降级

| 故障 | 检测 | 立即动作 | 恢复条件 |
| --- | --- | --- | --- |
| 深度超时 | 超过配置时限无新帧 | safety filter 零速度，unknown 不清空 | 连续多帧有效且时间同步正常 |
| odom 超时/跳变 | 时间或位姿健康检查失败 | 停止融合、取消任务、悬停 | 坐标重新稳定并通过相对位置检查 |
| EGO 无解 | 连续规划失败 | 保持安全点，任务标失败 | 分配新视点或地图变化 |
| EGO 命令过期 | 超过 0.25s | 零速度 | 收到当前任务的新鲜命令 |
| 中央地图失联 | 心跳超时 | 保留本机局部避障，执行原责任区保守任务 | 中央 revision 连续恢复 |
| 分配器失联 | 租约无法续期 | 不跨区，完成当前安全短段后悬停 | 重新注册并取得新任务 |
| MUX 切换失败 | 服务失败或选择结果不符 | tracking 和 EGO 都输出零，进入 HOLD | MUX 状态确认正常 |
| 目标瞬移 | 同 ID 位置不连续 | 清除旧动态占据，重新锁定/搜索 | 新观测连续稳定 |
| 其他飞机轨迹过期 | swarm trajectory 超时 | 增大安全距离并减速，不假设对方消失 | 收到连续有效轨迹 |
| safety filter 退出 | 最终话题停止 | PX4 保持最后命令存在风险，因此监督器必须快速重启或上游归零 | 节点恢复且输入健康 |

特别注意：速度接口的“最后一条非零命令是否会持续生效”必须在 PX4 1.11 + XTDrone 上实测。因此 safety filter 退出前的看门狗设计是第一阶段安全验收项，不能留到最后。

## 19. 启动与关闭顺序

### 19.1 启动顺序

```text
1. competition-clean 快速合规预检
2. Gazebo + 6 PX4 + 6 MAVROS
3. 6 个 XTDrone communication
4. pose_init、TF、Realsense、YOLO
5. safety_filter（先输出零速度）
6. 包含 takeoff、navigator、external 三个输入并输出到 raw_cmd_vel 的 6 个 MUX
7. 每机 perception_adapter
8. 每机 EGO 实例和 ego_adapter
9. coverage_map、frontier_manager、swarm_allocator
10. search_coordinator
11. 现有 target lock 和 tracking
12. 起飞就绪门控
13. 自动搜索
```

任何关键节点未 ready，不得进入起飞和搜索。

### 19.2 关闭顺序

1. 协调器停止分配新任务；
2. 使所有 EGO task 失效；
3. tracking 停止非零输出并释放目标锁；
4. safety filter 持续发布零速度一个可验证窗口；
5. 停止队伍算法节点；
6. 按现有 `1.sh` 有界清理 PX4、Gazebo、ROS 和临时文件；
7. 验证官方目录哈希和无残留进程。

## 20. 可观测性与日志

必须能从日志回答以下问题：

- 某个时刻每架飞机处于什么状态；
- 当前任务、前沿、地图 revision 和租约是什么；
- 为什么任务被分配给这架飞机；
- 为什么 EGO 规划失败或轨迹失效；
- 为什么 MUX 切换；
- 为什么 safety filter 限速或悬停；
- 人物锁由哪架飞机持有；
- 当前覆盖率、发现人数、碰撞数和剩余时间是多少。

建议诊断话题：

```text
/search/global/status
/search/map/coverage
/search/frontiers
/search/assignments
/typhoon_h480_N/search/status
/typhoon_h480_N/ego_adapter/status
/typhoon_h480_N/safety/status
```

日志使用稳定 `fault_code`，不要只输出自由文本。例如：

```text
DEPTH_TIMEOUT
ODOM_JUMP
EGO_NO_PATH
EGO_COMMAND_STALE
MUX_SELECT_FAILED
TASK_LEASE_EXPIRED
ALTITUDE_LIMIT
STOP_DISTANCE_INSUFFICIENT
```

## 21. 测试策略

### 21.1 单元测试

- 世界速度到机体 FLU 的坐标转换；
- yaw 跨越 `-pi/pi` 时的角度误差；
- PositionCommand 过期和 NaN 拒绝；
- 地图射线与覆盖判定；
- 动态人物 TTL 和瞬移清除；
- 增量前沿增加、合并、分裂和删除；
- 任务代价分项和硬约束；
- 租约超时、重复 ACK 和任务回收；
- 状态机非法转换拒绝；
- 高度、速度、加速度和停止距离限制。

### 21.2 接口契约测试

- 6 个 EGO 目标和输出话题完全隔离；
- 每机 EGO odom 映射到对应 `global_odom`；
- EGO 输入只选择 depth 或 cloud 之一；
- `fly_takeoff` 只发布 takeoff 输入，不再直发 XTDrone 最终话题；
- `simple_navigator` 与 `ego_adapter` 不同时发布 navigator 输入；
- MUX 输出只进入 `raw_cmd_vel`；
- XTDrone 最终话题只有 safety filter 一个发布者；
- PX4、XTDrone、Gazebo、EGO 外部树在构建和运行后哈希不变。

### 21.3 仿真验收阶段

#### 阶段 A：离线接口和坐标

- 录制单机 bag，验证深度、位姿和语义同步；
- 障碍点在 `map` 中不随飞机运动漂移；
- adapter 对预录 PositionCommand 输出方向正确；
- 安全层拒绝过期、NaN 和越界命令。

#### 阶段 B：单机地图和 EGO

- 无人物时绕过建筑和灯杆；
- 人物走过后不留下永久墙；
- 深度中断立即悬停；
- 目标在障碍物后时可重规划到达；
- 全程高度低于软件上限。

#### 阶段 C：双机控制权和避碰

- 两个 EGO 实例的目标与输出不串线；
- 交叉路径时保持最小安全距离；
- 一机切 tracking 时另一机继续搜索；
- 跟踪结束只执行新任务，不续跑旧轨迹；
- 任意时刻最终话题只有一个发布者。

#### 阶段 D：六机条带覆盖

- 6 个初始区域分配正确；
- 覆盖率随有效观测单调增加；
- 无人机不在同一区域长期重复；
- 中央失联时各机保留本区保守搜索；
- 单机故障后任务可以回收。

#### 阶段 E：前沿补漏

- 建筑后方未知区能形成前沿；
- 同一前沿不会被多架长期占用；
- 不可达前沿会退避而非死循环；
- 地图变化后旧黑名单可合理恢复；
- 收尾阶段拒绝低收益高风险远任务。

#### 阶段 F：完整比赛回归

至少选择多组随机地图种子，每组重复运行，记录：

```text
发现人数
稳定跟踪人数
首次发现时间
任务完成时间
有效覆盖率
重复覆盖率
碰撞次数
EGO 重规划和失败次数
HOLD 次数和原因
CPU / GPU / 内存峰值
退出后的残留进程
官方输入哈希
```

只有重复运行结果稳定，才逐步提高速度或缩小安全余量。

## 22. 分阶段实施顺序

实现必须按以下顺序推进，每阶段独立可验收：

1. **控制安全骨架**：后置 safety filter、唯一发布者检查、零速度看门狗。
2. **单机 EGO 适配**：只接 1 架飞机，完成 odom/depth/goal/PositionCommand/Twist 链路。
3. **单机语义地图**：人物不进入永久静态障碍，TTL 可验证。
4. **双机 EGO**：话题隔离、轨迹交换、最小间距和控制权切换。
5. **六机条带搜索**：先不做前沿，验证中央任务和覆盖记录。
6. **增量前沿补漏**：增加 FUEL 风格局部更新，不更换 EGO。
7. **任务竞价与回收**：增加 RACER 风格代价、租约和负载均衡。
8. **完整跟踪重入**：搜索、目标锁、tracking、EGO 新任务闭环。
9. **动态衰减优化**：按真实人物行为调节 TTL 和短时障碍。
10. **比赛回归与裁剪**：删除收益不足的复杂功能，固定版本和参数。

在阶段 2 单机链路稳定前，不启动六机 EGO；在阶段 5 条带搜索稳定前，不接前沿分配。这样每次故障都能定位到少量模块。

## 23. 验收标准

方案实施完成需要同时满足：

1. PX4、XTDrone、Gazebo、EGO 和官方模型未被修改；
2. 6 个 EGO 实例输入输出严格隔离；
3. 随机障碍地图中不依赖固定障碍坐标；
4. 深度异常时不会继续盲飞；
5. 每架 XTDrone 最终控制话题恰好一个发布者；
6. 起飞、EGO 和 tracking 任意时刻只有一个控制源拥有非零控制权；
7. 发现人物后其他飞机继续搜索；
8. 跟踪结束重新分配新任务，不恢复旧轨迹；
9. 动态人物不会在静态地图中留下永久障碍；
10. 六机覆盖率、重复覆盖率和碰撞数有自动统计；
11. 多组随机地图完整运行可重复；
12. 正常退出无残留进程，官方输入哈希不变；
13. 第三方来源、固定提交和许可证记录完整；
14. 队伍成员能够仅根据文档和日志解释每次任务分配与安全停车原因。

## 24. 实施前必须确认的开放项

以下问题不影响架构确认，但必须在写代码前用当前环境实测或向裁判确认：

1. 固定使用的 EGO-Planner-Swarm 提交号及其 ROS Noetic 构建结果；
2. EGO 依赖放在外部只读目录还是由比赛提交环境预装；
3. Realsense 深度 `CameraInfo` 的实际话题名和深度可靠量程；
4. `global_odom` 六机坐标对齐误差；
5. XTDrone 速度命令停止发布后 PX4 1.11 的实际保持行为；
6. EGO 多机轨迹广播在 6 实例下的 CPU 和通信占用；
7. Realsense 在赛事计分中按哪类传感器计价；
8. 裁判对 GPL-3.0 EGO 外部依赖、源码提交和公开仓库的具体要求；
9. 人物锁“完成”的判定是持续跟踪、到达距离、识别上报还是其他规则；
10. 比赛结束时要求悬停、降落还是仅停止计时。

没有证据时保留保守行为：限速、悬停、保持 unknown、拒绝多发布者。

## 25. 未来日常使用命令的目标形态

实现完成后仍应保持一个主入口，而不是要求选手手工打开大量终端：

```bash
cd /home/wangtao/robocup_fly/2025_ZZU_FLY-competition-clean
bash 1.sh 6 ego_fusion_search.yaml
```

这是目标接口，当前尚不可用。实际实现应扩展现有 `1.sh` 的模式参数或增加兼容入口，同时保留原固定航点模式作为降级和对照组：

```text
fixed-waypoint       现有基线
ego-single           单机 EGO 验证
ego-strip            六机条带覆盖
ego-fusion-search    完整前沿、分配和跟踪重入
```

模式切换只能改变队伍节点和参数，不能复制或修改外部官方源码。

## 26. 参考资料

- 赛事规则：仓库上级目录中的 2025 多旋翼无人机集群协同搜索仿真 PDF
- XTDrone 使用文档：https://www.yuque.com/xtdrone/manual_cn
- XTDrone 集群运动规划：https://www.yuque.com/xtdrone/manual_cn/swarm_motion_planning
- EGO-Planner-Swarm：https://github.com/ZJU-FAST-Lab/ego-planner-swarm
- FUEL：https://github.com/HKUST-Aerial-Robotics/FUEL
- RACER：https://github.com/Robotics-STAR-Lab/RACER
- FIESTA：https://github.com/HKUST-Aerial-Robotics/FIESTA
- Fast-Planner：https://github.com/HKUST-Aerial-Robotics/Fast-Planner
- Voxblox：https://github.com/ethz-asl/voxblox

正式引入任何第三方依赖前，必须重新核对具体提交、许可证和 ROS 接口，并更新 [THIRD_PARTY.md](THIRD_PARTY.md) 与 ownership 清单。本文不是法律意见。
