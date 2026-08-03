# 起飞完成与人物跟踪门控设计

## 背景与根因

完成三路 MUX 和后置安全过滤后，真实六机验收中只有 5 架无人机完成起飞。3 号机高度停留在约 2.31 米，最终触发：

```text
Takeoff incomplete; keeping zero takeoff input selected.
```

运行日志 `logs/competition-clean/launch-20260803-205852-G4ZBIU.log` 表明，起飞开始约 0.4 秒后，3 号机的 tracking 节点识别并锁定人物，随后将该机 MUX 从 takeoff 输入切换到 external 输入。`fly_takeoff` 仍在发布爬升速度，但这些命令已不再被 MUX 转发，因此该机无法到达 3 米。

当前 `fly_takeoff` 与 tracking 之间没有“全机起飞已经完成”的状态契约。Launch 中的节点书写顺序也不代表运行时顺序，不能依靠延迟启动规避竞争。

## 目标与边界

本次改动建立一个失效默认关闭的起飞完成门控：六机全部到达目标高度且全部成功交给 navigator 前，任何 tracking 节点都不能锁定人物或切换 MUX。

本次只修改队伍自有的 ROS 包、测试和文档，不修改：

- PX4、XTDrone 或 Gazebo；
- EGO-Planner-Swarm；
- `src/gazebo_ros_actor_plugin` 中逐字节核验的第三方源码；
- 官方 world、人物模型或无人机基础模型；
- 已建立的 takeoff、navigator、external 三路 MUX 和后置 `safety_filter` 拓扑。

本次不引入统一控制权仲裁器，不接入 EGO、地图、前沿搜索或任务分配。

## 状态契约

新增全局话题：

| 话题 | 类型 | 发布者 | 订阅者 | 语义 |
| --- | --- | --- | --- | --- |
| `/swarm/takeoff_complete` | `std_msgs/Bool` | `confident_takeoff_node` | 每架无人机的 `tracking_node` | `true` 表示全机起飞和 navigator 交权均已成功 |

发布器必须启用 ROS 1 latch。`fly_takeoff` 构造完成后立即发布 `false`，确保后启动或重启的 tracking 节点能够立刻获得当前状态，而不是根据节点启动顺序猜测。

ROS 1 的锁存值只在发布节点存活期间有效。因此成功发布 `true` 后，`confident_takeoff_node` 必须停止发送飞行命令但保持空闲存活，直至整套仿真关闭。失败路径可以退出，因为没有收到状态或发布者消失都按 `false` 处理。

只有以下条件全部成立时才允许发布 `true`：

1. 六架无人机均已收到位姿并到达允许误差内的目标高度；
2. 起飞流程没有超时或提前退出；
3. 六架无人机的 MUX 均已成功切换到各自 navigator 输入。

MUX 服务不可用、起飞超时、任一无人机未到达高度或部分 navigator 交权失败时，话题保持 `false`。部分交权失败仍沿用现有逻辑，将全机回滚到零速度 takeoff 输入。

## Tracking 门控行为

每个 tracking 状态机启动时将 `takeoff_complete` 初始化为 `false`。收到 `true` 前，状态机保持 `IDLE`，忽略本轮可见人物列表，并且不得执行以下副作用：

- 调用目标锁定服务；
- 发布 `PAUSE`；
- 将 MUX 切换到 external；
- 发布人物跟踪速度命令。

收到 `true` 后，tracking 恢复现有 IDLE、DETECTING、DASH、TRACKING、LOST 状态流，不改变人物优先级、识别确认时间、控制算法或丢失目标后的返航行为。

如果运行中状态从 `true` 变回 `false`，tracking 必须进入失效关闭状态：释放已经锁定的目标、清空跟踪内部状态并回到 `IDLE`。它不主动选择 takeoff 或 navigator 输入，也不发布 `PAUSE` 或 `RESUME`；控制权恢复由发出 `false` 的起飞流程负责，避免门控成为第二套 MUX 仲裁器。

重复收到相同状态必须是幂等操作，不得重复释放目标或产生额外控制切换。

## 代码边界

`fly` 包负责事实来源：它知道全机高度完成状态和 navigator 交权结果，因此只由它发布 `/swarm/takeoff_complete`。

`tracking` 包负责执行门控：状态订阅更新一个明确的布尔状态；状态机的更新入口在处理人物列表前检查该状态。门控逻辑不得放进 YOLO、目标查询服务或 MUX，避免感知层与飞行阶段耦合。

队伍 Launch 只负责使用统一话题名称，不依赖 sleep、节点书写顺序或进程退出状态建立同步。

## 异常处理

- 没有收到话题：按 `false` 处理，tracking 保持关闭。
- `fly_takeoff` 重启：新实例首先锁存 `false`，所有 tracking 节点随即释放目标并停止接管。
- 起飞节点失败退出：最后锁存值仍为 `false`，不会误开放 tracking。
- tracking 重启：订阅后立即获得锁存值；在获得消息前仍按 `false` 处理。
- navigator 交权部分成功：全机回滚 takeoff，门控不开放。

## 测试与验收

严格按测试先行实现。自动测试至少覆盖：

1. `fly_takeoff` 启动时发布锁存的 `false`；
2. 起飞未完成、超时和 MUX 交权失败路径不会发布 `true`；
3. 仅在全机起飞且 navigator 交权全部成功后发布 `true`；
4. tracking 默认关闭，存在人物目标时也不申请目标、不暂停任务、不切换 MUX；
5. 收到 `true` 后恢复原有目标申请和跟踪状态流；
6. 从 `true` 回退到 `false` 时恰好释放一次目标、清空状态，并且不切换 MUX；
7. 重复状态消息不产生重复副作用；
8. 最终 XTDrone 速度话题仍只有对应 `safety_filter` 一个队伍发布者。

完成单元和集成测试后运行完整 `competition-clean` 验证。真实六机验收必须确认：

- 六架无人机均到达目标高度；
- 全机成功切换到 navigator 后，门控才变为 `true`；
- 门控为 `false` 期间，即使相机已经检测到人物，也没有 tracking 目标锁定或 external MUX 切换；
- 门控开放后，人物识别和跟踪接管能够正常发生；
- `scripts/smoke_competition_clean.sh` 继续通过；
- PX4、XTDrone、Gazebo、EGO 和官方模型目录未改变。
