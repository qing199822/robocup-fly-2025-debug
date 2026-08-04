# look_up

该队伍包提供中央人物锁，避免多架无人机同时跟踪同一人物。支持的目标为 `green0`、`blue1`、`brown2`、`white3`、`red4` 和 `red5`。

## 目标状态

每个人物有三个状态：

- `AVAILABLE`：可以由任一无人机申请。
- `TRACKED`：已经被一架无人机申请，其他无人机的申请会失败。
- `COMPLETED`：已经完成规则要求的坐标广播，本场比赛中不能再次申请。

`/lookup/request_<target_id>` 只允许 `AVAILABLE -> TRACKED`。`/lookup/release_<target_id>` 将普通锁恢复为 `AVAILABLE`；对 `COMPLETED` 调用 release 会返回成功，但不会改变完成状态。`/lookup/complete_target` 只允许已锁定目标进入 `COMPLETED`，重复完成同一目标是幂等成功。

## 队伍内部消息

`CoordinateBroadcastHeartbeat.msg` 包含时间戳、无人机名和人物 ID。它只用于 tracking 计算连续有效广播进度，不替代发给裁判系统的 `ActorInfo`。

`spawn_mux_swarm.launch` 启动目标服务和多机 MUX，用于在 navigator、takeoff 与 tracking external 控制输入之间切换。PX4、XTDrone 和 Gazebo 不在本包维护范围内。
