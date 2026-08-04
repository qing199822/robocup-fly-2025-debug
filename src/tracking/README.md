# tracking

该队伍包根据 YOLO 检测框控制无人机跟踪人物。控制器使用检测框面积估计远近，用框中心误差控制横向和偏航，并用包含匀速、匀加速和高机动模型的 IMM 卡尔曼滤波器稳定目标状态。当前 C++ 控制链直接使用控制器输出；已有四层低通实现未接入最终输出，因为它会造成明显响应延迟。

## 状态与控制门控

状态机包含 `IDLE`、`DETECTING`、`DASH`、`TRACKING`、`LOST` 和 `RETURNING`。只有 `/swarm/takeoff_complete=true` 且本机 `/<vehicle>/mission/active=true` 时，tracking 才能申请人物、暂停巡逻或切换到 external MUX 输入。任一门控关闭时只释放当前人物并清理本次状态，不主动切换 MUX。

两个红色人物在检测结果中没有外观差异，因此按 `red4`、`red5` 顺序向 `look_up` 申请。其他人物使用检测类别名申请。

## 连续坐标广播与恢复巡逻

赛事条件是连续 15 秒成功广播正确人物 ID 和坐标，不是人物连续可见 15 秒。`bbox2coord_node.py` 每次成功发布裁判 `ActorInfo` 后，在本机话题发布结构化心跳：

```text
/<vehicle_type>_<vehicle_id>/coordinate_broadcast/heartbeat
look_up/CoordinateBroadcastHeartbeat
  header.stamp
  vehicle_name
  target_id
```

状态机只接受两个门控均打开、状态为 `DASH` 或 `TRACKING`、无人机名和当前锁定人物均匹配的心跳。默认规则参数位于 `config/params.yaml`：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `broadcast_confirmation_duration` | 15.0 s | 连续有效广播确认时间 |
| `broadcast_heartbeat_timeout` | 0.5 s | 超过该间隔会重新累计连续时间 |
| `tracking_session_timeout` | 20.0 s | 单次跟踪会话上限 |
| `retry_cooldown` | 5.0 s | 未确认超时后本机重试同一人物的等待时间 |

人物消失且已经确认 15 秒时，状态机进入 `RETURNING` 并完成目标；人物消失但尚未确认时只释放目标。人物一直存在到 20 秒上限时，已确认目标进入 `COMPLETED`，未确认目标只释放并进入本机 5 秒冷却。

返回顺序固定为：持续发布零 external 命令、成功切回 navigator MUX、完成或释放中央目标锁、只发送一次 `RESUME`。MUX 切换失败时会重试，切换成功前不得发送 `RESUME`；完成服务最多尝试 3 次，仍失败时退回普通释放，不能声称目标已完成。

## 验证

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_tracking
catkin_test_results build/test_results/tracking
```

自动化测试覆盖计时、错误心跳过滤、MUX 重试、完成/释放、单次 `RESUME`、冷却和门控关闭。自动化通过不等于真实六机全航程安全；当前全航程仍有已知碰撞问题。
