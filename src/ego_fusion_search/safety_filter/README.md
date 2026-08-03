# safety_filter

`safety_filter` 是队伍控制链的最后一道速度安全门。每架无人机只允许该节点发布 XTDrone 最终 `cmd_vel_flu` 话题。

## 输入与输出

对 `typhoon_h480_N`：

- 输入 `/typhoon_h480_N/control/raw_cmd_vel`：MUX 选中的 `geometry_msgs/Twist`；
- 输入 `/typhoon_h480_N/global_odom`：队伍统一坐标中的 `nav_msgs/Odometry`；
- 输出 `/xtdrone/typhoon_h480_N/cmd_vel_flu`：经过检查和限幅的最终速度；
- 输出 `/typhoon_h480_N/safety/status`：`OK` 或稳定的故障码。

## 当前安全规则

- 原始速度指令超过 `command_timeout` 时输出零速度；
- odometry 超过 `odom_timeout` 时输出零速度；
- NaN、Inf 或无效时间步输出零速度；
- 水平速度按向量模长限幅；
- 垂直速度、偏航速度和加速度受限；
- 达到高度边界时禁止继续越界的垂直速度。

默认参数集中在 `config/default.yaml`。本阶段尚未实现基于深度图和停止距离的障碍制动，不能将该功能写成“已经完成”。

## 控制所有权

takeoff、navigator 和 external 控制源只能发布各自的 MUX 输入。它们不得直接发布 `/xtdrone/typhoon_h480_N/cmd_vel_flu`。后续 EGO adapter 只会替换 navigator 输入发布者，不增加最终发布者。

```text
takeoff -----+
navigator ---+-> pose_cmd_mux -> raw_cmd_vel -> safety_filter -> cmd_vel_flu
external ----+
```

MUX 初始选择 takeoff。只有六机起飞全部完成后，`fly_takeoff` 才选择 navigator；任何超时、部分起飞失败或部分交权失败都保持或回滚到零速度 takeoff。

全局锁存话题 `/swarm/takeoff_complete` 默认发布 `false`。只有六机全部到高且 navigator 交权全部成功后才发布 `true`；此时 `confident_takeoff_node` 保持空闲存活以保存锁存值，但不再发送飞行命令。tracking 在门控开放前不能锁目标或选择 external，门控重新关闭时只释放目标和清空状态，不主动切换 MUX。

仿真运行时检查六个最终话题：

```bash
python3 scripts/check_final_control_publishers.py \
  --count 6 --vehicle-type typhoon_h480
```

健康状态应输出 `PASS final control topics have one safety_filter publisher each`。完整运行态检查使用 `bash scripts/smoke_competition_clean.sh`，并且报告中必须包含 `PASS takeoff gate /swarm/takeoff_complete`。
