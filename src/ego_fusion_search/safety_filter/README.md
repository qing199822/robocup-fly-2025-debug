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
