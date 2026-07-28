# Gazebo 人物模型抖动修复设计

## 背景与根因

比赛使用的官方 `robocup.world` 包含 6 个 Gazebo actor。每个 actor 同时加载：

- `libActorCollisionsPlugin.so`，为骨骼链接启用物理碰撞；
- `libros_actor_cmd_pose_plugin.so`，维护人物初始位置并接收 ROS 运动命令。

在当前 Gazebo Classic 11.15.1 环境中，即使 `/actor_N/cmd_motion` 没有发布者，人物位置仍会在初始位置和世界原点之间反复跳变。连续读取 `actor_1` 得到过 `(70, 22)`、`(0, 0)` 交替出现。使用官方 world 的临时副本，仅移除 `libActorCollisionsPlugin.so` 后，连续采样始终为 `(70, 22)`。因此抖动来自人物碰撞插件与 ROS 人物插件对 actor 位姿的逐帧争抢，而不是队伍导航、识别或控制节点发错命令。

## 目标与边界

修复后的 6 个人物必须稳定保留在各自初始位置，继续显示、参与相机成像和目标识别，并保留原有 ROS 人物控制插件。人物不再参与物理碰撞，这是本次修复接受的明确取舍。

以下官方或第三方输入保持逐字节不变：

- PX4 目录中的 `Tools/sitl_gazebo/worlds/robocup.world`；
- XTDrone 目录及人物模型；
- `src/gazebo_ros_actor_plugin` 中的第三方源码；
- Gazebo 系统组件。

不改变无人机模型、传感器参数、任务航点或队伍飞行算法。

## 方案

在队伍自有的 `competition_compliance` 包中增加 world 准备脚本。脚本使用 XML 解析器读取已经通过官方 manifest 校验的 `robocup.world`，将内容写入本次运行的私有临时目录，并且只删除满足以下全部条件的 XML 元素：

1. 元素位于 `/sdf/world/actor/plugin`；
2. `filename` 严格等于 `libActorCollisionsPlugin.so`。

生成结果必须满足：

- 恰好存在 6 个 actor；
- 恰好移除 6 个人物碰撞插件；
- 仍存在 6 个 `libros_actor_cmd_pose_plugin.so`；
- 每个人物的名称、皮肤、动画和 `init_pose` 保持不变；
- 输入文件不被写入；
- 输出是可解析的 XML/SDF 文件。

若输入结构不符合这些预期，脚本必须报错并阻止启动，不能生成部分修改的场景继续运行。

## 启动与清理

`1.sh` 继续先执行快速合规检查。检查通过后，在现有 `RUN_TMP_DIR` 中生成：

```text
robocup.world
typhoon_h480_realsense.sdf
```

启动器将临时 world 的绝对路径通过 launch 参数传给 `robocup_zzufly.launch`。launch 不再只依赖默认的 PX4 world 路径，但默认值仍保留，方便单独阅读和兼容现有调用方式。

临时 world 与临时无人机模型共用现有运行目录和清理流程。正常退出、启动失败和收到终止信号时，都由当前有界清理逻辑删除，不增加按进程名清理或外部目录写入。

## 测试与验证

先增加失败测试，再实现最小修复。自动测试覆盖：

- 生成器只删除 actor 下指定文件名的插件；
- 同名插件若位于非 actor 节点下不会被误删；
- actor 数量、碰撞插件数量或 ROS 插件数量异常时拒绝输出；
- 6 个人物的 `init_pose` 和关键动画配置保持不变；
- 输入文件内容和哈希不变；
- `1.sh` 将生成的 world 传给 launch，并沿用现有临时目录清理；
- `robocup_zzufly.launch` 声明并使用 world 参数。

静态测试通过后运行仓库 Python 回归、相关 Catkin 测试和合规检查。最终使用 Gazebo 做最小运行验证：确认 6 个 `/actor_N/cmd_motion` 订阅存在且无发布者时，人物连续位姿不再跳到原点；随后运行完整六机 smoke，确认相机、识别和飞行启动链路未受影响。
