# 第三方来源与许可证

本页记录仓库中不由队伍原创的可分发内容。来源和版本以 `src/competition_compliance/config/ownership.json` 为准；许可证只适用于对应第三方内容，不扩展到队伍代码。

| 内容 | 来源/版本 | 许可证与核验 |
| --- | --- | --- |
| `darknet_ros_msgs` | `leggedrobotics/darknet_ros`，1.1.4 | BSD；消息、构建文件和 `package.xml` 按 ownership 清单核验 |
| `gazebo_ros_actor_plugin` | XTDrone，提交 `8e88116dc15a19e5eba06300897fcfec4ab2da11` | Apache-2.0；与外部 XTDrone 树逐字节核验 |
| `ActorCollisionsPlugin` | XTDrone，提交 `8e88116dc15a19e5eba06300897fcfec4ab2da11` | Apache-2.0；源文件来自外部只读树，在本工作空间 out-of-tree 构建 |
| `EGO-Planner-Swarm`（外部运行依赖，不进入本仓库） | `ZJU-FAST-Lab/ego-planner-swarm`，提交 `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20` | GPL-3.0；`scripts/check_ego_external.py` 核验提交、干净工作树和接口文件 |
| `src/yolo/yolo11n_942.pt` | 仓库既有 YOLO 权重文件；原有说明未给出可核实的发布授权 | `NOASSERTION`；再分发前须取得权利人许可 |

Actor 插件的源码副本只用于本仓库构建接口，不能据此修改 XTDrone 外部源。PX4、XTDrone、Gazebo 和官方模型本身是安装环境依赖，不是本仓库重新许可的内容。

队伍包的许可证保持原声明：`fly`、`simple_navigator` 使用 BSD，`pose_init` 使用 Apache-2.0；其余队伍包中原先只有占位许可的包使用 `LicenseRef-Team-Code`。这个标识只表示队伍代码的保留权利，不授予新的开源许可，也不覆盖任何第三方文件。

`src/ego_fusion_search/local_mapping` 和 `src/ego_fusion_search/search_msgs` 是队伍自有源码。`local_mapping` 复用系统安装的 ROS、PCL 和 OpenCV，并依赖上表已登记的 `darknet_ros_msgs`。`search_msgs` 只使用 ROS 的 message generation 与 `std_msgs`。这两个包没有复制新的第三方源码进入仓库。

提交或公开 issue 时不要上传完整 PX4 归档、外部模型库、Python 虚拟环境或未获授权的权重副本；只提供版本、哈希、日志和必要的错误片段。
