# Competition-Clean 合规边界

## 规则与检查点

- 飞机基线：XTDrone `typhoon_h480_realsense`，提交 `8e88116dc15a19e5eba06300897fcfec4ab2da11`。
- Realsense 成像、量程和光学参数按 XTDrone 原文件逐字节校验。
- 唯一允许的模型差异是单个 `model://realsense_camera` include 的 direct `<pose>`；固定 joint 的 parent 必须是 `base_link`。
- 每次启动都强制执行快速预检，目标耗时不超过 2 秒；失败即停止启动。
- 完整验证应在环境变更后、参赛前和发布前执行。
- PX4、XTDrone、Gazebo 及官方模型目录是只读输入，构建前后均检查哈希；不向这些目录写入。
- 改动安装位姿后，队伍须检查不遮挡机体、不穿透机架、朝向满足任务需要；提交比赛前还须接受裁判审查。

## 官方文件清单

哈希来自 `src/competition_compliance/config/official_manifest.json`，算法为 SHA-256。

| 根变量 | 相对路径 | SHA-256 |
| --- | --- | --- |
| PX4_DIR | Tools/sitl_gazebo/models/typhoon_h480/typhoon_h480.sdf | `4f3ae25801c704e1f9e640eaf1717e6a06a688256ad8f6ad5a0872a2843c4680` |
| PX4_DIR | Tools/sitl_gazebo/worlds/robocup.world | `b17daad2b9662760aba6defbd1637214e6d4832e3828ec13ca342f544c6e0b98` |
| PX4_DIR | launch/single_vehicle_spawn_xtd.launch | `05bb251d1bebf28890cc03191a7fbbe0e121a5e2929a18b8968eb3d9ac071e7e` |
| XTDRONE_DIR | sitl_config/models/typhoon_h480/typhoon_h480.sdf | `1346f71a33130e3f5634b1513cc5598d1dc2693fdf30d13c2cf9dda2ef2cd29e` |
| XTDRONE_DIR | sitl_config/models/typhoon_h480_realsense/typhoon_h480_realsense.sdf | `3b056f3676e8f47b90421c5357eca8154e6686304855eb14467aa82bf60ddd46` |
| XTDRONE_DIR | sitl_config/models/realsense_camera/realsense_camera.sdf | `0745c705ac3a90cf16529a9b49729d34f49ce7b457998a4d3cc3f2fb6aab921c` |
| XTDRONE_DIR | sitl_config/models/realsense_camera/model.config | `87df068cf0db6a135c585431ed19060eeb10c49a7f33f21c292306004a832366` |
| XTDRONE_DIR | sitl_config/models/realsense_camera/meshes/realsense_camera.dae | `df88d7930c0f0fdcc80c9eb3e19f2af8296a965e8e6db055ef3bb8e0df14fb85` |
| XTDRONE_DIR | communication/multirotor_communication.py | `64c13f6ad6de9181208cf584ac1b796d49d4f153935369b41e64a4b893a74d27` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/actor_collisions/ActorCollisionsPlugin.cc | `e15f07b4a9cc19db1a05dd1aafd1b81557b2badf728cc28d666500034b34e499` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/actor_collisions/ActorCollisionsPlugin.hh | `78db47b17157eeb97676fc0ceecc95662dd1a8018c3730c492962ca431b61c29` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/CMakeLists.txt | `605eb23f6283b21fb67aa2efc3ddf0ca46dd79e292d05e8869f0638513efd786` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/LICENSE | `c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/README.md | `71365aa2b8c92ae0dcfbfb970132ead41e224a8489d1ffe4fb2706394278ddb7` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/include/actor_plugin_ros/ActorPluginRos.hpp | `c10b714a548e3e1544df9b224e91a8d8d58acae4e7d7f45f54e1446ca042c411` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/package.xml | `6e662ad661893ded902e6035196328e902d800c5301c431b7c3a321ab3eac595` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/res/waving.dae | `7330302c492898d37fac0cff1cbd26b4381a7254fa14ae9780d7d0b9603a4db7` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin/src/ActorPluginRos.cpp | `ac4bbe7b18aa7a89a50a1daba1648bd3563649bff67ac9b5868018d18664712c` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/CMakeLists.txt | `17e2d8b2c045a92d31b022bb4cf747d90911b288d73d7ab3715ae3a97b1e1b51` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/msg/ActorInfo.msg | `6a96273e5b133de9b94efd82940fb2fdb357234837d7a6a3dc05fa4878ff4ba1` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/msg/ActorMotion.msg | `f6f0f451411ba92053711251142483bda50cb1f06c41be4dfd45ad9b49c150ac` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/package.xml | `66f19fc8fb4fa7ae5d5e6d49475798a79976b865f5798d18e3cd0d9bc1c6601a` |
| XTDRONE_DIR | sitl_config/gazebo_plugin/gazebo_ros_actor_plugin/gazebo_ros_actor_cmd_plugin_msgs/srv/ToggleActorWaving.srv | `d73e9d1a650517a232fcc9f41500815544035edcfc815fef640dba5d75967abd` |
| PX4_DIR | Tools/setup_gazebo.bash | `20f6f1c974aa6ad876b77608f7bbfc1f30219e5fb9c7e6af0f5ba0c3016889e0` |
| PX4_DIR | build/px4_sitl_default/bin/px4 | `a20d61834431c521275f7d97c7c6efaab773fdb77541e78513bde54e97074be2` |
| XTDRONE_DIR | sitl_config/models/walker/walk_0.dae | `4dea2476b652a575cbed75ee2537f80d08cc1ccdf923d6055cdf6fd83dc88665` |
| GAZEBO_MODELS_DIR | cessna/model.sdf | `723beb85db3b6efd59f5c72f19245cbea60a05ed2f947b9ddb1913cdb052f8e9` |
| XTDRONE_PYTHONPATH | pyquaternion/__init__.py | `e0ba598f61b4531f0bff1a2bf1740280e950a071cd4b96525bdac02cb46c745c` |

## 证据和裁判展示

完整验证生成 `competition-artifacts/static-compliance.json` 和 `competition-artifacts/post-build-compliance.json`；启动与 smoke 日志写入 `logs/competition-clean/`。预检会打印六个数值的 mount pose（x、y、z、roll、pitch、yaw），并将同一值写入临时模型和静态 TF。向裁判展示该终端输出、`sensor_mount.yaml`、对应日志和本页清单；不要把临时模型当作新的官方飞机模型提交。
