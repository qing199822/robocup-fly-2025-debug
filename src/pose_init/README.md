# pose_init

由于比赛中不能直接订阅位姿真值，而我们的也没有使用激光雷达定位，因此我们将根据无人机的odom和imu数据经过处理后发布无人机在世界坐标系的位姿

```xml
/mavros/local_position/pose #无人机姿态
/mavros/odometry/in # 无人机里程计信息
```

无人机初始在地图坐标系的位置已知的

发布的话题是：(相当于位姿真值)

```xml
/mavros/vision_pose/pose
/mavros/vision_odom/odom
```

