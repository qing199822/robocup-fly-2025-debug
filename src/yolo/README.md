# yolo

## (1) 目标识别

安装pytorch(GPU版本)

```bash
pip install torch==2.1.2 torchvision==0.16.2 torchaudio==2.1.2 --index-url https://download.pytorch.org/whl/cu121
```

其中 我们的py版本是3.8，安装时会报networkx版本于py版本冲突，这时我们选一个低一点的版本安装即可

```bash
pip install networkx==2.8.8
```

安装yolov11

```bash
pip install ultralytics
```

检测能不能在GPU使用

```python
import torch

# 检查 CUDA 是否可用
if torch.cuda.is_available():
    try:
        # 1. 定义一个设备为 CUDA (即 GPU)
        device = torch.device("cuda")
        print("已成功定义 CUDA 设备。")

        # 2. 在 GPU 上创建一个张量
        x = torch.tensor([1.0, 2.0, 3.0], device=device)
        print("成功在 GPU 上创建张量:")
        print(x)

        # 3. 检查张量所在的设备
        print(f"张量所在的设备: {x.device}")

    except Exception as e:
        print(f"在尝试使用 GPU 时发生错误: {e}")
else:
    print("CUDA 不可用，无法在 GPU 上创建张量。")
```

## (2) 坐标解算

深度相机，高度不宜过高，追踪一定要平稳，默认云台角度为-30°

`bbox2coord_node.py` 将 RGB 检测框与深度、`CameraInfo` 和 TF 匹配，只有人物 ID、深度和坐标转换均有效时才向对应裁判话题发布 `actor_msgs/ActorInfo`。

成功发布 `ActorInfo` 后，节点才会发布队伍内部心跳：

```text
/<vehicle>/coordinate_broadcast/heartbeat
look_up/CoordinateBroadcastHeartbeat
```

心跳字段为同一次发布的时间戳、本机 `vehicle_name` 和检测类别 `target_id`。发布顺序严格为 `ActorInfo` 在前、心跳在后；如果裁判消息发布抛出异常，不得发送心跳。tracking 使用该心跳计算规则要求的 15 秒连续有效广播，不能用检测框可见时间代替。

聚焦测试：

```bash
python3 -m unittest tests.test_coordinate_reporting tests.test_camera_geometry -v
python3 -m py_compile src/yolo/coordinate_reporting.py src/yolo/bbox2coord_node.py
```
