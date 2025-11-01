#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS 相关库
import rospy
from sensor_msgs.msg import Image                   # ROS 图像消息类型
from cv_bridge import CvBridge                     # OpenCV <=> ROS 图像转换桥
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox  # 自定义目标检测消息类型
import sys 

# 图像与AI库
import cv2
import numpy as np
from ultralytics import YOLO  # ✅ Ultralytics YOLOv8/YOLO11n 模型接口
import torch

# --------------------------------------------------
# 初始化 YOLO 模型
# 'yolo11n.pt' 是你训练好的 YOLO 模型文件
# ultralytics 库会自动处理推理等流程
# --------------------------------------------------
if torch.cuda.is_available():
    device = 'cuda'
    rospy.loginfo(f"GPU可用，使用设备: {torch.cuda.get_device_name(0)}")
else:
    device = 'cpu'
    rospy.logwarn("GPU不可用，将使用CPU运行，性能会受影响")



model = YOLO('yolo11n_942.pt')  # 请确保路径正确，文件存在
model.to(device)  # 强制模型使用GPU或CPU

# 设置最小置信度阈值（低于此数值的检测框将被过滤掉）
CONF_THRESH = 0.4

# OpenCV 与 ROS 图像转换桥
bridge = CvBridge()

# --------------------------------------------------
# 图像订阅回调函数
# 每当接收到 ROS 图像时，执行以下函数
# --------------------------------------------------
def image_callback(msg):
    # 1. 将 ROS 图像消息转换为 OpenCV 格式（BGR）
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # 2. 使用 Ultralytics YOLO 模型进行推理
    # predict 返回的是一个列表，每一项是一个 `Results` 对象（支持 batch）
    results = model.predict(source=frame, conf=CONF_THRESH, verbose=False, device=device)

    # 3. 创建并初始化 ROS BoundingBoxes 消息
    boxes_msg = BoundingBoxes()
    boxes_msg.header = msg.header  # 使用图像的时间戳和坐标系

    # 4. 获取推理结果（这里只处理 batch 中的第 0 帧）
    result = results[0]

    # 5. 如果没有检测结果（即 boxes 为空），直接发布空消息并返回
    if result.boxes is None:
        pub.publish(boxes_msg)
        return

    # 6. 从 result 中提取检测框信息
    boxes = result.boxes.xyxy.cpu().numpy()   # 检测框 [x1, y1, x2, y2]
    confs = result.boxes.conf.cpu().numpy()   # 置信度
    classes = result.boxes.cls.cpu().numpy()  # 类别索引

    # 7. 遍历每个检测结果，构建 ROS BoundingBox 消息
    for i in range(len(boxes)):
        cls_id = int(classes[i])              # 获取类别索引
        # ✅ **修改点：只处理类别索引为 0 到 4 的情况**
        # 如果类别是 'female' (5) 或 'fire hydrant' (6)，则跳过此次循环
        if cls_id not in [0, 1, 2, 3, 4]:
            continue
        x1, y1, x2, y2 = boxes[i]             # 获取边界框坐标
        conf = confs[i]                       # 获取置信度
        label = f"{model.names[cls_id]}{cls_id}"           # 将类别索引转换为类别名

        # 构建单个 BoundingBox 消息
        bbox = BoundingBox()
        bbox.Class = label                    # 类别名（如 'person'）
        bbox.probability = float(conf)        # 置信度
        bbox.xmin = int(x1)
        bbox.ymin = int(y1)
        bbox.xmax = int(x2)
        bbox.ymax = int(y2)

        # 添加到 BoundingBoxes 列表中
        boxes_msg.bounding_boxes.append(bbox)

    # 8. 发布检测结果到 ROS 话题
    pub.publish(boxes_msg)

# --------------------------------------------------
# ROS 节点初始化入口
# --------------------------------------------------
if __name__ == '__main__':
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    # 初始化 ROS 节点，名称为 yolo11n_pedestrian_detector
    rospy.init_node(f'yolo11n_pedestrian_detector_{vehicle_type}_{vehicle_id}')

    # 创建发布器，发布检测框结果到 /yolo11n/bounding_boxes
    pub = rospy.Publisher(vehicle_type+'_'+vehicle_id+'/yolo11n/bounding_boxes', BoundingBoxes, queue_size=10)

    # 订阅摄像头图像话题，根据仿真环境修改话题名
    rospy.Subscriber(vehicle_type+'_'+vehicle_id+'/realsense/depth_camera/color/image_raw', Image, image_callback, queue_size=1, buff_size=2**24)

    # 日志输出
    rospy.loginfo("YOLO11n Pedestrian Detector (using ultralytics) started.")

    # 进入 ROS 循环，等待回调执行
    rospy.spin()
""" 
依赖安装
pip install ultralytics opencv-python

运行
roscore
rosrun your_package_name yolo11n.py
"""
