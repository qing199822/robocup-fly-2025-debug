#!/usr/bin/env python
# coding: utf-8
from collections import deque
import math
import re
import sys
import threading

import rospy
import tf2_ros
import tf2_geometry_msgs  # Registers PointStamped transforms with tf2_ros.

# 訊息類型
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_msgs.msg import TFMessage
from darknet_ros_msgs.msg import BoundingBoxes
from actor_msgs.msg import ActorInfo

# 工具
from cv_bridge import CvBridge
from camera_geometry import (
    DepthSample,
    deproject_pixel,
    depth_image_to_meters,
    roi_mean_depth,
    select_closest_depth_sample,
)

class CoordinateEstimator:
    """
    處理目標檢測結果，並估算其在世界座標系中的位置。
    """
    ROI_HALF_SIZE = 2  # 5x5 的感興趣區域（ROI）

    def __init__(self):
        """
        初始化節點、參數、訂閱者和發布者。
        """
        # 1. 節點初始化
        self.robot_name = sys.argv[1]
        rospy.init_node(f'coordinate_estimator_node_{self.robot_name}', anonymous=False)
        rospy.loginfo(f"節點 'coordinate_estimator_node_{self.robot_name}' 啟動中...")

        # 2. 工具和狀態變數
        self.cv_bridge = CvBridge()
        self.sensor_lock = threading.Lock()
        self.latest_camera_info = None
        self.active_tracking_id = None
        self.last_tracking_update_time = rospy.Time(0)
        
        # 3. 讀取ROS參數
        self._load_ros_params()
        self.depth_samples = deque(maxlen=self.depth_queue_size)

        # 4. TF2 設置
        self.transform_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_buffer)

        # 5. Actor 配置映射
        self.actor_definitions = {
            0: {'color': 'green', 'topic': '/actor_green_info'},
            1: {'color': 'blue',  'topic': '/actor_blue_info'},
            2: {'color': 'brown', 'topic': '/actor_brown_info'},
            3: {'color': 'white', 'topic': '/actor_white_info'},
            4: {'color': 'red',   'topic': '/actor_red1_info'},
            5: {'color': 'red',   'topic': '/actor_red2_info'}
        }

        # 6. 初始化ROS發布者
        self.actor_publishers = self._create_actor_publishers()
        
        # 7. 初始化ROS訂閱者
        self._setup_subscriptions()

        rospy.loginfo("座標估算器初始化完成，等待訊息...")

    def _load_ros_params(self):
        """從參數伺服器載入時間限制和座標系名稱。"""
        self.target_frame_id = rospy.get_param("~target_frame", "ground_plane")
        configured_sensor_delta = rospy.get_param("~maximum_sensor_delta", 0.15)
        try:
            self.maximum_sensor_delta = float(configured_sensor_delta)
        except (TypeError, ValueError, OverflowError) as error:
            message = "~maximum_sensor_delta 必須是有限的非負數。"
            rospy.logfatal(message)
            raise ValueError(message) from error
        if not math.isfinite(self.maximum_sensor_delta) or self.maximum_sensor_delta < 0:
            message = "~maximum_sensor_delta 必須是有限的非負數。"
            rospy.logfatal(message)
            raise ValueError(message)

        configured_queue_size = rospy.get_param("~depth_queue_size", 60)
        if (
            isinstance(configured_queue_size, bool)
            or not isinstance(configured_queue_size, int)
            or configured_queue_size <= 0
        ):
            message = "~depth_queue_size 必須是正整數。"
            rospy.logfatal(message)
            raise ValueError(message)
        self.depth_queue_size = configured_queue_size
        self.tracking_timeout_duration = rospy.Duration(rospy.get_param("~tracking_timeout", 1.0))

    def _create_actor_publishers(self):
        """根據 actor_definitions 字典創建並返回所有發布者。"""
        publishers = {}
        for actor_id, config in self.actor_definitions.items():
            topic = config['topic']
            publishers[actor_id] = rospy.Publisher(topic, ActorInfo, queue_size=1)
            rospy.loginfo(f"為 Actor ID {actor_id} 創建發布者，話題: {topic}")
        return publishers

    def _setup_subscriptions(self):
        """設置所有的ROS訂閱者。"""
        # TF 訊息訂閱
        rospy.Subscriber(f"/{self.robot_name}/tf", TFMessage, self._tf_callback)
        rospy.Subscriber("/tf_static", TFMessage, self._tf_static_callback)

        # 感測器和檢測結果訂閱
        rospy.Subscriber(f"/{self.robot_name}/realsense/depth_camera/depth/image_raw", Image, self._depth_image_callback, queue_size=1)
        rospy.Subscriber(
            f"/{self.robot_name}/realsense/depth_camera/color/camera_info",
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )
        rospy.Subscriber(f"/{self.robot_name}/yolo11n/bounding_boxes", BoundingBoxes, self.bounding_box_callback, queue_size=1)
        
        # 追踪狀態訂閱
        tracking_topic = f"/{self.robot_name}/tracking_node/yolo_human_tracking_{self.robot_name}"
        rospy.Subscriber(tracking_topic, String, self._tracking_status_callback)

    # --- 回呼函數 ---

    def _tf_callback(self, tf_message):
        """處理動態TF變換訊息。"""
        for transform in tf_message.transforms:
            self.transform_buffer.set_transform(transform, "default_authority")

    def _tf_static_callback(self, tf_message):
        """處理靜態TF變換訊息。"""
        for transform in tf_message.transforms:
            self.transform_buffer.set_transform_static(transform, "default_authority")

    def _camera_info_callback(self, message):
        """只保留完整且可用的官方相機標定。"""
        try:
            matrix_is_valid = (
                len(message.K) == 9
                and all(math.isfinite(float(value)) for value in message.K)
                and float(message.K[0]) > 0.0
                and float(message.K[4]) > 0.0
            )
            dimensions_are_valid = message.width > 0 and message.height > 0
            frame_is_valid = bool(message.header.frame_id)
        except (AttributeError, TypeError, ValueError, OverflowError):
            matrix_is_valid = False
            dimensions_are_valid = False
            frame_is_valid = False

        if not (matrix_is_valid and dimensions_are_valid and frame_is_valid):
            rospy.logwarn_throttle(2, "CameraInfo 無效，忽略本次標定訊息。")
            return
        with self.sensor_lock:
            self.latest_camera_info = message

    def _depth_image_callback(self, image_msg):
        """驗證深度訊息並把完整樣本加入有界佇列。"""
        try:
            encoding = image_msg.encoding
            frame_id = image_msg.header.frame_id
            stamp = image_msg.header.stamp
            if not frame_id:
                raise ValueError("深度圖 frame_id 為空")
            if stamp.is_zero():
                raise ValueError("深度圖時間戳為零")
            stamp_seconds = float(stamp.to_sec())
            if not math.isfinite(stamp_seconds) or stamp_seconds <= 0.0:
                raise ValueError("深度圖時間戳必須是有限正數")
            converted_frame = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
            depth_meters = depth_image_to_meters(converted_frame, encoding)
            sample = DepthSample(
                depth_meters=depth_meters,
                stamp=stamp,
                stamp_seconds=stamp_seconds,
                frame_id=frame_id,
                encoding=encoding,
            )
        except Exception as e:
            rospy.logwarn_throttle(2, f"深度圖樣本無效，忽略本次訊息: {e}")
            return
        with self.sensor_lock:
            self.depth_samples.append(sample)

    def _tracking_status_callback(self, status_msg):
        """解析並更新當前追踪的目標ID。"""
        try:
            parts = status_msg.data.split(':')
            if len(parts) == 3 and parts[0] in ["TRACKING", "DASH"]:
                self.active_tracking_id = parts[1]  # 例如 "red4"
                self.last_tracking_update_time = rospy.Time.now()
            else:
                self.active_tracking_id = None
        except Exception as e:
            rospy.logwarn(f"解析追踪狀態時出錯: {e}")

    def bounding_box_callback(self, detections_msg):
        """
        處理檢測到的邊界框的主回呼函數。
        """
        with self.sensor_lock:
            depth_samples = tuple(self.depth_samples)
            camera_info = self.latest_camera_info
        if not depth_samples or camera_info is None:
            rospy.logwarn_throttle(2, "深度樣本佇列或 CameraInfo 尚未接收，跳過處理。")
            return

        detection_frame = detections_msg.header.frame_id
        camera_frame = camera_info.header.frame_id
        if not detection_frame or detection_frame != camera_frame:
            rospy.logwarn_throttle(2, "彩色檢測結果與 CameraInfo 座標系不一致，跳過處理。")
            return

        if detections_msg.header.stamp.is_zero():
            rospy.logwarn_throttle(2, "彩色檢測結果時間戳為零，跳過處理。")
            return

        try:
            selected_sample = select_closest_depth_sample(
                depth_samples,
                detections_msg.header.stamp.to_sec(),
                self.maximum_sensor_delta,
            )
        except ValueError as error:
            rospy.logwarn_throttle(2, f"感測器時間戳無效，跳過處理: {error}")
            return
        if selected_sample is None:
            rospy.logwarn_throttle(2, "沒有時間匹配的深度圖樣本，跳過處理。")
            return

        if selected_sample.frame_id != camera_frame:
            rospy.logwarn_throttle(2, "深度圖與 CameraInfo 座標系不一致，跳過處理。")
            return

        depth_height, depth_width = selected_sample.depth_meters.shape
        if camera_info.width != depth_width or camera_info.height != depth_height:
            rospy.logwarn_throttle(2, "CameraInfo 尺寸與深度圖不一致，跳過處理。")
            return

        if not self._is_tracking_active():
            rospy.loginfo_throttle(5, "目前無活躍追踪目標，不進行座標計算。")
            return

        for detection in detections_msg.bounding_boxes:
            # 只處理與當前追踪目標匹配的檢測框
            if detection.Class != self.active_tracking_id:
                continue

            try:
                # 1. 獲取ROI的平均深度
                center_u = int((detection.xmin + detection.xmax) / 2)
                center_v = int((detection.ymin + detection.ymax) / 2)
                
                try:
                    mean_depth = roi_mean_depth(
                        selected_sample.depth_meters,
                        center_u,
                        center_v,
                        self.ROI_HALF_SIZE,
                    )
                except ValueError as error:
                    rospy.logwarn(f"'{detection.Class}' 的深度 ROI 無效，跳過處理: {error}")
                    continue
                if mean_depth is None:
                    rospy.logwarn(f"無法獲取 '{detection.Class}' 在 ({center_u}, {center_v}) 的有效深度。")
                    continue

                # 2. 計算在相機座標系下的3D點
                try:
                    point_in_camera = self._calculate_3d_point(
                        center_u, center_v, mean_depth, camera_info, selected_sample.stamp
                    )
                except ValueError as error:
                    rospy.logwarn(f"'{detection.Class}' 的相機反投影失敗，跳過處理: {error}")
                    continue

                # 3. 轉換座標到目標座標系 (e.g., ground_plane)
                point_in_target_frame = self._transform_point(point_in_camera, self.target_frame_id)
                if point_in_target_frame is None:
                    continue
                
                # 4. 根據ID發布ActorInfo訊息
                self._publish_actor_info(detection.Class, point_in_target_frame)

            except Exception as e:
                rospy.logerr(f"處理邊界框 '{detection.Class}' 時發生未知錯誤: {e}")

    # --- 輔助方法 ---

    def _is_tracking_active(self):
        """檢查追踪狀態是否在超時範圍內保持活躍。"""
        is_active = (self.active_tracking_id is not None and
                     (rospy.Time.now() - self.last_tracking_update_time) < self.tracking_timeout_duration)
        return is_active

    def _calculate_3d_point(self, u, v, depth_in_meters, camera_info, depth_stamp):
        """根據像素座標和深度，計算在相機座標系中的3D點。"""
        x, y, z = deproject_pixel(u, v, depth_in_meters, camera_info.K)
        
        point = PointStamped()
        point.header.frame_id = camera_info.header.frame_id
        point.header.stamp = depth_stamp
        point.point.x = x
        point.point.y = y
        point.point.z = z
        return point

    def _transform_point(self, point_stamped, target_frame):
        """使用TF2將一個點從源座標系轉換到目標座標系。"""
        try:
            return self.transform_buffer.transform(point_stamped, target_frame, timeout=rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF座標變換失敗從 {point_stamped.header.frame_id} 到 {target_frame}: {e}")
            return None

    def _publish_actor_info(self, class_name, final_point):
        """根據目標類別名稱，填充並發布ActorInfo訊息。"""
        match = re.search(r'\d+', class_name)
        if not match:
            rospy.logwarn_throttle(5, f"無法從類別名稱 '{class_name}' 中提取ID。")
            return
        
        actor_id = int(match.group(0))

        if actor_id in self.actor_publishers:
            config = self.actor_definitions[actor_id]
            publisher = self.actor_publishers[actor_id]
            
            # 處理red4和red5的特殊映射
            if class_name == "red4":
                publisher = self.actor_publishers[4]
            elif class_name == "red5":
                publisher = self.actor_publishers[5]

            actor_message = ActorInfo(
                cls=config['color'],
                x=final_point.point.x,
                y=final_point.point.y
            )
            
            publisher.publish(actor_message)
            rospy.loginfo(f"發布 {class_name} ({config['color']}) 位置到話題 '{publisher.name}': "
                          f"x={actor_message.x:.3f}, y={actor_message.y:.3f}")
        else:
            rospy.logwarn(f"檢測到未配置的 Actor ID: {actor_id}。")


if __name__ == '__main__':
    try:
        estimator = CoordinateEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("節點已關閉。")
    except Exception as e:
        rospy.logfatal(f"節點因致命錯誤而崩潰: {e}")
