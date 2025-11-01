#include "pose_init/pose_init.h"
#include <map>
#include <memory> // 用于 std::unique_ptr
#include <vector>

/**
 * 初始化一个无人机的位姿转换器。
 */
PoseTransformer::PoseTransformer(ros::NodeHandle& nh, const std::string& namespace_str, const DroneOffset& offset)
    : nh_(nh), namespace_(namespace_str), offset_(offset) {
    
    // 输入话题：无人机相对于其起飞点的局部位置
    std::string input_pose_topic = "/" + namespace_ + "/mavros/local_position/pose";
    std::string input_odom_topic = "/" + namespace_ + "/mavros/odometry/in";

    // 输出话题：无人机在全局地图坐标系中的位置
    std::string output_pose_topic = "/" + namespace_ + "/mavros/vision_pose/pose";
    std::string output_odom_topic = "/" + namespace_ + "/mavros/vision_odom/odom";

    // 创建发布者
    vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_pose_topic, 10);
    vision_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic, 10);

    // 创建订阅者
    local_pose_sub_ = nh_.subscribe(input_pose_topic, 10, &PoseTransformer::localPoseCallback, this);
    local_odom_sub_ = nh_.subscribe(input_odom_topic, 10, &PoseTransformer::localOdomCallback, this);

    ROS_INFO("为 %s 初始化坐标转换器。", namespace_.c_str());
    ROS_INFO("  -> 订阅: %s 和 %s", input_pose_topic.c_str(), input_odom_topic.c_str());
    ROS_INFO("  -> 发布: %s 和 %s", output_pose_topic.c_str(), output_odom_topic.c_str());
}

PoseTransformer::~PoseTransformer() {}

/**
 * 接收到局部PoseStamped消息后的回调函数。
 */
void PoseTransformer::localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& local_pose_msg) {
    // 创建一个新的PoseStamped消息用于发布
    geometry_msgs::PoseStamped global_pose_msg;

    // 1. 复制header和orientation信息，保持不变
    global_pose_msg.header = local_pose_msg->header;
    global_pose_msg.pose.orientation = local_pose_msg->pose.orientation;

    // 2. 加上偏移量，计算全局坐标
    global_pose_msg.pose.position.x = local_pose_msg->pose.position.x + offset_.x;
    global_pose_msg.pose.position.y = local_pose_msg->pose.position.y + offset_.y;
    global_pose_msg.pose.position.z = local_pose_msg->pose.position.z + offset_.z;

    // 3. 发布转换后的消息
    vision_pose_pub_.publish(global_pose_msg);
}

/**
 * 接收到局部Odometry消息后的回调函数。
 */
void PoseTransformer::localOdomCallback(const nav_msgs::Odometry::ConstPtr& local_odom_msg) {
    // 创建一个新的Odometry消息用于发布
    nav_msgs::Odometry global_odom_msg;

    // 1. 复制header和child_frame_id信息
    global_odom_msg.header = local_odom_msg->header;
    global_odom_msg.child_frame_id = local_odom_msg->child_frame_id;

    // 2. 复制姿态、协方差和速度信息（保持不变）
    global_odom_msg.pose.pose.orientation = local_odom_msg->pose.pose.orientation;
    global_odom_msg.pose.covariance = local_odom_msg->pose.covariance;
    global_odom_msg.twist = local_odom_msg->twist;

    // 3. 加上偏移量，计算全局坐标
    global_odom_msg.pose.pose.position.x = local_odom_msg->pose.pose.position.x + offset_.x;
    global_odom_msg.pose.pose.position.y = local_odom_msg->pose.pose.position.y + offset_.y;
    global_odom_msg.pose.pose.position.z = local_odom_msg->pose.pose.position.z + offset_.z;
    
    // 4. 发布转换后的消息
    vision_odom_pub_.publish(global_odom_msg);
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pose_transformer_node");
    ros::NodeHandle nh;

    // 使用 std::map 存储无人机出发点坐标（偏移量）
    std::map<std::string, DroneOffset> drone_offsets = {
        {"typhoon_h480_zzufly_0", {-17, -3, 0}},
        {"typhoon_h480_zzufly_1", {-14, -3, 0}},
        {"typhoon_h480_zzufly_2", {-17,  0, 0}},
        {"typhoon_h480_zzufly_3", {-14,  0, 0}},
        {"typhoon_h480_zzufly_4", {-17,  3, 0}},
        {"typhoon_h480_zzufly_5", {-14,  3, 0}}
    };

    // 为字典中的每一架无人机创建一个转换器实例
    std::vector<std::unique_ptr<PoseTransformer>> transformers;
    for (const auto& pair : drone_offsets) {
        transformers.push_back(std::make_unique<PoseTransformer>(nh, pair.first, pair.second));
    }

    ROS_INFO("所有无人机坐标转换器已启动。节点将持续运行...");

    // 保持节点运行，直到被关闭
    ros::spin();

    return 0;
}
