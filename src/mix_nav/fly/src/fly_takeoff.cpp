#include "fly/fly_takeoff.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace fly {

ConfidentTakeoff::ConfidentTakeoff(const std::string& drone_name, int drone_quantity, double target_altitude)
    : drone_name_(drone_name),
      drone_quantity_(drone_quantity),
      target_altitude_(target_altitude),
      rate_(RATE) {
    ros::NodeHandle private_nh("~");
    private_nh.param("startup_delay", startup_delay_, 2.0);
    private_nh.param("takeoff_timeout", takeoff_timeout_, 15.0);

    takeoff_complete_pub_ =
        nh_.advertise<std_msgs::Bool>("/swarm/takeoff_complete", 1, true);
    publishTakeoffComplete(false);
    
    // 初始化状态向量
    current_poses_.resize(drone_quantity_);
    mission_done_flags_.resize(drone_quantity_, false);
    pose_received_flags_.resize(drone_quantity_, false);
    
    // 初始化发布器和订阅器
    for (int i = 0; i < drone_quantity_; ++i) {
        const std::string vehicle = drone_name_ + "_" + std::to_string(i);
        // 创建命令发布器
        std::string cmd_topic = "/xtdrone/" + vehicle + "/cmd";
        cmd_pubs_.push_back(nh_.advertise<std_msgs::String>(cmd_topic, 1));
        
        // 创建速度发布器
        std::string vel_topic = "/" + vehicle + "/mux_inputs/takeoff/cmd_vel";
        vel_pubs_.push_back(nh_.advertise<geometry_msgs::Twist>(vel_topic, 1));

        const std::string service = "/" + vehicle + "/pose_cmd_mux/select";
        mux_select_clients_.push_back(
            nh_.serviceClient<topic_tools::MuxSelect>(service));
        
        // 创建位姿订阅器 - 使用lambda表达式绑定无人机ID
        std::string pose_topic = "/" + vehicle + "/mavros/local_position/pose";
        auto callback = [this, i](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            this->poseCallback(msg, i);
        };
        pose_subs_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(pose_topic, 1, callback));
    }
    
    ROS_INFO("ConfidentTakeoff initialized: drone_name=%s, quantity=%d, target_altitude=%.2f", 
             drone_name_.c_str(), drone_quantity_, target_altitude_);
}

void ConfidentTakeoff::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int drone_id) {
    // 第一次收到位姿信息时给出提示
    if (!pose_received_flags_[drone_id]) {
        ROS_INFO("--- Successfully received position information for drone %d! ---", drone_id);
        pose_received_flags_[drone_id] = true;
    }
    
    current_poses_[drone_id] = msg;
}

bool ConfidentTakeoff::allMissionDone() const {
    for (bool done : mission_done_flags_) {
        if (!done) return false;
    }
    return true;
}

bool ConfidentTakeoff::selectControl(int drone_id,
                                     const std::string& topic_name) {
    topic_tools::MuxSelect request;
    request.request.topic = topic_name;
    auto& client = mux_select_clients_.at(drone_id);
    if (!client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("MUX service unavailable for drone %d", drone_id);
        return false;
    }
    if (!client.call(request)) {
        ROS_ERROR("Failed to select MUX input for drone %d", drone_id);
        return false;
    }
    return true;
}

void ConfidentTakeoff::publishZeroVelocity() {
    const geometry_msgs::Twist zero;
    for (auto& publisher : vel_pubs_) {
        publisher.publish(zero);
    }
}

void ConfidentTakeoff::publishTakeoffComplete(bool complete) {
    std_msgs::Bool message;
    message.data = complete;
    takeoff_complete_pub_.publish(message);
}

void ConfidentTakeoff::run() {
    ROS_INFO("Confident takeoff script started. Process will begin in %.2f seconds.",
             startup_delay_);
    ros::Duration(startup_delay_).sleep();
    
    ROS_INFO("[Step 1] Sending velocity commands and requesting OFFBOARD and ARM...");
    
    // 创建爬升速度指令
    geometry_msgs::Twist climb_twist;
    climb_twist.linear.z = CLIMB_VELOCITY;

    for (int i = 0; i < drone_quantity_; ++i) {
        const std::string vehicle = drone_name_ + "_" + std::to_string(i);
        const std::string takeoff_topic =
            "/" + vehicle + "/mux_inputs/takeoff/cmd_vel";
        if (!selectControl(i, takeoff_topic)) {
            publishZeroVelocity();
            ROS_ERROR("Takeoff aborted because MUX ownership was not confirmed.");
            return;
        }
    }
    
    // 发送OFFBOARD和ARM指令
    std_msgs::String cmd_msg;
    cmd_msg.data = "OFFBOARD";
    for (auto& pub : cmd_pubs_) {
        pub.publish(cmd_msg);
    }
    
    ros::Duration(0.1).sleep();
    
    cmd_msg.data = "ARM";
    for (auto& pub : cmd_pubs_) {
        pub.publish(cmd_msg);
    }
    
    ROS_INFO("[Step 2] Entering climb and real-time altitude monitoring phase.");
    
    auto mission_start_time = ros::Time::now();
    ros::Time last_log_time = ros::Time::now();
    const double log_interval = 1.0; // 每秒打印一次
    
    while (ros::ok() && !allMissionDone()) {
        // 检查超时
        if ((ros::Time::now() - mission_start_time).toSec() >
            takeoff_timeout_) {
            ROS_ERROR("Mission timeout!");
            break;
        }
        
        // Stop each aircraft independently instead of making early arrivals
        // keep climbing while the slowest aircraft catches up.
        for (int i = 0; i < drone_quantity_; ++i) {
            if (!mission_done_flags_[i]) {
                vel_pubs_[i].publish(climb_twist);
            }
        }
        
        // 检查高度并更新状态
        for (int i = 0; i < drone_quantity_; ++i) {
            if (!mission_done_flags_[i] && current_poses_[i]) {
                double current_z = current_poses_[i]->pose.position.z;
                
                // 限制日志输出频率
                if ((ros::Time::now() - last_log_time).toSec() >= log_interval) {
                    ROS_INFO_THROTTLE(1, "Drone %d climbing... Current altitude: %.2f m", i, current_z);
                }
                
                if ((target_altitude_ - altitude_tolerance) < current_z) {
                    mission_done_flags_[i] = true;
                    ROS_INFO("====== Drone %d has reached target altitude! ======", i);
                }
            }
        }
        
        // 更新最后日志时间
        if ((ros::Time::now() - last_log_time).toSec() >= log_interval) {
            last_log_time = ros::Time::now();
        }
        
        ros::spinOnce();
        rate_.sleep();
    }
    
    ROS_INFO("Sending 'HOVER' command to stabilize and hand over control...");

    publishZeroVelocity();
    ros::Duration(0.25).sleep();
    
    // 发送HOVER指令
    cmd_msg.data = "HOVER";
    for (auto& publisher : cmd_pubs_) {
        for (int repeat = 0; repeat < 5; ++repeat) {
            publisher.publish(cmd_msg);
            rate_.sleep();
        }
    }

    if (!allMissionDone()) {
        ROS_ERROR("Takeoff incomplete; keeping zero takeoff input selected.");
        return;
    }

    ROS_INFO("All drones in the cluster have reached the target altitude!");
    for (int i = 0; i < drone_quantity_; ++i) {
        const std::string vehicle = drone_name_ + "_" + std::to_string(i);
        const std::string navigator_topic =
            "/" + vehicle + "/mux_inputs/navigator/cmd_vel";
        if (!selectControl(i, navigator_topic)) {
            ROS_ERROR("Navigator handoff failed for drone %d; rolling back all drones.", i);
            for (int rollback_id = 0; rollback_id < drone_quantity_;
                 ++rollback_id) {
                const std::string rollback_vehicle =
                    drone_name_ + "_" + std::to_string(rollback_id);
                const std::string takeoff_topic =
                    "/" + rollback_vehicle + "/mux_inputs/takeoff/cmd_vel";
                if (!selectControl(rollback_id, takeoff_topic)) {
                    ROS_ERROR("Failed to restore takeoff input for drone %d; safety watchdog remains active.",
                              rollback_id);
                }
            }
            publishZeroVelocity();
            return;
        }
    }

    publishTakeoffComplete(true);
    ROS_INFO("Cluster mission completed! Control has been handed over.");
    ROS_INFO("Takeoff gate is open; keeping latched status publisher alive.");
    ros::spin();
}

} // namespace fly
