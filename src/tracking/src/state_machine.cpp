#include "tracking/state_machine.h"
#include <Eigen/Dense>

TrackingStateMachine::TrackingStateMachine(ros::NodeHandle& nh,
                                             const std::string& vehicle_type,
                                             const std::string& vehicle_id,
                                             TrackingController& controller,
                                             ServiceManager& services)
    : nh_(nh),
      vehicle_type_(vehicle_type),
      vehicle_id_(vehicle_id),
      controller_(controller),
      services_(services)
{
    // Load parameters from the ROS Parameter Server
    double confirmation_duration_sec, lost_timeout_sec, dash_duration_sec, height_lower_delay_sec;
    nh_.param("state_machine/confirmation_duration", confirmation_duration_sec, 0.25);
    nh_.param("state_machine/lost_timeout", lost_timeout_sec, 3.0);
    nh_.param("state_machine/dash_duration", dash_duration_sec, 1.5);
    nh_.param("state_machine/height_lower_delay", height_lower_delay_sec, 0.5);

    confirmation_duration_ = ros::Duration(confirmation_duration_sec);
    lost_timeout_ = ros::Duration(lost_timeout_sec);
    DASH_DURATION_ = dash_duration_sec;
    HEIGHT_LOWER_DELAY_ = height_lower_delay_sec;

    nh_.param("state_machine/dash_trigger_area", DASH_TRIGGER_AREA_, 250.0);
    nh_.param("state_machine/min_dash_height", MIN_DASH_HEIGHT_, 3.0);
    nh_.param("state_machine/area_exit_threshold", AREA_EXIT_THRESHOLD_, 500.0);
    nh_.param("state_machine/lowered_tracking_height", LOWERED_TRACKING_HEIGHT_, 2.2);
    nh_.param("state_machine/lost_buffer_frames", LOST_BUFFER_FRAMES_, 5);

    // Initialize Publishers
    std::string xtdrone_namespace = "/xtdrone/" + vehicle_type_ + "_" + vehicle_id_;
    std::string external_command_topic = "/" + vehicle_type_ + "_" + vehicle_id_ +
                                         "/mux_inputs/external/pose_cmd";
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(external_command_topic, 1);
    cmd_pub_ = nh_.advertise<std_msgs::String>(xtdrone_namespace + "/cmd", 1);
    mission_control_pub_ = nh_.advertise<std_msgs::String>("/" + vehicle_type_ + "_" + vehicle_id_ + "/mission/control", 1);
    
    // 新增：追踪状态发布器，话题名与节点名相同
    tracking_status_pub_ = nh_.advertise<std_msgs::String>("yolo_human_tracking_" + vehicle_type_ + "_" + vehicle_id_, 1);
    
    smoother_ = std::make_unique<OutputSmoother>(nh_);
    last_update_time_ = ros::Time::now(); 
}

void TrackingStateMachine::update(const TargetMap& current_visible_targets,
                                    double height,
                                    const geometry_msgs::Pose& current_pose,
                                    const geometry_msgs::Twist& current_velocity_body)
{
    if (!takeoff_complete_) {
        return;
    }

    ros::Time now = ros::Time::now();
    double dt = (now - last_update_time_).toSec();
    last_update_time_ = now;
    ROS_INFO_THROTTLE(1.0, "Current dt: %f", dt);
    // 防止第一次运行或系统暂停时 dt 出现异常值
    if (dt <= 0.0 || dt > 0.5) {
        dt = 1.0 / 30.0; // 使用一个安全的默认值 (对应30Hz)
    }

    const darknet_ros_msgs::BoundingBox* locked_target_bbox = nullptr;
    if (!currently_tracked_target_id_.empty()) {
        auto it = current_visible_targets.find(currently_tracked_target_id_);
        if (it != current_visible_targets.end()) {
            locked_target_bbox = &it->second;
        }
    }

    switch (current_state_) {
        case State::IDLE:
            handleIdleState(current_visible_targets);
            break;
        case State::DETECTING:
            handleDetectingState(locked_target_bbox, height, current_velocity_body);
            break;
        case State::DASH:
            handleDashState(locked_target_bbox, height, dt);
            break;
        case State::TRACKING:
            handleTrackingState(locked_target_bbox, height, current_velocity_body, dt);
            break;
        case State::LOST:
            handleLostState(locked_target_bbox, dt);
            break;
    }
}

void TrackingStateMachine::setTakeoffComplete(bool complete) {
    if (takeoff_complete_ == complete) {
        return;
    }

    takeoff_complete_ = complete;
    if (takeoff_complete_) {
        last_update_time_ = ros::Time::now();
        ROS_INFO("[%s_%s Tracker] Takeoff gate opened.",
                 vehicle_type_.c_str(), vehicle_id_.c_str());
        return;
    }

    ROS_WARN("[%s_%s Tracker] Takeoff gate closed; resetting tracking state.",
             vehicle_type_.c_str(), vehicle_id_.c_str());
    resetForClosedTakeoffGate();
}

void TrackingStateMachine::handleIdleState(const TargetMap& current_visible_targets) {
    // Delayed release of previously tracked target
    if (!idle_entry_time_.is_zero() && !currently_tracked_target_id_.empty() &&
        (ros::Time::now() - idle_entry_time_).toSec() > 1.0) {
        releaseTarget(currently_tracked_target_id_);
        currently_tracked_target_id_.clear();
        idle_entry_time_ = ros::Time(0); // Reset timer
    }

    // Search for a new target
    if (currently_tracked_target_id_.empty() && !current_visible_targets.empty()) {
        // 先检查是否有red4
        auto red4_it = current_visible_targets.find("red4");
        if (red4_it != current_visible_targets.end()) {
            if (requestTarget("red4")) {
                // 成功获取red4
                currently_tracked_target_id_ = "red4";
                first_seen_time_ = ros::Time::now();
                current_state_ = State::DETECTING;
                idle_entry_time_ = ros::Time(0);
                ROS_INFO("[%s_%s Tracker] State: IDLE -> DETECTING (red4)", vehicle_type_.c_str(), vehicle_id_.c_str());
                return; // 直接返回，不继续寻找其他目标
            }
            // red4已被占用，继续尝试red5
        }
        
        // 检查是否有red5
        auto red5_it = current_visible_targets.find("red5");
        if (red5_it != current_visible_targets.end()) {
            if (requestTarget("red5")) {
                // 成功获取red5
                currently_tracked_target_id_ = "red5";
                first_seen_time_ = ros::Time::now();
                current_state_ = State::DETECTING;
                idle_entry_time_ = ros::Time(0);
                ROS_INFO("[%s_%s Tracker] State: IDLE -> DETECTING (red5)", vehicle_type_.c_str(), vehicle_id_.c_str());
                return; // 直接返回
            }
        }
        
        // 如果没有red4/red5或者都已被占用，按原逻辑寻找其他目标
        for (const auto& pair : current_visible_targets) {
            // 跳过已经尝试过的red4和red5
            if (pair.first == "red4" || pair.first == "red5") continue;
            
            if (requestTarget(pair.first)) {
                currently_tracked_target_id_ = pair.first;
                first_seen_time_ = ros::Time::now();
                current_state_ = State::DETECTING;
                idle_entry_time_ = ros::Time(0);
                ROS_INFO("[%s_%s Tracker] State: IDLE -> DETECTING", vehicle_type_.c_str(), vehicle_id_.c_str());
                break;
            }
        }
    }
}

void TrackingStateMachine::handleDetectingState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double height, const geometry_msgs::Twist& current_velocity_body) {
    if (locked_target_bbox == nullptr) {
        current_state_ = State::IDLE;
        idle_entry_time_ = ros::Time::now();
        ROS_INFO("[%s_%s Tracker] State: DETECTING -> IDLE", vehicle_type_.c_str(), vehicle_id_.c_str());
        // Target is implicitly released by the logic in handleIdleState
    } else if ((ros::Time::now() - first_seen_time_) > confirmation_duration_) {
        double current_area = (locked_target_bbox->xmax - locked_target_bbox->xmin) * (locked_target_bbox->ymax - locked_target_bbox->ymin);
        if (current_area < DASH_TRIGGER_AREA_ && height > MIN_DASH_HEIGHT_) {
            enterDashState(height);
        } else {
            enterTrackingState(*locked_target_bbox, height, current_velocity_body);
        }
    }
}

void TrackingStateMachine::handleDashState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double height, double dt) {
    double elapsed = (ros::Time::now() - dash_start_time_).toSec();
    bool exit_dash = false;

    if (elapsed >= DASH_DURATION_) {
        ROS_INFO("[%s_%s Tracker] Dash time expired (%.1fs), ending dash.", vehicle_type_.c_str(), vehicle_id_.c_str(), elapsed);
        exit_dash = true;
    }

    if (locked_target_bbox) {
        lost_frame_counter_ = 0;
        double current_area = (locked_target_bbox->xmax - locked_target_bbox->xmin) * (locked_target_bbox->ymax - locked_target_bbox->ymin);
        if (current_area >= AREA_EXIT_THRESHOLD_) {
            ROS_INFO("[%s_%s Tracker] Target area reached threshold (%d), completing dash early.", vehicle_type_.c_str(), vehicle_id_.c_str(), (int)current_area);
            exit_dash = true;
        }
    } else {
        lost_frame_counter_++;
        if (lost_frame_counter_ > LOST_BUFFER_FRAMES_) {
            ROS_WARN("[%s_%s Tracker] Target lost during dash, exiting dash mode.", vehicle_type_.c_str(), vehicle_id_.c_str());
            exit_dash = true;
        }
    }

    if (exit_dash) {
        if (locked_target_bbox) {
            enterTrackingFromDash(*locked_target_bbox);
        } else {
            enterLostState();
        }
    } else {
        double u_error = 0.0;
        if(locked_target_bbox) {
            u_error = ((locked_target_bbox->xmin + locked_target_bbox->xmax) / 2.0) - 320.0;
        }
        ControlCommand commands = controller_.calculate_dash_commands(locked_target_bbox != nullptr, u_error, height, dash_initial_height_);
        publishCommands(commands, dt);

        // 新增：发布追踪状态信息（Dash状态）
        std_msgs::String status_msg;
        status_msg.data = "DASH:" + currently_tracked_target_id_ + ":" + std::to_string(ros::Time::now().toSec());
        tracking_status_pub_.publish(status_msg);
        
        if (ros::Time::now().toSec() - floor(ros::Time::now().toSec()) < 0.033) {
             ROS_INFO_THROTTLE(0.5, "[%s_%s Dash] V:%.1f m/s | T:%.1f/%.1fs | H:%.1f/%.1fm",
                vehicle_type_.c_str(), vehicle_id_.c_str(), 5.0, elapsed, DASH_DURATION_, height, dash_initial_height_);
        }
    }
}

void TrackingStateMachine::handleTrackingState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double height, const geometry_msgs::Twist& current_velocity_body, double dt) {
    if (!kf_) {
        ROS_ERROR("[%s_%s Tracker] CRITICAL: Entered TRACKING state but KF is not initialized.", vehicle_type_.c_str(), vehicle_id_.c_str());
        current_state_ = State::IDLE;
        idle_entry_time_ = ros::Time::now();
        return;
    }

    kf_->predict();

    if (locked_target_bbox == nullptr) {
        lost_frame_counter_++;
        ROS_WARN_THROTTLE(0.5, "[%s_%s Tracker] Target temporarily lost... Buffer count: %d/%d",
                          vehicle_type_.c_str(), vehicle_id_.c_str(), lost_frame_counter_, LOST_BUFFER_FRAMES_);
        if (lost_frame_counter_ > LOST_BUFFER_FRAMES_) {
            enterLostState();
            return; // Exit early to avoid using old KF state
        }
    } else {
        lost_frame_counter_ = 0;
        Eigen::VectorXd z(3);
        z << (locked_target_bbox->xmin + locked_target_bbox->xmax) / 2.0,
             (locked_target_bbox->ymin + locked_target_bbox->ymax) / 2.0,
             (locked_target_bbox->xmax - locked_target_bbox->xmin) * (locked_target_bbox->ymax - locked_target_bbox->ymin);
        kf_->update(z);
    }

    checkHeightLowering();

    Eigen::VectorXd kf_state = kf_->getState();
    double current_target_height = height_lowered_ ? LOWERED_TRACKING_HEIGHT_ : target_height_;

    ControlCommand commands = controller_.calculate_tracking_commands(kf_state, height, current_target_height, current_velocity_body);
    publishCommands(commands, dt);
    
    // 新增：发布追踪状态信息
    std_msgs::String status_msg;
    status_msg.data = "TRACKING:" + currently_tracked_target_id_ + ":" + std::to_string(ros::Time::now().toSec());
    tracking_status_pub_.publish(status_msg);
    
    if (ros::Time::now().toSec() - floor(ros::Time::now().toSec()) < 0.033) {
        ROS_INFO_THROTTLE(0.5, "[%s_%s] KF Area:%d | Vel(X/Y/Z/Yaw):%.2f/%.2f/%.2f/%.2f | Mode: %s | T_Factor: %.2f",
            vehicle_type_.c_str(), vehicle_id_.c_str(), (int)kf_state(4),
            commands.x, commands.y, commands.z, commands.yaw,
            commands.mode.c_str(), commands.transition_factor);
    }
}

void TrackingStateMachine::handleLostState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double dt) {
    ControlCommand zero_cmds; // All fields default to 0.0
    publishCommands(zero_cmds, dt);

    if (locked_target_bbox != nullptr) {
        reacquireTarget(*locked_target_bbox);
    } else if ((ros::Time::now() - last_seen_time_) > lost_timeout_) {
        returnControlToMission(dt);
    }
}

// --- State Transition & Action Methods Implementation ---

void TrackingStateMachine::enterDashState(double height) {
    ROS_INFO("[%s_%s Tracker] Small target detected, triggering dash mode!", vehicle_type_.c_str(), vehicle_id_.c_str());
    pauseMission();
    services_.switchControl("/" + vehicle_type_ + "_" + vehicle_id_ + "/mux_inputs/external/pose_cmd");

    dash_start_time_ = ros::Time::now();
    dash_initial_height_ = height;
    lost_frame_counter_ = 0;
    current_state_ = State::DASH;
    ROS_INFO("[%s_%s Tracker] State: DETECTING -> DASH", vehicle_type_.c_str(), vehicle_id_.c_str());
}

void TrackingStateMachine::enterTrackingState(const darknet_ros_msgs::BoundingBox& target, double height, const geometry_msgs::Twist& current_velocity_body) {
    ROS_INFO("[%s_%s Tracker] Target confirmed, requesting control takeover...", vehicle_type_.c_str(), vehicle_id_.c_str());
    
    // Config values are loaded from param server in constructor
    double cruise_speed_threshold;
    nh_.param("state_machine/cruise_speed_threshold", cruise_speed_threshold, 1.5);
    controller_.set_cruise_transition(current_velocity_body, cruise_speed_threshold);

    pauseMission();
    services_.switchControl("/" + vehicle_type_ + "_" + vehicle_id_ + "/mux_inputs/external/pose_cmd");
    
    target_height_ = height;
    initializeKalmanFilter(target);
    
    tracking_start_time_ = ros::Time::now();
    height_lowered_ = false;
    lost_frame_counter_ = 0;
    current_state_ = State::TRACKING;

    ROS_INFO("[%s_%s Tracker] State: DETECTING -> TRACKING", vehicle_type_.c_str(), vehicle_id_.c_str());
    ROS_INFO("[%s_%s Tracker] Tracking timer started, will lower height after %.1f seconds.", vehicle_type_.c_str(), vehicle_id_.c_str(), HEIGHT_LOWER_DELAY_);
}

void TrackingStateMachine::enterTrackingFromDash(const darknet_ros_msgs::BoundingBox& target) {
    target_height_ = dash_initial_height_;
    initializeKalmanFilter(target);
    
    tracking_start_time_ = ros::Time::now();
    height_lowered_ = false;
    // was_cruising is handled internally in controller, no need to set here
    
    current_state_ = State::TRACKING;
    ROS_INFO("[%s_%s Tracker] State: DASH -> TRACKING", vehicle_type_.c_str(), vehicle_id_.c_str());
}

void TrackingStateMachine::enterLostState() {
    last_seen_time_ = ros::Time::now();
    current_state_ = State::LOST;
    ROS_INFO("[%s_%s Tracker] State: %s -> LOST", vehicle_type_.c_str(), vehicle_id_.c_str(), stateToString(current_state_).c_str());
}

void TrackingStateMachine::reacquireTarget(const darknet_ros_msgs::BoundingBox& target) {
    ROS_INFO("[%s_%s Tracker] State: LOST -> TRACKING (Target reacquired)", vehicle_type_.c_str(), vehicle_id_.c_str());
    initializeKalmanFilter(target);

    if (!height_lowered_) {
        tracking_start_time_ = ros::Time::now();
        ROS_INFO("[%s_%s Tracker] Resetting continuous tracking timer.", vehicle_type_.c_str(), vehicle_id_.c_str());
    }
    lost_frame_counter_ = 0;
    current_state_ = State::TRACKING;
}

void TrackingStateMachine::returnControlToMission(double dt) {
    ROS_INFO("[%s_%s Tracker] Lost timeout, returning control...", vehicle_type_.c_str(), vehicle_id_.c_str());
    services_.switchControl("/" + vehicle_type_ + "_" + vehicle_id_ + "/mux_inputs/navigator/cmd_vel");
    
    std_msgs::String hover_cmd;
    hover_cmd.data = "HOVER";
    cmd_pub_.publish(hover_cmd);
    
    ControlCommand zero_cmds;
    publishCommands(zero_cmds, dt);

    ROS_INFO("[%s_%s] Sending RESUME command to mission manager...", vehicle_type_.c_str(), vehicle_id_.c_str());
    std_msgs::String resume_cmd;
    resume_cmd.data = "RESUME";
    mission_control_pub_.publish(resume_cmd);

    tracking_start_time_ = ros::Time(0);
    height_lowered_ = false;
    idle_entry_time_ = ros::Time::now();
    current_state_ = State::IDLE;

    ROS_INFO("[%s_%s Tracker] State: LOST -> IDLE", vehicle_type_.c_str(), vehicle_id_.c_str());
}

// --- Helper Methods Implementation ---

void TrackingStateMachine::checkHeightLowering() {
    if (!height_lowered_ && !tracking_start_time_.is_zero() &&
        (ros::Time::now() - tracking_start_time_).toSec() > HEIGHT_LOWER_DELAY_) {
        ROS_INFO("[%s_%s Tracker] Tracking continuously for > %.1f seconds, lowering altitude to %.1f m.",
                 vehicle_type_.c_str(), vehicle_id_.c_str(), HEIGHT_LOWER_DELAY_, LOWERED_TRACKING_HEIGHT_);
        height_lowered_ = true;
    }
}

void TrackingStateMachine::initializeKalmanFilter(const darknet_ros_msgs::BoundingBox& target) {
    double u_init = (target.xmin + target.xmax) / 2.0;
    double v_init = (target.ymin + target.ymax) / 2.0;
    double s_init = (target.xmax - target.xmin) * (target.ymax - target.ymin);

    if (kf_) {
        kf_->reset(u_init, v_init, s_init);
    } else {
        double dt = 1.0 / 30.0; // Assuming 30Hz operation
        kf_ = std::make_unique<IMMFilter>(dt, u_init, v_init, s_init);
    }
}

void TrackingStateMachine::publishCommands(const ControlCommand& commands, double dt) {

    const ControlCommand& smoothed_commands = commands; // 直接使用原始指令
    //ControlCommand smoothed_commands = smoother_->smooth(commands, dt);
    geometry_msgs::Twist twist_cmd;
    twist_cmd.linear.x = commands.x;
    twist_cmd.linear.y = commands.y;
    twist_cmd.linear.z = commands.z;
    twist_cmd.angular.z = commands.yaw;
    // Note: Pitch is handled by a different mechanism in many flight controllers
    // and is not part of the standard cmd_vel Twist message.
    // If direct pitch control is needed, a different message type or topic is required.
    // For now, we publish the velocities as per the Python code's cmd_vel_pub.
    cmd_vel_pub_.publish(twist_cmd);
}

bool TrackingStateMachine::requestTarget(const std::string& target_id) {
    if (services_.requestTarget(target_id)) {
        currently_tracked_target_id_ = target_id;
        return true;
    }
    return false;
}

void TrackingStateMachine::releaseTarget(const std::string& target_id) {
    services_.releaseTarget(target_id);
}

void TrackingStateMachine::pauseMission() {
    ROS_INFO("[%s_%s] Sending PAUSE command to mission manager...", vehicle_type_.c_str(), vehicle_id_.c_str());
    std_msgs::String pause_cmd;
    pause_cmd.data = "PAUSE";
    mission_control_pub_.publish(pause_cmd);
    ros::Duration(0.1).sleep();
}

void TrackingStateMachine::resetForClosedTakeoffGate() {
    if (!currently_tracked_target_id_.empty()) {
        releaseTarget(currently_tracked_target_id_);
    }

    currently_tracked_target_id_.clear();
    kf_.reset();
    current_state_ = State::IDLE;
    idle_entry_time_ = ros::Time(0);
    first_seen_time_ = ros::Time(0);
    last_seen_time_ = ros::Time(0);
    tracking_start_time_ = ros::Time(0);
    dash_start_time_ = ros::Time(0);
    last_update_time_ = ros::Time::now();
    dash_initial_height_ = 0.0;
    target_height_ = 0.0;
    height_lowered_ = false;
    lost_frame_counter_ = 0;
}

std::string TrackingStateMachine::stateToString(State state) {
    switch(state) {
        case State::IDLE: return "IDLE";
        case State::DETECTING: return "DETECTING";
        case State::DASH: return "DASH";
        case State::TRACKING: return "TRACKING";
        case State::LOST: return "LOST";
        default: return "UNKNOWN";
    }
}
