#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <ros/ros.h>
#include <string>
#include <map>
#include <memory> // For std::unique_ptr

// Message and Service Types
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "tracking/smoothing.h"

// Project-specific headers
#include "tracking/controller.h"
#include "tracking/broadcast_progress.h"
#include "tracking/service_manager.h"
#include "tracking/kalman_filter.h"

// Define a type alias for clarity
using TargetMap = std::map<std::string, darknet_ros_msgs::BoundingBox>;

/**
 * @enum class State
 * @brief Defines the possible states of the tracking state machine.
 */
enum class State {
    IDLE,
    DETECTING,
    DASH,
    TRACKING,
    LOST,
    RETURNING
};

/**
 * @class TrackingStateMachine
 * @brief Manages the overall behavior of the drone tracking system.
 * It transitions between states like IDLE, DETECTING, TRACKING, etc.,
 * based on target visibility and internal timers.
 */
class TrackingStateMachine {
public:
    /**
     * @brief Constructor for the TrackingStateMachine.
     * @param nh ROS NodeHandle for parameter loading and publisher creation.
     * @param vehicle_type The type of the vehicle.
     * @param vehicle_id The ID of the vehicle.
     * @param controller A reference to the tracking controller object.
     * @param services A reference to the service manager object.
     */
    TrackingStateMachine(ros::NodeHandle& nh,
                         const std::string& vehicle_type,
                         const std::string& vehicle_id,
                         TrackingController& controller,
                         ServiceManager& services);

    /**
     * @brief The main update loop for the state machine. Called at a fixed rate.
     * @param current_visible_targets A map of currently visible targets.
     * @param height Current altitude of the drone.
     * @param current_pose The drone's current pose.
     * @param current_velocity_body The drone's current velocity in its body frame.
     */
    void update(const TargetMap& current_visible_targets,
                double height,
                const geometry_msgs::Pose& current_pose,
                const geometry_msgs::Twist& current_velocity_body);

    void setTakeoffComplete(bool complete);
    void setMissionActive(bool active);
    void recordCoordinateBroadcast(const std::string& vehicle_name,
                                   const std::string& target_id,
                                   const ros::Time& stamp);

private:
    enum class ReturnOutcome {
        RELEASE,
        COMPLETE,
        RELEASE_WITH_COOLDOWN
    };

    // --- State Handler Methods ---
    void handleIdleState(const TargetMap& current_visible_targets);
    void handleDetectingState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double height, const geometry_msgs::Twist& current_velocity_body);
    void handleDashState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double height, double dt);
    void handleTrackingState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double height, const geometry_msgs::Twist& current_velocity_body, double dt);
    void handleLostState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double dt);
    void handleReturningState(double dt);

    // --- State Transition & Action Methods ---
    void enterDashState(double height);
    void enterTrackingState(const darknet_ros_msgs::BoundingBox& target, double height, const geometry_msgs::Twist& current_velocity_body);
    void enterTrackingFromDash(const darknet_ros_msgs::BoundingBox& target);
    void enterLostState();
    void reacquireTarget(const darknet_ros_msgs::BoundingBox& target);
    void beginReturnToMission(ReturnOutcome outcome);
    void finalizeReturnToMission();

    // --- Helper Methods ---
    void checkHeightLowering();
    void initializeKalmanFilter(const darknet_ros_msgs::BoundingBox& target);
    void publishCommands(const ControlCommand& commands, double dt);
    bool requestTarget(const std::string& target_id);
    void releaseTarget(const std::string& target_id);
    void pauseMission();
    void startBroadcastSession();
    void updateControlGateState(const char* gate_name);
    void resetForClosedControlGate();
    void publishTrackingPhase(const std::string& phase,
                              const ros::Time& stamp);
    std::string stateToString(State state);

    // --- ROS and Core Components ---
    ros::NodeHandle& nh_;
    ros::Publisher tracking_status_pub_;
    ros::Publisher tracking_phase_pub_;
    std::string vehicle_type_;
    std::string vehicle_id_;
    TrackingController& controller_;
    ServiceManager& services_;
    std::unique_ptr<IMMFilter> kf_;
    std::unique_ptr<OutputSmoother> smoother_;
    std::unique_ptr<BroadcastProgress> broadcast_progress_;

    // --- ROS Publishers ---
    ros::Publisher cmd_vel_pub_;
    ros::Publisher cmd_pub_;
    ros::Publisher mission_control_pub_;

    // --- State Variables ---
    State current_state_ = State::IDLE;
    std::string currently_tracked_target_id_;
    ros::Time idle_entry_time_;
    ros::Time first_seen_time_;
    ros::Time last_seen_time_; // Used for timeout in LOST state
    ros::Time tracking_start_time_;
    ros::Time dash_start_time_;
    ros::Time last_update_time_; // 3. 添加用于计算 dt 的时间戳
    double dash_initial_height_ = 0.0;
    bool height_lowered_ = false;
    bool takeoff_complete_ = false;
    bool mission_active_ = false;
    int lost_frame_counter_ = 0;
    std::map<std::string, ros::Time> cooldown_until_;
    ReturnOutcome return_outcome_ = ReturnOutcome::RELEASE;
    bool navigator_selected_ = false;
    int complete_attempts_ = 0;
    bool heartbeat_received_ = false;
    bool broadcast_confirmation_logged_ = false;
    
    // --- Configuration Parameters (loaded from server) ---
    ros::Duration confirmation_duration_;
    ros::Duration lost_timeout_;
    double DASH_TRIGGER_AREA_;
    double MIN_DASH_HEIGHT_;
    double AREA_EXIT_THRESHOLD_;
    double DASH_DURATION_;
    double HEIGHT_LOWER_DELAY_;
    double LOWERED_TRACKING_HEIGHT_;
    int LOST_BUFFER_FRAMES_;
    double retry_cooldown_ = 5.0;
    double target_height_ = 0; // Runtime-set target height
};

#endif // STATE_MACHINE_H
