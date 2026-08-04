#ifndef SERVICE_MANAGER_H
#define SERVICE_MANAGER_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

// Include the required service definition headers
#include "topic_tools/MuxSelect.h"
#include "look_up/RequestTarget.h"
#include "look_up/ReleaseTarget.h"
#include "look_up/CompleteTarget.h"

/**
 * @class ServiceManager
 * @brief Manages all ROS service client interactions for the tracking node.
 *
 * This class handles the initialization and calling of services for:
 * - MUX control switching.
 * - Requesting and releasing target locks from a central lookup node.
 */
class ServiceManager {
public:
    /**
     * @brief Constructor for ServiceManager.
     * @param nh ROS NodeHandle for creating service clients.
     * @param vehicle_type The type of the vehicle (e.g., "iris").
     * @param vehicle_id The ID of the vehicle (e.g., "0").
     */
    ServiceManager(ros::NodeHandle& nh, const std::string& vehicle_type, const std::string& vehicle_id);

    /**
     * @brief Switches the control MUX to a specified topic.
     * @param topic_name The destination topic for the MUX.
     * @return True if the service call was successful, false otherwise.
     */
    bool switchControl(const std::string& topic_name);

    /**
     * @brief Requests to lock a target via the lookup service.
     * @param target_id The ID of the target to request (e.g., "person").
     * @return True if the target was successfully locked, false otherwise.
     */
    bool requestTarget(const std::string& target_id);

    /**
     * @brief Releases a locked target via the lookup service.
     * @param target_id The ID of the target to release.
     * @return True if the service call was successful, false otherwise.
     */
    bool releaseTarget(const std::string& target_id);

    /**
     * @brief Permanently marks a locked target as completed.
     * @param target_id The ID of the target whose report is complete.
     * @return True only when the completion service accepts the request.
     */
    bool completeTarget(const std::string& target_id);

private:
    /**
     * @brief Initializes the MUX selection service client.
     */
    void initMuxService();

    /**
     * @brief Initializes all target management service clients.
     */
    void initTargetServices();

    ros::NodeHandle nh_;
    std::string vehicle_type_;
    std::string vehicle_id_;

    // Client for switching the controller MUX
    ros::ServiceClient mux_select_client_;

    // Maps to hold clients for requesting and releasing multiple targets
    std::map<std::string, ros::ServiceClient> request_clients_;
    std::map<std::string, ros::ServiceClient> release_clients_;
    ros::ServiceClient complete_client_;

    // List of supported target IDs
    const std::vector<std::string> TARGET_IDS_ = {
        "green0", "blue1", "brown2", "white3", "red4", "red5", "person"
    };
};

#endif // SERVICE_MANAGER_H
