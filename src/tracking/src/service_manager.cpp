#include "tracking/service_manager.h"

ServiceManager::ServiceManager(ros::NodeHandle& nh, const std::string& vehicle_type, const std::string& vehicle_id)
    : nh_(nh), vehicle_type_(vehicle_type), vehicle_id_(vehicle_id)
{
    initMuxService();
    initTargetServices();
}

void ServiceManager::initMuxService() {
    std::string service_name = "/" + vehicle_type_ + "_" + vehicle_id_ + "/pose_cmd_mux/select";
    ROS_INFO("[%s_%s Tracker] Waiting for MUX service '%s'...",
             vehicle_type_.c_str(), vehicle_id_.c_str(), service_name.c_str());

    ros::service::waitForService(service_name); // Block until the service is available

    mux_select_client_ = nh_.serviceClient<topic_tools::MuxSelect>(service_name);

    ROS_INFO("[%s_%s Tracker] MUX service connected.",
             vehicle_type_.c_str(), vehicle_id_.c_str());
}

void ServiceManager::initTargetServices() {
    ROS_INFO("[%s_%s Tracker] Connecting to target management services...",
             vehicle_type_.c_str(), vehicle_id_.c_str());

    for (const auto& tid : TARGET_IDS_) {
        std::string req_name = "/lookup/request_" + tid;
        std::string rel_name = "/lookup/release_" + tid;

        // Wait for both request and release services for the current target ID
        ros::service::waitForService(req_name);
        ros::service::waitForService(rel_name);

        // Create clients and store them in the maps
        request_clients_[tid] = nh_.serviceClient<look_up::RequestTarget>(req_name);
        release_clients_[tid] = nh_.serviceClient<look_up::ReleaseTarget>(rel_name);
    }

    const std::string complete_name = "/lookup/complete_target";
    ros::service::waitForService(complete_name);
    complete_client_ = nh_.serviceClient<look_up::CompleteTarget>(complete_name);

    ROS_INFO("[%s_%s Tracker] All target management services connected.",
             vehicle_type_.c_str(), vehicle_id_.c_str());
}

bool ServiceManager::switchControl(const std::string& topic_name) {
    topic_tools::MuxSelect srv;
    srv.request.topic = topic_name;

    if (mux_select_client_.call(srv)) {
        ROS_INFO("[Tracker] Successfully requested MUX switch to: %s", topic_name.c_str());
        return true;
    } else {
        ROS_ERROR("[Tracker] Failed to call MUX service.");
        return false;
    }
}

bool ServiceManager::requestTarget(const std::string& target_id) {
    // Check if a client for this target_id exists
    if (request_clients_.find(target_id) == request_clients_.end()) {
        ROS_ERROR("[%s_%s] Attempted to request unknown target ID: %s",
                  vehicle_type_.c_str(), vehicle_id_.c_str(), target_id.c_str());
        return false;
    }

    look_up::RequestTarget srv;
    srv.request.target_id = target_id;

    ros::ServiceClient client = request_clients_[target_id];

    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("[%s_%s] Successfully locked target '%s'!",
                     vehicle_type_.c_str(), vehicle_id_.c_str(), target_id.c_str());
            return true;
        } else {
            ROS_INFO("[%s_%s] Target '%s' is already locked by another drone.",
                     vehicle_type_.c_str(), vehicle_id_.c_str(), target_id.c_str());
            return false;
        }
    } else {
        ROS_ERROR("[%s_%s] Failed to call request service for target '%s'.",
                  vehicle_type_.c_str(), vehicle_id_.c_str(), target_id.c_str());
        return false;
    }
}

bool ServiceManager::releaseTarget(const std::string& target_id) {
    if (release_clients_.find(target_id) == release_clients_.end()) {
        ROS_ERROR("[%s_%s] Attempted to release unknown target ID: %s",
                  vehicle_type_.c_str(), vehicle_id_.c_str(), target_id.c_str());
        return false;
    }

    look_up::ReleaseTarget srv;
    srv.request.target_id = target_id;

    ros::ServiceClient client = release_clients_[target_id];

    if (client.call(srv)) {
        ROS_INFO("[%s_%s Tracker] Released target '%s'",
                 vehicle_type_.c_str(), vehicle_id_.c_str(), target_id.c_str());
        return true;
    } else {
        ROS_ERROR("Failed to call release service for target '%s'.", target_id.c_str());
        return false;
    }
}

bool ServiceManager::completeTarget(const std::string& target_id) {
    look_up::CompleteTarget srv;
    srv.request.target_id = target_id;

    if (!complete_client_.call(srv)) {
        ROS_ERROR("[%s_%s Tracker] Failed to call completion service for target '%s'.",
                  vehicle_type_.c_str(), vehicle_id_.c_str(), target_id.c_str());
        return false;
    }
    if (!srv.response.success) {
        ROS_ERROR("[%s_%s Tracker] Completion service rejected target '%s'.",
                  vehicle_type_.c_str(), vehicle_id_.c_str(), target_id.c_str());
        return false;
    }

    ROS_INFO("[%s_%s Tracker] Completed target '%s'.",
             vehicle_type_.c_str(), vehicle_id_.c_str(), target_id.c_str());
    return true;
}
