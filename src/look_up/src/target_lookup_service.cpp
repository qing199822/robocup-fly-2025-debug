#include "look_up/target_lookup_service.h"
#include <ros/ros.h>
#include <algorithm>
#include <boost/bind.hpp>

using namespace boost::placeholders;  // 用于 _1, _2 等占位符

TargetLookupService::TargetLookupService()
{
    // 初始化目标状态
    for (const auto& target_id : TARGET_IDS)
    {
        target_status_[target_id] = STATE_AVAILABLE;
    }

    // 为每个目标ID创建服务
    for (const auto& target_id : TARGET_IDS)
    {
        std::string req_service_name = "/lookup/request_" + target_id;
        std::string rel_service_name = "/lookup/release_" + target_id;

        // 创建请求服务 - 使用boost::bind
        ros::ServiceServer req_server = nh_.advertiseService<look_up::RequestTarget::Request, look_up::RequestTarget::Response>(
            req_service_name,
            boost::bind(&TargetLookupService::handleRequestTarget, this, _1, _2)
        );
        request_services_.push_back(req_server);

        // 创建释放服务
        ros::ServiceServer rel_server = nh_.advertiseService<look_up::ReleaseTarget::Request, look_up::ReleaseTarget::Response>(
            rel_service_name,
            boost::bind(&TargetLookupService::handleReleaseTarget, this, _1, _2)
        );
        release_services_.push_back(rel_server);

        ROS_INFO_STREAM("服务已就绪: '" << req_service_name << "'");
        ROS_INFO_STREAM("服务已就绪: '" << rel_service_name << "'");
    }

    ROS_INFO("=========================================");
    ROS_INFO("目标锁定服务中心已成功初始化。");
    
    std::string target_list;
    for (size_t i = 0; i < TARGET_IDS.size(); ++i)
    {
        target_list += TARGET_IDS[i];
        if (i != TARGET_IDS.size() - 1) target_list += ", ";
    }
    ROS_INFO_STREAM("当前管理的目标列表: " << target_list);
    ROS_INFO("=========================================");
}

bool TargetLookupService::handleRequestTarget(look_up::RequestTarget::Request& req,
                                            look_up::RequestTarget::Response& res)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::string target_id = req.target_id;
    
    ROS_DEBUG_STREAM("处理请求目标: " << target_id);

    // 检查目标ID是否有效
    bool target_exists = false;
    for (const auto& valid_id : TARGET_IDS) {
        if (target_id == valid_id) {
            target_exists = true;
            break;
        }
    }

    if (!target_exists) {
        ROS_ERROR_STREAM("收到未知目标的请求: '" << target_id << "'。");
        res.success = false;
        return true;
    }

    if (target_status_[target_id] == STATE_AVAILABLE)
    {
        target_status_[target_id] = STATE_TRACKED;
        ROS_INFO_STREAM("请求已批准: 目标 '" << target_id << "' 已被锁定。");
        
        // 记录剩余可用目标
        std::vector<std::string> available_targets;
        for (const auto& pair : target_status_)
        {
            if (pair.second == STATE_AVAILABLE)
            {
                available_targets.push_back(pair.first);
            }
        }
        
        if (!available_targets.empty())
        {
            std::string available_list;
            for (size_t i = 0; i < available_targets.size(); ++i)
            {
                available_list += available_targets[i];
                if (i != available_targets.size() - 1) available_list += ", ";
            }
            ROS_INFO_STREAM("--- 当前剩余可用目标: " << available_list);
        }
        else
        {
            ROS_INFO("--- 所有目标均已被锁定。");
        }
        
        res.success = true;
    }
    else
    {
        ROS_WARN_STREAM("请求被拒绝: 目标 '" << target_id << "' 已被其他无人机追踪。");
        res.success = false;
    }
    
    return true;
}

bool TargetLookupService::handleReleaseTarget(look_up::ReleaseTarget::Request& req,
                                            look_up::ReleaseTarget::Response& res)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::string target_id = req.target_id;
    
    ROS_DEBUG_STREAM("处理释放目标: " << target_id);

    // 检查目标ID是否有效
    bool target_exists = false;
    for (const auto& valid_id : TARGET_IDS) {
        if (target_id == valid_id) {
            target_exists = true;
            break;
        }
    }

    if (!target_exists) {
        ROS_ERROR_STREAM("收到未知目标的释放请求: '" << target_id << "'。");
        res.success = false;
        return true;
    }

    if (target_status_[target_id] == STATE_TRACKED)
    {
        target_status_[target_id] = STATE_AVAILABLE;
        ROS_INFO_STREAM("目标 '" << target_id << "' 已被释放，现在可用。");

        // 记录当前可用目标
        std::vector<std::string> available_targets;
        for (const auto& pair : target_status_)
        {
            if (pair.second == STATE_AVAILABLE)
            {
                available_targets.push_back(pair.first);
            }
        }
        
        if (!available_targets.empty())
        {
            std::string available_list;
            for (size_t i = 0; i < available_targets.size(); ++i)
            {
                available_list += available_targets[i];
                if (i != available_targets.size() - 1) available_list += ", ";
            }
            ROS_INFO_STREAM("--- 当前可用目标: " << available_list);
        }
    }
    else
    {
        ROS_DEBUG_STREAM("目标 '" << target_id << "' 原本就是可用状态，无需释放。");
    }
    
    // 无论目标之前的状态是什么，都返回成功
    res.success = true;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_lookup_service");
    
    try
    {
        TargetLookupService service;
        ROS_INFO("目标锁定服务节点开始运行...");
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("目标锁定服务节点异常: " << e.what());
        return 1;
    }
    
    ROS_INFO("目标锁定服务节点已关闭。");
    return 0;
}