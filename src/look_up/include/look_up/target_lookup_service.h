#ifndef TARGET_LOOKUP_SERVICE_H
#define TARGET_LOOKUP_SERVICE_H

#include <ros/ros.h>
#include <look_up/CompleteTarget.h>
#include <look_up/RequestTarget.h>
#include <look_up/ReleaseTarget.h>
#include <vector>
#include <string>
#include <map>
#include <mutex>

// 定义目标状态的常量
const std::string STATE_AVAILABLE = "AVAILABLE";
const std::string STATE_TRACKED = "TRACKED";
const std::string STATE_COMPLETED = "COMPLETED";

// 目标ID列表
const std::vector<std::string> TARGET_IDS = {"green0", "blue1", "brown2", "white3", "red4", "red5", "person"};

class TargetLookupService
{
private:
   ros::NodeHandle nh_;
   std::map<std::string, std::string> target_status_;
   std::mutex mutex_;
   std::vector<ros::ServiceServer> request_services_;
   std::vector<ros::ServiceServer> release_services_;
   ros::ServiceServer complete_service_;

   /**
    * 处理"请求追踪目标"的服务回调函数
    * @param req 服务请求
    * @param res 服务响应
    * @return 是否成功处理请求
    */
   // *** 关键修改 1: 删除多余的第三个参数 ***
   bool handleRequestTarget(look_up::RequestTarget::Request& req,
                          look_up::RequestTarget::Response& res);

   /**
    * 处理"释放追踪目标"的服务回调函数
    * @param req 服务请求
    * @param res 服务响应
    * @return 是否成功处理请求
    */
   // *** 关键修改 2: 删除多余的第三个参数 ***
   bool handleReleaseTarget(look_up::ReleaseTarget::Request& req,
                          look_up::ReleaseTarget::Response& res);

   bool handleCompleteTarget(look_up::CompleteTarget::Request& req,
                             look_up::CompleteTarget::Response& res);

   bool isKnownTarget(const std::string& target_id) const;

public:
   /**
    * 构造函数 - 初始化目标锁定服务
    */
   TargetLookupService();

   /**
    * 析构函数
    */
   ~TargetLookupService() = default;

   // 删除拷贝构造函数和赋值操作符，确保单例行为
   TargetLookupService(const TargetLookupService&) = delete;
   TargetLookupService& operator=(const TargetLookupService&) = delete;
};

#endif // TARGET_LOOKUP_SERVICE_H
