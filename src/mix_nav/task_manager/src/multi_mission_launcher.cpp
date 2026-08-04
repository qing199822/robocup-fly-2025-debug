// src/multi_mission_launcher.cpp

#include <ros/ros.h>
#include <thread>
#include <vector>
#include <string>
#include <fstream>
#include "task_manager/mission_manager.h"
#include "task_manager/mission_definition.h"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "用法錯誤!" << std::endl;
        std::cerr << "用法: rosrun task_manager multi_mission_launcher <mission_file_path> <vehicle_id_1> [vehicle_id_2] ..." << std::endl;
        std::cerr << "示例: rosrun task_manager multi_mission_launcher ~/mission.json typhoon_h480_0 iris_0" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "multi_mission_launcher");
    ros::NodeHandle nh;

    std::string mission_file_path = argv[1];
    std::vector<std::string> target_vehicle_ids;
    for (int i = 2; i < argc; ++i) {
        target_vehicle_ids.push_back(argv[i]);
    }

    std::ifstream mission_file(mission_file_path);
    if (!mission_file.is_open()) {
        ROS_FATAL_STREAM("[Launcher] 無法打開任務文件: " << mission_file_path);
        return 1;
    }

    std::vector<task_manager::MissionDefinition> missions;
    try {
        missions = task_manager::loadMissionDefinitions(
            mission_file, target_vehicle_ids);
    } catch (const std::exception& error) {
        ROS_FATAL("[Launcher] mission validation failed: %s", error.what());
        return 1;
    }

    std::vector<std::thread> threads;
    for (const auto& mission : missions) {
        auto manager = std::make_shared<MissionManager>(
            mission.vehicle_id, mission.patrol_waypoints);
        threads.emplace_back(&MissionManager::run_mission, manager);
        ROS_INFO("[Launcher] 已為 %s 啟動任務線程。",
                 mission.vehicle_id.c_str());
    }

    for (auto& th : threads) {
        if (th.joinable()) {
            th.join();
        }
    }

    ROS_INFO("[Launcher] 所有無人機的任務均已完成。主程序退出。");

    return 0;
}
