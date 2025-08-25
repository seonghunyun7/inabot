#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <atomic>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <boost/stacktrace.hpp>
#include <boost/version.hpp>
#include <execinfo.h>
#include <fstream>
#include <ctime>
#include "utils/crash_handler.hpp"
//#include "utils/logger.h"

#include "motor.hpp"
#include "odometry.hpp"
#include "imu.hpp"
#include "monitor/monitor.hpp"

#include "planner/local/pure_pursuit/pure_pursuit.hpp"
#include "planner/local/mpc_motion_controller/mpc_motion_controller.hpp" 
#include "planner/global/global_planner.hpp"
#include "planner/lidar_obstacle/lidar_obstacle_detector.hpp"

using namespace Inabot;

rclcpp::executors::MultiThreadedExecutor* global_executor = nullptr;

void displayHelp()
{
    std::cout << "[INFO]   ___           _         _     " << std::endl;
    std::cout << "[INFO]  |_ _|_ __  ___| |_ _   _| |__  " << std::endl;
    std::cout << "[INFO]   | || '_ \\/ __| __| | | | '_ \\ " << std::endl;
    std::cout << "[INFO]   | || | | \\__ \\ |_| |_| | |_) |" << std::endl;
    std::cout << "[INFO]  |___|_| |_|___/\\__|\\__,_|_.__/ " << std::endl;
    std::cout << "[INFO] ..............................................." << std::endl;
    std::cout << "[INFO] Copyright (c) 2025 by InaBot. All rights reserved." << std::endl;
}

int main(int argc, char** argv)
{
    // Logger::get().SetSeverityMin(severity_level::info); // 로거 초기화 제거

    std::cout << "[INFO] Inabot core initialization." << std::endl;

    rclcpp::init(argc, argv);

    displayHelp();

    rclcpp::NodeOptions options;

    CrashHandler::setup();

/*
* 참고사항:
* - 빌드 옵션을 변경할 때는 기존 빌드와 설치 폴더를 반드시 삭제(clean)한 후 빌드하는 것을 권장합니다.
* - 예)
*   rm -rf build install log
* [빌드 옵션] cmake에서 USE_PURE_PURSUIT를 ON으로 설정하면 이 부분이 활성화됩니다.
*   - option(USE_PURE_PURSUIT "Use Pure Pursuit planner" OFF)
*/
#if _ACTIVATE__
    auto motor_node = std::make_shared<MotorDriver>(options);
    if (!motor_node->init()) {
        std::cout << "[INFO] MotorDriver initialization failed." << std::endl;
        return -1;
    }  
    auto odom_node = std::make_shared<OdometryNode>(options); 
    auto imu_node = std::make_shared<ImuNode>(options);
    auto monitor_node = std::make_shared<MonitorNode>(options);

#ifdef USE_PURE_PURSUIT 
    auto pure_pursuit_node = std::make_shared<pure_pursuit>(options);
    std::cout << "[INFO] Using Pure Pursuit planner." << std::endl;
#else
    auto mpc_motion_controller_node = std::make_shared<MpcMotionController>(options);
    std::cout << "[INFO] Using MPC motion controller." << std::endl;
#endif
#endif
    auto global_planner_node = std::make_shared<GlobalPlanner>(options);
    auto lidar_obstacle_detector_node = std::make_shared<LidarObstacleDetector>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    global_executor = &executor;

#if _ACTIVATE_
    executor.add_node(motor_node);  
    executor.add_node(odom_node);
    executor.add_node(imu_node);
    executor.add_node(monitor_node);

#ifdef USE_PURE_PURSUIT
    executor.add_node(pure_pursuit_node);
#else
    executor.add_node(mpc_motion_controller_node);
#endif
#endif
    executor.add_node(global_planner_node);
    executor.add_node(lidar_obstacle_detector_node);

    try {
        executor.spin();
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Exception caught: " << e.what() << std::endl;
    }

    std::cout << "[INFO] Inabot core exit." << std::endl;

    rclcpp::shutdown();
    return 0;
}