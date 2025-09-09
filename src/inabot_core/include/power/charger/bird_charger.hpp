#ifndef BIRD_CHARGER_NODE_HPP
#define BIRD_CHARGER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include "std_msgs/msg/string.hpp"
#include "inabot_msgs/srv/charger_control.hpp"
#include "inabot_msgs/msg/bird_charger_status.hpp"
#include "inabot_msgs/msg/can_frame.hpp"

#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <unordered_map>
#include <string>

class BirdErrorHandler {
public:
    BirdErrorHandler() {
        // Error codes
        error_table_ = {
            {1001, "Reverse current protection"},
            {1002, "Battery disconnected during charging"},
            {1004, "Temperature sensor electronics 1"},
            {1005, "Temperature sensor electronics 2"},
            {1006, "Temperature sensor coil 1"},
            {1007, "Temperature sensor coil 2"},
            {1008, "Temperature sensor rectifier 1"},
            {1009, "Temperature sensor rectifier 2"},
            {1012, "Overtemperature on electronics"},
            {1013, "Overtemperature on coil"},
            {1014, "Overtemperature on rectifier"},
            {1016, "No communication Bird <> Nest"},
            {1017, "Output contactor 1"},
            {1018, "Output contactor 2"},
            {1019, "Output contactor 3"},
            {1020, "Nest not found"},
            {1021, "Wireless link hardware failed"},
            {1022, "Pairing"},
            {1024, "Output voltage too low"},
            {1025, "Internal hardware"},
            {1031, "CAN timeout"},
            {1033, "Overcurrent protection"},
            {1034, "AC leakage current"},
            {1041, "HW not enabled"},
            {1042, "Output voltage too high"},
            {1043, "Internal circuit clock failed"},
            {1053, "Internal DCDC voltage 1"},
            {1054, "Internal DCDC voltage 2"},
            {1057, "Internal current sensor reference"},
            {1058, "Internal fuse broken"},
            {1059, "Temperature sensor electronics 1"},
            {1060, "Temperature sensor electronics 2"},
            {1061, "Temperature sensor coil 1"},
            {1062, "Temperature sensor coil 2"},
            {1063, "Temperature sensor rectifier 1"},
            {1064, "Temperature sensor rectifier 2"},
            {1067, "Dump circuit not resettable"},
            {1068, "Reverse current circuit not resettable"},
            {1069, "Contactor circuit"},
            {1070, "Wireless link hardware failed"},
            {1075, "Overcurrent circuit not resettable"},
            {1076, "Reset by software protection watchdog"}
        };

        // Warning codes
        warning_table_ = {
            {1, "Derating by electronics temperature"},
            {2, "Derating by coil temperature"},
            {3, "Derating by rectifier temperature"}
        };
    }

    std::string getErrorMessage(int code) const {
        auto it = error_table_.find(code);
        return it != error_table_.end() ? it->second : "Unknown error code";
    }

    std::string getWarningMessage(int code) const {
        auto it = warning_table_.find(code);
        return it != warning_table_.end() ? it->second : "Unknown warning code";
    }

private:
    std::unordered_map<int, std::string> error_table_;
    std::unordered_map<int, std::string> warning_table_;
};

class BirdChargerNode : public rclcpp::Node
{
public:
    BirdChargerNode(const rclcpp::NodeOptions &options);
    ~BirdChargerNode();

private:
    const uint8_t BIRD_NODE_ID_;
    std::atomic<bool> running_{true};
    int can_socket_{-1};
    std::string can_interface_;

    std::thread can_thread_;
    std::thread heartbeat_thread_;
    std::mutex can_mutex_;

    // TPDO1
    uint16_t tpdo_actual_current_{0};
    uint16_t tpdo_actual_voltage_{0};
    uint16_t tpdo_max_current_{0};
    uint8_t  tpdo_output_status_{0};

    // TPDO2
    uint8_t  tpdo_charging_state_{0};
    uint8_t  tpdo_warning_number_{0};
    uint8_t  tpdo_fault_number_{0};
    uint8_t  tpdo_alignment_{0};
    uint8_t  tpdo_thermal_usage_{0};
    uint16_t tpdo_charging_power_{0};

    // -------------------------------
    // Heartbeat 관련
    // -------------------------------
    uint8_t nmt_state_{0};// Heartbeat에서 수신한 NMT 상태 저장
    std::chrono::time_point<std::chrono::steady_clock> last_heartbeat_;

    // ROS Publisher (단일 커스텀 메시지)
    rclcpp::Publisher<inabot_msgs::msg::BirdChargerStatus>::SharedPtr pub_charger_status_;

    // for_dummy_data_test
    rclcpp::Subscription<inabot_msgs::msg::CanFrame>::SharedPtr test_can_sub_;

    BirdErrorHandler error_handler_;

    // Service
    rclcpp::Service<inabot_msgs::srv::ChargerControl>::SharedPtr charger_service_;

    rclcpp::TimerBase::SharedPtr status_timer_;

    // -------------------------------
    void loadParameters();
    void setupCAN();
    void canReceiveLoop();
    void heartbeatMonitorLoop();
    void publishStatus();

    void sendNMTOperational();
    void sendSDORequest(uint16_t index, uint8_t subindex, uint32_t value);
    void startCharging(float voltage, float current);
    void stopCharging();

    void chargerControlCallback(
        const inabot_msgs::srv::ChargerControl::Request::SharedPtr req,
        inabot_msgs::srv::ChargerControl::Response::SharedPtr res);

    std::string getFaultString(uint16_t fault_number) {
        return error_handler_.getErrorMessage(fault_number);
    }

    std::string getWarningString(uint8_t warning_number) {
        return error_handler_.getWarningMessage(warning_number);
    }
    
    void testCanFrameCallback(const inabot_msgs::msg::CanFrame::SharedPtr msg);
};

#endif // BIRD_CHARGER_NODE_HPP
