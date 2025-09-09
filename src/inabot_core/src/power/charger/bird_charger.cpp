/*
Buzzard40 Bird Charger Node (ROS2 Humble)
- Protocol: CANopen
- Node ID: Bird 0x64, Master 0x01
- Baudrate: Default 250Kbps (Supports 125K, 250K, 500K, 1M)
- RPDO / TPDO CAN ID 
    RPDO / TPDO CAN ID
    RPDO1: Master → Bird : 0x180 + NodeID → 0x180 + 0x64 = 0x1E4
    TPDO1: Bird → Master : 0x182 + NodeID → 0x182 + 0x64 = 0x1E6
    TPDO2: Bird → Master : 0x282 + NodeID → 0x282 + 0x64 = 0x2E6
    Heartbeat: 0x700 + NodeID = 0x764
- TPDO / RPDO Mapping:
    RPDO1: 0x180 + NodeID -> Master → System Command
    TPDO1: 0x182 + NodeID -> Bird → System Status 1
    TPDO2: 0x282 + NodeID -> Bird → System Status 2
- Heartbeat: Bird / Master 1000ms
- CANbus Interface: SocketCAN 'can0'
*/

#include "bird_charger.hpp"
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>

// ======================= Structures =======================
struct __attribute__((packed)) BirdTPDO1 {
    uint16_t actual_current;   // 0x2002-0, Unit 1/256 A
    uint16_t actual_voltage;   // 0x2101-0, Unit 1/256 V
    uint16_t max_current;      // 0x4212-0, Unit 1/16 A
    uint16_t output_status;    // 0x2006-0, Bitfield
};

struct __attribute__((packed)) BirdTPDO2 {
    uint8_t  charging_state;   // 0x2007-0
    uint8_t  warning_number;   // 0x2050-0
    uint16_t fault_number;     // 0x2051-0
    uint8_t  alignment;        // 0x2008-0
    uint8_t  thermal_usage;    // 0x2009-0
    uint16_t charging_power;   // 0x2102-0, Unit 1/16 W
};

// ======================= Class =======================
BirdChargerNode::BirdChargerNode(const rclcpp::NodeOptions &options)
: Node("buzzard40_bird_node", options), BIRD_NODE_ID_(0x64), running_(true)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Bird Charger Node");

    loadParameters();
#if __bird_charger_dummy_can_test__
    test_can_sub_ = this->create_subscription<inabot_msgs::msg::CanFrame>(
        "can_tx",
        10,
        std::bind(&BirdChargerNode::testCanFrameCallback, this, std::placeholders::_1)
    );
#endif
    // Publisher
    pub_charger_status_ = this->create_publisher<inabot_msgs::msg::BirdChargerStatus>(
        "bird_charger/status", 10);

    // Service
    charger_service_ = this->create_service<inabot_msgs::srv::ChargerControl>(
        "charger_control",
        std::bind(&BirdChargerNode::chargerControlCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    //setupCAN();
    // Threads
    can_thread_ = std::thread(&BirdChargerNode::canReceiveLoop, this);
    heartbeat_thread_ = std::thread(&BirdChargerNode::heartbeatMonitorLoop, this);

    // Timer: TPDO → ROS Publish
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&BirdChargerNode::publishStatus, this));
}

BirdChargerNode::~BirdChargerNode()
{
    running_ = false;
    if(can_thread_.joinable()) can_thread_.join();
    if(heartbeat_thread_.joinable()) heartbeat_thread_.join();
    if(can_socket_ > 0) close(can_socket_);
}

// -------------------------------
void BirdChargerNode::loadParameters()
{
    this->declare_parameter<std::string>("can_interface", "can0");
    this->get_parameter("can_interface", can_interface_);
}

void BirdChargerNode::setupCAN()
{
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open CAN socket");
        return;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ-1);
    ifr.ifr_name[IFNAMSIZ-1] = '\0';

    if(ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_WARN(this->get_logger(), "CAN interface %s not found", can_interface_.c_str());
        close(can_socket_);
        can_socket_ = -1;
        return;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot bind CAN socket");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "CAN socket connected to %s", can_interface_.c_str());
}

// -------------------------------
void BirdChargerNode::canReceiveLoop()
{
    struct can_frame frame;
    int reconnect_attempts = 0;
    const int max_reconnect_attempts = 60; // 1 min

    while(running_) {
        if(can_socket_ < 0) {
            if(reconnect_attempts < max_reconnect_attempts) {
                RCLCPP_WARN(this->get_logger(),
                    "CAN socket lost, trying to reconnect... (%d/%d)", 
                    reconnect_attempts+1, max_reconnect_attempts);
                setupCAN();
                reconnect_attempts++;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "CAN socket reconnect failed after %d attempts, exiting node", 
                    max_reconnect_attempts);
                running_ = false; // 루프 종료
                return;           // 쓰레드 종료
            }
            continue; // 재연결 시도 후 루프 처음으로
        }
    
        reconnect_attempts = 0; // 성공하면 초기화

        int nbytes = read(can_socket_, &frame, sizeof(frame));
        if(nbytes > 0) {
            std::lock_guard<std::mutex> lock(can_mutex_);

            // -------------------------------
            // Heartbeat 처리
            // -------------------------------
            if(frame.can_id == 0x700 + BIRD_NODE_ID_) {
                last_heartbeat_ = std::chrono::steady_clock::now();
                uint8_t prev_nmt = nmt_state_;
                nmt_state_ = frame.data[0];

                if(prev_nmt != nmt_state_) {
                    std::string state_str;
                    switch(nmt_state_) {
                        case 0x00: state_str = "Boot"; break;
                        case 0x7F: state_str = "Pre-operational"; break;
                        case 0x05: state_str = "Operational"; break;
                        case 0x04: state_str = "Stopped"; break;
                        default:   state_str = "Unknown"; break;
                    }
                    RCLCPP_INFO(this->get_logger(),
                        "[Heartbeat] NMT State changed: %s (0x%02X)", state_str.c_str(), nmt_state_);
                }
            }

            // -------------------------------
            // Operational 상태에서만 TPDO 처리
            // -------------------------------
            if(nmt_state_ == 0x05) {
                if(frame.can_id == 0x182 + BIRD_NODE_ID_) {
                    BirdTPDO1 tpdo1{};
                    std::memcpy(&tpdo1, frame.data, sizeof(tpdo1));
                    tpdo_actual_current_ = tpdo1.actual_current;
                    tpdo_actual_voltage_ = tpdo1.actual_voltage;
                    tpdo_max_current_    = tpdo1.max_current;
                    tpdo_output_status_  = tpdo1.output_status;

                    // DEBUG 레벨 로그
                    RCLCPP_DEBUG(this->get_logger(),
                        "[TPDO1] Current: %.2f A, Voltage: %.2f V, Max: %.2f A, Output: 0x%04X",
                        tpdo_actual_current_ / 256.0f,
                        tpdo_actual_voltage_ / 256.0f,
                        tpdo_max_current_ / 16.0f,
                        tpdo_output_status_);
                }

                if(frame.can_id == 0x282 + BIRD_NODE_ID_) {
                    BirdTPDO2 tpdo2{};
                    std::memcpy(&tpdo2, frame.data, sizeof(tpdo2));
                    tpdo_charging_state_ = tpdo2.charging_state;
                    tpdo_warning_number_ = tpdo2.warning_number;
                    tpdo_fault_number_   = tpdo2.fault_number;
                    tpdo_alignment_      = tpdo2.alignment;
                    tpdo_thermal_usage_  = tpdo2.thermal_usage;
                    tpdo_charging_power_ = tpdo2.charging_power;

                    RCLCPP_DEBUG(this->get_logger(),
                        "[TPDO2] State: %d, Fault: %d, Warning: %d, Alignment: %d, Thermal: %d, Power: %.2f W",
                        tpdo_charging_state_,
                        tpdo_fault_number_,
                        tpdo_warning_number_,
                        tpdo_alignment_,
                        tpdo_thermal_usage_,
                        tpdo_charging_power_ / 16.0f);
                }
            }

        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

// -------------------------------
void BirdChargerNode::publishStatus()
{
    inabot_msgs::msg::BirdChargerStatus msg;
    msg.charging_state = tpdo_charging_state_;
    msg.voltage        = tpdo_actual_voltage_ / 256.0f;
    msg.current        = tpdo_actual_current_ / 256.0f;
    msg.max_current    = tpdo_max_current_ / 16.0f;
    msg.output_status  = tpdo_output_status_;
    msg.fault_number   = tpdo_fault_number_;
    msg.fault_string   = getFaultString(tpdo_fault_number_);
    msg.warning_number = tpdo_warning_number_;
    msg.warning_string = getWarningString(tpdo_warning_number_);
    msg.alignment      = tpdo_alignment_;
    msg.thermal_usage  = tpdo_thermal_usage_;
    msg.charging_power = tpdo_charging_power_ / 16.0f;
    msg.nmt_state      = nmt_state_;
    pub_charger_status_->publish(msg);
}

// -------------------------------
void BirdChargerNode::heartbeatMonitorLoop()
{
    last_heartbeat_ = std::chrono::steady_clock::now();
    uint8_t prev_nmt = nmt_state_;

    while(running_) {
        
        if (can_socket_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return;
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat_);
        
        if(elapsed.count() > 2000) {
            RCLCPP_WARN(this->get_logger(), "Heartbeat not received for 2s!");
        }

        // NMT 상태가 바뀌었을 때만 로그
        if(prev_nmt != nmt_state_) {
            prev_nmt = nmt_state_;
            std::string state_str;
            switch(nmt_state_) {
                case 0x00: state_str = "Boot"; break;
                case 0x7F: state_str = "Pre-operational"; break;
                case 0x05: state_str = "Operational"; break;
                case 0x04: state_str = "Stopped"; break;
                default:   state_str = "Unknown"; break;
            }
            RCLCPP_INFO(this->get_logger(), "Bird NMT State changed: %s (0x%02X)", state_str.c_str(), nmt_state_);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void BirdChargerNode::sendNMTOperational()
{
    struct can_frame frame{};
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x01;           // Start remote node
    frame.data[1] = BIRD_NODE_ID_;  // Node ID
    std::lock_guard<std::mutex> lock(can_mutex_);
    write(can_socket_, &frame, sizeof(frame));
}

// -------------------------------
//Node → Bird SDO 요청 전송
//SDO Expedited Write(4바이트) 사용
//Index/Subindex → Object Dictionary 주소 지정
//Value → Little Endian으로 전송
void BirdChargerNode::sendSDORequest(uint16_t index, uint8_t subindex, uint32_t value)
{
    if (can_socket_ < 0) {
        return;
    }

    struct can_frame frame{};
    //CANopen SDO 규칙 -> 확인 필요함.
    //SDO 요청/응답은 0x600 ~ 0x67F 범위의 COB-ID 사용
    //요청 (Client → Server, 여기서는 Master → Bird):
    frame.can_id = 0x600 + BIRD_NODE_ID_; //frame.can_id = 0x600 + 0x64 = 0x664
    frame.can_dlc = 8;
    frame.data[0] = 0x23;           // expedited write, 4 bytes
    frame.data[1] = index & 0xFF;   // Index LSB
    frame.data[2] = index >> 8;     // Index MSB
    frame.data[3] = subindex;       // Subindex
    frame.data[4] = value & 0xFF;   // Value LSB
    frame.data[5] = (value >> 8) & 0xFF;
    frame.data[6] = (value >> 16) & 0xFF;
    frame.data[7] = (value >> 24) & 0xFF; // Value MSB
    std::lock_guard<std::mutex> lock(can_mutex_);
    write(can_socket_, &frame, sizeof(frame));
}

//Bird 충전 시작 명령 시퀀스
//RPDO 1 : MASTER - System Command 1
//Enable → Bird 연결 활성화
//Reset → Fault 초기화
//Mode → Voltage control
//Charge Voltage → 설정 전압
//Charge Current → 설정 전류
//Run → 충전 시작
void BirdChargerNode::startCharging(float voltage, float current)
{
    sendSDORequest(0x4200, 0, 1); // Enable (connect)
    sendSDORequest(0x4201, 0, 1); // Reset faults
    sendSDORequest(0x4203, 0, 1); // Mode = Voltage control
    sendSDORequest(0x2276, 0, static_cast<uint32_t>(voltage * 256));
    sendSDORequest(0x6070, 0, static_cast<uint32_t>(current * 16));
    sendSDORequest(0x4202, 0, 1); // Run (start)
}

void BirdChargerNode::stopCharging()
{
    sendSDORequest(0x4202, 0, 0); // Run = Stop
}

// -------------------------------
void BirdChargerNode::chargerControlCallback(
    const inabot_msgs::srv::ChargerControl::Request::SharedPtr req,
    inabot_msgs::srv::ChargerControl::Response::SharedPtr res)
{
    if(req->start) {
        RCLCPP_INFO(this->get_logger(),
                    "ChargerControl: START requested (Voltage=%.2f V, Current=%.2f A)",
                    req->voltage, req->current);
        startCharging(req->voltage, req->current);
    } else {
        RCLCPP_INFO(this->get_logger(), "ChargerControl: STOP requested");
        stopCharging();
    }

    res->success = true;
    RCLCPP_INFO(this->get_logger(), "ChargerControl: Response success = %s", 
                res->success ? "true" : "false");
}

void BirdChargerNode::testCanFrameCallback(const inabot_msgs::msg::CanFrame::SharedPtr msg)
{
    uint32_t can_id = msg->can_id;
    const std::vector<uint8_t>& data = msg->data;

    std::lock_guard<std::mutex> lock(can_mutex_);

    // -------------------------------
    // Heartbeat 처리
    if(can_id == 0x700 + BIRD_NODE_ID_ && !data.empty()) {
        last_heartbeat_ = std::chrono::steady_clock::now();
        uint8_t prev_nmt = nmt_state_;
        nmt_state_ = data[0];

        if(prev_nmt != nmt_state_) {
            std::string state_str;
            switch(nmt_state_) {
                case 0x00: state_str = "Boot"; break;
                case 0x7F: state_str = "Pre-operational"; break;
                case 0x05: state_str = "Operational"; break;
                case 0x04: state_str = "Stopped"; break;
                default:   state_str = "Unknown"; break;
            }
            RCLCPP_INFO(this->get_logger(),
                        "[Heartbeat] NMT State changed: %s (0x%02X)", state_str.c_str(), nmt_state_);
        }
    }

    // -------------------------------
    // Operational 상태에서만 TPDO 처리
    if(nmt_state_ == 0x05) {
        if(can_id == 0x182 + BIRD_NODE_ID_ && data.size() >= sizeof(BirdTPDO1)) {
            BirdTPDO1 tpdo1{};
            std::memcpy(&tpdo1, data.data(), sizeof(tpdo1));
            tpdo_actual_current_ = tpdo1.actual_current;
            tpdo_actual_voltage_ = tpdo1.actual_voltage;
            tpdo_max_current_    = tpdo1.max_current;
            tpdo_output_status_  = tpdo1.output_status;

            RCLCPP_INFO(this->get_logger(),
                         "[TPDO1] Current: %.2f A, Voltage: %.2f V, Max: %.2f A, Output: 0x%04X",
                         tpdo_actual_current_ / 256.0f,
                         tpdo_actual_voltage_ / 256.0f,
                         tpdo_max_current_ / 16.0f,
                         tpdo_output_status_);
        }

        if(can_id == 0x282 + BIRD_NODE_ID_ && data.size() >= sizeof(BirdTPDO2)) {
            BirdTPDO2 tpdo2{};
            std::memcpy(&tpdo2, data.data(), sizeof(tpdo2));
            tpdo_charging_state_ = tpdo2.charging_state;
            tpdo_warning_number_ = tpdo2.warning_number;
            tpdo_fault_number_   = tpdo2.fault_number;
            tpdo_alignment_      = tpdo2.alignment;
            tpdo_thermal_usage_  = tpdo2.thermal_usage;
            tpdo_charging_power_ = tpdo2.charging_power;

            RCLCPP_INFO(this->get_logger(),
                         "[TPDO2] State: %d, Fault: %d, Warning: %d, Alignment: %d, Thermal: %d, Power: %.2f W",
                         tpdo_charging_state_,
                         tpdo_fault_number_,
                         tpdo_warning_number_,
                         tpdo_alignment_,
                         tpdo_thermal_usage_,
                         tpdo_charging_power_ / 16.0f);
        }
    }
}