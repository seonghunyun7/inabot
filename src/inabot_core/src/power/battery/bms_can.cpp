/*
Battery CAN Node (ROS2 Humble)
- Protocol : CAN 2.0A, 250Kbps, 8 bytes
- Message 주기: 1초
- Connector: DSub-9pin Female (Pin2 – CAN L / Pin7 – CAN H)
- Data Format: Big Endian
- CAN 인터페이스가 없을 경우 안전하게 재시도하며 계속 대기
- 수신 데이터는 sensor_msgs/msg/BatteryState 로 publish
*/
#include "power/battery/bms_can.hpp"
#include <chrono>
#include <iostream>

CBmsCan::CBmsCan(const rclcpp::NodeOptions& options)
: Node("bms_can", options), can_socket_(-1)
{
    loadParameters();

    RCLCPP_INFO(this->get_logger(), "Starting BMS CAN node on %s", can_interface_.c_str());

    // 퍼블리셔: 커스텀 메시지
    batt_pub_ = this->create_publisher<inabot_msgs::msg::BmsState>("battery_state", 10);
 
    #if __DUMMY_TEST__
    // ================================
    // inabot_ws/python$ python3 dummy_bms_can_pub.py 
    // 테스트용 더미 CAN 프레임 구독
    // 토픽: "bms_can_frame"
    // 메시지 타입: inabot_msgs::msg::BmsCanFrame
    // 큐 사이즈: 10
    // =================================
    test_can_frame_sub_ = this->create_subscription<inabot_msgs::msg::BmsCanFrame>(
        "bms_can_frame", 10,
        std::bind(&CBmsCan::testCanFrameCallback, this, std::placeholders::_1)
    );
    #endif

    // 주기적 CAN 읽기 (1초)
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&CBmsCan::readCan, this));
}

CBmsCan::~CBmsCan()
{
    if (can_socket_ >= 0) {
        close(can_socket_);
    }
}

void CBmsCan::loadParameters()
{
    this->declare_parameter<std::string>("can_interface", "can0");
    this->get_parameter("can_interface", can_interface_);
}

void CBmsCan::connectCan()
{
    if (can_socket_ >= 0) return;

    struct ifreq ifr;
    struct sockaddr_can addr;

    if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to create CAN socket");
        can_socket_ = -1;
        return;
    }

    //std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ);
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_WARN(this->get_logger(), "CAN interface %s not found", can_interface_.c_str());
        close(can_socket_);
        can_socket_ = -1;
        return;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to bind CAN interface %s", can_interface_.c_str());
        close(can_socket_);
        can_socket_ = -1;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "CAN interface %s connected successfully", can_interface_.c_str());
}

void CBmsCan::readCan()
{
    // CAN 소켓이 없으면 연결 시도
    if (can_socket_ < 0) {
        static int retry_count = 0;

        connectCan();
        if (can_socket_ < 0) {
            retry_count++;
            RCLCPP_WARN(this->get_logger(), "CAN connect attempt #%d failed, retrying in 1 second...", retry_count);
        } else {
            retry_count = 0;
        }
        return;
    }

    struct can_frame frame;
    int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

    if (nbytes < 0) {
        RCLCPP_ERROR(this->get_logger(), "CAN read error, closing socket and will retry");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }

    if (nbytes < (int)sizeof(struct can_frame)) {
        RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame received");
        return;
    }

    if (parseMessage(frame.can_id, frame.data, frame.can_dlc)) {
        publishState();
    }
}

int16_t CBmsCan::toInt16BE(const uint8_t* d)
{
    return (int16_t)((d[0] << 8) | d[1]);
}

uint16_t CBmsCan::toUInt16BE(const uint8_t* d)
{
    return (uint16_t)((d[0] << 8) | d[1]);
}

bool CBmsCan::parseMessage(uint32_t msg_id, const uint8_t* data, size_t len)
{
    uint8_t battery_id = msg_id >> 8; // 상위 바이트: 배터리 ID
    uint8_t base_id    = msg_id & 0xFF; // 하위 바이트: 메시지 유형

    current_state_.battery_id = battery_id;

    if (base_id == 0x01 && len >= 8) {
        // 0x01: 배터리 기본 정보
        // data[0] : Hardware version * 0.1
        // data[1] : Firmware version * 0.1
        // data[2..3] : Capacity (mAh) / 10
        // data[4..5] : Current charge (mAh) / 10
        // data[6..7] : Cycle count
        current_state_.hardware_ver = data[0] * 0.1f;
        current_state_.firmware_ver = data[1] * 0.1f;
        current_state_.capacity     = toUInt16BE(&data[2]) / 10.0f;
        current_state_.charge       = toUInt16BE(&data[4]) / 10.0f;
        current_state_.cycle_count  = toUInt16BE(&data[6]);
        return true;
    } 
    else if (base_id == 0x02 && len >= 2) {
        // 0x02: 배터리 전체 전압
        // data[0..1] : Voltage (V) * 0.1
        current_state_.voltage = toUInt16BE(&data[0]) / 10.0f;
        return true;
    } 
    else if (base_id == 0x03 && len >= 8) {
        // 0x03: 배터리 상태 정보
        // data[0] : Battery status
        // data[2..3] : Current (A) / 10
        // data[4..5] : SOC (%) / 100
        // data[6] : SOH (%)
        // data[7] : Serial cells 수
        // 또한 보호, 경고, 밸런싱 플래그 포함
        current_state_.current      = toInt16BE(&data[2]) / 10.0f;
        current_state_.percentage   = toUInt16BE(&data[4]) / 100.0f;
        current_state_.soh          = data[6];
        current_state_.serial_cells = data[7];

        current_state_.battery_status   = data[0];
        current_state_.protection_flag  = toUInt16BE(&data[2]);
        current_state_.warning_flag     = toUInt16BE(&data[4]);
        current_state_.balancing_status = toUInt16BE(&data[6]);
        return true;
    } 
    else if (base_id == 0x04 && len >= 4) {
        // 0x04: 배터리 온도
        // data[0..1] : Temperature sensor 1 (°C) * 0.1
        // data[2..3] : Temperature sensor 2 (°C) * 0.1
        // 두 센서 평균값 저장
        float t1 = toInt16BE(&data[0]) / 10.0f;
        float t2 = toInt16BE(&data[2]) / 10.0f;
        current_state_.temperature = (t1 + t2) / 2.0f;
        return true;
    } 
    else if (base_id >= 0x05 && base_id <= 0x09 && len == 8) {
        // 0x05~0x09: 개별 셀 전압 (8 bytes = 4 cells)
        // base_id 0x05: 첫 프레임, 벡터 초기화
        if (base_id == 0x05) {
            current_state_.cell_voltage.clear();
        }

        for (size_t i = 0; i < 8; i += 2) {
            current_state_.cell_voltage.push_back(toUInt16BE(&data[i]) / 1000.0f); // V 단위
        }
        return true;
    }

    return false; // 정의되지 않은 메시지 ID
}

void CBmsCan::publishState()
{
    batt_pub_->publish(current_state_);
}

// 테스트용 CAN 프레임 콜백
void CBmsCan::testCanFrameCallback(const inabot_msgs::msg::BmsCanFrame::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
        "Received CAN Frame: ID=0x%X DLC=%u Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
        msg->id, msg->dlc,
        msg->data[0], msg->data[1], msg->data[2], msg->data[3],
        msg->data[4], msg->data[5], msg->data[6], msg->data[7]);

    if (parseMessage(msg->id, msg->data.data(), msg->dlc)) {
        publishState();
    }
}
