#include "sick_safety_controller/safety_controller.hpp"

SafetyController::SafetyController() : Node("safety_controller")
{
    this->declare_parameter<std::string>("modbus_ip", "192.168.0.10");
    this->declare_parameter<int>("modbus_port", 502);

    std::string ip;
    int port;
    this->get_parameter("modbus_ip", ip);
    this->get_parameter("modbus_port", port);

    // Modbus TCP 연결
    ctx_ = modbus_new_tcp(ip.c_str(), port);
    if (modbus_connect(ctx_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to Modbus");
        modbus_free(ctx_);
        ctx_ = nullptr;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    status_pub_ = this->create_publisher<inabot_msgs::msg::PLCStatus>("/sick_safety/plc_status", qos);

    control_sub_ = this->create_subscription<inabot_msgs::msg::PLCControl>(
        "/sick_safety/control", qos, std::bind(&SafetyController::controlCallback, this, std::placeholders::_1));

    // 500ms 주기 타이머
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SafetyController::timerCallback, this)
    );
}

SafetyController::~SafetyController()
{
    reset();
}

void SafetyController::reset()
{
    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "Modbus connection closed by reset()");
    }
}

std::vector<std::vector<bool>> SafetyController::readInputs()
{
    std::vector<bool> in_states, emo_states, out_states;

    if (!ctx_) return {in_states, emo_states, out_states};
 
    uint16_t regs[3];
    // Modbus TCP에서 256번 레지스터부터 3개 읽기
    // regs[0] = 256번 레지스터 → Safety Inputs
    // regs[1] = 257번 레지스터 → EMO / 상태
    // regs[2] = 258번 레지스터 → Safety Outputs

    int rc = modbus_read_registers(ctx_, 256, 3, regs);
    if (rc == -1) {
        RCLCPP_WARN(this->get_logger(), "Failed to read Modbus registers");
        return {in_states, emo_states, out_states};
    }

    // 각 16비트 레지스터를 16개의 bool 값으로 분리
    for (int i = 0; i < 16; i++) {
        // regs[0]에서 비트 i를 추출하여 in_states에 추가
        // (>> i) : i번째 비트로 쉬프트
        // & 0x1 : 최하위 비트만 추출 (0 또는 1)
        in_states.push_back((regs[0] >> i) & 0x1);

        // regs[1]에서 비트 i를 추출하여 emo_states에 추가
        emo_states.push_back((regs[1] >> i) & 0x1);

        // regs[2]에서 비트 i를 추출하여 out_states에 추가
        out_states.push_back((regs[2] >> i) & 0x1);
    }

    // 세 가지 상태 벡터를 포함한 2차원 벡터 반환
    // 0: in_states, 1: emo_states, 2: out_states
    return {in_states, emo_states, out_states};
}

void SafetyController::writeOutput(int coil_type, bool value)
{
    if (!ctx_) return;

    if (coil_type == TYPE_CHARGE) {
        uint16_t output_word = value ? (1 << 7) : 0;
        int rc = modbus_write_register(ctx_, 256, output_word);
        if (rc == -1)
            RCLCPP_WARN(this->get_logger(), "Failed to write Modbus register");
        else
            RCLCPP_INFO(this->get_logger(), "Wrote coil %d = %d", coil_type, value);
    } else {
        RCLCPP_WARN(this->get_logger(), "coil_type %d not implemented", coil_type);
    }
}

void SafetyController::timerCallback()
{
    if (!ctx_) return;

    uint16_t regs[3];  // 256~258번지
    int rc = modbus_read_registers(ctx_, 256, 3, regs);
    if (rc == -1) {
        RCLCPP_WARN(this->get_logger(), "Failed to read Modbus registers");
        return;
    }

    // ------------------------
    // IN / EMO / OUT 배열
    // ------------------------
    std::vector<bool> in_states, emo_states, out_states;
    for (int i = 0; i < 16; i++) {
        in_states.push_back((regs[0] >> i) & 0x1);
        emo_states.push_back((regs[1] >> i) & 0x1);
        out_states.push_back((regs[2] >> i) & 0x1);
    }

    std::stringstream ss_in, ss_emo, ss_out;
    ss_in << "IN : ";
    ss_emo << "EMO: ";
    ss_out << "OUT: ";
    for (int i = 0; i < 16; i++) {
        ss_in << in_states[i];
        ss_emo << emo_states[i];
        ss_out << out_states[i];
        if (i != 15) {
            ss_in << " ";
            ss_emo << " ";
            ss_out << " ";
        }
    }

    RCLCPP_INFO(this->get_logger(), "%s", ss_in.str().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", ss_emo.str().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", ss_out.str().c_str());

    inabot_msgs::msg::PLCStatus msg;
    msg.in_states = in_states;
    msg.emo_states = emo_states;
    msg.out_states = out_states;

    std::stringstream ss;
    ss << "Publishing PLCStatus -> IN: ";
    for (auto b : msg.in_states) ss << b << " ";
    ss << "| EMO: ";
    for (auto b : msg.emo_states) ss << b << " ";
    ss << "| OUT: ";
    for (auto b : msg.out_states) ss << b << " ";

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    status_pub_->publish(msg);
}


void SafetyController::controlCallback(const inabot_msgs::msg::PLCControl::SharedPtr msg)
{
    //
}
