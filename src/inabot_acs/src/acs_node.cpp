#include "acs_node.hpp"
#include <iostream>

AcsNode::AcsNode()
: Node("robot_acs_node")
{
    // ROS2 파라미터 선언
    this->declare_parameter<std::string>("broker_host", "localhost");
    this->declare_parameter<int>("broker_port", 1883);
    this->declare_parameter<std::string>("client_id", "robot_acs_client");
    this->declare_parameter<bool>("clean_session", true);
    this->declare_parameter<int>("keep_alive_interval", 60);
    this->declare_parameter<int>("max_inflight", 65535);
    this->declare_parameter<bool>("tls_enabled", false);

    // Heartbeat / AGV 식별 관련 파라미터
    this->declare_parameter<int>("heartbeat_interval", 15);
    this->declare_parameter<std::string>("manufacturer", "Inatech");  // 기본값 임시
    this->declare_parameter<std::string>("serial_number", "P3LDD02"); // 기본값 임시

    // LWT 관련 파라미터
    this->declare_parameter<bool>("lwt_enabled", true);
    this->declare_parameter<std::string>("lwt_topic", "");
    this->declare_parameter<std::string>("lwt_payload", "");
    this->declare_parameter<int>("lwt_qos", 1);
    this->declare_parameter<bool>("lwt_retain", true);
}

AcsNode::~AcsNode()
{
    if (mqtt_client_) mqtt_client_->disconnect();
    // smart pointer 사용 중이므로 mission_manager_는 자동 해제

    if (heartbeat_manager_) heartbeat_manager_->stop(); // OFFLINE 전송
}

void AcsNode::init()
{
    // rclcpp::Node의 shared_from_this() 사용
    auto self = this->rclcpp::Node::shared_from_this();
    mission_manager_ = std::make_shared<MissionManager>(self);

    msg_handler_ = std::make_shared<MessageHandler>(mission_manager_);
 
    // BrokerConfig / ClientConfig 설정
    BrokerConfig broker_cfg;
    broker_cfg.host = get_parameter("broker_host").as_string();
    broker_cfg.port = get_parameter("broker_port").as_int();
    broker_cfg.tls.enabled = get_parameter("tls_enabled").as_bool();

    ClientConfig client_cfg;
    client_cfg.id = get_parameter("client_id").as_string();
    client_cfg.clean_session = get_parameter("clean_session").as_bool();
    client_cfg.keep_alive_interval = get_parameter("keep_alive_interval").as_int();
    client_cfg.max_inflight = get_parameter("max_inflight").as_int();

    //LWT
    client_cfg.lwt_enabled = get_parameter("lwt_enabled").as_bool();
    client_cfg.lwt_topic = get_parameter("lwt_topic").as_string();
    client_cfg.lwt_payload = get_parameter("lwt_payload").as_string();
    client_cfg.lwt_qos = get_parameter("lwt_qos").as_int();
    client_cfg.lwt_retain = get_parameter("lwt_retain").as_bool();
    
    // MQTT Client 생성
    mqtt_client_ = std::make_shared<MqttClient>(broker_cfg, client_cfg, msg_handler_);

   // Heartbeat Manager 생성
    int heartbeat_interval = get_parameter("heartbeat_interval").as_int();  // Heartbeat 주기
    std::string manufacturer = get_parameter("manufacturer").as_string();
    std::string serial_number = get_parameter("serial_number").as_string();

    heartbeat_manager_ = std::make_shared<HeartbeatManager>(
        manufacturer,
        serial_number,
        [this](const std::string& topic, const std::string& payload) {
            if (mqtt_client_) mqtt_client_->publish(topic, payload);
        },
        heartbeat_interval   // Heartbeat 주기
    );

    heartbeat_manager_->start();

    std::cout << "[INFO] Robot ACS Node initialized." << std::endl;
}

void AcsNode::shutdownMqtt()
{
    // OFFLINE 전송
    if (heartbeat_manager_)  {
        heartbeat_manager_->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    if (mqtt_client_) {
        try {
            mqtt_client_->disconnect();
        } catch (const std::exception &e) {
            std::cerr << "[AcsNode] MQTT disconnect error: " << e.what() << std::endl;
        }
    }
}
