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
}

AcsNode::~AcsNode()
{
    if (mqtt_client_) mqtt_client_->disconnect();
    // smart pointer 사용 중이므로 mission_manager_는 자동 해제
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

    // MQTT Client 생성
    mqtt_client_ = std::make_shared<MqttClient>(broker_cfg, client_cfg, msg_handler_);

    std::cout << "[INFO] Robot ACS Node initialized." << std::endl;
}
