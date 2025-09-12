#pragma once
#include "rclcpp/rclcpp.hpp"
#include "mqtt_client.hpp"
#include "mission_manager.hpp"
#include "message_handler.hpp"
#include "heartbeat_manager.hpp"

#include "message_sender.hpp"

// acs_node.hpp
class AcsNode : public rclcpp::Node {
public:
    AcsNode();
    ~AcsNode();

    void init();

    // MQTT EXIT
    void shutdownMqtt();

private:
    std::shared_ptr<MissionManager> mission_manager_;
    std::shared_ptr<MessageHandler> msg_handler_;
    std::shared_ptr<MqttClient> mqtt_client_;

    //VDA5050 connection topic + Last Will/Heartbeat 적용
    //AGV가 온라인/오프라인 상태를 Master Control에 자동 보고
    //MQTT Last Will를 통해 예기치 않은 연결 끊김 감지
    //QoS=1, Retained flag 적용으로 Master Control에서 항상 최신 상태 확인 가능
    //ROS2 SignalHandler와 통합 → 종료 시 OFFLINE 상태 발행
    std::shared_ptr<HeartbeatManager> heartbeat_manager_;
    std::shared_ptr<MessageSender> robot_info_sender_;

};
