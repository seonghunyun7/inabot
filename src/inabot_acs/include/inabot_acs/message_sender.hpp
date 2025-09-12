#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mqtt_client.hpp"
#include <string>
#include <vector>
#include <unordered_map>

class MessageSender {
public:
    /**
     * @param node          : ROS2 노드 포인터 (shared_from_this() 필요)
     * @param mqtt_client   : MQTT 클라이언트
     * @param manufacturer  : AGV 제조사 (connection 메시지에 사용)
     * @param serial_number : AGV 일련번호 (connection 메시지에 사용)
     */
    MessageSender(std::shared_ptr<rclcpp::Node> node,
                  std::shared_ptr<MqttClient> mqtt_client,
                  const std::string& manufacturer,
                  const std::string& serial_number);

    ~MessageSender() = default;

    /**
     * ROS 토픽과 MQTT 토픽을 매핑하여 구독/전송 등록
     * @param ros_topic   : 로봇 정보를 받을 ROS2 토픽 이름
     * @param mqtt_topic  : FMS로 발행할 MQTT 토픽 이름
     */
    void addTopic(const std::string& ros_topic, const std::string& mqtt_topic);

private:
    void rosCallback(const std::string& mqtt_topic,
                     const std_msgs::msg::String::SharedPtr msg);

    struct SubInfo {
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
        std::string mqtt_topic;
    };

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MqttClient> mqtt_client_;
    std::vector<SubInfo> subscriptions_;

    std::string manufacturer_;
    std::string serial_number_;

    // 각 MQTT 토픽별 headerId 카운터
    std::unordered_map<std::string, uint32_t> header_counter_;
};
