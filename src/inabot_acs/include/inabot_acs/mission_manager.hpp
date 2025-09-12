#pragma once
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// order 구조체
struct NodePosition {
    double x;
    double y;
    std::string mapId;
    double allowedDeviationXY;
};

struct Action {  // InstantActions 및 Node 내부 Action 통합
    std::string actionId;
    std::string type;  // Order용: actionType으로도 매핑
    std::string actionType;        // Order 전용
    std::string actionDescription; // Order 전용
    std::string blockingType;      // Order 전용
    nlohmann::json parameters;     // InstantActions 전용
};

struct Node {
    std::string nodeId;
    int sequenceId;
    bool released;
    std::string nodeDescription;
    NodePosition position;

    std::vector<Action> actions;  // Node에 포함된 Actions
};

struct Edge {
    std::string edgeId;
    int sequenceId;
    bool released;
    std::string startNodeId;
    std::string endNodeId;
    double maxSpeed;
    double orientation;
    bool rotationAllowed;
};

struct Header {
    int headerId;
    std::string timestamp;
    std::string version;
    std::string manufacturer;
    std::string serialNumber;
};

class MissionManager {
public:
    using DisconnectCallback = std::function<void()>;

    explicit MissionManager(std::shared_ptr<rclcpp::Node> node);

    // VD5050D 전용 메시지 핸들러
    void handleOrder(const nlohmann::json& payload);
    void handleInstantActions(const nlohmann::json& payload);

    // TEST 메시지 핸들러
    void handleMission(const nlohmann::json& payload);
    void handleControl(const nlohmann::json& payload);
    void handleHeartbeat(const nlohmann::json& payload);

    void setDisconnectCallback(DisconnectCallback cb);

private:
    std::shared_ptr<rclcpp::Node> node_;

    // VD5050D 퍼블리셔
    // FMS => ROBOT
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr order_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr instant_actions_pub_;

    // TEST용 퍼블리셔
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

    DisconnectCallback disconnect_cb_;
};
