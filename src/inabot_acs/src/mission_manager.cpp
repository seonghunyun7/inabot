// mission_manager.cpp
#include "mission_manager.hpp"
#include <iostream>

MissionManager::MissionManager(std::shared_ptr<rclcpp::Node> node)
: node_(node)
{
    // TEST용 퍼블리셔
#if __FOR_TEST
    mission_pub_ = node_->create_publisher<std_msgs::msg::String>("mission_topic", 10);
    control_pub_ = node_->create_publisher<std_msgs::msg::String>("control_topic", 10);
    heartbeat_pub_ = node_->create_publisher<std_msgs::msg::String>("heartbeat_topic", 10);
#endif
    // VDA5050D Pub (FMS -> ROBOT)
    order_pub_ = node_->create_publisher<std_msgs::msg::String>("order_topic", 10);
    instant_actions_pub_ = node_->create_publisher<std_msgs::msg::String>("instant_actions_topic", 10);

    std::cout << "[mission_manager] Publishers created" << std::endl;
}

void MissionManager::setDisconnectCallback(DisconnectCallback cb)
{
    disconnect_cb_ = cb;
}

// VD5050D 핸들러
void MissionManager::handleOrder(const nlohmann::json& payload)
{
    std::cout << "[mission_manager] Handling order:\n" << payload.dump(4) << std::endl;

    std::vector<Node> nodes;
    for (auto& n : payload["nodes"]) {
        Node node;
        node.nodeId = n.value("nodeId", "");
        node.sequenceId = n.value("sequenceId", 0);
        node.released = n.value("released", false);
        node.nodeDescription = n.value("nodeDescription", "");
        node.position.x = n["nodePosition"].value("x", 0.0);
        node.position.y = n["nodePosition"].value("y", 0.0);
        node.position.mapId = n["nodePosition"].value("mapId", "");
        node.position.allowedDeviationXY = n["nodePosition"].value("allowedDeviationXY", 0.0);

        for (auto& a : n["actions"]) {
            Action action;
            action.actionId = a.value("actionId", "");
            action.actionType = a.value("actionType", "");
            action.actionDescription = a.value("actionDescription", "");
            action.blockingType = a.value("blockingType", "");
            node.actions.push_back(action);
        }
        nodes.push_back(node);
    }

    std::vector<Edge> edges;
    for (auto& e : payload["edges"]) {
        Edge edge;
        edge.edgeId = e.value("edgeId", "");
        edge.sequenceId = e.value("sequenceId", 0);
        edge.released = e.value("released", false);
        edge.startNodeId = e.value("startNodeId", "");
        edge.endNodeId = e.value("endNodeId", "");
        edge.maxSpeed = e.value("maxSpeed", 0.0);
        edge.orientation = e.value("orientation", 0.0);
        edge.rotationAllowed = e.value("rotationAllowed", false);
        edges.push_back(edge);
    }

    std_msgs::msg::String msg;
    msg.data = payload.dump();
    order_pub_->publish(msg);

    std::cout << "[mission_manager] Order published. Nodes: " << nodes.size()
              << ", Edges: " << edges.size() << std::endl;

    for (auto& node : nodes) {
        std::cout << "Node ID: " << node.nodeId
                  << ", Seq: " << node.sequenceId
                  << ", Pos: (" << node.position.x << ", " << node.position.y << ")"
                  << ", Actions: " << node.actions.size() << std::endl;

        for (auto& action : node.actions) {
            std::cout << "  ActionType: " << action.actionType
                      << ", ActionID: " << action.actionId
                      << ", Blocking: " << action.blockingType;
            if (!action.actionDescription.empty())
                std::cout << ", Description: " << action.actionDescription;
            std::cout << std::endl;
        }
    }
}

void MissionManager::handleInstantActions(const nlohmann::json& payload)
{
    try {
        Header header;
        header.headerId = payload.at("headerId").get<int>();
        header.timestamp = payload.at("timestamp").get<std::string>();
        header.version = payload.at("version").get<std::string>();
        header.manufacturer = payload.at("manufacturer").get<std::string>();
        header.serialNumber = payload.at("serialNumber").get<std::string>();

        std::cout << "[mission_manager] Header:\n"
                  << "  headerId: " << header.headerId << "\n"
                  << "  timestamp: " << header.timestamp << "\n"
                  << "  version: " << header.version << "\n"
                  << "  manufacturer: " << header.manufacturer << "\n"
                  << "  serialNumber: " << header.serialNumber << std::endl;

        std::vector<Action> actions;
        for (const auto& a : payload.at("actions")) {
            Action act;
            act.actionId = a.at("actionId").get<std::string>();
            act.type = a.at("type").get<std::string>();
            act.parameters = a.value("parameters", nlohmann::json::object());
            actions.push_back(act);

            std::cout << "  Action:\n"
                      << "    actionId: " << act.actionId << "\n"
                      << "    type: " << act.type << "\n"
                      << "    parameters: " << act.parameters.dump() << std::endl;
        }

        std_msgs::msg::String msg;
        msg.data = payload.dump();
        instant_actions_pub_->publish(msg);

    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[mission_manager] JSON parsing error: " << e.what() << std::endl;
    }
}

#if 0 // roboot -> fms
void MissionManager::handleConnection(const nlohmann::json& payload)
{
    std::cout << "[mission_manager] Handling connection:\n" 
              << payload.dump(4) << std::endl;

    std_msgs::msg::String msg; 
    msg.data = payload.dump();
    connection_pub_->publish(msg);

    // connectionState 확인
    if (payload.contains("connectionState")) {
        std::string state = payload["connectionState"];

        if (state == "OFFLINE" || state == "CONNECTIONBROKEN") {
            std::cout << "[mission_manager] Connection lost: " << state << std::endl;
            if (disconnect_cb_) {
                disconnect_cb_();
            }
        }
    }
}
#endif

// FOR_TEST 핸들러
void MissionManager::handleMission(const nlohmann::json& payload)
{
    std_msgs::msg::String msg; msg.data = payload.dump();
    mission_pub_->publish(msg);
}

void MissionManager::handleControl(const nlohmann::json& payload)
{
    std_msgs::msg::String msg; msg.data = payload.dump();
    control_pub_->publish(msg);
}

void MissionManager::handleHeartbeat(const nlohmann::json& payload)
{
    std_msgs::msg::String msg; msg.data = payload.dump();
    heartbeat_pub_->publish(msg);
}
