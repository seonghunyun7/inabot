#include "inabot_acs/topics.hpp"
#include "message_sender.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

using json = nlohmann::json;

// UTC ISO8601 문자열 생성
static std::string getCurrentUtcTimeString()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;

    std::tm tm_utc;
    gmtime_r(&t, &tm_utc);  // thread-safe UTC 변환

    std::ostringstream oss;
    oss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms.count() << "Z";

    return oss.str();
}

MessageSender::MessageSender(std::shared_ptr<rclcpp::Node> node,
                             std::shared_ptr<MqttClient> mqtt_client,
                             const std::string& manufacturer,
                             const std::string& serial_number)
: node_(node), mqtt_client_(mqtt_client),
  manufacturer_(manufacturer), serial_number_(serial_number)
{
    std::cout << "[MessageSender] Initialized with manufacturer=" 
              << manufacturer_ << ", serial_number=" << serial_number_ << std::endl;
}

void MessageSender::addTopic(const std::string& ros_topic, const std::string& mqtt_topic)
{
    auto sub = node_->create_subscription<std_msgs::msg::String>(
        ros_topic, 10,
        [this, mqtt_topic](const std_msgs::msg::String::SharedPtr msg){
            rosCallback(mqtt_topic, msg);
        }
    );

    subscriptions_.push_back({sub, mqtt_topic});
    std::cout << "[MessageSender] Subscribed ROS topic: " << ros_topic
              << " → MQTT topic: " << mqtt_topic << std::endl;
}

//ros_topic을 정의를 해야 한다.(아마 커스텀으로 만들어야 할 듯)
void MessageSender::rosCallback(const std::string& mqtt_topic,
                                const std_msgs::msg::String::SharedPtr msg)
{
    if (!mqtt_client_) return;

    std::string payload;

    if (mqtt_topic == inabot_acs::TOPIC_CONNECTION) {
        // connection 토픽: VDA5050 JSON 스키마
        json j;
        j["headerId"] = header_counter_[mqtt_topic]++;
        j["timestamp"] = getCurrentUtcTimeString();
        j["version"] = "2.0.0";
        j["manufacturer"] = manufacturer_;
        j["serialNumber"] = serial_number_;
        std::string state = msg->data.empty() ? "ONLINE" : msg->data;
        j["connection"] = state;
        payload = j.dump();

    } else if (mqtt_topic == inabot_acs::TOPIC_FACTSHEET) {
        // factsheet 토픽: VDA5050 JSON 구조 생성
        json j;
        j["headerId"] = header_counter_[mqtt_topic]++;
        j["timestamp"] = getCurrentUtcTimeString();
        j["version"] = "2.0.0";
        j["manufacturer"] = manufacturer_;
        j["serialNumber"] = serial_number_;

        // ROS 메시지를 기반으로 팩트시트 세부 정보를 넣는다
        // 여기서는 예제용, 실제 메시지 내용에 따라 변환 필요
        json factsheet;
        factsheet["typeSpecification"] = {
            {"seriesName", "P3 Series"},
            {"seriesDescription", "Differential drive AGV"},
            {"agvKinematic", "DIFF"},
            {"agvClass", "FORKLIFT"},
            {"maxLoadMass", 1500.0},
            {"localizationTypes", {"NATURAL", "REFLECTOR"}},
            {"navigationTypes", {"PHYSICAL_LINE_GUIDED", "AUTONOMOUS"}}
        };

        factsheet["physicalParameters"] = {
            {"speedMin", 0.1},
            {"speedMax", 1.5},
            {"angularSpeedMin", 0.0},
            {"angularSpeedMax", 1.0},
            {"accelerationMax", 0.5},
            {"decelerationMax", 0.5},
            {"heightMin", 0.5},
            {"heightMax", 1.2},
            {"width", 0.8},
            {"length", 1.2}
        };

        factsheet["protocolLimits"] = {
            {"maxStringLens", {{"msgLen", 1024}, {"serialNumber", 32}}},
            {"maxArrayLens", {{"orderNodes", 50}, {"orderEdges", 50}}}
        };

        factsheet["protocolFeatures"] = {
            {"optionalParameters", json::array()}
        };

        factsheet["agvGeometry"] = {
            {"wheelDefinitions", json::array()}
        };

        factsheet["loadSpecification"] = {
            {"loadPositions", json::array()},
            {"loadSets", json::array()}
        };

        factsheet["vehicleConfig"] = {
            {"versions", json::array()},
            {"network", {{"dnsServers", json::array()}, {"ntpServers", json::array()}, {"localIpAddress", ""}, {"netmask", ""}, {"defaultGateway", ""}}}
        };

        j["factsheet"] = factsheet;

        payload = j.dump();

    } else if (mqtt_topic == inabot_acs::TOPIC_STATE) {
        // state 토픽: VDA5050 state JSON 생성
        //AGV 위치, 속도, 적재물 등
        json j;
        j["headerId"] = header_counter_[mqtt_topic]++;
        j["timestamp"] = getCurrentUtcTimeString();
        j["version"] = "2.0.0";
        j["manufacturer"] = manufacturer_;
        j["serialNumber"] = serial_number_;

        // AGV 위치/속도/적재/동작 상태 예제
        j["agvPosition"] = {
            {"positionInitialized", true},
            {"x", 12.34},
            {"y", 56.78},
            {"theta", 1.57},
            {"mapId", "map01"},
            {"mapDescription", "Main floor"}
        };

        j["velocity"] = {
            {"vx", 0.5},
            {"vy", 0.0},
            {"omega", 0.1}
        };

        j["loads"] = json::array({
            {{"loadId", "LOAD001"}, {"loadType", "PALLET"}, {"loadPosition", "front"},
            {"boundingBoxReference", {{"x",0.0},{"y",0.0},{"z",0.0},{"theta",0.0}}},
            {"loadDimensions", {{"length",1.2},{"width",0.8},{"height",0.5},{"weight",300.0}}}}
        });

        j["driving"] = true;
        j["paused"] = false;
        j["newBaseRequest"] = false;
        j["distanceSinceLastNode"] = 2.5;

        j["nodeStates"] = json::array();
        j["edgeStates"] = json::array();
        j["actionStates"] = json::array();
        j["operatingMode"] = "AUTOMATIC";
        j["errors"] = json::array();
        j["information"] = json::array();
        j["safetyState"] = {{"emergencyStopAcknowledged", "AUTOACK"}, {"protectiveFieldViolated", false}};
        j["batteryState"] = {{"batteryCharge", 85.0}, {"batteryVoltage", 24.5}, {"batteryHealth", 80}, {"charging", false}, {"reach", 10}};

        payload = j.dump();

    } else {
        payload = msg->data;
    }

    mqtt_client_->publish(mqtt_topic, payload);
    std::cout << "[MessageSender] Sent to MQTT: " << mqtt_topic
              << " -> " << payload << std::endl;
}

