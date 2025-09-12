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

        std::string state = msg->data.empty() ? "NONE" : msg->data;

        // 기본 필드
        j["newBaseRequest"] = false;
        j["distanceSinceLastNode"] = 0;
        j["operatingMode"] = (state == "IDLE") ? "MANUAL" : "AUTOMATIC";
        j["QR"] = "";

        // 상태별 driving, paused, actionStates, batteryState 등
        if (state == "MOVING") {
            j["driving"] = true;
            j["paused"] = false;
            j["actionStates"] = json::array(); // 이동 중 특별 액션 없음

            j["agvPosition"] = {
                {"positionInitialized", true},
                {"localizationScore", 1},
                {"deviationRange", 0},
                {"x", 297.713073730469},
                {"y", 6.97085332870483},
                {"theta", -3.13170170783997},
                {"mapId", ""},
                {"mapDescription", ""}
            };

            j["velocity"] = { {"vx", -0.0}, {"vy", -0.0}, {"omega", 0.0} };

            j["batteryState"] = {
                {"batteryCharge", 35.7},
                {"batteryVoltage", 50.8},
                {"charging", false}
            };

        } else if (state == "IDLE") {
            j["driving"] = false;
            j["paused"] = false;
            j["actionStates"] = json::array();

            j["agvPosition"] = {
                {"positionInitialized", true},
                {"localizationScore", 1},
                {"deviationRange", 0},
                {"x", 265.194396972656},
                {"y", 2.88258290290833},
                {"theta", -0.0076024578884244},
                {"mapId", ""},
                {"mapDescription", ""}
            };

            j["velocity"] = { {"vx", 0.489341884851456}, {"vy", -0.848814129829407}, {"omega", 0.0} };

            j["batteryState"] = {
                {"batteryCharge", 65.6},
                {"batteryVoltage", 52.3},
                {"charging", false}
            };

        } else if (state == "CHARGING") {
            j["driving"] = false;
            j["paused"] = false;
            j["actionStates"] = {
                {{"actionId", "stopCharging"},
                {"actionType", "stopCharging"},
                {"actionStatus", "FINISHED"}}
            };

            j["agvPosition"] = {
                {"positionInitialized", true},
                {"localizationScore", 1},
                {"deviationRange", 0},
                {"x", 221.289108276367},
                {"y", -44.3882446289062},
                {"theta", -3.13039755821228},
                {"mapId", ""},
                {"mapDescription", ""}
            };

            j["velocity"] = { {"vx", 0.0}, {"vy", 0.0}, {"omega", 0.0} };

            j["batteryState"] = {
                {"batteryCharge", 66.0},
                {"batteryVoltage", 53.7},
                {"charging", true}
            };

        } else if (state == "LOADING") {
            j["driving"] = true;
            j["paused"] = false;
            j["actionStates"] = {
                {{"actionId", "detect"}, {"actionType", "detectObject"}, {"actionStatus", "RUNNING"}},
                {{"actionId", "pick"}, {"actionType", "pick"}, {"actionStatus", "WAITING"}}
            };

            j["agvPosition"] = {
                {"positionInitialized", true},
                {"localizationScore", 1},
                {"deviationRange", 0},
                {"x", 298.326721191406},
                {"y", 5.38703918457031},
                {"theta", -3.12417340278625},
                {"mapId", ""},
                {"mapDescription", ""}
            };

            j["velocity"] = { {"vx", -0.00089868635404855}, {"vy", 0.0021780279930681}, {"omega", 0.0} };

            j["batteryState"] = {
                {"batteryCharge", 35.6},
                {"batteryVoltage", 50.8},
                {"charging", false}
            };
        }

        // 공통 필드
        j["nodeStates"] = json::array();
        j["edgeStates"] = json::array();
        j["LoadFactor"] = { {"Front_Traction", 0}, {"Front_Steer", 0}, {"Rear_Traction", 0}, {"Rear_Steer", 0} };
        j["errors"] = json::array();
        j["information"] = json::array();
        j["safetyState"] = { {"eStop", "AUTOACK"}, {"fieldViolation", false} };

        payload = j.dump();
    } else {
        payload = msg->data;
    }

    mqtt_client_->publish(mqtt_topic, payload);
    std::cout << "[MessageSender] Sent to MQTT: " << mqtt_topic
              << " -> " << payload << std::endl;
}

