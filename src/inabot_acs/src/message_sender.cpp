#include "inabot_acs/topics.hpp"
#include "error_codes.hpp"
#include "state_data.hpp"
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
    if (mqtt_topic == inabot_acs::TOPIC_STATE) {
        auto sub = node_->create_subscription<std_msgs::msg::String>(
            ros_topic, 10,
            [this](const std_msgs::msg::String::SharedPtr msg){
                last_state_ = msg->data;  // 최신 state 저장
                std::cout << "[MessageSender] Updated STATE from ROS: " << last_state_ << std::endl;
            }
        );
        subscriptions_.push_back({sub, mqtt_topic});

        // state 전용 주기 타이머 시작 (예: 500ms)
        state_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { publishState(); }
        );
    }
    else {
        // 다른 토픽은 기존 로직 그대로
        auto sub = node_->create_subscription<std_msgs::msg::String>(
            ros_topic, 10,
            [this, mqtt_topic](const std_msgs::msg::String::SharedPtr msg){
                rosCallback(mqtt_topic, msg);
            }
        );
        subscriptions_.push_back({sub, mqtt_topic});
    }

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
        // state는 addTopic에서 처리 → 여기선 안보냄
        return;
    } else {
        payload = msg->data;
    }

    mqtt_client_->publish(mqtt_topic, payload);
    std::cout << "[MessageSender] Sent to MQTT: " << mqtt_topic
              << " -> " << payload << std::endl;
}

void MessageSender::publishState()
{
    // 구조체 초기화
    StateData stateData;

    stateData.state = last_state_.empty() ? "NONE" : last_state_;
    stateData.newBaseRequest = false;
    stateData.distanceSinceLastNode = 0.0;
    stateData.operatingMode = (stateData.state == "IDLE") ? "MANUAL" : "AUTOMATIC";
    stateData.QR = "";

    // 상태별 데이터 채우기
    if (stateData.state == "MOVING") {
        stateData.driving = true;
        stateData.paused = false;
        stateData.actionStates.clear();

        stateData.agvPosition = {true, 1.0, 0.0, 297.71, 6.97, -3.13, "", ""};
        stateData.velocity = {0.2, 0.0, 0.0};
        stateData.batteryState = {35.7, 50.8, false};
    } else if (stateData.state == "IDLE") {
        stateData.driving = false;
        stateData.paused = false;
        stateData.actionStates.clear();

        stateData.agvPosition = {true, 1.0, 0.0, 265.19, 2.88, -0.0076, "", ""};
        stateData.velocity = {0.0, 0.0, 0.0};
        stateData.batteryState = {65.6, 52.3, false};
    } else if (stateData.state == "CHARGING") {
        stateData.driving = false;
        stateData.paused = false;
        stateData.actionStates = { {"stopCharging", "stopCharging", "FINISHED"} };

        stateData.agvPosition = {true, 1.0, 0.0, 221.28, -44.38, -3.13, "", ""};
        stateData.velocity = {0.0, 0.0, 0.0};
        stateData.batteryState = {66.0, 53.7, true};
    } else if (stateData.state == "LOADING") {
        stateData.driving = true;
        stateData.paused = false;
        stateData.actionStates = {
            {"detect", "detectObject", "RUNNING"},
            {"pick", "pick", "WAITING"}
        };

        stateData.agvPosition = {true, 1.0, 0.0, 298.32, 5.38, -3.12, "", ""};
        stateData.velocity = {0.0, 0.0, 0.0};
        stateData.batteryState = {35.6, 50.8, false};
    } else {
        stateData.driving = false;
        stateData.paused = false;
        stateData.actionStates.clear();
        stateData.agvPosition.positionInitialized = false;
        stateData.velocity = {0.0, 0.0, 0.0};
        stateData.batteryState = {0.0, 0.0, false};
    }

    // 공통 필드 (예: node/edge, load, error 등)
    stateData.nodeStates.clear();
    stateData.edgeStates.clear();
    stateData.loadFactor = {0, 0, 0, 0};

    // -----------------------
    // 에러 코드 처리
    // -----------------------
    // 현재 발생한 에러 코드 목록 (실제 로직에서 결정)
    std::vector<std::string> activeErrorCodes = {"001", "1504"};  // 예시

    // 현재 UTC 시간
    std::string ts = getCurrentUtcTimeString();

    // ERROR_CODES → StateData::errors 변환
    populateErrors(stateData, activeErrorCodes, ts);

    stateData.information = { {"I001", "Routine check OK", ts} };
    stateData.safetyState = {"AUTOACK", false};

    // headerId 추가
    nlohmann::json j = stateData.toJson();
    j["headerId"] = header_counter_[inabot_acs::TOPIC_STATE]++;
    j["timestamp"] = ts;
    j["version"] = "2.0.0";
    j["manufacturer"] = manufacturer_;
    j["serialNumber"] = serial_number_;

    // MQTT 전송
    std::string payload = j.dump();
    mqtt_client_->publish(inabot_acs::TOPIC_STATE, payload);

    std::cout << "[MessageSender] Periodic STATE publish → " << payload << std::endl;
}
