#include "inabot_acs/topics.hpp"
#include "mqtt_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fmt/core.h>
#include <vector>

MqttClient::MqttClient(const BrokerConfig& broker_cfg,
                       const ClientConfig& client_cfg,
                       std::shared_ptr<MessageHandler> handler)
: msg_handler_(handler)
{
    std::cout << "[DEBUG] Broker Host: " << broker_cfg.host << std::endl;
    std::cout << "[DEBUG] Broker Port: " << broker_cfg.port << std::endl;
    std::cout << "[DEBUG] TLS Enabled: " << (broker_cfg.tls.enabled ? "true" : "false") << std::endl;
    std::cout << "[DEBUG] User: " << (broker_cfg.user.empty() ? "<empty>" : broker_cfg.user) << std::endl;
    std::cout << "[DEBUG] Client ID: " << client_cfg.id << std::endl;
    std::cout << "[DEBUG] Clean Session: " << (client_cfg.clean_session ? "true" : "false") << std::endl;
    std::cout << "[DEBUG] Keep Alive: " << client_cfg.keep_alive_interval << std::endl;
    std::cout << "[DEBUG] Max Inflight: " << client_cfg.max_inflight << std::endl;

    // URI 생성
    const std::string protocol = broker_cfg.tls.enabled ? "ssl" : "tcp";
    const std::string uri = fmt::format("{}://{}:{}", protocol, broker_cfg.host, broker_cfg.port);

    client_ = std::make_shared<mqtt::async_client>(uri, client_cfg.id);

    // 기본 옵션
    conn_opts_.set_clean_session(client_cfg.clean_session);
    conn_opts_.set_keep_alive_interval(client_cfg.keep_alive_interval);
    conn_opts_.set_max_inflight(client_cfg.max_inflight);
    conn_opts_.set_automatic_reconnect(true);

    // LWT 설정
    if (client_cfg.lwt_enabled) {
        mqtt::message will_msg(
            client_cfg.lwt_topic,     // 설정한 토픽
            client_cfg.lwt_payload,   // 설정한 페이로드
            client_cfg.lwt_qos,       // 설정한 QoS
            client_cfg.lwt_retain     // 설정한 Retain
        );
        conn_opts_.set_will(will_msg);
    }
    
    // TLS/SSL 설정
    if (broker_cfg.tls.enabled) {
        mqtt::ssl_options ssl_opts;
        ssl_opts.set_trust_store(broker_cfg.tls.ca_cert);
        if (!broker_cfg.tls.client_cert.empty() && !broker_cfg.tls.client_key.empty()) {
            ssl_opts.set_key_store(broker_cfg.tls.client_cert);
            ssl_opts.set_private_key(broker_cfg.tls.client_key);
        }
        conn_opts_.set_ssl(ssl_opts);
    }

    // 사용자 인증 (선택)
    if (!broker_cfg.user.empty()) {
        conn_opts_.set_user_name(broker_cfg.user);
        conn_opts_.set_password(broker_cfg.pass);
    }

    client_->set_callback(*this);

    try {
        // connect 완료 후 wait()로 연결 대기
        client_->connect(conn_opts_, nullptr, *this)->wait();
        std::cout << "[INFO] Connected to broker at " << uri << std::endl;

        // 구독할 토픽 목록 (FMS -> ROBOT)
        std::vector<std::string> topics = {
            //for_test
            #if _FOR_TEST
            inabot_acs::TOPIC_HEARTBEAT,
            inabot_acs::TOPIC_MISSION,
            inabot_acs::TOPIC_CONTROL,
            #endif    
            //vd5050d
            inabot_acs::TOPIC_ORDER,
            inabot_acs::TOPIC_INSTANT_ACTIONS,
        };

        for (const auto& t : topics) {
            client_->subscribe(t, 1)->wait();
            std::cout << "[INFO] Subscribed to topic: " << t << std::endl;
        }

        std::cout << "[INFO] Subscribed to topics: heartbeat, mission, control" << std::endl;

    } catch (const mqtt::exception& e) {
        std::cerr << "[ERROR] MQTT connect/subscribe error: " << e.what() << std::endl;
    }
}

MqttClient::~MqttClient()
{
    try { if (client_ && client_->is_connected()) client_->disconnect(); } catch (...) {}
}

void MqttClient::disconnect()
{
    try {
        if (client_ && client_->is_connected()) {
            client_->disconnect()->wait_for(std::chrono::seconds(3));
            std::cout << "[INFO] Disconnected from broker" << std::endl;
        }
    } catch (const mqtt::exception& e) {
        std::cerr << "[ERROR] Error during disconnect: " << e.what() << std::endl;
    }
}

void MqttClient::publish(const std::string& topic, const std::string& payload)
{
    try {
        client_->publish(topic, payload.c_str(), payload.size(), 1, false);
        std::cout << "[INFO] Published to topic " << topic << ": " << payload << std::endl;
    } catch (const mqtt::exception& e) {
        std::cerr << "[ERROR] Publish failed: " << e.what() << std::endl;
    }
}

// mqtt::callback
void MqttClient::message_arrived(mqtt::const_message_ptr msg)
{
    if (msg_handler_) msg_handler_->handleMessage(msg->get_topic(), msg->to_string());
}

void MqttClient::delivery_complete(mqtt::delivery_token_ptr) 
{
    //
}

void MqttClient::connected(const std::string&) 
{
    std::cout << "[INFO] Connected to broker" << std::endl;
}

void MqttClient::connection_lost(const std::string& cause) 
{
    std::cerr << "[WARN] Connection lost: " << cause << std::endl;
}

void MqttClient::on_success(const mqtt::token&) 
{
    std::cout << "[INFO] Action succeeded" << std::endl;
}

void MqttClient::on_failure(const mqtt::token& tok) 
{
    std::cerr << "[WARN] MQTT connect failed (code " << tok.get_return_code() << ")" << std::endl;
}
