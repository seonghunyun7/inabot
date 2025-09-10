#pragma once
#include "mqtt/async_client.h"
#include "message_handler.hpp"
#include <memory>
#include <string>

struct BrokerConfig {
    std::string host;
    int port;
    struct {
        bool enabled = false;
        std::string ca_cert;
        std::string client_cert;
        std::string client_key;
    } tls;
    std::string user; // optional
    std::string pass; // optional
};

struct ClientConfig {
    std::string id;
    bool clean_session = true;
    int keep_alive_interval = 60;
    int max_inflight = 65535;
};

class MqttClient : public virtual mqtt::callback,
                   public virtual mqtt::iaction_listener {
public:
    MqttClient(const BrokerConfig& broker_cfg, const ClientConfig& client_cfg,
               std::shared_ptr<MessageHandler> handler);
    ~MqttClient();

    void disconnect();
    void publish(const std::string& topic, const std::string& payload);


private:
    std::shared_ptr<mqtt::async_client> client_;
    mqtt::connect_options conn_opts_;
    std::shared_ptr<MessageHandler> msg_handler_;

    // mqtt::callback overrides
    void message_arrived(mqtt::const_message_ptr msg) override;
    void delivery_complete(mqtt::delivery_token_ptr token) override;
    void connected(const std::string& cause) override;
    void connection_lost(const std::string& cause) override;

    // mqtt::iaction_listener overrides
    void on_success(const mqtt::token& tok) override;
    void on_failure(const mqtt::token& tok) override;
};
