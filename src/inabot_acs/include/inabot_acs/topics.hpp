// include/inabot_acs/vdtopics.hpp
#pragma once
#include <string>

namespace inabot_acs {

    // 기존 토픽 (for test)
    inline const std::string TOPIC_HEARTBEAT = "heartbeat";
    inline const std::string TOPIC_MISSION   = "mission";
    inline const std::string TOPIC_CONTROL   = "control";

    // VD5050D 프로토콜 토픽
    inline const std::string TOPIC_ORDER             = "order";            // VD5050D
    inline const std::string TOPIC_INSTANT_ACTIONS   = "instantActions";   // VD5050D
    inline const std::string TOPIC_STATE             = "state";            // VD5050D
    inline const std::string TOPIC_VISUALIZATION     = "visualization";    // VD5050D
    inline const std::string TOPIC_CONNECTION        = "connection";       // VD5050D
    inline const std::string TOPIC_FACTSHEET         = "factsheet";        // VD5050D
}
