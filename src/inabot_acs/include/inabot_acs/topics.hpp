// include/inabot_acs/topics.hpp
#pragma once
#include <string>

namespace inabot_acs {

    // ========================================================
    // VDA5050 MQTT Topic 규칙
    // ========================================================
    // 구조: interfaceName/majorVersion/manufacturer/serialNumber/topic
    // 예시: uagv/v2/Inatech/P3LDD02/order
    // 
    // - interfaceName       : 사용되는 인터페이스 이름 (uagv)
    // - majorVersion        : VDA5050 주요 버전 (v2)
    // - manufacturer        : AGV 제조사 (Inatech)
    // - serialNumber        : AGV 고유 일련번호 (P3LDD02)
    // - topic               : order, state, instantActions 등
    //
    // 규칙:
    // 1) 토픽 이름은 camelCase
    // 2) 열거형 값은 UPPERCASE
    // 3) 선택적(optional) 필드는 italic 처리 (문서 참고:VD5050)
    // ========================================================

    // 테스트용 토픽
    #if _FOR_TEST
    inline const std::string TOPIC_HEARTBEAT = "heartbeat";
    inline const std::string TOPIC_MISSION   = "mission";
    inline const std::string TOPIC_CONTROL   = "control";
    #endif

    // VD5050D 프로토콜 토픽
    // FMS -> ROBOT
    inline const std::string TOPIC_ORDER           = "uagv/v2/Inatech/P3LDD02/order";           // master -> AGV
    inline const std::string TOPIC_INSTANT_ACTIONS = "uagv/v2/Inatech/P3LDD02/instantActions"; // master -> AGV

    // ROBOT -> FMS
    inline const std::string TOPIC_STATE           = "uagv/v2/Inatech/P3LDD02/state";           // AGV -> master
    inline const std::string TOPIC_VISUALIZATION   = "uagv/v2/Inatech/P3LDD02/visualization";   // optional, AGV -> master
    inline const std::string TOPIC_CONNECTION      = "uagv/v2/Inatech/P3LDD02/connection";      // Broker/AGV -> master
    inline const std::string TOPIC_FACTSHEET       = "uagv/v2/Inatech/P3LDD02/factsheet";       // AGV -> master
}
