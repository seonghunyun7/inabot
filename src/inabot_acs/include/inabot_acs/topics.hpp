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
    constexpr char MANUFACTURER[] = "RobotCompany";  // 각 제조사별로 변경 가능
    constexpr char SERIAL_NUMBER[] = "Robot001"; //각 로봇별로 변경 가능

    constexpr char INTERFACE_NAME[] = "uagv";
    constexpr char VDA_VERSION[] = "v2.0.0";
    constexpr char VDA_FULL_VERSION[] = "2.0.0";

    // 테스트용 토픽
    #if _FOR_TEST
    inline const std::string TOPIC_HEARTBEAT = "heartbeat";
    inline const std::string TOPIC_MISSION   = "mission";
    inline const std::string TOPIC_CONTROL   = "control";
    #endif

   // VD5050D 프로토콜 토픽 (FMS -> ROBOT)
    inline const std::string TOPIC_ORDER =
        std::string(INTERFACE_NAME) + "/" + VDA_VERSION + "/" + MANUFACTURER + "/" + SERIAL_NUMBER + "/order";
    inline const std::string TOPIC_INSTANT_ACTIONS =
        std::string(INTERFACE_NAME) + "/" + VDA_VERSION + "/" + MANUFACTURER + "/" + SERIAL_NUMBER + "/instantActions";

    // ROBOT -> FMS
    inline const std::string TOPIC_STATE =
        std::string(INTERFACE_NAME) + "/" + VDA_VERSION + "/" + MANUFACTURER + "/" + SERIAL_NUMBER + "/state";
    inline const std::string TOPIC_CONNECTION =
        std::string(INTERFACE_NAME) + "/" + VDA_VERSION + "/" + MANUFACTURER + "/" + SERIAL_NUMBER + "/connection";
    inline const std::string TOPIC_VISUALIZATION =
        std::string(INTERFACE_NAME) + "/" + VDA_VERSION + "/" + MANUFACTURER + "/" + SERIAL_NUMBER + "/visualization";
    inline const std::string TOPIC_FACTSHEET =
        std::string(INTERFACE_NAME) + "/" + VDA_VERSION + "/" + MANUFACTURER + "/" + SERIAL_NUMBER + "/factsheet";
}
