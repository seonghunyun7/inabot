//error_codes.hpp
#pragma once
#include <string>
#include <unordered_map>

// 에러 정보 구조체 (ERROR_CODES 기준)
struct ErrorCodeInfo {
    std::string errorCode;
    std::string description;
    std::string level;  // CRITICAL, WARNING, INFO 등
};

// 실제 로봇에서 사용하는 알람 코드 매핑
static const std::unordered_map<std::string, ErrorCodeInfo> ERROR_CODES = {
    {"001", {"001", "Derating by electronics temperature", "WARNING"}},
    {"002", {"002", "Derating by coil temperature", "WARNING"}},
    {"003", {"003", "Derating by rectifier temperature", "WARNING"}},
    {"1501", {"1501", "PLC Start Alarm", "CRITICAL"}},
    {"1502", {"1502", "Emergency Switch", "CRITICAL"}},
    {"1503", {"1503", "Bump Stop", "CRITICAL"}},
    {"1504", {"1504", "Front 2D Obstacle detected", "CRITICAL"}},
    {"1505", {"1505", "Rear 2D Obstacle detected", "CRITICAL"}},
    {"1506", {"1506", "Front Left 3D Obstacle detected", "CRITICAL"}},
    {"1507", {"1507", "Front Right 3D Obstacle detected", "CRITICAL"}},
    {"1508", {"1508", "Grating detected", "WARNING"}},
    {"1509", {"1509", "Fan Input Error", "WARNING"}},
    {"1510", {"1510", "Fan Output Error", "WARNING"}},
    {"2001", {"2001", "Drive1 Communication Error", "CRITICAL"}},
    {"2002", {"2002", "Drive2 Communication Error", "CRITICAL"}}
};
