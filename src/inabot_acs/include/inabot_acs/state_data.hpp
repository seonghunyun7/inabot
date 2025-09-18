/// state_data.hpp
#pragma once
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "error_codes.hpp"  // ERROR_CODES 포함

// -----------------------------
// AGV Position
// -----------------------------
struct AGVPosition {
    bool positionInitialized = false;
    double localizationScore = 0.0;
    double deviationRange = 0.0;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    std::string mapId;
    std::string mapDescription;

    nlohmann::json toJson() const {
        return {
            {"positionInitialized", positionInitialized},
            {"localizationScore", localizationScore},
            {"deviationRange", deviationRange},
            {"x", x},
            {"y", y},
            {"theta", theta},
            {"mapId", mapId},
            {"mapDescription", mapDescription}
        };
    }
};

// -----------------------------
// Velocity
// -----------------------------
struct Velocity {
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;

    nlohmann::json toJson() const {
        return {{"vx", vx}, {"vy", vy}, {"omega", omega}};
    }
};

// -----------------------------
// Battery
// -----------------------------
struct BatteryState {
    double batteryCharge = 0.0;
    double batteryVoltage = 0.0;
    bool charging = false;

    nlohmann::json toJson() const {
        return {{"batteryCharge", batteryCharge},
                {"batteryVoltage", batteryVoltage},
                {"charging", charging}};
    }
};

// -----------------------------
// Action State
// -----------------------------
struct ActionState {
    std::string actionId;
    std::string actionType;
    std::string actionStatus;

    nlohmann::json toJson() const {
        return {{"actionId", actionId},
                {"actionType", actionType},
                {"actionStatus", actionStatus}};
    }
};

// -----------------------------
// Error / Info
// -----------------------------
struct ErrorInfo {
    std::string errorCode;
    std::string description;
    std::string errorLevel;
    std::string timestamp;

    nlohmann::json toJson() const {
        return {{"errorCode", errorCode},
                {"description", description},
                {"errorLevel", errorLevel},
                {"timestamp", timestamp}};
    }
};

struct Info {
    std::string infoCode;
    std::string description;
    std::string timestamp;

    nlohmann::json toJson() const {
        return {{"infoCode", infoCode},
                {"description", description},
                {"timestamp", timestamp}};
    }
};

// -----------------------------
// Safety State
// -----------------------------
struct SafetyState {
    std::string eStop;
    bool fieldViolation;

    nlohmann::json toJson() const {
        return {{"eStop", eStop},
                {"fieldViolation", fieldViolation}};
    }
};

// -----------------------------
// Load Factor
// -----------------------------
struct LoadFactor {
    int Front_Traction = 0;
    int Front_Steer = 0;
    int Rear_Traction = 0;
    int Rear_Steer = 0;

    nlohmann::json toJson() const {
        return {{"Front_Traction", Front_Traction},
                {"Front_Steer", Front_Steer},
                {"Rear_Traction", Rear_Traction},
                {"Rear_Steer", Rear_Steer}};
    }
};

// -----------------------------
// 전체 State
// -----------------------------
struct StateData {
    std::string state;
    bool newBaseRequest = false;
    double distanceSinceLastNode = 0.0;
    std::string operatingMode;
    std::string QR;

    bool driving = false;
    bool paused = false;
    AGVPosition agvPosition;
    Velocity velocity;
    BatteryState batteryState;
    std::vector<ActionState> actionStates;

    std::vector<nlohmann::json> nodeStates;
    std::vector<nlohmann::json> edgeStates;
    LoadFactor loadFactor;
    std::vector<ErrorInfo> errors;
    std::vector<Info> information;
    SafetyState safetyState;

    nlohmann::json toJson() const {
        nlohmann::json j;
        j["state"] = state;
        j["newBaseRequest"] = newBaseRequest;
        j["distanceSinceLastNode"] = distanceSinceLastNode;
        j["operatingMode"] = operatingMode;
        j["QR"] = QR;

        j["driving"] = driving;
        j["paused"] = paused;
        j["agvPosition"] = agvPosition.toJson();
        j["velocity"] = velocity.toJson();
        j["batteryState"] = batteryState.toJson();

        j["actionStates"] = nlohmann::json::array();
        for (const auto& a : actionStates) j["actionStates"].push_back(a.toJson());

        j["nodeStates"] = nodeStates;
        j["edgeStates"] = edgeStates;
        j["LoadFactor"] = loadFactor.toJson();

        j["errors"] = nlohmann::json::array();
        for (const auto& e : errors) j["errors"].push_back(e.toJson());

        j["information"] = nlohmann::json::array();
        for (const auto& info : information) j["information"].push_back(info.toJson());

        j["safetyState"] = safetyState.toJson();
        return j;
    }
};

// -----------------------------
// ERROR_CODES → StateData::errors 변환 유틸
// -----------------------------
inline void populateErrors(StateData& stateData,
                           const std::vector<std::string>& activeErrorCodes,
                           const std::string& timestamp) {
    stateData.errors.clear();
    for (const auto& code : activeErrorCodes) {
        auto it = ERROR_CODES.find(code);
        if (it != ERROR_CODES.end()) {
            const auto& info = it->second;
            stateData.errors.push_back({info.errorCode, info.description, info.level, timestamp});
        } else {
            stateData.errors.push_back({code, "Unknown error code", "WARNING", timestamp});
        }
    }
}
