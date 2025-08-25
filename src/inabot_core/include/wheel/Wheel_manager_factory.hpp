// WheelManagerFactory.hpp
//Wheel_manager_factory.hpp
#pragma once
#include "wheel/i_wheel_manager.hpp"
#include <memory>

enum class WheelType {
    DIFFERENTIAL,
    STEERABLE
};

class WheelManagerFactory {
public:
    static std::unique_ptr<IWheelManager> createWheelManager(WheelType type);
};
