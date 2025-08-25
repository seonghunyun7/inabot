// Wheel_manager_factory.cpp
//#include "wheel/WheelManagerFactory.hpp"
#include "wheel/Wheel_manager_factory.hpp"
#include "wheel/differential_wheel_manager.hpp"
#include "wheel/steerable_wheel_manager.hpp"

std::unique_ptr<IWheelManager> WheelManagerFactory::createWheelManager(WheelType type) {
    switch(type) {
        case WheelType::DIFFERENTIAL:
            return std::make_unique<DifferentialWheelManager>();
        case WheelType::STEERABLE:
            return std::make_unique<SteerableWheelManager>();
        default:
            return nullptr;
    }
}
