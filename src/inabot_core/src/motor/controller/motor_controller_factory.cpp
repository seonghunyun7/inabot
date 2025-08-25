#include "motor/controller/motor_controller_factory.hpp"
#include <iostream>  // 추가

namespace Inabot {

std::shared_ptr<IMotorController> MotorControllerFactory::create(
    const std::string& type,
    std::shared_ptr</*CanInterface*/ICanInterface> can_driver)
{
    if (type == "roboteq") {
        std::cout << "[MotorControllerFactory] Created RoboteqMotorController" << std::endl;
        return std::make_shared<RoboteqMotorController>(can_driver);
    }
    else if (type == "dummy") {
        std::cout << "[MotorControllerFactory] Created DummyMotorController" << std::endl;
        return std::make_shared<DummyMotorController>(can_driver);
    }
    std::cerr << "[MotorControllerFactory] Unknown motor controller type: " << type << std::endl;
    return nullptr;
}

} // namespace Inabot
