#pragma once

#include "i_motor_controller.hpp"
#include "roboteq_motor_controller.hpp"
#include "motor/controller/dummy_motor_controller.hpp"
#include <memory>
#include <string>

namespace Inabot {

class MotorControllerFactory {
public:
    static std::shared_ptr<IMotorController> create(
        const std::string& type,
        std::shared_ptr</*CanInterface*/ICanInterface> can_driver);
};

} // namespace Inabot
