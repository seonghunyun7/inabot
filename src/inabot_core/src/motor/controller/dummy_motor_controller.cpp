#include "motor/controller/dummy_motor_controller.hpp"
#include <utils/logger.h>
#include <iostream>  // 추가

namespace Inabot {

DummyMotorController::DummyMotorController(std::shared_ptr</*CanInterface*/ICanInterface> can_driver)
: can_driver_(can_driver)
{
      std::cout << "[DummyMotorController] Created" << std::endl;
}

bool DummyMotorController::initialize()
{
    std::cout << "[DummyMotorController] initialize() called" << std::endl;
    // 실제 초기화 코드 없고, 더미라서 true 리턴
    return true;
}

} // namespace Inabot
