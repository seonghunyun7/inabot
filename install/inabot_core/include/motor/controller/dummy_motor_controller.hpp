#pragma once

#include "i_motor_controller.hpp"
//#include "can_interface.hpp"
#include "i_can_interface.hpp"
#include <memory>
#include <iostream>

namespace Inabot {

class DummyMotorController : public IMotorController {
public:
    explicit DummyMotorController(std::shared_ptr</*CanInterface*/ICanInterface> can_driver);

    bool initialize() override;

private:
    //std::shared_ptr<CanInterface> can_driver_;
    std::shared_ptr<ICanInterface> can_driver_;
};

} // namespace Inabot
