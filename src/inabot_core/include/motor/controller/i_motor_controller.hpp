#pragma once

#include <memory>

namespace Inabot {

class IMotorController {
public:
    virtual ~IMotorController() = default;

    // 초기화 수행
    virtual bool initialize() = 0;
};

} // namespace Inabot

