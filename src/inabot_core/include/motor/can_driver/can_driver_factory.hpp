#pragma once
#include <memory>
#include <string>
#include "i_can_interface.hpp"

namespace Inabot {

class CanDriverFactory {
public:
    static std::shared_ptr<ICanInterface> create(const std::string& type);
};

} // namespace Inabot
