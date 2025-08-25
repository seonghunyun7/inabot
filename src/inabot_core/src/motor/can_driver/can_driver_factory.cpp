#include "motor/can_driver/can_driver_factory.hpp"
#include "motor/can_driver/socket_can_driver.hpp"
#include "motor/can_driver/dummy_can_driver.hpp"
#include "motor/can_driver/peak_can_driver.hpp" // 필요 시 사용

#include <iostream> // std::cout, std::endl

namespace Inabot {

std::shared_ptr<ICanInterface> CanDriverFactory::create(const std::string& type) {
    if (type == "socket") {
        std::cout << "[CanDriverFactory] Created SocketCanDriver" << std::endl;
        return std::make_shared<SocketCanDriver>();
    } else if (type == "peak") {
        std::cout << "[CanDriverFactory] Created PeakCanDriver" << std::endl;
        return std::make_shared<PeakCanDriver>();
    } else if (type == "dummy") {
        std::cout << "[CanDriverFactory] Created DummyCanDriver" << std::endl;
        return std::make_shared<DummyCanDriver>();
    } else {
        std::cerr << "[CanDriverFactory] Unknown CAN driver type: " << type << std::endl;
        return nullptr;
    }
}

} // namespace Inabot
