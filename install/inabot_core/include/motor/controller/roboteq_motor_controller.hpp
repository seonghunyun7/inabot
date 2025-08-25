#pragma once

#include "i_motor_controller.hpp"
//#include "can_interface.hpp"
#include "i_can_interface.hpp"
#include <memory>
#include <vector>
#include <utility>  // for std::pair

#include "motor_node_id.hpp"

namespace Inabot {

class RoboteqMotorController : public IMotorController {
public:
    //RoboteqMotorController(std::shared_ptr<CanInterface> can_driver);
    RoboteqMotorController(std::shared_ptr<ICanInterface> can_driver);

    bool initialize() override;

private:
    //std::shared_ptr<CanInterface> can_driver_;
    std::shared_ptr<ICanInterface> can_driver_;

    // ===== Roboteq 전용 함수들 =====
    std::vector<uint8_t> createSDOWriteCommand(uint16_t index, uint8_t subindex, uint32_t value);
    bool sendSDOToNode(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value);
    std::vector<uint8_t> createSDOReadCommand(uint16_t index, uint8_t subindex);
    bool mapPDO(uint8_t node_id, uint16_t pdo_comm_index, uint16_t pdo_map_index,
                const std::vector<std::pair<uint16_t, uint8_t>>& mappings);
};

} // namespace Inabot
