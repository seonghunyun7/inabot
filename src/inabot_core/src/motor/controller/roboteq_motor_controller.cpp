#include "motor/controller/roboteq_motor_controller.hpp"
#include <vector>
#include <iostream>  // 추가

using namespace Inabot;

namespace Inabot {

RoboteqMotorController::RoboteqMotorController(std::shared_ptr</*CanInterface*/ICanInterface> can_driver)
: can_driver_(can_driver)
{
    std::cout << "[RoboteqMotorController] Created" << std::endl;
}

bool RoboteqMotorController::initialize()
{
    std::cout << "[RoboteqMotorController] initialize() called" << std::endl;
#if __TEST_BUILD_CODE__
    for (NodeId node : Inabot::allNodeIds()) {
        uint8_t node_val = static_cast<uint8_t>(node);
        std::cout << "  > Initializing node ID: " << static_cast<int>(node_val) << std::endl;

        // 0. 장치 정보 읽기 (선택적)
        auto read_cmd = createSDOReadCommand(0x100A, 0x00); // Software version
        can_driver_->sendFrame(0x600 + node_val, read_cmd);
        std::cout << "    - Sent software version read command" << std::endl;

        // 1. 제어 모드 설정 (Velocity Mode = 3)
        if (!sendSDOToNode(node_val, 0x6060, 0x00, 3)) {
            std::cerr << "    [ERROR] Failed to set mode for node: " << static_cast<int>(node_val) << std::endl;
            return false;
        }

        // 2. 제어워드 초기화
        sendSDOToNode(node_val, 0x6040, 0x00, 0x0006); // 준비
        sendSDOToNode(node_val, 0x6040, 0x00, 0x000F); // 작동 활성화
        std::cout << "    - Sent control words (0x0006 -> 0x000F)" << std::endl;

        // 3. TPDO 매핑 (예: 위치, 속도)
        std::vector<std::pair<uint16_t, uint8_t>> tpdo_map = {
            {0x6064, 0x00},  // 위치
            {0x606C, 0x00}   // 속도
        };
        
        std::cout << "    - Mapping TPDO..." << std::endl;
        mapPDO(node_val, 0x1800, 0x1A00, tpdo_map);

        // 4. RPDO 매핑 (예: 목표 속도)
        std::vector<std::pair<uint16_t, uint8_t>> rpdo_map = {
            {0x60FF, 0x00}  // 목표 속도
        };
        
        std::cout << "    - Mapping RPDO..." << std::endl;
        mapPDO(node_val, 0x1400, 0x1600, rpdo_map);
    }
#endif

    //RCLCPP_INFO(rclcpp::get_logger("RoboteqMotorController"), "모든 Roboteq 노드 초기화 완료");
    return true;
}

//표준 SDO Download (Write) 프레임 구조
//
//함수 이름	목적	호출 위치
//createSDOReadCommand()	장치 정보 읽기 (SDO)	루프 내 시작 부분 (optional)
//sendSDOToNode()	SDO 명령 전송 (제어모드, 설정 등)	제어모드 설정, 제어워드 전송 등
//mapPDO()	TPDO / RPDO 매핑	각 노드별 TPDO/RPDO 설정 구간

std::vector<uint8_t> RoboteqMotorController::createSDOWriteCommand(uint16_t index, uint8_t subindex, uint32_t value)
{
    std::vector<uint8_t> cmd(8, 0);
    cmd[0] = 0x23; // SDO Write with 4-byte value
    cmd[1] = index & 0xFF;
    cmd[2] = (index >> 8) & 0xFF;
    cmd[3] = subindex;
    cmd[4] = value & 0xFF;
    cmd[5] = (value >> 8) & 0xFF;
    cmd[6] = (value >> 16) & 0xFF;
    cmd[7] = (value >> 24) & 0xFF;
    return cmd;
}


bool RoboteqMotorController::sendSDOToNode(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value)
{
    uint32_t can_id = 0x600 + node_id;
    std::vector<uint8_t> data = createSDOWriteCommand(index, subindex, value);

    bool success = can_driver_->sendFrame(can_id, data);
    if (success) {
        std::cout << "      [SDO] Sent to node " << static_cast<int>(node_id)
                  << " index 0x" << std::hex << index
                  << " sub 0x" << std::hex << static_cast<int>(subindex)
                  << " value 0x" << std::hex << value << std::endl;
    } else {
        std::cerr << "      [ERROR] SDO send failed to node " << static_cast<int>(node_id) << std::endl;
    }

    return success;
}

std::vector<uint8_t> RoboteqMotorController::createSDOReadCommand(uint16_t index, uint8_t subindex)
{
    std::vector<uint8_t> cmd(8, 0);
    cmd[0] = 0x40;  // SDO Read command specifier
    cmd[1] = index & 0xFF;
    cmd[2] = (index >> 8) & 0xFF;
    cmd[3] = subindex;
    // 나머지는 padding
    return cmd;
}

bool RoboteqMotorController::mapPDO(uint8_t node_id, uint16_t pdo_comm_index, uint16_t pdo_map_index,
                         const std::vector<std::pair<uint16_t, uint8_t>>& mappings)
{
    //uint8_t node_id = static_cast<uint8_t>(node);
    // (1) Disable PDO during mapping
    std::vector<uint8_t> disable_pdo = createSDOWriteCommand(pdo_comm_index, 0x01, 0x80000000);
    can_driver_->sendFrame(0x600 + node_id, disable_pdo);

    // (2) Clear mapping count
    std::vector<uint8_t> clear_map = createSDOWriteCommand(pdo_map_index, 0x00, 0x00);
    can_driver_->sendFrame(0x600 + node_id, clear_map);

    // (3) Set mapping entries
    for (size_t i = 0; i < mappings.size(); ++i) {
        uint32_t map_entry = (mappings[i].first << 8) | 0x20;  // 0x20 = 32bit
        std::vector<uint8_t> map_cmd = createSDOWriteCommand(pdo_map_index, static_cast<uint8_t>(i + 1), map_entry);
        can_driver_->sendFrame(0x600 + node_id, map_cmd);
    }

    // (4) Set mapping count
    std::vector<uint8_t> set_map_count = createSDOWriteCommand(pdo_map_index, 0x00, static_cast<uint32_t>(mappings.size()));
    can_driver_->sendFrame(0x600 + node_id, set_map_count);

    // (5) Enable PDO
    std::vector<uint8_t> enable_pdo = createSDOWriteCommand(pdo_comm_index, 0x01, 0x00000080);  // valid
    can_driver_->sendFrame(0x600 + node_id, enable_pdo);

    return true;
}


} // namespace Inabot
