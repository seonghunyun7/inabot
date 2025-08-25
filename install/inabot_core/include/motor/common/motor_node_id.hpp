#pragma once

#include <vector>    // std::vector
#include <cstdint>   // uint32_t (NodeId의 underlying type)

namespace Inabot {

enum class NodeId : uint32_t {
    FrontTraction = 0x01,
    FrontSteering = 0x02,
    RearTraction  = 0x03,
    RearSteering  = 0x04
};

// 공통 노드 ID
inline std::vector<NodeId> allNodeIds() {
    return {
        NodeId::FrontTraction,
        NodeId::FrontSteering,
        NodeId::RearTraction,
        NodeId::RearSteering
    };
}

} // namespace Inabot
