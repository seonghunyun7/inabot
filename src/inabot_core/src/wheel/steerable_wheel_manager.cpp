// SteerableWheelManager.cpp
#include "wheel/steerable_wheel_manager.hpp"

void SteerableWheelManager::updateWheelData(const std::vector<int>& pulses, const std::vector<double>& velocities) {
    wheels_.clear();
    // pulses/velocities는 [front_traction, front_steering, rear_traction, rear_steering] 순서라고 가정
    if (pulses.size() < 4 || velocities.size() < 4) return;

    SteerableWheelPairData front;
    front.traction.pulse = pulses[0];
    front.traction.velocity = velocities[0];
    front.steering.pulse = pulses[1];
    front.steering.velocity = velocities[1];

    SteerableWheelPairData rear;
    rear.traction.pulse = pulses[2];
    rear.traction.velocity = velocities[2];
    rear.steering.pulse = pulses[3];
    rear.steering.velocity = velocities[3];

    wheels_.push_back(front);
    wheels_.push_back(rear);
}

void SteerableWheelManager::setSteeringAngles(const std::vector<double>& angles) {
    steering_angles_ = angles;
}

std::vector<double> SteerableWheelManager::getWheelVelocities() const {
    if (wheels_.size() < 2) return {};

    // kinematics 입력 순서 맞춰서 반환 {front_steering_angle, front_velocity, rear_steering_angle, rear_velocity}
    return {
        steering_angles_.size() > 0 ? steering_angles_[0] : wheels_[0].steering.velocity,  // front steering angle (rad)
        wheels_[0].traction.velocity,   // front wheel velocity (m/s)
        steering_angles_.size() > 1 ? steering_angles_[1] : wheels_[1].steering.velocity,  // rear steering angle (rad)
        wheels_[1].traction.velocity    // rear wheel velocity (m/s)
    };
}

std::vector<int> SteerableWheelManager::getWheelPulses() const {
    std::vector<int> pulses;
    for (const auto& pair : wheels_) {
        pulses.push_back(pair.traction.pulse);
        pulses.push_back(pair.steering.pulse);
    }
    return pulses;
}