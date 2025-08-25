#pragma once

/// @file motor_encoder_data.hpp
/// @brief Defines EncoderChannel enum and MotorEncoderData struct to hold raw and converted motor encoder data.

enum class EncoderChannel : int {
    FRONT = 0,
    REAR = 1,
    UNKNOWN = -1
};

struct MotorEncoderData {
    EncoderChannel channel = EncoderChannel::UNKNOWN;

    // Raw encoder position/velocity (e.g., pulses or rpm)
    int32_t traction_position_raw = 0;
    int32_t steering_position_raw = 0;
    int32_t traction_velocity_raw = 0;
    int32_t steering_velocity_raw = 0;

    // Converted position/velocity (e.g., meters, m/s)
    double traction_position = 0.0;
    double steering_position = 0.0;
    double traction_velocity = 0.0;  // in m/s
    double steering_velocity = 0.0;  // in m/s
};
