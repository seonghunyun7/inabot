// include/kinematics/types.hpp
#ifndef KINEMATICS_TYPES_HPP_
#define KINEMATICS_TYPES_HPP_

namespace kinematics
{
enum class DriveType
{
    DIFFERENTIAL_DRIVE,
    MECANUM,
    OMNIDIRECTIONAL,
    ACKERMANN,
    QUADROTOR,
    TWO_WHEEL_STEERING_DRIVE
};
} // namespace kinematics

#endif  // KINEMATICS_TYPES_HPP_