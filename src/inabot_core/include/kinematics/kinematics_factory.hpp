#pragma once

#include <memory>
#include <string>
#include <algorithm>  // for std::transform

#include "base_kinematics.hpp"
#include "differential_drive.hpp"
#include "mecanum_drive.hpp"
#include "omni_drive.hpp"
#include "quaternion_drive.hpp"
#include "steerable_wheel_drive.hpp"

namespace kinematics
{

class KinematicsFactory
{
public:

    static std::unique_ptr<BaseKinematics> create(std::string drive_type)
    {
        std::transform(drive_type.begin(), drive_type.end(), drive_type.begin(), ::tolower);

        if (drive_type == "differential")
        {
            return std::make_unique<DifferentialDrive>();
        }
        else if (drive_type == "quaternion")
        {
            return std::make_unique<QuaternionDrive>();
        }
        else if (drive_type == "traction_steering")
        {
            return std::make_unique<SteerableWheelDrive>();
        }
        else if (drive_type == "mecanum")
        {
            return std::make_unique<MecanumDrive>();
        }
        else if (drive_type == "omni")
        {
            return std::make_unique<OmniDrive>();
        }
        else
        {
            return nullptr;
        }
    }
};

}  // namespace kinematics
