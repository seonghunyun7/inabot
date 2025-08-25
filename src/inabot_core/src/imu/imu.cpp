#include <imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <csignal>
#include <memory>
#include <chrono>
#include <vector>
#include <sstream>

#define __LOG__ false

const std::string ImuNode::logger = "[ Y3SpaceDriver ] ";
const std::string ImuNode::MODE_ABSOLUTE = "absolute";
const std::string ImuNode::MODE_RELATIVE = "relative";

ImuNode::ImuNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("imu_node", options)
{  
    initParameters();

    serial_interface_ = std::make_unique<SerialInterface>(port, baudrate, timeout);
    
    try {
        bool connection_success = serial_interface_->serialConnect();
        if (!connection_success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to serial port.");
            return;
        }

    } catch (const std::runtime_error &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to serial port: %s", e.what());
        rclcpp::shutdown();
        throw;
    }

    RCLCPP_INFO(this->get_logger(), "connect to serial port. done ");

    serial_interface_->serialWriteString(SET_AXIS_DIRECTIONS_X_Forward_Y_Right_Z_Up);  // change axis directions
    rclcpp::sleep_for(std::chrono::milliseconds(1500));

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    default_qos.keep_last(10);
    this->imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10 /*default_qos*/);
    std::thread(&ImuNode::run, this).detach();
}

ImuNode::~ImuNode()
{
    if (serial_interface_) {
        serial_interface_->serialDisconnect();
    }
}

void ImuNode::initParameters()
{
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<int>("timeout", 60000);
    this->declare_parameter<std::string>("mode", "relative");
    this->declare_parameter<std::string>("frame", "imu_link");

    this->get_parameter("port", port);
    this->get_parameter("baudrate", baudrate);
    this->get_parameter("timeout", timeout);
    this->get_parameter("mode", mode);
    this->get_parameter("frame", frame);

    RCLCPP_INFO(this->get_logger(), "Port: %s", port.c_str());
    RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate);
    RCLCPP_INFO(this->get_logger(), "Timeout: %d", timeout);
    RCLCPP_INFO(this->get_logger(), "Mode: %s", mode.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame: %s", frame.c_str());
}

void ImuNode::restoreFactorySettings()
{
    serial_interface_->serialWriteString(RESTORE_FACTORY_SETTINGS);
}

const std::string ImuNode::getSoftwareVersion()
{
    serial_interface_->serialWriteString(GET_FIRMWARE_VERSION_STRING);

    const std::string buf = serial_interface_->serialReadLine();
    RCLCPP_INFO(this->get_logger(), "Software version: %s", buf.c_str());
    return buf;
}

const std::string ImuNode::getAxisDirection()
{
    serial_interface_->serialWriteString(GET_AXIS_DIRECTION);

    const std::string buf = serial_interface_->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "X: Right, Y: Up, Z: Forward";
        }
        else if ( buf == "1\r\n")
        {
            return "X: Right, Y: Forward, Z: Up";
        }
        else if ( buf == "2\r\n")
        {
            return "X: Up, Y: Right, Z: Forward";
        }
        else if (buf == "3\r\n")
        {
            return "X: Forward, Y: Right, Z: Up";
        }
        else if( buf == "4\r\n")
        {
            return "X: Up, Y: Forward, Z: Right";
        }
        else if( buf == "5\r\n")
        {
            return "X: Forward, Y: Up, Z: Right";
        }
        else if (buf == "19\r\n")
        {
            return "X: Forward, Y: Left, Z: Up";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Buffer indicates: %s", buf.c_str());
            return "Unknown";
        }
    }();

    RCLCPP_INFO(this->get_logger(), "Axis Direction:  %s", ret.c_str());
    return ret;
}

void ImuNode::startGyroCalibration(void)
{
    RCLCPP_INFO(this->get_logger(), "Starting Auto Gyro Calibration...");
    serial_interface_->serialWriteString(BEGIN_GYRO_AUTO_CALIB);
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Proceeding");
}

void ImuNode::setMIMode(bool on)
{
    if(on)
    {
        serial_interface_->serialWriteString(SET_MI_MODE_ENABLED);
    }
    else
    {
        serial_interface_->serialWriteString(SET_MI_MODE_DISABLED);
    }
}

const std::string ImuNode::getCalibMode()
{
    serial_interface_->serialWriteString(GET_CALIB_MODE);

    const std::string buf = serial_interface_->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Bias";
        }
        else if ( buf == "1\r\n")
        {
            return "Scale and Bias";
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Buffer indicates: %s", buf.c_str());
            return "Unknown";
        }
    }();

    RCLCPP_INFO(this->get_logger(), "Calibration Mode:  %s", ret.c_str());
    return ret;
}

const std::string ImuNode::getMIMode()
{
    serial_interface_->serialWriteString(GET_MI_MODE_ENABLED);

    const std::string buf = serial_interface_->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Disabled";
        }
        else if ( buf == "1\r\n")
        {
            return "Enabled";
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Buffer indicates: %s", buf.c_str());
            return "Unknown";
        }
    }();

    RCLCPP_INFO(this->get_logger(), "MI Mode: %s", ret.c_str());
    return ret;
}

//  Y3SpaceDriver 동작
// - Port: /dev/ttyACM0
// - Baudrate: 115200
// - Timeout: 3000ms
// - Mode: relative
// - Frame: center_imu
// - Connection Established with Port: /dev/ttyACM0, Baudrate: 115200
// - Auto Gyro Calibration
// - 소프트웨어 버전: 19Jun2024P56
// - 축 방향: X (Forward), Y (Right), Z (Up)
// - Calibration Mode: Scale and Bias
// - MI Mode: Disabled
// - Driver stream configuration: relative mode
void ImuNode::run()
{
    RCLCPP_INFO(this->get_logger(), "ImuNode run start ");

    std::vector<double> parsedVals;
    auto imuMsg = sensor_msgs::msg::Imu();
    rclcpp::sleep_for(std::chrono::milliseconds(300)); //usleep(300000);

    this->startGyroCalibration();
    this->getSoftwareVersion();
    this->getAxisDirection();
    this->getCalibMode();
    this->getMIMode();

    if (mode == MODE_ABSOLUTE)
    {
        RCLCPP_INFO(this->get_logger(), "Using absolute driver stream configuration");
        serial_interface_->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_ABSOLUTE);
    }
    else if (mode == MODE_RELATIVE)
    {
        RCLCPP_INFO(this->get_logger(), "Using relative driver stream configuration");
        serial_interface_->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_RELATIVE);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Unknown driver mode set... Defaulting to relative");
        serial_interface_->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_RELATIVE);
    }
    
    serial_interface_->serialWriteString(TARE_WITH_CURRENT_ORIENTATION);
    serial_interface_->serialWriteString(TARE_WITH_CURRENT_QUATERNION);
    serial_interface_->serialWriteString(SET_STREAMING_TIMING_10_MS); //SET_STREAMING_TIMING_10_MS  SET_STREAMING_TIMING_100_MS :     // 100ms -> 10ms
    serial_interface_->serialWriteString(START_STREAMING);

    RCLCPP_INFO(this->get_logger(), "ImuNode run - Ready");

    //다음은 각 모드에서 권장하는 ROS 발행 주기:
    //모드	센서 샘플링 주기 (최대)	권장 ROS 발행 주기	설명
    //Kalman AHRS 모드	250Hz	50Hz ~ 100Hz => 
    //QCOMP AHRS 모드	850Hz	100Hz ~ 250Hz
    //IMU 모드	1350Hz	250Hz ~ 500Hz (최대)
    //up to 250Hz with Kalman AHRS(higher with oversampling)
    rclcpp::Rate rate(50); //20ms 
    int line = 0;

    while(rclcpp::ok())
    {
        while(serial_interface_->available() > 0)
        {
            line += 1;
            std::string buf = serial_interface_->serialReadLine();
            std::stringstream ss(buf);
            double i;

            // Parse data from the line
            while (ss >> i)
            {
                parsedVals.push_back(i);
                if (ss.peek() == ',')
                ss.ignore();
            }

            // Should stop reading when line == number of tracked streams
            if(line == 4)
            {
                // Reset line tracker
                line = 0;
                imuMsg.header.stamp           = this->get_clock()->now();  // ROS 2 timestamp this->now();
                imuMsg.header.frame_id        = frame;
                {
                    std::lock_guard<std::mutex> lock(imu_data_mutex);

                    imuMsg.orientation.x          = parsedVals[0];
                    imuMsg.orientation.y          = parsedVals[1];
                    imuMsg.orientation.z          = parsedVals[2];
                    imuMsg.orientation.w          = parsedVals[3];
                    imuMsg.angular_velocity.x     = parsedVals[4];
                    imuMsg.angular_velocity.y     = parsedVals[5];
                    imuMsg.angular_velocity.z     = parsedVals[6];
                    imuMsg.linear_acceleration.x  = parsedVals[7];
                    imuMsg.linear_acceleration.y  = parsedVals[8];
                    imuMsg.linear_acceleration.z  = parsedVals[9];
                }
            
                imu_pub_->publish(imuMsg);
                parsedVals.clear();
            }
            #if __LOG__
            else
            {
                RCLCPP_WARN(this->get_logger(), "Invalid ImuNode data received: size mismatch.");
            }
            #endif
        }

        rate.sleep();
    }
}