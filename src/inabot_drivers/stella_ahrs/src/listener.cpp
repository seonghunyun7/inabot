#include "listener.h"
#include "MwAHRS.hpp"

// ----------------------------
// 전역 변수 정의
// ----------------------------
char recv_buf[8];
long id = 0;
int length = 0;
bool run = true;
std::shared_ptr<sensor_msgs::msg::Imu> imu = std::make_shared<sensor_msgs::msg::Imu>();

// ----------------------------
// AHRS 스레드
// ----------------------------
void* AHRS_thread(void* arg)
{
    (void)arg; 

    const int log_interval_ms = 100;
    auto last_log_time = std::chrono::steady_clock::now();

    while (run)
    {
        if (MW_SerialRecv(&id, &length, recv_buf))
        {
            switch ((int)(unsigned char)recv_buf[1])
            {
                case ACC:
                    imu->linear_acceleration.x = ((int)(unsigned char)recv_buf[2] | (int)(unsigned char)recv_buf[3] << 8) / 1000.0 * 9.8;
                    imu->linear_acceleration.y = ((int)(unsigned char)recv_buf[4] | (int)(unsigned char)recv_buf[5] << 8) / 1000.0 * 9.8;
                    imu->linear_acceleration.z = ((int)(unsigned char)recv_buf[6] | (int)(unsigned char)recv_buf[7] << 8) / 1000.0 * 9.8;
                    break;

                case GYO:
                    imu->angular_velocity.x = ((int)(unsigned char)recv_buf[2] | (int)(unsigned char)recv_buf[3] << 8) / 10.0 * 0.01745;
                    imu->angular_velocity.y = ((int)(unsigned char)recv_buf[4] | (int)(unsigned char)recv_buf[5] << 8) / 10.0 * 0.01745;
                    imu->angular_velocity.z = ((int)(unsigned char)recv_buf[6] | (int)(unsigned char)recv_buf[7] << 8) / 10.0 * 0.01745;
                    break;

                case DEG:
                {
                    float deg_x = ((int)(unsigned char)recv_buf[2] | (int)(unsigned char)recv_buf[3] << 8) / 100.0;
                    float deg_y = ((int)(unsigned char)recv_buf[4] | (int)(unsigned char)recv_buf[5] << 8) / 100.0;
                    float deg_z = ((int)(unsigned char)recv_buf[6] | (int)(unsigned char)recv_buf[7] << 8) / 100.0;

                    imu->orientation.w = (COS(deg_z) * COS(deg_y) * COS(deg_x)) + (SIN(deg_z) * SIN(deg_y) * SIN(deg_x));
                    imu->orientation.x = (COS(deg_z) * COS(deg_y) * SIN(deg_x)) - (SIN(deg_z) * SIN(deg_y) * COS(deg_x));
                    imu->orientation.y = (COS(deg_z) * SIN(deg_y) * COS(deg_x)) + (SIN(deg_z) * COS(deg_y) * SIN(deg_x));
                    imu->orientation.z = (SIN(deg_z) * COS(deg_y) * COS(deg_x)) - (COS(deg_z) * SIN(deg_y) * SIN(deg_x));
                }
                break;
            }

#if __LOG__
            auto now = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count();
            if (elapsed_ms >= log_interval_ms)
            {
                RCLCPP_INFO(rclcpp::get_logger("AHRS"),
                    "ACC: [%.3f, %.3f, %.3f] "
                    "GYRO: [%.3f, %.3f, %.3f] "
                    "ORI: [%.3f, %.3f, %.3f, %.3f]",
                    imu->linear_acceleration.x,
                    imu->linear_acceleration.y,
                    imu->linear_acceleration.z,
                    imu->angular_velocity.x,
                    imu->angular_velocity.y,
                    imu->angular_velocity.z,
                    imu->orientation.w,
                    imu->orientation.x,
                    imu->orientation.y,
                    imu->orientation.z);

                last_log_time = now;
            }
#endif
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return nullptr;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("stella_ahrs");

    node->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    node->declare_parameter<int>("baudrate", 115200);

    std::string serial_port = node->get_parameter("serial_port").as_string();
    int baudrate = node->get_parameter("baudrate").as_int();

    RCLCPP_INFO(node->get_logger(), "Serial Port: %s", serial_port.c_str());
    RCLCPP_INFO(node->get_logger(), "Baudrate: %d", baudrate);

    if (MW_SerialOpen(const_cast<char*>(serial_port.c_str()), baudrate) < 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open serial port: %s", serial_port.c_str());
        return -1;
    }

    Mw_AHRS_init(1);

    pthread_t thread;
    pthread_create(&thread, nullptr, AHRS_thread, nullptr);

    imu->orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    imu->angular_velocity_covariance = {0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02};
    imu->linear_acceleration_covariance = {0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04};

    imu->linear_acceleration.x = 0;
    imu->linear_acceleration.y = 0;
    imu->linear_acceleration.z = 0;

    imu->angular_velocity.x = 0;
    imu->angular_velocity.y = 0;
    imu->angular_velocity.z = 0;

    imu->orientation.w = 1;
    imu->orientation.x = 0;
    imu->orientation.y = 0;
    imu->orientation.z = 0;

    auto chatter_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu", 13); // 13 ???

    rclcpp::WallRate rate(10);
    rclcpp::TimeSource ts(node);
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    RCLCPP_INFO(node->get_logger(), "Stella AHRS node started.");

    while (rclcpp::ok())
    {
        imu->header.frame_id = "imu_link";
        imu->header.stamp = clock->now();

        chatter_pub->publish(*imu);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    run = false;
    Mw_SerialClose();
    pthread_join(thread, nullptr);

    rclcpp::shutdown();
    return 0;
}