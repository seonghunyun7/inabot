#ifndef SOUND_HPP
#define SOUND_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <future>
#include <string>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

class Sound : public rclcpp::Node
{
public:
    Sound();
    ~Sound();
    //ffplay 종료 등 정리 작업을 외부에서 호출 가능하게
    void shutdown_sound();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr audio_device_status_publisher_;

    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_; 

    std::atomic<bool> stop_loop_;
    std::future<void> loop_task_;
    std::atomic<double> volume_;
    bool audio_device_status_;
    std::string sound_dir_;  

    bool check_sound_card();
    bool check_sound_card_usb_audio();
    void play_sound(const std::string &file_type);
    std::string get_file_name(const std::string &file_type);
    void publish_status(const std::string &status);
    void publish_audio_device_status(bool status);
    void play_sound_callback(const std_msgs::msg::String::SharedPtr msg);
    void volume_callback(const std_msgs::msg::Float64::SharedPtr msg);
};

#endif // SOUND_HPP
