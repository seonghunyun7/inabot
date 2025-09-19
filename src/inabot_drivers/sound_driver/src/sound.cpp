#include "sound.hpp"
#include <cstdlib>
#include <sstream>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>

namespace fs = std::filesystem;

Sound::Sound() : Node("sound_node")
{
    std::string package_share_dir;
    try {
        package_share_dir = ament_index_cpp::get_package_share_directory("sound_driver");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get package share directory: %s", e.what());
        return;
    }

    sound_dir_ = fs::path(package_share_dir) / "data";  // data 폴더 경로
    RCLCPP_INFO(this->get_logger(), "Sound directory: %s", sound_dir_.c_str());

    this->declare_parameter<double>("volume", 0.5);              // 기본값 0.5
 
    double initial_volume;
    this->get_parameter("volume", initial_volume);
    volume_.store(initial_volume);
    RCLCPP_INFO(this->get_logger(), "Volume set to: %f", initial_volume);

#if _used_check
#if loca_test  // 사운드 카드 확인
    if (check_sound_card()) 
#else
    if (check_sound_card_usb_audio())
#endif
    {
        audio_device_status_ = true;
        RCLCPP_INFO(this->get_logger(), "USB-Audio card is properly detected.");
    } else {
        // 이 상태에서 어떻게 하지???
        audio_device_status_ = false;
        RCLCPP_ERROR(this->get_logger(), "No USB-Audio card detected.");
    }
#else
    audio_device_status_ = true;
#endif
    
    RCLCPP_INFO(this->get_logger(), "sound_node");

    rclcpp::QoS sound_qos = rclcpp::QoS(2)  // 최대 2개의 메시지 유지
        .best_effort()                      // 신뢰성: BEST_EFFORT (지연 최소화)
        .durability_volatile();             // 내구성: VOLATILE (메시지 전달 후 저장하지 않음)
    subscriptions_.push_back(this->create_subscription<std_msgs::msg::String>(
        "/sound/play", sound_qos, std::bind(&Sound::play_sound_callback, this, std::placeholders::_1))
    );

    status_publisher_ = this->create_publisher<std_msgs::msg::String>("/sound/status", 10);
    audio_device_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/sound/device_status", 10);
}

Sound::~Sound()
{
    shutdown_sound();
    RCLCPP_INFO(this->get_logger(), "Sound node is being destroyed.");
}

void Sound::shutdown_sound()
{
    // ffplay 종료
    std::string command = "pkill -f ffplay";
    int ret = system(command.c_str());
    if (ret != 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to kill ffplay processes.");
    } else {
        RCLCPP_INFO(this->get_logger(), "ffplay processes terminated.");
    }
}

bool Sound::check_sound_card()
{
    std::ifstream file("/proc/asound/cards");
    std::string line;
    bool found_card = false;

    // "sof-hda-dsp" 문자열을 찾으면 내장 오디오 장치가 감지된 것으로 간주
    while (std::getline(file, line)) {
        if (line.find("sof-hda-dsp") != std::string::npos) {
            found_card = true;
            break;
        }
    }

    return found_card;
}

bool Sound::check_sound_card_usb_audio()
{
    std::ifstream file("/proc/asound/cards");
    std::string line;
    bool found_card = false;

    // "USB-Audio" 또는 "Microchip" 문자열을 찾으면 USB 오디오 장치가 감지된 것으로 간주
    while (std::getline(file, line)) {
        if (line.find("USB-Audio") != std::string::npos || line.find("Microchip") != std::string::npos) {
            found_card = true;
            break;
        }
    }

    return found_card;
}

void Sound::play_sound(const std::string &file_type)
{
#if _used_check
    if (!audio_device_status_) {
        RCLCPP_WARN(this->get_logger(), "Audio device not ready.");
        publish_audio_device_status(false);
        return;
    }
#endif
    // 1. 파일 이름 가져오기
    std::string file_name = get_file_name(file_type);
    if (file_name.empty()) {
        publish_status("Error: Unknown file type");
        return;
    }

    // 2. 패키지 내 data/ 폴더 사용
    fs::path file_path = fs::path(sound_dir_) / file_name;
    if (!fs::exists(file_path)) {
        if (file_name == "stop") {
            // 재생 중지
            std::string command = "pkill -f ffplay";
            system(command.c_str());
            publish_status("Playback stopped");
        } else {
            RCLCPP_ERROR(this->get_logger(), "MP3 file does not exist: '%s'", file_path.c_str());
            publish_status("Error: MP3 file not found");
        }
        return;
    }

    // 3. 현재 볼륨 가져오기
    double current_volume = volume_.load();
    if (current_volume < 0.0) current_volume = 0.0;
    if (current_volume > 2.0) current_volume = 2.0;
    std::string volume_str = std::to_string(current_volume);

    // 4. ffplay 명령어 실행 (비동기 실행)
    std::string command = "ffplay -nodisp -loglevel quiet -autoexit -af \"volume=" + volume_str + "\" \"" + file_path.string() + "\" &";
    int ret = std::system(command.c_str());
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute ffplay command.");
        publish_status("Error: Failed to play MP3");
        return;
    }

    // 5. 상태 퍼블리시
    publish_status("MP3 playback started: " + file_name);
}

std::string Sound::get_file_name(const std::string &file_type)
{
    if (file_type == "default") return "run.mp3";
    if (file_type == "obstacle") return "obs.mp3";
    if (file_type == "error") return "err.mp3";
    if (file_type == "stop") return "stop";
    RCLCPP_ERROR(this->get_logger(), "Unknown file type: '%s'", file_type.c_str());
    return "";
}

void Sound::publish_status(const std::string &status)
{
    auto status_message = std_msgs::msg::String();
    status_message.data = status;
    status_publisher_->publish(status_message);
}

void Sound::publish_audio_device_status(bool status)
{
    auto status_message = std_msgs::msg::Bool();
    status_message.data = status;
    audio_device_status_publisher_->publish(status_message);
}

void Sound::play_sound_callback(const std_msgs::msg::String::SharedPtr msg)
{
    play_sound(msg->data);
}

void Sound::volume_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    double new_volume = msg->data;
    if (new_volume < 0.0 || new_volume > 2.0) { // 확장된 볼륨 범위
        RCLCPP_WARN(this->get_logger(), "Received invalid volume value: '%f'. Ignoring.", new_volume);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Volume updated to: %f", new_volume);
    volume_.store(new_volume);
}
