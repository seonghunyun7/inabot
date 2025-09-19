#include "ota_node.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdlib>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fcntl.h>
#include <errno.h>
#include <cstring>

namespace fs = std::filesystem;

OTANode::OTANode()
: Node("ota_node"), server_running_(true), server_fd_(-1), client_socket_fd_(-1)
{
    this->declare_parameter<std::string>("temp_archive", "/home/ysh/tmp/ota_update.tar.gz");
    this->declare_parameter<std::string>("ota_host", "0.0.0.0");
    this->declare_parameter<int>("ota_port", 5005);
    this->declare_parameter<std::string>("install_dir", "/home/ysh/inabot_ws/install");
    this->declare_parameter<std::string>("backup_dir_base", "/home/ysh/inabot_ws/install_backup_");

    this->get_parameter("temp_archive", TEMP_ARCHIVE_);
    this->get_parameter("ota_host", OTA_HOST_);
    this->get_parameter("ota_port", OTA_PORT_);
    this->get_parameter("install_dir", INSTALL_DIR_);
    this->get_parameter("backup_dir_base", BACKUP_DIR_BASE_);

    server_thread_ = std::thread(&OTANode::startServer, this);
    RCLCPP_INFO(this->get_logger(), "OTA Node started");
}

OTANode::~OTANode()
{
    stopServer();
}

void OTANode::publishStatus(const std::string &msg)
{
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
}

void OTANode::sendStatusToClient(const std::string &status)
{
    #if 0
    if (client_socket_fd_ < 0) return;
    std::string msg = status + "\n";
    send(client_socket_fd_, msg.c_str(), msg.size(), 0);
    #endif
}

void OTANode::startServer()
{
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    int opt = 1;

    // 1. 서버 소켓 생성
    if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket failed");
        return;
    }

    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(OTA_HOST_.c_str());
    address.sin_port = htons(OTA_PORT_);

    if (bind(server_fd_, (struct sockaddr *)&address, sizeof(address)) < 0) { perror("bind failed"); return; }
    if (listen(server_fd_, 1) < 0) { perror("listen failed"); return; }

    // 논블로킹 설정 (Ctrl+C 안전)
    fcntl(server_fd_, F_SETFL, O_NONBLOCK);

    publishStatus("OTA Server listening on " + OTA_HOST_ + ":" + std::to_string(OTA_PORT_));

    while (rclcpp::ok() && server_running_)
    {
        int new_socket = accept(server_fd_, (struct sockaddr *)&address, (socklen_t *)&addrlen);
        if (new_socket < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("accept failed");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        client_socket_fd_ = new_socket;
        sendStatusToClient("Connection received from client");

        // 2. 이전 임시 파일 제거
        if (fs::exists(TEMP_ARCHIVE_)) {
            try {
                fs::remove(TEMP_ARCHIVE_);
                sendStatusToClient("Previous OTA file removed");
            } catch (const fs::filesystem_error &e) {
                sendStatusToClient(std::string("Failed to remove existing OTA file: ") + e.what());
                close(new_socket);
                client_socket_fd_ = -1;
                continue;
            }
        }

        // 3. 파일 수신
        std::ofstream ofs(TEMP_ARCHIVE_, std::ios::binary);
        if (!ofs.is_open()) {
            sendStatusToClient("Failed to open temp archive for writing!");
            close(new_socket);
            client_socket_fd_ = -1;
            continue;
        }

        char buffer[4096];
        int bytes_read;
        size_t total_bytes = 0;
        while ((bytes_read = read(new_socket, buffer, sizeof(buffer))) > 0) {
            ofs.write(buffer, bytes_read);
            total_bytes += bytes_read;
            // 중간 상태 전송 가능
            sendStatusToClient("Receiving OTA file... " + std::to_string(total_bytes) + " bytes");
        }
        ofs.flush();
        ofs.close();
        sendStatusToClient("[OTA] File reception completed. Total bytes: " + std::to_string(total_bytes));

        if (!fs::exists(TEMP_ARCHIVE_)) {
            sendStatusToClient("OTA archive NOT created!");
            close(new_socket);
            client_socket_fd_ = -1;
            continue;
        }

        //
        prepareForOTA();

        // 4. 업데이트 배포
        sendStatusToClient("[OTA] Deploying update...");
        deployUpdate();
        sendStatusToClient("[OTA] Deploy finished.");

        // 5. 소켓 종료 (모든 상태 메시지 전송 후)
        close(new_socket);
        client_socket_fd_ = -1;
        sendStatusToClient("[OTA] Client socket closed.");

        // 6. reboot
    }

    // 서버 종료 시 소켓 정리
    if (server_fd_ > 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
}

void OTANode::prepareForOTA()
{
    RCLCPP_INFO(this->get_logger(), "Preparing system for OTA update...");
    int delay_sec = 2;
    
    // 패키지 prefix 가져오기
    std::string prefix;
    try {
        prefix = ament_index_cpp::get_package_prefix("inabot_ota");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get package prefix: %s", e.what());
        return;
    }

    // 2설치된 exit_all.sh 경로
    std::string script_path = prefix + "/lib/inabot_ota/exit_all.sh";

    //스크립트 실행
    RCLCPP_INFO(this->get_logger(), "Executing: %s", script_path.c_str());
    int ret = system(script_path.c_str());

    if(ret == 0) {
        RCLCPP_INFO(this->get_logger(), "All processes terminated successfully.");
    } else {
        RCLCPP_WARN(this->get_logger(), "exit_all.sh may have failed, return code: %d", ret);
    }

    //종료 후 잠시 지연
    // 멤버 함수 안에서 this 사용 가능
    RCLCPP_INFO(this->get_logger(), "Waiting %d seconds after process termination...", delay_sec);
    std::this_thread::sleep_for(std::chrono::seconds(delay_sec));
}

void OTANode::deployUpdate()
{
    std::string tmp_extract = "/home/ysh/tmp/ota_update_dir";

    sendStatusToClient("[OTA] deployUpdate started");

    // 1. 이전 임시 디렉토리 삭제
    fs::remove_all(tmp_extract);

    // 2. 압축 해제
    std::string cmd = "mkdir -p " + tmp_extract + " && tar -xzf " + TEMP_ARCHIVE_ + " -C " + tmp_extract;
    if (std::system(cmd.c_str()) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to extract archive!");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Archive extracted to %s", tmp_extract.c_str());

    // 3. 기존 install 백업
    std::time_t t = std::time(nullptr);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&t));
    std::string backup_dir = BACKUP_DIR_BASE_ + timestamp;

    try {
        fs::rename(INSTALL_DIR_, backup_dir);
        RCLCPP_INFO(this->get_logger(), "Existing install moved to %s", backup_dir.c_str());
    } catch (const fs::filesystem_error &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to backup install folder: %s", e.what());
        return;
    }

    // 4. 새 파일 설치: tmp_extract/install 안의 내용만 INSTALL_DIR_로 이동
    fs::path tmp_inner = fs::path(tmp_extract) / "install";
    if (!fs::exists(tmp_inner) || !fs::is_directory(tmp_inner)) {
        RCLCPP_ERROR(this->get_logger(), "Extracted folder structure invalid");
        return;
    }

    try {
        fs::create_directories(INSTALL_DIR_);  // INSTALL_DIR_가 비어있을 경우
        for (const auto &entry : fs::directory_iterator(tmp_inner)) {
            fs::path dest = fs::path(INSTALL_DIR_) / entry.path().filename();
            fs::rename(entry.path(), dest);
        }
        RCLCPP_INFO(this->get_logger(), "New install deployed to %s", INSTALL_DIR_.c_str());
    } catch (const fs::filesystem_error &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to deploy new install: %s", e.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "OTA update completed. Reboot required.");
}

void OTANode::stopServer()
{
    server_running_ = false;
    if (server_fd_ > 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
    if (client_socket_fd_ > 0) {
        close(client_socket_fd_);
        client_socket_fd_ = -1;
    }
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
}